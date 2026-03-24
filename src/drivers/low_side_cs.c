/**
 * low_side_cs.c — Low-side current sense implementation for STM32F411 +
 *                 TMC6300 + 2-shunt measurement.
 *
 * Closely mirrors SimpleFOC LowsideCurrentSense logic:
 *   init()             → low_side_cs_init()
 *   calibrateOffsets() → low_side_cs_calibrate_offsets()
 *   getPhaseCurrents() → low_side_cs_get_phase_currents()
 *   driverAlign()      → low_side_cs_driver_align()
 *
 * Low-level strategy
 * ==================
 * TIM1 is switched to center-aligned mode 1 so the counter ramps up then
 * down symmetrically.  The trough (counter = 0) is when all low-side FETs
 * are guaranteed ON, which is the ideal sampling window for low-side shunts.
 *
 * TIM1 CH4 is used as a pure internal compare (no GPIO output pin, OC4PE=0)
 * configured as PWM mode 1.  CCR4 is set to a small value corresponding to
 * OFFSET_NS after the trough, giving the TMC6300's ~500 ns propagation delay
 * time to settle before ADC sampling.
 *
 * TIM1_CH4 event (JEXTSEL = 0b0000) triggers ADC1 injected sequence on the
 * rising edge of the internal OC4REF pulse.
 *
 * Dead-time (BDTR.DTG):
 *   f_DTS = f_TIM1 = 100 MHz (CKD = 00, no prescaler division)
 *   t_DTS = 10 ns
 *   Target = 500 ns → 50 × t_DTS → DTG = 50 (0x32)  [DTG[7:5] = 000]
 *
 * Center-aligned period calculation:
 *   f_TIM1 = 100 MHz, prescaler = 0 (PSC = 0, counts in 10 ns steps)
 *   For center-aligned: f_PWM = f_TIM1 / (2 × ARR)
 *   25 kHz → ARR = 100e6 / (2 × 25000) = 2000
 *   Resolution = ARR = 2000 counts = 0.05% per count
 *
 * CCR4 offset calculation:
 *   OFFSET_NS configured at init (default 800 ns)
 *   offset_counts = (OFFSET_NS × f_TIM1) / 1e9
 *   E.g. 800 ns @ 100 MHz → 80 counts
 *   CCR4 = offset_counts  (triggers when CNT rises from 0 to offset_counts)
 *
 * ADC injected sequence:
 *   JL = 1  (2 conversions)
 *   JSQ3 → CH0 / PA0 → phase W  (Ic)  → JDR1
 *   JSQ4 → CH1 / PA1 → phase UV (Ia)  → JDR2
 *   JEXTSEL = 0b0000 = TIM1_CH4
 *   JEXTEN  = 0b01   = rising edge
 */

#include "low_side_cs.h"

#include <stm32f4xx.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/printk.h>
#include <math.h>

/* ---------------------------------------------------------------------------
 * Hardware constants
 * ------------------------------------------------------------------------- */
#define TIM1_CLOCK_HZ     100000000UL   /* 100 MHz TIM1 APB2 clock       */
#define PWM_FREQ_HZ       25000UL       /* 25 kHz centre-aligned PWM      */
/* ARR for centre-aligned: f_PWM = f_CLK / (2*ARR) → ARR = 2000          */
#define TIM1_ARR          (TIM1_CLOCK_HZ / (2UL * PWM_FREQ_HZ))  /* 2000 */

/* DTG = 50 → dead-time = 50 × 10 ns = 500 ns (DTG[7:5]=000 → t_DTS=10ns)*/
#define BDTR_DTG_500NS    50U

#define ADC_CALIBRATION_ROUNDS 2000

/* Zephyr ADC device (pin-mux only, registers configured manually below)  */
#define ADC_NODE DT_NODELABEL(adc1)
static const struct device *adc_dev = DEVICE_DT_GET(ADC_NODE);

static const struct adc_channel_cfg ch0_cfg = {
    .gain             = ADC_GAIN_1,
    .reference        = ADC_REF_INTERNAL,
    .acquisition_time = ADC_ACQ_TIME_DEFAULT,
    .channel_id       = 0,   /* PA0 – phase W (Ic) */
};
static const struct adc_channel_cfg ch1_cfg = {
    .gain             = ADC_GAIN_1,
    .reference        = ADC_REF_INTERNAL,
    .acquisition_time = ADC_ACQ_TIME_DEFAULT,
    .channel_id       = 1,   /* PA1 – phase UV (Ia) */
};

/* ---------------------------------------------------------------------------
 * low_side_cs_init_struct
 * ------------------------------------------------------------------------- */
void low_side_cs_init_struct(low_side_cs_t *cs,
                             float shunt_ohms,
                             float amp_gain,
                             uint32_t vref_mv,
                             uint32_t offset_trough_ns)
{
    if (!cs) return;

    cs->shunt_resistor      = shunt_ohms;
    cs->amp_gain            = amp_gain;
    cs->volts_to_amps_ratio = 1.0f / (amp_gain * shunt_ohms);
    cs->gain_a              = cs->volts_to_amps_ratio;
    cs->gain_c              = cs->volts_to_amps_ratio;
    cs->vref                = (float)vref_mv / 1000.0f;
    cs->adc_full_scale      = 4095.0f;   /* 12-bit ADC */
    cs->offset_trough_ns    = offset_trough_ns ? offset_trough_ns : 800U;
    cs->offset_ia           = 0.0f;
    cs->offset_ic           = 0.0f;
    cs->initialized         = false;
    cs->skip_align          = false;
}

/* ---------------------------------------------------------------------------
 * _configure_tim1_center_aligned
 *
 * Switches TIM1 from edge-aligned (as set by Zephyr PWM driver) to
 * center-aligned mode 1 and configures:
 *   - ARR for 25 kHz
 *   - Dead-time in BDTR (500 ns)
 *   - CH4 in PWM mode 1 as internal trigger source for ADC
 *   - TRGO routed to OC4REF (MMS = 0b111)  ← NOT used; we use direct CH4
 * ------------------------------------------------------------------------- */
static void _configure_tim1_center_aligned(uint32_t offset_trough_ns)
{
    /* 1. Disable timer temporarily */
    TIM1->CR1 &= ~TIM_CR1_CEN;

    /* 2. Centre-aligned mode 1 (CMS = 01): OC flags set on down-count only.
     *    This means CCR4 compare fires when CNT counts UP from 0, which is
     *    exactly at the trough + offset — the safe shunt sampling window. */
    TIM1->CR1 = (TIM1->CR1 & ~TIM_CR1_CMS_Msk)
              | TIM_CR1_CMS_0;           /* CMS = 01 = centre-aligned mode 1 */

    /* 3. Set ARR for 25 kHz.  ARR is already 2000 from Zephyr's edge-aligned
     *    init (period_ns / 10 ns = 40000 ns / 10 ns = 4000 half-period, but
     *    edge-aligned ARR = f_CLK/f_PWM - 1 = 3999).  We overwrite it to the
     *    correct centre-aligned value. */
    TIM1->ARR = TIM1_ARR - 1U;          /* ARR = 1999 → 25 kHz centre-aligned */

    /* 4. Dead-time and MOE (Main Output Enable) in BDTR.
     *    DTG[7:0] = 50 (0x32): t_DT = 50 × 10 ns = 500 ns.
     *    DTG[7:5] = 000 → formula: t_DT = DTG[7:0] × t_DTS = DTG × 10 ns.
     *    OSSR=1: off-state for complementary outputs — outputs low when MOE=0.
     *    LOCK=00: no register lock (we may still tune PID gains at runtime). */
    TIM1->BDTR = (TIM1->BDTR & ~(TIM_BDTR_DTG_Msk | TIM_BDTR_LOCK_Msk))
               | BDTR_DTG_500NS          /* DTG = 50 → 500 ns dead-time      */
               | TIM_BDTR_OSSR          /* off-state safety                  */
               | TIM_BDTR_MOE;          /* main output enable                */

    /* 5. Configure CH4 as internal PWM output (no GPIO, CCxP not relevant).
     *    PWM mode 1: OC4REF high while CNT < CCR4 → pulse at trough rising
     *    edge gives us the trigger exactly when we want it.
     *    CC4S = 00 (output), OC4M = 110 (PWM mode 1), OC4PE = 0 (preload off
     *    → CCR4 takes effect immediately when we write it). */
    TIM1->CCMR2 = (TIM1->CCMR2
                   & ~(TIM_CCMR2_CC4S_Msk | TIM_CCMR2_OC4M_Msk | TIM_CCMR2_OC4PE))
                | (0b110U << TIM_CCMR2_OC4M_Pos);  /* PWM mode 1 */

    /* Disable CH4 GPIO output — we only need the internal OC4REF signal.    */
    TIM1->CCER &= ~TIM_CCER_CC4E;

    /* 6. CCR4 = offset_counts.
     *    offset_counts = offset_ns × f_CLK / 1e9
     *    800 ns × 100 MHz / 1e9 = 80 counts.
     *    CCR4 = 0 would trigger at the very trough; we add the offset to wait
     *    for the driver propagation delay + switching noise to settle. */
    uint32_t offset_counts = (uint32_t)(((uint64_t)offset_trough_ns * TIM1_CLOCK_HZ) / 1000000000ULL);
    if (offset_counts == 0) offset_counts = 1;       /* never trigger at CNT=0 */
    if (offset_counts >= TIM1_ARR) offset_counts = TIM1_ARR / 4U;
    TIM1->CCR4 = offset_counts;

    /* 7. Route TRGO to OC4REF so the ADC injected group can use it.
     *    RM0383 Table 61: MMS = 0b111 → OC4REF → TRGO.
     *    ADC JEXTSEL must then be set to TIM1_TRGO (0b0001) instead of
     *    TIM1_CH4 (0b0000), because the injected external trigger mux on
     *    STM32F411 maps the *TRGO* signal, not the raw OC4REF.             */
    TIM1->CR2 = (TIM1->CR2 & ~TIM_CR2_MMS_Msk)
              | (0b111U << TIM_CR2_MMS_Pos);   /* MMS = OC4REF → TRGO       */

    /* 8. Force update event to reload ARR/CCR4 into shadow registers, then
     *    clear the UIF flag so we don't get a spurious update interrupt.   */
    TIM1->EGR  = TIM_EGR_UG;
    TIM1->SR  &= ~TIM_SR_UIF;

    /* 9. Re-enable timer */
    TIM1->CR1 |= TIM_CR1_CEN;

    printk("[CS-HW] TIM1 centre-aligned: ARR=%u, CCR4=%u (offset %u ns), "
           "DTG=%u (500 ns), MMS=OC4REF→TRGO\n",
           (unsigned)TIM1->ARR, (unsigned)TIM1->CCR4,
           (unsigned)offset_trough_ns, (unsigned)BDTR_DTG_500NS);
}

/* ---------------------------------------------------------------------------
 * _configure_adc_injected
 *
 * Routes ADC1 injected channels to TIM1_TRGO (which now carries OC4REF).
 * Channel layout:
 *   JSQ3 (1st conversion) → CH0 / PA0 → phase W  → JDR1
 *   JSQ4 (2nd conversion) → CH1 / PA1 → phase UV → JDR2
 *   JEXTSEL = 0b0001 = TIM1_TRGO  (STM32F411 RM0383 Table 77)
 * ------------------------------------------------------------------------- */
static void _configure_adc_injected(void)
{
    /* Pin-mux via Zephyr (must be done before touching registers) */
    if (!device_is_ready(adc_dev)) {
        printk("[CS-HW] ADC device not ready!\n");
        return;
    }
    adc_channel_setup(adc_dev, &ch0_cfg);
    adc_channel_setup(adc_dev, &ch1_cfg);

    /* Power down ADC to safely change JEXTSEL / JSQR */
    ADC1->CR2 &= ~ADC_CR2_ADON;

    /* Injected sequence: 2 conversions
     *   JL = 1 (value 1 → 2 conversions)
     *   JSQ3 = CH0 (PA0, phase W / Ic)  → result in JDR1
     *   JSQ4 = CH1 (PA1, phase UV / Ia) → result in JDR2             */
    ADC1->JSQR = ((2U - 1U) << ADC_JSQR_JL_Pos)
               | (0U        << ADC_JSQR_JSQ3_Pos)
               | (1U        << ADC_JSQR_JSQ4_Pos);

    /* External trigger: TIM1_TRGO (JEXTSEL=0b0001), rising edge (JEXTEN=01) */
    ADC1->CR2 = (ADC1->CR2 & ~(ADC_CR2_JEXTSEL_Msk | ADC_CR2_JEXTEN_Msk))
              | (1U << ADC_CR2_JEXTSEL_Pos)    /* 0b0001 = TIM1_TRGO        */
              | ADC_CR2_JEXTEN_0;             /* rising edge               */

    /* Enable JEOC interrupt so the FOC ISR is invoked when conversion done  */
    ADC1->CR1 |= ADC_CR1_JEOCIE;

    /* Power ADC back on */
    ADC1->CR2 |= ADC_CR2_ADON;

    printk("[CS-HW] ADC1 injected: CH0=PA0(Ic/W) CH1=PA1(Ia/UV) "
           "trigger=TIM1_TRGO(OC4REF) JEOC IRQ enabled\n");
}

/* ---------------------------------------------------------------------------
 * low_side_cs_init — public API
 * ------------------------------------------------------------------------- */
int low_side_cs_init(low_side_cs_t *cs)
{
    if (!cs) return 0;

    _configure_tim1_center_aligned(cs->offset_trough_ns);
    _configure_adc_injected();

    cs->initialized = true;
    return 1;
}

/* ---------------------------------------------------------------------------
 * _read_adc_raw — helper: wait for JEOC + return raw JDR values.
 * Timeout after one full PWM period (40 µs → 200 busy-wait steps of 1 µs).
 * Returns true if a valid conversion was captured.
 * ------------------------------------------------------------------------- */
static bool _read_adc_raw(uint16_t *raw_ic, uint16_t *raw_ia)
{
    for (int t = 0; t < 200; t++) {
        if (ADC1->SR & ADC_SR_JEOC) break;
        k_busy_wait(1);
    }
    if (!(ADC1->SR & ADC_SR_JEOC)) return false;

    ADC1->SR &= ~ADC_SR_JEOC;
    *raw_ic = (uint16_t)(ADC1->JDR1 & 0x0FFFU);  /* Phase W  (Ic) */
    *raw_ia = (uint16_t)(ADC1->JDR2 & 0x0FFFU);  /* Phase UV (Ia) */
    return true;
}

/* ---------------------------------------------------------------------------
 * _raw_to_volts
 * ------------------------------------------------------------------------- */
static inline float _raw_to_volts(uint16_t raw, float vref, float full_scale)
{
    return (float)raw * vref / full_scale;
}

/* ---------------------------------------------------------------------------
 * low_side_cs_calibrate_offsets — mirrors SimpleFOC calibrateOffsets()
 *
 * Averages 2000 HW-triggered samples while motor is at rest (zero torque).
 * Call AFTER motor settle phase (all PWM at 50% duty / voltage_limit/2 as
 * SimpleFOC does, or simply with zero target after FOC init).
 * ------------------------------------------------------------------------- */
void low_side_cs_calibrate_offsets(low_side_cs_t *cs)
{
    if (!cs || !cs->initialized) return;

    double sum_ia = 0.0, sum_ic = 0.0;
    int valid = 0;

    printk("[CS] Calibrating zero-current offsets (%d samples)...\n",
           ADC_CALIBRATION_ROUNDS);

    for (int i = 0; i < ADC_CALIBRATION_ROUNDS; i++) {
        uint16_t raw_ic, raw_ia;
        if (!_read_adc_raw(&raw_ic, &raw_ia)) continue;

        sum_ic += _raw_to_volts(raw_ic, cs->vref, cs->adc_full_scale);
        sum_ia += _raw_to_volts(raw_ia, cs->vref, cs->adc_full_scale);
        valid++;
    }

    if (valid > 0) {
        cs->offset_ic = (float)(sum_ic / valid);
        cs->offset_ia = (float)(sum_ia / valid);
    }

    printk("[CS] Offsets: Ia=%.4f V  Ic=%.4f V  (n=%d)\n",
           (double)cs->offset_ia, (double)cs->offset_ic, valid);
}

/* ---------------------------------------------------------------------------
 * low_side_cs_get_phase_currents — mirrors SimpleFOC getPhaseCurrents()
 *
 * Non-blocking: if JEOC not yet set, returns the previously calculated values
 * (safe — the FOC loop will call this once per period at most).
 *
 * 2-shunt wiring (PA0=Ic, PA1=Ia) → Ib reconstructed via KCL:
 *   Ia + Ib + Ic = 0  →  Ib = -(Ia + Ic)
 * ------------------------------------------------------------------------- */
static phase_current_t s_last_current = {0};  /* cached result */

phase_current_t low_side_cs_get_phase_currents(low_side_cs_t *cs)
{
    if (!cs || !cs->initialized) return s_last_current;

    if (!(ADC1->SR & ADC_SR_JEOC)) {
        return s_last_current;   /* not ready; reuse previous sample */
    }

    ADC1->SR &= ~ADC_SR_JEOC;

    uint16_t raw_ic = (uint16_t)(ADC1->JDR1 & 0x0FFFU);   /* Ic = phase W  */
    uint16_t raw_ia = (uint16_t)(ADC1->JDR2 & 0x0FFFU);   /* Ia = phase UV */

    float V_ic = _raw_to_volts(raw_ic, cs->vref, cs->adc_full_scale);
    float V_ia = _raw_to_volts(raw_ia, cs->vref, cs->adc_full_scale);

    /* Subtract zero-current offset, apply gain: I = (V - V_offset) * V2A */
    s_last_current.c  = (V_ic - cs->offset_ic) * cs->gain_c;   /* Ic (phase W)  */
    s_last_current.a  = (V_ia - cs->offset_ia) * cs->gain_a;   /* Ia (phase UV) */
    s_last_current.b  = -(s_last_current.a + s_last_current.c); /* Ib via KCL   */

    return s_last_current;
}

/* ---------------------------------------------------------------------------
 * low_side_cs_driver_align — mirrors SimpleFOC driverAlign()
 *
 * Verifies that measured current sign matches expected direction when a small
 * positive voltage is applied to phase A.  If inverted, flips gain_a / gain_c.
 * Requires the motor driver to be running (call after bldc_motor_enable()).
 *
 * Implementation note: we don't have a direct driver write here — the caller
 * must apply a small voltage via bldc_motor_set_phase_voltage() before calling
 * this, then pass the resulting phase_current_t for inspection.  In practice
 * SimpleFOC calls this inside init(); we expose it as a separate function so
 * haptic.c can integrate it at the right point in the startup sequence.
 * ------------------------------------------------------------------------- */
int low_side_cs_driver_align(low_side_cs_t *cs, float test_voltage)
{
    (void)test_voltage;   /* alignment happens via sign check after caller applies voltage */

    if (cs->skip_align) return 1;
    if (!cs->initialized) return 0;

    /* Read a fresh sample pair */
    uint16_t raw_ic, raw_ia;
    if (!_read_adc_raw(&raw_ic, &raw_ia)) return 0;   /* timeout */

    float Ic = (_raw_to_volts(raw_ic, cs->vref, cs->adc_full_scale) - cs->offset_ic) * cs->gain_c;
    float Ia = (_raw_to_volts(raw_ia, cs->vref, cs->adc_full_scale) - cs->offset_ia) * cs->gain_a;

    /* With phase A energised positively, Ia should be positive.
     * If negative → flip gain polarity for phase A. */
    if (Ia < 0.0f) {
        cs->gain_a = -cs->gain_a;
        printk("[CS] Align: phase A gain inverted\n");
    }
    if (Ic < 0.0f) {
        cs->gain_c = -cs->gain_c;
        printk("[CS] Align: phase C gain inverted\n");
    }

    return 1;
}
