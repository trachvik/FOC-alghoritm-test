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
    /*
     * Strategy: do NOT stop TIM1 or reset it via RCC.  Stopping TIM1 while
     * Zephyr's 6PWM driver owns it causes a hard-fault (Zephyr IRQ handler
     * sees the timer in an unexpected state).
     *
     * CCMR2, CCER, CCR4 and CR2 can all be written safely while the timer
     * is running (CEN=1).  We only add CH4 as an internal compare output
     * (no GPIO) to generate OC4REF, which is routed to TRGO via MMS=0b111.
     * ADC injected trigger: JEXTSEL=0b0001 = TIM1_TRGO.
     *
     * TIM1 remains in whichever mode the 6PWM driver left it (edge-aligned,
     * ARR = f_CLK/f_PWM - 1).  Sampling at CCR4 counts from period start is
     * valid for low-side current sense as long as CCR4 < min(CCRx) of any
     * active phase (i.e. within the guaranteed all-low-side-on window).
     */

    /* Read actual ARR — do NOT change it */
    uint32_t tim1_arr = TIM1->ARR;
    uint32_t offset_counts = (uint32_t)(
        ((uint64_t)offset_trough_ns * TIM1_CLOCK_HZ) / 1000000000ULL);
    if (offset_counts == 0U) offset_counts = 1U;
    if (offset_counts > tim1_arr / 4U) offset_counts = tim1_arr / 4U;

    printk("[TIM1] CEN=%u ARR=%u, target CCR4=%u (offset %u ns)\n",
           (unsigned)!!(TIM1->CR1 & TIM_CR1_CEN),
           (unsigned)tim1_arr, (unsigned)offset_counts,
           (unsigned)offset_trough_ns);
    k_msleep(30);

    /* CH4: output compare, PWM mode 1, no preload, CC4 GPIO output disabled.
     * OC4REF = 1 while CNT < CCR4 → rising edge at CNT==0 (period start). */
    printk("[TIM1] CCMR2...\n"); k_msleep(30);
    TIM1->CCMR2 = (TIM1->CCMR2
                   & ~(TIM_CCMR2_CC4S_Msk | TIM_CCMR2_OC4M_Msk | TIM_CCMR2_OC4PE))
                | (0b110U << TIM_CCMR2_OC4M_Pos);   /* PWM mode 1 */

    printk("[TIM1] CCER...\n"); k_msleep(30);
    TIM1->CCER &= ~TIM_CCER_CC4E;   /* no GPIO output */

    printk("[TIM1] CCR4=%u...\n", (unsigned)offset_counts); k_msleep(30);
    TIM1->CCR4 = offset_counts;

    /* Route OC4REF → TRGO so ADC injected trigger (JEXTSEL=TIM1_TRGO) fires
     * once per PWM period at the current-sense sampling point.             */
    printk("[TIM1] CR2 MMS=OC4REF...\n"); k_msleep(30);
    TIM1->CR2 = (TIM1->CR2 & ~TIM_CR2_MMS_Msk)
              | (0b111U << TIM_CR2_MMS_Pos);   /* MMS=111 → OC4REF → TRGO */

    printk("[CS-HW] TIM1 CH4/TRGO OK: ARR=%u CCR4=%u offset_ns=%u\n",
           (unsigned)TIM1->ARR, (unsigned)TIM1->CCR4,
           (unsigned)offset_trough_ns);
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
    printk("[ADC] checking device ready...\n"); k_msleep(30);
    /* Pin-mux via Zephyr (must be done before touching registers) */
    if (!device_is_ready(adc_dev)) {
        printk("[CS-HW] ADC device not ready!\n");
        return;
    }
    printk("[ADC] ch0 setup...\n"); k_msleep(30);
    adc_channel_setup(adc_dev, &ch0_cfg);
    printk("[ADC] ch1 setup...\n"); k_msleep(30);
    adc_channel_setup(adc_dev, &ch1_cfg);

    /*
     * Do NOT power-cycle the ADC (ADON=0 then ADON=1).
     * Zephyr's STM32 ADC driver may use DMA; turning ADON off while DMA is
     * active generates a DMA error interrupt → hard fault.
     * On STM32F4, injected sequence registers (JSQR, CR2.JEXTSEL/JEXTEN) can
     * be written safely while ADON=1 as long as no injected conversion is
     * currently in progress (which is guaranteed here — we haven't triggered
     * any).
     *
     * JEOCIE is intentionally left disabled: _read_adc_raw() polls SR.JEOC,
     * so no interrupt is needed.  Enabling JEOCIE without a Zephyr-registered
     * ISR entry would cause an unhandled-interrupt fault the first time the
     * injected group completes.
     */

    printk("[ADC] writing JSQR...\n"); k_msleep(30);
    /* Injected sequence: 2 conversions
     *   JL = 1 (value 1 → 2 conversions)
     *   JSQ3 = CH0 (PA0, phase W / Ic)  → result in JDR1
     *   JSQ4 = CH1 (PA1, phase UV / Ia) → result in JDR2             */
    ADC1->JSQR = ((2U - 1U) << ADC_JSQR_JL_Pos)
               | (0U        << ADC_JSQR_JSQ3_Pos)
               | (1U        << ADC_JSQR_JSQ4_Pos);

    printk("[ADC] setting up channels via Zephyr API...\n"); k_msleep(30);
    /* Only need Zephyr channel setup for pin-mux (MODER=analog, SMPR, SQR).
     * All further reads use adc_read() so we do NOT touch CR2/JSQR manually.
     * Zephyr's STM32 ADC driver owns CR2; raw JSWSTART writes conflict. */
    printk("[CS-HW] ADC1 using Zephyr adc_read() — no raw CR2/JSQR writes\n");
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
 * Zephyr adc_read() sequences — one per channel to avoid DR overrun.
 * STM32F4 has a single shared DR register; reading 2 channels in one
 * sequence without DMA causes OVR (CH1 overwrites CH0 before ISR reads it).
 * Two separate single-channel reads avoids this entirely.
 * ------------------------------------------------------------------------- */
static int16_t s_buf_ic;   /* CH0 / PA0 / phase W  (Ic) */
static int16_t s_buf_ia;   /* CH1 / PA1 / phase UV (Ia) */

static struct adc_sequence s_seq_ic = {
    .channels    = BIT(0),
    .buffer      = &s_buf_ic,
    .buffer_size = sizeof(s_buf_ic),
    .resolution  = 12,
};
static struct adc_sequence s_seq_ia = {
    .channels    = BIT(1),
    .buffer      = &s_buf_ia,
    .buffer_size = sizeof(s_buf_ia),
    .resolution  = 12,
};

/* ---------------------------------------------------------------------------
 * _read_adc_raw — read both shunt channels (sequential single-channel reads).
 * Returns true on success.
 * ------------------------------------------------------------------------- */
static bool _read_adc_raw(uint16_t *raw_ic, uint16_t *raw_ia)
{
    if (adc_read(adc_dev, &s_seq_ic) != 0) return false;
    if (adc_read(adc_dev, &s_seq_ia) != 0) return false;
    *raw_ic = (uint16_t)s_buf_ic;   /* PA0 = phase W  (Ic) */
    *raw_ia = (uint16_t)s_buf_ia;   /* PA1 = phase UV (Ia) */
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

    /* Diagnostic: first 3 raw samples to verify ADC is actually reading */
    for (int d = 0; d < 3; d++) {
        uint16_t r_ic, r_ia;
        bool ok = _read_adc_raw(&r_ic, &r_ia);
        printk("[CS] diag[%d]: ok=%d CH0(Ic)=%u CH1(Ia)=%u\n",
               d, (int)ok, (unsigned)r_ic, (unsigned)r_ia);
    }

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

    /* --- Synchronize to the low-side-on sampling window -------------------
     * TIM1 (edge-aligned up-counter, 25 kHz) drives the 3 low-side FETs via
     * CH1/CH2/CH3 in PWM mode 1.  All three low-sides are ON simultaneously
     * during the interval CNT ∈ [0, min(CCR1,CCR2,CCR3)].
     *
     * At the maximum haptic duty (0.32 A × 5.6 Ω = 1.79 V on a 4.2 V bus),
     * max high-side duty ≈ 43 %, so the simultaneous ON window is ≥ 57 % of
     * the 40 µs period ≈ 23 µs = 2300 TIM1 counts.
     *
     * Strategy: spin until CNT < 200 (2 µs from period start), then do both
     * adc_read() calls (~2 µs each).  Both reads finish at CNT ≈ 600, well
     * inside the 2300-count window.  Worst-case spin = one PWM period = 40 µs.
     * ---------------------------------------------------------------------- */
    uint32_t cnt;
    do { cnt = TIM1->CNT; } while (cnt > 200U);

    if (adc_read(adc_dev, &s_seq_ic) != 0) return s_last_current;
    if (adc_read(adc_dev, &s_seq_ia) != 0) return s_last_current;

    uint16_t raw_ic = (uint16_t)s_buf_ic;   /* Ic = phase W  */
    uint16_t raw_ia = (uint16_t)s_buf_ia;   /* Ia = phase UV */

    cs->last_raw_ic = (int16_t)raw_ic;
    cs->last_raw_ia = (int16_t)raw_ia;

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
