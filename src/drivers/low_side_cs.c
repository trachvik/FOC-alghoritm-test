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

    /*
     * Place CCR4 at 75% of the PWM period.
     *
     * In edge-aligned UP-counting PWM mode 1:
     *   OC4REF = HIGH while CNT < CCR4   → OC4REF = LOW from CNT = CCR4 to ARR
     * Falling edge of OC4REF at CNT = CCR4 → TRGO → ADC injected trigger.
     *
     * Safe-window requirement: CCR4 > CCR_max + dead_time_counts
     *   At I_limit=0.4A, Vq ≈ 5.6×0.4 = 2.24V, duty ≈ 44% → CCR_max ≈ 0.44×ARR
     *   dead_time ≈ 50 counts → safe window starts at ~0.44×ARR + 50 ≈ 45%
     *   CCR4 = 75% → 30% settling margin before trigger (>>enough).
     *
     * ADC completion margin: (ARR - CCR4) / f_CLK > t_ADC_conv (~2.5 µs)
     *   With CCR4 = 75%×ARR: remaining = 25%×ARR = ~10 µs >> 2.5 µs ✓
     *
     * JEXTEN = 0b10 (falling edge) in _configure_adc_injected().
     */
    uint32_t ccr4 = (tim1_arr >= 8U) ? (tim1_arr * 3U / 4U) : (tim1_arr / 2U);

    TIM1->CCMR2 = (TIM1->CCMR2
                   & ~(TIM_CCMR2_CC4S_Msk | TIM_CCMR2_OC4M_Msk | TIM_CCMR2_OC4PE))
                | (0b110U << TIM_CCMR2_OC4M_Pos);   /* PWM mode 1 */
    TIM1->CCER &= ~(TIM_CCER_CC4E | TIM_CCER_CC4P | TIM_CCER_CC4NP);  /* no GPIO, active-high */
    TIM1->CCR4 = ccr4;
    TIM1->CR2 = (TIM1->CR2 & ~TIM_CR2_MMS_Msk)
              | (0b111U << TIM_CR2_MMS_Pos);   /* MMS=111 → OC4REF → TRGO */

    printk("[CS] TIM1 ARR=%u CCR4=%u (trigger @ ~%u%% of period)\n",
           (unsigned)tim1_arr, (unsigned)ccr4,
           (unsigned)(100U * ccr4 / tim1_arr));
}

/* ---------------------------------------------------------------------------
 * _configure_adc_injected
 *
 * Routes ADC1 injected channels to TIM1_TRGO (which carries OC4REF falling
 * edge ≈ 97% into the PWM period — all low-side FETs guaranteed ON).
 *
 * Injected sequence (JL = 01 → 2 conversions):
 *   JSQ3 = CH0 / PA0 → phase W  (Ic) → JDR1   (1st conversion)
 *   JSQ4 = CH1 / PA1 → phase UV (Ia) → JDR2   (2nd conversion)
 *
 * Trigger:
 *   JEXTSEL = 0001 = TIM1_TRGO  (STM32F411 RM0383 Table 77)
 *   JEXTEN  = 10   = falling edge of OC4REF
 *
 * Both channels are sampled in a single hardware-triggered burst with no CPU
 * involvement.  Results are read from JDR1/JDR2 after JEOC flag is set.
 * This replaces the previous two sequential Zephyr adc_read() calls which
 * caused offset_ia ≈ 2.84 V (Ia was read one period late — high-side ON).
 * ------------------------------------------------------------------------- */
static void _configure_adc_injected(void)
{
    if (!device_is_ready(adc_dev)) {
        printk("[CS] ERROR: ADC device not ready!\n");
        return;
    }
    /* Let Zephyr configure the GPIO pin-mux and sample times (SMPR) only. */
    adc_channel_setup(adc_dev, &ch0_cfg);
    adc_channel_setup(adc_dev, &ch1_cfg);

    /*
     * Injected sequence register (JSQR):
     *   bits [21:20] JL   = 01   → 2 conversions (JSQ3 then JSQ4)
     *   bits [14:10] JSQ3 =  0   → CH0 = Ic (PA0) → result in JDR1
     *   bits [19:15] JSQ4 =  1   → CH1 = Ia (PA1) → result in JDR2
     */
    ADC1->JSQR = (1U << 20)   /* JL = 0b01 → 2 conversions */
               | (0U << 10)   /* JSQ3 = CH0 = Ic (PA0) → JDR1 */
               | (1U << 15);  /* JSQ4 = CH1 = Ia (PA1) → JDR2 */

    /*
     * CR2: configure external trigger for injected group.
     *   JEXTSEL [19:16] = 0001  → TIM1_TRGO
     *   JEXTEN  [21:20] = 10    → falling edge  (OC4REF falls at CNT = CCR4)
     * Clear any stale JEOC before activating the trigger.
     */
    /* SCAN=1 required for injected multi-channel sequence (JL=01 → 2 conversions) */
    ADC1->CR1 |= ADC_CR1_SCAN;

    /* Ensure ADC is powered on — Zephyr adc_channel_setup() configures pin-mux
     * and SMPR only; it does NOT call adc_read(), so ADON may still be 0.
     * Without ADON=1 the injected hardware trigger never fires (JEOC stays 0). */
    if (!(ADC1->CR2 & ADC_CR2_ADON)) {
        ADC1->CR2 |= ADC_CR2_ADON;
        /* tSTAB: negligible on STM32F4 but spin a moment to be safe */
        for (volatile int _i = 0; _i < 1000; _i++) {}
    }

    /* Hardware trigger: TIM1_TRGO falling edge (OC4REF via MMS=0b111 → TRGO).
     * JEXTSEL[3:0] = 0001 = TIM1_TRGO  (STM32F411 RM0383 Table 77)
     * JEXTEN[1:0]  = 10   = falling edge (OC4REF falls when CNT reaches CCR4) */
    ADC1->CR2 = (ADC1->CR2 & ~(ADC_CR2_JEXTSEL_Msk | ADC_CR2_JEXTEN_Msk))
              | (1U << ADC_CR2_JEXTSEL_Pos)   /* JEXTSEL = 0001 = TIM1_TRGO */
              | (2U << ADC_CR2_JEXTEN_Pos);   /* JEXTEN  = 10   = falling edge */

    ADC1->SR &= ~ADC_SR_JEOC;   /* clear any stale flag */

    printk("[CS] ADC1 injected: CH0\u2192JDR1(Ic) CH1\u2192JDR2(Ia), "
           "trigger=TIM1_TRGO falling edge @ CCR4=75%% period\n");
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
 * _wait_jeoc — wait for hardware-triggered injected conversion to complete,
 * then read both results.  Replaces the old Zephyr adc_read() path.
 *
 * Blocks until JEOC is set (max ~2 PWM periods = 80 µs at 25 kHz) or until
 * the timeout counter expires (~100 000 iterations ≈ a few ms at 100 MHz).
 * Returns false only on timeout (ADC or TIM1 not running).
 *
 * JDR1 = Ic (CH0 / PA0 / phase W)
 * JDR2 = Ia (CH1 / PA1 / phase UV)
 * ------------------------------------------------------------------------- */
static bool _wait_jeoc(uint16_t *raw_ic, uint16_t *raw_ia)
{
    /* Hardware fires injected conversion at CNT=CCR4 (75% of period) every
     * 40 µs.  JEOC is set ~2.5 µs after trigger.  Simply poll until set.
     * Timeout ≈ 100 000 iterations >> 2 full periods (safe for calibration). */
    uint32_t timeout = 100000U;
    while (!(ADC1->SR & ADC_SR_JEOC) && --timeout);
    if (!timeout) return false;
    ADC1->SR &= ~ADC_SR_JEOC;
    *raw_ic = (uint16_t)(ADC1->JDR1 & 0xFFFU);   /* Ic — 1st conv (JSQ3=CH0) */
    *raw_ia = (uint16_t)(ADC1->JDR2 & 0xFFFU);   /* Ia — 2nd conv (JSQ4=CH1) */
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

    /* Diagnostic: first 3 hardware-triggered samples to verify ADC firing */
    for (int d = 0; d < 3; d++) {
        uint16_t r_ic, r_ia;
        bool ok = _wait_jeoc(&r_ic, &r_ia);
        printk("[CS] diag[%d]: ok=%d CH0(Ic)=%u CH1(Ia)=%u\n",
               d, (int)ok, (unsigned)r_ic, (unsigned)r_ia);
    }

    for (int i = 0; i < ADC_CALIBRATION_ROUNDS; i++) {
        uint16_t raw_ic, raw_ia;
        if (!_wait_jeoc(&raw_ic, &raw_ia)) continue;

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

    /* --- Hardware-triggered injected ADC (non-blocking) -------------------
     * TIM1_TRGO (OC4REF falling edge at CNT = CCR4 ≈ 97% of period) fires
     * the ADC injected sequence automatically every PWM period.  Results land
     * in JDR1 (Ic) and JDR2 (Ia) with JEOC flag set on completion.
     *
     * Non-blocking: if JEOC is not set yet, return the cached value from the
     * previous period.  The FOC loop calls this once per ~40 µs period, so
     * JEOC will almost always be fresh.
     * ---------------------------------------------------------------------- */
    /* --- Hardware-triggered injected ADC (non-blocking) -------------------
     * TIM1_TRGO fires ADC injected sequence at CNT=CCR4 (75% period, ~30 µs
     * into each 40 µs PWM period).  JEOC is set ~2.5 µs later.
     * FOC loop runs at 1 kHz (1 ms) >> PWM period (40 µs): JEOC is always
     * fresh by the time we arrive here.  Non-blocking: return cached value
     * only if, somehow, we arrive before the first trigger fires.
     * ---------------------------------------------------------------------- */
    if (!(ADC1->SR & ADC_SR_JEOC)) return s_last_current;
    ADC1->SR &= ~ADC_SR_JEOC;

    uint16_t raw_ic = (uint16_t)(ADC1->JDR1 & 0xFFFU);   /* CH0 / Ic / PA0 */
    uint16_t raw_ia = (uint16_t)(ADC1->JDR2 & 0xFFFU);   /* CH1 / Ia / PA1 */

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

    /* Read a fresh hardware-triggered sample pair */
    uint16_t raw_ic, raw_ia;
    if (!_wait_jeoc(&raw_ic, &raw_ia)) return 0;   /* timeout */

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
