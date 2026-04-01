#include "bldc_driver_6pwm.h"
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/sys/printk.h>
#include <stm32f4xx.h>

/* Helper macros */
#define _ISSET(x) ((x) != NOT_SET)
#define _CONSTRAIN(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

/* Pin modes */
#define OUTPUT 1
#define INPUT 0

/* PWM devicetree nodes */
#define PWM_LOW_NODE DT_NODELABEL(pwm_low)
#define PWM_HIGH_NODE DT_NODELABEL(pwm_high)

/* PWM device instances */
static const struct device *pwm_low_dev = DEVICE_DT_GET(PWM_LOW_NODE);
static const struct device *pwm_high_dev = DEVICE_DT_GET(PWM_HIGH_NODE);

/* PWM configuration structure */
typedef struct {
    uint32_t period_ns;    /* PWM period in nanoseconds */
    bool configured;       /* Configuration status */
} pwm_6pwm_config_t;

static pwm_6pwm_config_t pwm_config = {0};

/* GPIO stubs - simplified */
static void pinMode(int pin, int mode)
{
    /* Stub - GPIO configured via devicetree */
}

static void digitalWrite(int pin, int value)
{
    /* Stub - GPIO handled via devicetree */
}

/**
 * Configure 6PWM timers
 */
static void* _configure6PWM(long pwm_frequency, float dead_zone,
                            int pwm_a_h, int pwm_a_l,
                            int pwm_b_h, int pwm_b_l,
                            int pwm_c_h, int pwm_c_l)
{
    /* Check if PWM devices are ready */
    if (!device_is_ready(pwm_low_dev)) {
        printk("ERROR: PWM low-side device not ready\n");
        return (void*)-1;
    }
    
    if (!device_is_ready(pwm_high_dev)) {
        printk("ERROR: PWM high-side device not ready\n");
        return (void*)-1;
    }
    
    /* Calculate PWM period in nanoseconds */
    if (pwm_frequency <= 0) {
        pwm_frequency = 25000;  /* Default 25 kHz */
    }
    pwm_config.period_ns = 1000000000UL / pwm_frequency;
    pwm_config.configured = true;
    
    printk("PWM configured: freq=%ld Hz, period=%u ns\n", 
           pwm_frequency, pwm_config.period_ns);
    
    /* Initialize all channels to 0% duty cycle.
     * Zephyr pwm_set_dt() also sets ARR = period_ns × f_CLK / 1e9 - 1.
     * At 96 MHz and period_ns=40000: ARR = 3839 for both TIM1 and TIM3. */
    pwm_set_dt(&(struct pwm_dt_spec){pwm_low_dev,  1, PWM_POLARITY_NORMAL}, pwm_config.period_ns, 0);
    pwm_set_dt(&(struct pwm_dt_spec){pwm_low_dev,  2, PWM_POLARITY_NORMAL}, pwm_config.period_ns, 0);
    pwm_set_dt(&(struct pwm_dt_spec){pwm_low_dev,  3, PWM_POLARITY_NORMAL}, pwm_config.period_ns, 0);
    pwm_set_dt(&(struct pwm_dt_spec){pwm_high_dev, 2, PWM_POLARITY_NORMAL}, pwm_config.period_ns, 0);
    pwm_set_dt(&(struct pwm_dt_spec){pwm_high_dev, 3, PWM_POLARITY_NORMAL}, pwm_config.period_ns, 0);
    pwm_set_dt(&(struct pwm_dt_spec){pwm_high_dev, 4, PWM_POLARITY_NORMAL}, pwm_config.period_ns, 0);

    /* -----------------------------------------------------------------------
     * Post-init hardware configuration (bypasses Zephyr PWM API).
     *
     * PROBLEM with naive two-timer design:
     *   TIM3 (high-side) and TIM1 (low-side) in edge-aligned PWM mode 1,
     *   both with CCR starting from 0, produce OVERLAPPING on-windows:
     *   high-side ON for [0 → dc×ARR] and low-side originally ON for
     *   [0 → (1-dc)×ARR] → shoot-through from 0 to min(dc, 1-dc)×ARR.
     *
     * FIX — three-part:
     *
     * 1. Invert TIM1 CH1/CH2/CH3 output polarity (CCER[CC1P/CC2P/CC3P]=1).
     *    With PWM mode 1 and active-low polarity:
     *      CH1 HIGH (gate open = FET ON) when CNT ≥ CCR1.
     *    We set CCR1 = (dc + dead_zone) × ARR, so the low-side turns ON only
     *    AFTER the high-side has turned OFF plus the dead-time gap:
     *      [0 → dc×ARR]         high-side ON
     *      [dc×ARR → (dc+dt)×ARR]  BOTH OFF (dead zone = dt)
     *      [(dc+dt)×ARR → ARR]   low-side ON  ← ADC samples here (97% of ARR)
     *
     * 2. Set TIM1 as MASTER (MMS=010 = Update event → TRGO on each overflow).
     *    The ADC is triggered via TIM1_CC4 (JEXTSEL=0000), NOT via TRGO, so
     *    TRGO is free for timer synchronisation.
     *
     * 3. Set TIM3 as SLAVE (SMS=100=Reset, TS=000=ITR0=TIM1_TRGO).
     *    TIM3 counter resets to 0 on every TIM1 update pulse, keeping both
     *    timers phase-locked despite Zephyr initialising them separately.
     * ----------------------------------------------------------------------- */

    /* 1. Invert TIM1 CH1/2/3 output polarity → low-side ON in tail of period */
    TIM1->CCER = (TIM1->CCER
                  & ~(TIM_CCER_CC1P | TIM_CCER_CC2P | TIM_CCER_CC3P))
               | TIM_CCER_CC1P | TIM_CCER_CC2P | TIM_CCER_CC3P;

    /* 2. TIM1 master: TRGO = Update event (fires on every ARR overflow) */
    TIM1->CR2 = (TIM1->CR2 & ~TIM_CR2_MMS_Msk)
              | (0b010U << TIM_CR2_MMS_Pos);   /* MMS=010 → Update → TRGO */

    /* 3. TIM3 slave: reset counter on TIM1_TRGO (ITR0 for TIM3) */
    TIM3->SMCR = (TIM3->SMCR
                  & ~(TIM_SMCR_SMS_Msk | TIM_SMCR_TS_Msk))
               | (0b100U << TIM_SMCR_SMS_Pos)   /* SMS=100 = Reset slave mode */
               | (0b000U << TIM_SMCR_TS_Pos);   /* TS=000  = ITR0 = TIM1_TRGO */

    printk("[6PWM] TIM1 ARR=%u (polarity inverted, master)  "
           "TIM3 ARR=%u (edge-aligned, slave)\n",
           (unsigned)TIM1->ARR, (unsigned)TIM3->ARR);

    return &pwm_config;
}

/**
 * Write duty cycle to 6PWM hardware — direct register access.
 *
 * After low_side_cs_init() switches TIM1 to center-aligned (ARR=2000),
 * the Zephyr pwm_set_dt() path would overwrite ARR back to 3999 on every
 * call (it recalculates ARR from period_ns each time).  We bypass it and
 * write CCR directly so center-aligned mode + ARR=2000 are preserved.
 *
 * Hardware wiring (from app.overlay / schematics):
 *   Phase A (U):  TIM1_CH1 (PA8)  = low-side,  TIM3_CH2 (PB5)  = high-side
 *   Phase B (V):  TIM1_CH2 (PA9)  = low-side,  TIM3_CH3 (PB0)  = high-side
 *   Phase C (W):  TIM1_CH3 (PA10) = low-side,  TIM3_CH4 (PB1)  = high-side
 *
 * TIM1 low-side channels: complementary outputs (CH1N/CH2N/CH3N used by TMC6300).
 * TIM3 high-side channels: standard PWM outputs.
 *
 * Both timers are edge-aligned at ARR=3839 (96 MHz / 25 kHz − 1).
 * TIM3 is phase-locked to TIM1 via slave-reset mode (see _configure6PWM).
 *
 * Duty convention:
 *   TIM3 high-side : CCR3 = dc × ARR             → ON from 0 to dc×period
 *   TIM1 low-side  : CCR1 = (dc + dt) × ARR       → INVERTED output polarity
 *                    (TIM1 CCER[CC1P]=1 → HIGH when CNT ≥ CCR1)
 *                    → ON from (dc+dt)×period to end-of-period
 *
 * Dead time is "software" dead time implemented by the duty offset (dt_frac).
 * Hardware BDTR dead-time is NOT used (TIM3 and TIM1 have no shared output).
 *
 * ARR values (read from registers after Zephyr pwm_set_dt init):
 *   Both TIM1 and TIM3: ARR = 3839  (96 MHz / 25 kHz − 1)
 */
#define TIM_EA_ARR    3839U    /* edge-aligned 25 kHz @ 96 MHz                */
#define DEAD_ZONE_FRAC  0.02f  /* 2% = 800 ns dead zone between HS OFF and LS ON */

static void _writeDutyCycle6PWM(float dc_a, float dc_b, float dc_c,
                                phase_state_t *phase_state, void *params)
{
    ARG_UNUSED(params);

    /* Clamp duty to [0, 1] */
    if (dc_a < 0.0f) dc_a = 0.0f; if (dc_a > 1.0f) dc_a = 1.0f;
    if (dc_b < 0.0f) dc_b = 0.0f; if (dc_b > 1.0f) dc_b = 1.0f;
    if (dc_c < 0.0f) dc_c = 0.0f; if (dc_c > 1.0f) dc_c = 1.0f;

    /* TIM3 high-side (edge-aligned, normal polarity):
     *   CCR3 = dc × ARR  →  CH HIGH for [0, dc×period] */
    uint32_t ccr3_a = (phase_state[0] == PHASE_ON || phase_state[0] == PHASE_HI)
                      ? (uint32_t)(dc_a * TIM_EA_ARR) : 0U;
    uint32_t ccr3_b = (phase_state[1] == PHASE_ON || phase_state[1] == PHASE_HI)
                      ? (uint32_t)(dc_b * TIM_EA_ARR) : 0U;
    uint32_t ccr3_c = (phase_state[2] == PHASE_ON || phase_state[2] == PHASE_HI)
                      ? (uint32_t)(dc_c * TIM_EA_ARR) : 0U;

    /* TIM1 low-side (edge-aligned, INVERTED polarity via CCER[CC1P]=1):
     *   CCR1 = (dc + dead_zone) × ARR
     *   With CC1P=1: CH HIGH when CNT ≥ CCR1 → low-side ON for [(dc+dt), 1] of period.
     *   Natural dead zone = dead_zone × period between HS turn-off and LS turn-on. */
    uint32_t ccr1_a = (phase_state[0] == PHASE_ON || phase_state[0] == PHASE_LO)
                      ? (uint32_t)((dc_a + DEAD_ZONE_FRAC) * TIM_EA_ARR) : TIM_EA_ARR;
    uint32_t ccr1_b = (phase_state[1] == PHASE_ON || phase_state[1] == PHASE_LO)
                      ? (uint32_t)((dc_b + DEAD_ZONE_FRAC) * TIM_EA_ARR) : TIM_EA_ARR;
    uint32_t ccr1_c = (phase_state[2] == PHASE_ON || phase_state[2] == PHASE_LO)
                      ? (uint32_t)((dc_c + DEAD_ZONE_FRAC) * TIM_EA_ARR) : TIM_EA_ARR;
    /* Clamp to ARR so CCR never exceeds ARR (which would keep low-side always ON) */
    if (ccr1_a > TIM_EA_ARR) ccr1_a = TIM_EA_ARR;
    if (ccr1_b > TIM_EA_ARR) ccr1_b = TIM_EA_ARR;
    if (ccr1_c > TIM_EA_ARR) ccr1_c = TIM_EA_ARR;

    /* Write TIM3 high-side CCRs (CH2=A, CH3=B, CH4=C) */
    TIM3->CCR2 = ccr3_a;
    TIM3->CCR3 = ccr3_b;
    TIM3->CCR4 = ccr3_c;

    /* Write TIM1 low-side CCRs (CH1=A, CH2=B, CH3=C) */
    TIM1->CCR1 = ccr1_a;
    TIM1->CCR2 = ccr1_b;
    TIM1->CCR3 = ccr1_c;
}

/**
 * Initialize BLDC 6PWM driver structure
 */
void bldc_driver_6pwm_init_struct(bldc_driver_6pwm_t *driver,
                                  int ph_a_h, int ph_a_l,
                                  int ph_b_h, int ph_b_l,
                                  int ph_c_h, int ph_c_l,
                                  int en)
{
    if (driver == NULL) {
        return;
    }
    
    /* Initialize pin numbers */
    driver->pwm_a_h = ph_a_h;
    driver->pwm_a_l = ph_a_l;
    driver->pwm_b_h = ph_b_h;
    driver->pwm_b_l = ph_b_l;
    driver->pwm_c_h = ph_c_h;
    driver->pwm_c_l = ph_c_l;
    driver->enable_pin = en;
    
    /* Default configuration */
    driver->voltage_power_supply = DEF_POWER_SUPPLY;
    driver->voltage_limit = NOT_SET;
    driver->pwm_frequency = NOT_SET;
    driver->dead_zone = 0.02f;  /* 2% dead zone */
    
    /* Initialize state */
    driver->dc_a = 0.0f;
    driver->dc_b = 0.0f;
    driver->dc_c = 0.0f;
    driver->phase_state[0] = PHASE_OFF;
    driver->phase_state[1] = PHASE_OFF;
    driver->phase_state[2] = PHASE_OFF;
    
    driver->initialized = false;
    driver->enable_active_high = true;
    driver->params = NULL;
}

/**
 * Initialize driver hardware
 */
int bldc_driver_6pwm_init_hw(bldc_driver_6pwm_t *driver)
{
    if (driver == NULL) {
        return DRIVER_INIT_FAILED;
    }
    
    /* Configure PWM pins as outputs */
    pinMode(driver->pwm_a_h, OUTPUT);
    pinMode(driver->pwm_b_h, OUTPUT);
    pinMode(driver->pwm_c_h, OUTPUT);
    pinMode(driver->pwm_a_l, OUTPUT);
    pinMode(driver->pwm_b_l, OUTPUT);
    pinMode(driver->pwm_c_l, OUTPUT);
    
    if (_ISSET(driver->enable_pin)) {
        pinMode(driver->enable_pin, OUTPUT);
    }
    
    /* Sanity check for voltage limit */
    if (!_ISSET(driver->voltage_limit) || 
        driver->voltage_limit > driver->voltage_power_supply) {
        driver->voltage_limit = driver->voltage_power_supply;
    }
    
    /* Set initial phase states to disabled */
    driver->phase_state[0] = PHASE_OFF;
    driver->phase_state[1] = PHASE_OFF;
    driver->phase_state[2] = PHASE_OFF;
    
    /* Set zero duty cycle */
    driver->dc_a = 0.0f;
    driver->dc_b = 0.0f;
    driver->dc_c = 0.0f;
    
    /* Configure 6PWM - hardware specific function */
    driver->params = _configure6PWM(driver->pwm_frequency,
                                   driver->dead_zone,
                                   driver->pwm_a_h, driver->pwm_a_l,
                                   driver->pwm_b_h, driver->pwm_b_l,
                                   driver->pwm_c_h, driver->pwm_c_l);
    
    driver->initialized = (driver->params != (void*)DRIVER_INIT_FAILED);
    
    return driver->initialized ? DRIVER_INIT_OK : DRIVER_INIT_FAILED;
}

/**
 * Enable motor driver
 */
void bldc_driver_6pwm_enable(bldc_driver_6pwm_t *driver)
{
    if (driver == NULL) {
        return;
    }
    
    /* Enable the driver via enable pin if available */
    if (_ISSET(driver->enable_pin)) {
        digitalWrite(driver->enable_pin, driver->enable_active_high ? 1 : 0);
    }
    
    /* Set phase states to enabled */
    bldc_driver_6pwm_set_phase_state(driver, PHASE_ON, PHASE_ON, PHASE_ON);
    
    /* Set zero PWM */
    bldc_driver_6pwm_set_pwm(driver, 0.0f, 0.0f, 0.0f);
}

/**
 * Disable motor driver
 */
void bldc_driver_6pwm_disable(bldc_driver_6pwm_t *driver)
{
    if (driver == NULL) {
        return;
    }
    
    /* Set phase states to disabled */
    bldc_driver_6pwm_set_phase_state(driver, PHASE_OFF, PHASE_OFF, PHASE_OFF);
    
    /* Set zero PWM */
    bldc_driver_6pwm_set_pwm(driver, 0.0f, 0.0f, 0.0f);
    
    /* Disable the driver via enable pin if available */
    if (_ISSET(driver->enable_pin)) {
        digitalWrite(driver->enable_pin, driver->enable_active_high ? 0 : 1);
    }
}

/**
 * Set phase voltages to the hardware
 */
void bldc_driver_6pwm_set_pwm(bldc_driver_6pwm_t *driver,
                              float ua, float ub, float uc)
{
    if (driver == NULL) {
        return;
    }
    
    /* Limit the voltage in driver */
    ua = _CONSTRAIN(ua, 0.0f, driver->voltage_limit);
    ub = _CONSTRAIN(ub, 0.0f, driver->voltage_limit);
    uc = _CONSTRAIN(uc, 0.0f, driver->voltage_limit);
    
    /* Calculate duty cycle - limited in [0,1] */
    driver->dc_a = _CONSTRAIN(ua / driver->voltage_power_supply, 0.0f, 1.0f);
    driver->dc_b = _CONSTRAIN(ub / driver->voltage_power_supply, 0.0f, 1.0f);
    driver->dc_c = _CONSTRAIN(uc / driver->voltage_power_supply, 0.0f, 1.0f);
    
    /* Write to hardware - hardware specific function */
    _writeDutyCycle6PWM(driver->dc_a, driver->dc_b, driver->dc_c,
                        driver->phase_state, driver->params);
}

/**
 * Set phase states
 */
void bldc_driver_6pwm_set_phase_state(bldc_driver_6pwm_t *driver,
                                      phase_state_t sa,
                                      phase_state_t sb,
                                      phase_state_t sc)
{
    if (driver == NULL) {
        return;
    }
    
    driver->phase_state[0] = sa;
    driver->phase_state[1] = sb;
    driver->phase_state[2] = sc;
}
