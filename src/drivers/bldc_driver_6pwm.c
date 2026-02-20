#include "bldc_driver_6pwm.h"
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/sys/printk.h>

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
    
    /* Initialize all channels to 0% duty cycle */
    pwm_set_dt(&(struct pwm_dt_spec){pwm_low_dev, 1, PWM_POLARITY_NORMAL}, 
               pwm_config.period_ns, 0);
    pwm_set_dt(&(struct pwm_dt_spec){pwm_low_dev, 2, PWM_POLARITY_NORMAL}, 
               pwm_config.period_ns, 0);
    pwm_set_dt(&(struct pwm_dt_spec){pwm_low_dev, 3, PWM_POLARITY_NORMAL}, 
               pwm_config.period_ns, 0);
    pwm_set_dt(&(struct pwm_dt_spec){pwm_high_dev, 2, PWM_POLARITY_NORMAL}, 
               pwm_config.period_ns, 0);
    pwm_set_dt(&(struct pwm_dt_spec){pwm_high_dev, 3, PWM_POLARITY_NORMAL}, 
               pwm_config.period_ns, 0);
    pwm_set_dt(&(struct pwm_dt_spec){pwm_high_dev, 4, PWM_POLARITY_NORMAL}, 
               pwm_config.period_ns, 0);
    
    return &pwm_config;
}

/**
 * Write duty cycle to 6PWM hardware
 */
static void _writeDutyCycle6PWM(float dc_a, float dc_b, float dc_c,
                                phase_state_t *phase_state, void *params)
{
    if (!pwm_config.configured) {
        return;
    }
    
    /* Convert duty cycle [0.0-1.0] to pulse width in nanoseconds */
    uint32_t pulse_a = (uint32_t)(dc_a * pwm_config.period_ns);
    uint32_t pulse_b = (uint32_t)(dc_b * pwm_config.period_ns);
    uint32_t pulse_c = (uint32_t)(dc_c * pwm_config.period_ns);
    
    /* Constrain to valid range */
    if (pulse_a > pwm_config.period_ns) pulse_a = pwm_config.period_ns;
    if (pulse_b > pwm_config.period_ns) pulse_b = pwm_config.period_ns;
    if (pulse_c > pwm_config.period_ns) pulse_c = pwm_config.period_ns;
    
    /* Phase A (U) - TIM3_CH2 (high), TIM1_CH1 (low) */
    if (phase_state[0] == PHASE_ON || phase_state[0] == PHASE_HI) {
        pwm_set_dt(&(struct pwm_dt_spec){pwm_high_dev, 2, PWM_POLARITY_NORMAL},
                   pwm_config.period_ns, pulse_a);
    } else {
        pwm_set_dt(&(struct pwm_dt_spec){pwm_high_dev, 2, PWM_POLARITY_NORMAL},
                   pwm_config.period_ns, 0);
    }
    
    if (phase_state[0] == PHASE_ON || phase_state[0] == PHASE_LO) {
        pwm_set_dt(&(struct pwm_dt_spec){pwm_low_dev, 1, PWM_POLARITY_NORMAL},
                   pwm_config.period_ns, pwm_config.period_ns - pulse_a);
    } else {
        pwm_set_dt(&(struct pwm_dt_spec){pwm_low_dev, 1, PWM_POLARITY_NORMAL},
                   pwm_config.period_ns, 0);
    }
    
    /* Phase B (V) - TIM3_CH3 (high), TIM1_CH2 (low) */
    if (phase_state[1] == PHASE_ON || phase_state[1] == PHASE_HI) {
        pwm_set_dt(&(struct pwm_dt_spec){pwm_high_dev, 3, PWM_POLARITY_NORMAL},
                   pwm_config.period_ns, pulse_b);
    } else {
        pwm_set_dt(&(struct pwm_dt_spec){pwm_high_dev, 3, PWM_POLARITY_NORMAL},
                   pwm_config.period_ns, 0);
    }
    
    if (phase_state[1] == PHASE_ON || phase_state[1] == PHASE_LO) {
        pwm_set_dt(&(struct pwm_dt_spec){pwm_low_dev, 2, PWM_POLARITY_NORMAL},
                   pwm_config.period_ns, pwm_config.period_ns - pulse_b);
    } else {
        pwm_set_dt(&(struct pwm_dt_spec){pwm_low_dev, 2, PWM_POLARITY_NORMAL},
                   pwm_config.period_ns, 0);
    }
    
    /* Phase C (W) - TIM3_CH4 (high), TIM1_CH3 (low) */
    if (phase_state[2] == PHASE_ON || phase_state[2] == PHASE_HI) {
        pwm_set_dt(&(struct pwm_dt_spec){pwm_high_dev, 4, PWM_POLARITY_NORMAL},
                   pwm_config.period_ns, pulse_c);
    } else {
        pwm_set_dt(&(struct pwm_dt_spec){pwm_high_dev, 4, PWM_POLARITY_NORMAL},
                   pwm_config.period_ns, 0);
    }
    
    if (phase_state[2] == PHASE_ON || phase_state[2] == PHASE_LO) {
        pwm_set_dt(&(struct pwm_dt_spec){pwm_low_dev, 3, PWM_POLARITY_NORMAL},
                   pwm_config.period_ns, pwm_config.period_ns - pulse_c);
    } else {
        pwm_set_dt(&(struct pwm_dt_spec){pwm_low_dev, 3, PWM_POLARITY_NORMAL},
                   pwm_config.period_ns, 0);
    }
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
