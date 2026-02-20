/**
 * Platform-specific implementation for Zephyr RTOS
 * Provides Arduino-like API compatibility layer
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/sys/printk.h>
#include "bldc_motor.h"
#include "bldc_driver_6pwm.h"
#include "as5048a.h"

/* Pin mode constants */
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

/**
 * Time functions
 */
unsigned long micros(void)
{
    return k_cyc_to_us_floor64(k_cycle_get_64());
}

void delay_ms(unsigned long ms)
{
    k_msleep(ms);
}

/**
 * GPIO functions - simplified stubs
 * TODO: Implement proper GPIO control using Zephyr GPIO API
 */
void pinMode(int pin, int mode)
{
    /* Stub implementation */
    /* In real implementation, configure GPIO pin using Zephyr API */
}

void digitalWrite(int pin, int value)
{
    /* Stub implementation */
    /* In real implementation, set GPIO pin using Zephyr API */
}

/**
 * 6PWM configuration
 * Configure PWM timers for 6-channel motor control
 */
void* _configure6PWM(long pwm_frequency, float dead_zone,
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
 * Write duty cycle to 6PWM
 * Outputs actual PWM signals to motor driver
 */
void _writeDutyCycle6PWM(float dc_a, float dc_b, float dc_c,
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
    
    /* Debug output occasionally */
    static int call_count = 0;
    if (call_count++ % 500 == 0) {
        printk("PWM: A=%.3f B=%.3f C=%.3f\n",
               (double)dc_a, (double)dc_b, (double)dc_c);
    }
}

/**
 * Sensor interface functions
 */

/* Global pointer to AS5048A device - set from main */
extern struct as5048a_device *g_as5048a;

void sensor_update(sensor_t *sensor)
{
    /* AS5048A is read on-demand, no update needed */
}

float sensor_get_angle(sensor_t *sensor)
{
    if (g_as5048a == NULL) return 0.0f;
    
    float angle_rad = 0.0f;
    if (as5048a_read_angle_rad(g_as5048a, &angle_rad) == 0) {
        return angle_rad;
    }
    return 0.0f;
}

float sensor_get_velocity(sensor_t *sensor)
{
    /* Calculate velocity from angle difference */
    /* TODO: Implement velocity calculation */
    return 0.0f;
}

bool sensor_needs_search(sensor_t *sensor)
{
    /* No index search needed for absolute encoder */
    return false;
}

/**
 * Driver interface functions
 */
void driver_set_pwm(bldc_driver_t *driver, float ua, float ub, float uc)
{
    if (driver == NULL) return;
    
    /* Cast to 6PWM driver and call its function */
    bldc_driver_6pwm_t *drv = (bldc_driver_6pwm_t*)driver;
    bldc_driver_6pwm_set_pwm(drv, ua, ub, uc);
}

void driver_enable(bldc_driver_t *driver)
{
    if (driver == NULL) return;
    
    /* Cast to 6PWM driver and call its function */
    bldc_driver_6pwm_t *drv = (bldc_driver_6pwm_t*)driver;
    bldc_driver_6pwm_enable(drv);
}

void driver_disable(bldc_driver_t *driver)
{
    if (driver == NULL) return;
    
    /* Cast to 6PWM driver and call its function */
    bldc_driver_6pwm_t *drv = (bldc_driver_6pwm_t*)driver;
    bldc_driver_6pwm_disable(drv);
}

bool driver_is_initialized(bldc_driver_t *driver)
{
    if (driver == NULL) return false;
    
    bldc_driver_6pwm_t *drv = (bldc_driver_6pwm_t*)driver;
    return drv->initialized;
}

float driver_get_voltage_limit(bldc_driver_t *driver)
{
    if (driver == NULL) return 0.0f;
    
    bldc_driver_6pwm_t *drv = (bldc_driver_6pwm_t*)driver;
    return drv->voltage_limit;
}

/**
 * Current sense interface functions
 */
void current_sense_enable(current_sense_t *cs)
{
    /* Stub - no current sensing yet */
}

void current_sense_disable(current_sense_t *cs)
{
    /* Stub - no current sensing yet */
}

bool current_sense_is_initialized(current_sense_t *cs)
{
    /* Stub - no current sensing yet */
    return false;
}

int current_sense_driver_align(current_sense_t *cs, float voltage, int8_t modulation_centered)
{
    /* Stub - no current sensing yet */
    return 1;  /* Return success */
}
