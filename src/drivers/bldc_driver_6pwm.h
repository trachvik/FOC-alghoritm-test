#ifndef BLDC_DRIVER_6PWM_H
#define BLDC_DRIVER_6PWM_H

#include <stdint.h>
#include <stdbool.h>

/* Phase state enum */
typedef enum {
    PHASE_OFF = 0,  /* both sides of the phase are off */
    PHASE_ON = 1,   /* both sides of the phase are driven with PWM, dead time is applied */
    PHASE_HI = 2,   /* only the high side of the phase is driven with PWM */
    PHASE_LO = 3,   /* only the low side of the phase is driven with PWM */
} phase_state_t;

/* NOT_SET constant */
#ifndef NOT_SET
#define NOT_SET -12345.0f
#endif

/* Default power supply voltage */
#define DEF_POWER_SUPPLY 12.0f

/* Driver initialization return codes */
#define DRIVER_INIT_OK 0
#define DRIVER_INIT_FAILED -1

/**
 * BLDC 6PWM Driver structure
 * 
 * This driver controls a BLDC motor using 6 PWM signals
 * (high and low side for each of the 3 phases)
 */
typedef struct {
    /* PWM pin numbers */
    int pwm_a_h;  /* phase A high-side PWM pin */
    int pwm_a_l;  /* phase A low-side PWM pin */
    int pwm_b_h;  /* phase B high-side PWM pin */
    int pwm_b_l;  /* phase B low-side PWM pin */
    int pwm_c_h;  /* phase C high-side PWM pin */
    int pwm_c_l;  /* phase C low-side PWM pin */
    int enable_pin; /* enable pin number (optional) */
    
    /* Configuration */
    float dead_zone;  /* dead-time percentage [0,1] for each PWM cycle */
    float voltage_power_supply; /* power supply voltage */
    float voltage_limit; /* limiting voltage set to the motor */
    long pwm_frequency; /* PWM frequency in Hz */
    
    /* State */
    float dc_a; /* currently set duty cycle on phase A */
    float dc_b; /* currently set duty cycle on phase B */
    float dc_c; /* currently set duty cycle on phase C */
    phase_state_t phase_state[3]; /* phase state (active/disabled) */
    
    bool initialized; /* true if driver was successfully initialized */
    bool enable_active_high; /* enable pin polarity */
    void *params; /* pointer to hardware specific parameters */
} bldc_driver_6pwm_t;

/**
 * Initialize BLDC 6PWM driver structure
 * 
 * @param driver Pointer to driver structure
 * @param ph_a_h Phase A high-side PWM pin
 * @param ph_a_l Phase A low-side PWM pin
 * @param ph_b_h Phase B high-side PWM pin
 * @param ph_b_l Phase B low-side PWM pin
 * @param ph_c_h Phase C high-side PWM pin
 * @param ph_c_l Phase C low-side PWM pin
 * @param en Enable pin (use NOT_SET if not used)
 */
void bldc_driver_6pwm_init_struct(bldc_driver_6pwm_t *driver,
                                  int ph_a_h, int ph_a_l,
                                  int ph_b_h, int ph_b_l,
                                  int ph_c_h, int ph_c_l,
                                  int en);

/**
 * Initialize driver hardware
 * 
 * @param driver Pointer to driver structure
 * @return DRIVER_INIT_OK on success, DRIVER_INIT_FAILED on failure
 */
int bldc_driver_6pwm_init_hw(bldc_driver_6pwm_t *driver);

/**
 * Enable motor driver
 * 
 * @param driver Pointer to driver structure
 */
void bldc_driver_6pwm_enable(bldc_driver_6pwm_t *driver);

/**
 * Disable motor driver
 * 
 * @param driver Pointer to driver structure
 */
void bldc_driver_6pwm_disable(bldc_driver_6pwm_t *driver);

/**
 * Set phase voltages to the hardware
 * 
 * @param driver Pointer to driver structure
 * @param ua Phase A voltage
 * @param ub Phase B voltage
 * @param uc Phase C voltage
 */
void bldc_driver_6pwm_set_pwm(bldc_driver_6pwm_t *driver, 
                              float ua, float ub, float uc);

/**
 * Set phase states
 * 
 * @param driver Pointer to driver structure
 * @param sa Phase A state
 * @param sb Phase B state
 * @param sc Phase C state
 */
void bldc_driver_6pwm_set_phase_state(bldc_driver_6pwm_t *driver,
                                      phase_state_t sa,
                                      phase_state_t sb,
                                      phase_state_t sc);

#endif /* BLDC_DRIVER_6PWM_H */
