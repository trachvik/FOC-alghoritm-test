#include "bldc_motor.h"
#include "bldc_driver_6pwm.h"
#include <string.h>
#include <math.h>
#include <arm_math.h>
#include <zephyr/kernel.h>

/* Helper macros */
#define _ISSET(x) ((x) != NOT_SET)
#define _CONSTRAIN(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

/* Default configuration values */
#define DEF_POWER_SUPPLY 12.0f
#define DEF_PID_VEL_P 0.5f
#define DEF_PID_VEL_I 10.0f
#define DEF_PID_VEL_D 0.0f
#define DEF_PID_VEL_RAMP 1000.0f
#define DEF_PID_VEL_LIMIT 12.0f
#define DEF_P_ANGLE_P 20.0f
#define DEF_VEL_LIM 20.0f
#define DEF_VEL_FILTER_TF 0.005f
#define DEF_PID_CURR_P 3.0f
#define DEF_PID_CURR_I 300.0f
#define DEF_PID_CURR_D 0.0f
#define DEF_PID_CURR_RAMP 1000.0f
#define DEF_CURR_FILTER_TF 0.005f
#define DEF_MOTION_DOWNSAMPLE 0
#define DEF_MON_DOWNSAMPLE 100

/**
 * Time functions
 */
unsigned long micros(void)
{
    return k_cyc_to_us_floor64(k_cycle_get_64());
}

/*void delay_ms(unsigned long ms)
{
    k_msleep(ms);
}*/

/**
 * Sensor interface functions (implemented in as5048a.c)
 */
extern void sensor_update(sensor_t *sensor);
extern float sensor_get_angle(sensor_t *sensor);
extern float sensor_get_velocity(sensor_t *sensor); // TO DO - not implemented yet, is this needed?
extern bool sensor_needs_search(sensor_t *sensor);

/**
 * Driver interface wrapper functions
 */
void driver_set_pwm(bldc_driver_t *driver, float ua, float ub, float uc)
{
    if (driver == NULL) return;
    
    bldc_driver_6pwm_t *drv = (bldc_driver_6pwm_t*)driver;
    bldc_driver_6pwm_set_pwm(drv, ua, ub, uc);
}

void driver_enable(bldc_driver_t *driver)
{
    if (driver == NULL) return;
    
    bldc_driver_6pwm_t *drv = (bldc_driver_6pwm_t*)driver;
    bldc_driver_6pwm_enable(drv);
}

void driver_disable(bldc_driver_t *driver)
{
    if (driver == NULL) return;
    
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
 * Current sense stubs
 */
// void current_sense_enable(current_sense_t *cs)
// {
//     /* Stub - no current sensing yet */
// }

// void current_sense_disable(current_sense_t *cs)
// {
//     /* Stub - no current sensing yet */
// }

// bool current_sense_is_initialized(current_sense_t *cs)
// {
//     /* Stub - no current sensing yet */
//     return false;
// }

// int current_sense_driver_align(current_sense_t *cs, float voltage, int8_t modulation_centered)
// {
//     /* Stub - no current sensing yet */
//     return 1;
// }

/* Trapezoid maps */
/*static const int trap_120_map[6][3] = {
    {_HIGH_IMPEDANCE, 1, -1},
    {-1, 1, _HIGH_IMPEDANCE},
    {-1, _HIGH_IMPEDANCE, 1},
    {_HIGH_IMPEDANCE, -1, 1},
    {1, -1, _HIGH_IMPEDANCE},
    {1, _HIGH_IMPEDANCE, -1}
};

static const int trap_150_map[12][3] = {
    {_HIGH_IMPEDANCE, 1, -1},
    {-1, 1, -1},
    {-1, 1, _HIGH_IMPEDANCE},
    {-1, 1, 1},
    {-1, _HIGH_IMPEDANCE, 1},
    {-1, -1, 1},
    {_HIGH_IMPEDANCE, -1, 1},
    {1, -1, 1},
    {1, -1, _HIGH_IMPEDANCE},
    {1, -1, -1},
    {1, _HIGH_IMPEDANCE, -1},
    {1, 1, -1}
};*/

/**
 * PID controller initialization
 */
void pid_controller_init(pid_controller_t *pid, float p, float i, float d, float ramp, float limit)
{
    if (pid == NULL) return;
    
    pid->p = p;
    pid->i = i;
    pid->d = d;
    pid->output_ramp = ramp;
    pid->limit = limit;
    pid->error_prev = 0.0f;
    pid->output_prev = 0.0f;
    pid->integral_prev = 0.0f;
    pid->timestamp_prev = 0;
}

/**
 * PID controller operator
 */
// float pid_controller_operator(pid_controller_t *pid, float error)
// {
//     if (pid == NULL) return 0.0f;
    
//     unsigned long timestamp = micros();
//     float ts = (timestamp - pid->timestamp_prev) * 1e-6f;
    
//     if (ts <= 0.0f || ts > 0.5f) ts = 1e-3f;
    
//     /* Proportional term */
//     float proportional = pid->p * error;
    
//     /* Integral term */
//     float integral = pid->integral_prev + pid->i * ts * 0.5f * (error + pid->error_prev);
//     integral = _CONSTRAIN(integral, -pid->limit, pid->limit);
    
//     /* Derivative term */
//     float derivative = pid->d * (error - pid->error_prev) / ts;
    
//     /* Calculate output */
//     float output = proportional + integral + derivative;
//     output = _CONSTRAIN(output, -pid->limit, pid->limit);
    
//     /* Apply ramp limit */
//     if (pid->output_ramp > 0) {
//         float output_rate = (output - pid->output_prev) / ts;
//         if (output_rate > pid->output_ramp) {
//             output = pid->output_prev + pid->output_ramp * ts;
//         } else if (output_rate < -pid->output_ramp) {
//             output = pid->output_prev - pid->output_ramp * ts;
//         }
//     }
    
//     /* Save state */
//     pid->integral_prev = integral;
//     pid->output_prev = output;
//     pid->error_prev = error;
//     pid->timestamp_prev = timestamp;
    
//     return output;
// }

/**
 * PID controller reset
 */
void pid_controller_reset(pid_controller_t *pid)
{
    if (pid == NULL) return;
    
    pid->error_prev = 0.0f;
    pid->output_prev = 0.0f;
    pid->integral_prev = 0.0f;
    pid->timestamp_prev = 0;
}

/**
 * Low pass filter initialization
 */
void lowpass_filter_init(lowpass_filter_t *lpf, float tf)
{
    if (lpf == NULL) return;
    
    lpf->tf = tf;
    lpf->y_prev = 0.0f;
    lpf->timestamp_prev = 0;
}

/**
 * Low pass filter operator
 */
float lowpass_filter_operator(lowpass_filter_t *lpf, float x)
{
    if (lpf == NULL) return x;
    
    unsigned long timestamp = micros();
    float dt = (timestamp - lpf->timestamp_prev) * 1e-6f;
    
    if (dt < 0.0f || dt > 0.5f) dt = 1e-3f;
    
    float alpha = lpf->tf / (lpf->tf + dt);
    float y = alpha * lpf->y_prev + (1.0f - alpha) * x;
    
    lpf->y_prev = y;
    lpf->timestamp_prev = timestamp;
    
    return y;
}

/**
 * Normalize angle to [0, 2PI]
 */
static float normalize_angle(float angle)
{
    /* fmodf not in CMSIS DSP, keep standard implementation */
    float a = fmodf(angle, _2PI);
    return a >= 0.0f ? a : (a + _2PI);
}

/**
 * Initialize BLDC motor structure
 */
void bldc_motor_init_struct(bldc_motor_t *motor, int pp, float r, float kv, float l)
{
    if (motor == NULL) return;
    
    memset(motor, 0, sizeof(bldc_motor_t));
    
    /* Motor physical parameters */
    motor->pole_pairs = pp;
    motor->phase_resistance = r;
    motor->kv_rating = _ISSET(kv) ? kv : NOT_SET;
    motor->phase_inductance = l;
    
    /* Default configuration */
    motor->voltage_sensor_align = 6.0f;  /* Higher voltage for better alignment */
    motor->velocity_index_search = 1.0f;
    motor->voltage_limit = DEF_POWER_SUPPLY;
    motor->current_limit = NOT_SET;
    motor->velocity_limit = DEF_VEL_LIM;
    
    motor->zero_electric_angle = NOT_SET;
    motor->sensor_direction = DIR_UNKNOWN;
    motor->sensor_offset = 0.0f;
    
    /* Control configuration */
    //motor->torque_controller = VOLTAGE;
    //motor->controller = TORQUE;
    motor->foc_modulation = SINE_PWM;
    motor->modulation_centered = 1;
    
    /* Initialize controllers */
    pid_controller_init(&motor->pid_current_q, DEF_PID_CURR_P, DEF_PID_CURR_I, 
                       DEF_PID_CURR_D, DEF_PID_CURR_RAMP, DEF_POWER_SUPPLY);
    pid_controller_init(&motor->pid_current_d, DEF_PID_CURR_P, DEF_PID_CURR_I,
                       DEF_PID_CURR_D, DEF_PID_CURR_RAMP, DEF_POWER_SUPPLY);
    pid_controller_init(&motor->pid_velocity, DEF_PID_VEL_P, DEF_PID_VEL_I,
                       DEF_PID_VEL_D, DEF_PID_VEL_RAMP, DEF_PID_VEL_LIMIT);
    pid_controller_init(&motor->p_angle, DEF_P_ANGLE_P, 0.0f, 0.0f, 0.0f, DEF_VEL_LIM);
    
    lowpass_filter_init(&motor->lpf_current_q, DEF_CURR_FILTER_TF);
    lowpass_filter_init(&motor->lpf_current_d, DEF_CURR_FILTER_TF);
    lowpass_filter_init(&motor->lpf_velocity, DEF_VEL_FILTER_TF);
    lowpass_filter_init(&motor->lpf_angle, 0.0f);
    
    motor->motion_downsample = DEF_MOTION_DOWNSAMPLE;
    motor->monitor_downsample = DEF_MON_DOWNSAMPLE;
    
    motor->motor_status = MOTOR_UNINITIALIZED;
    motor->enabled = 0;
}

/**
 * Link driver to motor
 */
void bldc_motor_link_driver(bldc_motor_t *motor, bldc_driver_t *driver)
{
    if (motor == NULL) return;
    motor->driver = driver;
}

/**
 * Link sensor to motor
 */
void bldc_motor_link_sensor(bldc_motor_t *motor, sensor_t *sensor)
{
    if (motor == NULL) return;
    motor->sensor = sensor;
}

/**
 * Link current sense to motor
 */
/*void bldc_motor_link_current_sense(bldc_motor_t *motor, current_sense_t *current_sense)
{
    if (motor == NULL) return;
    motor->current_sense = current_sense;
}*/

/**
 * Calculate shaft angle
 */
float bldc_motor_shaft_angle(bldc_motor_t *motor)
{
    if (motor == NULL || motor->sensor == NULL) return 0.0f;
    
    float angle = sensor_get_angle(motor->sensor) - motor->sensor_offset;
    return (float)motor->sensor_direction * angle;
}

/**
 * Calculate shaft velocity
 */
float bldc_motor_shaft_velocity(bldc_motor_t *motor)
{
    if (motor == NULL || motor->sensor == NULL) return 0.0f;
    
    float vel = sensor_get_velocity(motor->sensor);
    return lowpass_filter_operator(&motor->lpf_velocity, 
                                   (float)motor->sensor_direction * vel);
}

/**
 * Calculate electrical angle
 */
float bldc_motor_electrical_angle(bldc_motor_t *motor)
{
    if (motor == NULL) return 0.0f;
    
    float angle = bldc_motor_shaft_angle(motor) * motor->pole_pairs - motor->zero_electric_angle;
    return normalize_angle(angle);
}

/**
 * Initialize motor hardware
 */
int bldc_motor_init(bldc_motor_t *motor)
{
    if (motor == NULL) return 0;
    
    if (motor->driver == NULL || !driver_is_initialized(motor->driver)) {
        motor->motor_status = MOTOR_INIT_FAILED;
        return 0;
    }
    
    motor->motor_status = MOTOR_INITIALIZING;
    
    /* Sanity check for voltage limit */
    float driver_voltage_limit = driver_get_voltage_limit(motor->driver);
    if (motor->voltage_limit > driver_voltage_limit) {
        motor->voltage_limit = driver_voltage_limit;
    }
    if (motor->voltage_sensor_align > motor->voltage_limit) {
        motor->voltage_sensor_align = motor->voltage_limit;
    }
    
    /* Update controller limits */
    /*if (motor->current_sense != NULL) {
        motor->pid_current_q.limit = motor->voltage_limit;
        motor->pid_current_d.limit = motor->voltage_limit;
    }*/
    
    if (_ISSET(motor->phase_resistance) /*|| motor->torque_controller != VOLTAGE*/) {
        motor->pid_velocity.limit = motor->current_limit;
    } else {
        motor->pid_velocity.limit = motor->voltage_limit;
    }
    
    motor->p_angle.limit = motor->velocity_limit;
    
    /* Set default direction for open loop if no sensor */
    /*if (motor->sensor == NULL) {
        if ((motor->controller == VELOCITY_OPENLOOP || motor->controller == ANGLE_OPENLOOP) &&
            motor->sensor_direction == DIR_UNKNOWN) {
            motor->sensor_direction = DIR_CW;
        }
    }*/
    
    k_msleep(500);  /* Short delay for stability */
    bldc_motor_enable(motor);
    k_msleep(500);
    
    motor->motor_status = MOTOR_UNCALIBRATED;
    return 1;
}

/**
 * Disable motor
 */
void bldc_motor_disable(bldc_motor_t *motor)
{
    if (motor == NULL || motor->driver == NULL) return;
    
    /*if (motor->current_sense != NULL) {
        current_sense_disable(motor->current_sense);
    }*/
    
    driver_set_pwm(motor->driver, 0.0f, 0.0f, 0.0f);
    driver_disable(motor->driver);
    
    motor->enabled = 0;
}

/**
 * Enable motor
 */
void bldc_motor_enable(bldc_motor_t *motor)
{
    if (motor == NULL || motor->driver == NULL) return;
    
    driver_enable(motor->driver);
    driver_set_pwm(motor->driver, 0.0f, 0.0f, 0.0f);
    
    /*if (motor->current_sense != NULL) {
        current_sense_enable(motor->current_sense);
    }*/
    
    /* Reset controllers */
    pid_controller_reset(&motor->pid_velocity);
    pid_controller_reset(&motor->p_angle);
    pid_controller_reset(&motor->pid_current_q);
    pid_controller_reset(&motor->pid_current_d);
    
    motor->enabled = 1;
}

/**
 * Set phase voltage using FOC (simplified version)
 */
void bldc_motor_set_phase_voltage(bldc_motor_t *motor, float uq, float ud, float angle_el)
{
    if (motor == NULL || motor->driver == NULL) return;
    
    /* Calculate sin and cos for Park transform */
    float ca = cosf(angle_el);
    float sa = sinf(angle_el);
    
    /* Inverse Park transform using ARM CMSIS DSP */
    float u_alpha, u_beta;
    arm_inv_park_f32(ud, uq, &u_alpha, &u_beta, sa, ca);
    
    motor->u_alpha = u_alpha;
    motor->u_beta = u_beta;
    
    /* Inverse Clarke transform using ARM CMSIS DSP */
    float ua, ub;
    arm_inv_clarke_f32(u_alpha, u_beta, &ua, &ub);
    motor->ua = ua;
    motor->ub = ub;
    motor->uc = -(ua + ub);  /* Ia + Ib + Ic = 0 */
    
    /* Center around voltage_limit/2 if modulation_centered */
    if (motor->modulation_centered) {
        float mid = 0.5f * motor->voltage_limit;
        motor->ua += mid;
        motor->ub += mid;
        motor->uc += mid;
    }
    
    /* Set PWM */
    driver_set_pwm(motor->driver, motor->ua, motor->ub, motor->uc);
}

/**
 * Initialize FOC algorithm with sensor alignment
 */
int bldc_motor_init_foc(bldc_motor_t *motor)
{
    if (motor == NULL) return 0;
    
    motor->motor_status = MOTOR_CALIBRATING;
    
    /* If no sensor, only openloop is possible */
    /*if (motor->sensor == NULL) {
        if (motor->controller == VELOCITY_OPENLOOP || motor->controller == ANGLE_OPENLOOP) {
            motor->motor_status = MOTOR_READY;
            return 1;
        }
        motor->motor_status = MOTOR_CALIB_FAILED;
        bldc_motor_disable(motor);
        return 0;
    }*/
    
    /* Sensor detected - perform alignment */
    sensor_update(motor->sensor);
    
    /* Check if sensor needs index search */
    if (sensor_needs_search(motor->sensor)) {
        motor->motor_status = MOTOR_CALIB_FAILED;
        bldc_motor_disable(motor);
        return 0;
    }
    
    float voltage_align = motor->voltage_sensor_align;
    
    /* Determine sensor direction if unknown */
    if (motor->sensor_direction == DIR_UNKNOWN) {
        /* Gradually ramp up voltage to avoid sudden jerk */
        for (int i = 0; i < 100; i++) {
            float voltage_ramp = voltage_align * i / 100.0f;
            bldc_motor_set_phase_voltage(motor, voltage_ramp, 0.0f, _3PI_2);
            k_msleep(5);
        }
        k_msleep(300);
        
        /* Move one electrical revolution forward - smoother with more steps */
        for (int i = 0; i <= 1000; i++) {
            float angle = _3PI_2 + _2PI * i / 1000.0f;
            bldc_motor_set_phase_voltage(motor, voltage_align, 0.0f, angle);
            sensor_update(motor->sensor);
            k_msleep(1);  /* 1ms for smoother motion */
        }
        
        /* Read angle in the middle */
        sensor_update(motor->sensor);
        float mid_angle = sensor_get_angle(motor->sensor);
        
        /* Move one electrical revolution backward */
        for (int i = 1000; i >= 0; i--) {
            float angle = _3PI_2 + _2PI * i / 1000.0f;
            bldc_motor_set_phase_voltage(motor, voltage_align, 0.0f, angle);
            sensor_update(motor->sensor);
            k_msleep(1);  /* 1ms for smoother motion */
        }
        
        /* Gradually ramp down voltage */
        for (int i = 100; i >= 0; i--) {
            float voltage_ramp = voltage_align * i / 100.0f;
            bldc_motor_set_phase_voltage(motor, voltage_ramp, 0.0f, _3PI_2);
            k_msleep(2);
        }
        
        sensor_update(motor->sensor);
        float end_angle = sensor_get_angle(motor->sensor);
        bldc_motor_set_phase_voltage(motor, 0.0f, 0.0f, 0.0f);
        k_msleep(200);
        
        /* Determine direction from movement */
        float moved = fabsf(mid_angle - end_angle);
        if (moved < MIN_ANGLE_DETECT_MOVEMENT) {
            /* Failed to detect movement */
            motor->motor_status = MOTOR_CALIB_FAILED;
            bldc_motor_disable(motor);
            return 0;
        } else if (mid_angle < end_angle) {
            motor->sensor_direction = DIR_CCW;
        } else {
            motor->sensor_direction = DIR_CW;
        }
    }
    
    /* Zero electric angle alignment */
    if (!_ISSET(motor->zero_electric_angle)) {
        /* Gradually ramp to alignment position */
        for (int i = 0; i < 100; i++) {
            float voltage_ramp = voltage_align * i / 100.0f;
            bldc_motor_set_phase_voltage(motor, voltage_ramp, 0.0f, _3PI_2);
            k_msleep(3);
        }
        k_msleep(500);
        
        /* Read sensor and get zero electric angle */
        sensor_update(motor->sensor);
        motor->zero_electric_angle = 0.0f;
        motor->zero_electric_angle = bldc_motor_electrical_angle(motor);
        
        k_msleep(20);
        
        /* Gradually ramp down voltage for smooth stop */
        for (int i = 100; i >= 0; i--) {
            float voltage_ramp = voltage_align * i / 100.0f;
            bldc_motor_set_phase_voltage(motor, voltage_ramp, 0.0f, _3PI_2);
            k_msleep(2);
        }
        
        /* Stop motor */
        bldc_motor_set_phase_voltage(motor, 0.0f, 0.0f, 0.0f);
        k_msleep(200);
    }
    
    motor->motor_status = MOTOR_READY;
    return 1;
}

/**
 * FOC loop - SimpleFOC style
 * Update sensor and electrical angle, then apply the voltages calculated in move()
 */
void bldc_motor_loop_foc(bldc_motor_t *motor)
{
    if (motor == NULL) return;
    
    /* Sensor is read (and cached) externally before calling this function.
     * sensor_update() must be called once per cycle by the caller. */
    
    /* If open-loop or disabled, do nothing */
    /*if (motor->controller == VELOCITY_OPENLOOP || 
        motor->controller == ANGLE_OPENLOOP) {
        return;
    }*/
    
    /* If disabled, do nothing */
    if (!motor->enabled) {
        return;
    }
    
    /* Calculate electrical angle from sensor */
    motor->electrical_angle = bldc_motor_electrical_angle(motor);
    
    /* Apply the phase voltages that were calculated in move() */
    /* voltage.q and voltage.d are set by move() */
    bldc_motor_set_phase_voltage(motor, motor->voltage.q, motor->voltage.d, motor->electrical_angle);
}

/**
 * Motion control loop - SimpleFOC style
 * Calculate voltages based on control mode and target
 */
void bldc_motor_move(bldc_motor_t *motor, float target)
{
    if (motor == NULL) return;
    
    /* Set target if provided */
    if (_ISSET(target)) {
        motor->target = target;
    }
    
    /* Update shaft angle and velocity for calculations */
    //if (motor->controller != ANGLE_OPENLOOP && motor->controller != VELOCITY_OPENLOOP) {
        motor->shaft_angle = bldc_motor_shaft_angle(motor);
    //}
    motor->shaft_velocity = bldc_motor_shaft_velocity(motor);
    
    /* If disabled, do nothing */
    if (!motor->enabled) return;
    
    /* Calculate back-EMF voltage if KV rating available */
    /* U_bemf = velocity / (KV * sqrt(3)) [in mechanical rad/s to electrical V] */
    #define _SQRT3 1.732050808f
    #define _RPM_TO_RADS 0.10471975512f
    if (_ISSET(motor->kv_rating)) {
        motor->voltage_bemf = motor->shaft_velocity / (motor->kv_rating * _SQRT3) / _RPM_TO_RADS;
    }
    
    /* Estimate motor current if phase resistance available and no current sense */
    /*if (motor->current_sense == NULL && _ISSET(motor->phase_resistance)) {
        motor->current.q = (motor->voltage.q - motor->voltage_bemf) / motor->phase_resistance;
    }*/
    
    /* Motion control based on controller type */
    //switch (motor->controller) {
        //case TORQUE:
            /* Torque control - voltage mode */
            if (motor->torque_controller == VOLTAGE) {
                /* Pure voltage control - target is voltage directly */
                motor->voltage.q = motor->target;
                
                /* Constrain q voltage */
                motor->voltage.q = _CONSTRAIN(motor->voltage.q, -motor->voltage_limit, motor->voltage_limit);
                
                /* Set d-axis voltage to 0 for simple voltage control */
                motor->voltage.d = 0.0f;
            } else {
                /* Current control mode */
                motor->current_sp = motor->target;
            }
        //     break;
            
        // case VELOCITY:
        //     /* Velocity control with PID */
        //     /* TODO: Implement velocity PID */
        //     break;
            
        // case ANGLE:
        //     /* Angle control with cascaded PID */
        //     /* TODO: Implement angle PID */
        //     break;
            
        // case VELOCITY_OPENLOOP:
        // case ANGLE_OPENLOOP:
        //     /* Open loop modes */
        //     /* TODO: Implement open loop control */
        //     break;
    //}
}
