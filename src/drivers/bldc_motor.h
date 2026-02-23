#ifndef BLDC_MOTOR_H
#define BLDC_MOTOR_H

#include <stdint.h>
#include <stdbool.h>
#include <arm_math.h>

/* Forward declarations */
typedef struct bldc_driver bldc_driver_t;
typedef struct sensor sensor_t;
//typedef struct current_sense current_sense_t;

/* Constants */
#define NOT_SET -12345.0f
#define _HIGH_IMPEDANCE 0

/* Math constants - derived from ARM CMSIS DSP PI constant */
/* All values computed from ARM PI for consistency and optimization */
#define _PI                PI             /* ARM CMSIS DSP PI = 3.14159265358979f */
#define _PI_2              (PI / 2.0f)    /* PI/2 */
#define _PI_3              (PI / 3.0f)    /* PI/3 */
#define _2PI               (2.0f * PI)    /* 2*PI */
#define _3PI_2             (3.0f * PI / 2.0f)  /* 3*PI/2 */
#define MIN_ANGLE_DETECT_MOVEMENT (_2PI/101.0f)

/* Direction enum */
typedef enum {
    DIR_UNKNOWN = 0,
    DIR_CW = 1,
    DIR_CCW = -1
} direction_t;

/* Motion control type */
/*typedef enum {
    TORQUE = 0x00,
    VELOCITY = 0x01,
    ANGLE = 0x02,
    VELOCITY_OPENLOOP = 0x03,
    ANGLE_OPENLOOP = 0x04
} motion_control_type_t;*/

/* Torque control type */ // TO DO - simplify
typedef enum {
    VOLTAGE = 0x00,
    DC_CURRENT = 0x01,
    FOC_CURRENT = 0x02
} torque_control_type_t;

/* FOC modulation type */ // TO DO - simplify
typedef enum {
    SINE_PWM = 0x00,
    SPACE_VECTOR_PWM = 0x01,
    TRAPEZOID_120 = 0x02,
    TRAPEZOID_150 = 0x03
} foc_modulation_type_t;

/* FOC motor status */
typedef enum {
    MOTOR_UNINITIALIZED = 0x00,
    MOTOR_INITIALIZING = 0x01,
    MOTOR_UNCALIBRATED = 0x02,
    MOTOR_CALIBRATING = 0x03,
    MOTOR_READY = 0x04,
    MOTOR_ERROR = 0x08,
    MOTOR_CALIB_FAILED = 0x0E,
    MOTOR_INIT_FAILED = 0x0F
} foc_motor_status_t;

/* DQ voltage structure */
typedef struct {
    float d;
    float q;
} dq_voltage_t;

/* DQ current structure */ // TO DO - do I need this?
typedef struct {
    float d;
    float q;
} dq_current_t;

/* PID controller structure */
typedef struct {
    float p;              /* Proportional gain */
    float i;              /* Integral gain */
    float d;              /* Derivative gain */
    float output_ramp;    /* Maximum speed of change of output */
    float limit;          /* Maximum output value */
    
    float error_prev;     /* Last tracking error */
    float output_prev;    /* Last PID output */
    float integral_prev;  /* Last integral component */
    unsigned long timestamp_prev; /* Last execution timestamp */
} pid_controller_t;

/* Low pass filter structure */
typedef struct {
    float tf;             /* Time constant */
    float y_prev;         /* Previous filtered value */
    unsigned long timestamp_prev; /* Last execution timestamp */
} lowpass_filter_t;

/**
 * BLDC Motor structure
 * 
 * Complete structure for controlling a BLDC motor using FOC algorithm
 */
typedef struct {
    /* Motor physical parameters */
    int pole_pairs;           /* Number of pole pairs */
    float phase_resistance;   /* Motor phase resistance [Ohm] */
    float kv_rating;          /* Motor KV rating (rpm/V) */
    float phase_inductance;   /* Motor phase inductance [H] */
    
    /* State variables */
    float target;             /* Current target value */
    float feed_forward_velocity; /* Feed forward velocity */
    float shaft_angle;        /* Current motor angle [rad] */
    float electrical_angle;   /* Current electrical angle [rad] */
    float shaft_velocity;     /* Current motor velocity [rad/s] */
    float current_sp;         /* Target current (q current) */
    float shaft_velocity_sp;  /* Current target velocity */
    float shaft_angle_sp;     /* Current target angle */
    
    /* Voltage and current */
    dq_voltage_t voltage;     /* Current d and q voltage */
    dq_current_t current;     /* Current d and q current */
    float voltage_bemf;       /* Estimated back-EMF voltage */
    float u_alpha, u_beta;    /* Phase voltages alpha and beta */
    float ua, ub, uc;         /* Current phase voltages */
    
    /* Configuration parameters */
    float voltage_sensor_align; /* Sensor alignment voltage */
    float velocity_index_search; /* Velocity for index search */
    
    /* Limiting variables */
    float voltage_limit;      /* Global voltage limit */
    float current_limit;      /* Global current limit */
    float velocity_limit;     /* Global velocity limit */
    
    /* Motor status */
    int8_t enabled;           /* Motor enabled/disabled flag */
    foc_motor_status_t motor_status; /* Current motor status */
    
    /* PWM modulation */
    foc_modulation_type_t foc_modulation; /* Modulation algorithm */
    int8_t modulation_centered; /* Centered modulation flag */
    
    /* Control configuration */
    torque_control_type_t torque_controller; // TO DO keep voltage AND current control for now, but maybe simplify later
    //motion_control_type_t controller;
    
    /* Controllers and filters */
    pid_controller_t pid_current_q;
    pid_controller_t pid_current_d;
    lowpass_filter_t lpf_current_q;
    lowpass_filter_t lpf_current_d;
    pid_controller_t pid_velocity;
    pid_controller_t p_angle;
    lowpass_filter_t lpf_velocity;
    lowpass_filter_t lpf_angle;
    
    unsigned int motion_downsample;
    unsigned int motion_cnt;
    
    /* Sensor related */
    float sensor_offset;      /* User defined sensor zero offset */
    float zero_electric_angle; /* Absolute zero electric angle */
    direction_t sensor_direction; /* Sensor direction */
    bool pp_check_result;     /* Pole pair check result */
    
    /* Monitoring */
    unsigned int monitor_downsample;
    unsigned int monitor_cnt;
    uint8_t monitor_variables;
    
    /* Hardware links */
    bldc_driver_t *driver;
    sensor_t *sensor;
    //current_sense_t *current_sense;
    
    /* Open loop variables */
    //long open_loop_timestamp;
    
} bldc_motor_t;

/**
 * Initialize BLDC motor structure
 * 
 * @param motor Pointer to motor structure
 * @param pp Pole pairs number
 * @param r Phase resistance [Ohm]
 * @param kv KV rating (rpm/V)
 * @param l Phase inductance [H]
 */
void bldc_motor_init_struct(bldc_motor_t *motor, int pp, float r, float kv, float l);

/**
 * Link driver to motor
 * 
 * @param motor Pointer to motor structure
 * @param driver Pointer to driver structure
 */
void bldc_motor_link_driver(bldc_motor_t *motor, bldc_driver_t *driver);

/**
 * Link sensor to motor
 * 
 * @param motor Pointer to motor structure
 * @param sensor Pointer to sensor structure
 */
void bldc_motor_link_sensor(bldc_motor_t *motor, sensor_t *sensor);

/**
 * Link current sense to motor
 * 
 * @param motor Pointer to motor structure
 * @param current_sense Pointer to current sense structure
 */
//void bldc_motor_link_current_sense(bldc_motor_t *motor, current_sense_t *current_sense);

/**
 * Initialize motor hardware
 * 
 * @param motor Pointer to motor structure
 * @return 1 on success, 0 on failure
 */
int bldc_motor_init(bldc_motor_t *motor);

/**
 * Disable motor
 * 
 * @param motor Pointer to motor structure
 */
void bldc_motor_disable(bldc_motor_t *motor);

/**
 * Enable motor
 * 
 * @param motor Pointer to motor structure
 */
void bldc_motor_enable(bldc_motor_t *motor);

/**
 * Initialize FOC algorithm
 * 
 * @param motor Pointer to motor structure
 * @return 1 on success, 0 on failure
 */
int bldc_motor_init_foc(bldc_motor_t *motor);

/**
 * FOC loop - run as fast as possible
 * 
 * @param motor Pointer to motor structure
 */
void bldc_motor_loop_foc(bldc_motor_t *motor);

/**
 * Motion control loop
 * 
 * @param motor Pointer to motor structure
 * @param target Target value (or NOT_SET to use motor->target)
 */
void bldc_motor_move(bldc_motor_t *motor, float target);

/**
 * Set phase voltage using FOC
 * 
 * @param motor Pointer to motor structure
 * @param uq Voltage in q axis
 * @param ud Voltage in d axis
 * @param angle_el Electrical angle
 */
void bldc_motor_set_phase_voltage(bldc_motor_t *motor, float uq, float ud, float angle_el);

/**
 * Calculate shaft angle
 * 
 * @param motor Pointer to motor structure
 * @return Shaft angle in radians
 */
float bldc_motor_shaft_angle(bldc_motor_t *motor);

/**
 * Calculate shaft velocity
 * 
 * @param motor Pointer to motor structure
 * @return Shaft velocity in rad/s
 */
//float bldc_motor_shaft_velocity(bldc_motor_t *motor); // TO DO - not im

/**
 * Calculate electrical angle
 * 
 * @param motor Pointer to motor structure
 * @return Electrical angle in radians
 */
float bldc_motor_electrical_angle(bldc_motor_t *motor);

/* PID controller functions */
void pid_controller_init(pid_controller_t *pid, float p, float i, float d, float ramp, float limit);
float pid_controller_operator(pid_controller_t *pid, float error);
void pid_controller_reset(pid_controller_t *pid);   // TO DO is this needed?

/* Low pass filter functions */
void lowpass_filter_init(lowpass_filter_t *lpf, float tf);
//float lowpass_filter_operator(lowpass_filter_t *lpf, float x);  // TO DO - do I need this?

#endif /* BLDC_MOTOR_H */
