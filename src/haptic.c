#include "haptic.h"
#include "drivers/as5048a.h"
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>

#define NUM_STEPS 8
#define SUPPLY_VOLTAGE 5.0f
#define HAPTIC_OUTPUT_GAIN 1.55f
#define HAPTIC_VOLTAGE_LIMIT 3.0f

/* PWM pin definitions for 6PWM BLDC driver */
/* TODO: Update these pin numbers based on your actual hardware */
#define PWM_AH_PIN  0   /* Phase A high-side */
#define PWM_AL_PIN  1   /* Phase A low-side */
#define PWM_BH_PIN  2   /* Phase B high-side */
#define PWM_BL_PIN  3   /* Phase B low-side */
#define PWM_CH_PIN  4   /* Phase C high-side */
#define PWM_CL_PIN  5   /* Phase C low-side */
#define ENABLE_PIN  NOT_SET  /* Optional enable pin */

/* Motor parameters */
#define MOTOR_POLE_PAIRS 11     /* Number of pole pairs */
#define MOTOR_PHASE_RESISTANCE 5.6f  /* Ohms */
#define MOTOR_KV_RATING 320.0f  /* rpm/V */
#define MOTOR_INDUCTANCE 0.0001f /* H */

/* AS5048A encoder from devicetree */
#define AS5048A_NODE DT_NODELABEL(as5048a)
static const struct spi_dt_spec as5048a_spi = SPI_DT_SPEC_GET(AS5048A_NODE, SPI_WORD_SET(16) | SPI_TRANSFER_MSB | SPI_MODE_CPHA, 0);

/* Haptic state variables */
static float start_angle = 0.0f;
static int step_count_buffer = 0;
static int num_steps_old = NUM_STEPS;
static float last_voltage = 0.0f;


int haptic_init(bldc_motor_t *motor, bldc_driver_t *driver, sensor_t *encoder)
{
    bldc_driver_6pwm_t *driver_6pwm = (bldc_driver_6pwm_t *)driver;
    struct as5048a_device *as5048a = (struct as5048a_device *)encoder;

    /* Initialize AS5048A encoder */
    printk("1. Initializing AS5048A encoder...\n");
    if (as5048a_init(as5048a, &as5048a_spi) < 0)
    {
        printk("   ERROR: Failed to initialize AS5048A: %d\n");
        return -1;
    }
    printk("   [OK] AS5048A ready\n\n");

    /* Initialize BLDC 6PWM Driver */
    printk("2. Initializing 6PWM driver...\n");
    bldc_driver_6pwm_init_struct(driver_6pwm,
                                 PWM_AH_PIN, PWM_AL_PIN,
                                 PWM_BH_PIN, PWM_BL_PIN,
                                 PWM_CH_PIN, PWM_CL_PIN,
                                 ENABLE_PIN);

    /* Configure driver parameters */
    driver_6pwm->pwm_frequency = 25000;  /* 25 kHz PWM */
    driver_6pwm->voltage_power_supply = SUPPLY_VOLTAGE;
    driver_6pwm->voltage_limit = HAPTIC_VOLTAGE_LIMIT;
    driver_6pwm->dead_zone = 0.02f;  /* 2% dead time */

    if (bldc_driver_6pwm_init_hw(driver_6pwm) != DRIVER_INIT_OK)
    {
        printk("   ERROR: Failed to initialize driver\n");
        return -1;
    }
    printk("   [OK] Driver initialized\n\n");

    /* Initialize BLDC Motor */
    printk("3. Initializing BLDC motor...\n");
    bldc_motor_init_struct(motor,
                           MOTOR_POLE_PAIRS,
                           MOTOR_PHASE_RESISTANCE,
                           MOTOR_KV_RATING,
                           MOTOR_INDUCTANCE);

    /* Link driver to motor */
    bldc_motor_link_driver(motor, driver);

    /* Link sensor to motor - use dummy pointer, actual access via g_as5048a */
    bldc_motor_link_sensor(motor, encoder);

    /* Configure motor parameters */
    motor->voltage_limit = HAPTIC_VOLTAGE_LIMIT;
    motor->velocity_limit = 20.0f;  /* rad/s */
    //motor->voltage_sensor_align = 3.0f;

    if (!bldc_motor_init(motor))
    {
        printk("   ERROR: Failed to initialize motor\n");
        return -1;
    }
    printk("   [OK] Motor initialized\n\n");

    /* Run FOC calibration */
    printk("4. Running FOC calibration...\n");
    printk("   This will align sensor and motor phases\n");
    printk("   Motor will move slightly during calibration\n");
    k_msleep(1000);

    if (!bldc_motor_init_foc(motor))
    {
        printk("   ERROR: FOC calibration failed\n");
        printk("   Motor status: %d\n", motor->motor_status);
        return -1;
    }
    printk("   [OK] FOC calibration complete!\n\n");

    printk("================================================\n");
    printk("  System Ready - Motor Status: %d\n", motor->motor_status);
    printk("  Entering main control loop...\n");
    printk("================================================\n\n");

    /* Set motor to torque control mode (voltage) */
    motor->controller = TORQUE;
    motor->target = 0.0f;  /* Start with zero torque */

    /* Settle motor at zero torque after calibration */
    for (int i = 0; i < 100; i++) {
        bldc_motor_loop_foc(motor);
        bldc_motor_move(motor, 0.0f);
        k_msleep(1);
    }

    k_msleep(500);
    
    /* Store start angle for relative position calculation */
    start_angle = sensor_get_angle(encoder);
    step_count_buffer = 0;
    num_steps_old = NUM_STEPS;

    printk("Starting motor control in 2 seconds...\n");
    k_msleep(2000);
    return 0;
}

void haptic_loop(bldc_motor_t *motor, sensor_t *encoder)
{
    struct as5048a_device *as5048a = (struct as5048a_device *)encoder;
    uint16_t raw_angle;
    float target_voltage;
    float voltage_filter_alpha = 0.8f;
    //static uint64_t last_print = 0;
    
    //uint64_t now = k_uptime_get();
    int num_steps = NUM_STEPS;
    
    /* Read encoder */
    if (as5048a_read_raw(as5048a, &raw_angle) != 0) {
        return;
    }
    
    /* Convert raw angle to radians (0 to 2π) */
    float current_angle = ((float)raw_angle / 16384.0f) * _2PI;
    
    /* UPDATE SENSOR and RUN FOC FIRST, then MOVE */
    bldc_motor_loop_foc(motor);
    
    /* Handle num_steps change with step persistence */
    if (num_steps != num_steps_old) {
        step_count_buffer = (int)((num_steps_old > 0) ? ((float)num_steps_old / _2PI) * (current_angle - start_angle) : 0);
        start_angle = current_angle;
        num_steps_old = num_steps;
    }
    
    /* Calculate relative angle */
    float angle_rel = current_angle - start_angle;
    
    /* Normalize angle to [0, 2π] */
    while (angle_rel > _2PI) angle_rel -= _2PI;
    while (angle_rel < 0) angle_rel += _2PI;
    
    /* Calculate step position */
    float step_size = (num_steps > 0) ? (_2PI / (float)num_steps) : _2PI;
    int step_count_abs = (num_steps > 0) ? ((float)num_steps / _2PI) * angle_rel : 0;
    float between_steps_pos = angle_rel - step_count_abs * step_size + step_size / 2;
    
    /* Debug print */
    /*if (now - last_print > 500) {
        printk("Angle: %.1f°, Vel: %.2f rad/s\n", 
               angle_rel * 180.0f / 3.14159f, motor->shaft_velocity);
        last_print = now;
    }*/
    
    /* Smooth mode */
    if (num_steps == 0) {
        float damping = 0.3f;
        target_voltage = -damping * motor->shaft_velocity;
        
        float abs_vel = (motor->shaft_velocity < 0) ? -motor->shaft_velocity : motor->shaft_velocity;
        if (abs_vel < 0.5f) {
            target_voltage = 0.0f;
        } else if (abs_vel < 3.0f) {
            float scale = (abs_vel - 0.5f) / 2.5f;
            target_voltage *= scale;
        }
        
        target_voltage = 0.08f * target_voltage + 0.92f * last_voltage;
    }
    /* Detent mode */
    else {
        float norm_pos = between_steps_pos / step_size;
        target_voltage = -motor->voltage_limit * 0.2f * sinf(_2PI * norm_pos);
        
        float dist = (norm_pos - 0.5f < 0) ? -(norm_pos - 0.5f) : (norm_pos - 0.5f);
        
        if (dist < 0.08f) {
            target_voltage = 0.0f;
        } else if (dist < 0.15f) {
            float scale = (dist - 0.08f) / 0.07f;
            target_voltage *= scale;
        }
        
        target_voltage = voltage_filter_alpha * target_voltage + (1.0f - voltage_filter_alpha) * last_voltage;
    }
    
    last_voltage = target_voltage;

    float commanded_voltage = target_voltage * HAPTIC_OUTPUT_GAIN;
    if (commanded_voltage > motor->voltage_limit) commanded_voltage = motor->voltage_limit;
    if (commanded_voltage < -motor->voltage_limit) commanded_voltage = -motor->voltage_limit;
    
    /* SEND VOLTAGE AFTER loopFOC */
    bldc_motor_move(motor, commanded_voltage);
}