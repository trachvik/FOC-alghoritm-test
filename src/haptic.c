#include "haptic.h"
#include "drivers/as5048a.h"
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <math.h>
#include <arm_math.h>

/* Sensor abstraction - implemented in as5048a.c */
extern void sensor_update(sensor_t *sensor);
extern float sensor_get_angle(sensor_t *sensor);

/* Sensor abstraction - implemented in as5048a.c */
extern void sensor_update(sensor_t *sensor);
extern float sensor_get_angle(sensor_t *sensor);

#define NUM_STEPS 0
#define SUPPLY_VOLTAGE 5.0f
//#define HAPTIC_OUTPUT_GAIN 2.0f
#define HAPTIC_VOLTAGE_LIMIT SUPPLY_VOLTAGE

/* Smooth-mode hybrid resistance tuning */
#define SMOOTH_RESIST_LEVEL              0.55f
#define SMOOTH_ACTIVE_VOLT_MAX           0.90f
#define SMOOTH_ACTIVE_VEL_DEADBAND       0.08f
#define SMOOTH_ACTIVE_VEL_RANGE          5.0f
#define SMOOTH_BLEND_VEL_LOW             2.0f
#define SMOOTH_BLEND_VEL_HIGH            12.0f
#define SMOOTH_PASSIVE_FULL_EFFECT_VEL   18.0f
#define SMOOTH_PASSIVE_DUTY_MIN          0.35f
#define SMOOTH_PASSIVE_DUTY_MAX          0.95f
#define SMOOTH_ACTIVE_VOLT_MIN           0.12f
#define SMOOTH_ACTIVE_COULOMB_VOLT       0.22f
#define SMOOTH_ACTIVE_VISCOUS_GAIN       0.08f
#define SMOOTH_DESIRED_DRAG_PROXY        1.9f
#define SMOOTH_PASSIVE_VEL_FLOOR         0.35f
#define SMOOTH_ACTIVE_DEFICIT_GAIN       0.22f

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
static const struct gpio_dt_spec user_button = GPIO_DT_SPEC_GET_OR(DT_ALIAS(sw0), gpios, {0});

/* Haptic state variables */
static float start_angle = 0.0f;
static int step_count_buffer = 0;
static int num_steps_old = NUM_STEPS;
static float last_voltage = 0.0f;
static int step_count = 0;
static float smooth_passive_duty_lp = 0.0f;

static float clampf_local(float x, float min_val, float max_val)
{
    if (x < min_val) return min_val;
    if (x > max_val) return max_val;
    return x;
}

/* FOC control loop thread - triggered by k_timer at 10 kHz */
static bldc_motor_t *g_motor_ptr = NULL;
static K_SEM_DEFINE(haptic_sem, 0, 1);
static struct k_timer haptic_timer;

#define HAPTIC_THREAD_STACK_SIZE 2048
#define HAPTIC_THREAD_PRIORITY   0       /* Highest preemptible priority */
static K_THREAD_STACK_DEFINE(haptic_stack, HAPTIC_THREAD_STACK_SIZE);
static struct k_thread haptic_thread_data;

static void haptic_timer_cb(struct k_timer *timer)
{
    ARG_UNUSED(timer);
    k_sem_give(&haptic_sem);
}

static void haptic_thread_fn(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);
    while (1) {
        k_sem_take(&haptic_sem, K_FOREVER);
        haptic_loop(g_motor_ptr);
    }
}

int haptic_update_num_steps_from_button(void)
{
    static bool initialized = false;
    static bool last_pressed = false;
    static int current_num_steps = NUM_STEPS;
    static int64_t last_change_ms = 0;

    if (!initialized) {
        if (user_button.port != NULL && gpio_is_ready_dt(&user_button)) {
            gpio_pin_configure_dt(&user_button, GPIO_INPUT);
            int init_state = gpio_pin_get_dt(&user_button);
            last_pressed = (init_state > 0);
            last_change_ms = k_uptime_get();
        }
        initialized = true;
    }

    if (user_button.port == NULL || !gpio_is_ready_dt(&user_button)) {
        return current_num_steps;
    }

    bool pressed = gpio_pin_get_dt(&user_button) > 0;
    int64_t now_ms = k_uptime_get();

    if (pressed && !last_pressed && (now_ms - last_change_ms) > 180) {
        if (current_num_steps == 0) {
            current_num_steps = 20;
        } else if (current_num_steps <= 8) {
            current_num_steps = 0;
        } else {
            current_num_steps -= 4;
        }
        last_change_ms = now_ms;
    }

    last_pressed = pressed;
    return current_num_steps;
}


int haptic_init(bldc_motor_t *motor, bldc_driver_t *driver, sensor_t *encoder)
{
    bldc_driver_6pwm_t *driver_6pwm = (bldc_driver_6pwm_t *)driver;
    struct as5048a_device *as5048a = (struct as5048a_device *)encoder;

    /* Initialize AS5048A encoder */
    printk("1. Initializing AS5048A encoder...\n");
    if (as5048a_init(as5048a, &as5048a_spi) < 0)
    {
        printk("   ERROR: Failed to initialize AS5048A: %d\n", -1);
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
    //motor->controller = TORQUE;
    motor->target = 0.0f;  /* Start with zero torque */

    /* Settle motor at zero torque after calibration */
    for (int i = 0; i < 100; i++) {
        bldc_motor_loop_foc(motor);
        bldc_motor_move(motor, 0.0f);
        k_msleep(1);
    }

    k_msleep(500);
    
    /* Store start angle for relative position calculation */
    uint16_t startup_raw = 0;
    if (as5048a_read_raw(as5048a, &startup_raw) == 0) {
        start_angle = ((float)startup_raw / 16384.0f) * _2PI;
    } else {
        start_angle = 0.0f;
    }
    step_count_buffer = 0;
    num_steps_old = NUM_STEPS;


    printk("Starting motor control in 2 seconds...\n");
    k_msleep(2000);

    /* Start 10 kHz (100 µs) FOC control loop */
    g_motor_ptr = motor;
    k_timer_init(&haptic_timer, haptic_timer_cb, NULL);
    k_thread_create(&haptic_thread_data, haptic_stack,
                    K_THREAD_STACK_SIZEOF(haptic_stack),
                    haptic_thread_fn, NULL, NULL, NULL,
                    HAPTIC_THREAD_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&haptic_thread_data, "haptic_foc");
    k_timer_start(&haptic_timer, K_USEC(100), K_USEC(100));

    return 0;
}

void haptic_loop(bldc_motor_t *motor)
{
    float target_voltage = 0.0f;
    float voltage_filter_alpha = 0.8f;

    int num_steps = haptic_update_num_steps_from_button();
    
    /* Read encoder via sensor abstraction linked in motor struct */
    sensor_update(motor->sensor);
    float current_angle = sensor_get_angle(motor->sensor);
    
    /* Calculate relative angle */
    float angle_rel = current_angle - start_angle;

    // Preserve position when num_steps is changed
    if (num_steps != num_steps_old) {
        step_count_buffer = step_count;
        start_angle = current_angle;
        num_steps_old = num_steps;
        angle_rel = 0.0f;
    }
    
    /* Normalize angle to [0, 2π] */
    while (angle_rel > _2PI) angle_rel -= _2PI;
    while (angle_rel < 0) angle_rel += _2PI;
    
    /* Calculate step position */
    float step_size = (num_steps > 0) ? (_2PI / (float)num_steps) : _2PI;
    float step_count_f = (num_steps > 0) ? ((float)num_steps / _2PI) * angle_rel : 0.0f;
    step_count = (int)roundf(step_count_f) + step_count_buffer;
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
        float vel = motor->shaft_velocity;
        float abs_vel = (vel < 0.0f) ? -vel : vel;

        /* Constant-resistance passive target: duty * omega ~= const */
        float passive_duty = (SMOOTH_RESIST_LEVEL * SMOOTH_DESIRED_DRAG_PROXY) /
                             ((abs_vel > SMOOTH_PASSIVE_VEL_FLOOR) ? abs_vel : SMOOTH_PASSIVE_VEL_FLOOR);
        passive_duty = clampf_local(passive_duty, SMOOTH_PASSIVE_DUTY_MIN, SMOOTH_PASSIVE_DUTY_MAX);

        /* Smooth duty transitions to reduce buzz during speed changes */
        smooth_passive_duty_lp = 0.92f * smooth_passive_duty_lp + 0.08f * passive_duty;
        float passive_duty_cmd = clampf_local(smooth_passive_duty_lp, SMOOTH_PASSIVE_DUTY_MIN, SMOOTH_PASSIVE_DUTY_MAX);

        /* Active low-speed compensation fills only missing drag at low speed */
        float active_voltage = 0.0f;
        if (abs_vel > SMOOTH_ACTIVE_VEL_DEADBAND && abs_vel < SMOOTH_ACTIVE_VEL_RANGE) {
            float sign = (vel > 0.0f) ? -1.0f : 1.0f; /* oppose motion */
            float desired_drag = SMOOTH_RESIST_LEVEL * SMOOTH_DESIRED_DRAG_PROXY;
            float passive_drag = passive_duty_cmd * abs_vel;
            float drag_deficit = desired_drag - passive_drag;
            if (drag_deficit < 0.0f) {
                drag_deficit = 0.0f;
            }

            float low_speed_weight = 1.0f - clampf_local(abs_vel / SMOOTH_ACTIVE_VEL_RANGE, 0.0f, 1.0f);
            float active_mag = (SMOOTH_ACTIVE_COULOMB_VOLT +
                                SMOOTH_ACTIVE_VISCOUS_GAIN * abs_vel +
                                SMOOTH_ACTIVE_DEFICIT_GAIN * drag_deficit) * low_speed_weight;
            active_mag = clampf_local(active_mag, 0.0f, SMOOTH_ACTIVE_VOLT_MAX);

            if (active_mag > 0.0f && active_mag < SMOOTH_ACTIVE_VOLT_MIN) {
                active_mag = SMOOTH_ACTIVE_VOLT_MIN;
            }
            active_voltage = sign * active_mag;
        }

        /* Continuous passive braking (quiet): keep PHASE_LO and modulate low-side duty.
         * In this driver implementation, PHASE_LO low-side on-time is (1 - Ua/Vs). */
        bldc_driver_6pwm_set_phase_state((bldc_driver_6pwm_t *)motor->driver,
                                         PHASE_LO, PHASE_LO, PHASE_LO);
        float passive_pwm_voltage = (1.0f - passive_duty_cmd) * motor->voltage_limit;
        passive_pwm_voltage = clampf_local(passive_pwm_voltage, 0.0f, motor->voltage_limit);
        bldc_driver_6pwm_set_pwm((bldc_driver_6pwm_t *)motor->driver,
                                 passive_pwm_voltage, passive_pwm_voltage, passive_pwm_voltage);

        /* Very low-speed compensation uses active control to keep resistance constant. */
        if (active_voltage > 0.001f || active_voltage < -0.001f) {
            bldc_driver_6pwm_set_phase_state((bldc_driver_6pwm_t *)motor->driver,
                                             PHASE_ON, PHASE_ON, PHASE_ON);
            bldc_motor_loop_foc(motor);
            bldc_motor_move(motor, active_voltage);
            last_voltage = active_voltage;
            return;
        }

        last_voltage = 0.0f;
        return;


    }
    /* Detent mode */
    else {
        bldc_motor_loop_foc(motor);
        bldc_driver_6pwm_set_phase_state((bldc_driver_6pwm_t *)motor->driver,
                                         PHASE_ON, PHASE_ON, PHASE_ON);
        float norm_pos = between_steps_pos / step_size;
        target_voltage = -motor->voltage_limit * 0.2f * sinf(_2PI * norm_pos); // does not work well with arm_sin_f32 for some reason, maybe due to precision issues, so using math.h sinf instead
        
        float dist = (norm_pos - 0.5f < 0) ? -(norm_pos - 0.5f) : (norm_pos - 0.5f);
        
        if (dist < 0.05f) {
            target_voltage = 0.0f;
        } else if (dist < 0.15f) {
            float scale = (dist - 0.05f) / 0.1f;
            target_voltage *= scale;
        }
        
        float scaling_factor = 0.3f; // for smoother detents and less noise
        target_voltage = voltage_filter_alpha * target_voltage + (1.0f - voltage_filter_alpha) * last_voltage * scaling_factor;
    }
    
    
    last_voltage = target_voltage;

   // float commanded_voltage = target_voltage * HAPTIC_OUTPUT_GAIN;
    //if (commanded_voltage > motor->voltage_limit) commanded_voltage = motor->voltage_limit;
    //if (commanded_voltage < -motor->voltage_limit) commanded_voltage = -motor->voltage_limit;
    
    /* SEND VOLTAGE AFTER loopFOC */
    bldc_motor_move(motor, target_voltage);
    
}