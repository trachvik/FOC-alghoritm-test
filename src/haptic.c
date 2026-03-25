#include "haptic.h"
#include "drivers/as5048a.h"
#include "drivers/low_side_cs.h"
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <math.h>
/* Raw STM32F4 register access — used by low_side_cs.c for TIM1 and ADC1.
 * Kept here only to satisfy the stm32f4xx.h include needed by the inline
 * sensor / driver helpers below; all register manipulation has moved to
 * low_side_cs.c. */
#include <stm32f4xx.h>

/* Sensor abstraction - implemented in as5048a.c */
extern void sensor_update(sensor_t *sensor);
extern float sensor_get_angle(sensor_t *sensor);

/* Sensor abstraction - implemented in as5048a.c */
extern void sensor_update(sensor_t *sensor);
extern float sensor_get_angle(sensor_t *sensor);

#define NUM_STEPS 8
/* 1S Li-ion: max 4.2 V (plný náboj), nominál 3.7 V.
 * Voltage limit pro FOC PID omezovač = maximální napětí sběrnice.
 * Nastaveno na 4.2 V aby PID nepožadoval víc, než může invertor dodat. */
#define SUPPLY_VOLTAGE       4.2f
#define HAPTIC_VOLTAGE_LIMIT SUPPLY_VOLTAGE

/* PWM pin definitions for 6PWM BLDC driver */
/* TODO: Update these pin numbers based on your actual hardware */
#define PWM_AH_PIN 0       /* Phase A high-side */
#define PWM_AL_PIN 1       /* Phase A low-side */
#define PWM_BH_PIN 2       /* Phase B high-side */
#define PWM_BL_PIN 3       /* Phase B low-side */
#define PWM_CH_PIN 4       /* Phase C high-side */
#define PWM_CL_PIN 5       /* Phase C low-side */
#define ENABLE_PIN NOT_SET /* Optional enable pin */

/* Motor parameters */
#define MOTOR_POLE_PAIRS 11         /* Number of pole pairs */
#define MOTOR_PHASE_RESISTANCE 5.6f /* Ohms */
#define MOTOR_KV_RATING 320.0f      /* rpm/V */
#define MOTOR_INDUCTANCE 0.0001f    /* H */
/* Torque constant: Kt = (3/2)·p·Ke_phase  [GM3506, p=11]
 * Derived from no-load back-EMF: Kt = (3/2)·11·0.004716 = 0.0778 N·m/A_q */
#define MOTOR_KT            0.0778f  /* N·m per A of q-axis current */
/* S mid-rail biasem: meritelny rozsah ±0.458 A, haptic detent pouziva max ~0.32 A */
#define HAPTIC_TORQUE_LIMIT  0.025f  /* N·m — i_q ≈ 0.32 A, v rozahu ±0.458 A */
#define MOTOR_CURRENT_LIMIT  0.4f    /* A   — s rezervou pod max ±0.458 A */

/* AS5048A encoder from devicetree */
#define AS5048A_NODE DT_NODELABEL(as5048a)
static const struct spi_dt_spec as5048a_spi = SPI_DT_SPEC_GET(AS5048A_NODE, SPI_WORD_SET(16) | SPI_TRANSFER_MSB | SPI_MODE_CPHA, 0);
static const struct gpio_dt_spec user_button = GPIO_DT_SPEC_GET_OR(DT_ALIAS(sw0), gpios, {0});

/* ADC pro snímání proudu fází */
#define ADC_NODE DT_NODELABEL(adc1)
#define ADC_SHUNT_OHMS    0.120f  /* 120 mΩ shunt rezistor */
#define ADC_AMP_GAIN      30.0f   /* efektivni zesílení — viz zapojeni nize:
                                   * IN+ = shunt_top pres R1=1k a V_bias=1.65V pres R2=30k
                                   * Rg=1k, Rf=30k → G_eff = R2/R1 × Rf/Rg / (1+R2/R1) = 30
                                   * V_out = 30 × V_shunt + 1.65V */
/* Meritelny rozsah: ±(V_bias / G_eff / Rshunt) = ±(1.65/30/0.12) = ±0.458 A */
#define ADC_VREF_MV       3300U   /* 3.3V interní reference */
/* Sampling offset after trough: TMC6300 propagation ~500 ns + settling        */
/* 800 ns @ 100 MHz → CCR4 = 80 counts (see low_side_cs.c for derivation)     */
#define ADC_TROUGH_OFFSET_NS  800U

/* Low-side current sense instance — mirrors SimpleFOC LowsideCurrentSense     */
static low_side_cs_t g_cs;

static const struct device *adc_dev = DEVICE_DT_GET(DT_NODELABEL(adc1));

static const struct adc_channel_cfg adc_ch0_cfg = {
    .gain             = ADC_GAIN_1,
    .reference        = ADC_REF_INTERNAL,
    .acquisition_time = ADC_ACQ_TIME_DEFAULT,
    .channel_id       = 0,  /* PA0 - faze W (Ic) */
};
static const struct adc_channel_cfg adc_ch1_cfg = {
    .gain             = ADC_GAIN_1,
    .reference        = ADC_REF_INTERNAL,
    .acquisition_time = ADC_ACQ_TIME_DEFAULT,
    .channel_id       = 1,  /* PA1 - faze U+V (Ia) */
};

static void adc_init(void)
{
    /* Pin-mux via Zephyr — low_side_cs_init() also calls this internally,
     * but we keep the explicit setup here for legacy compatibility in case
     * the Zephyr device tree binding needs it before register-level config. */
    if (!device_is_ready(adc_dev)) {
        printk("[ADC] device not ready!\n");
        return;
    }
    adc_channel_setup(adc_dev, &adc_ch0_cfg);
    adc_channel_setup(adc_dev, &adc_ch1_cfg);
    /* All register-level ADC + TIM1 configuration (center-aligned, CCR4
     * trigger, dead-time, BDTR) is done inside low_side_cs_init().         */
}

/*
 * Zapojení ADC shuntů (2-shunt měření):
 *   PA0 / ADC1_IN0  = Ic  (fáze W / fáze C)
 *   PA1 / ADC1_IN1  = Ia  (fáze U) — označeno "U+V" na HW
 *   Ib = -(Ia + Ic)  dle Kirchhoffova zákona
 *
 * Veškerá logika (offset calibration, Clarke/Park, PID) je v:
 *   low_side_cs.c  — analogie SimpleFOC LowsideCurrentSense
 *   bldc_motor.c   — bldc_motor_loop_foc() (loopFOC ekvivalent)
 *
 * Postup inicializace:
 *   1. low_side_cs_init_struct(&g_cs, ...)
 *   2. bldc_driver_6pwm_init_hw(...)    ← set TIM1 running
 *   3. low_side_cs_init(&g_cs)          ← switch TIM1 to center-aligned, CCR4
 *   4. bldc_motor_link_current_sense(motor, &g_cs)
 *   5. (FOC cal, settle)
 *   6. low_side_cs_calibrate_offsets(&g_cs)
 *   7. motor->torque_controller = FOC_CURRENT
 */

/* Haptic state variables */
static float start_angle = 0.0f;
static int step_count_buffer = 0;
static int num_steps_old = NUM_STEPS;
static float last_torque = 0.0f;
static int step_count = 0;

/* FOC control loop thread - triggered by k_timer at 10 kHz */
static bldc_motor_t *g_motor_ptr = NULL;
static K_SEM_DEFINE(haptic_sem, 0, 1);
static struct k_timer haptic_timer;

#define HAPTIC_THREAD_STACK_SIZE 2048
#define HAPTIC_THREAD_PRIORITY 0 /* Highest preemptible priority */
static K_THREAD_STACK_DEFINE(haptic_stack, HAPTIC_THREAD_STACK_SIZE);
static struct k_thread haptic_thread_data;

static void haptic_timer_cb(struct k_timer *timer)
{
    ARG_UNUSED(timer);
    k_sem_give(&haptic_sem);
}

static void haptic_thread_fn(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);
    printk("haptic_thread_fn started\n");
    while (1)
    {
        k_sem_take(&haptic_sem, K_FOREVER);
        haptic_loop(g_motor_ptr);
        // Debug: vypiš stav tlačítka každých 200 ms
        static int64_t last_debug = 0;
        int64_t now_debug = k_uptime_get();
        if (now_debug - last_debug > 200)
        {
            printk("Button PB8 state: %d\n", gpio_pin_get_dt(&user_button));
            last_debug = now_debug;
        }
    }
}

int haptic_update_num_steps_from_button(void)
{
    static bool initialized = false;
    static bool last_pressed = false;
    static int current_num_steps = NUM_STEPS;
    static int64_t last_change_ms = 0;

    if (!initialized)
    {
        if (user_button.port != NULL && gpio_is_ready_dt(&user_button))
        {
            gpio_pin_configure_dt(&user_button, GPIO_INPUT | GPIO_PULL_UP);
            int init_state = gpio_pin_get_dt(&user_button);
            last_pressed = (init_state > 0);
            last_change_ms = k_uptime_get();
        }
        initialized = true;
    }

    if (user_button.port == NULL || !gpio_is_ready_dt(&user_button))
    {
        return current_num_steps;
    }

    // Správná detekce stisku pro aktivní LOW tlačítko
    bool pressed = gpio_pin_get_dt(&user_button) == 0;
    int64_t now_ms = k_uptime_get();

    if (pressed && !last_pressed && (now_ms - last_change_ms) > 180)
    {
        if (current_num_steps == 0)
        {
            current_num_steps = 20;
        }
        else if (current_num_steps <= 8)
        {
            current_num_steps = 0;
        }
        else
        {
            current_num_steps -= 4;
        }
        last_change_ms = now_ms;
    }

    last_pressed = pressed;
    return current_num_steps;
}

/* Pomocna funkce: blikne LED N-krat (bezpecne i z kontextu haptic_init) */
static void _led_blink_n(int n)
{
    const struct device *gpioc = DEVICE_DT_GET(DT_NODELABEL(gpioc));
    if (!device_is_ready(gpioc)) return;
    gpio_pin_configure(gpioc, 13, GPIO_OUTPUT);
    k_msleep(400); /* pauza pred sekvenci */
    for (int i = 0; i < n; i++) {
        gpio_pin_set(gpioc, 13, 0); /* LED ON (active low) */
        k_msleep(200);
        gpio_pin_set(gpioc, 13, 1); /* LED OFF */
        k_msleep(200);
    }
    k_msleep(600); /* pauza po sekvenci */
}

int haptic_init(bldc_motor_t *motor, bldc_driver_t *driver, sensor_t *encoder)
{
    printk("[haptic_init] entered\n");
    k_msleep(200);
    bldc_driver_6pwm_t *driver_6pwm = (bldc_driver_6pwm_t *)driver;
    struct as5048a_device *as5048a = (struct as5048a_device *)encoder;

    /* --- Step 1: AS5048A encoder ----------------------------------------- */
    _led_blink_n(1);  /* 1 blik = dosli jsme sem */
    printk("1. Initializing AS5048A encoder...\n");
    k_msleep(50);
    if (as5048a_init(as5048a, &as5048a_spi) < 0)
    {
        printk("   ERROR: Failed to initialize AS5048A: %d\n", -1);
        k_msleep(50);
        return -1;
    }
    printk("   [OK] AS5048A ready\n\n");
    k_msleep(50);

    /* --- Step 2: 6PWM driver (TIM1 low-side, TIM3 high-side) ------------- */
    _led_blink_n(2);  /* 2 bliky = AS5048A OK */
    printk("2. Initializing 6PWM driver...\n");
    k_msleep(50);
    bldc_driver_6pwm_init_struct(driver_6pwm,
                                 PWM_AH_PIN, PWM_AL_PIN,
                                 PWM_BH_PIN, PWM_BL_PIN,
                                 PWM_CH_PIN, PWM_CL_PIN,
                                 ENABLE_PIN);

    driver_6pwm->pwm_frequency        = 25000;
    driver_6pwm->voltage_power_supply = SUPPLY_VOLTAGE;
    driver_6pwm->voltage_limit        = HAPTIC_VOLTAGE_LIMIT;
    driver_6pwm->dead_zone            = 0.02f;

    if (bldc_driver_6pwm_init_hw(driver_6pwm) != DRIVER_INIT_OK)
    {
        printk("   ERROR: Failed to initialize driver\n");
        k_msleep(50);
        return -1;
    }
    printk("   [OK] Driver initialized\n\n");
    k_msleep(50);

    /* --- Step 3: Low-side current sense (mirrors SimpleFOC LowsideCS) -----
     * Must be called AFTER bldc_driver_6pwm_init_hw() so that TIM1 is
     * running before low_side_cs_init() reconfigures CR1/BDTR/CCR4.        */
    printk("3. Initializing low-side current sense...\n");
    _led_blink_n(3);  /* 3 bliky = driver OK */
    k_msleep(50);
    /* Pin-mux via Zephyr ADC driver (must happen before register access) */
    adc_init();

    low_side_cs_init_struct(&g_cs,
                            ADC_SHUNT_OHMS,
                            ADC_AMP_GAIN,
                            ADC_VREF_MV,
                            ADC_TROUGH_OFFSET_NS);
    if (!low_side_cs_init(&g_cs))
    {
        printk("   ERROR: low_side_cs_init failed\n");
        k_msleep(50);
        return -1;
    }
    printk("   [OK] Current sense ready (center-aligned TIM1, CCR4 trigger)\n\n");
    k_msleep(50);

    /* --- Step 4: BLDC Motor ----------------------------------------------- */
    printk("4. Initializing BLDC motor...\n");
    _led_blink_n(4);  /* 4 bliky = current sense OK */
    k_msleep(50);
    bldc_motor_init_struct(motor,
                           MOTOR_POLE_PAIRS,
                           MOTOR_PHASE_RESISTANCE,
                           MOTOR_KV_RATING,
                           MOTOR_INDUCTANCE);

    bldc_motor_link_driver(motor, driver);
    bldc_motor_link_sensor(motor, encoder);
    bldc_motor_link_current_sense(motor, &g_cs);   /* NEW: wire current sense */

    motor->voltage_limit  = HAPTIC_VOLTAGE_LIMIT;
    motor->velocity_limit = 20.0f;

    if (!bldc_motor_init(motor))
    {
        printk("   ERROR: Failed to initialize motor\n");
        k_msleep(50);
        return -1;
    }
    printk("   [OK] Motor initialized\n\n");
    k_msleep(50);

    /* --- Step 5: FOC sensor calibration ---------------------------------- */
    printk("5. Running FOC calibration (sensor alignment)...\n");
    _led_blink_n(5);  /* 5 bliku = motor OK */
    k_msleep(1000);

    if (!bldc_motor_init_foc(motor))
    {
        printk("   ERROR: FOC calibration failed\n");
        printk("   Motor status: %d\n", motor->motor_status);
        return -1;
    }
    printk("   [OK] FOC calibration complete!\n\n");

    /* --- Step 6: Configure FOC_CURRENT torque mode ----------------------- */
    motor->torque_controller = FOC_CURRENT;
    motor->current_limit     = MOTOR_CURRENT_LIMIT;     /* 0.8 A — bezpecny limit ADC rozsahu */

    motor->pid_current_q.p     = 0.5f;
    motor->pid_current_q.i     = 20.0f;
    motor->pid_current_q.d     = 0.0f;
    motor->pid_current_q.limit = motor->voltage_limit;

    motor->pid_current_d.p     = 0.5f;
    motor->pid_current_d.i     = 20.0f;
    motor->pid_current_d.d     = 0.0f;
    motor->pid_current_d.limit = motor->voltage_limit;

    motor->lpf_current_q.tf = 0.005f;
    motor->lpf_current_d.tf = 0.005f;

    pid_controller_reset(&motor->pid_current_q);
    pid_controller_reset(&motor->pid_current_d);
    motor->current.d = 0.0f;
    motor->current.q = 0.0f;
    printk("   [OK] FOC_CURRENT torque mode configured (Ilim=%.1f A)\n",
           motor->current_limit);

    printk("================================================\n");
    printk("  System Ready - Motor Status: %d\n", motor->motor_status);
    printk("  Entering main control loop...\n");
    printk("================================================\n\n");

    motor->target = 0.0f; /* Start with zero current */

    /* Settle motor at zero torque so ADC sees true zero before calibration */
    for (int i = 0; i < 100; i++)
    {
        bldc_motor_loop_foc(motor);
        bldc_motor_move(motor, 0.0f);
        k_msleep(1);
    }

    k_msleep(500);

    /* --- Step 7: Calibrate ADC zero-current offsets ----------------------
     * Motor is stopped and zero torque is commanded.  TIM1 TRGO is already
     * running so the injected ADC conversions are being triggered.
     * Mirrors SimpleFOC calibrateOffsets() — averages 2000 samples.        */
    printk("7. Calibrating current-sense offsets...\n");
    low_side_cs_calibrate_offsets(&g_cs);
    printk("   [OK] Offsets: Ia=%.4f A  Ic=%.4f A\n",
           (double)g_cs.offset_ia, (double)g_cs.offset_ic);

    /* Store start angle for relative position calculation */
    uint16_t startup_raw = 0;
    if (as5048a_read_raw(as5048a, &startup_raw) == 0)
    {
        start_angle = ((float)startup_raw / 16384.0f) * _2PI;
    }
    else
    {
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
    //sensor_update(motor->sensor);
    //bldc_motor_loop_foc(motor);
    //bldc_motor_move(motor, motor->voltage_limit);

        float target_torque;
        float torque_filter_alpha = 0.8f;

        //int num_steps = haptic_update_num_steps_from_button();
        int num_steps = NUM_STEPS;

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

    //     /* Smooth mode */
    //     if (num_steps == 0) {
    //         /*float damping = 0.5f;
    //         target_voltage = -damping * motor->shaft_velocity;

    //         float abs_vel = (motor->shaft_velocity < 0) ? -motor->shaft_velocity : motor->shaft_velocity;
    //         if (abs_vel < 0.1f) {
    //             target_voltage = 0.0f;
    //         } else if (abs_vel < 3.0f) {
    //             float scale = (abs_vel - 0.1f) / 2.9f;
    //             target_voltage *= scale;
    //         }

    //         target_voltage = 0.08f * target_voltage + 0.92f * last_voltage;*/
    //         // passive braking: short all three phases to low side
    //         bldc_driver_6pwm_set_phase_state((bldc_driver_6pwm_t *)motor->driver,
    //                                          PHASE_LO, PHASE_LO, PHASE_LO);
    //         bldc_driver_6pwm_set_pwm((bldc_driver_6pwm_t *)motor->driver, 0.0f, 0.0f, 0.0f);
    //         last_voltage = 0.0f;

    //     }
    //     /* Detent mode */
    //     else {
            bldc_driver_6pwm_set_phase_state((bldc_driver_6pwm_t *)motor->driver,
                                             PHASE_ON, PHASE_ON, PHASE_ON);

            /* Haptic detent: sinusový profil točivého momentu [N·m] → i_q = T / Kt */
            float norm_pos = between_steps_pos / step_size;
            target_torque = -HAPTIC_TORQUE_LIMIT * sinf(_2PI * norm_pos);

            float dist = (norm_pos - 0.5f < 0) ? -(norm_pos - 0.5f) : (norm_pos - 0.5f);

            if (dist < 0.05f) {
                target_torque = 0.0f;
            } else if (dist < 0.15f) {
                float scale = (dist - 0.05f) / 0.1f;
                target_torque *= scale;
            }

            float scaling_factor = 0.3f;
            target_torque = torque_filter_alpha * target_torque + (1.0f - torque_filter_alpha) * last_torque * scaling_factor;

            last_torque = target_torque;

            /* Prevod: tocivý moment → q-osa proud  i_q = T / Kt [A] */
            float target_current = target_torque / MOTOR_KT;
            if (target_current >  motor->current_limit) target_current =  motor->current_limit;
            if (target_current < -motor->current_limit) target_current = -motor->current_limit;

            /* low_side_cs_get_phase_currents() is called inside bldc_motor_loop_foc()
             * via motor->current_sense — no manual current_a/b assignment needed. */
            bldc_motor_move(motor, target_current);  /* set current_sp / voltage_sp */
            bldc_motor_loop_foc(motor);              /* execute FOC (reads currents internally) */

            /* Debug: print live current readings every 500 ms */
            static int64_t last_adc_dbg = 0;
            int64_t now_adc = k_uptime_get();
            if (now_adc - last_adc_dbg > 500) {
                printk("[CS] T=%.4f Nm  Iq_sp=%.3f A  Ia=%.4f A  Ib=%.4f A  Iq=%.4f A\n",
                       (double)target_torque, (double)target_current,
                       (double)motor->current_a, (double)motor->current_b,
                       (double)motor->current.q);
                last_adc_dbg = now_adc;
            }
    //    }
}