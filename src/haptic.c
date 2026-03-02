#include "haptic.h"
#include "drivers/as5048a.h"
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <math.h>
/* Raw STM32F4 register access for ADC1 injected hardware trigger via TIM1_TRGO.
 * Zephyr ADC driver is used only for GPIO pin-mux; register-level config is
 * needed for the injected-channel / external-trigger features Zephyr does not
 * yet expose through its generic ADC API. */
#include <stm32f4xx.h>

/* Sensor abstraction - implemented in as5048a.c */
extern void sensor_update(sensor_t *sensor);
extern float sensor_get_angle(sensor_t *sensor);

/* Sensor abstraction - implemented in as5048a.c */
extern void sensor_update(sensor_t *sensor);
extern float sensor_get_angle(sensor_t *sensor);

#define NUM_STEPS 8
#define SUPPLY_VOLTAGE 5.0f
 #define HAPTIC_OUTPUT_GAIN 2.0f
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

/* AS5048A encoder from devicetree */
#define AS5048A_NODE DT_NODELABEL(as5048a)
static const struct spi_dt_spec as5048a_spi = SPI_DT_SPEC_GET(AS5048A_NODE, SPI_WORD_SET(16) | SPI_TRANSFER_MSB | SPI_MODE_CPHA, 0);
static const struct gpio_dt_spec user_button = GPIO_DT_SPEC_GET_OR(DT_ALIAS(sw0), gpios, {0});

/* ADC pro snímání proudu fází */
#define ADC_NODE DT_NODELABEL(adc1)
#define ADC_RESOLUTION 12
#define ADC_VREF_MV       3300    /* 3.3V interní reference */
#define ADC_SHUNT_OHMS    0.120f  /* 120 mΩ shunt rezistor */
#define ADC_AMP_GAIN      31.0f   /* zesílení zesilovače */

/* FOC current control parameters */
/* LPF pro proudy je konfigurován přes motor->lpf_current_q/d (viz bldc_motor_init_struct) */

static const struct device *adc_dev = DEVICE_DT_GET(DT_NODELABEL(adc1));

static const struct adc_channel_cfg adc_ch0_cfg = {
    .gain             = ADC_GAIN_1,
    .reference        = ADC_REF_INTERNAL,
    .acquisition_time = ADC_ACQ_TIME_DEFAULT,
    .channel_id       = 0,  /* PA0 - faze W */
};
static const struct adc_channel_cfg adc_ch1_cfg = {
    .gain             = ADC_GAIN_1,
    .reference        = ADC_REF_INTERNAL,
    .acquisition_time = ADC_ACQ_TIME_DEFAULT,
    .channel_id       = 1,  /* PA1 - faze U+V */
};

static float phase_w_voltage  = 0.0f;
static float phase_uv_voltage = 0.0f;
static float phase_w_current  = 0.0f;  /* phase W current [A] */
static float phase_uv_current = 0.0f;  /* phase UV current [A] */

/* Zero-current ADC voltage offsets — calibrated at startup with motor stopped.
 * Mirrors SimpleFOC LowsideCurrentSense::calibrateOffsets():  the op-amp
 * output sits at ~VCC/2 (≈1.65 V) when no current flows; subtracting this
 * gives a bipolar reading centred on 0 V = 0 A. */
static float offset_w  = 0.0f;  /* zero-current voltage for phase W  [V] */
static float offset_uv = 0.0f;  /* zero-current voltage for phase UV [V] */

static void adc_init(void)
{
    /* --- Step 1: configure GPIO pin-mux via Zephyr driver --- */
    if (!device_is_ready(adc_dev)) {
        printk("[ADC] device not ready!\n");
        return;
    }
    adc_channel_setup(adc_dev, &adc_ch0_cfg);
    adc_channel_setup(adc_dev, &adc_ch1_cfg);

    /* --- Step 2: route TIM1 UPDATE event to TRGO ---
     *
     * TIM1 remains in its current edge-aligned mode (same 25 kHz frequency as
     * TIM3) so that both timers stay in sync — mismatched frequencies between
     * low-side (TIM1) and high-side (TIM3) would distort the phase voltages.
     *
     * At counter = 0 (start of each PWM period in edge-aligned mode) all three
     * TIM1 channels simultaneously assert their output HIGH, which (for
     * normal polarity) turns all low-side FETs ON through the TMC6300.  That
     * instant is the guaranteed valid window for bottom-side shunt sampling.
     *
     * MMS = 0b010 → Update event routed to TRGO → triggers ADC1 injected.
     */
    TIM1->CR2 = (TIM1->CR2 & ~TIM_CR2_MMS_Msk)
              | TIM_CR2_MMS_1;                               /* TRGO = Update event          */

    /* --- Step 3: configure ADC1 injected channels triggered by TIM1_TRGO ---
     *
     * STM32F4 "injected" channels are purpose-built for motor current sensing:
     * separate trigger (JEXTSEL/JEXTEN), sequence register (JSQR) and result
     * registers (JDR1/JDR2) — zero interference with regular conversions.
     *
     * STM32F411 RM0383 Table 77 — injected trigger JEXTSEL map (ADC1/ADC2):
     *   0b0000 = TIM1_CC4
     *   0b0001 = TIM1_TRGO  ← used here (Update event at counter = 0)
     */
    ADC1->CR2 &= ~ADC_CR2_ADON;                              /* power down to reconfigure    */

    /* Injected sequence: 2 conversions — JSQ3 first, then JSQ4       */
    /* JL = 1 → 2 conversions; JDR1 = JSQ3 = CH0 (PA0, phase W)      */
    /*                          JDR2 = JSQ4 = CH1 (PA1, phase UV)     */
    ADC1->JSQR = ((2U - 1U) << ADC_JSQR_JL_Pos)             /* JL = 1 → 2 conversions       */
               | (0U        << ADC_JSQR_JSQ3_Pos)            /* JSQ3: CH0 → PA0 → phase W    */
               | (1U        << ADC_JSQR_JSQ4_Pos);           /* JSQ4: CH1 → PA1 → phase UV   */

    /* JEXTSEL = 0b0001 (TIM1_TRGO), JEXTEN = 0b01 (rising edge)      */
    ADC1->CR2 = (ADC1->CR2 & ~(ADC_CR2_JEXTSEL_Msk | ADC_CR2_JEXTEN_Msk))
              | (1U << ADC_CR2_JEXTSEL_Pos)                  /* TIM1_TRGO                    */
              | ADC_CR2_JEXTEN_0;                            /* rising edge trigger          */

    ADC1->CR2 |= ADC_CR2_ADON;                               /* re-enable ADC                */

    printk("[ADC] HW-triggered injected init OK: "
           "CH0=PA0(phW) CH1=PA1(phUV) trigger=TIM1_TRGO edge-aligned 25kHz\n");
}

static void adc_read_phases(void)
{
    /* JEOC is set by hardware after the TIM1_TRGO-triggered injected conversion
     * sequence completes (i.e. at the bottom of every PWM period, when all
     * low-side FETs are conducting).  If the flag is not yet set the FOC loop
     * reuses the previous sample — still far better than an unsynchronised
     * software-triggered read. */
    if (!(ADC1->SR & ADC_SR_JEOC)) {
        return;  /* conversion not ready yet; keep previous values */
    }
    ADC1->SR &= ~ADC_SR_JEOC;  /* clear flag before reading result registers */

    /* JDR1 = 1st injected conversion = JSQ3 = CH0 = PA0 = phase W  */
    /* JDR2 = 2nd injected conversion = JSQ4 = CH1 = PA1 = phase UV */
    uint16_t raw_w  = (uint16_t)(ADC1->JDR1 & 0x0FFFU);
    uint16_t raw_uv = (uint16_t)(ADC1->JDR2 & 0x0FFFU);

    phase_w_voltage  = (float)raw_w  / 4095.0f * (ADC_VREF_MV / 1000.0f);
    phase_uv_voltage = (float)raw_uv / 4095.0f * (ADC_VREF_MV / 1000.0f);
    /* Subtract zero-current offset, then compute current.
     * I = (V_adc - V_offset) / (amp_gain * R_shunt)
     * Mirrors SimpleFOC: current.a = (_readADCVoltageLowSide() - offset_ia) * gain_a */
    phase_w_current  = (phase_w_voltage  - offset_w)  / (ADC_AMP_GAIN * ADC_SHUNT_OHMS);
    phase_uv_current = (phase_uv_voltage - offset_uv) / (ADC_AMP_GAIN * ADC_SHUNT_OHMS);
}

/* Calibrate zero-current ADC offsets — call once at startup with motor stopped
 * and zero voltage applied.  Mirrors SimpleFOC calibrateOffsets(): 2000 samples
 * averaged via the hardware trigger, each spaced one PWM period apart. */
#define ADC_CALIBRATION_ROUNDS 2000
static void adc_calibrate_offsets(void)
{
    float sum_w = 0.0f, sum_uv = 0.0f;
    int   valid = 0;

    printk("[ADC] Calibrating zero-current offsets (%d samples)...\n",
           ADC_CALIBRATION_ROUNDS);

    for (int i = 0; i < ADC_CALIBRATION_ROUNDS; i++) {
        /* Wait for next HW-triggered injected conversion (JEOC).
         * At 25 kHz, one PWM period = 40 µs; timeout at 200 µs. */
        for (int t = 0; t < 200; t++) {
            if (ADC1->SR & ADC_SR_JEOC) break;
            k_busy_wait(1);
        }
        if (!(ADC1->SR & ADC_SR_JEOC)) continue;  /* timed out — skip sample */

        ADC1->SR &= ~ADC_SR_JEOC;
        sum_w  += (float)(ADC1->JDR1 & 0x0FFFU) / 4095.0f * (ADC_VREF_MV / 1000.0f);
        sum_uv += (float)(ADC1->JDR2 & 0x0FFFU) / 4095.0f * (ADC_VREF_MV / 1000.0f);
        valid++;
    }

    if (valid > 0) {
        offset_w  = sum_w  / valid;
        offset_uv = sum_uv / valid;
    }
    printk("[ADC] Offsets: W=%.4f V  UV=%.4f V  (n=%d)\n",
           (double)offset_w, (double)offset_uv, valid);
}

/*
 * Zapojení ADC shuntů (2-shunt měření):
 *   PA0 (phase_w_current)  = Ic  (fáze W / fáze C)
 *   PA1 (phase_uv_current) = Ia  (fáze U) -- označeno "U+V" na HW
 *   Ib = -(Ia + Ic)  dle Kirchhoffova zákona
 *
 * Clarke + Park transformace a PID regulátory proudu jsou plně uvnitř
 * bldc_motor_loop_foc() – inspirováno SimpleFOC loopFOC().
 * Stačí nastavit motor->torque_controller = FOC_CURRENT a před každým
 * voláním bldc_motor_loop_foc() zapsat motor->current_a a motor->current_b.
 */

/* Haptic state variables */
static float start_angle = 0.0f;
static int step_count_buffer = 0;
static int num_steps_old = NUM_STEPS;
static float last_voltage = 0.0f;
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
    driver_6pwm->pwm_frequency = 25000; /* 25 kHz PWM */
    driver_6pwm->voltage_power_supply = SUPPLY_VOLTAGE;
    driver_6pwm->voltage_limit = HAPTIC_VOLTAGE_LIMIT;
    driver_6pwm->dead_zone = 0.02f; /* 2% dead time */

    if (bldc_driver_6pwm_init_hw(driver_6pwm) != DRIVER_INIT_OK)
    {
        printk("   ERROR: Failed to initialize driver\n");
        return -1;
    }
    printk("   [OK] Driver initialized\n\n");

    /* Initialize ADC AFTER the PWM driver so that TIM1 is fully configured
     * before we set TIM1->CR2 MMS=Update.  If called earlier, bldc_driver_6pwm_init_hw()
     * would reconfigure TIM1 and silently reset the TRGO routing. */
    adc_init();
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
    motor->velocity_limit = 20.0f; /* rad/s */
    // motor->voltage_sensor_align = 3.0f;

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

    /* FOC VOLTAGE mode — does not rely on current sensing.
     * Reverted from FOC_CURRENT because UV phase ADC reads 0.000V (possibly
     * disconnected or misconfigured), making the current PID destabilising.
     * Tune and re-enable FOC_CURRENT once ADC readings are verified. */
    motor->torque_controller = VOLTAGE;
    motor->current_limit     = 0.8f;

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
    printk("   [OK] VOLTAGE torque mode configured (Ilim=%.1f A)\n",
           motor->current_limit);

    printk("================================================\n");
    printk("  System Ready - Motor Status: %d\n", motor->motor_status);
    printk("  Entering main control loop...\n");
    printk("================================================\n\n");

    motor->target = 0.0f; /* Start with zero current */

    /* Settle motor at zero torque after calibration */
    for (int i = 0; i < 100; i++)
    {
        bldc_motor_loop_foc(motor);
        bldc_motor_move(motor, 0.0f);
        k_msleep(1);
    }

    k_msleep(500);

    /* Calibrate zero-current ADC offsets (motor stopped, zero torque).
     * Must happen AFTER FOC init and settle so TIM1 TRGO is running and
     * no current is flowing — mirrors SimpleFOC calibrateOffsets(). */
    adc_calibrate_offsets();

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

        /* Snímej hodnoty z ADC (faze W a U+V) */
        adc_read_phases();

        float target_voltage;
        float voltage_filter_alpha = 0.8f;

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

            float norm_pos = between_steps_pos / step_size;
            target_voltage = -motor->current_limit * 0.2f * sinf(_2PI * norm_pos);

            float dist = (norm_pos - 0.5f < 0) ? -(norm_pos - 0.5f) : (norm_pos - 0.5f);

            if (dist < 0.05f) {
                target_voltage = 0.0f;
            } else if (dist < 0.15f) {
                float scale = (dist - 0.05f) / 0.1f;
                target_voltage *= scale;
            }

            float scaling_factor = 0.3f;
            target_voltage = voltage_filter_alpha * target_voltage + (1.0f - voltage_filter_alpha) * last_voltage * scaling_factor;

            last_voltage = target_voltage;

            /* Cílový proud: target_voltage nyní odpovídá žádanému q-proudu [A] */
            float target_current = target_voltage * HAPTIC_OUTPUT_GAIN;
            if (target_current >  motor->current_limit) target_current =  motor->current_limit;
            if (target_current < -motor->current_limit) target_current = -motor->current_limit;

            /* Nastaví naměřené fázové proudy do struktury motoru.
             * bldc_motor_loop_foc() z nich provede Clarke + Park + PID interně
             * (stejně jako SimpleFOC loopFOC() s TorqueControlType::foc_current). */
            motor->current_a = phase_uv_current;                        /* Ia – fáze U */
            motor->current_b = -(phase_uv_current + phase_w_current);   /* Ib – z KCL */

            bldc_motor_move(motor, target_current);  /* set current_sp / voltage_sp */
            bldc_motor_loop_foc(motor);              /* execute FOC */

            /* Debug: print live ADC current readings every 500 ms */
            static int64_t last_adc_dbg = 0;
            int64_t now_adc = k_uptime_get();
            if (now_adc - last_adc_dbg > 500) {
                printk("[CS] W=%.4f A  UV=%.4f A  Ia=%.4f A  Ib=%.4f A\n",
                       (double)phase_w_current, (double)phase_uv_current,
                       (double)motor->current_a, (double)motor->current_b);
                last_adc_dbg = now_adc;
            }
    //    }
}