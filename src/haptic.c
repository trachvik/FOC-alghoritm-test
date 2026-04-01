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
/* Smooth mode: dampovaci koeficient [A·s/rad]. */
#define SMOOTH_KD            0.0025f
/* Detent mode: peak current of restoring spring [A].
 * Lower than current_limit to leave headroom for damping current.
 * Too high → strong vibration (underdamped spring).  0.20 A is a good start. */
#define HAPTIC_DETENT_AMPLITUDE  0.20f
/* Detent mode: velocity damping [A·s/rad].
 * Must be << HAPTIC_DETENT_AMPLITUDE / (max_velocity).
 * At max_velocity=3 rad/s: headroom = 0.30/3=0.10 → KD < 0.05.
 * Critical damping: KD_crit ≈ 0.015 A/(rad/s) for GM3506 knob inertia.
 * KD > KD_crit → overdamped (no snap), KD >> → spring fully masked → motor floats. */
#define DETENT_KD            0.012f

/* FOC_CURRENT loop tuning
 * =========================
 * P = R_phase = 5.6 – correct DC gain: at standstill Vq = P×err = R×Iq_sp when err=Iq_sp.
 * I = 50  – corrects BEMF-induced steady-state error at speed (anti-windup via limit).
 * LIMIT = 4.2 V – applies to both proportional+integral, prevents saturation.
 * LPF_TF = 1 ms – smooths ADC quantisation noise; bandwidth = 159 Hz at 1 kHz loop. */
#define FOC_PID_P            MOTOR_PHASE_RESISTANCE   /* 5.6 V/A        */
#define FOC_PID_I            0.0f                     /* I=0: integrátor zakázán — s šumným Iq_meas způsobuje nestabilitu */
#define FOC_PID_LIMIT        HAPTIC_VOLTAGE_LIMIT     /* 4.2 V          */
#define CURRENT_LPF_TF       0.005f                   /* s (= 5 ms, α=0.83 @ 1kHz, f_3dB=32Hz) */


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
#define ADC_VREF_MV       3300U   /* 3.3V VDD reference (ADC_REF_VDD_1) — správně pro MCP602 s bias 1.65V */
                                /* Dříve ADC_REF_INTERNAL = VREFINT = 1.21V (chyba ~2.7×) */
/* Sampling offset after trough: TMC6300 propagation ~500 ns + settling        */
/* 800 ns @ 100 MHz → CCR4 = 80 counts (see low_side_cs.c for derivation)     */
#define ADC_TROUGH_OFFSET_NS  800U

/* Low-side current sense instance — mirrors SimpleFOC LowsideCurrentSense     */
static low_side_cs_t g_cs;

static const struct device *adc_dev = DEVICE_DT_GET(DT_NODELABEL(adc1));

static const struct adc_channel_cfg adc_ch0_cfg = {
    .gain             = ADC_GAIN_1,
    .reference        = ADC_REF_INTERNAL, /* STM32F4: jedina platna volba; ADC pouziva VREF+=VDD=3.3V */
    .acquisition_time = ADC_ACQ_TIME_DEFAULT,
    .channel_id       = 0,  /* PA0 - faze W (Ic) */
};
static const struct adc_channel_cfg adc_ch1_cfg = {
    .gain             = ADC_GAIN_1,
    .reference        = ADC_REF_INTERNAL, /* STM32F4: jedina platna volba; ADC pouziva VREF+=VDD=3.3V */
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

/* Anti-cogging LUT odstranen */

/* Haptic state variables */
static float start_angle = 0.0f;
static int step_count_buffer = 0;
static int num_steps_old = NUM_STEPS;
static float last_torque = 0.0f;
static int step_count = 0;
static float haptic_prev_angle = -1.0f;   /* pro výpočet inst_vel v haptic_loop */
static float haptic_inst_vel   =  0.0f;   /* vypočítaná rychlost [rad/s], LPF α=0.2 */
static float smooth_iq_filt = 0.0f;  /* IIR pre-filter na proudový setpoint smooth mode */
static float detent_iq_filt = 0.0f;  /* IIR pre-filter na proudový setpoint detent mode */
/* FOC control loop thread - triggered by k_timer at 10 kHz */
static bldc_motor_t *g_motor_ptr = NULL;
static K_SEM_DEFINE(haptic_sem, 0, 1);
static struct k_timer haptic_timer;

#define HAPTIC_THREAD_STACK_SIZE 4096
#define HAPTIC_THREAD_PRIORITY 0 /* Highest preemptible priority */
static K_THREAD_STACK_DEFINE(haptic_stack, HAPTIC_THREAD_STACK_SIZE);
static struct k_thread haptic_thread_data;

/* Aktualni pocet kroku — meneno tlacitkem v haptic_update_num_steps_from_button() */
static int g_num_steps = NUM_STEPS;

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
    }
}

int haptic_update_num_steps_from_button(void)
{
    static bool initialized = false;
    static bool last_pressed = false;
    static int64_t last_change_ms = 0;

    if (!initialized) {
        /* Nakonfiguruj PA3 jako vstup s pull-up primo pres registry
         * (obchazi Zephyr GPIO abstrakci, ktera muze byt konfliktni) */
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;  /* hodiny pro GPIOA */
        /* PA3: MODER[7:6]=00 (vstup), PUPDR[7:6]=01 (pull-up) */
        GPIOA->MODER  &= ~(3U << 6);   /* input mode */
        GPIOA->PUPDR  &= ~(3U << 6);
        GPIOA->PUPDR  |=  (1U << 6);   /* pull-up */
        last_change_ms = k_uptime_get();
        uint32_t pa3 = (GPIOA->IDR >> 3) & 1U;
        last_pressed = (pa3 == 0);  /* 0V = stisknuto */
        printk("[BTN] init: phys_PA3=%u last_pressed=%d\n",
               (unsigned)pa3, (int)last_pressed);
        initialized = true;
    }

    /* Cteni primo z IDR — 0V na PA3 = stisknuto */
    bool pressed = (((GPIOA->IDR >> 3) & 1U) == 0);
    int64_t now_ms = k_uptime_get();

    /* Rising edge s debounce 180 ms */
    if (pressed && !last_pressed && (now_ms - last_change_ms) > 180) {
        if (g_num_steps == 0) {
            g_num_steps = 20;
        } else if (g_num_steps <= 8) {
            g_num_steps = 0;
        } else {
            g_num_steps -= 4;
        }
        last_change_ms = now_ms;
        printk("[BTN] PRESS -> num_steps=%d\n", g_num_steps);
    }
    if (!pressed && last_pressed) {
        last_change_ms = now_ms;  /* reset debounce na release taky */
    }

    last_pressed = pressed;
    return g_num_steps;
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
    bldc_driver_6pwm_t *driver_6pwm = (bldc_driver_6pwm_t *)driver;
    struct as5048a_device *as5048a = (struct as5048a_device *)encoder;

    /* --- Step 1: AS5048A encoder ----------------------------------------- */
    printk("1. Initializing AS5048A encoder...\n");
    if (as5048a_init(as5048a, &as5048a_spi) < 0)
    {
        printk("   ERROR: Failed to initialize AS5048A\n");
        return -1;
    }
    printk("   [OK] AS5048A ready\n");

    /* --- Step 2: 6PWM driver (TIM1 low-side, TIM3 high-side) ------------- */
    printk("2. Initializing 6PWM driver...\n");
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
        return -1;
    }
    printk("   [OK] Driver initialized\n");

    /* --- Step 3: Low-side current sense ---------------------------------- */
    printk("3. Initializing current sense...\n");
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
        return -1;
    }
    printk("   [OK] Current sense ready\n");

    /* --- Step 3.5: ADC zero-current offset calibration ------------------- */
    /* MUST run before init_foc(). Once FOC alignment executes, phase currents
     * flow and corrupt the offset reading.  Right here TIM1 runs at neutral
     * 50% duty and no motor command has been issued — shunts carry zero A. */
    printk("3.5. Calibrating current-sense offsets...\n");
    k_msleep(100);   /* let op-amp outputs stabilize after driver power-on */
    low_side_cs_calibrate_offsets(&g_cs);
    printk("   [OK] Offsets: Ia=%.4f V  Ic=%.4f V\n",
           (double)g_cs.offset_ia, (double)g_cs.offset_ic);

    /* --- Step 4: BLDC Motor ----------------------------------------------- */
    printk("4. Initializing BLDC motor...\n");
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
        return -1;
    }
    printk("   [OK] Motor initialized\n");

    /* --- Step 5: FOC sensor calibration ---------------------------------- */
    printk("5. Running FOC calibration...\n");
    k_msleep(1000);  /* sensor needs to settle after PWM start */

    if (!bldc_motor_init_foc(motor))
    {
        printk("   ERROR: FOC calibration failed\n");
        printk("   Motor status: %d\n", motor->motor_status);
        return -1;
    }
    printk("   [OK] FOC calibration complete!\n");

    /* --- Step 6: FOC_CURRENT torque mode --------------------------------- */
    /* Gains: P = R_phase so DC gain is correct at standstill.               *
     *        I = 50 to eliminate BEMF-driven steady-state error at speed.   *
     * Integral anti-windup: _CONSTRAIN(integral, ±limit) in pid_controller. *
     * LPF: tf=1ms at 1kHz loop → α=0.5, f_3dB=159 Hz.                      */
    motor->torque_controller = FOC_CURRENT;
    motor->current_limit     = MOTOR_CURRENT_LIMIT;
    motor->voltage_limit     = HAPTIC_VOLTAGE_LIMIT;

    motor->pid_current_q.p     = FOC_PID_P;
    motor->pid_current_q.i     = FOC_PID_I;
    motor->pid_current_q.d     = 0.0f;
    motor->pid_current_q.limit = FOC_PID_LIMIT;

    /* D-axis PID: DISABLED (Vd = 0 always).
     * At haptic speeds < 10 rad/s: L*di/dt ≈ 0.1mH*5A/1ms = 0.5V << R*I = 2.24V.
     * Nonzero Vd is unnecessary and caused saturation + Iq measurement corruption.
     * With P=I=D=0, pid_controller_operator() returns 0.0 → Vd=0 exactly. */
    motor->pid_current_d.p     = 0.0f;
    motor->pid_current_d.i     = 0.0f;
    motor->pid_current_d.d     = 0.0f;
    motor->pid_current_d.limit = 0.0f;

    motor->lpf_current_q.tf  = CURRENT_LPF_TF;
    motor->lpf_current_d.tf  = CURRENT_LPF_TF;

    printk("   [OK] FOC_CURRENT: P=%.1f  I=%.0f  Vlim=%.2f V  Ilim=%.2f A  LPF tf=%.3f s\n",
           (double)FOC_PID_P, (double)FOC_PID_I,
           (double)FOC_PID_LIMIT, (double)MOTOR_CURRENT_LIMIT,
           (double)CURRENT_LPF_TF);

    /* --- Step 7: Current-sense self-test --------------------------------- */
    /* Command Iq_sp = +0.1 A for 1 s, print Iq_meas every 100 ms.           *
     * With R=5.6 Ω and P-only: Vq ≈ R*Iq_sp = 0.56 V at steady state.       *
     * Expected Iq_meas ≈ 0.09–0.10 A.  If near-zero → ADC/Clarke/Park issue. */
    printk("7. Current-sense self-test (Iq_sp=0.10 A for 1 s)...\n");
    for (int _t = 0; _t < 200; _t++) {
        sensor_update(motor->sensor);
        bldc_motor_move(motor, 0.10f);
        bldc_motor_loop_foc(motor);
        if (_t % 20 == 19) {
            printk("   [CSTEST] t=%dms  Iq_sp=0.100 A  Iq_meas=%.3f A  Id=%.3f A"
                   "  Vq=%.3f V  raw_ia=%d raw_ic=%d\n",
                   (_t + 1) * 5,
                   (double)motor->current.q,
                   (double)motor->current.d,
                   (double)motor->voltage.q,
                   (int)g_cs.last_raw_ia,
                   (int)g_cs.last_raw_ic);
        }
        k_msleep(5);
    }
    /* Ramp down to zero */
    sensor_update(motor->sensor);
    bldc_motor_move(motor, 0.0f);
    bldc_motor_loop_foc(motor);
    k_msleep(100);
    printk("   [OK] Self-test complete\n");

    /* Reset all current-loop state before entering haptic loop.
     * Self-test accumulated q-PID integral; if not cleared, Vq starts at ~0.8V
     * even with Iq_sp=0, causing immediate unintended torque on entry. */
    pid_controller_reset(&motor->pid_current_q);
    pid_controller_reset(&motor->pid_current_d);
    lowpass_filter_init(&motor->lpf_current_q, CURRENT_LPF_TF);
    lowpass_filter_init(&motor->lpf_current_d, CURRENT_LPF_TF);

    printk("================================================\n");
    printk("  System Ready - Motor Status: %d\n", motor->motor_status);
    printk("  Entering main control loop...\n");
    printk("================================================\n\n");

    motor->target = 0.0f;

    /* Current-sense offsets already calibrated in step 3.5 (before FOC).    */

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

    printk("Starting haptic control loop...\n");

    /* Start 1 kHz FOC control loop */
    g_motor_ptr = motor;
    k_timer_init(&haptic_timer, haptic_timer_cb, NULL);
    k_thread_create(&haptic_thread_data, haptic_stack,
                    K_THREAD_STACK_SIZEOF(haptic_stack),
                    haptic_thread_fn, NULL, NULL, NULL,
                    HAPTIC_THREAD_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&haptic_thread_data, "haptic_foc");
    k_timer_start(&haptic_timer, K_MSEC(1), K_MSEC(1));

    return 0;
}

void haptic_loop(bldc_motor_t *motor)
{
    //sensor_update(motor->sensor);
    //bldc_motor_loop_foc(motor);
    //bldc_motor_move(motor, motor->voltage_limit);

        float target_torque;

        int num_steps = haptic_update_num_steps_from_button();

        /* Read encoder via sensor abstraction linked in motor struct */
        sensor_update(motor->sensor);
        float current_angle = sensor_get_angle(motor->sensor);

        /* Počítej inst_vel přímo z enkodéru (motor->shaft_velocity je vždy 0
         * kvůli LPF při tomto rozvrhu volání). LPF α=0.2 potlačí 1ms šum. */
        if (haptic_prev_angle < 0.0f) haptic_prev_angle = current_angle;
        float d_ang = current_angle - haptic_prev_angle;
        while (d_ang >  _2PI * 0.5f) d_ang -= _2PI;
        while (d_ang < -_2PI * 0.5f) d_ang += _2PI;
        haptic_prev_angle = current_angle;
        /* α=0.1: more smoothing than 0.25 to suppress 1-LSB encoder quantisation noise
         * (~0.38 rad/s per tick). Effective bandwidth ≈ 16 Hz — enough for haptic. */
        haptic_inst_vel = 0.1f * (d_ang * 1000.0f) + 0.9f * haptic_inst_vel;

        /* Calculate relative angle */
        float angle_rel = current_angle - start_angle;

        // Preserve position when num_steps is changed; reset IIR filters on transition
        if (num_steps != num_steps_old) {
            step_count_buffer = step_count;
            start_angle = current_angle;
            num_steps_old = num_steps;
            angle_rel = 0.0f;
            smooth_iq_filt = 0.0f;
            detent_iq_filt = 0.0f;
        }

        /* Normalize angle to [0, 2π] */
        while (angle_rel > _2PI) angle_rel -= _2PI;
        while (angle_rel < 0) angle_rel += _2PI;

        /* Debug print every 500 ms */
        static int64_t last_print = 0;
        int64_t now_print = k_uptime_get();
        bool do_print = (now_print - last_print > 500);
        if (do_print) last_print = now_print;

        bldc_driver_6pwm_set_phase_state((bldc_driver_6pwm_t *)motor->driver,
                                         PHASE_ON, PHASE_ON, PHASE_ON);

        if (num_steps == 0) {
            /* ---- Smooth mode: velocity damping ---- */
            float vel = haptic_inst_vel;

            /* Smooth mode: velocity damping.
             * Iq_sp = (KD / Kt) * vel  [A]  — positive vel → +Iq opposes motion (verified).
             * Passed directly to FOC_CURRENT PID — PID closes the current loop. */
            float iq_sp = (SMOOTH_KD / MOTOR_KT) * vel;
            if (iq_sp >  motor->current_limit) iq_sp =  motor->current_limit;
            if (iq_sp < -motor->current_limit) iq_sp = -motor->current_limit;

            /* Light IIR on setpoint to suppress 1-LSB encoder quantisation spikes */
            smooth_iq_filt = 0.8f * iq_sp + 0.2f * smooth_iq_filt;

            bldc_motor_move(motor, smooth_iq_filt);
            bldc_motor_loop_foc(motor);

            if (do_print) {
                printk("[SMOOTH] vel=%.2f  Iq_sp=%.3f A  Iq_meas=%.3f A  Vq=%.3f V\n",
                       (double)haptic_inst_vel, (double)smooth_iq_filt,
                       (double)motor->current.q, (double)motor->voltage.q);
            }

        } else {
            /* ---- Detent mode: sinusoidal haptic profile ---- */
            float step_size  = _2PI / (float)num_steps;

            /* normalized ∈ [0, num_steps): position in step units.
             * roundf() correctly finds nearest detent for both +/- displacements. */
            float normalized = angle_rel / step_size;
            float nearest_f  = roundf(normalized);
            step_count       = (int)nearest_f + step_count_buffer;

            /* Error from nearest detent center, ∈ (-0.5, 0.5).
             * Single stable equilibrium at norm_err=0.
             * dT/d(norm_err)|0 = -K·2π < 0  →  unconditionally stable. */
            /* Error from nearest detent center, ∈ (-0.5, 0.5).
             * Stable at norm_err=0: restoring spring +A·sin(2π·err).
             * No dead-band — spring must act at all displacements for a snap feel. */
            float norm_err = normalized - nearest_f;

            float detent_iq = HAPTIC_DETENT_AMPLITUDE * sinf(_2PI * norm_err);
            last_torque = detent_iq;

            /* Velocity damping: same sign as smooth mode (+KD*vel brakes) */
            float iq_sp = detent_iq + DETENT_KD * haptic_inst_vel;
            if (iq_sp >  motor->current_limit) iq_sp =  motor->current_limit;
            if (iq_sp < -motor->current_limit) iq_sp = -motor->current_limit;

            /* Light IIR α=0.7 on current setpoint to suppress encoder noise */
            detent_iq_filt = 0.7f * iq_sp + 0.3f * detent_iq_filt;

            bldc_motor_move(motor, detent_iq_filt);
            bldc_motor_loop_foc(motor);

            if (do_print) {
                printk("[DETENT] steps=%d  err=%.3f  Iq_sp=%.3f A  Iq_meas=%.3f A  Vq=%.3f V\n",
                       num_steps, (double)norm_err, (double)detent_iq_filt,
                       (double)motor->current.q, (double)motor->voltage.q);
            }
        }
}