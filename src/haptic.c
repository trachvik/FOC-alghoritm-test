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
/* Smooth mode: dampovaci koeficient [A·s/rad].
 * Kd=0.025 → pri 1 rad/s = 0.025 A = 1.9 mN·m → citelny odpor pri pomalom pohybe. */
#define SMOOTH_KD            0.006f  /* N·m/(rad/s) — tlumicí moment; 0.006/0.0778≈0.077 A/(rad/s)
                                        * pri 5 rad/s → 0.38 A (citelny odpor) */
/* Detent mode: dampovaci koeficient.  Potlacuje oscilacie na hranicich kroku.
 * Kd=0.008 → pri 5 rad/s = 0.04 A protisila; pri stani ≈0 A (bez vplyvu). */
#define DETENT_KD            0.008f

/* Anti-cogging compensation LUT
 * ACAL_SIZE = 512 → rozlišení 2π/512 ≈ 0.7° na záznam LUT.
 * ACAL_CAL_I  = kalibrační proud [A]; nesmí být příliš malý (motor se neroztočí)
 *               ani příliš velký (zkreslení třením).
 * ACAL_SAMPLE_MS = doba vzorkování [ms]; motor musí stihnout ≥2 otáčky.
 * ACAL_GAIN   = přepočet odchylky [rad/s] → korekční proud [A].
 *               Empiricky: ~0.5 pro GM3506, snižte pokud se motor přehřívá. */
#define ACAL_SIZE        512
#define ACAL_CAL_I       0.4f        /* [A] proud pro rozběh — max limit = max dostupný výkon */
#define ACAL_CHECK_I     0.30f       /* [A] proud při vzorkování */
#define ACAL_CAL_V       3.0f        /* [V] přímo napěťový příkaz pro rozběh — překoná cogging bez omezení proudem */
#define ACAL_SAMPLE_MS   30000       /* [ms] doba vzorkování (30 s) — třikrát více průměrování */
#define ACAL_SMOOTH_TAP  9           /* počet sousedů: 9×0.703°=6.3° < 16.4° cogging perioda (11PP) */
                                       /* 27-tap=19°>16.4° → ničí 88% signálu! */
/* ACAL_GAIN: vel_deviation [rad/s] → korekční proud [A].
 * Pomalejší kalibrace (avg~3-5 rad/s) → větší odchylky (±2-4 rad/s)
 * → gain=0.030 dá ~0.090-0.120 A korekci. */
#define ACAL_GAIN        0.060f

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

/* Anti-cogging LUT — indexováno mechanickým úhlem hřídele [0, 2π) */
static float cogging_lut[ACAL_SIZE];
static bool  cogging_valid = false;

/* Vrátí index LUT pro daný mechanický úhel */
static inline int _acal_idx(float angle)
{
    float a = angle;
    while (a <  0.0f)  a += _2PI;
    while (a >= _2PI)  a -= _2PI;
    int idx = (int)(a * (float)ACAL_SIZE / _2PI);
    if ((unsigned)idx >= (unsigned)ACAL_SIZE) idx = 0;
    return idx;
}

/* Lineárně interpolovaná korekce z LUT [A] pro daný úhel */
static float anticogging_ff(float angle)
{
    if (!cogging_valid) return 0.0f;
    float a = angle;
    while (a <  0.0f)  a += _2PI;
    while (a >= _2PI)  a -= _2PI;
    float f  = a * (float)ACAL_SIZE / _2PI;
    int   i0 = (int)f;
    int   i1 = (i0 + 1) % ACAL_SIZE;
    float t  = f - (float)i0;
    return cogging_lut[i0] * (1.0f - t) + cogging_lut[i1] * t;
}

/* Haptic state variables */
static float start_angle = 0.0f;
static int step_count_buffer = 0;
static int num_steps_old = NUM_STEPS;
static float last_torque = 0.0f;
static int step_count = 0;
static float haptic_prev_angle = -1.0f;   /* pro výpočet inst_vel v haptic_loop */
static float haptic_inst_vel   =  0.0f;   /* vypočítaná rychlost [rad/s], LPF α=0.2 */
static float smooth_current_filt = 0.0f;  /* IIR filtr výstupního proudu smooth mode */
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

/* ---------------------------------------------------------------------------
 * Anti-cogging calibration
 * Pomalu roztočí motor konstantním proudem ACAL_CAL_I po dobu ACAL_SAMPLE_MS.
 * V každém LUT slotu naměří průměrnou rychlost a odchylku od globálního
 * průměru přepočítá na korekční proud uložený do cogging_lut[].
 *
 * Vyvolej z haptic_init() PŘED startem haptic_timer (motor ještě neběží).
 * Trvá přibližně (1 + ACAL_SAMPLE_MS/1000) sekund.
 * --------------------------------------------------------------------------- */
static void haptic_calibrate_anticogging(bldc_motor_t *motor)
{
    printk("[ACAL] Zacinam kalibraci (cca %d s)...\n",
           1 + ACAL_SAMPLE_MS / 1000);
    cogging_valid = false;
    for (int i = 0; i < ACAL_SIZE; i++) cogging_lut[i] = 0.0f;

    /* Statické akumulátory — vyhnutí se velkým lokálním proměnným na zásobníku */
    static float vel_acc[ACAL_SIZE];
    static int   vel_cnt[ACAL_SIZE];
    for (int i = 0; i < ACAL_SIZE; i++) { vel_acc[i] = 0.0f; vel_cnt[i] = 0; }

    /* --- Fáze 0: bidirectionální jog (500 ms) — uvolnění z cogging jamky ---- */
    /* motor může být uvízlý v poloze kde +I nestačí překonat kolísání           */
    /* → rychlé střídání směrů ho vytrže, pak FOC zvolí směr dle polohy         */
    printk("[ACAL] Jog faze: uvolnuji rotor z cogging...\n");
{
        /* Jog přes napěťový příkaz (ne proud) — překoná omezení Vq=R×I při stání */
        for (int j = 0; j < 500; j++) {
            sensor_update(motor->sensor);
            float sign = ((j / 50) % 2 == 0) ? 1.0f : -1.0f;
            /* Přímo nastavíme q-ovou napětí přes větší hodnotu než R×Ilim */
            bldc_motor_set_phase_voltage(motor, sign * ACAL_CAL_V, 0.0f,
                bldc_motor_electrical_angle(motor));
            k_msleep(1);
        }
    }

    /* --- Fáze 1: rozběh (2 s) — print shaft_velocity pro diagnostiku ------- */
    sensor_update(motor->sensor);
    float prev_angle_runup = sensor_get_angle(motor->sensor);
    float runup_vel_sum = 0.0f;   /* pro kontrolu otáčení na konci rozběhu */
    int   runup_vel_n   = 0;
    for (int i = 0; i < 2000; i++) {
        sensor_update(motor->sensor);
        /* Vysoký proud jen prvních 500 ms (překonání koggingu), pak nižší pro lepší SNR */
        /* Prvních 1500 ms: napěťový příkaz ACAL_CAL_V (překoná Vq=R×I omezení)
         * Poté: bldc_motor_move s ACAL_CHECK_I pro přesnější snapshot rychlosti */
        if (i < 1500) {
            bldc_motor_set_phase_voltage(motor, ACAL_CAL_V, 0.0f,
                bldc_motor_electrical_angle(motor));
        } else {
            bldc_motor_move(motor, ACAL_CHECK_I);
            bldc_motor_loop_foc(motor);
        }

        /* Výpočet rychlosti z rozdílu úhlů (obchází LPF uvnitř motoru) */
        float a = sensor_get_angle(motor->sensor);
        float da = a - prev_angle_runup;
        while (da >  _2PI * 0.5f) da -= _2PI;
        while (da < -_2PI * 0.5f) da += _2PI;
        prev_angle_runup = a;

        /* Akumuluj rychlost posledních 200 ms rozběhu pro kontrolu otáčení */
        if (i >= 1800) { runup_vel_sum += da * 1000.0f; runup_vel_n++; }

        if (i % 200 == 199) {
            float inst_vel = da * 1000.0f;  /* rad/s, dt=1ms */
            printk("[ACAL] runup %d ms: inst_vel=%.3f rad/s  shaft_vel=%.3f\n",
                   i + 1, (double)inst_vel, (double)motor->shaft_velocity);
        }
        k_msleep(1);
    }

    /* Pokud se motor v posledních 200 ms rozběhu netočil, přeskočíme kalibraci hned */
    float runup_avg_vel = (runup_vel_n > 0) ? runup_vel_sum / (float)runup_vel_n : 0.0f;
    if (fabsf(runup_avg_vel) < 1.0f) {
        printk("[ACAL] VAROVANI: motor se neroztocil pri rozbehu (avg_vel=%.3f) — preskakuji\n",
               (double)runup_avg_vel);
        bldc_motor_move(motor, 0.0f);
        bldc_motor_loop_foc(motor);
        return;
    }

    /* --- Fáze 2: vzorkování rychlosti (z rozdílu úhlů, ne z LPF) ---------- */
    sensor_update(motor->sensor);
    float prev_angle = sensor_get_angle(motor->sensor);
    k_msleep(1);

    float vel_sum = 0.0f;
    int   vel_n   = 0;
    int   last_pct = -1;

    for (int i = 0; i < ACAL_SAMPLE_MS; i++) {
        sensor_update(motor->sensor);
        bldc_motor_move(motor, ACAL_CHECK_I);   /* nižší proud = lepší SNR */
        bldc_motor_loop_foc(motor);

        float angle = sensor_get_angle(motor->sensor);

        /* Instantánní rychlost z enkodéru — dt = 1 ms */
        float d_angle = angle - prev_angle;
        while (d_angle >  _2PI * 0.5f) d_angle -= _2PI;
        while (d_angle < -_2PI * 0.5f) d_angle += _2PI;
        float vel = d_angle * 1000.0f;   /* [rad/s] */
        prev_angle = angle;

        if (fabsf(vel) > 0.005f) {   /* přijímáme oba směry otáčení */
            int idx = _acal_idx(angle);
            vel_acc[idx] += vel;
            vel_cnt[idx]++;
            vel_sum += vel;
            vel_n++;
        }

        /* Průběžný výpis každých 20 % */
        int pct = i * 100 / ACAL_SAMPLE_MS;
        if (pct != last_pct && pct % 20 == 0) {
            printk("[ACAL] %d %%  inst_vel=%.4f rad/s  vzorky=%d\n",
                   pct, (double)(d_angle * 1000.0f), vel_n);
            last_pct = pct;
        }
        k_msleep(1);
    }

    /* --- Fáze 3: výpočet LUT ---------------------------------------------- */
    float avg_vel = (vel_n > 0) ? vel_sum / (float)vel_n : 0.0f;
    printk("[ACAL] prumerna rychlost=%.3f rad/s  vzorky=%d\n",
           (double)avg_vel, vel_n);

    if (fabsf(avg_vel) < 0.001f) {
        printk("[ACAL] VAROVANI: motor se neroztocil (vel<0.001) — preskakuji\n");
        bldc_motor_move(motor, 0.0f);
        bldc_motor_loop_foc(motor);
        return;
    }

    float clamp = MOTOR_CURRENT_LIMIT * 0.25f;   /* max FF = 25 % Ilim ≈ 0.10 A */
    int filled = 0;
    for (int i = 0; i < ACAL_SIZE; i++) {
        if (vel_cnt[i] > 0) {
            float slot_vel = vel_acc[i] / (float)vel_cnt[i];
            /* slot_vel - avg_vel: at cogging peaks motor slows relative to avg
             * → slot_vel > avg_vel (less negative) → corr positive → CW push;
             * this matches sign convention: positive current = CW force.         */
            float corr = ACAL_GAIN * (slot_vel - avg_vel);
            if (corr >  clamp) corr =  clamp;
            if (corr < -clamp) corr = -clamp;
            cogging_lut[i] = corr;
            filled++;
        }
    }

    printk("[ACAL] LUT: %d/%d slotu pokryto (%d %%)\n",
           filled, ACAL_SIZE, filled * 100 / ACAL_SIZE);

    if (filled >= ACAL_SIZE * 3 / 4) {
        /* Post-processing: klouzavý průměr šířky ACAL_SMOOTH_TAP.
         * GM3506 (11pp) má ~23 slotů/hump; 27-tap filtr potlačí slot-to-slot
         * šum a zachová prostorový signál coggingu. */
        static float lut_tmp[ACAL_SIZE];
        for (int i = 0; i < ACAL_SIZE; i++) {
            float sum = 0.0f;
            int half = ACAL_SMOOTH_TAP / 2;
            for (int k = -half; k <= half; k++) {
                sum += cogging_lut[(i + k + ACAL_SIZE) % ACAL_SIZE];
            }
            lut_tmp[i] = sum / (float)ACAL_SMOOTH_TAP;
        }
        for (int i = 0; i < ACAL_SIZE; i++) cogging_lut[i] = lut_tmp[i];

        cogging_valid = true;
        printk("[ACAL] Anti-cogging AKTIVNI (LUT vyhlazen %d-tap)\n", ACAL_SMOOTH_TAP);
    } else {
        printk("[ACAL] Nedostatecne pokryti — anti-cogging VYPNUT\n");
    }

    bldc_motor_move(motor, 0.0f);
    bldc_motor_loop_foc(motor);
}

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
    /* P = R_phase → Vq = R * (Iq_sp - Iq_meas).                            *
     * I = 0 intentionally: Iq_meas ≈ 0 (current sense not yet working), so *
     * a non-zero I-term would wind up in the direction of the dominant      *
     * setpoint history and create CW/CCW asymmetry.  With I=0 the output   *
     * is Vq = R * Iq_sp — symmetric, identical to VOLTAGE mode.            *
     * Restore I=5 once Iq_meas correctly tracks Iq_sp.                     */
    motor->torque_controller = FOC_CURRENT;
    motor->current_limit     = MOTOR_CURRENT_LIMIT;

    motor->pid_current_q.p     = MOTOR_PHASE_RESISTANCE;  /* V/A = Ω */
    motor->pid_current_q.i     = 0.0f;   /* 0 until Iq_meas works */
    motor->pid_current_q.d     = 0.0f;
    motor->pid_current_q.limit = 1.5f;

    motor->pid_current_d.p     = MOTOR_PHASE_RESISTANCE;
    motor->pid_current_d.i     = 0.0f;   /* 0 until Iq_meas works */
    motor->pid_current_d.d     = 0.0f;
    motor->pid_current_d.limit = 1.5f;

    motor->lpf_current_q.tf = 0.005f;
    motor->lpf_current_d.tf = 0.005f;

    pid_controller_reset(&motor->pid_current_q);
    pid_controller_reset(&motor->pid_current_d);
    motor->current.d = 0.0f;
    motor->current.q = 0.0f;
    printk("   [OK] FOC_CURRENT: P=%.1f V/A  I=0 (Iq_meas broken)  Ilim=%.2f A\n",
           (double)MOTOR_PHASE_RESISTANCE, (double)MOTOR_CURRENT_LIMIT);

    printk("================================================\n");
    printk("  System Ready - Motor Status: %d\n", motor->motor_status);
    printk("  Entering main control loop...\n");
    printk("================================================\n\n");

    motor->target = 0.0f;

    /* Settle at zero torque so ADC reads true zero-current bias for calibration */
    for (int i = 0; i < 20; i++)
    {
        bldc_motor_loop_foc(motor);
        bldc_motor_move(motor, 0.0f);
        k_msleep(1);
    }

    /* --- Step 7: Calibrate ADC zero-current offsets ---------------------- */
    printk("7. Calibrating current-sense offsets...\n");
    low_side_cs_calibrate_offsets(&g_cs);
    printk("   [OK] Offsets: Ia=%.4f V  Ic=%.4f V\n",
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

    /* --- Step 8: Anti-cogging calibration ------------------------------- */
    printk("8. Kalibruji anti-cogging LUT...\n");
    haptic_calibrate_anticogging(motor);

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
        float torque_filter_alpha = 0.8f;

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
        haptic_inst_vel = 0.2f * (d_ang * 1000.0f) + 0.8f * haptic_inst_vel;

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

        /* Debug print every 500 ms */
        static int64_t last_print = 0;
        int64_t now_print = k_uptime_get();
        bool do_print = (now_print - last_print > 500);
        if (do_print) last_print = now_print;

        bldc_driver_6pwm_set_phase_state((bldc_driver_6pwm_t *)motor->driver,
                                         PHASE_ON, PHASE_ON, PHASE_ON);

        if (num_steps == 0) {
            /* ---- Smooth mode: torque-based pipeline (stejný jako detent mode) --
             * target_torque [N·m] = SMOOTH_KD × vel  (tlumičí moment)
             * target_current = target_torque / MOTOR_KT + ff + i_bemf               */
            float vel = haptic_inst_vel;
            float ff_raw = anticogging_ff(current_angle);
            /* Dead-band: hodnoty pod 5 mA jsou čistý enkodérový šum, ne cogging */
            float ff_base = (ff_raw > -0.005f && ff_raw < 0.005f) ? 0.0f : ff_raw;
            /* Škálování ff podle rychlosti: při stoji ff=0 (žádný hum), při |vel|>4 ff=plný.
             * Zabrání vibracím LUT při stoji a maskování pocitu kroků v detent mode. */
            float vel_abs = haptic_inst_vel < 0.0f ? -haptic_inst_vel : haptic_inst_vel;
            float ff_scale = vel_abs / (vel_abs + 2.0f);  /* 0 při stoji, ~0.67 při 4 rad/s */
            float ff = ff_base * ff_scale;

            target_torque  = SMOOTH_KD * vel;              /* N·m, tlumičí moment (bez scale_factor) */
            last_torque    = target_torque;

            float target_current = target_torque / MOTOR_KT;
            if (target_current >  motor->current_limit) target_current =  motor->current_limit;
            if (target_current < -motor->current_limit) target_current = -motor->current_limit;

            /* IIR filtr výstupu: α=0.15 → cutoff ~24 Hz, silnější potlačení 1kHz šumu.
             * ff se přičítá AŽ PO filtru — bez fázového zpoždění → přesná kompenzace coggingu. */
            smooth_current_filt = 0.15f * target_current + 0.85f * smooth_current_filt;
            float final_current = smooth_current_filt + ff;
            if (final_current >  motor->current_limit) final_current =  motor->current_limit;
            if (final_current < -motor->current_limit) final_current = -motor->current_limit;

            last_torque = 0.0f;
            bldc_motor_move(motor, final_current);
            bldc_motor_loop_foc(motor);

            if (do_print) {
                printk("[SMOOTH] vel=%.2f  Iq=%.3f A  ff=%.4f(x%.2f)  valid=%d\n",
                       (double)haptic_inst_vel, (double)final_current,
                       (double)ff_base, (double)ff_scale, (int)cogging_valid);
            }

        } else {
            /* ---- Detent mode: sinusový haptic profil -------------------- */
            float step_size   = _2PI / (float)num_steps;
            float step_count_f = ((float)num_steps / _2PI) * angle_rel;
            step_count = (int)roundf(step_count_f) + step_count_buffer;
            int step_count_abs = (int)step_count_f;
            float between_steps_pos = angle_rel - step_count_abs * step_size + step_size / 2.0f;
            float scale_factor = 0.75f;  /* pro snížení citlivosti a lepší stabilitu */

            float norm_pos = between_steps_pos / step_size;
            target_torque = -HAPTIC_TORQUE_LIMIT * sinf(_2PI * norm_pos) * scale_factor;

            float dist = (norm_pos - 0.5f < 0.0f) ? -(norm_pos - 0.5f) : (norm_pos - 0.5f);
            if (dist < 0.05f) {
                target_torque = 0.0f;
            } else if (dist < 0.15f) {
                float scale = (dist - 0.05f) / 0.1f;
                target_torque *= scale;
            }

            target_torque = torque_filter_alpha * target_torque
                          + (1.0f - torque_filter_alpha) * last_torque * 0.3f;
            last_torque = target_torque;

            float target_current = target_torque / MOTOR_KT;
            /* Velocity damping: same sign convention as smooth mode (+Kd*vel brakes). */
            target_current += DETENT_KD * haptic_inst_vel;
            if (target_current >  motor->current_limit) target_current =  motor->current_limit;
            if (target_current < -motor->current_limit) target_current = -motor->current_limit;

            bldc_motor_move(motor, target_current);
            bldc_motor_loop_foc(motor);

            if (do_print) {
                printk("[DETENT] steps=%d  Iq=%.3f A  Iq_meas=%.3f A\n",
                       num_steps, (double)target_current, (double)motor->current.q);
            }
        }
}