/**
 * low_side_cs.h — Low-side current sense for 2-shunt TMC6300 + STM32F411
 *
 * Mirrors SimpleFOC LowsideCurrentSense interface (init / calibrateOffsets /
 * getPhaseCurrents) but implemented as plain C for Zephyr.
 *
 * Hardware wiring (2-shunt, see schematic):
 *   ADC1_IN0 / PA0  → op-amp output for phase W  shunt  (= Ic)
 *   ADC1_IN1 / PA1  → op-amp output for phase UV shunt  (= Ia)
 *   Ib = -(Ia + Ic)  via Kirchhoff's current law
 *
 * ADC trigger:
 *   TIM1 center-aligned mode 1, CH4 compare (no GPIO output) fires ADC1
 *   injected group at counter trough + OFFSET_NS (overcomes TMC6300 ~500 ns
 *   propagation delay and settling time before sampling).
 *
 * Gain chain (per-phase, configurable at init):
 *   I [A] = (V_adc - V_offset) / (amp_gain * shunt_ohms)
 *         = (V_adc - V_offset) * volts_to_amps_ratio
 */

#ifndef LOW_SIDE_CS_H
#define LOW_SIDE_CS_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------------------------------------------------------------------
 * Phase current result — mirrors SimpleFOC PhaseCurrent_s
 * ------------------------------------------------------------------------- */
typedef struct {
    float a;   /* Phase A (U) current [A] */
    float b;   /* Phase B (V) current [A] — reconstructed via KCL if not measured */
    float c;   /* Phase C (W) current [A] */
} phase_current_t;

/* ---------------------------------------------------------------------------
 * Low-side current sense configuration
 * ------------------------------------------------------------------------- */
typedef struct {
    /* Shunt resistor value [Ω] */
    float shunt_resistor;

    /* Op-amp gain (dimensionless) */
    float amp_gain;

    /* Derived: 1 / (amp_gain * shunt_resistor) — V→A conversion factor */
    float volts_to_amps_ratio;

    /* Per-channel gain trim (set to volts_to_amps_ratio by default) */
    float gain_a;   /* Phase A */
    float gain_c;   /* Phase C (W) */

    /* ADC reference voltage [V] */
    float vref;

    /* ADC full-scale counts (4095 for 12-bit) */
    float adc_full_scale;

    /* Sampling offset at trough [ns] — added to CCR4 to skip driver
     * propagation delay and switching noise.  Default: 800 ns. */
    uint32_t offset_trough_ns;

    /* Calibrated zero-current ADC voltage offsets [V] */
    float offset_ia;
    float offset_ic;

    /* true after successful init + calibration */
    bool initialized;

    /* Skip driver alignment check (set true if wiring verified) */
    bool skip_align;
} low_side_cs_t;

/* ---------------------------------------------------------------------------
 * API — mirrors SimpleFOC LowsideCurrentSense
 * ------------------------------------------------------------------------- */

/**
 * Populate the configuration structure.
 * Must be called before low_side_cs_init().
 *
 * @param cs            Pointer to structure to fill
 * @param shunt_ohms    Shunt resistor value [Ω]
 * @param amp_gain      Op-amp gain (e.g. 31.0 for INA180A3)
 * @param vref_mv       ADC reference voltage in mV (e.g. 3300)
 * @param offset_trough_ns  ADC sampling offset from trough [ns] (default 800)
 */
void low_side_cs_init_struct(low_side_cs_t *cs,
                             float shunt_ohms,
                             float amp_gain,
                             uint32_t vref_mv,
                             uint32_t offset_trough_ns);

/**
 * Initialize hardware (ADC injected channels + TIM1 center-aligned + CCR4
 * trigger).  Call AFTER bldc_driver_6pwm_init_hw() so TIM1 is already
 * running and we can safely reconfigure CR1/CR2/CCR4.
 *
 * @param cs   Pointer to low_side_cs_t (already filled by low_side_cs_init_struct)
 * @return  1 on success, 0 on failure
 */
int low_side_cs_init(low_side_cs_t *cs);

/**
 * Calibrate zero-current offsets.
 * Motor must be stationary with zero voltage applied (mirrors SimpleFOC
 * calibrateOffsets).  Blocks for ~80 ms (2000 × one PWM period at 25 kHz).
 *
 * @param cs   Pointer to initialised low_side_cs_t
 */
void low_side_cs_calibrate_offsets(low_side_cs_t *cs);

/**
 * Read phase currents from the last hardware-triggered ADC conversion.
 * Non-blocking — returns cached values if JEOC not yet set.
 * Mirrors SimpleFOC getPhaseCurrents().
 *
 * @param cs   Pointer to initialised low_side_cs_t
 * @return     phase_current_t with .a / .b / .c in Amperes
 */
phase_current_t low_side_cs_get_phase_currents(low_side_cs_t *cs);

/**
 * Check polarity alignment with motor driver.
 * Applies a small voltage on phase A and verifies the current sign.
 * Returns 1 if alignment OK (or skip_align=true), 0 if mismatch.
 * Pass the motor's voltage_limit for the test voltage.
 *
 * @param cs            Pointer to initialised low_side_cs_t
 * @param test_voltage  Voltage [V] to apply during alignment
 * @return              1 = OK, 0 = gain polarity mismatch detected
 */
int low_side_cs_driver_align(low_side_cs_t *cs, float test_voltage);

#ifdef __cplusplus
}
#endif

#endif /* LOW_SIDE_CS_H */
