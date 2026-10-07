#ifndef FOC_H
#define FOC_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// foc_output_t: result of one FOC iteration (foc_step).
//
// Contains both the voltage commands for the modulator and the measured
// d/q currents for telemetry / outer-loop feedback.
// ============================================================================
typedef struct {
    float v_alpha;  // Voltage command in stationary α-axis (normalized, ≈ -1..+1)
    float v_beta;   // Voltage command in stationary β-axis (normalized, ≈ -1..+1)
    float i_d;      // Measured d-axis (flux) current after Park transform (for telemetry)
    float i_q;      // Measured q-axis (torque) current after Park transform (for telemetry)
} foc_output_t;

// ============================================================================
// svpwm_output_t: result of the SVPWM modulator (foc_calc_svpwm).
//
// Three unsigned PWM compare values in timer ticks (0 … PWM_PERIOD),
// ready to be sent to the FPGA's center-aligned PWM generators.
// ============================================================================
typedef struct {
    uint16_t pwm_a;  // Phase-A duty cycle in timer ticks
    uint16_t pwm_b;  // Phase-B duty cycle in timer ticks
    uint16_t pwm_c;  // Phase-C duty cycle in timer ticks
} svpwm_output_t;

// Initialize all per-axis PI controllers and zero current references.
// Must be called once at startup before any other foc_* functions.
void foc_init(void);

// Set the q-axis (torque) current reference for a given axis.
// Called by the outer position/velocity servo loop to command motor torque.
void foc_set_iq_ref(uint32_t axis, float iq_ref);

// Clear the PI integrator accumulators for a given axis.
// Call when disabling/re-enabling an axis to avoid integral windup transients.
void foc_reset_integrators(uint32_t axis);

// Execute one FOC iteration: Clarke → Park → PI regulators → inverse Park.
// Returns voltage commands (v_alpha, v_beta) and measured currents (i_d, i_q).
foc_output_t foc_step(uint32_t axis, float i_a, float i_b, float angle);

// Convert stationary-frame voltage commands (v_alpha, v_beta) into three
// PWM compare values via space-vector modulation with min-max injection.
svpwm_output_t foc_calc_svpwm(float v_alpha, float v_beta);


#ifdef __cplusplus
}
#endif

#endif