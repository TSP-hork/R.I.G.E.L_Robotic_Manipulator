#include "foc.h"
#include "config.h"
#include "pid.h"
#include <math.h>

#define AXIS_MAX 3                      // Maximum number of motor axes supported
#define ONE_OVER_SQRT3  0.57735026919f  // Pre-computed 1/√3, used in Clarke transform

// ============================================================================
// Per-axis PI current controllers for the d-q frame.
// Two controllers per axis: one for the d-axis (flux), one for the q-axis (torque).
// ============================================================================
static pi_controller_t pi_d[AXIS_MAX];  // d-axis (flux) PI controllers
static pi_controller_t pi_q[AXIS_MAX];  // q-axis (torque) PI controllers

// Per-axis current setpoints (references)
static float iq_ref[AXIS_MAX];          // q-axis current reference — set by the outer servo loop
static float id_ref[AXIS_MAX];          // d-axis current reference — always 0 (no field weakening)

// PI controller tuning constants (shared by all axes)
#define FOC_KP    0.5f                  // Proportional gain
#define FOC_KI    0.0004f               // Integral gain (accumulated per call)
#define FOC_LIMIT 1.0f                  // Output saturation limit (normalized ±1.0)

// ============================================================================
// foc_init: initialize all per-axis PI controllers and zero the current references.
// Must be called once at startup before any foc_step() calls.
// ============================================================================
void foc_init(void) {
    for (int i = 0; i < AXIS_MAX; i++) {
        pi_init(&pi_d[i], FOC_KP, FOC_KI, FOC_LIMIT);
        pi_init(&pi_q[i], FOC_KP, FOC_KI, FOC_LIMIT);
        iq_ref[i] = 0.0f;
        id_ref[i] = 0.0f;
    }
}

// ============================================================================
// foc_set_iq_ref: set the q-axis (torque) current reference for a given axis.
// Called by the outer position/velocity servo loop to command motor torque.
// ============================================================================
void foc_set_iq_ref(uint32_t axis, float ref) {
    if (axis < AXIS_MAX) {
        iq_ref[axis] = ref;
    }
}

// ============================================================================
// foc_step: execute one iteration of the Field-Oriented Control algorithm.
//
// Inputs:
//   axis  — motor axis index (0, 1, or 2)
//   i_a   — measured phase-A current (amps, from ADC)
//   i_b   — measured phase-B current (amps, from ADC)
//   angle — electrical rotor angle in radians (from encoder × pole pairs)
//
// Returns:
//   foc_output_t containing v_alpha, v_beta (voltage commands in stationary
//   frame, normalized ±1.0) and i_d, i_q (measured currents in rotating frame,
//   for telemetry/debugging).
//
// Processing pipeline:
//   1. Clarke transform:       (ia, ib) → (iα, iβ)
//   2. Park transform:         (iα, iβ) → (id, iq)   using rotor angle
//   3. PI current regulators:  error → (vd, vq)
//   4. Inverse Park transform: (vd, vq) → (vα, vβ)   back to stationary frame
// ============================================================================
foc_output_t foc_step(uint32_t axis, 
                      float i_a, float i_b, float angle) {
    foc_output_t out = {0};

    if (axis >= AXIS_MAX) return out;  // Guard against invalid axis index

    // ===== Clarke Transform: 3-phase currents → 2-axis stationary frame (αβ) =====
    // Uses the two-sensor variant: ic = -(ia + ib) is implied, not measured.
    float i_alpha = i_a;
    float i_beta  = (i_a + 2.0f * i_b) * ONE_OVER_SQRT3;

    // ===== Park Transform: stationary αβ → rotating dq frame (aligned with rotor flux) =====
    float sin_a = sinf(angle);  // sin(θ_electrical)
    float cos_a = cosf(angle);  // cos(θ_electrical)

    float i_d_real =  i_alpha * cos_a + i_beta * sin_a;   // d-axis current (flux component)
    float i_q_real = -i_alpha * sin_a + i_beta * cos_a;   // q-axis current (torque component)

    // Store measured d/q currents in the output struct for telemetry / debugging
    out.i_d = i_d_real;
    out.i_q = i_q_real;

    // ===== PI Current Regulators =====
    // d-axis: regulate to zero (no field weakening) → minimizes reactive current
    float error_d = id_ref[axis] - i_d_real;  // id_ref is always 0
    // q-axis: regulate to iq_ref set by the outer servo loop → controls torque
    float error_q = iq_ref[axis] - i_q_real;

    float v_d = pi_step(&pi_d[axis], error_d);  // d-axis voltage command
    float v_q = pi_step(&pi_q[axis], error_q);  // q-axis voltage command

    // ===== Inverse Park Transform: rotating dq → stationary αβ =====
    // Converts the voltage commands back to the stationary reference frame
    // so they can be fed into the SVPWM modulator.
    out.v_alpha = v_d * cos_a - v_q * sin_a;
    out.v_beta  = v_d * sin_a + v_q * cos_a;

    return out;
}

// ============================================================================
// foc_calc_svpwm: Space-Vector PWM modulator.
//
// Converts voltage commands in the stationary αβ frame (range ≈ -1..+1) into
// three unsigned PWM timer compare values (0 .. PWM_PERIOD) suitable for the
// FPGA's center-aligned PWM generators.
//
// Algorithm:
//   1. Inverse Clarke: (vα, vβ) → (va, vb, vc)  — balanced 3-phase voltages
//   2. Center-aligned injection: shift all three by the midpoint of min/max
//      to maximize DC bus utilization (equivalent to 3rd-harmonic injection).
//   3. Normalize from (-0.5 … +0.5) to (0 … 1).
//   4. Clip to [0, 1] for safety.
//   5. Scale to integer timer ticks (0 … PWM_PERIOD).
// ============================================================================
#define PWM_PERIOD_F ((float)PWM_PERIOD)  // PWM counter ceiling as float, for scaling

svpwm_output_t foc_calc_svpwm(float v_alpha, float v_beta) {
    svpwm_output_t out;

    // --- Inverse Clarke: αβ → three-phase voltages (va, vb, vc) ---
    float v_a = v_alpha;
    float v_b = -0.5f * v_alpha + 0.8660254f * v_beta;   // 0.866… = √3/2
    float v_c = -0.5f * v_alpha - 0.8660254f * v_beta;

    // --- SVPWM center-aligned (min-max) injection ---
    // Find the minimum and maximum of the three phase voltages
    float v_min = v_a;
    if (v_b < v_min) v_min = v_b;
    if (v_c < v_min) v_min = v_c;

    float v_max = v_a;
    if (v_b > v_max) v_max = v_b;
    if (v_c > v_max) v_max = v_c;

    // The center offset shifts all three phases so that the waveform is
    // centered within the PWM range — this is the space-vector equivalent
    // of third-harmonic injection and increases usable voltage by ~15%.
    float v_center = -0.5f * (v_min + v_max);

    v_a += v_center;
    v_b += v_center;
    v_c += v_center;

    // --- Normalize: shift from (-0.5 … +0.5) range to (0 … 1) range ---
    v_a += 0.5f;
    v_b += 0.5f;
    v_c += 0.5f;

    // --- Clamp to [0, 1] to prevent PWM overflow/underflow ---
    if (v_a < 0.0f) v_a = 0.0f; else if (v_a > 1.0f) v_a = 1.0f;
    if (v_b < 0.0f) v_b = 0.0f; else if (v_b > 1.0f) v_b = 1.0f;
    if (v_c < 0.0f) v_c = 0.0f; else if (v_c > 1.0f) v_c = 1.0f;

    // --- Convert normalized duty (0..1) to integer timer ticks (0..PWM_PERIOD) ---
    out.pwm_a = (uint16_t)(v_a * PWM_PERIOD_F);
    out.pwm_b = (uint16_t)(v_b * PWM_PERIOD_F);
    out.pwm_c = (uint16_t)(v_c * PWM_PERIOD_F);

    return out;
}

// ============================================================================
// foc_reset_integrators: clear the accumulated integral terms of both d/q PI
// controllers for a given axis. Should be called when an axis is disabled or
// re-enabled, to prevent integral windup from causing a current spike on restart.
// ============================================================================
void foc_reset_integrators(uint32_t axis) {
    if (axis < AXIS_MAX) {
        pi_reset_integral(&pi_d[axis]);
        pi_reset_integral(&pi_q[axis]);
    }
}