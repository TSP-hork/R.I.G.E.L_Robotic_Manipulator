#ifndef PID_H
#define PID_H

// ============================================================================
// pi_controller_t: lightweight Proportional-Integral (PI) controller.
//
// Used inside the FOC current loop (one instance for d-axis, one for q-axis,
// per motor axis). All functions are static inline for zero call overhead —
// critical for the 20 kHz control-loop timing budget.
// ============================================================================
typedef struct {
    float kp;          // Proportional gain
    float ki;          // Integral gain (already includes the sample period dt)
    float integral;    // Accumulated integral term
    float out_limit;   // Symmetric output saturation limit (± out_limit)
} pi_controller_t;

// ----------------------------------------------------------------------------
// pi_init: configure a PI controller with gains and output limit, and zero
// the integrator. Must be called before the first pi_step().
// ----------------------------------------------------------------------------
static inline void pi_init(pi_controller_t *pi, 
                           float kp, float ki, float limit) {
    pi->kp = kp;
    pi->ki = ki;
    pi->integral = 0.0f;
    pi->out_limit = limit;
}

// ----------------------------------------------------------------------------
// pi_reset: clear the integrator accumulator to zero.
// (Generic reset — same as pi_reset_integral, kept for API symmetry.)
// ----------------------------------------------------------------------------
static inline void pi_reset(pi_controller_t *pi) {
    pi->integral = 0.0f;
}

// ----------------------------------------------------------------------------
// pi_step: compute one PI iteration.
//
//   error = (reference − measurement), sign convention: positive error
//           means the output should increase.
//
// Returns the controller output, clamped to ± out_limit.
//
// The integral term is accumulated first (ki already contains dt), then
// clamped independently (anti-windup) before being summed with the
// proportional term. The final sum is clamped again to enforce the hard
// output limit.
// ----------------------------------------------------------------------------
static inline float pi_step(pi_controller_t *pi, float error) {
    // Accumulate integral term (ki already scaled by dt, so no separate dt multiply)
    pi->integral += pi->ki * error;

    // Anti-windup: clamp the integrator itself to prevent runaway accumulation
    // when the output is saturated (e.g. during current limit or stall)
    if (pi->integral > pi->out_limit) pi->integral = pi->out_limit;
    if (pi->integral < -pi->out_limit) pi->integral = -pi->out_limit;

    // Sum proportional and integral contributions
    float output = pi->kp * error + pi->integral;

    // Hard clamp the total output to the allowed range
    if (output > pi->out_limit) output = pi->out_limit;
    if (output < -pi->out_limit) output = -pi->out_limit;

    return output;
}

// ----------------------------------------------------------------------------
// pi_reset_integral: clear the integrator accumulator to zero.
// Call this when an axis is disabled or re-enabled to avoid a current spike
// caused by stale integral windup from a previous operating condition.
// ----------------------------------------------------------------------------
static inline void pi_reset_integral(pi_controller_t *pi) {
    pi->integral = 0.0f;
}

#endif