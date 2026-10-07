#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// servo.h — PUBLIC CONTRACT for the position/velocity servo layer.
//
// This is the outer control loop that sits between the trajectory planner
// (which says WHERE the joint should be) and the FOC current loop (which
// makes the motor produce a commanded torque). It runs at the servo rate
// (target ~1 kHz) — slower than FOC (20 kHz), faster than trajectory (~100 Hz).
//
//   trajectory  →  [ SERVO ]  →  FOC
//   pos/vel/acc     iq_ref       phase currents
//
// Control structure (per axis): a classic two-stage cascade with feed-forward.
//
//   position error → P gain ─┐
//                            ├→ velocity command
//        target velocity ────┘   (feed-forward: don't wait for position error
//                                  to build up before moving)
//
//   velocity error → PI gain ─┐
//                             ├→ current (iq) command
//    target accel × ka ───────┘   (feed-forward: pre-empt the torque needed
//                                   to accelerate the known inertia)
//
// Feed-forward is what makes a trajectory-driven servo track tightly: the
// planner already knows the desired velocity and acceleration at every
// instant, so we hand them straight to the loop instead of forcing the
// feedback terms to reconstruct them from error. The PID only has to correct
// the residual (disturbances, model error, friction).
//
// SWAPPABILITY / DECOUPLING:
//   • This layer takes plain floats for the setpoint (pos/vel/acc) and the
//     measurement (pos/vel). It does NOT include trajectory.h or any encoder
//     header — the caller wires those together. So servo has zero compile-time
//     dependency on how the trajectory is generated or how the encoder is read.
//   • It does NOT call the FOC layer itself. servo_step() RETURNS the iq_ref;
//     the caller passes it to foc_set_iq_ref(). This keeps servo pure,
//     portable, and desktop-testable, and means integrating it touches no
//     working code — you add one call site.
//
// Pure portable logic — no hardware access, no global state. Desktop-testable.
// ============================================================================

#ifndef SERVO_MAX_AXES
#define SERVO_MAX_AXES 6   // Design target is a 6-axis arm (prototype uses 3)
#endif

// ----------------------------------------------------------------------------
// servo_gains_t: tuning for one axis. Set once at init; may be re-tuned live.
// ----------------------------------------------------------------------------
typedef struct {
    float kp_pos;      // Position P gain      (velocity per unit of position error)
    float kp_vel;      // Velocity P gain      (current  per unit of velocity error)
    float ki_vel;      // Velocity I gain      (already includes dt; see pid.h)
    float ff_vel;      // Velocity feed-forward scale (0..1; usually 1.0)
    float ff_acc;      // Acceleration feed-forward scale → current (∝ inertia/Kt)
    float vel_limit;   // Clamp on the internal velocity command (units/s)
    float iq_limit;    // Clamp on the output current reference (amps)
} servo_gains_t;

// ----------------------------------------------------------------------------
// servo_setpoint_t: the reference for one axis at the current instant.
// Layout matches the trajectory layer's per-axis output (pos/vel/acc), so the
// caller can feed a traj_state_t straight in — but servo does not depend on
// that type, it just reads three floats.
// ----------------------------------------------------------------------------
typedef struct {
    float pos;  // Desired position
    float vel;  // Desired velocity (feed-forward)
    float acc;  // Desired acceleration (feed-forward)
} servo_setpoint_t;

// ----------------------------------------------------------------------------
// servo_init: configure all axes' gains and zero their internal integrators.
// Call once at startup before servo_step().
//
//   gains      — array of per-axis gains, length axis_count
//   axis_count — number of axes (1 .. SERVO_MAX_AXES)
//
// Returns 0 on success, negative on bad arguments.
// ----------------------------------------------------------------------------
int servo_init(const servo_gains_t *gains, uint32_t axis_count);

// ----------------------------------------------------------------------------
// servo_step: run one servo iteration for one axis and return the current
// (iq) reference to command.
//
//   axis         — axis index (0 .. axis_count-1)
//   sp           — position/velocity/accel setpoint for this instant
//   measured_pos — actual position (from encoder, same units as sp.pos)
//   measured_vel — actual velocity (from encoder-derived estimate)
//
// Returns the torque-producing current reference iq_ref (amps), already
// clamped to the axis's iq_limit. The caller passes this to foc_set_iq_ref().
//
// On an invalid axis index the function returns 0.0f (safe: no torque).
// ----------------------------------------------------------------------------
float servo_step(uint32_t axis, const servo_setpoint_t *sp,
                 float measured_pos, float measured_vel);

// ----------------------------------------------------------------------------
// servo_reset: clear the internal velocity-loop integrator for one axis.
// Call when enabling/re-enabling an axis to avoid an integral-windup kick,
// mirroring foc_reset_integrators() in the FOC layer.
// ----------------------------------------------------------------------------
void servo_reset(uint32_t axis);

// ----------------------------------------------------------------------------
// servo_set_gains: update one axis's gains at runtime (e.g. for auto-tuning).
// Integrator state is preserved. Returns 0 on success, negative on bad axis.
// ----------------------------------------------------------------------------
int servo_set_gains(uint32_t axis, const servo_gains_t *gains);

#ifdef __cplusplus
}
#endif

#endif
