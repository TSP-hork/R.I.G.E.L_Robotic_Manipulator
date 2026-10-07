#include "servo.h"
#include "pid.h"     // reuse the project's PI controller (static inline, anti-windup)

// ============================================================================
// servo.c — per-axis cascade position/velocity controller.
//
// The velocity loop is a PI controller reused from 2_utility/control/pid.h
// (the same building block the FOC current loop uses). The position loop is a
// single proportional gain — standard for a servo cascade, because the
// velocity feed-forward from the trajectory already supplies the bulk of the
// motion command, leaving the position term only to null out steady-state
// offset.
//
// No dynamic allocation, no hardware, no global side effects beyond this
// file's static per-axis state.
// ============================================================================

// Per-axis controller state and tuning. Sized to the compile-time max.
static pi_controller_t s_vel_pi[SERVO_MAX_AXES];  // velocity-loop PI controllers
static servo_gains_t   s_gains[SERVO_MAX_AXES];   // per-axis gains
static uint32_t        s_axis_count = 0;          // active axes

int servo_init(const servo_gains_t *gains, uint32_t axis_count) {
    if (gains == 0 || axis_count == 0 || axis_count > SERVO_MAX_AXES) {
        return -1;
    }

    for (uint32_t i = 0; i < axis_count; i++) {
        s_gains[i] = gains[i];
        // The velocity PI owns kp_vel/ki_vel and clamps its own output to the
        // current limit. pi_init also zeros the integrator.
        pi_init(&s_vel_pi[i], gains[i].kp_vel, gains[i].ki_vel,
                gains[i].iq_limit);
    }
    s_axis_count = axis_count;
    return 0;
}

float servo_step(uint32_t axis, const servo_setpoint_t *sp,
                 float measured_pos, float measured_vel) {
    if (axis >= s_axis_count || sp == 0) {
        return 0.0f;  // invalid axis → command zero torque (safe default)
    }

    const servo_gains_t *g = &s_gains[axis];

    // ===== Position loop (P) + velocity feed-forward =====
    // The trajectory's desired velocity is fed forward directly; the P term
    // only corrects the residual position error. This is what lets the joint
    // follow a moving target without lagging behind it.
    float pos_error = sp->pos - measured_pos;
    float vel_cmd   = g->kp_pos * pos_error + g->ff_vel * sp->vel;

    // Clamp the internal velocity command so a large position error can't
    // demand an unphysical slew rate.
    if (vel_cmd >  g->vel_limit) vel_cmd =  g->vel_limit;
    if (vel_cmd < -g->vel_limit) vel_cmd = -g->vel_limit;

    // ===== Velocity loop (PI) + acceleration feed-forward =====
    // The PI nulls the velocity error; the accel feed-forward pre-supplies the
    // torque needed to accelerate the known inertia (ff_acc ≈ inertia / Kt),
    // so the integrator doesn't have to chase acceleration transients.
    float vel_error = vel_cmd - measured_vel;
    float iq = pi_step(&s_vel_pi[axis], vel_error) + g->ff_acc * sp->acc;

    // Final hard clamp on the commanded current (the PI already clamps its own
    // contribution, but the accel feed-forward is added afterwards).
    if (iq >  g->iq_limit) iq =  g->iq_limit;
    if (iq < -g->iq_limit) iq = -g->iq_limit;

    return iq;
}

void servo_reset(uint32_t axis) {
    if (axis < s_axis_count) {
        pi_reset_integral(&s_vel_pi[axis]);
    }
}

int servo_set_gains(uint32_t axis, const servo_gains_t *gains) {
    if (axis >= s_axis_count || gains == 0) {
        return -1;
    }
    s_gains[axis] = *gains;
    // Update the PI's gains/limit in place, preserving the integrator so a
    // live re-tune doesn't cause a torque discontinuity.
    s_vel_pi[axis].kp        = gains->kp_vel;
    s_vel_pi[axis].ki        = gains->ki_vel;
    s_vel_pi[axis].out_limit = gains->iq_limit;
    return 0;
}
