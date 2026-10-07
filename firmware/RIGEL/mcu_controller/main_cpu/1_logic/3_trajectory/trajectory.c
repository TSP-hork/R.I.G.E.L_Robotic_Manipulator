#include "trajectory.h"
#include "interpolator.h"   // internal implementation — NOT exposed to consumers

// ============================================================================
// trajectory.c — thin façade over the interpolator implementation.
//
// The public handle traj_t is an opaque byte buffer. Internally it stores one
// interp_path_t. This file is the ONE place that bridges the public contract
// to the current implementation; replacing the implementation means editing
// only this file plus interpolator.c / s_curve.c — trajectory.h stays fixed,
// and therefore so does every consumer.
// ============================================================================

// The opaque buffer must be large enough to hold the real implementation
// struct. If a future implementation grows past TRAJ_STORAGE_BYTES, this
// fails the build with a clear message instead of overflowing at runtime.
#ifdef __cplusplus
static_assert(sizeof(interp_path_t) <= sizeof(traj_t),
              "traj_t storage too small — increase TRAJ_STORAGE_BYTES");
static_assert(TRAJ_MAX_AXES <= INTERP_MAX_AXES,
              "TRAJ_MAX_AXES exceeds interpolator capacity");
#else
_Static_assert(sizeof(interp_path_t) <= sizeof(traj_t),
               "traj_t storage too small — increase TRAJ_STORAGE_BYTES");
_Static_assert(TRAJ_MAX_AXES <= INTERP_MAX_AXES,
               "TRAJ_MAX_AXES exceeds interpolator capacity");
#endif

// Reinterpret the opaque storage as the implementation type. Safe because
// traj_t reserves >= sizeof(interp_path_t) bytes and is 8-byte aligned.
static inline interp_path_t *as_path(traj_t *t) {
    return (interp_path_t *)t->_bytes;
}
static inline const interp_path_t *as_path_const(const traj_t *t) {
    return (const interp_path_t *)t->_bytes;
}

int traj_plan_path(traj_t *t,
                   const float *waypoints, uint32_t waypoint_count,
                   const traj_limits_t *limits, uint32_t axis_count) {
    if (waypoint_count > TRAJ_MAX_WAYPOINTS) {
        return -1;
    }
    // traj_limits_t and interp_limits_t are layout-identical (three floats in
    // the same order). Reinterpret rather than copy — keeps the façade free.
    return interp_plan_path(as_path(t), waypoints, waypoint_count,
                            (const interp_limits_t *)limits, axis_count);
}

int traj_plan_move(traj_t *t,
                   const float *start, const float *target,
                   const traj_limits_t *limits, uint32_t axis_count) {
    // A single move is a 2-waypoint path: [start..][target..] laid out
    // contiguously. Build that layout on the stack, then reuse the path
    // planner so there is exactly one planning code path to maintain.
    float wp[2 * TRAJ_MAX_AXES];
    if (axis_count == 0 || axis_count > TRAJ_MAX_AXES) {
        return -1;
    }
    for (uint32_t i = 0; i < axis_count; i++) {
        wp[i]              = start[i];
        wp[axis_count + i] = target[i];
    }
    return traj_plan_path(t, wp, 2, limits, axis_count);
}

void traj_sample(const traj_t *t, float time_s, traj_state_t *out) {
    // traj_state_t is {pos,vel,acc}; the interpolator fills three parallel
    // arrays. Gather into locals, then scatter into the struct array so the
    // public API stays "one struct per axis" regardless of internal layout.
    const interp_path_t *path = as_path_const(t);
    float pos[TRAJ_MAX_AXES], vel[TRAJ_MAX_AXES], acc[TRAJ_MAX_AXES];

    interp_sample_path(path, time_s, pos, vel, acc);

    for (uint32_t i = 0; i < path->axis_count; i++) {
        out[i].pos = pos[i];
        out[i].vel = vel[i];
        out[i].acc = acc[i];
    }
}

float traj_duration(const traj_t *t) {
    return interp_path_duration(as_path_const(t));
}

uint32_t traj_axis_count(const traj_t *t) {
    return as_path_const(t)->axis_count;
}
