#ifndef S_CURVE_H
#define S_CURVE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// INTERNAL IMPLEMENTATION HEADER — do NOT include from outside 3_trajectory/.
// The public entry point for the whole layer is "trajectory.h". This file is
// a building block used by interpolator.c and may change freely.
// ============================================================================

// ============================================================================
// s_curve — jerk-limited (double-S / seven-segment) motion profile.
//
// Generates a smooth, rest-to-rest single-DOF trajectory that respects three
// kinematic limits simultaneously:
//
//   • maximum velocity      v_max   (units/s)
//   • maximum acceleration  a_max   (units/s^2)
//   • maximum jerk          j_max   (units/s^3)
//
// The profile is composed of up to seven phases:
//
//   1. jerk up      (acceleration rises 0 → +a_lim)      jerk = +j_max
//   2. const accel  (acceleration held at +a_lim)         jerk = 0
//   3. jerk down    (acceleration falls +a_lim → 0)       jerk = -j_max
//   4. cruise       (constant velocity v_lim)             accel = 0
//   5. jerk down    (acceleration falls 0 → -a_lim)       jerk = -j_max
//   6. const decel  (acceleration held at -a_lim)         jerk = 0
//   7. jerk up      (acceleration rises -a_lim → 0)       jerk = +j_max
//
// Because motion is jerk-limited, acceleration is continuous — there are no
// instantaneous torque steps that would excite mechanical resonance in the
// arm. This is the profile the (planned) servo loop follows as its position
// reference.
//
// Only the symmetric rest-to-rest case is handled: the profile starts and
// ends at zero velocity and zero acceleration. Phases 2 and/or 6 collapse to
// zero duration when a_max cannot be reached; phase 4 collapses when v_max
// cannot be reached. The planner detects and handles every such case.
//
// This module is pure portable logic — no hardware access, no global state.
// It can be compiled and unit-tested on a desktop PC.
// ============================================================================

// ----------------------------------------------------------------------------
// scurve_state_t: sampled kinematic state at a single instant in time.
// ----------------------------------------------------------------------------
typedef struct {
    float pos;  // Position (same units as start/end passed to scurve_plan)
    float vel;  // Velocity (units/s)
    float acc;  // Acceleration (units/s^2)
} scurve_state_t;

// ----------------------------------------------------------------------------
// scurve_profile_t: a fully planned profile.
//
// Internally stored as seven constant-jerk segments. Each segment carries its
// own start conditions so that sampling is a single cubic evaluation with no
// case analysis beyond locating the active segment. Treat the fields as
// opaque — use scurve_sample() / scurve_duration() to read the profile.
// ----------------------------------------------------------------------------
typedef struct {
    float seg_t0[7];    // Cumulative start time of each segment (s)
    float seg_dur[7];   // Duration of each segment (s); may be 0
    float seg_jerk[7];  // Constant jerk applied during each segment (units/s^3)
    float seg_p0[7];    // Position at the start of each segment
    float seg_v0[7];    // Velocity at the start of each segment
    float seg_a0[7];    // Acceleration at the start of each segment
    float total_time;   // Total profile duration (s)
} scurve_profile_t;

// ----------------------------------------------------------------------------
// scurve_plan: compute a jerk-limited profile from `start` to `end`.
//
//   start, end — absolute positions; direction is derived from their sign.
//   v_max      — velocity limit  (> 0)
//   a_max      — acceleration limit (> 0)
//   j_max      — jerk limit (> 0)
//
// Returns 0 on success, or a negative value if any limit is non-positive.
// A zero-length move (start == end) yields a valid profile of zero duration
// that always samples to the start position.
// ----------------------------------------------------------------------------
int scurve_plan(scurve_profile_t *p,
                float start, float end,
                float v_max, float a_max, float j_max);

// ----------------------------------------------------------------------------
// scurve_sample: evaluate the profile at time t (seconds from motion start).
//
// t is clamped to [0, total_time], so sampling before the start returns the
// start state (rest) and sampling after the end returns the end state (rest).
// ----------------------------------------------------------------------------
scurve_state_t scurve_sample(const scurve_profile_t *p, float t);

// ----------------------------------------------------------------------------
// scurve_duration: total time the profile takes to execute (seconds).
// ----------------------------------------------------------------------------
static inline float scurve_duration(const scurve_profile_t *p) {
    return p->total_time;
}

// ----------------------------------------------------------------------------
// scurve_scale_time: stretch a planned profile in time by factor `s` (s ≥ 1),
// in place. The path (start, end, intermediate positions) is preserved
// exactly; only the timing changes. Velocity scales by 1/s, acceleration by
// 1/s², jerk by 1/s³ — so the stretched profile still starts and ends at rest
// and never exceeds the (now lower) effective limits.
//
// Used by the multi-axis interpolator to slow the faster axes down so every
// axis reaches its target at the same instant. s < 1 (speeding up) is
// rejected because it would violate the original kinematic limits; such calls
// leave the profile unchanged.
// ----------------------------------------------------------------------------
void scurve_scale_time(scurve_profile_t *p, float s);

#ifdef __cplusplus
}
#endif

#endif
