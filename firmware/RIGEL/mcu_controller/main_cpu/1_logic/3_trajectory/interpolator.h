#ifndef INTERPOLATOR_H
#define INTERPOLATOR_H

#include <stdint.h>
#include "s_curve.h"

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// INTERNAL IMPLEMENTATION HEADER — do NOT include from outside 3_trajectory/.
// The public entry point for the whole layer is "trajectory.h". Consumers
// (e.g. the servo loop) must speak to trajectory.h only; this header and
// s_curve.h are free to change as the implementation evolves.
// ============================================================================

// ============================================================================
// interpolator — multi-axis, multi-waypoint trajectory sequencing on top of
// the single-DOF jerk-limited s_curve profiles.
//
// Two responsibilities:
//
//   1. Time synchronization across axes. When several joints must move from
//      one waypoint to the next, each has its own natural duration (a big
//      move takes longer than a small one). The interpolator plans every
//      axis, finds the slowest, and time-stretches the faster ones so all
//      axes start and stop together. The result is a straight line in
//      joint space with smooth, coordinated motion — no axis finishes early
//      and waits.
//
//   2. Waypoint sequencing. A list of waypoints is executed one segment at a
//      time; sampling with a monotonically increasing time walks through the
//      whole path. Each segment is a rest-to-rest move (the arm comes to a
//      stop at every waypoint), which keeps the math simple and predictable.
//
// This is the (planned) 100 Hz trajectory layer: it produces the position /
// velocity / acceleration references that the (planned) 1 kHz servo loop
// tracks. It is pure portable logic — no hardware, desktop-testable.
//
// The maximum axis count is fixed at compile time (INTERP_MAX_AXES) to avoid
// dynamic allocation, matching the project's "no malloc" embedded policy.
// ============================================================================

#ifndef INTERP_MAX_AXES
#define INTERP_MAX_AXES 6   // Design target is a 6-axis arm (prototype uses 3)
#endif

// ----------------------------------------------------------------------------
// interp_limits_t: per-axis kinematic limits handed to the planner.
// ----------------------------------------------------------------------------
typedef struct {
    float v_max;  // Velocity limit     (units/s,   > 0)
    float a_max;  // Acceleration limit (units/s^2, > 0)
    float j_max;  // Jerk limit         (units/s^3, > 0)
} interp_limits_t;

// ----------------------------------------------------------------------------
// interp_move_t: a single time-synchronized move across all axes.
//
// Holds one planned (and time-scaled) s_curve per axis, all sharing the same
// duration. Produced by interp_plan_move(); sampled with interp_sample_move().
// ----------------------------------------------------------------------------
typedef struct {
    scurve_profile_t axis[INTERP_MAX_AXES];  // One profile per axis
    uint32_t         axis_count;              // Number of active axes
    float            duration;                // Common duration of the move (s)
} interp_move_t;

// ----------------------------------------------------------------------------
// interp_plan_move: plan a synchronized move from `start[]` to `target[]`.
//
//   start[], target[] — per-axis start/target positions (length axis_count)
//   limits[]          — per-axis kinematic limits       (length axis_count)
//   axis_count        — number of axes (1 .. INTERP_MAX_AXES)
//
// Every axis is planned independently, then the axes that finish sooner are
// time-stretched to the duration of the slowest axis, so all axes arrive at
// their targets simultaneously.
//
// Returns 0 on success, negative on bad arguments (axis_count out of range or
// any limit non-positive).
// ----------------------------------------------------------------------------
int interp_plan_move(interp_move_t *m,
                     const float *start, const float *target,
                     const interp_limits_t *limits, uint32_t axis_count);

// ----------------------------------------------------------------------------
// interp_sample_move: sample every axis of a planned move at time t (seconds).
//
//   out_pos, out_vel, out_acc — output arrays of length m->axis_count.
//     Any of the three may be NULL if that quantity is not needed.
//
// t is clamped to [0, duration] per axis, so the move holds at its start
// before t=0 and rests at its target after t=duration.
// ----------------------------------------------------------------------------
void interp_sample_move(const interp_move_t *m, float t,
                        float *out_pos, float *out_vel, float *out_acc);

// ============================================================================
// Multi-waypoint sequencing
// ============================================================================

// ----------------------------------------------------------------------------
// interp_path_t: a sequence of synchronized moves through a waypoint list.
//
// Waypoints are supplied as a flat, row-major array: waypoint k occupies
// elements [k*axis_count .. k*axis_count + axis_count). With N waypoints
// there are N-1 moves. The arm comes to rest at each interior waypoint.
// ----------------------------------------------------------------------------
#ifndef INTERP_MAX_MOVES
#define INTERP_MAX_MOVES 16   // Maximum segments in one planned path
#endif

typedef struct {
    interp_move_t move[INTERP_MAX_MOVES];  // One move between each waypoint pair
    float         move_t0[INTERP_MAX_MOVES]; // Cumulative start time of each move
    uint32_t      move_count;               // Number of moves (= waypoints - 1)
    uint32_t      axis_count;               // Axes per waypoint
    float         total_time;               // Total path duration (s)
} interp_path_t;

// ----------------------------------------------------------------------------
// interp_plan_path: plan a rest-to-rest path through `waypoint_count`
// waypoints (row-major, `axis_count` values each).
//
// Returns 0 on success, negative on bad arguments (fewer than 2 waypoints,
// too many moves for INTERP_MAX_MOVES, bad axis_count, or bad limits).
// ----------------------------------------------------------------------------
int interp_plan_path(interp_path_t *path,
                     const float *waypoints, uint32_t waypoint_count,
                     const interp_limits_t *limits, uint32_t axis_count);

// ----------------------------------------------------------------------------
// interp_sample_path: sample the whole path at time t (seconds from start).
// Locates the active move and delegates to interp_sample_move(). t is clamped
// to [0, total_time].
// ----------------------------------------------------------------------------
void interp_sample_path(const interp_path_t *path, float t,
                        float *out_pos, float *out_vel, float *out_acc);

// ----------------------------------------------------------------------------
// interp_path_duration: total execution time of a planned path (seconds).
// ----------------------------------------------------------------------------
static inline float interp_path_duration(const interp_path_t *path) {
    return path->total_time;
}

#ifdef __cplusplus
}
#endif

#endif
