#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// trajectory.h — PUBLIC CONTRACT for the trajectory-planning layer.
//
// This is the ONLY header the rest of the firmware (e.g. the servo loop)
// should include from 3_trajectory/. It is a stable façade: the consumer
// speaks to these types and functions and never sees how a trajectory is
// actually computed.
//
// The current implementation (trajectory.c → interpolator.c → s_curve.c)
// generates jerk-limited, time-synchronized, multi-waypoint motion. You can
// replace that implementation wholesale — B-splines, corner blending,
// look-ahead, anything — by rewriting the .c files. As long as the functions
// below keep their meaning, no consumer code changes: you literally keep the
// same call sites.
//
// Design rules that make this swappable:
//   • The handle `traj_t` is OPAQUE to the consumer — a fixed-size byte buffer.
//     You never read its fields, so you never depend on the internals.
//   • No dynamic allocation. The consumer owns the storage (stack or static).
//   • No hardware access. Pure portable logic — desktop-testable.
//
// Units are caller-defined (radians, mm, encoder ticks, …). The planner only
// requires that positions, velocities, accelerations and jerks are expressed
// in a consistent set (unit, unit/s, unit/s², unit/s³).
// ============================================================================

// Compile-time capacities. Override in the build if a larger arm/path is
// needed; kept as macros so all storage stays static (no malloc).
#ifndef TRAJ_MAX_AXES
#define TRAJ_MAX_AXES 6      // Design target is a 6-axis arm (prototype uses 3)
#endif
#ifndef TRAJ_MAX_WAYPOINTS
#define TRAJ_MAX_WAYPOINTS 17  // → up to 16 moves between them
#endif

// ----------------------------------------------------------------------------
// traj_state_t: sampled reference for ONE axis at one instant.
// This is what the servo loop consumes as its setpoint.
// ----------------------------------------------------------------------------
typedef struct {
    float pos;  // Position reference
    float vel;  // Velocity reference (feed-forward)
    float acc;  // Acceleration reference (feed-forward)
} traj_state_t;

// ----------------------------------------------------------------------------
// traj_limits_t: per-axis kinematic limits the planner must respect.
// ----------------------------------------------------------------------------
typedef struct {
    float v_max;  // Max velocity     (unit/s,   > 0)
    float a_max;  // Max acceleration (unit/s²,  > 0)
    float j_max;  // Max jerk         (unit/s³,  > 0)
} traj_limits_t;

// ----------------------------------------------------------------------------
// traj_t: OPAQUE planned-trajectory handle.
//
// Treat this as a black box. Allocate one (on the stack or statically), pass
// its address to traj_plan_path(), then sample it. Its size is fixed at
// compile time so no allocation is ever needed. The union with a byte array
// and doubles guarantees worst-case size and alignment regardless of what the
// active implementation stores inside.
// ----------------------------------------------------------------------------
#ifndef TRAJ_STORAGE_BYTES
// Sized to hold the current interpolator implementation with margin. If a
// future implementation needs more, bump this one number; the static_assert
// in trajectory.c will tell you if it is ever too small.
#define TRAJ_STORAGE_BYTES 4096
#endif

typedef union {
    uint8_t _bytes[TRAJ_STORAGE_BYTES];  // Reserve worst-case size
    double  _align;                      // Force 8-byte alignment
} traj_t;

// ----------------------------------------------------------------------------
// traj_plan_path: plan a rest-to-rest trajectory through a list of waypoints.
//
//   t          — handle to fill (caller-owned storage)
//   waypoints  — row-major array: waypoint k = waypoints[k*axis_count ...].
//                Consecutive waypoints define the moves. With N waypoints
//                there are N-1 moves; the arm comes to rest at each waypoint.
//   waypoint_count — number of waypoints (2 .. TRAJ_MAX_WAYPOINTS)
//   limits     — per-axis limits, length axis_count
//   axis_count — number of axes (1 .. TRAJ_MAX_AXES)
//
// Axes within a move are time-synchronized: every axis starts and finishes
// each move together, so the path is straight in joint space.
//
// Returns 0 on success, negative on invalid arguments.
// ----------------------------------------------------------------------------
int traj_plan_path(traj_t *t,
                   const float *waypoints, uint32_t waypoint_count,
                   const traj_limits_t *limits, uint32_t axis_count);

// ----------------------------------------------------------------------------
// traj_plan_move: convenience planner for a single move (start[] → target[]).
// Equivalent to a 2-waypoint path. Same return convention.
// ----------------------------------------------------------------------------
int traj_plan_move(traj_t *t,
                   const float *start, const float *target,
                   const traj_limits_t *limits, uint32_t axis_count);

// ----------------------------------------------------------------------------
// traj_sample: read the per-axis reference at time `time_s` (seconds from the
// start of the trajectory).
//
//   out — array of length axis_count (as planned). Element i receives the
//         pos/vel/acc reference for axis i.
//
// `time_s` is clamped to [0, duration]: sampling before the start holds the
// first waypoint at rest; sampling after the end holds the last waypoint at
// rest. This makes it safe to keep sampling past completion.
// ----------------------------------------------------------------------------
void traj_sample(const traj_t *t, float time_s, traj_state_t *out);

// ----------------------------------------------------------------------------
// traj_duration: total execution time of the planned trajectory (seconds).
// ----------------------------------------------------------------------------
float traj_duration(const traj_t *t);

// ----------------------------------------------------------------------------
// traj_axis_count: number of axes the trajectory was planned for.
// ----------------------------------------------------------------------------
uint32_t traj_axis_count(const traj_t *t);

#ifdef __cplusplus
}
#endif

#endif
