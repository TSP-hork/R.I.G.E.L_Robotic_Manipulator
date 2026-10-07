#include "interpolator.h"

// ============================================================================
// Multi-axis interpolation and waypoint sequencing.
//
// The heavy lifting (jerk-limited profiling, time-scaling) lives in s_curve.
// This file only coordinates: it plans one profile per axis, aligns their
// durations, and stitches successive moves into a path.
// ============================================================================

int interp_plan_move(interp_move_t *m,
                     const float *start, const float *target,
                     const interp_limits_t *limits, uint32_t axis_count) {
    if (axis_count == 0 || axis_count > INTERP_MAX_AXES) {
        return -1;
    }

    // ===== Plan each axis independently and track the longest duration =====
    float max_dur = 0.0f;
    for (uint32_t i = 0; i < axis_count; i++) {
        int rc = scurve_plan(&m->axis[i], start[i], target[i],
                             limits[i].v_max, limits[i].a_max, limits[i].j_max);
        if (rc != 0) {
            return -2;  // propagate a bad-limit error from the profiler
        }
        float d = scurve_duration(&m->axis[i]);
        if (d > max_dur) max_dur = d;
    }

    // ===== Time-synchronize: stretch every shorter axis to max_dur =====
    // The slowest axis paces the move; scurve_scale_time ignores s ≤ 1, so
    // the pacing axis (and any zero-length axis, duration 0) is left as-is.
    for (uint32_t i = 0; i < axis_count; i++) {
        float d = scurve_duration(&m->axis[i]);
        if (d > 0.0f && d < max_dur) {
            scurve_scale_time(&m->axis[i], max_dur / d);
        }
    }

    m->axis_count = axis_count;
    m->duration   = max_dur;
    return 0;
}

void interp_sample_move(const interp_move_t *m, float t,
                        float *out_pos, float *out_vel, float *out_acc) {
    for (uint32_t i = 0; i < m->axis_count; i++) {
        scurve_state_t s = scurve_sample(&m->axis[i], t);
        if (out_pos) out_pos[i] = s.pos;
        if (out_vel) out_vel[i] = s.vel;
        if (out_acc) out_acc[i] = s.acc;
    }
}

int interp_plan_path(interp_path_t *path,
                     const float *waypoints, uint32_t waypoint_count,
                     const interp_limits_t *limits, uint32_t axis_count) {
    if (waypoint_count < 2) {
        return -1;  // need at least a start and an end
    }
    if (axis_count == 0 || axis_count > INTERP_MAX_AXES) {
        return -1;
    }

    uint32_t moves = waypoint_count - 1;
    if (moves > INTERP_MAX_MOVES) {
        return -1;  // path too long for the fixed-size storage
    }

    // ===== Plan each waypoint-to-waypoint move and accumulate start times ==
    float t_accum = 0.0f;
    for (uint32_t k = 0; k < moves; k++) {
        const float *from = &waypoints[(k)     * axis_count];
        const float *to   = &waypoints[(k + 1) * axis_count];

        int rc = interp_plan_move(&path->move[k], from, to, limits, axis_count);
        if (rc != 0) {
            return -2;
        }
        path->move_t0[k] = t_accum;
        t_accum += path->move[k].duration;
    }

    path->move_count = moves;
    path->axis_count = axis_count;
    path->total_time = t_accum;
    return 0;
}

void interp_sample_path(const interp_path_t *path, float t,
                        float *out_pos, float *out_vel, float *out_acc) {
    // Clamp to the path window. An empty path (no moves) has nothing to
    // sample; leave the outputs untouched.
    if (path->move_count == 0) {
        return;
    }
    if (t < 0.0f) t = 0.0f;
    if (t > path->total_time) t = path->total_time;

    // Locate the active move: the last one whose start time is ≤ t. Because
    // every move has strictly positive duration (interior waypoints differ),
    // this lands on the correct segment; the final clamp handles t == total.
    uint32_t idx = 0;
    for (uint32_t k = 0; k < path->move_count; k++) {
        if (t >= path->move_t0[k]) idx = k;
    }

    float local_t = t - path->move_t0[idx];
    interp_sample_move(&path->move[idx], local_t, out_pos, out_vel, out_acc);
}
