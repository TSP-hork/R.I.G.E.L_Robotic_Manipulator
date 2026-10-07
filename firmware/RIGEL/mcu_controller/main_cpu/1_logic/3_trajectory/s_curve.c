#include "s_curve.h"
#include <math.h>

// ============================================================================
// Jerk-limited (double-S) profile planner.
//
// The planner works on the unsigned distance D = |end - start| and reasons
// about a single symmetric rest-to-rest move. Direction is folded back in at
// the very end by signing the jerk of every segment, which propagates the
// correct sign through the integrated velocity, acceleration, and position.
//
// Three nested feasibility questions decide the shape of the profile:
//
//   1. Is v_max reached?  (is the move long enough to cruise?)
//   2. If not, is a_max still reached?  (trapezoidal vs. triangular accel)
//
// Each answer collapses one or more of the seven segments to zero duration.
// The segment table below is always seven entries; unused ones simply carry
// a duration of 0 and are skipped during sampling.
// ============================================================================

// Segment layout (index → meaning). Phases 4-6 mirror phases 0-2.
//   0: jerk +   (acceleration ramps 0 → +a_lim)
//   1: jerk 0   (acceleration held at +a_lim)          — collapses if a_max unreached
//   2: jerk -   (acceleration ramps +a_lim → 0)
//   3: jerk 0   (cruise at +v_lim, acceleration 0)     — collapses if v_max unreached
//   4: jerk -   (acceleration ramps 0 → -a_lim)
//   5: jerk 0   (acceleration held at -a_lim)          — collapses if a_max unreached
//   6: jerk +   (acceleration ramps -a_lim → 0)

// ----------------------------------------------------------------------------
// build_segments: fill the seven-segment table by forward-integrating the
// constant-jerk kinematics, given the three ramp durations and the signed
// jerk magnitude. Tj = jerk-ramp time, Tca = const-accel time, Tv = cruise.
// ----------------------------------------------------------------------------
static void build_segments(scurve_profile_t *p, float start,
                           float Tj, float Tca, float Tv, float jerk) {
    const float dur[7]  = { Tj, Tca, Tj, Tv, Tj, Tca, Tj };
    const float jsn[7]  = { jerk, 0.0f, -jerk, 0.0f, -jerk, 0.0f, jerk };

    float t0 = 0.0f;
    float p0 = start, v0 = 0.0f, a0 = 0.0f;

    for (int i = 0; i < 7; i++) {
        float dt = dur[i];
        float J  = jsn[i];

        p->seg_t0[i]   = t0;
        p->seg_dur[i]  = dt;
        p->seg_jerk[i] = J;
        p->seg_p0[i]   = p0;
        p->seg_v0[i]   = v0;
        p->seg_a0[i]   = a0;

        // Advance the integrator to the end of this segment (start of next).
        // a(dt) = a0 + J·dt
        // v(dt) = v0 + a0·dt + ½·J·dt²
        // p(dt) = p0 + v0·dt + ½·a0·dt² + ⅙·J·dt³
        float dt2 = dt * dt;
        p0 = p0 + v0 * dt + 0.5f * a0 * dt2 + (1.0f / 6.0f) * J * dt2 * dt;
        v0 = v0 + a0 * dt + 0.5f * J * dt2;
        a0 = a0 + J * dt;
        t0 += dt;
    }

    p->total_time = t0;
}

int scurve_plan(scurve_profile_t *p,
                float start, float end,
                float v_max, float a_max, float j_max) {
    // Reject non-physical limits — a profile with a zero or negative limit
    // has no meaning and would divide by zero below.
    if (v_max <= 0.0f || a_max <= 0.0f || j_max <= 0.0f) {
        return -1;
    }

    float delta = end - start;
    float dir   = (delta >= 0.0f) ? 1.0f : -1.0f;
    float D     = fabsf(delta);

    // Zero-length move: a valid profile that always samples to `start`.
    if (D == 0.0f) {
        build_segments(p, start, 0.0f, 0.0f, 0.0f, 0.0f);
        return 0;
    }

    float Tj, Tca, Tv;

    // ===== Step 1: assume v_max is reached; size the acceleration phase =====
    if (v_max * j_max >= a_max * a_max) {
        // a_max IS reached at cruise velocity → trapezoidal acceleration.
        Tj  = a_max / j_max;
        Tca = v_max / a_max - a_max / j_max;   // ≥ 0 by the condition above
    } else {
        // a_max is NOT reached even at v_max → triangular acceleration.
        Tj  = sqrtf(v_max / j_max);
        Tca = 0.0f;
    }
    float Ta = 2.0f * Tj + Tca;   // full acceleration-phase duration

    // ===== Step 2: is the move long enough to actually reach v_max? =====
    // Distance consumed by the acceleration + deceleration phases (each has
    // average velocity v_max/2) is v_max·Ta. Anything beyond that is cruise.
    if (D >= v_max * Ta) {
        Tv = (D - v_max * Ta) / v_max;   // cruise at v_max for the remainder
    } else {
        // v_max is NOT reached → no cruise; re-size the accel phase for D.
        Tv = 0.0f;

        // Threshold distance below which a_max is also not reached.
        // (Distance of a pure triangular-accel bump 0→a_max→0, doubled.)
        float D_amax = 2.0f * a_max * a_max * a_max / (j_max * j_max);

        if (D >= D_amax) {
            // a_max still reached → solve for the const-accel duration Tca.
            // With Tj = a_max/j_max fixed and v_peak = a_max·(Tj+Tca):
            //   D = v_peak · Ta = a_max·(Tj+Tca)·(2·Tj+Tca)
            // → a_max·Tca² + 3·a_max·Tj·Tca + (2·a_max·Tj² − D) = 0
            Tj = a_max / j_max;
            float A = a_max;
            float B = 3.0f * a_max * Tj;
            float C = 2.0f * a_max * Tj * Tj - D;
            float disc = B * B - 4.0f * A * C;
            if (disc < 0.0f) disc = 0.0f;   // guard float round-off
            Tca = (-B + sqrtf(disc)) / (2.0f * A);
            if (Tca < 0.0f) Tca = 0.0f;
        } else {
            // Neither v_max nor a_max reached → fully triangular accel.
            //   D = 2·j_max·Tj³  →  Tj = cbrt(D / (2·j_max))
            Tj  = cbrtf(D / (2.0f * j_max));
            Tca = 0.0f;
        }
    }

    build_segments(p, start, Tj, Tca, Tv, dir * j_max);
    return 0;
}

void scurve_scale_time(scurve_profile_t *p, float s) {
    // Only slowing down is physically valid — speeding up would breach the
    // limits the profile was planned against. Ignore s ≤ 1 (and the s == 1
    // no-op) to keep the call idempotent for the pacing axis.
    if (s <= 1.0f) {
        return;
    }

    float inv_s2 = 1.0f / (s * s);
    float inv_s3 = inv_s2 / s;

    for (int i = 0; i < 7; i++) {
        // Times stretch by s.
        p->seg_t0[i]  *= s;
        p->seg_dur[i] *= s;
        // Segment start velocity scales 1/s, start acceleration 1/s².
        // (Positions are unchanged — the path is identical.)
        p->seg_v0[i]  /= s;
        p->seg_a0[i]  *= inv_s2;
        // Jerk scales 1/s³.
        p->seg_jerk[i] *= inv_s3;
    }
    p->total_time *= s;
}

scurve_state_t scurve_sample(const scurve_profile_t *p, float t) {
    scurve_state_t s;

    // Clamp to the valid time window so out-of-range samples return the
    // resting start/end states rather than extrapolating off the profile.
    if (t <= 0.0f) {
        s.pos = p->seg_p0[0];
        s.vel = p->seg_v0[0];
        s.acc = p->seg_a0[0];
        return s;
    }
    if (t >= p->total_time) {
        t = p->total_time;
    }

    // Locate the active segment: the last one whose start time is ≤ t.
    // Zero-duration segments are transparent to this search.
    int idx = 0;
    for (int i = 0; i < 7; i++) {
        if (p->seg_dur[i] <= 0.0f) continue;
        if (t >= p->seg_t0[i]) idx = i;
    }

    // Local time within the segment, then a single cubic evaluation.
    float dt  = t - p->seg_t0[idx];
    float J   = p->seg_jerk[idx];
    float a0  = p->seg_a0[idx];
    float v0  = p->seg_v0[idx];
    float p0  = p->seg_p0[idx];
    float dt2 = dt * dt;

    s.acc = a0 + J * dt;
    s.vel = v0 + a0 * dt + 0.5f * J * dt2;
    s.pos = p0 + v0 * dt + 0.5f * a0 * dt2 + (1.0f / 6.0f) * J * dt2 * dt;
    return s;
}
