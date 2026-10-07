// ============================================================================
// test_trajectory.c — desktop unit tests for the 3_trajectory logic layer.
//
// These tests exercise s_curve and interpolator with NO hardware. They verify
// the guarantees the servo loop will depend on:
//
//   • the profile reaches the commanded endpoint
//   • it starts and ends at rest (zero velocity AND zero acceleration)
//   • velocity, acceleration, and jerk never exceed their limits
//   • position is monotonic for a single move and continuous across samples
//   • multi-axis moves are time-synchronized (all axes finish together)
//   • a multi-waypoint path passes through every waypoint
//
// Build & run on a desktop (the module is portable C11):
//
//   gcc -std=c11 -O2 -Wall -Wextra -I ../1_logic/3_trajectory \
//       test_trajectory.c \
//       ../1_logic/3_trajectory/s_curve.c \
//       ../1_logic/3_trajectory/interpolator.c \
//       ../1_logic/3_trajectory/trajectory.c \
//       -lm -o test_trajectory
//   ./test_trajectory
//
// This file lives under tests/ — outside the directories CMake globs for the
// firmware build — so it never ends up on the MCU.
// ============================================================================

#include "s_curve.h"
#include "interpolator.h"
#include "trajectory.h"   // public façade — the contract the servo loop uses

#include <stdio.h>
#include <math.h>
#include <stdlib.h>

// ---- tiny test framework ---------------------------------------------------
static int g_checks = 0;
static int g_fails  = 0;

#define CHECK(cond, msg) do {                                             \
    g_checks++;                                                           \
    if (!(cond)) {                                                        \
        g_fails++;                                                        \
        printf("  FAIL: %s  (%s:%d)\n", (msg), __FILE__, __LINE__);       \
    }                                                                     \
} while (0)

#define CHECK_NEAR(a, b, tol, msg) \
    CHECK(fabsf((a) - (b)) <= (tol), msg)

// Numerical tolerance. Limits are checked with a small relative margin to
// absorb float round-off in the segment integration.
#define LIM_MARGIN 1.003f   // allow 0.3% overshoot from accumulated rounding

// ----------------------------------------------------------------------------
// Sweep a single profile at fine time resolution and assert every invariant.
// Returns the achieved end position via *end_pos.
// ----------------------------------------------------------------------------
static void sweep_profile(const scurve_profile_t *p,
                          float start, float target,
                          float v_max, float a_max, float j_max) {
    float T = scurve_duration(p);
    int   N = 2000;
    float dt = (T > 0.0f) ? T / (float)N : 0.0f;

    float prev_pos = start;
    float prev_vel = 0.0f;
    float prev_acc = 0.0f;
    float dir = (target >= start) ? 1.0f : -1.0f;

    for (int i = 0; i <= N; i++) {
        float t = dt * (float)i;
        scurve_state_t s = scurve_sample(p, t);

        // --- limits ---
        CHECK(fabsf(s.vel) <= v_max * LIM_MARGIN, "velocity within v_max");
        CHECK(fabsf(s.acc) <= a_max * LIM_MARGIN, "acceleration within a_max");

        // --- monotonic position for a single rest-to-rest move ---
        // (velocity never changes sign, so position advances one direction)
        if (i > 0) {
            float dpos = (s.pos - prev_pos) * dir;
            CHECK(dpos >= -1e-4f, "position monotonic in move direction");

            // numeric jerk estimate stays bounded
            if (dt > 0.0f) {
                float jerk_est = (s.acc - prev_acc) / dt;
                CHECK(fabsf(jerk_est) <= j_max * 1.1f + 1e-3f,
                      "jerk within j_max");
            }
        }
        prev_pos = s.pos;
        prev_vel = s.vel;
        prev_acc = s.acc;
    }
    (void)prev_vel;

    // --- boundary conditions ---
    scurve_state_t s0 = scurve_sample(p, 0.0f);
    scurve_state_t sT = scurve_sample(p, T);
    CHECK_NEAR(s0.pos, start,  1e-4f, "starts at start position");
    CHECK_NEAR(s0.vel, 0.0f,   1e-4f, "starts at rest (v=0)");
    CHECK_NEAR(s0.acc, 0.0f,   1e-4f, "starts at rest (a=0)");
    CHECK_NEAR(sT.pos, target, 1e-3f, "reaches target position");
    CHECK_NEAR(sT.vel, 0.0f,   1e-3f, "ends at rest (v=0)");
    CHECK_NEAR(sT.acc, 0.0f,   1e-3f, "ends at rest (a=0)");
}

// ----------------------------------------------------------------------------
static void test_scurve_cases(void) {
    printf("test_scurve_cases\n");

    struct { float start, target, v, a, j; const char *name; } cases[] = {
        // Long move: reaches v_max (has a cruise phase)
        { 0.0f,  100.0f, 20.0f, 40.0f, 200.0f, "long / cruise" },
        // Medium move: reaches a_max but not v_max
        { 0.0f,   10.0f, 50.0f, 40.0f, 200.0f, "medium / no cruise" },
        // Short move: reaches neither v_max nor a_max (fully triangular jerk)
        { 0.0f,    0.2f, 50.0f, 40.0f, 200.0f, "short / triangular" },
        // Negative direction
        { 5.0f,  -95.0f, 20.0f, 40.0f, 200.0f, "negative direction" },
        // Non-zero start offset
        { 3.3f,   77.7f, 10.0f, 25.0f, 100.0f, "offset start" },
    };

    for (size_t c = 0; c < sizeof(cases) / sizeof(cases[0]); c++) {
        scurve_profile_t p;
        int rc = scurve_plan(&p, cases[c].start, cases[c].target,
                             cases[c].v, cases[c].a, cases[c].j);
        CHECK(rc == 0, "scurve_plan succeeds");
        CHECK(scurve_duration(&p) > 0.0f, "duration positive");
        printf("  [%s] T = %.4f s\n", cases[c].name, scurve_duration(&p));
        sweep_profile(&p, cases[c].start, cases[c].target,
                      cases[c].v, cases[c].a, cases[c].j);
    }
}

// ----------------------------------------------------------------------------
static void test_scurve_edge(void) {
    printf("test_scurve_edge\n");

    scurve_profile_t p;

    // Zero-length move: valid, zero duration, always at start.
    int rc = scurve_plan(&p, 42.0f, 42.0f, 10.0f, 10.0f, 100.0f);
    CHECK(rc == 0, "zero-length plan succeeds");
    CHECK_NEAR(scurve_duration(&p), 0.0f, 1e-6f, "zero-length duration is 0");
    scurve_state_t s = scurve_sample(&p, 0.0f);
    CHECK_NEAR(s.pos, 42.0f, 1e-6f, "zero-length samples to start");
    s = scurve_sample(&p, 5.0f);   // past the (zero) end
    CHECK_NEAR(s.pos, 42.0f, 1e-6f, "zero-length holds after end");

    // Bad limits are rejected.
    CHECK(scurve_plan(&p, 0.0f, 1.0f,  0.0f, 10.0f, 100.0f) < 0, "reject v=0");
    CHECK(scurve_plan(&p, 0.0f, 1.0f, 10.0f,  0.0f, 100.0f) < 0, "reject a=0");
    CHECK(scurve_plan(&p, 0.0f, 1.0f, 10.0f, 10.0f,  -1.0f) < 0, "reject j<0");

    // Out-of-range sampling clamps to the rest states.
    scurve_plan(&p, 0.0f, 10.0f, 5.0f, 20.0f, 100.0f);
    s = scurve_sample(&p, -3.0f);
    CHECK_NEAR(s.vel, 0.0f, 1e-5f, "sample before start is at rest");
    s = scurve_sample(&p, 1e6f);
    CHECK_NEAR(s.pos, 10.0f, 1e-3f, "sample far past end holds at target");
}

// ----------------------------------------------------------------------------
static void test_time_scale(void) {
    printf("test_time_scale\n");

    scurve_profile_t p;
    scurve_plan(&p, 0.0f, 50.0f, 20.0f, 40.0f, 200.0f);
    float T0 = scurve_duration(&p);

    scurve_scale_time(&p, 2.0f);
    CHECK_NEAR(scurve_duration(&p), 2.0f * T0, 1e-4f, "duration doubled");

    // Path preserved: endpoints unchanged, still rest-to-rest, peak velocity
    // halved (1/s). Sweep at the reduced limits.
    scurve_state_t sT = scurve_sample(&p, scurve_duration(&p));
    CHECK_NEAR(sT.pos, 50.0f, 1e-3f, "scaled profile still reaches target");
    CHECK_NEAR(sT.vel, 0.0f,  1e-3f, "scaled profile ends at rest");
    sweep_profile(&p, 0.0f, 50.0f, 20.0f / 2.0f, 40.0f / 4.0f, 200.0f / 8.0f);

    // s <= 1 is a no-op.
    float Tb = scurve_duration(&p);
    scurve_scale_time(&p, 0.5f);
    CHECK_NEAR(scurve_duration(&p), Tb, 1e-6f, "scale by <1 is ignored");
}

// ----------------------------------------------------------------------------
static void test_multiaxis_sync(void) {
    printf("test_multiaxis_sync\n");

    const uint32_t N = 3;
    float start[3]  = { 0.0f, 0.0f, 0.0f };
    float target[3] = { 100.0f, 5.0f, 30.0f };   // very different distances
    interp_limits_t lim[3] = {
        { 20.0f, 40.0f, 200.0f },
        { 20.0f, 40.0f, 200.0f },
        { 20.0f, 40.0f, 200.0f },
    };

    interp_move_t m;
    int rc = interp_plan_move(&m, start, target, lim, N);
    CHECK(rc == 0, "interp_plan_move succeeds");

    // Every axis must report the exact same duration as the move.
    for (uint32_t i = 0; i < N; i++) {
        CHECK_NEAR(scurve_duration(&m.axis[i]), m.duration, 1e-4f,
                   "axis synchronized to common duration");
    }

    // At t = duration every axis is at its target and at rest.
    float pos[3], vel[3], acc[3];
    interp_sample_move(&m, m.duration, pos, vel, acc);
    for (uint32_t i = 0; i < N; i++) {
        CHECK_NEAR(pos[i], target[i], 1e-3f, "axis reaches its target");
        CHECK_NEAR(vel[i], 0.0f,      1e-3f, "axis ends at rest (v)");
        CHECK_NEAR(acc[i], 0.0f,      1e-3f, "axis ends at rest (a)");
    }

    // Midway, no axis may exceed its own limits (the stretched axes run slower).
    int M = 500;
    for (int k = 0; k <= M; k++) {
        float t = m.duration * (float)k / (float)M;
        interp_sample_move(&m, t, pos, vel, acc);
        for (uint32_t i = 0; i < N; i++) {
            CHECK(fabsf(vel[i]) <= lim[i].v_max * LIM_MARGIN, "sync vel limit");
            CHECK(fabsf(acc[i]) <= lim[i].a_max * LIM_MARGIN, "sync acc limit");
        }
    }
}

// ----------------------------------------------------------------------------
static void test_path(void) {
    printf("test_path\n");

    const uint32_t N = 2;   // 2 axes
    // 4 waypoints → 3 moves. Row-major: {a0,a1} per waypoint.
    float wp[] = {
        0.0f,   0.0f,
        10.0f,  5.0f,
        10.0f, 20.0f,
        -5.0f,  0.0f,
    };
    uint32_t wp_count = 4;
    interp_limits_t lim[2] = {
        { 15.0f, 30.0f, 150.0f },
        { 15.0f, 30.0f, 150.0f },
    };

    interp_path_t path;
    int rc = interp_plan_path(&path, wp, wp_count, lim, N);
    CHECK(rc == 0, "interp_plan_path succeeds");
    CHECK(path.move_count == wp_count - 1, "one move per waypoint pair");
    CHECK(interp_path_duration(&path) > 0.0f, "path duration positive");
    printf("  path total time = %.4f s over %u moves\n",
           interp_path_duration(&path), path.move_count);

    // The path must pass through every waypoint at the move boundaries, at rest.
    float pos[2], vel[2];
    float t = 0.0f;
    for (uint32_t k = 0; k < path.move_count; k++) {
        interp_sample_path(&path, t, pos, vel, NULL);
        for (uint32_t i = 0; i < N; i++) {
            CHECK_NEAR(pos[i], wp[k * N + i], 1e-3f, "at waypoint position");
            CHECK_NEAR(vel[i], 0.0f, 1e-3f, "at rest at waypoint");
        }
        t += path.move[k].duration;
    }
    // Final waypoint.
    interp_sample_path(&path, interp_path_duration(&path), pos, vel, NULL);
    for (uint32_t i = 0; i < N; i++) {
        CHECK_NEAR(pos[i], wp[(wp_count - 1) * N + i], 1e-3f, "at final waypoint");
    }

    // Continuity: no position jump between adjacent samples.
    float T = interp_path_duration(&path);
    int   S = 4000;
    float prev[2] = { 0.0f, 0.0f };
    for (int s = 0; s <= S; s++) {
        float tt = T * (float)s / (float)S;
        interp_sample_path(&path, tt, pos, NULL, NULL);
        if (s > 0) {
            for (uint32_t i = 0; i < N; i++) {
                CHECK(fabsf(pos[i] - prev[i]) < 1.0f, "path position continuous");
            }
        }
        prev[0] = pos[0];
        prev[1] = pos[1];
    }

    // Bad-argument rejection.
    CHECK(interp_plan_path(&path, wp, 1, lim, N) < 0, "reject <2 waypoints");
    CHECK(interp_plan_path(&path, wp, 4, lim, 0) < 0, "reject axis_count 0");
}

// ----------------------------------------------------------------------------
// Exercise the PUBLIC façade (trajectory.h) exactly as the servo loop will:
// opaque handle, plan a path, sample per-axis structs. Verifies the contract,
// not the internals.
static void test_facade(void) {
    printf("test_facade\n");

    const uint32_t N = 3;
    float wp[] = {
        0.0f,  0.0f,  0.0f,
        10.0f, 5.0f,  30.0f,
        -5.0f, 5.0f,  0.0f,
    };
    traj_limits_t lim[3] = {
        { 15.0f, 30.0f, 150.0f },
        { 15.0f, 30.0f, 150.0f },
        { 15.0f, 30.0f, 150.0f },
    };

    traj_t traj;   // opaque, stack-allocated — no malloc, no internal fields
    int rc = traj_plan_path(&traj, wp, 3, lim, N);
    CHECK(rc == 0, "traj_plan_path succeeds");
    CHECK(traj_axis_count(&traj) == N, "axis count reported");
    CHECK(traj_duration(&traj) > 0.0f, "duration positive");

    // Sample the way the servo loop would: fixed dt, per-axis struct output.
    traj_state_t st[3];
    traj_sample(&traj, 0.0f, st);
    for (uint32_t i = 0; i < N; i++) {
        CHECK_NEAR(st[i].pos, wp[i], 1e-3f, "facade starts at first waypoint");
        CHECK_NEAR(st[i].vel, 0.0f,  1e-3f, "facade starts at rest");
    }
    traj_sample(&traj, traj_duration(&traj), st);
    for (uint32_t i = 0; i < N; i++) {
        CHECK_NEAR(st[i].pos, wp[2 * N + i], 1e-3f, "facade ends at last waypoint");
        CHECK_NEAR(st[i].vel, 0.0f, 1e-3f, "facade ends at rest");
    }

    // Single-move convenience wrapper.
    float s0[2] = { 0.0f, 0.0f }, s1[2] = { 20.0f, -10.0f };
    traj_limits_t l2[2] = { { 10.0f, 20.0f, 100.0f }, { 10.0f, 20.0f, 100.0f } };
    rc = traj_plan_move(&traj, s0, s1, l2, 2);
    CHECK(rc == 0, "traj_plan_move succeeds");
    traj_sample(&traj, traj_duration(&traj), st);
    CHECK_NEAR(st[0].pos, 20.0f,  1e-3f, "move axis0 reaches target");
    CHECK_NEAR(st[1].pos, -10.0f, 1e-3f, "move axis1 reaches target");
}

// ----------------------------------------------------------------------------
int main(void) {
    printf("=== trajectory layer tests ===\n");
    test_scurve_cases();
    test_scurve_edge();
    test_time_scale();
    test_multiaxis_sync();
    test_path();
    test_facade();

    printf("\n%d checks, %d failures\n", g_checks, g_fails);
    if (g_fails == 0) {
        printf("ALL TESTS PASSED\n");
        return 0;
    }
    printf("TESTS FAILED\n");
    return 1;
}
