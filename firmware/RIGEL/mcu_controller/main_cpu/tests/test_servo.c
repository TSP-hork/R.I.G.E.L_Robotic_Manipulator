// ============================================================================
// test_servo.c — desktop unit tests for the 1_servo layer.
//
// Verifies the servo cascade in closed loop against a simple inertial plant,
// driven by a real trajectory sample. Checks the guarantees the integrated
// system will depend on:
//
//   • output current is always within the configured iq_limit
//   • an invalid axis index commands zero torque (safe default)
//   • driving a moving trajectory target, the joint tracks it closely
//   • at the end of the move the joint settles to the target (zero error)
//   • servo_reset clears the integrator
//
// The plant model here is intentionally minimal (current → torque → accel →
// vel → pos) — enough to confirm the control law is correct and stable, not
// to model the real arm. Real tuning happens on hardware.
//
// Build & run on a desktop:
//
//   gcc -std=c11 -O2 -Wall -Wextra \
//       -I ../1_logic/1_servo -I ../2_utility/control/pid \
//       -I ../1_logic/3_trajectory \
//       test_servo.c \
//       ../1_logic/1_servo/servo.c \
//       ../1_logic/3_trajectory/s_curve.c \
//       ../1_logic/3_trajectory/interpolator.c \
//       ../1_logic/3_trajectory/trajectory.c \
//       -lm -o test_servo
//   ./test_servo
//
// Lives under tests/ — outside the CMake-globbed firmware dirs.
// ============================================================================

#include "servo.h"
#include "trajectory.h"

#include <stdio.h>
#include <math.h>

static int g_checks = 0, g_fails = 0;
#define CHECK(cond, msg) do {                                        \
    g_checks++;                                                      \
    if (!(cond)) { g_fails++;                                        \
        printf("  FAIL: %s (%s:%d)\n", (msg), __FILE__, __LINE__); } \
} while (0)
#define CHECK_NEAR(a,b,tol,msg) CHECK(fabsf((a)-(b))<=(tol), msg)

// ----------------------------------------------------------------------------
static void test_limits_and_guards(void) {
    printf("test_limits_and_guards\n");

    servo_gains_t g = {
        .kp_pos = 60.0f, .kp_vel = 0.8f, .ki_vel = 0.02f,
        .ff_vel = 1.0f,  .ff_acc = 0.004f,
        .vel_limit = 12.0f, .iq_limit = 10.0f,
    };
    CHECK(servo_init(&g, 1) == 0, "servo_init succeeds");

    // Huge position error must not produce current beyond iq_limit.
    servo_setpoint_t sp = { .pos = 1e6f, .vel = 0.0f, .acc = 0.0f };
    float iq = servo_step(0, &sp, 0.0f, 0.0f);
    CHECK(fabsf(iq) <= g.iq_limit + 1e-6f, "iq clamped to limit (positive)");

    sp.pos = -1e6f;
    iq = servo_step(0, &sp, 0.0f, 0.0f);
    CHECK(fabsf(iq) <= g.iq_limit + 1e-6f, "iq clamped to limit (negative)");

    // Invalid axis → zero torque.
    iq = servo_step(5, &sp, 0.0f, 0.0f);
    CHECK_NEAR(iq, 0.0f, 1e-9f, "invalid axis commands zero torque");

    // At the setpoint with zero velocity and a fresh integrator → ~zero output.
    servo_reset(0);
    servo_setpoint_t rest = { .pos = 0.0f, .vel = 0.0f, .acc = 0.0f };
    iq = servo_step(0, &rest, 0.0f, 0.0f);
    CHECK_NEAR(iq, 0.0f, 1e-6f, "at rest at target → zero current");
}

// ----------------------------------------------------------------------------
// Closed-loop tracking against an inertial plant driven by a trajectory.
static void test_closed_loop_tracking(void) {
    printf("test_closed_loop_tracking\n");

    // Plant: iq·Kt = torque; accel = (torque - friction·vel)/J.
    const float Kt = 0.05f;      // Nm/A
    const float J  = 2.0e-4f;    // kg·m^2
    const float fr = 1.0e-4f;    // viscous friction
    const float dt = 1.0e-3f;    // 1 kHz servo tick

    servo_gains_t g = {
        .kp_pos = 60.0f, .kp_vel = 0.8f, .ki_vel = 0.02f,
        .ff_vel = 1.0f,  .ff_acc = J / Kt,       // inertia feed-forward
        .vel_limit = 12.0f, .iq_limit = 10.0f,
    };
    servo_init(&g, 1);
    servo_reset(0);

    // One-axis trajectory: rest-to-rest move of 3.0 units.
    float start[1]  = { 0.0f };
    float target[1] = { 3.0f };
    traj_limits_t lim[1] = { { 8.0f, 40.0f, 400.0f } };
    traj_t traj;
    CHECK(traj_plan_move(&traj, start, target, lim, 1) == 0, "plan move ok");

    float T = traj_duration(&traj);
    CHECK(T > 0.0f, "trajectory has duration");

    float pos = 0.0f, vel = 0.0f;
    float max_err = 0.0f, max_iq = 0.0f;
    int steps = (int)((T + 0.5f) / dt);   // run 0.5 s past the end to settle

    for (int k = 0; k < steps; k++) {
        float t = (float)k * dt;

        traj_state_t st;
        traj_sample(&traj, t, &st);   // one axis → one struct

        servo_setpoint_t sp = { .pos = st.pos, .vel = st.vel, .acc = st.acc };
        float iq = servo_step(0, &sp, pos, vel);

        // Integrate the plant (semi-implicit Euler).
        float torque = iq * Kt - fr * vel;
        float acc    = torque / J;
        vel += acc * dt;
        pos += vel * dt;

        float err = st.pos - pos;
        if (fabsf(err) > max_err) max_err = fabsf(err);
        if (fabsf(iq)  > max_iq)  max_iq  = fabsf(iq);
        CHECK(fabsf(iq) <= g.iq_limit + 1e-6f, "iq within limit each tick");
    }

    printf("  max tracking err = %.4f, max iq = %.3f A, final pos = %.5f\n",
           max_err, max_iq, pos);
    CHECK(max_err < 0.05f, "tracks trajectory within 0.05 units");
    CHECK_NEAR(pos, 3.0f, 2e-3f, "settles at target after move");
}

// ----------------------------------------------------------------------------
int main(void) {
    printf("=== servo layer tests ===\n");
    test_limits_and_guards();
    test_closed_loop_tracking();

    printf("\n%d checks, %d failures\n", g_checks, g_fails);
    if (g_fails == 0) { printf("ALL TESTS PASSED\n"); return 0; }
    printf("TESTS FAILED\n");
    return 1;
}
