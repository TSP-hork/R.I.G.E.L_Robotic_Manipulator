// ============================================================================
// test_kinematics.c — desktop unit tests for the 2_kinematics logic layer.
//
// Exercises the general FK, the closed-form 3-axis positioner, the segment
// dispatch, and the Jacobian with NO hardware. It verifies the guarantees the
// motion stack will depend on:
//
//   • FK is consistent with the closed-form geometry (a known pose lands where
//     hand calculation says it should)
//   • IK is the true inverse of FK: FK(IK(pose)) == pose  (round-trip) across a
//     grid of reachable targets
//   • targets outside the workspace are reported unreachable (no bogus angles)
//   • the elbow branch follows the seed (elbow-up stays up, elbow-down stays
//     down) instead of flipping
//   • the deg/rad convenience converters round-trip
//   • the analytic Jacobian matches a finite-difference Jacobian of FK
//   • manipulability collapses toward 0 as the arm stretches to a singularity
//
// Build & run on a desktop (portable C11):
//
//   gcc -std=c11 -O2 -Wall -Wextra -I ../1_logic/2_kinematics \
//       test_kinematics.c \
//       ../1_logic/2_kinematics/kinematics.c \
//       ../1_logic/2_kinematics/seg_position3.c \
//       ../1_logic/2_kinematics/seg_wrist3.c \
//       -lm -o test_kinematics
//   ./test_kinematics
//
// This file lives under tests/ — outside the directories CMake globs for the
// firmware build — so it never ends up on the MCU.
// ============================================================================

#include "kinematics.h"
#include "seg_position3.h"

#include <stdio.h>
#include <math.h>

// ---- tiny test framework ---------------------------------------------------
static int g_checks = 0;
static int g_fails  = 0;

#define CHECK(cond, msg) do {                                             \
    g_checks++;                                                           \
    if (!(cond)) { g_fails++; printf("  FAIL: %s\n", (msg)); }            \
} while (0)

#define CHECK_NEAR(a, b, tol, msg) do {                                   \
    g_checks++;                                                           \
    float _d = (float)fabs((double)(a) - (double)(b));                    \
    if (_d > (tol)) { g_fails++;                                          \
        printf("  FAIL: %s  (|%.6g - %.6g| = %.3g > %.3g)\n",             \
               (msg), (double)(a), (double)(b), (double)_d, (double)(tol)); } \
} while (0)

// ---- a concrete test arm (arbitrary but plausible lengths in mm) -----------
static seg_position3_geom_t g_geom = {
    .d1 = 150.0f,   // base height
    .a1 =  40.0f,   // shoulder horizontal offset
    .l2 = 250.0f,   // upper arm
    .l3 = 220.0f,   // forearm
};

// Build a 3-axis chain (positioner only) around g_geom.
static void build_chain(kin_chain_t *chain) {
    kin_dh_t dh[3];
    seg_position3_fill_dh(&g_geom, dh);

    // Wide-open soft limits so limit-clamping never masks a maths error here.
    kin_joint_limits_t lim[3];
    for (int i = 0; i < 3; i++) { lim[i].min = -KIN_PI * 2; lim[i].max = KIN_PI * 2; }

    int rc = kin_chain_init(chain, dh, lim, 3);
    if (rc != 0) { printf("  FATAL: kin_chain_init rc=%d\n", rc); }

    kin_segment_t seg;
    rc = seg_position3_make(&seg, &g_geom, 0);
    if (rc != 0) { printf("  FATAL: seg_position3_make rc=%d\n", rc); }
    rc = kin_chain_add_segment(chain, &seg);
    if (rc != 0) { printf("  FATAL: kin_chain_add_segment rc=%d\n", rc); }
}

// ---- tests -----------------------------------------------------------------

// FK of the all-zero pose should match a direct hand calculation.
// At q = {0,0,0}: the arm is stretched straight out along +X in the base plane.
//   horiz = a1 + l2 + l3,  x = horiz, y = 0, z = d1
static void test_fk_known_pose(void) {
    printf("test_fk_known_pose\n");
    kin_chain_t chain; build_chain(&chain);

    float q[3] = {0.0f, 0.0f, 0.0f};
    kin_pose_t p;
    CHECK(kin_fk(&chain, q, &p) == 0, "kin_fk returns 0");

    float expect_x = g_geom.a1 + g_geom.l2 + g_geom.l3;
    CHECK_NEAR(kin_pose_x(&p), expect_x, 1e-2f, "x at zero pose");
    CHECK_NEAR(kin_pose_y(&p), 0.0f,     1e-2f, "y at zero pose");
    CHECK_NEAR(kin_pose_z(&p), g_geom.d1, 1e-2f, "z at zero pose");
}

// Core guarantee: IK is the exact inverse of FK over a grid of joint configs.
// For each config we compute its tool point with FK, ask IK for joints that
// reach that point, then FK those joints and require the SAME point. (We compare
// tool points, not joint values, because the other elbow branch is also valid.)
static void test_roundtrip(void) {
    printf("test_roundtrip (FK(IK(pose)) == pose)\n");
    kin_chain_t chain; build_chain(&chain);

    const float samples[] = { -1.2f, -0.5f, 0.0f, 0.4f, 0.9f, 1.3f };
    const int   n = (int)(sizeof(samples) / sizeof(samples[0]));

    int tested = 0;
    for (int i = 0; i < n; i++) {
      for (int j = 0; j < n; j++) {
        for (int k = 0; k < n; k++) {
            float q[3]   = { samples[i], samples[j], samples[k] };
            kin_pose_t target;
            kin_fk(&chain, q, &target);

            // Seed with the same elbow sign so IK returns this branch.
            float seed[3] = { samples[i], samples[j], samples[k] };
            float qs[3];
            int rc = kin_ik(&chain, &target, seed, qs);
            CHECK(rc == 0, "IK solves an FK-generated (reachable) target");
            if (rc != 0) continue;

            kin_pose_t back;
            kin_fk(&chain, qs, &back);
            CHECK_NEAR(kin_pose_x(&back), kin_pose_x(&target), 1e-2f, "roundtrip x");
            CHECK_NEAR(kin_pose_y(&back), kin_pose_y(&target), 1e-2f, "roundtrip y");
            CHECK_NEAR(kin_pose_z(&back), kin_pose_z(&target), 1e-2f, "roundtrip z");
            tested++;
        }
      }
    }
    printf("  (%d reachable configs checked)\n", tested);
}

// Targets beyond the arm's reach must be rejected, not silently "solved".
static void test_unreachable(void) {
    printf("test_unreachable\n");
    kin_chain_t chain; build_chain(&chain);

    float seed[3] = {0,0,0};
    float q[3];

    // Way past full extension along X.
    float far = g_geom.a1 + g_geom.l2 + g_geom.l3 + 100.0f;
    kin_pose_t p_far = kin_pose_from_xyz(far, 0.0f, g_geom.d1);
    CHECK(kin_ik(&chain, &p_far, seed, q) != 0, "far target rejected");

    // Inside the dead zone (closer than |l2 - l3| from the shoulder), placed
    // right at the shoulder height on the yaw axis.
    kin_pose_t p_near = kin_pose_from_xyz(g_geom.a1, 0.0f, g_geom.d1);
    CHECK(kin_ik(&chain, &p_near, seed, q) != 0, "inner dead-zone target rejected");
}

// The elbow branch must follow the seed sign, so a moving arm keeps its
// configuration instead of snapping to the mirror solution.
static void test_elbow_branch(void) {
    printf("test_elbow_branch\n");
    kin_chain_t chain; build_chain(&chain);

    // A target reachable with either elbow sign (moderately bent).
    float q_ref[3] = { 0.3f, 0.6f, 0.8f };   // elbow-down (q3 > 0)
    kin_pose_t target; kin_fk(&chain, q_ref, &target);

    float q[3];
    float seed_down[3] = { 0.0f, 0.0f,  0.5f };  // hint: positive elbow
    CHECK(kin_ik(&chain, &target, seed_down, q) == 0, "IK ok (elbow-down seed)");
    CHECK(q[2] > 0.0f, "elbow-down seed → q3 > 0");

    float seed_up[3]   = { 0.0f, 0.0f, -0.5f };  // hint: negative elbow
    CHECK(kin_ik(&chain, &target, seed_up, q) == 0, "IK ok (elbow-up seed)");
    CHECK(q[2] < 0.0f, "elbow-up seed → q3 < 0");
}

// Convenience converters round-trip.
static void test_converters(void) {
    printf("test_converters\n");
    CHECK_NEAR(kin_rad(180.0f), KIN_PI, 1e-5f, "180 deg == pi rad");
    CHECK_NEAR(kin_deg(KIN_PI), 180.0f, 1e-3f, "pi rad == 180 deg");
    CHECK_NEAR(kin_deg(kin_rad(37.5f)), 37.5f, 1e-3f, "deg->rad->deg identity");
}

// Analytic Jacobian must match a finite-difference Jacobian of FK.
static void test_jacobian_vs_numeric(void) {
    printf("test_jacobian_vs_numeric\n");
    kin_chain_t chain; build_chain(&chain);

    float q[3] = { 0.4f, -0.3f, 0.7f };
    float J[6 * 3];
    CHECK(kin_jacobian(&chain, q, J) == 0, "kin_jacobian returns 0");

    // Numerical linear-velocity columns: d(tool position)/d(q_i).
    const float h = 1e-4f;
    for (int i = 0; i < 3; i++) {
        float qp[3] = { q[0], q[1], q[2] };
        float qm[3] = { q[0], q[1], q[2] };
        qp[i] += h; qm[i] -= h;
        kin_pose_t pp, pm;
        kin_fk(&chain, qp, &pp);
        kin_fk(&chain, qm, &pm);
        float dxdi = (kin_pose_x(&pp) - kin_pose_x(&pm)) / (2 * h);
        float dydi = (kin_pose_y(&pp) - kin_pose_y(&pm)) / (2 * h);
        float dzdi = (kin_pose_z(&pp) - kin_pose_z(&pm)) / (2 * h);
        CHECK_NEAR(J[0 * 3 + i], dxdi, 1e-1f, "Jacobian linear x column");
        CHECK_NEAR(J[1 * 3 + i], dydi, 1e-1f, "Jacobian linear y column");
        CHECK_NEAR(J[2 * 3 + i], dzdi, 1e-1f, "Jacobian linear z column");
    }
}

// Manipulability shrinks toward 0 as the arm approaches full extension.
static void test_manipulability_singularity(void) {
    printf("test_manipulability_singularity\n");
    kin_chain_t chain; build_chain(&chain);

    float q_bent[3]    = { 0.2f, 0.3f, 1.2f };   // comfortably bent elbow
    float q_straight[3]= { 0.2f, 0.3f, 0.02f };  // nearly straight → singular

    float w_bent    = kin_manipulability(&chain, q_bent);
    float w_straight= kin_manipulability(&chain, q_straight);
    CHECK(w_bent >= 0.0f && w_straight >= 0.0f, "manipulability is non-negative");
    CHECK(w_straight < w_bent, "straighter arm is closer to a singularity");
}

int main(void) {
    printf("=== kinematics tests ===\n");
    test_fk_known_pose();
    test_roundtrip();
    test_unreachable();
    test_elbow_branch();
    test_converters();
    test_jacobian_vs_numeric();
    test_manipulability_singularity();

    printf("\n%d checks, %d failures\n", g_checks, g_fails);
    return g_fails == 0 ? 0 : 1;
}
