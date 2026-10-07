#include "seg_position3.h"

#include <math.h>   // sinf, cosf, sqrtf, atan2f, acosf, fabsf

// ============================================================================
// seg_position3.c — closed-form IK for the anthropomorphic (elbow) positioner.
//
// Arm (your base):
//   q1 base yaw  (vertical axis)  — chooses the vertical plane of the target,
//   q2 shoulder  (pitch)          — parallel-axis planar 2-link with…
//   q3 elbow     (pitch, parallel to q2).
//
// The DH rows filled by seg_position3_fill_dh() are, per axis:
//   axis0:  a=a1  alpha=+π/2  d=d1  theta0=0
//   axis1:  a=l2  alpha=0     d=0   theta0=0
//   axis2:  a=l3  alpha=0     d=0   theta0=0
//
// Composing them (see kinematics.c dh_transform) gives the tool position:
//   px = cos(q1)·(R + a1),  py = sin(q1)·(R + a1),  pz = d1 + Z
// where, in the shoulder's vertical plane,
//   R = l2·cos(q2) + l3·cos(q2+q3)      (horizontal reach from shoulder)
//   Z = l2·sin(q2) + l3·sin(q2+q3)      (vertical reach from shoulder)
//
// We invert exactly that, so kin_fk() and this solver agree bit-for-bit (what
// makes the round-trip test pass).
// ============================================================================

// Small epsilon for reachability / degeneracy tests (mm and unitless).
#define P3_EPS 1e-4f

// ----------------------------------------------------------------------------
// The actual closed-form inverse. Fills q[0..2].
// ----------------------------------------------------------------------------
static int position3_solve(const kin_segment_t *seg,
                           const kin_pose_t *target,
                           const float *seed,
                           float *q_out,
                           kin_pose_t *residual) {
    const seg_position3_geom_t *g = (const seg_position3_geom_t *)seg->ctx;

    // --- target tool point (mm), orientation is ignored by a positioner ---
    float px = kin_pose_x(target);
    float py = kin_pose_y(target);
    float pz = kin_pose_z(target);

    // --- q1: which vertical plane does the target lie in? ---
    // horizontal distance from the yaw axis:
    float horiz = sqrtf(px * px + py * py);
    float q1 = atan2f(py, px);

    // In-plane coordinates of the target relative to the SHOULDER:
    //   R measured outward from the shoulder, Z measured up from the shoulder.
    float R = horiz - g->a1;     // subtract the shoulder's horizontal offset
    float Z = pz - g->d1;        // subtract the base height

    // --- planar 2-link reachability: distance shoulder→target ---
    float D2 = R * R + Z * Z;
    float D  = sqrtf(D2);
    float reach_max = g->l2 + g->l3;
    float reach_min = fabsf(g->l2 - g->l3);
    if (D > reach_max + P3_EPS || D < reach_min - P3_EPS) {
        return -1;               // outside the annulus the 2-link arm can reach
    }

    // --- q3 from the law of cosines: D² = l2² + l3² + 2·l2·l3·cos(q3) ---
    float cos_q3 = (D2 - g->l2 * g->l2 - g->l3 * g->l3) /
                   (2.0f * g->l2 * g->l3);
    // Clamp against round-off so acosf stays in domain right at the boundary.
    if (cos_q3 >  1.0f) cos_q3 =  1.0f;
    if (cos_q3 < -1.0f) cos_q3 = -1.0f;
    float q3_mag = acosf(cos_q3);   // ∈ [0, π]

    // Two branches: elbow-down (+) and elbow-up (−). Pick the one whose q3 is
    // closest to the seed so the arm keeps its current elbow configuration
    // instead of snapping through a flip.
    float q3_a =  q3_mag;
    float q3_b = -q3_mag;
    float q3 = (fabsf(q3_a - seed[2]) <= fabsf(q3_b - seed[2])) ? q3_a : q3_b;

    // --- q2: aim the base of the 2-link, then correct for the elbow bend ---
    //   q2 = atan2(Z, R) − atan2(l3·sin q3, l2 + l3·cos q3)
    float s3 = sinf(q3), c3 = cosf(q3);
    float q2 = atan2f(Z, R) - atan2f(g->l3 * s3, g->l2 + g->l3 * c3);

    q_out[0] = q1;
    q_out[1] = q2;
    q_out[2] = q3;

    // --- residual for a following wrist segment (only if one exists) ---
    // A positioner controls the tool POINT, not its orientation, so it hands
    // the wrist the leftover orientation expressed in frame 3. Computing that
    // handoff is the wrist segment's concern; the working skeleton in
    // seg_wrist3.c defines it. Until a wrist is registered, `residual` is NULL
    // and there is nothing to pass on.
    if (residual) {
        // Pass the target through unchanged as a placeholder frame. The wrist
        // skeleton documents the correct  residual = inv(T_0_3) · target  form
        // and will replace this when it is implemented.
        *residual = *target;
    }
    return 0;
}

// ----------------------------------------------------------------------------
// DH rows that reproduce this geometry for the general FK.
// ----------------------------------------------------------------------------
void seg_position3_fill_dh(const seg_position3_geom_t *geom, kin_dh_t dh_out[3]) {
    // axis 0: base yaw, lifted d1, offset a1, twisted +90° so axes 2/3 pitch.
    dh_out[0].a = geom->a1;  dh_out[0].alpha = KIN_PI * 0.5f;
    dh_out[0].d = geom->d1;  dh_out[0].theta0 = 0.0f;
    // axis 1: upper arm, planar.
    dh_out[1].a = geom->l2;  dh_out[1].alpha = 0.0f;
    dh_out[1].d = 0.0f;      dh_out[1].theta0 = 0.0f;
    // axis 2: forearm, planar.
    dh_out[2].a = geom->l3;  dh_out[2].alpha = 0.0f;
    dh_out[2].d = 0.0f;      dh_out[2].theta0 = 0.0f;
}

// ----------------------------------------------------------------------------
// Segment factory.
// ----------------------------------------------------------------------------
int seg_position3_make(kin_segment_t *seg,
                       const seg_position3_geom_t *geom,
                       uint32_t first_axis) {
    if (seg == 0 || geom == 0) {
        return -1;
    }
    seg->dof        = 3;
    seg->first_axis = first_axis;
    seg->ctx        = geom;       // geometry stays with the caller (calibratable)
    seg->solve      = position3_solve;
    return 0;
}
