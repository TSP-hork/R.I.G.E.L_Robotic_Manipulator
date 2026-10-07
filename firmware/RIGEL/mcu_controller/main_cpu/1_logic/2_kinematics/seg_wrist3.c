#include "seg_wrist3.h"

// ============================================================================
// seg_wrist3.c — wrist orientation IK.  SKELETON ONLY.
//
// Enable by setting SEG_WRIST3_IMPL when you have a physical wrist to test
// against, then remove the #error and fill in the marked sections. Until then
// the guard keeps un-tested orientation maths out of the firmware — the same
// discipline as the CORDIC sincos backend.
// ============================================================================

#ifndef SEG_WRIST3_IMPL

// ---------------------------------------------------------------------------
// Provide a real (linkable) factory so the rest of the build/tests compile even
// while the solver is unimplemented. Registering this segment is what would be
// blocked — the solver pointer refuses to run rather than lie.
// ---------------------------------------------------------------------------
static int wrist3_solve_unimplemented(const kin_segment_t *seg,
                                      const kin_pose_t *target,
                                      const float *seed,
                                      float *q_out,
                                      kin_pose_t *residual) {
    (void)seg; (void)target; (void)seed; (void)q_out; (void)residual;
    return -100;   // "wrist not implemented" — kin_ik() propagates this
}

int seg_wrist3_make(kin_segment_t *seg,
                    const seg_wrist3_geom_t *geom,
                    uint32_t first_axis) {
    if (seg == 0 || geom == 0 || geom->dof == 0 || geom->dof > 3) {
        return -1;
    }
    seg->dof        = geom->dof;
    seg->first_axis = first_axis;
    seg->ctx        = geom;
    seg->solve      = wrist3_solve_unimplemented;
    return 0;
}

#else  // SEG_WRIST3_IMPL — the template you fill in against real hardware

#error "seg_wrist3 is a template. Define the ZYZ/roll extraction below, verify \
it against your physical wrist, then remove this #error. See the notes for the \
dof = 1 / 2 / 3 cases and the residual-orientation handoff."

// ---------------------------------------------------------------------------
// TEMPLATE — how the wrist solver is structured on a real arm.
//
// #include <math.h>
//
// static int wrist3_solve(const kin_segment_t *seg,
//                         const kin_pose_t *target,
//                         const float *seed,
//                         float *q_out,
//                         kin_pose_t *residual) {
//     const seg_wrist3_geom_t *g = (const seg_wrist3_geom_t *)seg->ctx;
//
//     // 1. Read the residual orientation R (3×3) out of `target->m`. This is
//     //    the orientation the wrist must produce (positioner already removed
//     //    its own contribution — see the handoff note in the header).
//
//     // 2. Extract the wrist angles from R:
//     //    dof == 3 (spherical wrist, ZYZ Euler):
//     //        q5 = atan2( sqrt(R02² + R12²), R22 );      // pitch
//     //        if (fabsf(sin(q5)) > eps) {
//     //            q4 = atan2( R12,  R02 );                // yaw before
//     //            q6 = atan2( R21, -R20 );                // roll after
//     //        } else {
//     //            // gimbal-lock branch: q5 ≈ 0 or π, q4+q6 coupled → fix q4,
//     //            // put the rest in q6, and flag low manipulability.
//     //        }
//     //        pick the branch (q5, and the ±π alternative) nearest `seed`.
//     //
//     //    dof == 2 (pitch+roll): solve q5,q6 to point+spin the tool axis;
//     //        the unreachable yaw shows up as residual error — report it.
//     //
//     //    dof == 1 (roll only): q(last) = roll of R about the tool axis.
//
//     // 3. Optional tool offset g->d_tool along the final axis if the tool
//     //    frame is displaced from the wrist centre.
//
//     // 4. residual (if non-NULL): orientation error left unrealised by a
//     //    reduced-DOF wrist; NULL/identity for a full 3-DOF spherical wrist.
//
//     return 0;   // or negative if the orientation is unreachable
// }
//
// int seg_wrist3_make(kin_segment_t *seg,
//                     const seg_wrist3_geom_t *geom,
//                     uint32_t first_axis) {
//     if (seg == 0 || geom == 0 || geom->dof == 0 || geom->dof > 3) return -1;
//     seg->dof = geom->dof; seg->first_axis = first_axis;
//     seg->ctx = geom;      seg->solve = wrist3_solve;
//     return 0;
// }

#endif // SEG_WRIST3_IMPL
