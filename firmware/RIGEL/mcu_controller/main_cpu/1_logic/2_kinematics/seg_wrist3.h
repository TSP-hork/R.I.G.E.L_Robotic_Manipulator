#ifndef SEG_WRIST3_H
#define SEG_WRIST3_H

#include "kinematics.h"

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// seg_wrist3 — SKELETON IK segment for the wrist (orientation).  *** NOT YET
// IMPLEMENTED *** — the body is guarded by #error, exactly like the CORDIC
// sincos backend, because there is no physical wrist on the prototype to test
// against yet. It carries the full contract so you can flesh it out against
// hardware later without touching the core solver.
//
// ----------------------------------------------------------------------------
// WHAT IT WILL DO
// ----------------------------------------------------------------------------
// The positioner (seg_position3) places the tool POINT and hands the wrist the
// leftover ORIENTATION as a residual pose. The wrist's job is to choose its
// joint angles so the tool frame's orientation matches the target's.
//
// This segment is PARAMETRIC in how many DOF the wrist has, so the SAME code
// covers a 4-, 5-, or 6-axis arm — you just build it with dof = 1, 2, or 3:
//
//   dof = 1  →  roll only            → 4-axis arm  (spin tool about its axis)
//   dof = 2  →  pitch + roll         → 5-axis arm  (point + spin; no full yaw)
//   dof = 3  →  full ZYZ orientation → 6-axis arm  (spherical wrist, any
//                                                    orientation reachable)
//
// A 3-DOF spherical wrist (axes 4,5,6 intersecting at a point) is what makes
// the closed-form handoff exact: position and orientation decouple (Pieper).
// With 1 or 2 DOF the wrist can only realise a subset of orientations — the
// solver should match what it can and report the rest as residual error.
//
// ----------------------------------------------------------------------------
// THE RESIDUAL HANDOFF (the "mini-contract" between segments)
// ----------------------------------------------------------------------------
// When implemented, the wrist receives `target` already expressed such that the
// orientation it must produce is  R_wrist = R_0_3ᵀ · R_target , where R_0_3 is
// the orientation the positioner established at the wrist base. In practice the
// dispatcher gives the wrist the residual pose the positioner emits, so the
// wrist only reads the rotation part and extracts its Euler-like angles from it.
// ============================================================================

// wrist geometry placeholder — fill with the real link offsets when building
// the physical wrist (e.g. d4/d6 along the tool axis for a spherical wrist).
typedef struct {
    uint32_t dof;   // 1, 2, or 3 — how many orientation DOF this wrist has
    float    d_tool; // tool-length offset along the final axis (mm), if any
} seg_wrist3_geom_t;

// Build the wrist segment. `first_axis` normally follows the positioner (=3).
// Returns 0 on success. (Currently the solver body #errors at compile time —
// this factory is provided so the wiring is ready.)
int seg_wrist3_make(kin_segment_t *seg,
                    const seg_wrist3_geom_t *geom,
                    uint32_t first_axis);

#ifdef __cplusplus
}
#endif

#endif
