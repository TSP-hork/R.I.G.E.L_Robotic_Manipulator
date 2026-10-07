#ifndef SEG_POSITION3_H
#define SEG_POSITION3_H

#include "kinematics.h"

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// seg_position3 — WORKING closed-form IK segment for a 3-axis positioner.
//
// This is YOUR arm's base: an anthropomorphic ("elbow") manipulator —
//   axis 1  base yaw    : rotates the whole arm about the vertical (your ОПУ),
//   axis 2  shoulder    : pitch in the vertical plane,
//   axis 3  elbow       : pitch, axis PARALLEL to the shoulder (your clevis).
//
// Because axes 2 and 3 are parallel and axis 1 is perpendicular to them, the
// position problem splits cleanly:
//   • axis 1 picks the vertical plane the target lies in  (q1 = atan2(y,x)),
//   • axes 2,3 are a planar 2-link arm reaching the target within that plane
//     (law of cosines).
// This gives an exact solution in microseconds, with the two classic branches
// (elbow-up / elbow-down); we pick the one nearest the seed so motion is smooth.
//
// GEOMETRY AS DATA: the four lengths live in seg_position3_geom_t and can be
// overwritten by calibration. seg_position3_fill_dh() emits the matching DH
// rows so the general FK in kinematics.c describes the very same arm — keeping
// FK and this closed form consistent (what makes round-trip tests pass).
// ============================================================================

// ----------------------------------------------------------------------------
// seg_position3_geom_t: the arm's measurable lengths (mm). Nominal values come
// from CAD; calibration may replace them.
//
//        z
//        |            (elbow)      (tool point)
//        |             o------L3------o
//        |            /
//        |         L2/
//        |          /
//   d1   |     o---o (shoulder, raised d1 above base, offset a1 out)
//        |  a1
//    ____|__________________  base plane, axis-1 vertical
//
//   d1 : base height   — shoulder height above the base frame origin
//   a1 : shoulder reach— horizontal offset of the shoulder from the yaw axis
//        (often 0 if the shoulder sits on the axis)
//   l2 : upper arm     — shoulder→elbow length
//   l3 : forearm       — elbow→tool-point length
// ----------------------------------------------------------------------------
typedef struct {
    float d1;   // base height       (mm)
    float a1;   // shoulder reach    (mm)
    float l2;   // upper-arm length  (mm)
    float l3;   // forearm length    (mm)
} seg_position3_geom_t;

// Fill the 3 DH rows that make kin_fk() reproduce this geometry. Call after
// choosing geom; pass dh_out[0..2] into kin_chain_init() for axes 0..2.
void seg_position3_fill_dh(const seg_position3_geom_t *geom, kin_dh_t dh_out[3]);

// Build a segment that solves axes [first_axis .. first_axis+2] with this
// geometry. `geom` is stored by pointer (ctx) and must outlive the chain —
// keep it in the same static/stack storage as the chain. Returns 0 on success.
int seg_position3_make(kin_segment_t *seg,
                       const seg_position3_geom_t *geom,
                       uint32_t first_axis);

#ifdef __cplusplus
}
#endif

#endif
