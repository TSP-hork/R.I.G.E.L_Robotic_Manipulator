#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// kinematics.h — PUBLIC CONTRACT for the kinematics layer.
//
// This is the ONLY header the rest of the firmware should include from
// 2_kinematics/. Like trajectory.h it is a stable façade: consumers speak to
// these types and functions and never see how the arm is actually solved.
//
//   trajectory / operator  →  [ KINEMATICS ]  →  servo (per joint)
//   Cartesian pose (mm,rad)    joint angles (rad)
//
// ----------------------------------------------------------------------------
// WHAT THIS LAYER DOES
// ----------------------------------------------------------------------------
//   Forward kinematics (FK): joint angles → tool pose. Easy, unique, always
//       works. Computed by composing per-axis transforms down the chain, so it
//       is fully general — ANY number of axes.
//
//   Inverse kinematics (IK): tool pose → joint angles. Hard, may have several
//       solutions or none. Solved by DELEGATION: the chain is a list of
//       SEGMENTS, each owning a coupled group of axes (a base positioner owns
//       3, a wrist owns 1..3). Each segment consumes the degrees of freedom it
//       is responsible for and hands the RESIDUAL pose to the next segment.
//
//       Why segments and not single axes? Because IK does not decompose per
//       axis — the axes inside one group hold each other and can only be solved
//       together (like a system of equations). The segment is the smallest unit
//       whose maths is still correct. Between segments the handoff is exact
//       (this is the spherical-wrist decomposition of Pieper, 1968).
//
//   Jacobian: maps joint velocities ↔ tool velocity, and its "manipulability"
//       reveals singularities (poses where the arm momentarily loses the
//       ability to move in some direction and IK blows up).
//
// ----------------------------------------------------------------------------
// EXTENSIBILITY — how you add axes later
// ----------------------------------------------------------------------------
//   A chain is an ordered list of segments. To go from a 3-axis positioner to a
//   4/5/6-axis arm you APPEND a wrist segment that declares how many DOF it
//   eats (1 = roll only, 2 = pitch+roll, 3 = full orientation). The core solver
//   does not change — it just walks whatever segments you registered. This is
//   the same "compose a chain of blocks" idea as the servo extension chain,
//   applied to kinematics at the granularity where the maths stays valid.
//
// ----------------------------------------------------------------------------
// UNITS
// ----------------------------------------------------------------------------
//   Canonical units INSIDE this layer are radians and millimetres — trig only
//   works in radians and one length unit keeps everything consistent. For
//   convenience at the edge (someone who prefers degrees) tiny inline
//   converters are provided below; they are the whole "translator" layer — the
//   maths never sees anything but rad/mm.
//
// Pure portable logic — no hardware, no dynamic allocation, desktop-testable.
// ============================================================================

// Compile-time capacity. Override in the build for a bigger arm; keeps all
// storage static (no malloc), matching the trajectory/servo layers.
#ifndef KIN_MAX_AXES
#define KIN_MAX_AXES 6      // design target is a 6-axis arm (prototype uses 3)
#endif
#ifndef KIN_MAX_SEGMENTS
#define KIN_MAX_SEGMENTS 4  // e.g. positioner(3) + wrist(3), room to grow
#endif

// ----------------------------------------------------------------------------
// kin_pose_t: a rigid tool pose = position + orientation, expressed as a 4×4
// homogeneous transform (column-vector convention, row-major storage):
//
//        | R00 R01 R02  Px |     R = 3×3 rotation (orientation)
//    T = | R10 R11 R12  Py |     P = translation (position, mm)
//        | R20 R21 R22  Pz |
//        |  0   0   0    1 |
//
// This same type is the "mini-contract" passed between segments: a segment
// receives a target pose, solves its own axes, and produces the residual pose
// the next segment must still achieve. Storing the full 4×4 (not just x/y/z)
// is what lets orientation flow down the chain to the wrist.
// ----------------------------------------------------------------------------
typedef struct {
    float m[4][4];
} kin_pose_t;

// ----------------------------------------------------------------------------
// kin_dh_t: one row of Denavit–Hartenberg parameters describing how axis i is
// attached to axis i-1. This is the GEOMETRY OF THE ARM AS DATA — it is passed
// into kin_chain_init() and can be OVERWRITTEN BY CALIBRATION at runtime, so
// the real (measured) arm can differ from the CAD model without touching code.
//
//   Standard DH, per joint i (all revolute here → theta is the variable):
//     a      link length      : offset along X_{i-1}     (mm)
//     alpha  link twist        : rotation about X_{i-1}   (rad)
//     d      link offset       : offset along Z_i         (mm)
//     theta0 joint angle zero  : constant added to the commanded angle (rad),
//                                i.e. the mechanical zero of that axis
// ----------------------------------------------------------------------------
typedef struct {
    float a;        // link length   (mm)
    float alpha;    // link twist    (rad)
    float d;        // link offset   (mm)
    float theta0;   // joint zero offset (rad) — folded into calibration
} kin_dh_t;

// ----------------------------------------------------------------------------
// kin_joint_limits_t: soft range of one axis (rad). IK results are checked
// against these; a solution outside the range is reported unreachable.
// ----------------------------------------------------------------------------
typedef struct {
    float min;      // lower bound (rad)
    float max;      // upper bound (rad)
} kin_joint_limits_t;

// Forward declaration — the chain is defined below but segments reference it.
typedef struct kin_chain kin_chain_t;

// ----------------------------------------------------------------------------
// kin_segment_t: contract for ONE segment solver (a coupled group of axes).
//
// A segment is a small struct carrying:
//   • dof        — how many joints it owns (and therefore consumes),
//   • first_axis — index of its first joint in the chain's joint array,
//   • solve()    — closed-form or numeric IK for JUST those joints,
//   • ctx        — opaque pointer to the segment's own geometry/state.
//
// solve() contract:
//   in   target : the pose this segment must achieve with its own joints,
//                 expressed in the segment's base frame (the chain feeds it the
//                 residual left by earlier segments).
//   in   seed   : previous joint values for this segment (for branch selection,
//                 e.g. stay on the elbow-up solution nearest to where we are).
//   out  q_out  : the solved joint angles (rad), length == dof.
//   out  residual : the pose the NEXT segment must still achieve (this
//                 segment's contribution removed). May be NULL for the last
//                 segment. For a positioner this is the leftover orientation
//                 handed to the wrist.
//   return : 0 on success, negative if the target is unreachable by this
//            segment (out of range / outside its workspace).
//
// Keeping solve() as a function pointer is what makes the chain open-ended:
// append another segment with its own solve() and the core dispatcher runs it
// unchanged. At ~100 Hz the indirect call costs a few cycles — negligible.
// ----------------------------------------------------------------------------
typedef struct kin_segment {
    uint32_t dof;         // joints owned by this segment (1..KIN_MAX_AXES)
    uint32_t first_axis;  // index of its first joint within the chain
    const void *ctx;      // segment-specific geometry (opaque to the core)

    int (*solve)(const struct kin_segment *seg,
                 const kin_pose_t *target,
                 const float *seed,
                 float *q_out,
                 kin_pose_t *residual);
} kin_segment_t;

// ----------------------------------------------------------------------------
// kin_chain_t: the whole arm = DH geometry for FK + a list of IK segments.
//
// Not opaque (unlike traj_t) on purpose: the caller assembles it explicitly by
// registering segments, which is how the "add more axes later" story works. It
// still owns fixed-size storage — no allocation.
// ----------------------------------------------------------------------------
struct kin_chain {
    uint32_t           axis_count;              // total joints (1..KIN_MAX_AXES)
    kin_dh_t           dh[KIN_MAX_AXES];        // geometry (calibratable)
    kin_joint_limits_t limits[KIN_MAX_AXES];    // per-axis soft limits (rad)

    uint32_t           segment_count;           // registered IK segments
    kin_segment_t      segments[KIN_MAX_SEGMENTS];

    kin_pose_t         base;                    // world → axis-0 base frame
};

// ============================================================================
// Chain assembly
// ============================================================================

// Initialise a chain from a DH table and per-axis limits. Sets base = identity
// and clears the segment list. `dh` and `limits` are copied in (so the caller's
// arrays need not outlive the call). Returns 0 on success, negative on bad args.
int kin_chain_init(kin_chain_t *chain,
                   const kin_dh_t *dh,
                   const kin_joint_limits_t *limits,
                   uint32_t axis_count);

// Append one IK segment. Segments MUST be registered in chain order and their
// dof must tile the axes exactly (first segment first_axis=0, next follows on).
// Returns 0 on success, negative if it would overflow or leave a gap/overlap.
int kin_chain_add_segment(kin_chain_t *chain, const kin_segment_t *seg);

// Overwrite the DH row of one axis at runtime — this is the CALIBRATION hook.
// Lets measured geometry replace the nominal CAD values without a rebuild.
// Returns 0 on success, negative on bad axis index.
int kin_calibrate_axis(kin_chain_t *chain, uint32_t axis, const kin_dh_t *dh);

// ============================================================================
// Forward kinematics — always general, any axis count
// ============================================================================

// Compute the tool pose for a set of joint angles (rad).
//   q   : joint angles, length axis_count
//   out : receives the tool pose (world frame)
// Returns 0 on success, negative on bad args.
int kin_fk(const kin_chain_t *chain, const float *q, kin_pose_t *out);

// ============================================================================
// Inverse kinematics — delegated across segments
// ============================================================================

// Solve joint angles that place the tool at `target`.
//   target : desired tool pose (world frame)
//   seed   : current joint angles (rad, length axis_count) — used to pick the
//            solution branch nearest the current pose and to warm-start any
//            numeric segment. Pass the last known joints; must not be NULL.
//   q_out  : solved joint angles (rad, length axis_count)
// Returns 0 on success, negative if the target is unreachable / out of limits.
int kin_ik(const kin_chain_t *chain,
           const kin_pose_t *target,
           const float *seed,
           float *q_out);

// ============================================================================
// Jacobian & singularities
// ============================================================================

// Geometric Jacobian J (6×N) at joint configuration q, row-major:
//   rows 0..2 = linear velocity of the tool per joint rate (mm/rad),
//   rows 3..5 = angular velocity of the tool per joint rate (1),
//   so  [v; w] = J · qdot.  `jac` must hold 6*axis_count floats.
// Returns 0 on success, negative on bad args.
int kin_jacobian(const kin_chain_t *chain, const float *q, float *jac);

// Yoshikawa manipulability measure w = sqrt(det(J·Jᵀ)) at q. It shrinks toward
// 0 as the arm approaches a singularity — compare against a small threshold to
// detect "about to lose a DOF" before commanding through it. Returns a negative
// value on bad args (valid measures are ≥ 0).
float kin_manipulability(const kin_chain_t *chain, const float *q);

// ============================================================================
// Unit converters — the entire "convenience" layer (rad/mm is canonical)
// ============================================================================
#define KIN_PI        3.14159265358979f
#define KIN_DEG2RAD   (KIN_PI / 180.0f)
#define KIN_RAD2DEG   (180.0f / KIN_PI)

static inline float kin_deg(float radians) { return radians * KIN_RAD2DEG; }
static inline float kin_rad(float degrees) { return degrees * KIN_DEG2RAD; }

// Convenience pose accessors (position is already mm; these just read it out).
static inline float kin_pose_x(const kin_pose_t *p) { return p->m[0][3]; }
static inline float kin_pose_y(const kin_pose_t *p) { return p->m[1][3]; }
static inline float kin_pose_z(const kin_pose_t *p) { return p->m[2][3]; }

// Build a position-only pose (identity orientation) from x/y/z in mm — handy
// for the 3-axis positioner where orientation is not yet controlled.
static inline kin_pose_t kin_pose_from_xyz(float x, float y, float z) {
    kin_pose_t p = {{ {1,0,0,x}, {0,1,0,y}, {0,0,1,z}, {0,0,0,1} }};
    return p;
}

#ifdef __cplusplus
}
#endif

#endif
