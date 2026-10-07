#include "kinematics.h"

#include <math.h>   // sinf, cosf, sqrtf  (desktop libm; on MCU the FPU handles the arithmetic)

// ============================================================================
// kinematics.c — the core solver: general FK, IK dispatch, Jacobian.
//
// This file is arm-agnostic. It knows how to:
//   • compose per-axis DH transforms into a tool pose (FK),
//   • walk the registered segments and let each solve its own joints (IK),
//   • build the geometric Jacobian and a singularity measure.
//
// The arm-SPECIFIC maths (the actual closed-form for the 3-axis positioner,
// the wrist, …) lives in the seg_*.c files behind the kin_segment_t contract.
// Add a segment there and this file runs it unchanged.
// ============================================================================

// ----------------------------------------------------------------------------
// 4×4 rigid-transform helpers (row-major, column-vector convention).
// ----------------------------------------------------------------------------

// C = A · B  (C may not alias A or B)
static void mat_mul(const kin_pose_t *A, const kin_pose_t *B, kin_pose_t *C) {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            float s = 0.0f;
            for (int k = 0; k < 4; k++) {
                s += A->m[i][k] * B->m[k][j];
            }
            C->m[i][j] = s;
        }
    }
}

// Identity pose.
static kin_pose_t mat_identity(void) {
    kin_pose_t I = {{ {1,0,0,0}, {0,1,0,0}, {0,0,1,0}, {0,0,0,1} }};
    return I;
}

// Standard-DH transform for one joint from its four parameters and the
// commanded angle. This is the textbook A_i matrix:
//
//   A_i = Rot_z(theta) · Trans_z(d) · Trans_x(a) · Rot_x(alpha)
//
//       | cθ  -sθ·cα   sθ·sα   a·cθ |
//   =   | sθ   cθ·cα  -cθ·sα   a·sθ |
//       | 0     sα      cα       d   |
//       | 0      0       0       1   |
//
// The joint's mechanical zero (theta0) is added to the commanded angle here, so
// callers always pass the logical joint angle and calibration lives in the DH.
static kin_pose_t dh_transform(const kin_dh_t *dh, float theta_cmd) {
    float theta = theta_cmd + dh->theta0;
    float ct = cosf(theta),     st = sinf(theta);
    float ca = cosf(dh->alpha), sa = sinf(dh->alpha);
    float a = dh->a, d = dh->d;

    kin_pose_t T = {{
        { ct, -st * ca,  st * sa, a * ct },
        { st,  ct * ca, -ct * sa, a * st },
        { 0.0f,   sa,       ca,     d    },
        { 0.0f, 0.0f,     0.0f,   1.0f   },
    }};
    return T;
}

// ============================================================================
// Chain assembly
// ============================================================================

int kin_chain_init(kin_chain_t *chain,
                   const kin_dh_t *dh,
                   const kin_joint_limits_t *limits,
                   uint32_t axis_count) {
    if (chain == 0 || dh == 0 || limits == 0 ||
        axis_count == 0 || axis_count > KIN_MAX_AXES) {
        return -1;
    }

    chain->axis_count = axis_count;
    for (uint32_t i = 0; i < axis_count; i++) {
        chain->dh[i]     = dh[i];
        chain->limits[i] = limits[i];
    }
    chain->segment_count = 0;
    chain->base = mat_identity();
    return 0;
}

int kin_chain_add_segment(kin_chain_t *chain, const kin_segment_t *seg) {
    if (chain == 0 || seg == 0 || seg->solve == 0 || seg->dof == 0) {
        return -1;
    }
    if (chain->segment_count >= KIN_MAX_SEGMENTS) {
        return -2;  // no room
    }

    // Segments must tile the joints exactly, in order and without gaps: the
    // first starts at axis 0, each subsequent one starts where the previous
    // ended. This guarantees the residual-pose handoff stays coherent.
    uint32_t expected_first = 0;
    if (chain->segment_count > 0) {
        const kin_segment_t *prev = &chain->segments[chain->segment_count - 1];
        expected_first = prev->first_axis + prev->dof;
    }
    if (seg->first_axis != expected_first) {
        return -3;  // gap or overlap with the previous segment
    }
    if (seg->first_axis + seg->dof > chain->axis_count) {
        return -4;  // runs past the end of the chain
    }

    chain->segments[chain->segment_count++] = *seg;
    return 0;
}

int kin_calibrate_axis(kin_chain_t *chain, uint32_t axis, const kin_dh_t *dh) {
    if (chain == 0 || dh == 0 || axis >= chain->axis_count) {
        return -1;
    }
    chain->dh[axis] = *dh;   // measured geometry replaces nominal — no rebuild
    return 0;
}

// ============================================================================
// Forward kinematics
// ============================================================================
//
// Walk the chain accumulating T = base · A_0 · A_1 · … · A_{n-1}. This is the
// "kinematic chain" computed all together, exactly as you pictured it — each
// joint contributes one matrix and they multiply in sequence.

int kin_fk(const kin_chain_t *chain, const float *q, kin_pose_t *out) {
    if (chain == 0 || q == 0 || out == 0) {
        return -1;
    }

    kin_pose_t acc = chain->base;
    for (uint32_t i = 0; i < chain->axis_count; i++) {
        kin_pose_t Ai = dh_transform(&chain->dh[i], q[i]);
        kin_pose_t tmp;
        mat_mul(&acc, &Ai, &tmp);
        acc = tmp;
    }
    *out = acc;
    return 0;
}

// ============================================================================
// Inverse kinematics — segment dispatch
// ============================================================================
//
// The chain hands each segment the pose it must still achieve and collects the
// residual it passes on. For the current single-segment positioner this is just
// one call; the loop is what lets a wrist (or more) slot in behind it later
// with no change here.

int kin_ik(const kin_chain_t *chain,
           const kin_pose_t *target,
           const float *seed,
           float *q_out) {
    if (chain == 0 || target == 0 || seed == 0 || q_out == 0) {
        return -1;
    }
    if (chain->segment_count == 0) {
        return -2;  // no solvers registered
    }

    // The first segment works in the chain's base frame. If base != identity,
    // fold it in so segments always see a target relative to axis 0. (Positioner
    // geometry below assumes the base frame; keeping base = identity is the
    // simple path and what the tests use.)
    kin_pose_t residual = *target;

    for (uint32_t s = 0; s < chain->segment_count; s++) {
        const kin_segment_t *seg = &chain->segments[s];
        const float *seg_seed = &seed[seg->first_axis];
        float       *seg_q    = &q_out[seg->first_axis];

        // Last segment need not produce a residual.
        kin_pose_t next_residual;
        kin_pose_t *rp = (s + 1 < chain->segment_count) ? &next_residual : 0;

        int rc = seg->solve(seg, &residual, seg_seed, seg_q, rp);
        if (rc != 0) {
            return rc;  // this segment cannot reach — propagate the failure
        }
        if (rp) {
            residual = next_residual;
        }
    }

    // Enforce soft joint limits on the assembled solution.
    for (uint32_t i = 0; i < chain->axis_count; i++) {
        if (q_out[i] < chain->limits[i].min ||
            q_out[i] > chain->limits[i].max) {
            return -5;  // solved, but a joint is outside its allowed range
        }
    }
    return 0;
}

// ============================================================================
// Geometric Jacobian
// ============================================================================
//
// For an all-revolute chain, column i of the Jacobian is:
//     J_v[i] = z_{i} × (p_end − p_{i})     (linear part)
//     J_w[i] = z_{i}                        (angular part)
// where z_i is the axis-i rotation axis (world frame) and p_i its origin, both
// taken from the running FK product BEFORE applying joint i. We accumulate the
// per-joint frames once, then fill the 6×N matrix.

int kin_jacobian(const kin_chain_t *chain, const float *q, float *jac) {
    if (chain == 0 || q == 0 || jac == 0) {
        return -1;
    }

    uint32_t n = chain->axis_count;

    // Origins and z-axes of each joint frame (world), plus the end origin.
    float z[KIN_MAX_AXES][3];   // rotation axis of joint i (world)
    float p[KIN_MAX_AXES][3];   // origin of joint i (world)
    float p_end[3];

    kin_pose_t acc = chain->base;
    for (uint32_t i = 0; i < n; i++) {
        // Frame BEFORE joint i is applied: its Z column is the joint axis, its
        // translation is the joint origin.
        z[i][0] = acc.m[0][2]; z[i][1] = acc.m[1][2]; z[i][2] = acc.m[2][2];
        p[i][0] = acc.m[0][3]; p[i][1] = acc.m[1][3]; p[i][2] = acc.m[2][3];

        kin_pose_t Ai = dh_transform(&chain->dh[i], q[i]);
        kin_pose_t tmp;
        mat_mul(&acc, &Ai, &tmp);
        acc = tmp;
    }
    p_end[0] = acc.m[0][3]; p_end[1] = acc.m[1][3]; p_end[2] = acc.m[2][3];

    // Fill columns. Row-major 6×n: element (r,i) at jac[r*n + i].
    for (uint32_t i = 0; i < n; i++) {
        float dx = p_end[0] - p[i][0];
        float dy = p_end[1] - p[i][1];
        float dz = p_end[2] - p[i][2];

        // linear = z × (p_end − p_i)
        jac[0 * n + i] = z[i][1] * dz - z[i][2] * dy;
        jac[1 * n + i] = z[i][2] * dx - z[i][0] * dz;
        jac[2 * n + i] = z[i][0] * dy - z[i][1] * dx;
        // angular = z
        jac[3 * n + i] = z[i][0];
        jac[4 * n + i] = z[i][1];
        jac[5 * n + i] = z[i][2];
    }
    return 0;
}

// ----------------------------------------------------------------------------
// Manipulability w = sqrt(det(G)), where G is the smaller of the two Gram
// matrices of the 6×N Jacobian:
//
//   • N ≥ 6 (redundant/full):   G = J·Jᵀ  (6×6)   — the classic Yoshikawa form.
//   • N < 6 (under-actuated):   G = Jᵀ·J  (N×N)   — REQUIRED here, because a
//     6×N Jacobian with N<6 has rank ≤ N, so det(J·Jᵀ) is ALWAYS 0 and would be
//     useless. det(Jᵀ·J) is the squared product of J's singular values and
//     still collapses toward 0 at a singularity — the early-warning signal.
//
// Our 3-axis positioner is the N<6 case; using Jᵀ·J is what makes the measure
// meaningful (it shrinks as the elbow straightens). We build the min(6,N)-sized
// symmetric matrix and take its determinant by unpivoted LU (it is PSD; a
// vanishing pivot means singular → determinant 0).
// ----------------------------------------------------------------------------
static float gram_det(float *A, uint32_t m) {
    // A is an m×m symmetric PSD matrix, row-major (A[i*m + j]). Destroyed here.
    float det = 1.0f;
    for (uint32_t k = 0; k < m; k++) {
        float piv = A[k * m + k];
        if (fabsf(piv) < 1e-12f) {
            return 0.0f;  // singular → not full column rank at this pose
        }
        det *= piv;
        for (uint32_t i = k + 1; i < m; i++) {
            float f = A[i * m + k] / piv;
            for (uint32_t j = k; j < m; j++) {
                A[i * m + j] -= f * A[k * m + j];
            }
        }
    }
    return det;
}

float kin_manipulability(const kin_chain_t *chain, const float *q) {
    if (chain == 0 || q == 0) {
        return -1.0f;
    }

    uint32_t n = chain->axis_count;
    float jac[6 * KIN_MAX_AXES];
    if (kin_jacobian(chain, q, jac) != 0) {
        return -1.0f;
    }

    // Choose the smaller Gram matrix so it is full-rank when the arm is not at a
    // singularity. rows of J indexed 0..5, columns 0..n-1 (stored jac[r*n+c]).
    if (n >= 6) {
        // G = J · Jᵀ  (6×6)
        float G[6 * 6];
        for (uint32_t r = 0; r < 6; r++) {
            for (uint32_t c = 0; c < 6; c++) {
                float s = 0.0f;
                for (uint32_t i = 0; i < n; i++) {
                    s += jac[r * n + i] * jac[c * n + i];
                }
                G[r * 6 + c] = s;
            }
        }
        float det = gram_det(G, 6);
        if (det < 0.0f) det = 0.0f;
        return sqrtf(det);
    } else {
        // G = Jᵀ · J  (n×n) — the correct form for an under-actuated arm.
        float G[KIN_MAX_AXES * KIN_MAX_AXES];
        for (uint32_t a = 0; a < n; a++) {
            for (uint32_t b = 0; b < n; b++) {
                float s = 0.0f;
                for (uint32_t r = 0; r < 6; r++) {
                    s += jac[r * n + a] * jac[r * n + b];
                }
                G[a * n + b] = s;
            }
        }
        float det = gram_det(G, n);
        if (det < 0.0f) det = 0.0f;
        return sqrtf(det);
    }
}
