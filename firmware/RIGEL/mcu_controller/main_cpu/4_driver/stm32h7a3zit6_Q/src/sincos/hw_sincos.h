#ifndef HW_SINCOS_H
#define HW_SINCOS_H

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// hw_sincos — driver-layer sine/cosine provider.
//
// This implements the function named in the hardware contract
// (3_abstract/hw_contract.h):
//
//     static inline void hw_sincos(float angle, float *s, float *c);
//
// but as a real (non-inline) function, because the body is too large to
// inline sensibly and because the backend is selected at compile time.
//
// The point of routing sin/cos through this single function is SWAPPABILITY.
// The FOC layer wants sin(θ) AND cos(θ) of the SAME angle every cycle. Today
// foc.c calls sinf()+cosf() separately (libm reduces the argument twice).
// Calling hw_sincos() instead:
//   • reduces the argument ONCE and returns both values, and
//   • can be redirected to a hardware accelerator (CORDIC / FPGA) by flipping
//     one #define — with NO change to the FOC call site.
//
// INTEGRATION (your action, later — not done here):
//   In foc.c, replace:
//       float sin_a = sinf(angle);
//       float cos_a = cosf(angle);
//   with:
//       float sin_a, cos_a;
//       hw_sincos(angle, &sin_a, &cos_a);
//   Nothing else changes. Pick the backend in hw_sincos.c.
// ============================================================================

// Compute sine and cosine of `angle` (radians, any finite value).
//   *s ← sin(angle)
//   *c ← cos(angle)
void hw_sincos(float angle, float *s, float *c);

#ifdef __cplusplus
}
#endif

#endif
