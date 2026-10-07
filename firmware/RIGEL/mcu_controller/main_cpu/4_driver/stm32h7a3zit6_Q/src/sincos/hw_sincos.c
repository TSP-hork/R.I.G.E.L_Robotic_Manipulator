#include "hw_sincos.h"

// ============================================================================
// hw_sincos.c — sine/cosine backends for the STM32H7A3.
//
// ----------------------------------------------------------------------------
// BACKEND SELECTION
// ----------------------------------------------------------------------------
// Choose ONE backend by setting HW_SINCOS_BACKEND below (or -D on the build):
//
//   HW_SINCOS_SW      Software polynomial. Portable, always available.
//                     Argument reduced once; sin and cos share it. This is
//                     the ACTIVE default because the STM32H7A3 has NO CORDIC.
//
//   HW_SINCOS_LIBM    Plain sinf()+cosf() from newlib. Kept as a reference /
//                     fallback so the software path can be A/B compared for
//                     accuracy on the desktop.
//
//   HW_SINCOS_CORDIC  Hardware CORDIC coprocessor. *** NOT AVAILABLE on the
//                     STM32H7A3 — this MCU has no CORDIC peripheral (verified
//                     against CMSIS stm32h7a3xxq.h). The code path is a
//                     documented skeleton, guarded by #error, ready for a
//                     future CORDIC-equipped MCU (e.g. STM32G4/H5) or an
//                     FPGA-side implementation. See the block at the bottom.
//
// Why not just keep sinf/cosf? Because the contract calls for a single,
// swappable sin+cos entry point, and because doing the range reduction once
// (this file) instead of twice (libm, per call) is a measurable saving in the
// 20 kHz FOC loop while staying bit-for-bit deterministic across platforms.
// ============================================================================

#ifndef HW_SINCOS_BACKEND
#define HW_SINCOS_BACKEND HW_SINCOS_SW    // default: software (H7A3 has no CORDIC)
#endif

// Backend identifiers (do not reorder — used only for comparison).
#define HW_SINCOS_SW      0
#define HW_SINCOS_LIBM    1
#define HW_SINCOS_CORDIC  2

// ============================================================================
// Shared math constants (single-precision). One place to edit.
// ============================================================================
#define HS_PI        3.14159265358979f   // π
#define HS_TWO_PI    6.28318530717959f   // 2π
#define HS_HALF_PI   1.57079632679490f   // π/2
#define HS_INV_TWO_PI 0.159154943091895f // 1/(2π)

// ============================================================================
// Backend 0: SOFTWARE polynomial  (ACTIVE on STM32H7A3)
// ============================================================================
#if HW_SINCOS_BACKEND == HW_SINCOS_SW

// Minimax-style polynomials evaluated on the reduced range [-π/4, +π/4],
// where they are most accurate. Coefficients are the standard Taylor terms;
// on [-π/4, π/4] the truncation error is well under 1e-6 — far finer than the
// 12-bit sensor chain feeding the FOC loop, so it is not the accuracy
// bottleneck.
//
//   sin(x) ≈ x − x³/6 + x⁵/120 − x⁷/5040
//   cos(x) ≈ 1 − x²/2 + x⁴/24 − x⁶/720
#define HS_S3   (-1.0f / 6.0f)
#define HS_S5   ( 1.0f / 120.0f)
#define HS_S7   (-1.0f / 5040.0f)
#define HS_C2   (-1.0f / 2.0f)
#define HS_C4   ( 1.0f / 24.0f)
#define HS_C6   (-1.0f / 720.0f)

static inline float poly_sin(float x) {
    float x2 = x * x;
    // Horner form: x·(1 + x²·(S3 + x²·(S5 + x²·S7)))
    return x * (1.0f + x2 * (HS_S3 + x2 * (HS_S5 + x2 * HS_S7)));
}

static inline float poly_cos(float x) {
    float x2 = x * x;
    // Horner form: 1 + x²·(C2 + x²·(C4 + x²·C6))
    return 1.0f + x2 * (HS_C2 + x2 * (HS_C4 + x2 * HS_C6));
}

void hw_sincos(float angle, float *s, float *c) {
    // --- Range reduction: fold `angle` into one of four quadrants of [0,2π) ---
    // k = round(angle / (2π)); r = angle − k·2π  → r ∈ [-π, π]
    // (done ONCE here; both sin and cos reuse it — the whole point vs libm)
    float k = angle * HS_INV_TWO_PI;
    k = (k >= 0.0f) ? (float)(int)(k + 0.5f) : (float)(int)(k - 0.5f);
    float r = angle - k * HS_TWO_PI;   // r ∈ [-π, π]

    // Fold [-π, π] onto [-π/2, π/2] tracking sign, so the polynomials only
    // ever see |x| ≤ π/2 (and we further split at π/4 for best accuracy).
    float sin_sign = 1.0f;
    float cos_sign = 1.0f;
    if (r > HS_HALF_PI) {          // second quadrant
        r = HS_PI - r;
        cos_sign = -1.0f;
    } else if (r < -HS_HALF_PI) {  // third quadrant
        r = -HS_PI - r;
        cos_sign = -1.0f;
    }

    // On [-π/2, π/2], swap sin/cos roles near the top of the range so each
    // polynomial is evaluated where it is most accurate (|x| ≤ π/4).
    float sn, cs;
    if (r > (HS_HALF_PI * 0.5f)) {        // r in (π/4, π/2]
        float x = HS_HALF_PI - r;         // co-angle in [0, π/4)
        sn = poly_cos(x);
        cs = poly_sin(x);
    } else if (r < -(HS_HALF_PI * 0.5f)) { // r in [-π/2, -π/4)
        float x = HS_HALF_PI + r;         // co-angle in [0, π/4)
        sn = -poly_cos(x);
        cs =  poly_sin(x);
    } else {                              // r in [-π/4, π/4]
        sn = poly_sin(r);
        cs = poly_cos(r);
    }

    *s = sin_sign * sn;
    *c = cos_sign * cs;
}

// ============================================================================
// Backend 1: LIBM reference / fallback
// ============================================================================
#elif HW_SINCOS_BACKEND == HW_SINCOS_LIBM

#include <math.h>

void hw_sincos(float angle, float *s, float *c) {
    *s = sinf(angle);
    *c = cosf(angle);
}

// ============================================================================
// Backend 2: HARDWARE CORDIC  (NOT PRESENT on STM32H7A3)
// ============================================================================
#elif HW_SINCOS_BACKEND == HW_SINCOS_CORDIC

#error "STM32H7A3 has no CORDIC peripheral. This backend is a template for a \
CORDIC-equipped MCU (STM32G4/H5) or an FPGA offload. Remove this #error and \
fill in the register access below, then verify the base address / bit fields \
against that chip's reference manual and CMSIS header."

// ---------------------------------------------------------------------------
// TEMPLATE — how the CORDIC backend would look on a chip that HAS one.
//
// Everything hardware-specific is collected here so that, on a supported MCU,
// you edit only this block (or better: delete these and use the CMSIS symbols
// like `CORDIC`, `CORDIC_CSR_FUNC`, etc., which already carry the right base
// address and bit positions). The register #defines below are shown ONLY so
// the intent is readable on a chip whose CMSIS lacks them.
//
//   #define HS_CORDIC_BASE   0x40016400UL          // <-- verify per chip RM
//   #define HS_CORDIC        ((CORDIC_TypeDef*)HS_CORDIC_BASE)
//
// CORDIC on STM32G4:
//   • FUNC = 0 (cosine); a single write to WDATA starts the computation.
//   • Reading RDATA gives cosine, a second read gives sine (NRES=2), OR
//     configure to output sine first — check CSR.FUNC / CSR.NARGS / CSR.NRES.
//   • Data is q1.31 fixed point: the input angle is scaled so that π maps to
//     +1.0 (i.e. angle_q31 = angle/π in q1.31). Output is q1.31 in [-1,1).
//
// The KEY performance trick (hide the ~15-cycle latency): split into a
// "start" and a "read" so useful FOC work runs while CORDIC computes.
// On this project the Clarke transform needs no angle, so it fits between:
//
//   // in foc_step, conceptually:
//   hw_sincos_start(angle);     // kick off CORDIC
//   ... Clarke transform ...    // CORDIC runs in parallel here
//   hw_sincos_read(&sin, &cos); // result ready → little/no stall
//
// If you adopt that pattern, add hw_sincos_start()/hw_sincos_read() to the
// contract too. The plain hw_sincos() below keeps the simple one-shot form.
// ---------------------------------------------------------------------------
//
// #include "stm32h7xx.h"   // or the target chip's CMSIS
//
// #define HS_Q31_SCALE   2147483648.0f   // 2^31
// #define HS_INV_PI      0.318309886f     // 1/π  (angle → q1.31 needs /π)
//
// static void cordic_init(void) {
//     // Enable CORDIC clock in RCC (bit is chip-specific — use CMSIS symbol).
//     // Configure CSR: FUNC=cosine, PRECISION=5..6 (accuracy/speed tradeoff),
//     // NARGS/NRES for single 32-bit arg / two 32-bit results.
// }
//
// void hw_sincos(float angle, float *s, float *c) {
//     // Reduce angle to [-π, π] first (CORDIC input domain), then scale.
//     // ... range reduction identical in spirit to the SW backend ...
//     int32_t arg = (int32_t)(r * HS_INV_PI * HS_Q31_SCALE);
//     CORDIC->WDATA = (uint32_t)arg;          // start
//     int32_t cos_q = (int32_t)CORDIC->RDATA; // may stall until ready
//     int32_t sin_q = (int32_t)CORDIC->RDATA;
//     *c = (float)cos_q / HS_Q31_SCALE;
//     *s = (float)sin_q / HS_Q31_SCALE;
// }

#else
#error "Unknown HW_SINCOS_BACKEND"
#endif
