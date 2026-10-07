#ifndef CONFIG_H
#define CONFIG_H

// ============================================================
// Operating mode selection (compile-time)
//   0 = Open-loop rotation test (no encoder feedback)
//   1 = Encoder readback test (motor disabled)
//   2 = Static d-axis alignment (calibrate electrical offset)
//   3 = Align → closed-loop FOC (single axis, with calibrated offset)
//   4 = Full closed-loop FOC (all axes, production mode)
// ============================================================
#define RUN_MODE  3

// ============================================================
// System parameters
// ============================================================
#define AXIS_COUNT      3       // Number of motor axes in the system
#define PWM_PERIOD      1350    // PWM counter ceiling (27 MHz / 20 kHz = 1350)

// ============================================================
// Motor parameters
// ============================================================
#define POLE_PAIRS      2       // Number of electrical pole pairs in the motor
#define ENCODER_CPR     2048    // Encoder counts per mechanical revolution (after X4 decoding)

// ============================================================
// ADC / current sensing
// ============================================================
#define CURRENT_SCALE   0.5f    // Conversion factor: ADC counts → amperes
#define CURRENT_OFFSET  2048    // ADC zero-current reading (mid-scale for 12-bit ADC)

// ============================================================
// Calibration and test constants
// ============================================================
#define ELECTRICAL_OFFSET  0.0f     // Electrical angle offset in radians (for mode 4)
#define TEST_IQ_REF        0.12f    // q-axis current reference for mode 3 testing (amps)
#define ALIGN_TIME         3000     // Alignment duration in packets (~3000 / 20 kHz ≈ 150 ms)
#define ALIGN_AMPLITUDE    0.16f    // Voltage amplitude during alignment phase (normalized)

// ============================================================
// Math constants
// ============================================================
#define PI_F            3.14159265358979f    // π as single-precision float
#define TWO_PI_F        6.28318530717959f    // 2π as single-precision float

#endif