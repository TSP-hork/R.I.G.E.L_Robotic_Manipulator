#ifndef TYPES_H
#define TYPES_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================
// FOC-related data types
// ============================================================

// Three-phase quantity (currents, voltages, or duties)
typedef struct {
    float a;  // Phase A component
    float b;  // Phase B component
    float c;  // Phase C component
} phase_t;

// Rotor state estimate (from encoder processing)
typedef struct {
    float angle;     // Electrical angle in radians
    float velocity;  // Angular velocity (rad/s or equivalent)
} rotor_t;

// ============================================================
// SPI packet definitions: per-axis sub-structures
// ============================================================

// --- FPGA → H7 (received by STM32): one axis, 8 bytes ---
// Contains raw ADC current measurements and the encoder position
// snapshot captured by the FPGA at the PWM center point.
typedef struct __attribute__((packed)) {
    uint16_t i_a_raw;       // Raw 12-bit ADC reading for phase-A current (0..4095)
    uint16_t i_b_raw;       // Raw 12-bit ADC reading for phase-B current (0..4095)
    uint32_t enc_pos;       // Encoder position in quadrature ticks (32-bit, wrapping)
} axis_rx_t;

// --- H7 → FPGA (transmitted by STM32): one axis, 8 bytes ---
// Contains the PWM duty commands computed by FOC/SVPWM, plus control flags.
typedef struct __attribute__((packed)) {
    uint16_t pwm_a;         // Phase-A PWM compare value (0 .. PWM_PERIOD ticks)
    uint16_t pwm_b;         // Phase-B PWM compare value
    uint16_t pwm_c;         // Phase-C PWM compare value
    uint16_t flags;         // Control flags — bit 0: axis enable (1 = active, 0 = gates off)
} axis_tx_t;

// ============================================================
// SPI packet definitions: full frame (all axes in one transfer)
//
// The frame size adapts automatically to AXIS_COUNT defined in
// config.h. For 3 axes the frame is:
//   header(4) + 3×axis(8) + footer(4) = 32 bytes = 256 bits,
// matching DATA_WIDTH in the FPGA's SPI master.
// ============================================================
#include "config.h"

// --- Full RX frame: FPGA → H7 (sensor data) ---
typedef struct __attribute__((packed)) {
    uint32_t  sync;                     // Sync/magic word: must be 0xAA55AA55 for a valid frame
    axis_rx_t axis[AXIS_COUNT];         // Per-axis ADC + encoder data (8 bytes × AXIS_COUNT)
    uint32_t  sequence;                 // Monotonically increasing frame counter from FPGA
} spi_rx_packet_t;

// --- Full TX frame: H7 → FPGA (commands) ---
typedef struct __attribute__((packed)) {
    uint32_t  magic;                    // Response magic: 0xDEADBEEF — FPGA checks this for link_ok
    axis_tx_t axis[AXIS_COUNT];         // Per-axis PWM duties + enable flags (8 bytes × AXIS_COUNT)
    uint32_t  status;                   // Host-side processed-frame counter (for diagnostics)
} spi_tx_packet_t;

// ============================================================
// Compile-time size checks
// Ensures the struct packing matches the FPGA's DATA_WIDTH and
// that RX/TX frames are exactly the same size (full-duplex SPI
// requires equal-length transfers in both directions).
// ============================================================
#ifdef __cplusplus
static_assert(sizeof(spi_rx_packet_t) == 4 + 8 * AXIS_COUNT + 4,
              "RX packet size mismatch");
static_assert(sizeof(spi_tx_packet_t) == 4 + 8 * AXIS_COUNT + 4,
              "TX packet size mismatch");
static_assert(sizeof(spi_rx_packet_t) == sizeof(spi_tx_packet_t),
              "RX and TX packets must be same size");
#else
_Static_assert(sizeof(spi_rx_packet_t) == 4 + 8 * AXIS_COUNT + 4,
               "RX packet size mismatch");
_Static_assert(sizeof(spi_tx_packet_t) == 4 + 8 * AXIS_COUNT + 4,
               "TX packet size mismatch");
_Static_assert(sizeof(spi_rx_packet_t) == sizeof(spi_tx_packet_t),
               "RX and TX packets must be same size");
#endif

// Well-known magic constants used by both FPGA and MCU firmware
#define SPI_MAGIC       0xDEADBEEFu              // TX magic header — FPGA validates link_ok against this
#define SPI_SYNC        0xAA55AA55u               // RX sync header — MCU validates incoming frames against this
#define SPI_PACKET_SIZE sizeof(spi_rx_packet_t)   // Total frame size in bytes (used for DMA configuration)

#ifdef __cplusplus
}
#endif

#endif