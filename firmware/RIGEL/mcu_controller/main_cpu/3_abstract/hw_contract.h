// abstract/hw_contract.h
#pragma once

// ============================================================
// hw_contract.h — Every driver/ MUST provide hw_impl.h containing:
//
// --- FOC hardware ---
//   static inline phase_t  hw_read_currents(void);
//   static inline rotor_t  hw_read_rotor(void);
//   static inline void     hw_write_pwm(phase_t duty);
//   static inline void     hw_sincos(float angle, float *s, float *c);
//   static inline void     hw_enable_gate(void);
//   static inline void     hw_disable_gate(void);
//   static inline uint32_t hw_micros(void);
//
// --- SPI slave communication ---
//   static inline volatile spi_rx_packet_t* hw_spi_rx_buf(void);
//   static inline volatile spi_tx_packet_t* hw_spi_tx_buf(void);
//   static inline uint32_t hw_spi_rx_count(void);
//
// ============================================================
// Notes on specific contract functions
// ============================================================
//
// hw_sincos(angle, *s, *c):
//   Provided by driver/src/sincos/hw_sincos.{c,h} (NON-inline — the body is
//   large and backend-selectable). Computes sin AND cos of the same angle
//   from a single range reduction. Backend chosen at compile time in
//   hw_sincos.c (HW_SINCOS_BACKEND): software polynomial (default on this
//   MCU), libm reference, or CORDIC (template only — STM32H7A3 has NO CORDIC
//   peripheral; the CORDIC path is gated by #error for a future G4/H5 MCU or
//   FPGA offload). To adopt it in FOC, replace the sinf()/cosf() pair in
//   foc.c with one hw_sincos() call — no other change.
//
// ------------------------------------------------------------
// Cascade wiring (planned integration — no working code changed yet):
//
//   trajectory.h : traj_sample()   -> pos/vel/acc reference (per axis)
//        |
//   servo.h      : servo_step()     -> iq_ref (per axis, current-limited)
//        |
//   foc.h        : foc_set_iq_ref(axis, iq_ref)   <- EXISTING working API
//        |
//   foc.h        : foc_step() inside the 20 kHz ISR (unchanged)
//
//   The servo layer RETURNS iq_ref; the integrator (a future 1 kHz task or a
//   prescaled branch of the ISR) is what calls foc_set_iq_ref(). Every layer
//   stays testable in isolation and foc.c needs no edit.
//
// ============================================================