#pragma once

#include "types.h"
#include "spi.h"

// ============================================================================
// Hardware abstraction layer — inline accessors for SPI buffers and
// placeholder stubs for future direct-hardware FOC functions.
//
// Currently the FOC loop runs entirely inside the EXTI ISR using the
// SPI RX/TX buffers directly. These wrappers exist to provide a clean
// HAL interface if the architecture is refactored later (e.g. moving
// FOC into a timer ISR with direct ADC/PWM register access).
// ============================================================================

// ============================================================
// SPI slave buffer access
// ============================================================

// Get a pointer to the DMA receive buffer (FPGA → STM32, sensor data)
static inline volatile spi_rx_packet_t* hw_spi_rx_buf(void) {
    return spi_get_rx();
}

// Get a pointer to the DMA transmit buffer (STM32 → FPGA, PWM commands)
static inline volatile spi_tx_packet_t* hw_spi_tx_buf(void) {
    return spi_get_tx();
}

// Get the total number of SPI frames processed since boot
static inline uint32_t hw_spi_rx_count(void) {
    return spi_get_packet_count();
}

// ============================================================
// FOC hardware stubs (to be implemented when moving FOC to
// direct ADC/PWM access instead of SPI-mediated control)
// ============================================================

// Read three-phase motor currents from ADC hardware.
// STUB: returns zero currents — actual implementation will read ADC registers.
static inline phase_t hw_read_currents(void) {
    return (phase_t){0.0f, 0.0f, 0.0f};
}

// Write three-phase PWM duty cycles to timer compare registers.
// STUB: does nothing — actual implementation will write to TIM1/TIM8.
static inline void hw_write_pwm(phase_t duty) {
    (void)duty;  // Suppress unused parameter warning
}

// Return a microsecond timestamp for non-blocking timing.
// STUB: returns 0 — actual implementation will read TIM6->CNT or DWT->CYCCNT.
static inline uint32_t hw_micros(void) {
    return 0;
}