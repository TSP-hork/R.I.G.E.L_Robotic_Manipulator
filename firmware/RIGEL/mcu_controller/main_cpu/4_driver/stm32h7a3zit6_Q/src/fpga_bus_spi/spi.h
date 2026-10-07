#ifndef SPI_H
#define SPI_H

#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

// Initialize SPI3 as a DMA-driven slave, with EXTI4 on PA4 (CS) for
// both-edge interrupt. Must be called once during system startup before
// the FPGA begins sending SPI frames.
void spi_slave_init(void);

// Return a pointer to the DMA receive buffer (FPGA → STM32 sensor data).
// The buffer resides in SRAM1 (D2 domain) for DMA1 accessibility.
volatile spi_rx_packet_t* spi_get_rx(void);

// Return a pointer to the DMA transmit buffer (STM32 → FPGA PWM commands).
// The buffer resides in SRAM1 (D2 domain) for DMA1 accessibility.
volatile spi_tx_packet_t* spi_get_tx(void);

// Return the total number of SPI frames processed since boot.
// Used for diagnostics and as the 'status' field in the TX packet.
uint32_t spi_get_packet_count(void);

// Increment the internal SPI frame counter by one.
// Called by the EXTI handler after each successfully processed frame.
void spi_increment_count(void);

#ifdef __cplusplus
}
#endif

#endif