#ifndef SPI_H
#define SPI_H

#include <stdint.h>

// Initialize SPI1 in Slave Mode with DMA.

void SPI_Init(void);

// Start SPI DMA transfer.

void SPI_Start(void);

#endif // SPI_H