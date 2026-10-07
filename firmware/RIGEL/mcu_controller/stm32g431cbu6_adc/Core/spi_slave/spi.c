#include "spi.h"
#include "adc.h"
#include "stm32g4xx.h"

void SPI_Init(void) {
    // Enable the clock for SPI1 peripheral in APB2 bus
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    // Configure PA4, PA5, PA6, PA7 to Alternate Function mode (0b10)
    // Clear bits 8-15 (FF00) and set bits to 10101010 (AA00)
    GPIOA->MODER &= ~0x0000FF00; GPIOA->MODER |= 0x0000AA00;

    // Select AF5 (SPI1) for PA4-PA7 in Alternate Function Low Register
    // Clear nibbles for pins 4-7 and set them to 0101 (5)
    GPIOA->AFR[0] &= ~0xFFFF0000; GPIOA->AFR[0] |= 0x55550000;

    // Reset SPI1 Control Register 1 (default: Slave, CPOL=0, CPHA=0, MSB First)
    SPI1->CR1 = 0;

    // Configure Control Register 2: Data Size = 16-bit (1111), Enable TX DMA request
    SPI1->CR2 = (15 << SPI_CR2_DS_Pos) | SPI_CR2_TXDMAEN;

    // Configure DMAMUX to route SPI1_TX request (ID 11) to DMA1 Channel 2
    DMAMUX1_Channel1->CCR = 11; 

    // Reset DMA1 Channel 2 Configuration Register
    DMA1_Channel2->CCR = 0;

    // Configure DMA CCR: 16-bit Mem/Periph size, Memory Increment, Dir: Mem->Periph, Circular mode
    DMA1_Channel2->CCR = (1 << DMA_CCR_MSIZE_Pos) | (1 << DMA_CCR_PSIZE_Pos) | 
                         DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_CIRC;

    // Set Peripheral Address to SPI1 Data Register
    DMA1_Channel2->CPAR = (uint32_t)&SPI1->DR;

    // Set Memory Address to the source buffer (ADC data)
    DMA1_Channel2->CMAR = (uint32_t)&adc_dual_data;

    // Set number of data items to transfer: 2 words
    DMA1_Channel2->CNDTR = 2; 

    // Enable DMA1 Channel 2
    DMA1_Channel2->CCR |= DMA_CCR_EN;

    // Enable SPI Peripheral
    SPI1->CR1 |= SPI_CR1_SPE;
}
void SPI_Start(void) {
    // Enable DMA Channel to start waiting for SPI TX requests
    // Note: SPI1->CR1_SPE must be enabled previously
    DMA1_Channel2->CCR |= DMA_CCR_EN;
}