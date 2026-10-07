#include "stm32h7xx.h"
#include "types.h"
#include "spi.h"

// ============================================================================
// SPI RX/TX buffers — placed in SRAM1 (D2 domain) because the DMA1 controller
// on STM32H7 can only access SRAM1/SRAM2, NOT the main DTCM (D1 domain).
// Using the wrong memory region would cause silent DMA transfer failures.
// ============================================================================
static volatile spi_rx_packet_t spi_rx __attribute__((section(".sram1")));
static volatile spi_tx_packet_t spi_tx __attribute__((section(".sram1")));

// Accessor functions for the IRQ handler (returns pointers to the DMA buffers)
volatile spi_rx_packet_t* spi_get_rx(void) { return &spi_rx; }
volatile spi_tx_packet_t* spi_get_tx(void) { return &spi_tx; }

// Frame counter: incremented after each successfully processed SPI transaction
static volatile uint32_t spi_packet_count = 0;
uint32_t spi_get_packet_count(void) { return spi_packet_count; }
void spi_increment_count(void) { spi_packet_count++; }

// ============================================================================
// spi_slave_init: configure SPI3 as a DMA-driven slave, triggered by EXTI
// on the CS line (PA4). The FPGA master controls CS and SCK.
//
// Pin mapping (directly from the STM32H7 alternate-function table):
//   PA4  — SPI CS input   (directly read via GPIO, not SPI hardware CS)
//   PB3  — SPI3_SCK       (AF6)
//   PB4  — SPI3_MISO      (AF6) — STM32 transmits to FPGA on this pin
//   PB5  — SPI3_MOSI      (AF7) — STM32 receives from FPGA on this pin
//
// DMA channels:
//   DMA1 Stream 0 — SPI3 RX (DMAMUX request 61)
//   DMA1 Stream 1 — SPI3 TX (DMAMUX request 62)
//
// Interrupt:
//   EXTI4 on PA4 (both edges) — falling edge sets up DMA and enables SPI,
//   rising edge processes the received data (see EXTI4_IRQHandler in main ISR).
// ============================================================================
void spi_slave_init(void) {
    // --- Enable peripheral clocks ---
    RCC->AHB1ENR  |= (1u << 0) | (1u << 2);   // DMA1 (bit 0) and GPIOC (bit 2) — note: GPIOA/B may already be on
    RCC->APB1LENR |= (1u << 15);               // SPI3 peripheral clock
    RCC->APB4ENR  |= RCC_APB4ENR_SYSCFGEN;     // SYSCFG (needed for EXTI mux)
    for (volatile int i = 0; i < 200; i++);     // Short delay for clock stabilization

    // --- PA4: CS input (directly read by GPIO, not used as SPI hardware CS) ---
    GPIOA->MODER &= ~(3u << 8);                // PA4 = input mode (MODER bits [9:8] = 00)
    GPIOA->PUPDR  = (GPIOA->PUPDR & ~(3u << 8)) | (1u << 8);  // PA4 pull-up (idle CS is high)

    // --- PB3/PB4/PB5: SPI3 alternate function pins ---
    // Set PB3, PB4, PB5 to alternate function mode (MODER = 10 for each)
    GPIOB->MODER  = (GPIOB->MODER & ~((3u<<6)|(3u<<8)|(3u<<10)))
                   | ((2u<<6)|(2u<<8)|(2u<<10));
    // Assign alternate functions: PB3=AF6(SCK), PB4=AF6(MISO), PB5=AF7(MOSI)
    GPIOB->AFR[0] = (GPIOB->AFR[0] & ~0x00FFF000)
                   | (6u<<12) | (6u<<16) | (7u<<20);
    // High-speed output on PB4 (MISO) to keep up with 13.5 MHz SCK
    GPIOB->OSPEEDR |= (3u << 8);

    // --- EXTI4: configure PA4 as interrupt source on both edges ---
    SYSCFG->EXTICR[1] &= ~0xFu;    // EXTI4 mapped to port A (EXTICR value 0 = PA)
    EXTI->RTSR1 |= (1u << 4);      // Enable rising-edge trigger  (CS release)
    EXTI->FTSR1 |= (1u << 4);      // Enable falling-edge trigger (CS assert)
    EXTI->PR1    = (1u << 4);       // Clear any pending EXTI4 flag
    EXTI->IMR1  |= (1u << 4);      // Unmask EXTI4 interrupt
    NVIC_SetPriority(EXTI4_IRQn, 0); // Highest priority — FOC latency is critical
    NVIC_EnableIRQ(EXTI4_IRQn);

    // --- Pre-fill the TX buffer with safe defaults ---
    spi_tx.magic  = SPI_MAGIC;      // 0xDEADBEEF — FPGA checks this for link validation
    spi_tx.status = 0;              // Frame counter starts at 0

    // Zero all per-axis PWM commands and disable all gates
    for (int i = 0; i < AXIS_COUNT; i++) {
        spi_tx.axis[i].pwm_a  = 0;
        spi_tx.axis[i].pwm_b  = 0;
        spi_tx.axis[i].pwm_c  = 0;
        spi_tx.axis[i].flags  = 0;  // bit 0 = 0 → axis disabled (safe state)
    }

    // --- DMA1 Stream 0: SPI3 RX (peripheral → memory) ---
    DMAMUX1_Channel0->CCR = 61;                  // DMAMUX request ID 61 = SPI3_RX
    DMA1_Stream0->PAR  = (uint32_t)&SPI3->RXDR;  // Source: SPI3 receive data register
    DMA1_Stream0->M0AR = (uint32_t)&spi_rx;       // Destination: RX buffer in SRAM1
    DMA1_Stream0->CR   = DMA_SxCR_MINC;           // Memory increment mode; direction = periph→mem (default)

    // --- DMA1 Stream 1: SPI3 TX (memory → peripheral) ---
    DMAMUX1_Channel1->CCR = 62;                  // DMAMUX request ID 62 = SPI3_TX
    DMA1_Stream1->PAR  = (uint32_t)&SPI3->TXDR;  // Destination: SPI3 transmit data register
    DMA1_Stream1->M0AR = (uint32_t)&spi_tx;       // Source: TX buffer in SRAM1
    DMA1_Stream1->CR   = DMA_SxCR_MINC | DMA_SxCR_DIR_0;  // Memory increment; direction = mem→periph

    // --- SPI3 configuration: slave mode, 8-bit frames, DMA-driven ---
    SPI3->CR1  = 0;                              // Peripheral disabled; master bit = 0 → slave mode
    SPI3->CFG1 = (7u << SPI_CFG1_DSIZE_Pos)      // Data size = 8 bits (DSIZE = 7 means 7+1 = 8)
               | SPI_CFG1_RXDMAEN | SPI_CFG1_TXDMAEN;  // Enable DMA requests for both RX and TX
    SPI3->CFG2 = SPI_CFG2_SSM;                   // Software slave management (CS handled by EXTI, not SPI HW)
    SPI3->CR2  = 0;                              // No fixed transfer size (DMA controls byte count)
}