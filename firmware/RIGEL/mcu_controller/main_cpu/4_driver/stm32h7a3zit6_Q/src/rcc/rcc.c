#include "stm32h7xx.h"

// ============================================================================
// rcc_init: configure the STM32H7 clock tree for maximum performance.
//
// Target configuration:
//   HSE input:    8 MHz (external bypass oscillator)
//   PLL1 VCO:     8 MHz / 1 × 70 = 560 MHz
//   SYSCLK (P):   560 / 2 = 280 MHz  (CPU core clock)
//   AHB (HCLK):   280 MHz / 1 = 280 MHz
//   APB1:         280 MHz / 2 = 140 MHz  (SPI3, timers)
//   APB2:         280 MHz / 2 = 140 MHz  (SPI1, timers)
//   APB4:         280 MHz / 1 = 280 MHz  (SYSCFG, EXTI)
//
// Also configures:
//   - SMPS power supply (instead of LDO) for better efficiency
//   - VOS0 voltage scaling (required for >225 MHz operation)
//   - Flash wait states (6 WS for 280 MHz)
// ============================================================================
void rcc_init(void) {
    // Enable SYSCFG clock (needed for EXTI multiplexer and power control access)
    RCC->APB4ENR |= RCC_APB4ENR_SYSCFGEN;
    for (volatile int i = 0; i < 1000; i++);  // Wait for clock to stabilize

    // ---- Power supply: switch from LDO to SMPS for better efficiency ----
    // Required before increasing voltage scaling to VOS0.
    PWR->CR3 = (PWR->CR3 & ~PWR_CR3_LDOEN) | PWR_CR3_SMPSEN;
    for (volatile uint32_t d = 0; d < 100000; d++);  // Wait for SMPS to stabilize

    // ---- Voltage scaling: set VOS0 (highest performance level) ----
    // VOS0 allows SYSCLK up to 280 MHz on STM32H723/725/730/733/735.
    PWR->SRDCR = (PWR->SRDCR & ~PWR_SRDCR_VOS)
               | (3UL << PWR_SRDCR_VOS_Pos);         // VOS = 11 → Scale 0
    while (!(PWR->CSR1 & PWR_CSR1_ACTVOSRDY));        // Wait until voltage scaling is ready
    while (((PWR->CSR1 >> 14) & 3u) != 3u);           // Confirm active VOS matches requested level

    // ---- HSE: enable external high-speed oscillator in bypass mode ----
    // Bypass mode is used when an external clock signal (not a crystal) feeds the HSE pin.
    RCC->CR |= RCC_CR_HSEBYP;     // Select bypass (external clock source, not crystal)
    RCC->CR |= RCC_CR_HSEON;      // Turn on HSE
    while (!(RCC->CR & RCC_CR_HSERDY));  // Wait until HSE is stable and ready

    // ---- PLL1 configuration: HSE(8 MHz) / M(1) × N(70) = 560 MHz VCO ----
    // P divider = 2 → SYSCLK = 280 MHz
    RCC->CR &= ~RCC_CR_PLL1ON;            // Disable PLL1 before reconfiguring
    while (RCC->CR & RCC_CR_PLL1RDY);     // Wait until PLL1 is fully stopped

    // PLL input: HSE, pre-divider M = 1 → PLL input = 8 MHz
    RCC->PLLCKSELR = (RCC->PLLCKSELR
                     & ~(RCC_PLLCKSELR_PLLSRC | RCC_PLLCKSELR_DIVM1))
                   | RCC_PLLCKSELR_PLLSRC_HSE               // Source = HSE
                   | (1UL << RCC_PLLCKSELR_DIVM1_Pos);      // M = 1 (8 MHz / 1 = 8 MHz to PLL)

    // PLL1 VCO settings:
    //   VCOSEL = 0 → wide VCO range (192–960 MHz)
    //   RGE = 3 → PLL input range 8–16 MHz
    //   FRACEN = 0 → integer mode (no fractional-N)
    //   DIVP1EN = 1 → enable P output (SYSCLK)
    RCC->PLLCFGR = (RCC->PLLCFGR
                   & ~(RCC_PLLCFGR_PLL1VCOSEL
                      | RCC_PLLCFGR_PLL1RGE
                      | RCC_PLLCFGR_PLL1FRACEN))
                 | (3UL << RCC_PLLCFGR_PLL1RGE_Pos)         // Input range 8–16 MHz
                 | RCC_PLLCFGR_DIVP1EN;                      // Enable DIVP output

    // PLL multiplier and dividers:
    //   N = 70  → VCO = 8 × 70 = 560 MHz
    //   P = 2   → SYSCLK = 560 / 2 = 280 MHz
    //   Q = 2   → 280 MHz (available for peripherals)
    //   R = 2   → 280 MHz (available for peripherals)
    RCC->PLL1DIVR = ((70 - 1) << RCC_PLL1DIVR_N1_Pos)       // N = 70
                  | ((2 - 1)  << RCC_PLL1DIVR_P1_Pos)       // P = 2
                  | ((2 - 1)  << RCC_PLL1DIVR_Q1_Pos)       // Q = 2
                  | ((2 - 1)  << RCC_PLL1DIVR_R1_Pos);      // R = 2
    RCC->PLL1FRACR = 0;                                      // No fractional component

    // Start PLL1 and wait for lock
    RCC->CR |= RCC_CR_PLL1ON;
    while (!(RCC->CR & RCC_CR_PLL1RDY));  // Wait until PLL1 output is stable

    // ---- Flash latency: 6 wait states required for 280 MHz operation ----
    // Also set the programming delay (WRHIGHFREQ = 2 for 225–280 MHz range)
    FLASH->ACR = (FLASH->ACR & ~(FLASH_ACR_LATENCY | (3u << 4)))
               | FLASH_ACR_LATENCY_6WS                      // 6 wait states
               | (2u << 4);                                  // WRHIGHFREQ = 2
    while ((FLASH->ACR & FLASH_ACR_LATENCY) != FLASH_ACR_LATENCY_6WS);  // Confirm applied

    // ---- Bus clock dividers ----
    // AHB  = SYSCLK / 1 = 280 MHz (CPU, DMA, GPIO)
    // APB1 = AHB / 2    = 140 MHz (SPI3, TIM2-7, USART2-5)
    // APB2 = AHB / 2    = 140 MHz (SPI1, TIM1/8, USART1/6)
    // APB4 = SYSCLK / 1 = 280 MHz (SYSCFG, EXTI, LPUART1)
    RCC->CDCFGR1 = 0;                                       // HPRE = /1 (no AHB division)
    RCC->CDCFGR2 = RCC_CDCFGR2_CDPPRE1_DIV2                 // APB1 = AHB / 2
                 | RCC_CDCFGR2_CDPPRE2_DIV2;                // APB2 = AHB / 2

    // ---- Switch system clock source to PLL1 ----
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL1;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL1);  // Wait until switch is confirmed

    // Update the CMSIS global variable so HAL/delay functions know the clock speed
    SystemCoreClock = 280000000;
}