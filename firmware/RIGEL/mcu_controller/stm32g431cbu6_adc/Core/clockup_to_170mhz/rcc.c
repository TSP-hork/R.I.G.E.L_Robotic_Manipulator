#include "stm32g4xx.h"
#include "rcc.h"

void RCC_Init(void) {
    // 1. Enable FPU (Floating Point Unit) - critical for math!
    SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));

    // 2. Configure Flash Latency (required for 170 MHz operation)
    FLASH->ACR &= ~FLASH_ACR_LATENCY;
    FLASH->ACR |= FLASH_ACR_LATENCY_4WS; // 4 Wait States

    // 3. Enable HSI (Internal High Speed Clock 16 MHz)
    RCC->CR |= RCC_CR_HSION;
    while(!(RCC->CR & RCC_CR_HSIRDY));

    // 4. Configure PLL
    // Input 16 MHz -> PLLM (/4) -> 4 MHz -> PLLN (*85) -> 340 MHz -> PLLR (/2) -> 170 MHz
    RCC->PLLCFGR = 0;
    RCC->PLLCFGR |= (3 << RCC_PLLCFGR_PLLM_Pos);  // /4
    RCC->PLLCFGR |= (85 << RCC_PLLCFGR_PLLN_Pos); // *85
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLREN;           // Enable R output
    RCC->PLLCFGR |= (2 << RCC_PLLCFGR_PLLSRC_Pos); // Source = HSI

    // 5. Enable PLL
    RCC->CR |= RCC_CR_PLLON;
    while(!(RCC->CR & RCC_CR_PLLRDY));

    // 6. Switch System Clock to PLL
    RCC->CFGR |= (3 << RCC_CFGR_SW_Pos);
    while ((RCC->CFGR & RCC_CFGR_SWS) != (3 << RCC_CFGR_SWS_Pos));
    
}