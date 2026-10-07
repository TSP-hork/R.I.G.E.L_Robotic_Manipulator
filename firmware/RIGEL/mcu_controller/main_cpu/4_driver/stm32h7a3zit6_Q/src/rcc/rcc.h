#ifndef RCC_H
#define RCC_H

#include "stm32h7xx.h"

// Initialize the full STM32H7 clock tree for 280 MHz operation.
// Configures: SMPS power supply, VOS0 voltage scaling, HSE bypass (8 MHz),
// PLL1 (8/1 × 70 / 2 = 280 MHz SYSCLK), flash wait states (6 WS),
// and bus dividers (AHB=280, APB1=140, APB2=140, APB4=280 MHz).
// Must be called early in main() before any peripheral initialization.
void rcc_init(void);

#endif // RCC_H