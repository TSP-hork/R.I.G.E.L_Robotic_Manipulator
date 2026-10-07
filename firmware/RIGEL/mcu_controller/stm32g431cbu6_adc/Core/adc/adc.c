#include "adc.h"
#include "stm32g4xx.h"

// Buffer to store 32-bit packed ADC results (ADC2 in MSB, ADC1 in LSB)
volatile uint32_t adc_dual_data = 0;

// Command word for DMA trigger: ADSTART to start, keeping ADVREGEN and ADEN set
// Preserving ENABLE and REGULATOR bits is necessary to prevent ADC shutdown during write
static const uint32_t KICK_ADC = ADC_CR_ADSTART | ADC_CR_ADVREGEN | ADC_CR_ADEN;

void ADC_Init(void) {

    // 1. Clock Configuration

    // Enable GPIOA, GPIOB, and ADC1/2 clocks on AHB2 bus
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_ADC12EN;

    // Enable DMA1 and DMAMUX clocks on AHB1 bus
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMAMUX1EN;

    // Enable SYSCFG clock on APB2 (required for EXTI configuration)
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;


    // 2. GPIO Configuration

    // Set PB12 and PB15 to Analog mode (0b11)
    GPIOB->MODER |= (3 << GPIO_MODER_MODE12_Pos) | (3 << GPIO_MODER_MODE15_Pos);


    // 3. EXTI Configuration (PA4 trigger sniffing)

    // Select Port A for EXTI4 in SYSCFG External Interrupt Configuration Register 2
    SYSCFG->EXTICR[1] &= ~SYSCFG_EXTICR2_EXTI4;

    // Disable Interrupt Mask on Line 4 (CPU interrupt not needed)
    EXTI->IMR1 &= ~EXTI_IMR1_IM4; 

    // Enable Event Mask on Line 4 to allow event generation for DMAMUX
    EXTI->EMR1 |= EXTI_EMR1_EM4;  

    // Enable Falling Trigger on Line 4 (triggers on CS falling edge)
    EXTI->FTSR1 |= EXTI_FTSR1_FT4; 


    // 4. ADC Common Configuration

    // DUAL=00110: Regular simultaneous mode
    // MDMA=10: DMA mode 2 (32-bit packed transfer)
    // CKMODE=11: Synchronous clock mode (HCLK/4)
    // DELAY=0010: 2 ADC clock cycles delay between sampling phases
    ADC12_COMMON->CCR = (6 << ADC_CCR_DUAL_Pos) | 
                        (2 << ADC_CCR_MDMA_Pos) | 
                        (3 << ADC_CCR_CKMODE_Pos) |
                        (2 << ADC_CCR_DELAY_Pos);


    // 5. DMA1 Channel 1 Configuration: Data Transfer (ADC -> RAM)

    // Route ADC1 request (ID 5) to DMA1 Channel 1 via DMAMUX
    DMAMUX1_Channel0->CCR = 5;

    // Reset DMA1 Channel 1 control register
    DMA1_Channel1->CCR = 0;
    
    // Configure DMA: 32-bit Memory/Peripheral size, Circular mode, Memory Increment
    DMA1_Channel1->CCR = (2 << DMA_CCR_MSIZE_Pos) | (2 << DMA_CCR_PSIZE_Pos) | DMA_CCR_CIRC;
    
    // Set Peripheral Address to ADC Common Data Register
    DMA1_Channel1->CPAR = (uint32_t)&ADC12_COMMON->CDR;
    
    // Set Memory Address to destination buffer
    DMA1_Channel1->CMAR = (uint32_t)&adc_dual_data;
    
    // Set Data Number to 1 (transfer one 32-bit word per trigger)
    DMA1_Channel1->CNDTR = 1;
    
    // Enable DMA1 Channel 1
    DMA1_Channel1->CCR |= DMA_CCR_EN;


    // 6. DMA1 Channel 3 Configuration: Trigger Mechanism (RAM -> ADC_CR)
    // Configure DMAMUX Request Generator 0: Signal ID 6 (EXTI4), Falling Edge, Enable
    DMAMUX1_RequestGenerator0->RGCR = (6 << DMAMUX_RGxCR_SIG_ID_Pos) | (2 << DMAMUX_RGxCR_GPOL_Pos) | DMAMUX_RGxCR_GE;
    
    // Map DMAMUX Channel 2 (connected to DMA1 Ch3) to Request Generator 0 (ID 1)
    DMAMUX1_Channel2->CCR = 1; 

    // Reset DMA1 Channel 3 control register
    DMA1_Channel3->CCR = 0;

    // Configure DMA: 32-bit size, Dir: Memory->Peripheral, Circular mode
    DMA1_Channel3->CCR = (2 << DMA_CCR_MSIZE_Pos) | (2 << DMA_CCR_PSIZE_Pos) | DMA_CCR_DIR | DMA_CCR_CIRC;
    
    // Set Peripheral Address to ADC1 Control Register
    DMA1_Channel3->CPAR = (uint32_t)&ADC1->CR;

    // Set Memory Address to the command constant
    DMA1_Channel3->CMAR = (uint32_t)&KICK_ADC;

    // Set Data Number to 1 (transfer one command word per trigger)
    DMA1_Channel3->CNDTR = 1;
   
    // Enable DMA1 Channel 3
    DMA1_Channel3->CCR |= DMA_CCR_EN;


    // 7. ADC Power-Up and Calibration
    // Exit Deep Power Down mode for both ADCs
    ADC1->CR &= ~ADC_CR_DEEPPWD; ADC2->CR &= ~ADC_CR_DEEPPWD;
    
    // Enable Internal Voltage Regulator for both ADCs
    ADC1->CR |= ADC_CR_ADVREGEN; ADC2->CR |= ADC_CR_ADVREGEN;

    // Wait for regulator stabilization time (T_ADCVREG_STUP)
    for(volatile int i=0; i<30000; i++);
    
    // Perform calibration for ADC1 (must be done while ADEN=0)
    ADC1->CR |= ADC_CR_ADCAL; while(ADC1->CR & ADC_CR_ADCAL);
    
    // Perform calibration for ADC2
    ADC2->CR |= ADC_CR_ADCAL; while(ADC2->CR & ADC_CR_ADCAL);


    // 8. Конфигурация// 8. ADC Specific Configuration
    // Configure ADC1 (Master): Enable DMA, Circular DMA mode, Overrun mode (overwrite old data)
    ADC1->CFGR = ADC_CFGR_DMAEN | ADC_CFGR_DMACFG | ADC_CFGR_OVRMOD; 
    
    // Configure ADC2 (Slave): Enable Overrun mode only. 
    // DMAEN must be 0 in Slave CFGR when MDMA mode is used in Common register.
    ADC2->CFGR = ADC_CFGR_OVRMOD; 

    // Configure Sample Time: 2.5 ADC clock cycles (000) - FASTEST
    ADC1->SMPR2 &= ~ADC_SMPR2_SMP11; // Clear bits = 000 (2.5 cycles)
    ADC2->SMPR2 &= ~ADC_SMPR2_SMP15; // Clear bits = 000 (2.5 cycles)

    // Configure Sequence Length to 1 conversion (L=0 by default) and select channels
    ADC1->SQR1 = (11 << ADC_SQR1_SQ1_Pos); // Channel 11
    ADC2->SQR1 = (15 << ADC_SQR1_SQ1_Pos); // Channel 15


    // 9. Enable ADCs

    // Enable ADC2 (Slave) first, wait for Ready flag
    ADC2->CR |= ADC_CR_ADEN; while(!(ADC2->ISR & ADC_ISR_ADRDY));

    // Enable ADC1 (Master), wait for Ready flag
    ADC1->CR |= ADC_CR_ADEN; while(!(ADC1->ISR & ADC_ISR_ADRDY));
    
    // Clear all status flags in ISR register to ensure clean start
    ADC1->ISR = 0xFFFFFFFF;
    ADC2->ISR = 0xFFFFFFFF;
}

void ADC_Start(void) {

    // Manually start Master ADC once to arm the system
    // Slave ADC starts automatically in Dual mode
    ADC1->CR |= ADC_CR_ADSTART;
}