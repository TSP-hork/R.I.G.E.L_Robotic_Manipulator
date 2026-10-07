#ifndef ADC_H
#define ADC_H

#include <stdint.h>

extern volatile uint32_t adc_dual_data;

void ADC_Start(void);
void ADC_Init(void);

#endif