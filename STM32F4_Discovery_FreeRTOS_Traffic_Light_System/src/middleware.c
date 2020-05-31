#include "stm32f4xx.h"

void writeBoard(uint32_t boardState) {
	for(int i = 0; i<32;i++){
		uint32_t nextBit = 0x1 & (boardState >> i);
		GPIO_ToggleBits(GPIOB, GPIO_Pin_1);
		GPIO_Write(GPIOB, nextBit);
		GPIO_ToggleBits(GPIOB, GPIO_Pin_1);
	}
}

uint16_t getPotValue() {
  ADC_SoftwareStartConv(ADC1);
  while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC)) {
	return ADC_GetConversionValue(ADC1);
  }
  return 0;
}
