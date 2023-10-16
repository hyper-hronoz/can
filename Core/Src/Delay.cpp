#include "Delay.h"

void Delay::__init__() {
  RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
  TIM4->PSC = 35;
  TIM4->ARR = 999;
  TIM4->CR1 |= TIM_CR1_CEN;
}

void Delay::wait(uint32_t ms) {
  for (uint16_t i = 0; i < ms; i++) {
    TIM4->SR &= ~(0b1 << 0);
    while (!(TIM4->SR & (0b1 << 0))) {
    }
  }
}

uint8_t Delay::timeout(volatile uint32_t &reg, const uint32_t comparable, volatile uint32_t timeout) {
  for (uint32_t i = 0; i < timeout; i++) {
    if (reg & comparable) {
      return 0;
    } 
  }
  return 1;
}
