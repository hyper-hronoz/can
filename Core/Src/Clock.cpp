#include "Clock.h"

void Clock::__init__() {
  for (uint8_t i = 0;; i++) {
    if (RCC->CR & (1 << RCC_CR_HSIRDY_Pos))
      break;
    if (i == 255)
      return;
  }
  RCC->CFGR |= RCC_CFGR_PLLMULL9;
  RCC->CFGR |= RCC_CFGR_SW_1;
  RCC->CFGR &= ~(RCC_CFGR_PLLSRC);
  RCC->CR |= RCC_CR_PLLON;
  for (uint8_t i = 0;; i++) {
    if (RCC->CR & (1U << RCC_CR_PLLRDY_Pos))
      break;
    if (i == 255) {
      RCC->CR &= ~(1U << RCC_CR_PLLON_Pos);
    }
  }
}
