#include "LED.h"

void LED::__init__() {
  RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;

  GPIOC->CRH |= (GPIO_CRH_MODE13_0 | GPIO_CRH_MODE13_1);
  GPIOC->CRH &= ~(GPIO_CRH_CNF13_0 | GPIO_CRH_CNF13_1);

  this->led_off();
}

void LED::led_on() { GPIOC->ODR &= ~(GPIO_ODR_ODR13); }

void LED::led_off() { GPIOC->ODR |= GPIO_ODR_ODR13; }

void LED::led_toggle() { GPIOC->ODR ^= GPIO_ODR_ODR13; }
