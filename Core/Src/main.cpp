#include "stm32f1xx.h"

class Led {
public:
  inline Led() {
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;

    GPIOC->CRH |= (GPIO_CRH_MODE13_0 | GPIO_CRH_MODE13_1);
    GPIOC->CRH &= ~(GPIO_CRH_CNF13_0 | GPIO_CRH_CNF13_1);

    this->led_off();
  }

  inline void led_on() {
    GPIOC->ODR &= ~(0b1 << 13);
  }

  inline void led_off() {
    GPIOC->ODR |= GPIO_ODR_ODR13;
  }
};

int main() {
  Led led;
  led.led_on();
  return 0; 
}
