#ifndef __UART__
#define __UART__

#include "stm32f1xx.h"

class UART {
  private:
    void configure_GPIO_recieve() {
      GPIOB->CRL &= ~(GPIO_CRL_MODE7_Msk);
      GPIOB->CRL |= GPIO_CRL_MODE7;
      GPIOB->CRL &= ~(GPIO_CRL_CNF7_Msk);
      GPIOB->CRL |= GPIO_CRL_CNF7_1;
    }

    void configure_GPIO_transmit() {
      GPIOB->CRL &= ~(GPIO_CRL_MODE6_Msk);
      GPIOB->CRL |= GPIO_CRL_MODE6;
      GPIOB->CRL &= ~(GPIO_CRL_CNF6_Msk);
      GPIOB->CRL |= GPIO_CRL_CNF6_1;
    }

    void configure_UART() {
    }

  public:
    void __init__() {
      RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

      this->configure_GPIO_recieve();
      this->configure_GPIO_transmit();
    }

    void recieve() {

    }

    void transmit() {

    }
};

#endif
