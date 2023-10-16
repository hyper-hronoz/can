#include "Clock.h"
#include "Delay.h"
#include "LED.h"

void Clock::configure_APB1_clock(Clock_configuration_INRQ header) {
  RCC->CIR &= ~(RCC_CFGR_PPRE1_Msk);
  RCC->CIR |= (header.PLL_prescaler << RCC_CFGR_PPRE1_Pos);
}

void Clock::configure_APB2_clock(Clock_configuration_INRQ header) {}

void Clock::configure_clock_source(Clock_INRQ header) {
  RCC->CR |= RCC_CR_HSION;
  while (!(RCC->CR & RCC_CR_HSIRDY)) {
  }

  if (header.clock_control_INRQ.enable_PLL) {
    RCC->CFGR &= ~(RCC_CFGR_PLLMULL_Msk);
    RCC->CFGR |= (header.clock_configuration_INRQ.PLL_multiplier
                  << RCC_CFGR_PLLMULL_Pos);

    RCC->CFGR &= ~(RCC_CFGR_SW_Msk);
    RCC->CFGR |=
        (header.clock_configuration_INRQ.clock_source << RCC_CFGR_SW_Pos);

    RCC->CFGR &= ~(RCC_CFGR_PLLSRC);
    RCC->CFGR |=
        (header.clock_configuration_INRQ.PLL_enable_HSE << RCC_CFGR_PLLSRC_Pos);

    RCC->CR |= RCC_CR_PLLON;

    // for (uint8_t i = 0;; i++) {
    //   if (RCC->CR & (1U << RCC_CR_PLLRDY_Pos))
    //     break;
    //   if (i == 255) {
    //     RCC->CR &= ~(1U << RCC_CR_PLLON_Pos);
    //   }
    // }

    uint8_t status = Delay().timeout(RCC->CR, RCC_CR_PLLRDY, 255);
    if (status == 1) {
      LED().led_timeout_exception();
    }
  }
}

void Clock::__init__(Clock_INRQ header) {
  this->configure_APB1_clock(header.clock_configuration_INRQ);
  this->configure_APB2_clock(header.clock_configuration_INRQ);

  this->configure_clock_source(header);
}
