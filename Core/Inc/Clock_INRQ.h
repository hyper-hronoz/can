#ifndef __CLOCK_INRQ__
#define __CLOCK_INRQ__

#include "stm32f1xx.h"

typedef struct {
  uint8_t enable_HSE;
  uint8_t enable_HSI;
  uint8_t enable_PLL;

  uint8_t enbale_CSS;
  uint8_t enalbe_HSEBYP; // hse bypass enable
} Clock_control_INRQ;

enum Clock_system_source_selector {
  HSI = 0b00,
  HSE = 0b01,
  PLL = 0b10,
};

typedef struct {
  uint8_t APB1_prescaler;
  uint8_t APB2_prescaler;
  uint8_t AHB_prescaler;

  uint8_t PLL_prescaler;
  uint8_t PLL_multiplier;
  uint8_t PLL_enable_HSE;

  Clock_system_source_selector clock_source;
} Clock_configuration_INRQ;

typedef struct {
  Clock_configuration_INRQ clock_configuration_INRQ;
  Clock_control_INRQ clock_control_INRQ;
} Clock_INRQ;

#endif
