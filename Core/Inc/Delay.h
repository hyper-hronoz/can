#ifndef __DELAY__
#define __DELAY__

#include "stm32f1xx.h"

class Delay {
public:
  void __init__(uint8_t prescaler);

  void wait(uint32_t ms);

  uint8_t timeout(volatile uint32_t &reg, const  uint32_t comparable, volatile uint32_t timeout);
};

#endif
