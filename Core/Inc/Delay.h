#ifndef __DELAY__
#define __DELAY__

#include "stm32f1xx.h"

class Delay {
public:
  void __init__();

  void wait(uint32_t ms);
};

#endif
