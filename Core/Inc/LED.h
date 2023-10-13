#ifndef __LED__
#define __LED__

#include "stm32f1xx.h"

class LED {
public:
  void __init__();

  void led_on();

  void led_off();

  void led_toggle();
};

#endif
