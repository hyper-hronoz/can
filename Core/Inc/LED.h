#ifndef __LED__
#define __LED__

#include "stm32f1xx.h"
#include "Delay.h"

class LED {
public:
  void __init__();

  void led_on();

  void led_off();

  void led_toggle();

  void led_timeout_exception();
};

#endif
