#ifndef __CLOCK__
#define __CLOCK__

#include "stm32f1xx.h"
#include "Clock_INRQ.h"

class Clock {
private:
  void configure_APB1_clock(Clock_configuration_INRQ header);
  void configure_APB2_clock(Clock_configuration_INRQ header);
  void configure_AHB_clock(Clock_configuration_INRQ header);
  void configure_clock_source(Clock_INRQ header);
public:
  void __init__(Clock_INRQ header);
  void reset_clock();
};

#endif
