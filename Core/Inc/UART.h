#ifndef __UART__
#define __UART__

#include "stm32f1xx.h"
#include "UART_INRQ.h"
#include "Delay.h"

class UART {
  public:
    static uint8_t index;
    static uint8_t buffer_fifo[8];
  private:
    void configure_GPIO_recieve();

    void configure_GPIO_transmit();

    void configure_UART(UART_INRQ header);

  public:
    void __init__(UART_INRQ header);

    void recieve();

    void transmit(uint8_t *data, uint32_t size);
};

#endif
