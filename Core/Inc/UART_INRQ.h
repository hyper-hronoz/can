#ifndef __UART_INRQ__
#define __UART_INRQ__

#include "stm32f1xx.h"

typedef struct
{
  uint32_t bus_freq;
  uint32_t baudrate;
  uint8_t enable_word_9bit; // else 8bit
  uint8_t stop_bits_amount; 
  uint8_t enable_parity;   

  uint8_t enable_transmitter;
  uint8_t enable_reciever;
} UART_INRQ;


#endif
