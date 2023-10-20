#ifndef __CAN_INRA__
#define __CAN_INRA__

#include "stm32f1xx.h"

typedef struct {
  uint8_t filter_bank1;
  uint8_t filter_bank2;

  uint8_t filter_position;

  uint8_t list_mode_enable; // or else it will be mask mode

  uint8_t scale_32_enable; // or else two 16bit banks

  uint8_t fifo1_enable; // or else fifo0

  uint8_t activate_filter;
} CAN_filter_INRQ;

typedef struct {
  uint8_t time_segment_1;
  uint8_t time_segment_2;
  uint32_t SJW;
  uint32_t loop_back;
  uint8_t prescaler;
  uint8_t silent_mode;
} CAN_bit_timing_INRQ;

typedef struct {
  uint16_t tx_ID;
  uint8_t data_length;
} CAN_Tx_INRQ;

typedef struct {
  uint16_t rx_ID;
  uint8_t data_length;
  uint8_t remote_transmission_req;
  uint8_t extended_id;
} CAN_Rx_INRQ;

typedef struct {
  CAN_Tx_INRQ can_tx;
  CAN_Rx_INRQ can_rx;
  CAN_bit_timing_INRQ can_bit_timing;
  CAN_filter_INRQ can_filter;
} CAN_INRQ;


#endif
