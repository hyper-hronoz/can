#include "CAN_INRQ.h"
#include "Clock.h"
#include "Clock_INRQ.h"
#include "Delay.h"
#include "LED.h"
#include "UART.h"
#include "stm32f103xb.h"
#include "stm32f1xx.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <cstring>

UART_INRQ UART_header;
CAN_INRQ can_header;

Clock_INRQ clock_header;

struct Node_can_settings {
  uint8_t filter_id = 2 << 5;
  uint8_t transmit_node_id = 1;
};

Node_can_settings node_settings;

void strrev(uint8_t (&str)[8]) {
  // if the string is empty
  if (!str) {
    return;
  }
  // pointer to start and end at the string
  uint32_t i = 0;
  uint32_t j = sizeof(str) - 1;

  // reversing string
  while (i < j) {
    uint8_t c = str[i];
    str[i] = str[j];
    str[j] = c;
    i++;
    j--;
  }
}

class CAN {
private:
  void configure_gpio_rx() {
    GPIOA->CRH &= ~(GPIO_CRH_MODE11_Msk);
    GPIOA->CRH &= ~(GPIO_CRH_CNF11_Msk);
    GPIOA->CRH |= GPIO_CRH_CNF11_1;
    GPIOA->ODR |= GPIO_ODR_ODR11;
  }

  void configure_gpio_tx() {
    GPIOA->CRH &= ~GPIO_CRH_CNF12;
    GPIOA->CRH |= GPIO_CRH_CNF12_1;
    GPIOA->CRH |= GPIO_CRH_MODE12;
  }

  void configure_gpio() {
    this->configure_gpio_tx();
    this->configure_gpio_rx();
  }

  void configure_timings(CAN_bit_timing_INRQ header) {
    CAN1->BTR &= ~(CAN_BTR_LBKM_Msk);
    CAN1->BTR |= (header.loop_back << CAN_BTR_LBKM_Pos);

    CAN1->BTR &= ~(CAN_BTR_SILM_Msk);
    CAN1->BTR |= (header.silent_mode << CAN_BTR_SILM_Pos);

    CAN1->BTR &= ~(CAN_BTR_BRP);
    CAN1->BTR |= ((header.prescaler - 1) << CAN_BTR_BRP_Pos);

    CAN1->BTR &= ~(CAN_BTR_TS1_Msk);
    CAN1->BTR |= ((header.time_segment_1 - 1) << CAN_BTR_TS1_Pos);

    CAN1->BTR &= ~(CAN_BTR_TS2_Msk);
    CAN1->BTR |= ((header.time_segment_2 - 1) << CAN_BTR_TS2_Pos);

    CAN1->BTR |= (header.SJW << CAN_BTR_SJW_Pos);
  }

  void configure_tx_mailboxes(CAN_Tx_INRQ header) {
    CAN1->sTxMailBox[0].TIR &= ~CAN_TI0R_RTR;
    CAN1->sTxMailBox[0].TIR &= ~CAN_TI0R_IDE;

    CAN1->sTxMailBox[0].TIR &= ~CAN_TI0R_STID;
    CAN1->sTxMailBox[0].TIR |= (header.tx_ID << CAN_TI0R_STID_Pos);

    CAN1->sTxMailBox[0].TDTR &= ~CAN_TDT0R_DLC;
    CAN1->sTxMailBox[0].TDTR |= (header.data_length << CAN_TDT0R_DLC_Pos);
  }

  void configure_recieving(CAN_Rx_INRQ header) {
    // identifier
    CAN1->sFIFOMailBox[0].RIR &= ~(CAN_RI1R_IDE);
    CAN1->sFIFOMailBox[0].RIR |= (header.extended_id << CAN_RI1R_IDE_Pos);


    // remove transmission request
    CAN1->sFIFOMailBox[0].RIR &= ~(CAN_RI1R_RTR_Msk); // data frame
    CAN1->sFIFOMailBox[0].RIR |=
        (header.remote_transmission_req << CAN_RI1R_RTR_Pos); // data frame
                                                              //
    // extended id
    CAN1->sFIFOMailBox[0].RIR |=
        (header.extended_id << CAN_RI1R_IDE_Pos);

    // configuring filter for mailbox ex: 0 - index of filter
    CAN1->sFIFOMailBox[0].RDTR &= ~(CAN_RDT0R_FMI_Msk);
    CAN1->sFIFOMailBox[0].RDTR |= (header.rx_ID << CAN_RDT0R_FMI_Pos);

    // configure data length in bytes
    CAN1->sFIFOMailBox[0].RDTR &= ~(CAN_RDT0R_DLC_Msk);
    CAN1->sFIFOMailBox[0].RDTR |= (header.data_length << CAN_RDT0R_DLC_Pos);
  }

  void filter_to_two_16bit_scale() { CAN1->FS1R &= ~(CAN_FS1R_FSC0); }

  void configure_filter(CAN_filter_INRQ header) {
    CAN1->FMR |= CAN_FMR_FINIT;

    // list mode strict filtering
    CAN1->FM1R &= ~(CAN_FM1R_FBM0);
    CAN1->FM1R |= (header.list_mode_enable << header.filter_position);

    // filter scale
    CAN1->FS1R &= ~(CAN_FS1R_FSC0);
    CAN1->FS1R |= (header.scale_32_enable << header.filter_position);

    CAN1->sFilterRegister[header.filter_position].FR1 = header.filter_bank1;
    CAN1->sFilterRegister[header.filter_position].FR2 = header.filter_bank2;

    CAN1->FFA1R &= ~(0b1 << header.filter_position);
    CAN1->FFA1R |= (header.fifo1_enable << header.filter_position);

    // activate filter
    CAN1->FA1R |= (header.activate_filter << header.filter_position);

    CAN1->FMR &= ~(CAN_FMR_FINIT);
  }

  void configure_interrupts() {
    CAN1->IER |= CAN_IER_FMPIE0;
    CAN1->IER |= CAN_IER_FFIE0;

    NVIC_EnableIRQ(CAN1_RX0_IRQn);
    NVIC_EnableIRQ(CAN1_TX_IRQn);
  }

  void can_INRQ(CAN_INRQ header) {

    CAN1->MCR |= CAN_MCR_INRQ;

    while (!(CAN1->MSR & CAN_MSR_INAK)) {
    };

    CAN1->MCR |= CAN_MCR_NART;
    CAN1->MCR |= CAN_MCR_AWUM;

    this->configure_timings(header.can_bit_timing);
    this->configure_tx_mailboxes(header.can_tx);
    this->configure_recieving(header.can_rx);
    this->configure_filter(header.can_filter);
    this->configure_interrupts();

    CAN1->MCR &= ~CAN_MCR_INRQ;
  }

public:
  inline CAN() {}

  inline void __init__(CAN_INRQ header) {
    RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

    this->configure_gpio();
    this->can_INRQ(header);
  }

  uint8_t transmit(uint8_t *pData, uint8_t dataLength, CAN_Tx_INRQ header) {
    uint16_t i = 0;
    uint8_t j = 0;
    while (!(CAN1->TSR & CAN_TSR_TME0)) {
      i++;
      if (i > 0xEFFF)
        return 1;
    }
    i = 0;
    CAN1->sTxMailBox[0].TDLR = 0;
    CAN1->sTxMailBox[0].TDHR = 0;
    while (i < dataLength) {
      if (i > (header.data_length - 1)) {
        CAN1->sTxMailBox[0].TIR |= CAN_TI0R_TXRQ;
        dataLength -= i;
        j++;
        while (!(CAN1->TSR & CAN_TSR_TME0)) {
          i++;
          if (i > 0xEFFF)
            return 1;
        }
        if (CAN1->TSR & CAN_TSR_TXOK0) {
        }
        i = 0;
        CAN1->sTxMailBox[0].TDLR = 0;
        CAN1->sTxMailBox[0].TDHR = 0;
      }
      if (i < 4)
        CAN1->sTxMailBox[0].TDLR |=
            (pData[i + j * header.data_length] * 1U << (i * 8));
      else
        CAN1->sTxMailBox[0].TDHR |=
            (pData[i + j * header.data_length] * 1U << (i * 8 - 32));
      i++;
    }
    CAN1->sTxMailBox[0].TIR |= CAN_TI0R_TXRQ;
    uint8_t error_code = ((CAN1->ESR & CAN_ESR_LEC) >> CAN_ESR_LEC_Pos);
    uint8_t ts_ok = CAN1->TSR << CAN_TSR_TXOK0_Pos;
    if (ts_ok)
      return 0;
    else
      return error_code;
  }

  void change_trasmission_id(uint8_t mailbox, uint32_t id) {
    CAN1->sTxMailBox[mailbox].TIR &= ~CAN_TI0R_STID;
    CAN1->sTxMailBox[mailbox].TIR |= (id << CAN_TI0R_STID_Pos);
  }

  void recieve(CAN_Rx_INRQ header, uint8_t (&str)[8]) {
    uint32_t recieved_data_hight = CAN1->sFIFOMailBox[0].RDHR;
    uint32_t recieved_data_low = CAN1->sFIFOMailBox[0].RDLR;
    uint8_t messages_amount = CAN1->RF0R << CAN_RF0R_FMP0_Pos;

    uint8_t data[8] = {};
    for (uint8_t i = 0; i < header.data_length; i++) {
      if (i < 4) {
        uint8_t f = recieved_data_low >> (i * header.data_length);
        data[i] = recieved_data_low >> (i * header.data_length);
      } else {
        data[i] = recieved_data_hight >> (i * header.data_length - 32);
      }
    }
    for (uint8_t i = 0; i < header.data_length; i++) {
      str[i] = data[i];
    }
    CAN1->RF0R |= CAN_RF0R_RFOM0;
    messages_amount = CAN1->RF0R << CAN_RF0R_FMP0_Pos;
  }
};

extern "C" void USART1_IRQHandler(void) {
  uint8_t rxd = USART1->DR;
  uint8_t rx_data[1];
  rx_data[0] = rxd;
  uint8_t rx_str[sizeof(UART::buffer_fifo)] = "";
  UART::buffer_fifo[UART::index % sizeof(UART::buffer_fifo)] = rxd;
  UART::index++;
  
  UART().transmit(rx_data, sizeof(rx_data));
  const char *info = (const char *)UART::buffer_fifo;
  if (strstr((const char *)UART::buffer_fifo, "\\0") != NULL) {
    for (uint8_t i = 0; i < sizeof(UART::buffer_fifo) / sizeof(uint8_t); i++) {
      rx_str[i] = UART::buffer_fifo[i];
      UART::buffer_fifo[i] = 0;
    }
    CAN().change_trasmission_id(0, node_settings.transmit_node_id);
    strrev(rx_str);
    CAN_Tx_INRQ info = can_header.can_tx; 
    CAN().transmit(rx_str, sizeof(rx_str), can_header.can_tx);
    // LED().led_toggle();
  }
  return;
}

extern "C" void CAN1_RX0_IRQHandler(void) {
  uint8_t data[8] = "";
  LED().led_toggle();
  CAN().recieve(can_header.can_rx, data);
  CAN().transmit(data, sizeof(data), can_header.can_tx);
  UART().transmit(data, sizeof(data));
}

int main() {
  Delay().__init__(8);
  LED().__init__();

  clock_header.clock_control_INRQ.enable_HSE = 0;
  clock_header.clock_control_INRQ.enable_HSI = 1;
  clock_header.clock_control_INRQ.enable_PLL = 1;
  clock_header.clock_control_INRQ.enbale_CSS = 0;
  clock_header.clock_control_INRQ.enalbe_HSEBYP = 0;

  clock_header.clock_configuration_INRQ.clock_source =
      Clock_system_source_selector::PLL;
  clock_header.clock_configuration_INRQ.PLL_multiplier = 0b0111; // 9
  clock_header.clock_configuration_INRQ.PLL_enable_HSE = 0;
  clock_header.clock_configuration_INRQ.PLL_prescaler = 0;
  clock_header.clock_configuration_INRQ.APB1_prescaler = 0;
  clock_header.clock_configuration_INRQ.APB2_prescaler = 0;

  Clock clock;
  clock.__init__(clock_header);

  Delay().__init__(36);

  LED().led_off();

  SystemCoreClockUpdate();
  __IO uint32_t clock_value = SystemCoreClock;

  // timing configuration
  can_header.can_bit_timing.prescaler = 18;
  can_header.can_bit_timing.time_segment_1 = 13;
  can_header.can_bit_timing.time_segment_2 = 2;
  can_header.can_bit_timing.SJW = 1;
  can_header.can_bit_timing.loop_back = 0;
  can_header.can_bit_timing.silent_mode = 0;

  // transmit configuration
  can_header.can_tx.tx_ID = node_settings.transmit_node_id;
  can_header.can_tx.data_length = 8;

  // recieve configuration
  can_header.can_rx.rx_ID = 0;
  can_header.can_rx.data_length = 8;
  can_header.can_rx.extended_id = 0;
  can_header.can_rx.remote_transmission_req = 0;

  // filter configuration
  can_header.can_filter.fifo1_enable = 0;
  can_header.can_filter.filter_position = 0;
  can_header.can_filter.filter_bank1 = node_settings.filter_id;
  can_header.can_filter.filter_bank2 = node_settings.filter_id;
  can_header.can_filter.scale_32_enable = 0;
  can_header.can_filter.list_mode_enable = 1;
  can_header.can_filter.activate_filter = 1;

  CAN can;
  can.__init__(can_header);

  UART_header.baudrate = 3750;
  UART_header.enable_transmitter = 1;
  UART_header.enable_reciever = 1;
  UART_header.enable_word_9bit = 0;
  UART_header.enable_parity = 0;

  UART uart;
  uart.__init__(UART_header);

  uint8_t temp = 0;
  uint8_t data[] = "some str";
  uint8_t uart_pending[] = ".";

  volatile uint16_t counter = 0;

  // 125	0.0000	18	16	13	2	87.5	 0x001c0011

  // !todo in loop try to change id of transmission i bet it is not
  while (1) {
    // can.transmit(data, sizeof(data), can_header.can_tx);
    // Delay().wait(200);
    // uart.transmit(uart_pending, sizeof(uart_pending));
    // Delay().wait(500);
  }

  return 0;
}
