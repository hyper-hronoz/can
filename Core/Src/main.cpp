#include "CAN_INRQ.h"
#include "stm32f103xb.h"
#include "stm32f1xx.h"
#include <stdio.h>
#include <stdlib.h>


#define CAN_ID_STD (0x00000000U)
#define CAN_RTR_DATA (0x00000000U)

#define DATA_LENGTH_CODE 8

class Clock {
public:
  inline Clock() {}

  inline void __init__() {
    for (uint8_t i = 0;; i++) {
      if (RCC->CR & (1 << RCC_CR_HSIRDY_Pos))
        break;
      if (i == 255)
        return;
    }
    RCC->CFGR |= RCC_CFGR_PLLMULL9;
    RCC->CFGR |= RCC_CFGR_SW_1;
    RCC->CFGR &= ~(RCC_CFGR_PLLSRC);
    RCC->CR |= RCC_CR_PLLON;
    for (uint8_t i = 0;; i++) {
      if (RCC->CR & (1U << RCC_CR_PLLRDY_Pos))
        break;
      if (i == 255) {
        RCC->CR &= ~(1U << RCC_CR_PLLON_Pos);
      }
    }
  }
};

class Delay {
public:
  Delay() {}

  inline void __init__() {
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
    TIM4->PSC = 35;
    TIM4->ARR = 999;
    TIM4->CR1 |= TIM_CR1_CEN;
  }

  void wait(uint32_t ms) {
    for (uint16_t i = 0; i < ms; i++) {
      TIM4->SR &= ~(0b1 << 0);
      while (!(TIM4->SR & (0b1 << 0))) {
      }
    }
  }
};

class LED {
public:
  inline LED() {}

  inline void __init__() {
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;

    GPIOC->CRH |= (GPIO_CRH_MODE13_0 | GPIO_CRH_MODE13_1);
    GPIOC->CRH &= ~(GPIO_CRH_CNF13_0 | GPIO_CRH_CNF13_1);

    this->led_off();
  }

  inline void led_on() { GPIOC->ODR &= ~(GPIO_ODR_ODR13); }

  inline void led_off() { GPIOC->ODR |= GPIO_ODR_ODR13; }

  inline void led_toggle() { GPIOC->ODR ^= GPIO_ODR_ODR13; }
};

class CAN {
private:
  void configure_can_gpio_rx() {
    GPIOA->CRH &= ~GPIO_CRH_CNF11;
    GPIOA->CRH |= GPIO_CRH_CNF11_1;
    GPIOA->CRH |= GPIO_CRH_MODE11;
  }

  void configure_can_gpio_tx() {
    GPIOA->CRH &= ~GPIO_CRH_CNF12;
    GPIOA->CRH |= GPIO_CRH_CNF12_1;
    GPIOA->CRH |= GPIO_CRH_MODE12;
  }

  void configure_can_gpio() {
    this->configure_can_gpio_tx();
    this->configure_can_gpio_rx();
  }

  void configure_can_timings(CAN_bit_timing_INRQ header) {
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

  void configure_can_tx_mailboxes(CAN_Tx_INRQ header) {
    CAN1->sTxMailBox[0].TIR &= ~CAN_TI0R_RTR;
    CAN1->sTxMailBox[0].TIR &= ~CAN_TI0R_IDE;

    CAN1->sTxMailBox[0].TIR &= ~CAN_TI0R_STID;
    CAN1->sTxMailBox[0].TIR |= (header.tx_ID << CAN_TI0R_STID_Pos);

    CAN1->sTxMailBox[0].TDTR &= ~CAN_TDT0R_DLC;
    CAN1->sTxMailBox[0].TDTR |= (header.data_length << CAN_TDT0R_DLC_Pos);
  }

  void configure_recieving(CAN_Rx_INRQ header) {
    // standart identifier
    CAN1->sFIFOMailBox[0].RIR &= ~(CAN_RI1R_IDE);

    // remove transmission request
    CAN1->sFIFOMailBox[0].RIR &= ~(CAN_RTR_DATA); // data frame

    // configuring filter for mailbox ex: 0 - index of filter
    CAN1->sFIFOMailBox[0].RDTR &= ~(CAN_RDT0R_FMI_Msk);
    CAN1->sFIFOMailBox[0].RDTR |= (header.rx_ID << CAN_RDT0R_FMI_Pos);

    // configure data length in bytes
    CAN1->sFIFOMailBox[0].RDTR &= ~(CAN_RDT0R_DLC_Msk);
    CAN1->sFIFOMailBox[0].RDTR |= (header.data_length << CAN_RDT0R_DLC_Pos);
  }

  void filter_to_mask() { CAN1->FM1R &= ~(CAN_FM1R_FBM0); }

  void filter_to_list() {
    CAN1->FM1R &= ~(CAN_FM1R_FBM0);
    CAN1->FM1R |= CAN_FM1R_FBM0;
  }

  void filter_to_fifo0() { CAN1->FFA1R &= ~(CAN_FFA1R_FFA0); }

  void filter_to_fifo1() {
    CAN1->FFA1R &= ~(CAN_FFA1R_FFA0);
    CAN1->FFA1R |= CAN_FFA1R_FFA0;
  }

  void filter_to_32bit_scale() {
    CAN1->FS1R &= ~(CAN_FS1R_FSC0);
    CAN1->FS1R |= CAN_FS1R_FSC0;
  }

  void filter_to_two_16bit_scale() { CAN1->FS1R &= ~(CAN_FS1R_FSC0); }

  void configure_can_filter(CAN_filter_INRQ header) {
    CAN1->FMR |= CAN_FMR_FINIT;

    // list mode strict filtering
    CAN1->FM1R &= ~(CAN_FM1R_FBM0);
    CAN1->FM1R |= (header.list_mode_enable << header.filter_position);

    // filter scale
    CAN1->FS1R &= ~(CAN_FS1R_FSC0);
    CAN1->FS1R |= (header.scale_32_enable << header.filter_position);

    // filter 0 id 2
    CAN1->sFilterRegister[0].FR1 = header.filter_bank1;
    CAN1->sFilterRegister[0].FR2 = header.filter_bank2;

    this->filter_to_fifo0();

    CAN1->FMR &= ~(CAN_FMR_FINIT);

    // activate filter 0
    CAN1->FA1R |= CAN_FA1R_FACT0;
  }

  void configure_can_interrupts() {
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

    this->configure_can_timings(header.can_bit_timing);
    this->configure_can_tx_mailboxes(header.can_tx);
    this->configure_recieving(header.can_rx);
    this->configure_can_filter(header.can_filter);
    this->configure_can_interrupts();

    CAN1->MCR &= ~CAN_MCR_INRQ;
  }

public:
  inline CAN() {}

  inline void __init__(CAN_INRQ header) {
    RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

    this->configure_can_gpio();
    this->can_INRQ(header);
  }

  uint8_t transmit(uint8_t *pData, uint8_t dataLength) {
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
      if (i > (DATA_LENGTH_CODE - 1)) {
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
            (pData[i + j * DATA_LENGTH_CODE] * 1U << (i * 8));
      else
        CAN1->sTxMailBox[0].TDHR |=
            (pData[i + j * DATA_LENGTH_CODE] * 1U << (i * 8 - 32));
      i++;
    }
    CAN1->sTxMailBox[0].TIR |= CAN_TI0R_TXRQ;
    uint8_t error_code = ((CAN1->ESR & CAN_ESR_LEC) >> CAN_ESR_LEC_Pos);
    if (CAN1->TSR & CAN_TSR_TXOK0)
      return 0;
    else
      return error_code;
  }

  void recieve(CAN_Rx_INRQ header) {
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
    CAN1->RF0R |= CAN_RF0R_RFOM0;
    messages_amount = CAN1->RF0R << CAN_RF0R_FMP0_Pos;
  }
};

CAN_INRQ inrq_config;
extern "C" void CAN1_RX0_IRQHandler(void) {
  CAN().recieve(inrq_config.can_rx);
  LED().led_toggle();
}

int main() {
  Clock clock;
  clock.__init__();

  Delay().__init__();

  LED led;
  led.__init__();
  led.led_on();

  SystemCoreClockUpdate();
  __IO uint32_t clock_value = SystemCoreClock;

  // timing configuration
  inrq_config.can_bit_timing.prescaler = 18;
  inrq_config.can_bit_timing.time_segment_1 = 13;
  inrq_config.can_bit_timing.time_segment_2 = 2;
  inrq_config.can_bit_timing.SJW = 1;
  inrq_config.can_bit_timing.loop_back = 1;
  inrq_config.can_bit_timing.silent_mode = 0;

  // transmit configuration
  inrq_config.can_tx.tx_ID = 0;
  inrq_config.can_tx.data_length = 8;

  // recieve configuration
  inrq_config.can_rx.rx_ID = 0;
  inrq_config.can_rx.data_length = 8;

  // filter configuration
  inrq_config.can_filter.filter_bank1 = 0;
  inrq_config.can_filter.filter_bank2 = 0;
  inrq_config.can_filter.scale_32_enable = 1;
  inrq_config.can_filter.list_mode_enable = 1;
  inrq_config.can_filter.filter_position = 0;

  CAN can;
  can.__init__(inrq_config);

  uint8_t temp = 0;
  uint8_t data[] =
      "iother fucker kiss by the iron fiest we are sainted by the storm";

  volatile uint16_t counter = 0;

  // 125	0.0000	18	16	13	2	87.5	 0x001c0011

  while (1) {
    can.transmit(data, sizeof(data));
    Delay().wait(1000);
  }

  return 0;
}
