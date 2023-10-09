#include "stm32f103xb.h"
#include "stm32f1xx.h"
#include <stdio.h>
#include <stdlib.h>

#define CAN_ID_STD (0x00000000U)
#define CAN_RTR_DATA (0x00000000U)

#define DATA_LENGTH_CODE 8

class Clock {
public:
  inline Clock() {
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
  Delay() {
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
  inline LED() {
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;

    GPIOC->CRH |= (GPIO_CRH_MODE13_0 | GPIO_CRH_MODE13_1);
    GPIOC->CRH &= ~(GPIO_CRH_CNF13_0 | GPIO_CRH_CNF13_1);

    this->led_off();
  }

  inline void led_on() { GPIOC->ODR &= ~(GPIO_ODR_ODR13); }

  inline void led_off() { GPIOC->ODR |= GPIO_ODR_ODR13; }
};

typedef struct {
  uint32_t StdId;

  uint32_t ExtId;

  uint32_t IDE;

  uint32_t RTR;

  uint32_t DLC;

  uint32_t Timestamp;

  uint32_t FilterMatchIndex;

} CAN_RxHeaderTypeDef;

typedef struct {
  uint32_t StdId;

  uint32_t ExtId;

  uint32_t IDE;

  uint32_t RTR;

  uint32_t DLC;

  FunctionalState TransmitGlobalTime;

} CAN_TxHeaderTypeDef;

typedef struct {
  uint8_t prescaler;
  uint8_t time_segment_1;
  uint8_t time_segment_2;
  uint32_t SJW;
  uint32_t loop_back;
  uint16_t tx_ID;
  uint16_t rx_ID;
} CAN_INRQ;

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

  void configure_can_timings(CAN_INRQ header) {
    CAN1->BTR |= (header.loop_back << CAN_BTR_LBKM_Pos);

    CAN1->BTR &= ~(CAN_BTR_BRP);
    CAN1->BTR |= ((header.prescaler - 1) << CAN_BTR_BRP_Pos);

    CAN1->BTR &= ~(CAN_BTR_TS1_Msk);
    CAN1->BTR |= ((header.time_segment_1 - 1) << CAN_BTR_TS1_Pos);

    CAN1->BTR &= ~(CAN_BTR_TS2_Msk);
    CAN1->BTR |= ((header.time_segment_2 - 1) << CAN_BTR_TS2_Pos);

    CAN1->BTR |= (header.SJW << CAN_BTR_SJW_Pos);
  }

  void configure_can_tx_mailboxes(CAN_INRQ header) {
    CAN1->sTxMailBox[0].TIR &= ~CAN_TI0R_RTR;
    CAN1->sTxMailBox[0].TIR &= ~CAN_TI0R_IDE;

    CAN1->sTxMailBox[0].TIR &= ~CAN_TI0R_STID;
    CAN1->sTxMailBox[0].TIR |= (header.tx_ID << CAN_TI0R_STID_Pos);

    CAN1->sTxMailBox[0].TDTR &= ~CAN_TDT0R_DLC;
    CAN1->sTxMailBox[0].TDTR |= (8 << CAN_TDT0R_DLC_Pos);
  }

  void configure_recieving() {
    CAN1->sFIFOMailBox[0].RIR &= ~(CAN_RI1R_IDE);
    CAN1->sFIFOMailBox[0].RIR &= ~(CAN_RTR_DATA);

    CAN1->sFIFOMailBox[0].RDTR &= ~(CAN_RDT0R_FMI_Msk);
    CAN1->sFIFOMailBox[0].RDTR |= (0 << CAN_RDT0R_FMI_Pos);

    CAN1->sFIFOMailBox[0].RDTR &= ~(CAN_RDT0R_DLC_Msk);
    CAN1->sFIFOMailBox[0].RDTR |= (8 << CAN_RDT0R_DLC_Pos);
  }

  void configure_can_filter() {
    CAN1->FMR |= CAN_FMR_FINIT;

    // list mode strict filtering
    CAN1->FM1R |= CAN_FM1R_FBM0;

    // filter scale singl 32 bit
    CAN1->FS1R |= CAN_FS1R_FSC0;

    // filter 0 id 2
    CAN1->sFilterRegister[0].FR1 = 2;

    // filter 0 to fifo0
    CAN1->FFA1R &= ~(CAN_FFA1R_FFA0);

    // activate filter 0
    CAN1->FA1R |= CAN_FA1R_FACT0;

    CAN1->FMR &= ~(CAN_FMR_FINIT);
  }

  void can_INRQ(CAN_INRQ header) {

    CAN1->MCR |= CAN_MCR_INRQ;

    while (!(CAN1->MSR & CAN_MSR_INAK)) {
    };

    CAN1->MCR |= CAN_MCR_NART;
    CAN1->MCR |= CAN_MCR_AWUM;

    this->configure_can_timings(header);
    this->configure_can_tx_mailboxes(header);
    this->configure_can_filter();

    CAN1->MCR &= ~CAN_MCR_INRQ;
  }

public:
  inline CAN(CAN_INRQ header) {
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

  uint8_t recieve() {}
};

int main() {
  Clock clock;

  LED led;
  led.led_on();

  SystemCoreClockUpdate();
  __IO uint32_t clock_value = SystemCoreClock;

  CAN_INRQ inrq_config;
  inrq_config.prescaler = 18;
  inrq_config.time_segment_1 = 13;
  inrq_config.time_segment_2 = 2;
  inrq_config.SJW = 1;
  inrq_config.loop_back = 1;
  inrq_config.tx_ID = 2;

  CAN can(inrq_config);

  uint8_t temp = 0;
  uint8_t data[] =
      "all aboard kiss by the iron fiest we are sainted by the storm";

  volatile uint16_t counter = 0;

  // 125	0.0000	18	16	13	2	87.5	 0x001c0011

  while (1) {
    can.transmit(data, sizeof(data));
    Delay().wait(1000);
  }

  return 0;
}
