#include "stm32f1xx.h"
#include <stdlib.h>

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

class CAN {
public:
  inline CAN() {
    RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

    GPIOA->CRH &= ~GPIO_CRH_CNF11;
    GPIOA->CRH |= GPIO_CRH_CNF11_1;
    GPIOA->CRH |= GPIO_CRH_MODE11;
    GPIOA->CRH &= ~GPIO_CRH_CNF12;
    GPIOA->CRH |= GPIO_CRH_CNF12_1;
    GPIOA->CRH |= GPIO_CRH_MODE12;

    CAN1->MCR |= CAN_MCR_INRQ;

    CAN1->MCR |= CAN_MCR_NART;

    CAN1->MCR |= CAN_MCR_AWUM;

    CAN1->BTR &= ~CAN_BTR_BRP;
    CAN1->BTR |= 8U << CAN_BTR_BRP_Pos;

    CAN1->BTR &= ~(0xFU << CAN_BTR_TS1_Pos);
    CAN1->BTR |= 12U << CAN_BTR_TS1_Pos;

    CAN1->BTR &= ~(7U << CAN_BTR_TS2_Pos);
    CAN1->BTR |= 1U << CAN_BTR_TS2_Pos;

    CAN1->sTxMailBox[0].TIR &= ~CAN_TI0R_RTR;

    CAN1->sTxMailBox[0].TIR &= ~CAN_TI0R_IDE;
    CAN1->sTxMailBox[0].TIR |= (0x556U << CAN_TI0R_STID_Pos);

    CAN1->sTxMailBox[0].TDTR &= ~CAN_TDT0R_DLC;
    CAN1->sTxMailBox[0].TDTR |= (DATA_LENGTH_CODE << CAN_TDT0R_DLC_Pos);

    CAN1->MCR &= ~CAN_MCR_INRQ;
  }

  uint8_t send(char *pData, uint8_t dataLength) {
    uint16_t i = 0;
    uint8_t j = 0;

    while (!(CAN1->TSR & CAN_TSR_TME0)) {
      i++;
      if (i > 0xEFFF)
        return 1;
    }

    CAN1->sTxMailBox[0].TDLR = 0;
    CAN1->sTxMailBox[0].TDHR = 0;

    while (i < dataLength) {
      if (i > (DATA_LENGTH_CODE - 1)) {
        CAN1->sTxMailBox[0].TIR |= CAN_TI0R_TXRQ; /* Transmit Mailbox Request */
        dataLength -= i;
        j++;
        while (!(CAN1->TSR & CAN_TSR_TME0)) { /* Transmit mailbox 0 empty? */
          i++;
          if (i > 0xEFFF)
            return 1;
        }
        if (CAN1->TSR & CAN_TSR_TXOK0) {
        } /* Tx OK? */
        // else return ((CAN1->ESR & CAN_ESR_LEC)>>CAN_ESR_LEC_Pos); /* return
        // Last error code */
        i = 0;
        CAN1->sTxMailBox[0].TDLR = 0;
        CAN1->sTxMailBox[0].TDHR = 0;
      }
      if (i < 4) {
        CAN1->sTxMailBox[0].TDLR |=
            (pData[i + j * DATA_LENGTH_CODE] * 1U << (i * 8));
      } else {
        CAN1->sTxMailBox[0].TDHR |=
            (pData[i + j * DATA_LENGTH_CODE] * 1U << (i * 8 - 32));
      }
      i++;
    }

    CAN1->sTxMailBox[0].TIR |= CAN_TI0R_TXRQ;
    if (CAN1->TSR & CAN_TSR_TXOK0)
      return 0;
    else
      return ((CAN1->ESR & CAN_ESR_LEC) >> CAN_ESR_LEC_Pos);

    return 0;
  }
};

int main() {
  Clock clock;

  LED led;
  led.led_on();

  CAN can;

  char *data = "110";

  uint32_t counter = 0;
  can.send(data, sizeof(data));
  while (counter < 0xFFFF)
    counter++;
  counter = 0;

  return 0;
}
