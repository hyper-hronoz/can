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

uint8_t can_send2(uint8_t *pData, uint8_t dataLength) {
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
      // else return ((CAN1->ESR & CAN_ESR_LEC)>>CAN_ESR_LEC_Pos); / return Last
      // error code /
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
  CAN1->sTxMailBox[0].TIR |= CAN_TI0R_TXRQ; /* Transmit Mailbox Request */
  uint8_t error_code = ((CAN1->ESR & CAN_ESR_LEC) >> CAN_ESR_LEC_Pos);
  if (CAN1->TSR & CAN_TSR_TXOK0)
    return 0;
  else
    return error_code; /* return Last error code */
}

class CAN {
public:
  inline CAN() {
    RCC->APB2ENR = 0;
    RCC->APB1ENR = 0;

    RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

    CAN_TxHeaderTypeDef header;

    header.IDE = CAN_ID_STD;   // setting ID to Standard ID
    header.StdId = 0x446;      // Identifier
    header.RTR = CAN_RTR_DATA; // Setting Remote Transmission Request
    header.DLC = 2;            // Data Length of Data bytes

    /* PA11 - CAN_RX */ // as laternate function
    // GPIOA->CRH	&= ~GPIO_CRH_CNF11;   /* CNF11 = 00 */
    // GPIOA->CRH	|= GPIO_CRH_CNF11_1;  /* CNF11 = 10 -> AF Out |
    // Push-pull (CAN_RX) */ GPIOA->CRH 	|= GPIO_CRH_MODE11;   /* MODE8 =
    // 11 -> Maximum output speed 50 MHz */

    GPIOA->CRH &= ~GPIO_CRH_MODE11_Msk;
    GPIOA->CRH &= ~(GPIO_CRH_CNF11_Msk);
    GPIOA->CRH |= GPIO_CRH_CNF11_1;
    // pull up
    // GPIOA->ODR	|= GPIO_ODR_ODR11;  /* CNF11 = 10 -> AF Out | Push-pull
    // (CAN_RX) */ pull down GPIOA->ODR	&= ~(GPIO_ODR_ODR11);  /* CNF11 = 10 ->
    // AF Out | Push-pull (CAN_RX) */

    /* PA12 - CAN_TX */
    GPIOA->CRH &= ~GPIO_CRH_CNF12; /* CNF12 = 00 */
    GPIOA->CRH |=
        GPIO_CRH_CNF12_1; /* CNF12 = 10 -> AF Out | Push-pull (CAN_TX) */
    GPIOA->CRH |=
        GPIO_CRH_MODE12; /* MODE8 = 11 -> Maximum output speed 50 MHz */

    CAN1->MCR |= CAN_MCR_INRQ;

    while (!(CAN1->MSR & CAN_MSR_INAK)) {
    };

    CAN1->MCR |= CAN_MCR_NART;
    CAN1->MCR |= CAN_MCR_AWUM;

    // disable silent mode
    // CAN1->BTR &= ~(CAN_BTR_SILM_Msk);
    // disable loop back mode
    // CAN1->BTR &= ~(CAN_BTR_LBKM_Msk);

    // disable silent mode
    // CAN1->BTR |= CAN_BTR_SILM_Msk;
    // disable loop back mode
    CAN1->BTR |= CAN_BTR_LBKM;

    CAN1->BTR &= ~(CAN_BTR_BRP);
    CAN1->BTR |= ((18 - 1) << CAN_BTR_BRP_Pos);

    CAN1->BTR &= ~(CAN_BTR_TS1_Msk);
    CAN1->BTR |= ((13 - 1) << CAN_BTR_TS1_Pos);

    CAN1->BTR &= ~(CAN_BTR_TS2_Msk);
    CAN1->BTR |= ((2 - 1) << CAN_BTR_TS2_Pos);

    CAN1->BTR |= CAN_BTR_SJW_1;

    CAN1->sTxMailBox[0].TIR &= ~CAN_TI0R_RTR;
    CAN1->sTxMailBox[0].TIR &= ~CAN_TI0R_IDE;

    CAN1->sTxMailBox[0].TIR &= ~CAN_TI0R_STID;
    CAN1->sTxMailBox[0].TIR |= (777 << CAN_TI0R_STID_Pos);

    CAN1->sTxMailBox[0].TDTR &= ~CAN_TDT0R_DLC;
    CAN1->sTxMailBox[0].TDTR |= (8 << CAN_TDT0R_DLC_Pos);

    CAN1->MCR &= ~CAN_MCR_INRQ;
  }

  uint8_t send(uint8_t *pData, uint8_t dataLength) {
    // waiting until it get empty
    while (!(CAN1->TSR & CAN_TSR_TME0)) {
    }

    CAN1->sTxMailBox[0].TDLR = 'I';
    CAN1->sTxMailBox[0].TDHR = 'I';

    CAN1->sTxMailBox[0].TIR |= CAN_TI0R_TXRQ;

    // waiting until request for mailbox 1 complited
    while (!(CAN1->MSR & (0b1 << 0))) {
    };

    uint8_t is_transmission_ok = CAN1->TSR & CAN_TSR_TXOK0;
    uint8_t error = (CAN1->ESR & CAN_ESR_LEC) >> CAN_ESR_LEC_Pos;
    printf("I am slizer");
  }
};

int main() {
  Clock clock;

  LED led;
  led.led_on();

  SystemCoreClockUpdate();
  __IO uint32_t clock_value = SystemCoreClock;

  CAN can;

  uint8_t temp = 0;
  uint8_t data[] =
      "all aboard kiss by the iron fiest we are sainted by the storm";

  volatile uint16_t counter = 0;

  // can.send(data, sizeof(data));
  // can_send2(data, sizeof(data));
  // can_send2(data, sizeof(data));
  while (1) {
    can_send2(data, sizeof(data));
    // can_send2(data, sizeof(data));
    // uint8_t error_code = can.send(data, sizeof(data));
  }

  return 0;
}
