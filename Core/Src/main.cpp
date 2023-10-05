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

typedef struct {
  uint32_t StdId; /*!< Specifies the standard identifier.
                       This parameter must be a number between Min_Data = 0 and
                     Max_Data = 0x7FF. */

  uint32_t ExtId; /*!< Specifies the extended identifier.
                       This parameter must be a number between Min_Data = 0 and
                     Max_Data = 0x1FFFFFFF. */

  uint32_t IDE; /*!< Specifies the type of identifier for the message that will
                   be transmitted. This parameter can be a value of @ref
                   CAN_identifier_type */

  uint32_t RTR; /*!< Specifies the type of frame for the message that will be
                   transmitted. This parameter can be a value of @ref
                   CAN_remote_transmission_request */

  uint32_t DLC; /*!< Specifies the length of the frame that will be transmitted.
                     This parameter must be a number between Min_Data = 0 and
                   Max_Data = 8. */

  uint32_t Timestamp; /*!< Specifies the timestamp counter value captured on
                         start of frame reception.
                          @note: Time Triggered Communication Mode must be
                         enabled. This parameter must be a number between
                         Min_Data = 0 and Max_Data = 0xFFFF. */

  uint32_t FilterMatchIndex; /*!< Specifies the index of matching acceptance
                          filter element. This parameter must be a number
                          between Min_Data = 0 and Max_Data = 0xFF. */

} CAN_RxHeaderTypeDef;

typedef struct {
  uint32_t StdId; /*!< Specifies the standard identifier.
                       This parameter must be a number between Min_Data = 0 and
                     Max_Data = 0x7FF. */

  uint32_t ExtId; /*!< Specifies the extended identifier.
                       This parameter must be a number between Min_Data = 0 and
                     Max_Data = 0x1FFFFFFF. */

  uint32_t IDE; /*!< Specifies the type of identifier for the message that will
                   be transmitted. This parameter can be a value of @ref
                   CAN_identifier_type */

  uint32_t RTR; /*!< Specifies the type of frame for the message that will be
                   transmitted. This parameter can be a value of @ref
                   CAN_remote_transmission_request */

  uint32_t DLC; /*!< Specifies the length of the frame that will be transmitted.
                     This parameter must be a number between Min_Data = 0 and
                   Max_Data = 8. */

  FunctionalState
      TransmitGlobalTime; /*!< Specifies whether the timestamp counter value
              captured on start of frame transmission, is sent in DATA6 and
              DATA7 replacing pData[6] and pData[7].
              @note: Time Triggered Communication Mode must be enabled.
              @note: DLC must be programmed as 8 bytes, in order these 2 bytes
              are sent. This parameter can be set to ENABLE or DISABLE. */

} CAN_TxHeaderTypeDef;

class CAN {
public:
  inline CAN() {
    RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

    CAN_TxHeaderTypeDef header;

    header.IDE = CAN_ID_STD;   // setting ID to Standard ID
    header.StdId = 0x446;      // Identifier
    header.RTR = CAN_RTR_DATA; // Setting Remote Transmission Request
    header.DLC = 2;            // Data Length of Data bytes

    // PA11 - CAN_RX
    GPIOA->CRH &= ~(GPIO_CRH_CNF11_Msk);
    // GPIOA->CRH |= GPIO_CRH_CNF11_1;
    GPIOA->ODR |= GPIO_ODR_ODR11;
    GPIOA->CRH |= GPIO_CRH_MODE11_1;

    // PA12 - CAN_TX
    GPIOA->CRH &= ~(GPIO_CRH_CNF12_Msk);
    GPIOA->CRH |= GPIO_CRH_CNF12_1;
    GPIOA->CRH |= GPIO_CRH_MODE12;

    CAN1->MCR |= CAN_MCR_INRQ;

    CAN1->MCR |= CAN_MCR_NART;
    CAN1->MCR |= CAN_MCR_AWUM;

    CAN1->BTR &= ~(CAN_BTR_BRP);

    CAN1->BTR |= ((40 - 1) << CAN_BTR_BRP_Pos);

    CAN1->BTR &= ~(CAN_BTR_TS1_Msk);
    CAN1->BTR |= ((12 - 1) << CAN_BTR_TS1_Pos);

    CAN1->BTR &= ~(CAN_BTR_TS2_Msk);
    CAN1->BTR |= ((2 - 1) << CAN_BTR_TS2_Pos);

    // may be u should use SJW settings
    CAN1->sTxMailBox[0].TIR &= ~CAN_TI0R_RTR;

    // standart identifier
    CAN1->sTxMailBox[0].TIR &= ~CAN_TI0R_IDE;

    CAN1->sTxMailBox[0].TIR &= ~CAN_TI0R_STID;
    CAN1->sTxMailBox[0].TIR |= (777 << CAN_TI0R_STID_Pos);

    // data length 8
    CAN1->sTxMailBox[0].TDTR &= ~CAN_TDT0R_DLC;
    CAN1->sTxMailBox[0].TDTR |= (8 << CAN_TDT0R_DLC_Pos);

    CAN1->MCR &= ~CAN_MCR_INRQ;
  }

  uint8_t send(uint8_t *pData, uint8_t dataLength) {
    while (!(CAN1->TSR & CAN_TSR_TME0)) {
    }

    CAN1->sTxMailBox[0].TDLR = 1003;
    CAN1->sTxMailBox[0].TDHR = 1101;

    CAN1->sTxMailBox[0].TIR |= CAN_TI0R_TXRQ;

    while (!(CAN1->TSR & CAN_TSR_TME0)) {
    }

    uint8_t is_transmission_ok = CAN1->TSR & CAN_TSR_TXOK0;
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
  uint8_t data[] = "hello_there";

  volatile uint16_t counter = 0;

  while (1) {
    uint8_t error_code = can.send(data, sizeof(data));
  }

  return 0;
}
