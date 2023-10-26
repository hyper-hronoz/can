#include "UART.h"
#include "Delay.h"
#include "LED.h"

uint8_t UART::buffer[UART_buffer_size] = "";
uint8_t UART::index = 0;

void UART::configure_GPIO_recieve() {
  GPIOA->CRH &= ~(GPIO_CRH_CNF10_0);
  GPIOA->CRH |= GPIO_CRH_CNF10_1;
  GPIOA->CRH &= ~(GPIO_CRH_MODE10);
  GPIOA->ODR |= GPIO_ODR_ODR10;
}

void UART::configure_GPIO_transmit() {
  GPIOA->CRH &= ~(GPIO_CRH_MODE9_Msk);
  GPIOA->CRH |= GPIO_CRH_MODE9;
  GPIOA->CRH &= ~(GPIO_CRH_CNF9_Msk);
  GPIOA->CRH |= GPIO_CRH_CNF9_1;
}

void UART::transmit(uint8_t *data, uint32_t size) {
  for (uint32_t i = 0; i < size; i++) {
    if (Delay().timeout(USART1->SR, USART_SR_TXE,
                        10000)) { //  transmission empty
      LED().led_timeout_exception();
    }

    USART1->DR = data[i];

    if (Delay().timeout(USART1->SR, USART_SR_TC,
                        10000)) { // transmission complited
      LED().led_timeout_exception();
    }
  }
}

void UART::configure_UART(UART_INRQ header) {
  USART1->CR1 |= USART_CR1_UE;

  USART1->BRR = header.baudrate;

  USART1->CR1 &= ~(USART_CR1_M_Msk);
  USART1->CR1 |= (header.enable_word_9bit << USART_CR1_M_Pos);

  USART1->CR1 &= ~(USART_CR1_PCE_Msk);
  USART1->CR1 &= ~(header.enable_parity << USART_CR1_PCE_Pos);

  USART1->CR1 &= ~(USART_CR1_TE_Msk);
  USART1->CR1 |= (header.enable_transmitter << USART_CR1_TE_Pos);

  USART1->CR1 &= ~(USART_CR1_RE_Msk);
  USART1->CR1 |= (header.enable_reciever << USART_CR1_RE_Pos);

  USART1->CR1 |= USART_CR1_RXNEIE;
  // USART1->CR1 |= USART_CR1_IDLEIE;
  NVIC_EnableIRQ (USART1_IRQn);
}

void UART::__init__(UART_INRQ header) {
  RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
  RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

  this->configure_GPIO_recieve();
  this->configure_GPIO_transmit();

  this->configure_UART(header);
}

void UART::clear_buffer() {
  for (uint8_t i = 0; i < sizeof(UART::buffer); i++) {
    UART::buffer[i] = 0;
  }
  UART::index = 0;
}
