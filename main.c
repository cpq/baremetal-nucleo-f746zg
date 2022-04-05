// Copyright (c) 2022 Cesanta Software Limited
// All rights reserved

#include "mcu.h"

#define LED1 PIN('B', 0)   // On-board LED pin
#define LED2 PIN('B', 7)   // On-board LED pin
#define LED3 PIN('B', 14)  // On-board LED pin

int main(void) {
  init_ram();
  // init_clock();
  RCC->AHB1ENR |= BIT(0) | BIT(1) | BIT(2) | BIT(3);  // GPIO{A,B,C,D}
  RCC->APB1ENR |= BIT(18) | BIT(17);                  // USART{3,2}
  RCC->APB2ENR |= BIT(4);                             // USART1

  gpio_output(LED1);
  gpio_output(LED2);
  gpio_output(LED3);
  gpio_on(LED1);
  gpio_on(LED2);
  gpio_on(LED3);

  struct uart *uart = UART3;
  uart_init(uart, 115200);

  for (;;) {
    gpio_toggle(LED1);
    uart_write_byte(uart, 'a');             // And write it back
    if (uart_read_ready(uart)) {            // Is UART readable?
      uint8_t byte = uart_read_byte(uart);  // Read one byte
      uart_write_byte(uart, byte);          // And write it back
    }
    spin(199999);
  }

  return 0;
}

__attribute__((naked)) void ETH_IRQHandler(void) {
}
