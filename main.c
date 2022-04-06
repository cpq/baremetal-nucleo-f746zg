// Copyright (c) 2022 Cesanta Software Limited
// All rights reserved

#include "mcu.h"

#define LED1 PIN('B', 0)   // On-board LED pin
#define LED2 PIN('B', 7)   // On-board LED pin
#define LED3 PIN('B', 14)  // On-board LED pin

static uint32_t s_ticks;
static uint32_t s_ic;

int main(void) {
  init_ram();
  // init_clock();
  RCC->AHB1ENR |= BIT(0) | BIT(1) | BIT(2) | BIT(3);      // GPIO{A,B,C,D}
  RCC->APB1ENR |= BIT(18) | BIT(17);                      // USART{3,2}
  RCC->APB2ENR |= BIT(4);                                 // USART1
  RCC->AHB1ENR |= BIT(25) | BIT(26) | BIT(27) | BIT(28);  // Ethernet

  SysTick_Config(FREQ / 1000);  // Increment s_ticks every millisecond
  NVIC_Enable_IRQ(61);          // Ethernet

  gpio_output(LED1);
  gpio_on(LED1);

  struct uart *uart = UART3;
  uart_init(uart, 115200);

  for (;;) {
    char buf[20];
    int len = snprintf(buf, sizeof(buf), "%lu %02lu:%02lu:%02lu.%lu", s_ic,
                       s_ticks / 3600000, (s_ticks / 1000 / 60) % 60,
                       (s_ticks / 1000) % 60, s_ticks % 1000);
    uart_write_buf(uart, buf, (size_t) len);
    uart_write_byte(uart, '\n');
    gpio_toggle(LED1);
    spin(999999);
  }

  return 0;
}

void irq_systick(void) {
  s_ticks++;
}

void irq_eth(void) {
  s_ic++;
}
