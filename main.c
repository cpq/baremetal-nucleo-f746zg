// Copyright (c) 2022 Cesanta Software Limited
// All rights reserved

#include "mcu.h"
#include "mongoose.h"

#define LED1 PIN('B', 0)   // On-board LED pin
#define LED2 PIN('B', 7)   // On-board LED pin
#define LED3 PIN('B', 14)  // On-board LED pin
#define BTN1 PIN('C', 13)  // On-board user button

static uint64_t s_ticks, s_exti, s_eth;
static struct mg_mgr s_mgr;

uint64_t mg_millis(void) {
  return s_ticks;
}

static void timer_fn(void *arg) {
  struct uart *uart = arg;
  char buf[20];
  size_t len = mg_snprintf(buf, sizeof(buf), "%llu %llu %llu %d\n", s_ticks,
                           s_exti, s_exti, gpio_read(BTN1));
  uart_write_buf(uart, buf, len);
  gpio_toggle(LED1);
}

int main(void) {
  init_ram();
  RCC->AHB1ENR |= BIT(0) | BIT(1) | BIT(2) | BIT(3);  // GPIO{A,B,C,D}
  RCC->APB1ENR |= BIT(18) | BIT(17);                  // USART{3,2}
  RCC->APB2ENR |= BIT(4) | BIT(14);                   // USART1, SYSCFG

  // RCC->AHB1ENR |= BIT(25) | BIT(26) | BIT(27) | BIT(28);  // Ethernet
  // NVIC_Enable_IRQ(61);          // Ethernet

  gpio_output(LED1);            // Set LED to output
  gpio_input(BTN1);             // Set button to input
  systick_config(FREQ / 1000);  // Increment s_ticks every millisecond
  irq_attach(BTN1);             // Attach BTN1 to exti

  static struct uart *uart = UART3;  // UART3 is wired to the debug port
  uart_init(uart, 115200);           // Initialise it

  struct mg_timer t;
  mg_mgr_init(&s_mgr);
  mg_timer_init(&t, 1000, MG_TIMER_REPEAT | MG_TIMER_RUN_NOW, timer_fn, uart);

  for (;;) asm volatile("wfi");  // Idle, wait for interrupt
  return 0;
}

void irq_systick(void) {
  mg_mgr_poll(&s_mgr, 0);
  s_ticks++;
}
void irq_exti(void) {
  s_exti++;
}
void irq_eth(void) {
  s_eth++;
}
