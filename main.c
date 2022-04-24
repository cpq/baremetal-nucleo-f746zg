// Copyright (c) 2022 Cesanta Software Limited
// All rights reserved

#include "mcu.h"
#include "mongoose.h"

#define LED1 PIN('B', 0)   // On-board LED pin
#define LED2 PIN('B', 7)   // On-board LED pin
#define LED3 PIN('B', 14)  // On-board LED pin
#define BTN1 PIN('C', 13)  // On-board user button

static uint64_t s_ticks, s_exti;  // Counters, increased by IRQ handlers
static struct mg_mgr s_mgr;       // Mongoose event manager

static void timer_fn(void *arg) {  // Timer function
  uint32_t sr = eth_read_phy(PHY_ADDR, PHY_BSR);
  const char *link_status = sr & BIT(2) ? "up" : "down";
  printf("--> %6u %p %s\n", (unsigned) s_ticks, arg, link_status);
  eth_send("boo", 3);
  gpio_toggle(LED2);
}

uint64_t mg_millis(void) {  // Declare our own uptime function
  return s_ticks;           // Return number of milliseconds since boot
}

void halt(void) {     // Catch-all handler for all sorts of faults
  gpio_output(LED3);  // Setup red LED
  for (;;) spin(2999999), gpio_toggle(LED3);  // Blink LED infinitely
}

int main(void) {
  ram_init();                        // Initialise RAM - bss, etc
  clock_init();                      // Set clock to 216MHz
  systick_init(FREQ / 1000);         // Increment s_ticks every millisecond
  gpio_output(LED2);                 // Set LED to output
  gpio_input(BTN1);                  // Set button to input
  irq_exti_attach(BTN1);             // Attach BTN1 to exti
  static struct uart *uart = UART3;  // Initialise UART3
  uart_init(uart, 115200);           // It is wired to the debug port
  mg_mgr_init(&s_mgr);               // Every 1000 milliseconds
  mg_timer_add(&s_mgr, 500, MG_TIMER_REPEAT | MG_TIMER_RUN_NOW, timer_fn, 0);
  eth_init();                    // Init ethernet
  for (;;) (void) 0;             // Busy loop
  for (;;) asm volatile("wfi");  // Idle loop: wait for interrupt
  return 0;
}

void irq_systick(void) {  // Systick IRQ handler
  mg_mgr_poll(&s_mgr, 0);
  s_ticks++;
}

void irq_exti(void) {  // EXTI IRQ handler
  s_exti++;
  if (EXTI->PR & BIT(PINNO(BTN1))) EXTI->PR = BIT(PINNO(BTN1));
}
