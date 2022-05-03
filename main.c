// Copyright (c) 2022 Cesanta Software Limited
// All rights reserved

#include "drivers/mip_driver_stm32.h"
#include "mcu.h"
#include "mip.h"
#include "mongoose.h"

#define LED1 PIN('B', 0)   // On-board LED pin (green)
#define LED2 PIN('B', 7)   // On-board LED pin (blue)
#define LED3 PIN('B', 14)  // On-board LED pin (red)
#define BTN1 PIN('C', 13)  // On-board user button

static uint64_t s_ticks, s_exti;     // Counters, increased by IRQ handlers
static time_t s_boot_timestamp = 0;  // Updated by SNTP
static struct mg_connection *s_sntp_conn = NULL;  // SNTP connection

// We have no valid system time(), and we need it for TLS. Implement it
time_t time(time_t *tp) {
  time_t t = s_boot_timestamp + (time_t) (mg_millis() / 1000);
  if (tp != NULL) *tp = t;
  return t;
}

// SNTP connection event handler. When we get a response from an SNTP server,
// adjust s_boot_timestamp. We'll get a valid time from that point on
static void sfn(struct mg_connection *c, int ev, void *ev_data, void *fn_data) {
  if (ev == MG_EV_SNTP_TIME) {
    uint64_t t = *(uint64_t *) ev_data;
    MG_INFO(("%lu SNTP: %lld ms from epoch", c->id, t));
    s_boot_timestamp = (time_t) ((t - mg_millis()) / 1000);
  } else if (ev == MG_EV_CLOSE) {
    s_sntp_conn = NULL;
  }
  (void) fn_data;
}

// SNTP timer function. Sync up time periodically
static void sntp_cb(void *param) {
  struct mg_mgr *mgr = (struct mg_mgr *) param;
  if (s_sntp_conn == NULL) s_sntp_conn = mg_sntp_connect(mgr, NULL, sfn, NULL);
  if (s_boot_timestamp < 9999) mg_sntp_send(s_sntp_conn, 0UL);
}

static void blink_cb(void *arg) {  // Blink periodically
  // struct mip_if *ifp = (struct mip_if *) arg;
  MG_INFO(("ticks: %u", (unsigned) s_ticks));
  gpio_toggle(LED2);
  (void) arg;
}

// Server event handler
static void fn(struct mg_connection *c, int ev, void *ev_data, void *fn_data) {
  if (ev == MG_EV_POLL) return;
  MG_DEBUG(("%lu %p %d %p %p", c->id, c, ev, ev_data, fn_data));
  if (ev == MG_EV_HTTP_MSG) {
    struct mg_http_message *hm = (struct mg_http_message *) ev_data;
    mg_http_reply(c, 200, "", "Received request to %.*s", (int) hm->uri.len,
                  hm->uri.ptr);
  } else if (ev == MG_EV_READ) {
    struct mg_str *data = ev_data;
    MG_INFO(("READ: %d [%.*s]", (int) data->len, (int) data->len, data->ptr));
  }
}

int main(void) {
  static struct uart *uart = UART3;  // Use UART3 - its attached to debug
  ram_init();                        // Initialise RAM - bss, etc
  clock_init();                      // Set clock to 216MHz
  systick_init(FREQ / 1000);         // Increment s_ticks every millisecond
  gpio_output(LED1);                 // Setup green LED
  gpio_output(LED2);                 // Setup blue LED
  gpio_input(BTN1);                  // Set button to input
  irq_exti_attach(BTN1);             // Attach BTN1 to exti
  uart_init(uart, 115200);           // It is wired to the debug port

  // Initialise Ethernet. Enable MAC GPIO pins, see
  // https://www.farnell.com/datasheets/2014265.pdf section 6.10
  uint16_t pins[] = {PIN('A', 1),  PIN('A', 2),  PIN('A', 7),
                     PIN('B', 13), PIN('C', 1),  PIN('C', 4),
                     PIN('C', 5),  PIN('G', 11), PIN('G', 13)};
  for (size_t i = 0; i < sizeof(pins) / sizeof(pins[0]); i++) {
    gpio_init(pins[i], GPIO_MODE_AF, GPIO_OTYPE_PUSH_PULL, GPIO_SPEED_INSANE,
              GPIO_PULL_NONE, 11);
  }
  nvic_enable_irq(61);                          // Setup Ethernet IRQ handler
  RCC->APB2ENR |= BIT(14);                      // Enable SYSCFG
  SYSCFG->PMC |= BIT(23);                       // Use RMII. Goes first!
  RCC->AHB1ENR |= BIT(25) | BIT(26) | BIT(27);  // Enable Ethernet clocks

  struct mg_mgr mgr;  // Initialise Mongoose event manager
  mg_mgr_init(&mgr);  // and attach it to the MIP interface
  mg_listen(&mgr, "http://0.0.0.0:80", fn, NULL);
  mg_timer_add(&mgr, 1000, MG_TIMER_REPEAT, blink_cb, &mgr);
  mg_timer_add(&mgr, 5000, MG_TIMER_REPEAT, sntp_cb, &mgr);
  mg_log_set("3");

  // Initialise Mongoose network stack
  struct mip_ipcfg ipcfg = {
      {0xaa, 0xbb, 0xcc, 1, 2, 3}, 0x0202a8c0, 0xffffff, 0x0102a8c0};
  // struct mip_ipcfg ipcfg = {{0xaa, 0xbb, 0xcc, 1, 2, 3}, 0, 0, 0};
  struct mip_driver stm32_driver = {.init = mip_driver_stm32_init,
                                    .tx = mip_driver_stm32_tx,
                                    .rxcb = mip_driver_stm32_setrx,
                                    .status = mip_driver_stm32_status};
  mip_init(&mgr, &ipcfg, &stm32_driver);

  for (;;) mg_mgr_poll(&mgr, 0);  // Infinite event loop

  return 0;
}

uint64_t mg_millis(void) {  // Declare our own uptime function
  return s_ticks;           // Return number of milliseconds since boot
}

void halt(void) {                             // Catch-all fault handler
  gpio_output(LED3);                          // Setup red LED
  for (;;) spin(2999999), gpio_toggle(LED3);  // Blink LED infinitely
}

void irq_systick(void) {  // Systick IRQ handler
  s_ticks++;
}

void irq_exti(void) {  // EXTI IRQ handler
  s_exti++;
  if (EXTI->PR & BIT(PINNO(BTN1))) EXTI->PR = BIT(PINNO(BTN1));
  gpio_write(LED1, gpio_read(BTN1));  // No debounce. Turn LED if button pressed
}
