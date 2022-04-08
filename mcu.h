// Copyright (c) 2022 Cesanta Software Limited
// All rights reserved
// https://www.st.com/resource/en/reference_manual/dm00124865-stm32f75xxx-and-stm32f74xxx-advanced-arm-based-32-bit-mcus-stmicroelectronics.pdf
// Memory map: 2.2.2

#pragma once

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define BIT(x) (1UL << (x))
#define REG(x) ((volatile uint32_t *) (x))
#define PIN(bank, num) ((((bank) - 'A') << 8) | (num))
#define FREQ 16000000

static inline void init_ram(void) {
  extern uint32_t _sbss, _ebss;
  extern uint32_t _sdata, _edata, _sidata;
  memset(&_sbss, 0, (size_t) (((char *) &_ebss - (char *) &_sbss)));
  memcpy(&_sdata, &_sidata, (size_t) (((char *) &_edata - (char *) &_sdata)));
}

static inline void spin(volatile uint32_t count) {
  while (count--) asm("nop");
}

struct nvic {
  volatile uint32_t ISER[8], RESERVED0[24], ICER[8], RSERVED1[24], ISPR[8],
      RESERVED2[24], ICPR[8], RESERVED3[24], IABR[8], RESERVED4[56], IP[240],
      RESERVED5[644], STIR;
};
#define NVIC ((struct nvic *) 0xe000e100)
static inline void nvic_set_prio(int irq, uint32_t prio) {
  NVIC->IP[irq] = prio << 4;
}
static inline void nvic_enable_irq(int irq) {
  NVIC->ISER[irq >> 5] = (uint32_t) (1 << (irq & 31));
}

struct systick {
  volatile uint32_t CTRL, LOAD, VAL, CALIB;
};
#define SYSTICK ((struct systick *) 0xe000e010)
static inline void systick_config(uint32_t ticks) {
  if ((ticks - 1) > 0xffffff) return;  // Systick timer is 24 bit
  SYSTICK->LOAD = ticks - 1;
  SYSTICK->VAL = 0;
  SYSTICK->CTRL = BIT(0) | BIT(1) | BIT(2);  // Enable systick
}

struct flash {
  volatile uint32_t ACR, KEYR, OPTKEYR, SR, CR, AR, RESERVED, OBR, WRPR;
};
#define FLASH ((struct flash *) 0x40022000)

enum { GPIO_MODE_INPUT, GPIO_MODE_OUTPUT, GPIO_MODE_AF, GPIO_MODE_ANALOG };
enum { GPIO_OTYPE_PUSH_PULL, GPIO_OTYPE_OPEN_DRAIN };
enum { GPIO_SPEED_LOW, GPIO_SPEED_MEDIUM, GPIO_SPEED_HIGH, GPIO_SPEED_INSANE };
enum { GPIO_PULL_NONE, GPIO_PULL_UP, GPIO_PULL_DOWN };

struct gpio {
  volatile uint32_t MODER, OTYPER, OSPEEDR, PUPDR, IDR, ODR, BSRR, LCKR, AFR[2];
};
#define GPIO(N) ((struct gpio *) (0x40020000 + 0x400 * (N)))

static struct gpio *gpio_bank(uint16_t pin) {
  return GPIO(pin >> 8);
}
static inline void gpio_on(uint16_t pin) {
  gpio_bank(pin)->ODR |= BIT(pin & 255);
}
static inline void gpio_off(uint16_t pin) {
  gpio_bank(pin)->ODR &= ~BIT(pin & 255);
}
static inline void gpio_toggle(uint16_t pin) {
  gpio_bank(pin)->ODR ^= BIT(pin & 255);
}
static inline int gpio_read(uint16_t pin) {
  return gpio_bank(pin)->IDR & BIT(pin & 255) ? 1 : 0;
}
static inline void gpio_init(uint16_t pin, uint8_t mode, uint8_t type,
                             uint8_t speed, uint8_t pull, uint8_t af) {
  struct gpio *gpio = gpio_bank(pin);
  uint8_t n = (uint8_t) (pin & 255);
  gpio->MODER &= ~(3UL << (n * 2));
  gpio->MODER |= ((uint32_t) mode) << (n * 2);
  gpio->OTYPER &= ~(1UL << n);
  gpio->OTYPER |= ((uint32_t) type) << n;
  gpio->OSPEEDR &= ~(3UL << (n * 2));
  gpio->OSPEEDR |= ((uint32_t) speed) << (n * 2);
  gpio->PUPDR &= ~(3UL << (n * 2));
  gpio->PUPDR |= ((uint32_t) pull) << (n * 2);
  gpio->AFR[n >> 3] &= ~(15UL << ((n & 7) * 4));
  gpio->AFR[n >> 3] |= ((uint32_t) af) << ((n & 7) * 4);
}
static inline void gpio_input(uint16_t pin) {
  gpio_init(pin, GPIO_MODE_INPUT, GPIO_OTYPE_PUSH_PULL, GPIO_SPEED_HIGH,
            GPIO_PULL_NONE, 0);
}
static inline void gpio_output(uint16_t pin) {
  gpio_init(pin, GPIO_MODE_OUTPUT, GPIO_OTYPE_PUSH_PULL, GPIO_SPEED_HIGH,
            GPIO_PULL_NONE, 0);
}

struct syscfg {
  volatile uint32_t MEMRMP, PMC, EXTICR[4], RESERVED[2], CMPCR;
};
#define SYSCFG ((struct syscfg *) 0x40013800)

struct exti {
  volatile uint32_t IMR, EMR, RTSR, FTSR, SWIER, PR;
};
#define EXTI ((struct exti *) 0x40013c00)

static inline void irq_attach(uint16_t pin) {
  uint8_t bank = (uint8_t) (pin >> 8), n = (uint8_t) (pin & 255);
  SYSCFG->EXTICR[n / 4] &= ~(15UL << ((n % 4) * 4));
  SYSCFG->EXTICR[n / 4] |= (uint32_t) (bank << ((n % 4) * 4));
  EXTI->IMR |= BIT(n);
  EXTI->RTSR |= BIT(n);
  EXTI->FTSR |= BIT(n);
  // nvic_set_prio(6 + n / 4, 1);
  nvic_enable_irq(n <= 4 ? 6 + n : n < 10 ? 23 : 40);
  // SYSCFG->EXTICR[3] = 32;
  // nvic_enable_irq(9);
}

struct rcc {
  volatile uint32_t CR, PLLCFGR, CFGR, CIR, AHB1RSTR, AHB2RSTR, AHB3RSTR,
      RESERVED0, APB1RSTR, APB2RSTR, RESERVED1[2], AHB1ENR, AHB2ENR, AHB3ENR,
      RESERVED2, APB1ENR, APB2ENR, RESERVED3[2], AHB1LPENR, AHB2LPENR,
      AHB3LPENR, RESERVED4, APB1LPENR, APB2LPENR, RESERVED5[2], BDCR, CSR,
      RESERVED6[2], SSCGR, PLLI2SCFGR, PLLSAICFGR, DCKCFGR1, DCKCFGR2;
};
#define RCC ((struct rcc *) 0x40023800)

struct uart {
  volatile uint32_t CR1, CR2, CR3, BRR, GTPR, RTOR, RQR, ISR, ICR, RDR, TDR;
};
#define UART1 ((struct uart *) 0x40011000)
#define UART2 ((struct uart *) 0x40004400)
#define UART3 ((struct uart *) 0x40004800)

static inline void uart_init(struct uart *uart, unsigned long baud) {
  // https://www.st.com/resource/en/datasheet/stm32f746zg.pdf
  uint8_t af = 0;           // Alternate function
  uint16_t rx = 0, tx = 0;  // pins
  if (uart == UART1) af = 4, tx = PIN('A', 9), rx = PIN('A', 10);
  if (uart == UART2) af = 4, tx = PIN('A', 2), rx = PIN('A', 3);
  if (uart == UART3) af = 7, tx = PIN('D', 8), rx = PIN('D', 9);
  gpio_init(tx, GPIO_MODE_AF, GPIO_OTYPE_PUSH_PULL, GPIO_SPEED_HIGH, 0, af);
  gpio_init(rx, GPIO_MODE_AF, GPIO_OTYPE_PUSH_PULL, GPIO_SPEED_HIGH, 0, af);
  uart->CR1 = 0;                          // Disable this UART
  uart->BRR = FREQ / baud;                // Set baud rate, TRM 29.5.4
  uart->CR1 |= BIT(0) | BIT(2) | BIT(3);  // Set UE, RE, TE
}
static inline void uart_write_byte(struct uart *uart, uint8_t byte) {
  uart->TDR = byte;
  while ((uart->ISR & BIT(7)) == 0) spin(1);
}
static inline void uart_write_buf(struct uart *uart, char *buf, size_t len) {
  while (len-- > 0) uart_write_byte(uart, *(uint8_t *) buf++);
}
static inline int uart_read_ready(struct uart *uart) {
  return uart->ISR & BIT(5);  // If RXNE bit is set, data is ready
}
static inline uint8_t uart_read_byte(struct uart *uart) {
  return (uint8_t) (uart->RDR & 255);
}

#if 0
static inline void init_clock(void) {
  FLASH->ACR = 0x12;                     // Set flash
  RCC->CR |= BIT(0);                     // HSI ON, 16 MHz
  while (!(RCC->CR & BIT(1))) (void) 0;  // Wait

  // f(VCO clock)               = f(PLL clock input) x (PLLN / PLLM)
  // f(PLL general clock)       = f(VCO clock) / PLLP
  // f(USB OTG FS, SDMMC, RNG)  = f(VCO clock) / PLLQ
  //                   PLLM          PLLN           PLLP        SRC = HSI
  RCC->PLLCFGR &= ~((63UL << 0) | (511UL << 6) | (3UL << 16) | (1UL << 22));
  RCC->PLLCFGR |= (32UL << 0) | (54UL << 6) | (0UL << 16);
  // RCC->PLLCFGR |= (2UL << 0) | (54UL << 6) | (0UL << 16);

  RCC->CR |= BIT(24);                     // PLL ON
  while (!(RCC->CR & BIT(25))) (void) 0;  // Wait
  RCC->CFGR &= ~3UL;                      // Clear clock source
  RCC->CFGR |= 2;                         // Set PLL clock source
}
#endif
