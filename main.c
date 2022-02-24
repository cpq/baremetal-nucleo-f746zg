// Copyright (c) 2022 Cesanta Software Limited
// All rights reserved
// https://www.st.com/resource/en/reference_manual/dm00124865-stm32f75xxx-and-stm32f74xxx-advanced-arm-based-32-bit-mcus-stmicroelectronics.pdf
// Memory map: 2.2.2

#include <stdint.h>
#include <string.h>

#define BIT(x) (1UL << (x))
#define PIN(bank, num) ((((bank) - 'A') << 8) | (num))

static inline void init_ram(void) {
  extern uint32_t _sbss, _ebss;
  extern uint32_t _sdata, _edata, _sidata;
  memset(&_sbss, 0, (size_t) (((char *) &_ebss - (char *) &_sbss)));
  memcpy(&_sdata, &_sidata, (size_t) (((char *) &_edata - (char *) &_sdata)));
}

static inline void spin(volatile uint32_t count) {
  while (count--) asm("nop");
}

#if 0
struct systick {
  volatile uint32_t CTRL, LOAD, VAL, CALIB;
};
#define SysTick ((struct systick *) 0xe000e010)

struct nvic {
  volatile uint32_t ISER[8], RESERVED0[24], ICER[8], RSERVED1[24], ISPR[8],
      RESERVED2[24], ICPR[8], RESERVED3[24], IABR[8], RESERVED4[56], IP[240],
      RESERVED5[644], STIR;
} NVIC_Type;
#define NVIC ((struct nvic *) 0xe000e100)

static inline void Set_NVIC_Prio(int irq, uint32_t prio) {
  NVIC->IP[irq] = prio << 4;
}

static inline uint32_t SysTick_Config(uint32_t ticks) {
  if ((ticks - 1) > 0xffffff) return 1;  // Systick timer is 24 bit
  SysTick->LOAD = ticks - 1;
  SysTick->VAL = 0;
  SysTick->CTRL = BIT(0) | BIT(1) | BIT(2);
  return 0;
}
#endif

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

static inline struct gpio *gpio_bank(uint16_t pin) {
  return (struct gpio *) (0x40020000 + 0x400 * (pin >> 8));
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
  if (n < 8) {
    gpio->AFR[0] &= 15UL << (n * 4);
    gpio->AFR[0] |= ((uint32_t) af) << (n * 4);
  } else {
    gpio->AFR[1] &= 15UL << ((n - 8) * 4);
    gpio->AFR[1] |= ((uint32_t) af) << ((n - 8) * 4);
  }
}
static inline void gpio_input(uint16_t pin) {
  gpio_init(pin, GPIO_MODE_INPUT, GPIO_OTYPE_PUSH_PULL, GPIO_SPEED_MEDIUM,
            GPIO_PULL_NONE, 0);
}
static inline void gpio_output(uint16_t pin) {
  gpio_init(pin, GPIO_MODE_OUTPUT, GPIO_OTYPE_PUSH_PULL, GPIO_SPEED_MEDIUM,
            GPIO_PULL_NONE, 0);
}

struct rcc {
  volatile uint32_t CR, PLLCFGR, CFGR, CIR, AHB1RSTR, AHB2RSTR, AHB3RSTR,
      RESERVED0, APB1RSTR, APB2RSTR, RESERVED1[2], AHB1ENR, AHB2ENR, AHB3ENR,
      RESERVED2, APB1ENR, APB2ENR, RESERVED3[2], AHB1LPENR, AHB2LPENR,
      AHB3LPENR, RESERVED4, APB1LPENR, APB2LPENR, RESERVED5[2], BDCR, CSR,
      RESERVED6[2], SSCGR, PLLI2SCFGR, PLLSAICFGR, DCKCFGR1, DCKCFGR2;
};
#define RCC ((struct rcc *) 0x40023800)

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

#define LED1 PIN('B', 0)   // On-board LED pin
#define LED2 PIN('B', 7)   // On-board LED pin
#define LED3 PIN('B', 14)  // On-board LED pin

int main(void) {
  // init_ram();
  // init_clock();
  RCC->AHB1ENR |= BIT(0) | BIT(1) | BIT(2) | BIT(6);  // Init GPIO banks A,B,C,G

  gpio_output(LED1);
  gpio_output(LED2);
  gpio_output(LED3);
  gpio_on(LED2);
  gpio_on(LED3);

  for (;;) {
    spin(99999);
    gpio_toggle(LED1);
  }

  return 0;
}

__attribute__((naked)) void ETH_IRQHandler(void) {
}
