#include "mcu.h"

#define ETH_PKT_SIZE 1536
#define ETH_BUF_COUNT 3
#define ETHBUF __attribute__((aligned(4), __section__(.ram_no_cache)))
static uint8_t s_rxbuf[ETH_BUF_COUNT][ETH_PKT_SIZE];  // RX ethernet buffers
static uint8_t s_txbuf[ETH_BUF_COUNT][ETH_PKT_SIZE];  // TX ethernet buffers
static uint32_t s_rxdesc[ETH_BUF_COUNT][4];           // RX descriptors
static uint32_t s_txdesc[ETH_BUF_COUNT][4];           // TX descriptors

uint32_t eth_read_phy(uint8_t addr, uint8_t reg) {
  ETH->MACMIIAR &= (7 << 2);
  ETH->MACMIIAR |= ((uint32_t) addr << 11) | ((uint32_t) reg << 6) | BIT(0);
  while (ETH->MACMIIAR & BIT(0)) spin(1);
  return ETH->MACMIIDR;
}

void eth_write_phy(uint8_t addr, uint8_t reg, uint32_t val) {
  ETH->MACMIIDR = val;
  ETH->MACMIIAR &= (7 << 2);
  ETH->MACMIIAR |=
      ((uint32_t) addr << 11) | ((uint32_t) reg << 6) | BIT(0) | BIT(1);
  while (ETH->MACMIIAR & BIT(0)) spin(1);
}

static void dma_buffers_init(uint32_t desc[ETH_BUF_COUNT][4],
                             uint8_t buf[ETH_BUF_COUNT][ETH_PKT_SIZE]) {
  for (int i = 0; i < ETH_BUF_COUNT; i++) {
    desc[i][0] = BIT(31);
    desc[i][1] = sizeof(buf[i]) | BIT(14);  // 2nd address chained
    desc[i][2] = (uint32_t) &buf[i];
    desc[i][3] = (uint32_t) &desc[i == ETH_BUF_COUNT - 1 ? 0 : i + 1];
  }
}

void eth_init(void) {
  // Enable MAC GPIO pins. See
  // https://www.farnell.com/datasheets/2014265.pdf section 6.10
  uint8_t mode = GPIO_MODE_AF, type = GPIO_OTYPE_PUSH_PULL,
          speed = GPIO_SPEED_HIGH, pull = GPIO_PULL_NONE, af = 11;
  gpio_init(PIN('A', 1), mode, type, speed, pull, af);   // Ref clock
  gpio_init(PIN('A', 2), mode, type, speed, pull, af);   // MDIO
  gpio_init(PIN('C', 1), mode, type, speed, pull, af);   // MDC
  gpio_init(PIN('A', 7), mode, type, speed, pull, af);   // RX data valid
  gpio_init(PIN('C', 4), mode, type, speed, pull, af);   // RXD0
  gpio_init(PIN('C', 5), mode, type, speed, pull, af);   // RXD1
  gpio_init(PIN('G', 11), mode, type, speed, pull, af);  // TX enable
  gpio_init(PIN('G', 13), mode, type, speed, pull, af);  // TXD0
  gpio_init(PIN('B', 13), mode, type, speed, pull, af);  // TXD1

  nvic_enable_irq(61);                          // Setup Ethernet IRQ handler
  RCC->APB2ENR |= BIT(14);                      // Enable SYSCFG
  SYSCFG->PMC |= BIT(23);                       // Use RMII. Goes first!
  RCC->AHB1ENR |= BIT(25) | BIT(26) | BIT(27);  // Enable Ethernet clocks
  RCC->AHB1RSTR |= BIT(25);                     // ETHMAC force reset
  RCC->AHB1RSTR &= ~BIT(25);                    // ETHMAC release reset
  ETH->DMABMR |= BIT(0);                        // Software reset
  while ((ETH->DMABMR & BIT(0)) != 0) spin(1);  // Wait until done
  ETH->DMABMR |= BIT(1);                        // RX has prio
  ETH->MACIMR = BIT(3) | BIT(9);                // Mask timestamp & PMT IT
  ETH->MACMIIAR = 4 << 2;                       // MDC clock 150-216 MHz, 38.8.1
  ETH->MACFCR = 0;                              // Disable flow control
  ETH->MACFFR = BIT(0);                         // Promisc mode
  eth_write_phy(PHY_ADDR, PHY_BCR, BIT(15));    // Reset PHY
  eth_write_phy(PHY_ADDR, PHY_BCR, BIT(12));    // Set autonegotiation
  dma_buffers_init(s_rxdesc, s_rxbuf);          // Init buffers
  ETH->DMARDLAR = (uint32_t) s_rxdesc;          // RX descriptors
  dma_buffers_init(s_txdesc, s_txbuf);          // Init buffers
  ETH->DMATDLAR = (uint32_t) s_txdesc;          // RX descriptors
  ETH->DMAIER = BIT(0) | BIT(6) | BIT(16);      // Enable DMA interrupts
  ETH->MACCR |= BIT(2) | BIT(3) | BIT(11);      // Start TX, RX, duplex
  ETH->DMAOMR = BIT(21) | BIT(25);              // DMA mode: RSF & TSF
  ETH->DMAOMR |= BIT(1) | BIT(13);              // DMA mode: start rx/tx
}

void eth_send(const void *buf, size_t len) {
  s_txdesc[0][0] |= BIT(31) | BIT(28) | BIT(29);  // Own, last seg, first seg;
  s_txdesc[0][1] = (uint32_t) len | BIT(14);
  memcpy(&s_txbuf[0][0], buf, len);
  ETH->DMASR = BIT(2);  // Set TBUS
  ETH->DMATPDR = 0;
}
