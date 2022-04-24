#include "mcu.h"

#define ETH_PKT_SIZE 2048
#define ETH_DESC_CNT 3
#define ETH_DS 4
#define _EB __attribute__((__section__(".sram2")))
//#define _EB
static uint32_t s_rxdesc[ETH_DESC_CNT][ETH_DS] _EB;      // RX descriptors
static uint32_t s_txdesc[ETH_DESC_CNT][ETH_DS] _EB;      // TX descriptors
static uint8_t s_rxbuf[ETH_DESC_CNT][ETH_PKT_SIZE] _EB;  // RX ethernet buffers
static uint8_t s_txbuf[ETH_DESC_CNT][ETH_PKT_SIZE] _EB;  // TX ethernet buffers
static uint32_t s_nirq, s_nrcv, s_nsnt, s_desc;

uint32_t eth_read_phy(uint8_t addr, uint8_t reg) {
  ETH->MACMIIAR &= (7 << 2);
  ETH->MACMIIAR |= ((uint32_t) addr << 11) | ((uint32_t) reg << 6);
  ETH->MACMIIAR |= BIT(0);
  while (ETH->MACMIIAR & BIT(0)) spin(1);
  return ETH->MACMIIDR;
}

void eth_write_phy(uint8_t addr, uint8_t reg, uint32_t val) {
  ETH->MACMIIDR = val;
  ETH->MACMIIAR &= (7 << 2);
  ETH->MACMIIAR |= ((uint32_t) addr << 11) | ((uint32_t) reg << 6) | BIT(1);
  ETH->MACMIIAR |= BIT(0);
  while (ETH->MACMIIAR & BIT(0)) spin(1);
}

static void dma_buffers_init(uint32_t desc[ETH_DESC_CNT][ETH_DS],
                             uint8_t buf[ETH_DESC_CNT][ETH_PKT_SIZE]) {
  for (int i = 0; i < ETH_DESC_CNT; i++) {
    desc[i][0] = BIT(31);
    desc[i][1] = sizeof(buf[i]) | BIT(14);  // 2nd address chained
    desc[i][2] = (uint32_t) buf[i];
    desc[i][3] = (uint32_t) desc[(i + 1) % ETH_DESC_CNT];
  }
}

void eth_init(void) {
  // Enable MAC GPIO pins. See
  // https://www.farnell.com/datasheets/2014265.pdf section 6.10
  uint16_t pins[] = {PIN('A', 1),  PIN('A', 2),  PIN('A', 7),
                     PIN('B', 13), PIN('C', 1),  PIN('C', 4),
                     PIN('C', 5),  PIN('G', 11), PIN('G', 13)};
  for (size_t i = 0; i < sizeof(pins) / sizeof(pins[0]); i++) {
    gpio_init(pins[i], GPIO_MODE_AF, GPIO_OTYPE_PUSH_PULL, GPIO_SPEED_HIGH,
              GPIO_PULL_NONE, 11);
  }

  nvic_enable_irq(61);                          // Setup Ethernet IRQ handler
  RCC->APB2ENR |= BIT(14);                      // Enable SYSCFG
  SYSCFG->PMC |= BIT(23);                       // Use RMII. Goes first!
  RCC->AHB1ENR |= BIT(25) | BIT(26) | BIT(27);  // Enable Ethernet clocks
  RCC->AHB1RSTR |= BIT(25);                     // ETHMAC force reset
  RCC->AHB1RSTR &= ~BIT(25);                    // ETHMAC release reset
  ETH->DMABMR |= BIT(0);                        // Software reset
  while ((ETH->DMABMR & BIT(0)) != 0) spin(1);  // Wait until done
  ETH->DMABMR = BIT(13) | BIT(16) | BIT(22) | BIT(23) | BIT(25);
  // ETH->MACIMR = BIT(3) | BIT(9);              // Mask timestamp & PMT IT
  ETH->MACMIIAR = 4 << 2;                     // MDC clock 150-216 MHz, 38.8.1
  ETH->MACFCR = BIT(7);                       // Disable zero quarta pause
  ETH->MACFFR = BIT(31);                      // Receive all
  eth_write_phy(PHY_ADDR, PHY_BCR, BIT(15));  // Reset PHY
  eth_write_phy(PHY_ADDR, PHY_BCR, BIT(12));  // Set autonegotiation
  dma_buffers_init(s_rxdesc, s_rxbuf);        // Init buffers
  ETH->DMARDLAR = (uint32_t) s_rxdesc;        // RX descriptors
  dma_buffers_init(s_txdesc, s_txbuf);        // Init buffers
  ETH->DMATDLAR = (uint32_t) s_txdesc;        // RX descriptors
  ETH->DMAIER = BIT(0) | BIT(6) | BIT(16);    // TIE, RIE, NISE
  ETH->MACCR = BIT(2) | BIT(3) | BIT(11) | BIT(14);  // RE, TE, Duplex, Fast
  ETH->DMAOMR = BIT(1) | BIT(21) | BIT(25);          // SR, TSF, RSF
}

void eth_send(const void *buf, size_t len) {
#if 0
  s_txdesc[0][0] |= BIT(31) | BIT(28) | BIT(29);  // Own, last seg, first seg;
  s_txdesc[0][1] = (uint32_t) len | BIT(14);
  memcpy(&s_txbuf[0][0], buf, len);
  ETH->DMASR = BIT(2);  // TBUS
  ETH->DMATPDR = 0;
#endif
  (void) buf, (void) len;

  return;
  printf("  %lx %lx %lx %lx | %lx %lx %lx %lx | %lx %lx %lx %lx\n", s_nirq,
         s_nrcv, s_nsnt, ETH->DMASR, s_rxdesc[0][0], s_rxdesc[0][1],
         s_rxdesc[0][2], s_rxdesc[0][3], s_rxdesc[1][0], s_rxdesc[1][1],
         s_rxdesc[1][2], s_rxdesc[1][3]);
}

void irq_eth(void) {  // Ethernet IRQ handler
  s_nirq++;
  if (ETH->DMASR & BIT(6)) {
    uint32_t len = (s_rxdesc[s_desc][0] >> 16) & (BIT(14) - 1);
    s_nrcv++;
    printf("-------> %lu\n", len);
    s_rxdesc[s_desc][0] = BIT(31);
    s_desc = (s_desc + 1) % ETH_DESC_CNT;
  }
  printf("  %lx %lx %lx %lx | %lx\n", s_nirq, s_nrcv, s_nsnt, ETH->DMASR,
         ETH->DMARDLAR);
  // ETH->DMASR = BIT(16);
  ETH->DMASR = ~0UL;
}
