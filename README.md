# Bare metal firmware for NUCLEO-F746ZG board

- No dependencies: no HAL, no CMSIS
- Hand-written [mcu.h](mcu.h) header (no CMSIS) based on [datasheet](https://www.st.com/resource/en/reference_manual/rm0385-stm32f75xxx-and-stm32f74xxx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)
- No RTOS
- SysTick based software timer
- LED blink
- Ethernet networking: HTTP server based on [Mongoose Library](https://github.com/cesanta/mongoose) with experimental built-in TCP/IP stack
- Ethernet, systick are interrupt driven

## Requirements

- GNU make
- [ARM GCC](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm) toolchain for build
- [st-link](https://github.com/stlink-org/stlink) for flashing
