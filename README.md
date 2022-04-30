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

## Usage

Plugin your Nucleo board into USB, and attach an Ethernet cable.
To build and flash:

```sh
make clean flash
```

To see debug log, use any serial monitor program like `cu`:

```sh
cu -l /dev/ttyACM0 -s 115200
```

Alternatively, build `esputil` utility from https://github.com/cpq/mdk,
then use can use a single command to rebuild and monitor logs:

```
make clean flash mon
```
