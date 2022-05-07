# Bare metal firmware for NUCLEO-F746ZG board

- No dependencies: no HAL, no CMSIS, no RTOS
- Hand-written [mcu.h](mcu.h) header based on a [datasheet](https://www.st.com/resource/en/reference_manual/rm0385-stm32f75xxx-and-stm32f74xxx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)
- Interrupt-driven ethernet driver and systick
- Integrated [mongoose](https://github.com/cesanta/mongoose) and
  [mip](https://github.com/cesanta/mip) that implement
  HTTP server, HTTP server, and SNTP time synchronisation
- SysTick interrupt-driven software timer, blue LED blink
- EXTI interrupt-driven user button handler, turns off/on green LED
- Catch-all fault handler that blinks red LED


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

The `mon` target uses `cat` command for serial port monitoring:

```
make clean flash mon
```

## Benchmark

A quick comparison is made with the Zephyr-based implementation, see
source code at [http-server](https://github.com/cesanta/mongoose/tree/master/examples/zephyr/http-server).
Note: `IP` in the table below is the IP address printed on the console after
boot.

|         | Zephyr implementation | This bare-metal implementation |
| ------- | --------------------- | ------------------------------ |
| Command | siege -c 5 -t 5s http://IP:8000/api/stats| siege -c 5 -t 5s http://IP/api/stats |
| Requests | 13 | 338 |
| Per second | 2.75 | 71 |

Whilst not comprehensive, this quick benchmark shows a 25x performance
difference.
