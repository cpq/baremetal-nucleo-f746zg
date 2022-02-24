TARGET  = firmware
ARCH    = arm-none-eabi
OPTS		= -W -Wall -g3 -Os
MCUFL   = -mcpu=cortex-m7 -mthumb -mfloat-abi=softfp -mfpu=vfpv4
CFLAGS  = $(OPTS) $(MCUFL) $(INCS) $(DEFS)
LDFLAGS = -Tlink.ld -nostartfiles -nostdlib --specs nosys.specs -lc -Wl,--gc-sections -Wl,-Map=$@.map

SOURCES = boot.s main.c #syscalls.c

$(TARGET).bin: $(TARGET).elf
	$(ARCH)-objcopy -O binary $< $@

$(TARGET).elf: $(SOURCES)
	$(ARCH)-gcc $(SOURCES) $(CFLAGS) $(LDFLAGS) -o $@

flash: $(TARGET).bin
	st-flash --reset write $< 0x8000000

openocd:
	openocd -f openocd.cfg

ARGS ?= -ex 'b main'
gdb: $(TARGET).elf
	$(ARCH)-gdb \
  -ex 'set confirm off' \
  -ex 'target extended-remote :3333' \
  -ex 'monitor arm semihosting enable' \
  -ex 'monitor reset halt' \
  -ex 'load' \
  -ex 'monitor reset init' \
  $(ARGS) \
  -ex 'r' \
  $(TARGET).elf


clean:
	@rm -rf $(TARGET).*
