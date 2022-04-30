TARGET  = firmware
ARCH    = arm-none-eabi
OPTS    ?= -g3 -O0 -ffunction-sections -fdata-sections
WARN    ?= -W -Wall -Wextra -Werror -Wundef -Wshadow -Wdouble-promotion -Wformat-truncation -fno-common -Wconversion
DEFS    ?= -I. -Imongoose -Imip -DMG_ARCH=MG_ARCH_CUSTOM -DMIP_DEBUG=1 -DMG_ENABLE_CUSTOM_MILLIS=1 -DMG_ENABLE_LINES=1
MCUFL   ?= -mcpu=cortex-m7 -mthumb -mfloat-abi=softfp -mfpu=vfpv4
CFLAGS  ?= $(WARN) $(OPTS) $(MCUFL) $(DEFS) $(DEFS)
LDFLAGS ?= -Tlink.ld -nostartfiles -nostdlib --specs nano.specs -lc -lgcc -Wl,--gc-sections -Wl,-Map=$@.map
SOURCES = boot.s main.c eth.c syscalls.c mip/mip.c mongoose/mongoose.c mongoose_custom.c

all: $(TARGET).bin

mongoose/mongoose.c:
	git clone --depth 1 https://github.com/cesanta/mongoose

mip/mip.c:
	git clone --depth 1 -b 0.1.0 https://github.com/cesanta/mip

$(TARGET).bin: $(TARGET).elf
	$(ARCH)-objcopy -O binary $< $@

$(TARGET).elf: $(SOURCES) mcu.h
	$(ARCH)-gcc $(SOURCES) $(CFLAGS) $(LDFLAGS) -o $@

# Note: on "unknown chip id" flash error, wire BOOT0 to VDD and st-flash erase
flash: $(TARGET).bin
	st-flash --reset write $(TARGET).bin 0x8000000

mon:
	esputil -p /dev/ttyACM0 monitor

openocd:
	openocd -f openocd.cfg

ARGS ?= -ex 'b main'
gdb: $(TARGET).elf
	$(ARCH)-gdb -ex 'set confirm off' -ex 'target extended-remote :3333' -ex 'monitor arm semihosting enable' -ex 'monitor reset halt' -ex 'load' -ex 'monitor reset init' $(ARGS) -ex 'r' $(TARGET).elf

clean:
	@rm -rf $(TARGET).* *.su
