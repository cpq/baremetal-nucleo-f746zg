TARGET  = firmware
CROSS   = arm-none-eabi
OPTS    ?= -g3 -Os -ffunction-sections -fdata-sections
WARN    ?= -W -Wall -Wextra -Werror -Wundef -Wshadow -Wdouble-promotion -Wformat-truncation -fno-common -Wconversion
DEFS    ?= -I. -Imongoose -Imip -DMG_ARCH=MG_ARCH_NEWLIB -DMIP_DEBUG=1 -DMG_ENABLE_CUSTOM_MILLIS=1 -DMG_ENABLE_LINES=1
MCUFL   ?= -mcpu=cortex-m7 -mthumb -mfloat-abi=softfp -mfpu=vfpv4
CFLAGS  ?= $(WARN) $(OPTS) $(MCUFL) $(DEFS) $(DEFS)
LDFLAGS ?= -Tlink.ld -nostartfiles -nostdlib --specs nano.specs -lc -lgcc -Wl,--gc-sections -Wl,-Map=$@.map
SOURCES = boot.c main.c syscalls.c mip/mip.c mip/drivers/mip_driver_stm32.c mongoose/mongoose.c

all: $(TARGET).bin

mongoose/mongoose.c:
	git clone --depth 1 https://github.com/cesanta/mongoose

mip/mip.c:
	git clone --depth 1 -b 0.1.5 https://github.com/cesanta/mip

$(TARGET).bin build: $(TARGET).elf
	$(CROSS)-objcopy -O binary $< $@

$(TARGET).elf: $(SOURCES) mcu.h
	$(CROSS)-gcc $(SOURCES) $(CFLAGS) $(LDFLAGS) -o $@

# Note: on "unknown chip id" flash error, wire BOOT0 to VDD and st-flash erase
flash: $(TARGET).bin
	st-flash --reset write $(TARGET).bin 0x8000000

mon:
	cat /dev/ttyACM0  # Or, use your favorite terminal utility like cu, minicom

clean:
	@rm -rf $(TARGET).* *.su
