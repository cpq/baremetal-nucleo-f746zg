TARGET  = firmware
CROSS   = arm-none-eabi
OPTS    ?= -g3 -Os -ffunction-sections -fdata-sections
WARN    ?= -W -Wall -Wextra -Werror -Wundef -Wshadow -Wdouble-promotion -Wformat-truncation -fno-common -Wconversion
DEFS    ?= -I. -Imongoose -Imip -DMG_ARCH=MG_ARCH_NEWLIB -DMIP_DEBUG=1 -DMG_ENABLE_CUSTOM_MILLIS=1 -DMG_ENABLE_LINES=1
MCUFL   ?= -mcpu=cortex-m7 -mthumb -mfloat-abi=softfp -mfpu=vfpv4
CFLAGS  ?= $(WARN) $(OPTS) $(MCUFL) $(DEFS) $(DEFS) $(EXTRA)
LDFLAGS ?= -Tlink.ld -nostartfiles -nostdlib --specs nano.specs -lc -lgcc -Wl,--gc-sections -Wl,-Map=$@.map
SOURCES = boot.c main.c syscalls.c mip/mip.c mip/drivers/mip_driver_stm32.c mongoose/mongoose.c
PORT    ?= /dev/ttyACM0

all: $(TARGET).bin

mongoose/mongoose.c:
	git clone --depth 1 https://github.com/cesanta/mongoose

mip/mip.c:
	git clone --depth 1 https://github.com/cesanta/mip

$(TARGET).bin: $(TARGET).elf
	$(CROSS)-objcopy -O binary $< $@

$(TARGET).elf: $(SOURCES) mcu.h
	$(CROSS)-gcc $(SOURCES) $(CFLAGS) $(LDFLAGS) -o $@

dash: $(TARGET).elf
dash: DEFS += -DDASH -DMG_ENABLE_PACKED_FS=1
dash: SOURCES += mongoose/examples/device-dashboard/packed_fs.c mongoose/examples/device-dashboard/web.c

su: CFLAGS += -fstack-usage
su: $(TARGET).elf
	cat *.su | sort -rnk2 | head -10

# Note: on "unknown chip id" flash error, wire BOOT0 to VDD and st-flash erase
flash: $(TARGET).bin
	st-flash --reset write $(TARGET).bin 0x8000000

mon:
	cat < $(PORT)  # Or, use your favorite terminal utility like cu, minicom

clean:
	@rm -rf $(TARGET).* *.su
