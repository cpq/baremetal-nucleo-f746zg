TARGET  = firmware
CROSS   = arm-none-eabi
OPTS    ?= -g3 -Os -ffunction-sections -fdata-sections
WARN    ?= -W -Wall -Wextra -Werror -Wundef -Wshadow -Wdouble-promotion -Wformat-truncation -fno-common -Wconversion
DEFS    ?= -I. -Imongoose -DMG_ARCH=MG_ARCH_NEWLIB -DMIP_DEBUG=1 -DMG_ENABLE_CUSTOM_MILLIS=1 -DMG_ENABLE_LINES=1 -DMG_ENABLE_MIP=1
MCUFL   ?= -mcpu=cortex-m7 -mthumb -mfloat-abi=softfp -mfpu=vfpv4
CFLAGS  ?= $(WARN) $(OPTS) $(MCUFL) $(DEFS) $(DEFS) $(EXTRA)
LDFLAGS ?= -Tlink.ld -nostartfiles -nostdlib --specs nano.specs -lc -lgcc -Wl,--gc-sections -Wl,-Map=$@.map
SOURCES = boot.c main.c syscalls.c mongoose/drivers/mip_driver_stm32.c mongoose/mongoose.c

all: $(TARGET).bin

mongoose/mongoose.c:
	git clone --depth 1 https://github.com/cesanta/mongoose

$(TARGET).bin: $(TARGET).elf
	$(CROSS)-objcopy -O binary $< $@

$(TARGET).elf: $(SOURCES) mcu.h
	$(CROSS)-gcc $(SOURCES) $(CFLAGS) $(LDFLAGS) -o $@

# Build an interactive device dashboard
dash: $(TARGET).elf
dash: DEFS += -DDASH -DMG_ENABLE_PACKED_FS=1
dash: SOURCES += mongoose/examples/device-dashboard/packed_fs.c mongoose/examples/device-dashboard/web.c

# Show top 10 stack-hungry functions
su: CFLAGS += -fstack-usage
su: $(TARGET).elf
	cat *.su | sort -rnk2 | head -10

# Note: on "unknown chip id" flash error, wire BOOT0 to VDD and st-flash erase
flash: $(TARGET).bin
	st-flash --reset write $(TARGET).bin 0x8000000

clean:
	@rm -rf $(TARGET).* *.su
