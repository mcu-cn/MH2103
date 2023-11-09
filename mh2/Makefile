
# Setup the toolchain
CROSS_PREFIX=arm-none-eabi-
SRC_DR := mh2
LIB_DR := lib/mh2

dirs-y += src/$(SRC_DR) src/generic lib/fast-hash
dirs-$(CONFIG_MACH_MH21) += $(LIB_DR)

MCU := $(shell echo $(CONFIG_MCU))
MCU_UPPER := $(shell echo $(CONFIG_MCU) | tr a-z A-Z | tr X x) #将MCU名a-z变到A-Z（变大写），再把X变小写

CFLAGS-$(CONFIG_MACH_MH21) += -mcpu=cortex-m3 -I$(LIB_DR)/include
CFLAGS += $(CFLAGS-y) -D$(MCU_UPPER) -mthumb -Ilib/cmsis-core -Ilib/fast-hash

CFLAGS_klipper.elf += --specs=nano.specs --specs=nosys.specs
CFLAGS_klipper.elf += -T $(OUT)src/generic/armcm_link.ld
CFLAGS_klipper.elf += -L$(LIB_DR) -lcrypt_gnu10 -lmh2103_gnu10
$(OUT)klipper.elf: $(OUT)src/generic/armcm_link.ld

# Add source files
src-y += $(SRC_DR)/watchdog.c $(SRC_DR)/gpio.c $(SRC_DR)/clockline.c $(SRC_DR)/dfu_reboot.c
src-y += generic/crc16_ccitt.c
src-y += generic/armcm_boot.c generic/armcm_irq.c generic/armcm_reset.c
src-$(CONFIG_MACH_MH21) += ../$(LIB_DR)/system_mh21xx.c
src-$(CONFIG_MACH_MH21) += $(SRC_DR)/adc.c
src-$(CONFIG_MACH_MH21) += $(SRC_DR)/mh21.c generic/armcm_timer.c $(SRC_DR)/i2c.c
spi-src-y := $(SRC_DR)/spi.c
src-$(CONFIG_HAVE_GPIO_SPI) += $(spi-src-y)
sdio-src-y := $(SRC_DR)/sdio.c
src-$(CONFIG_HAVE_GPIO_SDIO) += $(sdio-src-y)
usb-src-$(CONFIG_HAVE_MH2_USBFS) := $(SRC_DR)/usbfs.c
usb-src-$(CONFIG_HAVE_MH2_USBOTG) := $(SRC_DR)/usbotg.c
src-$(CONFIG_USBSERIAL) += $(usb-src-y) $(SRC_DR)/chipid.c generic/usb_cdc.c
serial-src-y := $(SRC_DR)/serial.c
src-$(CONFIG_SERIAL) += $(serial-src-y) generic/serial_irq.c
canbus-src-y := generic/canserial.c ../lib/fast-hash/fasthash.c
canbus-src-$(CONFIG_HAVE_MH2_CANBUS) += $(SRC_DR)/can.c
canbus-src-$(CONFIG_HAVE_MH2_FDCANBUS) += $(SRC_DR)/fdcan.c
src-$(CONFIG_CANSERIAL) += $(canbus-src-y) generic/canbus.c $(SRC_DR)/chipid.c
src-$(CONFIG_USBCANBUS) += $(usb-src-y) $(canbus-src-y)
src-$(CONFIG_USBCANBUS) += $(SRC_DR)/chipid.c generic/usb_canbus.c
src-$(CONFIG_HAVE_GPIO_HARD_PWM) += $(SRC_DR)/hard_pwm.c

# Binary output file rules
target-y += $(OUT)firmware.hex

# $(OUT)klipper.bin: $(OUT)klipper.elf
# 	@echo "  Creating bin file $@"
# 	$(Q)$(OBJCOPY) -O binary $< $@

$(OUT)firmware.hex: $(OUT)klipper.elf
	@echo "  Creating hex file $@"
	$(Q)$(OBJCOPY) -O ihex $< $@
