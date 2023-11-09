#ifndef __MH2_INTERNAL_H
#define __MH2_INTERNAL_H
// Local definitions for MH2 code

#include "autoconf.h" // CONFIG_MACH_MH21

#if CONFIG_MACH_MH20
#include "mh20xx.h"
#elif CONFIG_MACH_MH21
#include "mh21xx.h"
#elif CONFIG_MACH_MH22
#include "mh22xx.h"
#elif CONFIG_MACH_MH24
#include "mh24xx.h"
#elif CONFIG_MACH_MH27
#include "mh27xx.h"
#endif

// gpio.c
extern GPIO_TypeDef * const digital_regs[];
#define GPIO(PORT, NUM) (((PORT)-'A') * 16 + (NUM))
#define GPIO2PORT(PIN) ((PIN) / 16)
#define GPIO2BIT(PIN) (1<<((PIN) % 16))

// gpioperiph.c
#define GPIO_INPUT 0
#define GPIO_OUTPUT 1
#define GPIO_OPEN_DRAIN 0x100
#define GPIO_HIGH_SPEED 0x200
#define GPIO_FUNCTION(fn) (2 | ((fn) << 4))
#define GPIO_ANALOG 3
void gpio_peripheral(uint32_t gpio, uint32_t mode, int pullup);

// clockline.c
void enable_pclock(uint32_t periph_base);
int is_enabled_pclock(uint32_t periph_base);

// dfu_reboot.c
void dfu_reboot(void);
void dfu_reboot_check(void);

// mh2??.c
struct cline { volatile uint32_t *en, *rst; uint32_t bit; };
struct cline lookup_clock_line(uint32_t periph_base);
uint32_t get_pclock_frequency(uint32_t periph_base);
void gpio_clock_enable(GPIO_TypeDef *regs);

#endif // internal.h
