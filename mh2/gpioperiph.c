// Code to setup gpio on MH2 chip (except for MH21)
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "internal.h" // gpio_peripheral

// Set the mode, extended function and speed of a pin
void
gpio_peripheral(uint32_t gpio, uint32_t mode, int pullup)
{
    GPIO_TypeDef *regs = digital_regs[GPIO2PORT(gpio)];

    // Enable GPIO clock
    gpio_clock_enable(regs);

    // Configure GPIO
    uint32_t mode_bits = mode & 0xf, func = (mode >> 4) & 0xf;
    uint32_t od = (mode >> 8) & 0x1, hs = (mode >> 9) & 0x1;
    uint32_t pup = pullup ? (pullup > 0 ? 1 : 2) : 0;
    uint32_t pos = gpio % 16, af_reg = pos / 8;
    uint32_t af_shift = (pos % 8) * 4, af_msk = 0x0f << af_shift;
    uint32_t m_shift = pos * 2, m_msk = 0x03 << m_shift;

    regs->AFR[af_reg] = (regs->AFR[af_reg] & ~af_msk) | (func << af_shift);
    regs->MODER = (regs->MODER & ~m_msk) | (mode_bits << m_shift);
    regs->PUPDR = (regs->PUPDR & ~m_msk) | (pup << m_shift);
    regs->OTYPER = (regs->OTYPER & ~(1 << pos)) | (od << pos);

    // Setup OSPEEDR:
    // MH20 is ~10Mhz at 50pF
    // MH22 is ~25Mhz at 40pF
    // MH24 is ~50Mhz at 40pF
    // MH27 is ~50Mhz at 40pF
    // MH2G0 is ~30Mhz at 50pF
    // MH2H7 is ~85Mhz at 50pF
    uint32_t ospeed = hs ? 0x03 : (CONFIG_MACH_MH20 ? 0x01 : 0x02);
    regs->OSPEEDR = (regs->OSPEEDR & ~m_msk) | (ospeed << m_shift);
}
