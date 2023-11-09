// Code to setup clocks and gpio on mh2103
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "autoconf.h" // CONFIG_CLOCK_REF_FREQ
#include "board/armcm_boot.h" // VectorTable
#include "board/armcm_reset.h" // try_request_canboot
#include "board/irq.h" // irq_disable
#include "board/misc.h" // bootloader_request
#include "internal.h" // enable_pclock
#include "sched.h" // sched_main
#include "mh2_rcc_cfg.h"


/****************************************************************
 * Clock setup
 ****************************************************************/

#define HSI_VALUE    ((uint32_t)8000000)
#define HSE_VALUE    ((uint32_t)8000000)
static __I uint8_t APBAHBPrescTable[16] = {0, 0, 0, 0, 1, 2, 3, 4, 1, 2, 3, 4, 6, 7, 8, 9};
static __I uint8_t ADCPrescTable[4] = {2, 4, 6, 8};
void RCC_GetClocksFreq(RCC_ClocksTypeDef* RCC_Clocks)
{
  uint32_t tmp = 0, pllmull = 0, pllsource = 0, presc = 0;
  uint32_t i;

  /* Get SYSCLK source -------------------------------------------------------*/
  tmp = RCC->CFGR & (uint32_t)0x0000000C;
  
  switch (tmp)
  {
    case 0x00:  /* HSI used as system clock */
      RCC_Clocks->SYSCLK_Frequency = HSI_VALUE;
      break;
    case 0x04:  /* HSE used as system clock */
      RCC_Clocks->SYSCLK_Frequency = HSE_VALUE;
      break;
    case 0x08:  /* PLL used as system clock */

      /* Get PLL clock source and multiplication factor ----------------------*/
      pllmull = RCC->CFGR & (uint32_t)0x003C0000;
      pllsource = RCC->CFGR & (uint32_t)0x00010000;
	
	  if (RCC->CFGR >> 28 & 0x01)
	  {
		i = 0x01;
		pllmull = (i << 4 | ( pllmull >> 18)) + 1;
	  }
	  else
	  {
		pllmull = ( pllmull >> 18) + 2 ;
	  }
		  

      if (pllsource == 0x00)
      {/* HSI oscillator clock divided by 2 selected as PLL clock entry */
        RCC_Clocks->SYSCLK_Frequency = (HSI_VALUE >> 1) * pllmull;
      }
      else
      {
        /* HSE selected as PLL clock entry */
        if ((RCC->CFGR & (uint32_t)0x00020000) != (uint32_t)RESET)
        {/* HSE oscillator clock divided by 2 */
          RCC_Clocks->SYSCLK_Frequency = (HSE_VALUE >> 1) * pllmull;
        }
        else
        {
          RCC_Clocks->SYSCLK_Frequency = HSE_VALUE * pllmull;
        }
 
      }
      break;

    default:
      RCC_Clocks->SYSCLK_Frequency = HSI_VALUE;
      break;
  }

  /* Compute HCLK, PCLK1, PCLK2 and ADCCLK clocks frequencies ----------------*/
  /* Get HCLK prescaler */
  tmp = RCC->CFGR & (uint32_t)0x000000F0;
  tmp = tmp >> 4;
  presc = APBAHBPrescTable[tmp];
  /* HCLK clock frequency */
  RCC_Clocks->HCLK_Frequency = RCC_Clocks->SYSCLK_Frequency >> presc;
  /* Get PCLK1 prescaler */
  tmp = RCC->CFGR & (uint32_t)0x00000700;
  tmp = tmp >> 8;
  presc = APBAHBPrescTable[tmp];
  /* PCLK1 clock frequency */
  RCC_Clocks->PCLK1_Frequency = RCC_Clocks->HCLK_Frequency >> presc;
  /* Get PCLK2 prescaler */
  tmp = RCC->CFGR & (uint32_t)0x00003800;
  tmp = tmp >> 11;
  presc = APBAHBPrescTable[tmp];
  /* PCLK2 clock frequency */
  RCC_Clocks->PCLK2_Frequency = RCC_Clocks->HCLK_Frequency >> presc;
  /* Get ADCCLK prescaler */
  tmp = RCC->CFGR & (uint32_t)0x0000C000;
  tmp = tmp >> 14;
  presc = ADCPrescTable[tmp];
  /* ADCCLK clock frequency */
  RCC_Clocks->ADCCLK_Frequency = RCC_Clocks->PCLK2_Frequency / presc;
}
int intToStr(int num, char* str, int mode) //mode = 2, 8, 16, 10
{
    int tempv = num, n = 0;
    uint32_t temp,t;
    if (!num){
        str[0] = '0';
        str[1] = 0;
        return 1;
    }
    if (mode == 10 && tempv < 0){
        str[0] = '-';
        n = 1;
        temp = -tempv;
    }
    else temp  = tempv;
    for (t = temp; t; n++)t /= mode;//求出显示位数
    str[n] = 0;
    for (t = n - 1; temp > 0; t--){
        str[t] = temp % mode;
        if(str[t] > 9)str[t] = str[t] + 'A' - 10;
        else str[t] += '0';
        temp /= mode;
    }
    return n;
}

#define FREQ_PERIPH (CONFIG_CLOCK_FREQ / 2)

// Map a peripheral address to its enable bits
struct cline
lookup_clock_line(uint32_t periph_base)
{
    if (periph_base >= AHBPERIPH_BASE) {
        uint32_t bit = 1 << ((periph_base - AHBPERIPH_BASE) / 0x400);
        return (struct cline){.en=&RCC->AHBENR, .bit=bit};
    } else if (periph_base >= APB2PERIPH_BASE) {
        uint32_t bit = 1 << ((periph_base - APB2PERIPH_BASE) / 0x400);
        return (struct cline){.en=&RCC->APB2ENR, .rst=&RCC->APB2RSTR, .bit=bit};
    } else {
        uint32_t bit = 1 << ((periph_base - APB1PERIPH_BASE) / 0x400);
        return (struct cline){.en=&RCC->APB1ENR, .rst=&RCC->APB1RSTR, .bit=bit};
    }
}

// Return the frequency of the given peripheral clock
uint32_t
get_pclock_frequency(uint32_t periph_base)
{
    RCC_ClocksTypeDef rcc;
    RCC_GetClocksFreq(&rcc);
    switch(periph_base) {
    case (uint32_t)TIM2:
    case (uint32_t)TIM3:
    case (uint32_t)TIM4:
    case (uint32_t)TIM5:
    case (uint32_t)TIM6:
    case (uint32_t)TIM7:
    case (uint32_t)SPI2:
    case (uint32_t)SPI3:
    case (uint32_t)CAN1:
    case (uint32_t)USB:
    case (uint32_t)USART2:
    case (uint32_t)USART3:
    case (uint32_t)UART4:
    case (uint32_t)UART5: return rcc.PCLK1_Frequency;
    case (uint32_t)SPI1:
    case (uint32_t)ADC1:
    case (uint32_t)ADC2:
    case (uint32_t)ADC3:
    case (uint32_t)TIM1:
    case (uint32_t)USART1: return rcc.PCLK2_Frequency;
    }
    return FREQ_PERIPH;
}

// Enable a GPIO peripheral clock
void
gpio_clock_enable(GPIO_TypeDef *regs)
{
    uint32_t rcc_pos = ((uint32_t)regs - APB2PERIPH_BASE) / 0x400;
    RCC->APB2ENR |= 1 << rcc_pos;
    RCC->APB2ENR;
}

void rccDeInit(void)
{
  /* Set HSION bit */
  RCC->CR |= (uint32_t)0x00000001;

  /* Reset SW, HPRE, PPRE1, PPRE2, ADCPRE and MCO bits */
  RCC->CFGR &= (uint32_t)0xF8FF0000;
  
  /* Reset HSEON, CSSON and PLLON bits */
  RCC->CR &= (uint32_t)0xFEF6FFFF;

  /* Reset HSEBYP bit */
  RCC->CR &= (uint32_t)0xFFFBFFFF;

  /* Reset PLLSRC, PLLXTPRE, PLLMUL and USBPRE/OTGFSPRE bits */
  RCC->CFGR &= (uint32_t)0xFF80FFFF;

  /* Disable all interrupts and clear pending bits  */
  RCC->CIR = 0x009F0000;
}
// Main clock setup called at chip startup
static void
clock_setup(void)
{
    #if CONFIG_MH2_SYSCLK_216
        #if CONFIG_MH2_CLOCK_REF_INTERNAL //216必须使用外部晶振
        #error must config external crystal when sysclk is 216MHz
        #endif
        uint32_t MH_RCC_PLLConfig(uint32_t RCC_PLLSource, uint32_t RCC_PLLMul, uint8_t Latency);
        rccDeInit();
        RCC->CR |= RCC_CR_HSEON;//打开HSE
        while(!(RCC->CR & RCC_CR_HSERDY));//待HSE稳定
        *(__IO uint32_t *) CR_PLLON_BB = 0;
        MH_RCC_PLLConfig(RCC_PLLSource_HSE_Div1,RCC_PLLMul_27,1);
        RCC->CR |= RCC_CR_PLLON;
        while (!(RCC->CR & RCC_CR_PLLRDY));//待PLL稳定
        uint32_t temp = RCC->CFGR;
        temp &= 0xFFFFFFFC; //[1:0]
        temp |= 0x00000002; //选择PLL作系统时钟源
        temp &= 0xFFFFFF0F; //[7:4]
        // temp |= 0x00000000; //HCLK-div1
        temp &= 0xFFFFF8FF; //[10:8]
        temp |= 0x00000400; //PPRE1-div2
        temp &= 0xFFFFC7FF; //[13:11]
        // temp |= 0x00000000; //PPRE2-div1
        RCC->CFGR = temp;
    #else
        // Configure and enable PLL
        uint32_t cfgr;
        if (!CONFIG_MH2_CLOCK_REF_INTERNAL) {
            // Configure 72Mhz PLL from external crystal (HSE)
            RCC->CR |= RCC_CR_HSEON;
            uint32_t div = CONFIG_CLOCK_FREQ / (CONFIG_CLOCK_REF_FREQ / 2);
            cfgr = 1 << RCC_CFGR_PLLSRC_Pos;
            if ((div & 1) && div <= 16)
                cfgr |= RCC_CFGR_PLLXTPRE_HSE_DIV2;
            else
                div /= 2;
            cfgr |= (div - 2) << RCC_CFGR_PLLMULL_Pos;
        } else {
            // Configure 72Mhz PLL from internal 8Mhz oscillator (HSI)
            uint32_t div2 = (CONFIG_CLOCK_FREQ / 8000000) * 2;
            cfgr = ((0 << RCC_CFGR_PLLSRC_Pos)
                    | ((div2 - 2) << RCC_CFGR_PLLMULL_Pos));
        }
        cfgr |= RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_PPRE2_DIV2 | RCC_CFGR_ADCPRE_DIV8;
        RCC->CFGR = cfgr;
        RCC->CR |= RCC_CR_PLLON;

        // Set flash latency
        FLASH->ACR = (2 << FLASH_ACR_LATENCY_Pos) | FLASH_ACR_PRFTBE;

        // Wait for PLL lock
        while (!(RCC->CR & RCC_CR_PLLRDY))
            ;

        // Switch system clock to PLL
        RCC->CFGR = cfgr | RCC_CFGR_SW_PLL;
        while ((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_PLL)
            ;
    #endif
}


/****************************************************************
 * GPIO setup
 ****************************************************************/

static void
mh21_alternative_remap(uint32_t mapr_mask, uint32_t mapr_value)
{
    // The MAPR register is a mix of write only and r/w bits
    // We have to save the written values in a global variable
    static uint32_t mapr = 0;

    mapr &= ~mapr_mask;
    mapr |= mapr_value;
    AFIO->MAPR = mapr;
}

#define STM_OSPEED 0x1 // ~10Mhz at 50pF

// Set the mode and extended function of a pin
void
gpio_peripheral(uint32_t gpio, uint32_t mode, int pullup)
{
    GPIO_TypeDef *regs = digital_regs[GPIO2PORT(gpio)];

    // Enable GPIO clock
    gpio_clock_enable(regs);

    // Configure GPIO
    uint32_t pos = gpio % 16, shift = (pos % 8) * 4, msk = 0xf << shift, cfg;
    if (mode == GPIO_INPUT) {
        cfg = pullup ? 0x8 : 0x4;
    } else if (mode == GPIO_OUTPUT) {
        cfg = STM_OSPEED;
    } else if (mode == (GPIO_OUTPUT | GPIO_OPEN_DRAIN)) {
        cfg = 0x4 | STM_OSPEED;
    } else if (mode == GPIO_ANALOG) {
        cfg = 0x0;
    } else {
        if (mode & GPIO_OPEN_DRAIN)
            // Alternate function with open-drain mode
            cfg = 0xc | STM_OSPEED;
        else if (pullup > 0)
            // Alternate function input pins use GPIO_INPUT mode on the mh21
            cfg = 0x8;
        else
            cfg = 0x8 | STM_OSPEED;
    }
    if (pos & 0x8)
        regs->CRH = (regs->CRH & ~msk) | (cfg << shift);
    else
        regs->CRL = (regs->CRL & ~msk) | (cfg << shift);

    if (pullup > 0)
        regs->BSRR = 1 << pos;
    else if (pullup < 0)
        regs->BSRR = 1 << (pos + 16);

    if (gpio == GPIO('A', 13) || gpio == GPIO('A', 14))
        // Disable SWD to free PA13, PA14
        mh21_alternative_remap(AFIO_MAPR_SWJ_CFG_Msk,
                                  AFIO_MAPR_SWJ_CFG_DISABLE);

    // MH21 remaps functions to pins in a very different
    // way from other MH2s.
    // Code below is emulating a few mappings to work like an MH24
    uint32_t func = (mode >> 4) & 0xf;
    if (func == 1) {
        // TIM2
        if (gpio == GPIO('A', 15) || gpio == GPIO('B', 3))
            mh21_alternative_remap(AFIO_MAPR_TIM2_REMAP_Msk,
                                      AFIO_MAPR_TIM2_REMAP_PARTIALREMAP1);
        else if (gpio == GPIO('B', 10) || gpio == GPIO('B', 11))
            mh21_alternative_remap(AFIO_MAPR_TIM2_REMAP_Msk,
                                      AFIO_MAPR_TIM2_REMAP_PARTIALREMAP2);
    } else if (func == 2) {
        // TIM3 and TIM4
        if (gpio == GPIO('B', 4) || gpio == GPIO('B', 5))
            mh21_alternative_remap(AFIO_MAPR_TIM3_REMAP_Msk,
                                      AFIO_MAPR_TIM3_REMAP_PARTIALREMAP);
        else if (gpio == GPIO('C', 6) || gpio == GPIO('C', 7)
                 || gpio == GPIO('C', 8) || gpio == GPIO('C', 9))
            mh21_alternative_remap(AFIO_MAPR_TIM3_REMAP_Msk,
                                      AFIO_MAPR_TIM3_REMAP_FULLREMAP);
        else if (gpio == GPIO('D', 12) || gpio == GPIO('D', 13)
                 || gpio == GPIO('D', 14) || gpio == GPIO('D', 15))
            mh21_alternative_remap(AFIO_MAPR_TIM4_REMAP_Msk,
                                      AFIO_MAPR_TIM4_REMAP);
    } else if (func == 4) {
        // I2C
        if (gpio == GPIO('B', 8) || gpio == GPIO('B', 9))
            mh21_alternative_remap(AFIO_MAPR_I2C1_REMAP_Msk,
                                      AFIO_MAPR_I2C1_REMAP);
    } else if (func == 5) {
        // SPI
        if (gpio == GPIO('B', 3) || gpio == GPIO('B', 4)
            || gpio == GPIO('B', 5))
            mh21_alternative_remap(AFIO_MAPR_SPI1_REMAP_Msk,
                                      AFIO_MAPR_SPI1_REMAP);
    } else if (func == 7) {
        // USART
        if (gpio == GPIO('B', 6) || gpio == GPIO('B', 7))
            mh21_alternative_remap(AFIO_MAPR_USART1_REMAP_Msk,
                                      AFIO_MAPR_USART1_REMAP);
        else if (gpio == GPIO('D', 5) || gpio == GPIO('D', 6))
            mh21_alternative_remap(AFIO_MAPR_USART2_REMAP_Msk,
                                      AFIO_MAPR_USART2_REMAP);
        else if (gpio == GPIO('D', 8) || gpio == GPIO('D', 9))
            mh21_alternative_remap(AFIO_MAPR_USART3_REMAP_Msk,
                                      AFIO_MAPR_USART3_REMAP_FULLREMAP);
    } else if (func == 9) {
        // CAN
        if (gpio == GPIO('B', 8) || gpio == GPIO('B', 9))
            mh21_alternative_remap(AFIO_MAPR_CAN_REMAP_Msk,
                                      AFIO_MAPR_CAN_REMAP_REMAP2);
        if (gpio == GPIO('D', 0) || gpio == GPIO('D', 1))
            mh21_alternative_remap(AFIO_MAPR_CAN_REMAP_Msk,
                                      AFIO_MAPR_CAN_REMAP_REMAP3);
    }
}


/****************************************************************
 * Bootloader
 ****************************************************************/

// Reboot into USB "HID" bootloader
static void
usb_hid_bootloader(void)
{
    irq_disable();
    RCC->APB1ENR |= RCC_APB1ENR_PWREN | RCC_APB1ENR_BKPEN;
    PWR->CR |= PWR_CR_DBP;
    BKP->DR4 = 0x424C; // HID Bootloader magic key
    PWR->CR &=~ PWR_CR_DBP;
    NVIC_SystemReset();
}

// Reboot into USB "mh2duino" bootloader
static void
usb_mh2duino_bootloader(void)
{
    irq_disable();
    RCC->APB1ENR |= RCC_APB1ENR_PWREN | RCC_APB1ENR_BKPEN;
    PWR->CR |= PWR_CR_DBP;
    BKP->DR10 = 0x01; // mh2duino bootloader magic key
    PWR->CR &=~ PWR_CR_DBP;
    NVIC_SystemReset();
}

// Handle reboot requests
void
bootloader_request(void)
{
    try_request_canboot();
    if (CONFIG_MH2_FLASH_START_800)
        usb_hid_bootloader();
    else if (CONFIG_MH2_FLASH_START_2000)
        usb_mh2duino_bootloader();
}


/****************************************************************
 * Startup
 ****************************************************************/
// Main entry point - called from armcm_boot.c:ResetHandler()
void
armcm_main(void)
{
    // Run SystemInit() and then restore VTOR
    SystemInit();
    
    SCB->VTOR = (uint32_t)VectorTable;

    // Reset peripheral clocks (for some bootloaders that don't)
    RCC->AHBENR = 0x14;
    RCC->APB1ENR = 0;
    RCC->APB2ENR = 0;

    // Setup clocks
    clock_setup();

    // Disable JTAG to free PA15, PB3, PB4
    enable_pclock(AFIO_BASE);
    mh21_alternative_remap(AFIO_MAPR_SWJ_CFG_Msk, AFIO_MAPR_SWJ_CFG_JTAGDISABLE);
    
    sched_main();
}
