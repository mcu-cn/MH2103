
/** @addtogroup CMSIS
  * @{
  */

/** @addtogroup mh21xx
  * @{
  */
    
#ifndef __MH21XX_H
#define __MH21XX_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */
  
/** @addtogroup Library_configuration_section
  * @{
  */

/**
  * @brief MH2 Family
  */
#if !defined (MH21)
#define MH21
#endif

/* Uncomment the line below according to the target STM32L device used in your 
   application 
  */

#if !defined (MH2100xB) && !defined (MH2100xE) && !defined (MH2101x6) && \
    !defined (MH2101xB) && !defined (MH2101xE) && !defined (MH2101xG) && !defined (MH2102x6) && !defined (MH2102xB) && !defined (MH2103x6) && \
    !defined (MH2103xB) && !defined (MH2103xE) && !defined (MH2103xG) && !defined (MH2105xC) && !defined (MH2107xC)
  /* #define MH2100xB  */   /*!< MH2100C4, MH2100R4, MH2100C6, MH2100R6, MH2100C8, MH2100R8, MH2100V8, MH2100CB, MH2100RB and MH2100VB */
  /* #define MH2100xE */    /*!< MH2100RC, MH2100VC, MH2100ZC, MH2100RD, MH2100VD, MH2100ZD, MH2100RE, MH2100VE and MH2100ZE */
  /* #define MH2101x6  */   /*!< MH2101C4, MH2101R4, MH2101T4, MH2101C6, MH2101R6 and MH2101T6 Devices */
  /* #define MH2101xB  */   /*!< MH2101C8, MH2101R8, MH2101T8, MH2101V8, MH2101CB, MH2101RB, MH2101TB and MH2101VB */
  /* #define MH2101xE */    /*!< MH2101RC, MH2101VC, MH2101ZC, MH2101RD, MH2101VD, MH2101ZD, MH2101RE, MH2101VE and MH2101ZE */ 
  /* #define MH2101xG  */   /*!< MH2101RF, MH2101VF, MH2101ZF, MH2101RG, MH2101VG and MH2101ZG */
  /* #define MH2102x6 */    /*!< MH2102C4, MH2102R4, MH2102C6 and MH2102R6 */
  /* #define MH2102xB  */   /*!< MH2102C8, MH2102R8, MH2102CB and MH2102RB */
  /* #define MH2103x6  */   /*!< MH2103C4, MH2103R4, MH2103T4, MH2103C6, MH2103R6 and MH2103T6 */
  /* #define MH2103xB  */   /*!< MH2103C8, MH2103R8, MH2103T8, MH2103V8, MH2103CB, MH2103RB, MH2103TB and MH2103VB */
  /* #define MH2103xE */    /*!< MH2103RC, MH2103VC, MH2103ZC, MH2103RD, MH2103VD, MH2103ZD, MH2103RE, MH2103VE and MH2103ZE */
  /* #define MH2103xG  */   /*!< MH2103RF, MH2103VF, MH2103ZF, MH2103RG, MH2103VG and MH2103ZG */
  /* #define MH2105xC */    /*!< MH2105R8, MH2105V8, MH2105RB, MH2105VB, MH2105RC and MH2105VC */
  /* #define MH2107xC  */   /*!< MH2107RB, MH2107VB, MH2107RC and MH2107VC */  
#endif

/*  Tip: To avoid modifying this file each time you need to switch between these
        devices, you can define the device in your toolchain compiler preprocessor.
  */
  
#if !defined  (USE_HAL_DRIVER)
/**
 * @brief Comment the line below if you will not use the peripherals drivers.
   In this case, these drivers will not be included and the application code will 
   be based on direct access to peripherals registers 
   */
  /*#define USE_HAL_DRIVER */
#endif /* USE_HAL_DRIVER */

/**
  * @brief CMSIS Device version number V4.3.1
  */
#define __MH21_CMSIS_VERSION_MAIN   (0x04) /*!< [31:24] main version */
#define __MH21_CMSIS_VERSION_SUB1   (0x03) /*!< [23:16] sub1 version */
#define __MH21_CMSIS_VERSION_SUB2   (0x01) /*!< [15:8]  sub2 version */
#define __MH21_CMSIS_VERSION_RC     (0x00) /*!< [7:0]  release candidate */ 
#define __MH21_CMSIS_VERSION        ((__MH21_CMSIS_VERSION_MAIN << 24)\
                                       |(__MH21_CMSIS_VERSION_SUB1 << 16)\
                                       |(__MH21_CMSIS_VERSION_SUB2 << 8 )\
                                       |(__MH21_CMSIS_VERSION_RC))

/**
  * @}
  */

/** @addtogroup Device_Included
  * @{
  */

#if defined(MH2100xB)
  #include "stm32f100xb.h"
#elif defined(MH2100xE)
  #include "stm32f100xe.h"
#elif defined(MH2101x6)
  #include "stm32f101x6.h"
#elif defined(MH2101xB)
  #include "stm32f101xb.h"
#elif defined(MH2101xE)
  #include "stm32f101xe.h"
#elif defined(MH2101xG)
  #include "stm32f101xg.h"
#elif defined(MH2102x6)
  #include "stm32f102x6.h"
#elif defined(MH2102xB)
  #include "stm32f102xb.h"
#elif defined(MH2103x6)
  #include "stm32f103x6.h"
#elif defined(MH2103xB)
  #include "stm32f103xb.h"
#elif defined(MH2103xE)
  #include "stm32f103xe.h"
#elif defined(MH2103xG)
  #include "stm32f103xg.h"
#elif defined(MH2105xC)
  #include "stm32f105xc.h"
#elif defined(MH2107xC)
  #include "stm32f107xc.h"
#else
 #error "Please select first the target MH21xx device used in your application (in mh21xx.h file)"
#endif

/**
  * @}
  */

/** @addtogroup Exported_types
  * @{
  */  
typedef enum 
{
  RESET = 0, 
  SET = !RESET
} FlagStatus, ITStatus;

typedef enum 
{
  DISABLE = 0, 
  ENABLE = !DISABLE
} FunctionalState;
#define IS_FUNCTIONAL_STATE(STATE) (((STATE) == DISABLE) || ((STATE) == ENABLE))

typedef enum
{
  SUCCESS = 0U,
  ERROR = !SUCCESS
} ErrorStatus;

/**
  * @}
  */


/** @addtogroup Exported_macros
  * @{
  */
#define SET_BIT(REG, BIT)     ((REG) |= (BIT))

#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))

#define READ_BIT(REG, BIT)    ((REG) & (BIT))

#define CLEAR_REG(REG)        ((REG) = (0x0))

#define WRITE_REG(REG, VAL)   ((REG) = (VAL))

#define READ_REG(REG)         ((REG))

#define MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))

#define POSITION_VAL(VAL)     (__CLZ(__RBIT(VAL))) 


/**
  * @}
  */

#if defined (USE_HAL_DRIVER)
 #include "stm32f1xx_hal.h"
#endif /* USE_HAL_DRIVER */


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __MH21xx_H */
/**
  * @}
  */

/**
  * @}
  */
  



/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
