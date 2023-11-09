
#define RCC_OFFSET                      (RCC_BASE - PERIPH_BASE)
#define CR_OFFSET                       (RCC_OFFSET + 0x00)
#define PLLON_BitNumber                 0x18
#define CR_PLLON_BB                     (PERIPH_BB_BASE + (CR_OFFSET * 32) + (PLLON_BitNumber * 4))

#define RCC_PLLSource_HSE_Div1          ((uint32_t)0x00010000)
#define RCC_PLLSource_HSE_Div2          ((uint32_t)0x00030000)

#define RCC_PLLMul_2                    ((uint32_t)0x00000000)
#define RCC_PLLMul_3                    ((uint32_t)0x00040000)
#define RCC_PLLMul_4                    ((uint32_t)0x00080000)
#define RCC_PLLMul_5                    ((uint32_t)0x000C0000)
#define RCC_PLLMul_6                    ((uint32_t)0x00100000)
#define RCC_PLLMul_7                    ((uint32_t)0x00140000)
#define RCC_PLLMul_8                    ((uint32_t)0x00180000)
#define RCC_PLLMul_9                    ((uint32_t)0x001C0000)
#define RCC_PLLMul_10                   ((uint32_t)0x00200000)
#define RCC_PLLMul_11                   ((uint32_t)0x00240000)
#define RCC_PLLMul_12                   ((uint32_t)0x00280000)
#define RCC_PLLMul_13                   ((uint32_t)0x002C0000)
#define RCC_PLLMul_14                   ((uint32_t)0x00300000)
#define RCC_PLLMul_15                   ((uint32_t)0x00340000)
#define RCC_PLLMul_16                   ((uint32_t)0x00380000)
#define RCC_PLLMul_17                   ((uint32_t)0x10000000)
#define RCC_PLLMul_18                   ((uint32_t)0x10040000)
#define RCC_PLLMul_19                   ((uint32_t)0x10080000)
#define RCC_PLLMul_20                   ((uint32_t)0x100C0000)
#define RCC_PLLMul_21                   ((uint32_t)0x10100000)
#define RCC_PLLMul_22                   ((uint32_t)0x10140000)
#define RCC_PLLMul_23                   ((uint32_t)0x10180000)
#define RCC_PLLMul_24                   ((uint32_t)0x101C0000)
#define RCC_PLLMul_25                   ((uint32_t)0x10200000)
#define RCC_PLLMul_26                   ((uint32_t)0x10240000)
#define RCC_PLLMul_27                   ((uint32_t)0x10280000)
#define RCC_PLLMul_28                   ((uint32_t)0x102C0000)
#define RCC_PLLMul_29                   ((uint32_t)0x10300000)
#define RCC_PLLMul_30                   ((uint32_t)0x10340000)
#define RCC_PLLMul_31                   ((uint32_t)0x10380000)
#define RCC_PLLMul_32                   ((uint32_t)0x103C0000)

typedef struct
{
  uint32_t SYSCLK_Frequency;  /*!< returns SYSCLK clock frequency expressed in Hz */
  uint32_t HCLK_Frequency;    /*!< returns HCLK clock frequency expressed in Hz */
  uint32_t PCLK1_Frequency;   /*!< returns PCLK1 clock frequency expressed in Hz */
  uint32_t PCLK2_Frequency;   /*!< returns PCLK2 clock frequency expressed in Hz */
  uint32_t ADCCLK_Frequency;  /*!< returns ADCCLK clock frequency expressed in Hz */
}RCC_ClocksTypeDef;
void RCC_GetClocksFreq(RCC_ClocksTypeDef* RCC_Clocks);
int intToStr(int num, char* str, int mode);
