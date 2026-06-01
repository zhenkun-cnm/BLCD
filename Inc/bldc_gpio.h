#ifndef __BLDC_GPIO_H
#define __BLDC_GPIO_H

#include "stm32f0xx.h"  /* 直接共享官方基地址定义（如 GPIOA, GPIOB） */

/* ========================================================================== */
/* 1. 业务层引脚枚举（仅此一家，绝对不与任何库重复）                            */
/* ========================================================================== */
typedef enum {
    BLDC_PIN_AD = 0,    /* 下桥臂 A */
    BLDC_PIN_BD,        /* 下桥臂 B */
    BLDC_PIN_CD,        /* 下桥臂 C */
    BLDC_PIN_PWM_IN,    /* 示例：复用输入引脚 */
    BLDC_PIN_MAX
} BLDC_Gpio_Idx_t;

/* ========================================================================== */
/* 2. 寄存器专用全功能配置枚举（全部加 REG_ 前缀，100% 避开 HAL 库重名）          */
/* ========================================================================== */

/* 2.1 纯数字引脚号枚举 (0 ~ 15) */
typedef enum {
    REG_PIN_0  = 0,  REG_PIN_1  = 1,  REG_PIN_2  = 2,  REG_PIN_3  = 3,
    REG_PIN_4  = 4,  REG_PIN_5  = 5,  REG_PIN_6  = 6,  REG_PIN_7  = 7,
    REG_PIN_8  = 8,  REG_PIN_9  = 9,  REG_PIN_10 = 10, REG_PIN_11 = 11,
    REG_PIN_12 = 12, REG_PIN_13 = 13, REG_PIN_14 = 14, REG_PIN_15 = 15
} REG_Pin_Num_t;

/* 2.2 引脚功能模式枚举 (MODER: 2-bit) */
typedef enum {
    REG_MODE_IN     = 0x0u,    /* 输入模式 */
    REG_MODE_OUT    = 0x1u,    /* 通用输出模式 */
    REG_MODE_AF     = 0x2u,    /* 复用功能模式 */
    REG_MODE_ANALOG = 0x3u     /* 模拟输入输出模式 */
} REG_Mode_t;

/* 2.3 输出类型枚举 (OTYPER: 1-bit) */
typedef enum {
    REG_OTYPE_PP    = 0x0u,    /* 推挽输出 (Push-Pull) */
    REG_OTYPE_OD    = 0x1u     /* 开漏输出 (Open-Drain) */
} REG_OType_t;

/* 2.4 输出速度枚举 (OSPEEDR: 2-bit) */
typedef enum {
    REG_SPEED_LOW   = 0x0u,    /* 低速 (2MHz) */
    REG_SPEED_MED   = 0x1u,    /* 中速 (10MHz) */
    REG_SPEED_HIGH  = 0x3u     /* 高速 (50MHz) */
} REG_Speed_t;

/* 2.5 内部上下拉电阻枚举 (PUPDR: 2-bit) */
typedef enum {
    REG_PUPD_NONE   = 0x0u,    /* 无上下拉 */
    REG_PUPD_UP     = 0x1u,    /* 开启上拉电阻 */
    REG_PUPD_DOWN   = 0x2u     /* 开启下拉电阻 */
} REG_PUpD_t;

/* 2.6 AF 复用选择枚举 (AFR: 4-bit) */
typedef enum {
    REG_AF_0 = 0,  REG_AF_1 = 1,  REG_AF_2 = 2,  REG_AF_3 = 3,
    REG_AF_4 = 4,  REG_AF_5 = 5,  REG_AF_6 = 6,  REG_AF_7 = 7
} REG_Af_t;

/* ========================================================================== */
/* 3. 公共接口声明                                                            */
/* ========================================================================== */
void BSP_GPIO_Init(void);
void BSP_GPIO_WriteHigh(BLDC_Gpio_Idx_t pin);
void BSP_GPIO_WriteLow(BLDC_Gpio_Idx_t pin);
void BSP_GPIO_Toggle(BLDC_Gpio_Idx_t pin);
uint8_t BSP_GPIO_ReadInput(BLDC_Gpio_Idx_t pin);

#endif /* __BLDC_GPIO_H */

