#include "bldc_gpio.h"

/* 映射表结构体：内部成员对齐 REG_ 前缀枚举 */
typedef struct {
    GPIO_TypeDef* port;       /* 官方物理端口指针 (如 GPIOA, GPIOB) */
    REG_Pin_Num_t  pin_num;    /* 引脚号 */
    REG_Mode_t     mode;       /* 输入 / 输出 / 复用 / 模拟 */
    REG_OType_t    otype;      /* 推挽 / 开漏 */
    REG_Speed_t    speed;      /* 低速 / 中速 / 高速 */
    REG_PUpD_t     pupd;       /* 无 / 上拉 / 下拉 */
    REG_Af_t       af_sel;     /* 复用通道选择 (REG_AF_0 ~ REG_AF_7) */
} Gpio_Config_Map_t;

/* ========================================================================== */
/* 核心引脚物理映射配置表                                                      */
/* 全部用全新的 REG_ 前缀枚举填充，既保证了代码可读性，又做到了 0 报错风险     */
/* ========================================================================== */
static const Gpio_Config_Map_t s_gpio_config_table[BLDC_PIN_MAX] = {
    /* 业务引脚名          官方端口  引脚号       输入输出模式    输出类型      输出速度        上下拉电阻     AF通道 */
    [BLDC_PIN_AD]     = { GPIOB, REG_PIN_7, REG_MODE_OUT, REG_OTYPE_PP, REG_SPEED_HIGH, REG_PUPD_DOWN, REG_AF_0 },
    [BLDC_PIN_BD]     = { GPIOB, REG_PIN_6, REG_MODE_OUT, REG_OTYPE_PP, REG_SPEED_HIGH, REG_PUPD_DOWN, REG_AF_0 },
    [BLDC_PIN_CD]     = { GPIOB, REG_PIN_5, REG_MODE_OUT, REG_OTYPE_PP, REG_SPEED_HIGH, REG_PUPD_DOWN, REG_AF_0 },
    
    /* 示例：如需配置复用输入引脚（例如将 PA0 复用为 TIM2_CH1 -> AF2） */
    [BLDC_PIN_PWM_IN] = { GPIOA, REG_PIN_0, REG_MODE_AF,  REG_OTYPE_PP, REG_SPEED_HIGH, REG_PUPD_NONE, REG_AF_2 }
};

/**
  * @brief  GPIO 全功能寄存器自动化初始化（支持高低位 AF 自动路由）
  */
void BSP_GPIO_Init(void)
{
    /* 1. 强行开启常用 GPIO 分组的 AHB 总线时钟 */
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN;

    /* 2. 扫描配置表，自动进行精准的位运算写入 */
    for (uint32_t i = 0; i < BLDC_PIN_MAX; i++)
    {
        const Gpio_Config_Map_t *cfg = &s_gpio_config_table[i];
        GPIO_TypeDef *gpiox = cfg->port;
        uint32_t pin = (uint32_t)cfg->pin_num;

        /* 安全防护：如果是普通输出引脚，初始化前强行将其拉低，规避电调初始浮空导通风险 */
        if (cfg->mode == REG_MODE_OUT) {
            gpiox->BRR = (1u << pin);
        }

        /* A. 配置功能模式 (MODER: 每个引脚占用 2 位) */
        gpiox->MODER &= ~(0x3u << (pin * 2));               
        gpiox->MODER |=  ((uint32_t)cfg->mode << (pin * 2)); 

        /* B. 配置输出类型 (OTYPER: 每个引脚占用 1 位) */
        gpiox->OTYPER &= ~(0x1u << pin);
        gpiox->OTYPER |=  ((uint32_t)cfg->otype << pin);

        /* C. 配置输出速度 (OSPEEDR: 每个引脚占用 2 位) */
        gpiox->OSPEEDR &= ~(0x3u << (pin * 2));
        gpiox->OSPEEDR |=  ((uint32_t)cfg->speed << (pin * 2));

        /* D. 配置内部上下拉 (PUPDR: 每个引脚占用 2 位) */
        gpiox->PUPDR &= ~(0x3u << (pin * 2));
        gpiox->PUPDR |=  ((uint32_t)cfg->pupd << (pin * 2));

        /* E. 配置复用功能寄存器 (AFR: 每个引脚占用 4 位) */
        if (cfg->mode == REG_MODE_AF)
        {
            if (pin < 8u)
            {
                /* Pin 0 ~ Pin 7 属于低位寄存器 AFR[0] */
                gpiox->AFR[0] &= ~(0xFu << (pin * 4));                   
                gpiox->AFR[0] |=  ((uint32_t)cfg->af_sel << (pin * 4));   
            }
            else
            {
                /* Pin 8 ~ Pin 15 属于高位寄存器 AFR[1] */
                uint32_t pin_high_idx = pin - 8u;                        
                gpiox->AFR[1] &= ~(0xFu << (pin_high_idx * 4));          
                gpiox->AFR[1] |=  ((uint32_t)cfg->af_sel << (pin_high_idx * 4));
            }
        }
    }
}

/* 以下是保持高效率原子操作的公共操作接口 */
void BSP_GPIO_WriteHigh(BLDC_Gpio_Idx_t pin)
{
    if (pin >= BLDC_PIN_MAX) return;
    const Gpio_Config_Map_t *map = &s_gpio_config_table[pin];
    map->port->BSRR = (1u << map->pin_num);
}

void BSP_GPIO_WriteLow(BLDC_Gpio_Idx_t pin)
{
    if (pin >= BLDC_PIN_MAX) return;
    const Gpio_Config_Map_t *map = &s_gpio_config_table[pin];
    map->port->BRR = (1u << map->pin_num);
}

void BSP_GPIO_Write(BLDC_Gpio_Idx_t pin, uint8_t value)
{
    if (pin >= BLDC_PIN_MAX) return;
    const Gpio_Config_Map_t *map = &s_gpio_config_table[pin];
    if (value) {
        map->port->BSRR = (1u << map->pin_num);
    } else {
        map->port->BRR = (1u << map->pin_num);
    }
}

void BSP_GPIO_Toggle(BLDC_Gpio_Idx_t pin)
{
    if (pin >= BLDC_PIN_MAX) return;
    const Gpio_Config_Map_t *map = &s_gpio_config_table[pin];
    map->port->ODR ^= (1u << map->pin_num);
}

uint8_t BSP_GPIO_ReadInput(BLDC_Gpio_Idx_t pin)
{
    if (pin >= BLDC_PIN_MAX) return 0;
    const Gpio_Config_Map_t *map = &s_gpio_config_table[pin];
    
    if (map->port->IDR & (1u << map->pin_num)) {
        return 1;
    }
    return 0;
}

