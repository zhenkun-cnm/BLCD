#include "bldc_led.h"
#include "stm32f0xx.h"

/* WS2812B 常量定义 */
#define LED_NUM             1       /* 电调灯数量：1颗 */
#define BITS_PER_LED        24      /* 每颗灯 24 位 (G-R-B) */
#define RESET_SLOTS         40      /* 复位信号所需的低电平点空位(>50us) */
#define DMA_BUF_SIZE        (LED_NUM * BITS_PER_LED + RESET_SLOTS)

/* TIM16 在 48MHz 下的 PWM 脉宽定义 (ARR = 59, 周期 = 1.25us) */
#define PWM_CODE_1          42      /* 约 0.87us 高电平 -> 码 1 */
#define PWM_CODE_0          18      /* 约 0.37us 高电平 -> 码 0 */

/* 24位颜色结构体：WS2812B 内部顺序为 G-R-B */
typedef struct {
    uint8_t g;
    uint8_t r;
    uint8_t b;
} RGB_Color_t;

/* 颜色常量定义 (G, R, B) */
const RGB_Color_t COLOR_NONE    = {0,   0,   0};
const RGB_Color_t COLOR_GREEN   = {255, 0,   0};
const RGB_Color_t COLOR_YELLOW  = {150, 255, 0};   /* 黄色 */
const RGB_Color_t COLOR_RED     = {0,   255, 0};
const RGB_Color_t COLOR_ORANGE  = {60,  255, 0};   /* 橙色 */
const RGB_Color_t COLOR_BLUE    = {0,   0,   255};
const RGB_Color_t COLOR_WHITE   = {255, 255, 255};
const RGB_Color_t COLOR_PURPLE  = {0,   160, 255};  /* 紫色 */
const RGB_Color_t COLOR_CYAN    = {180, 0,   255};  /* 浅蓝 */

/* 全局变量 */
static uint32_t s_dma_buffer[DMA_BUF_SIZE];
static volatile LED_Status_t s_current_status = LED_STATUS_INIT;
static volatile uint8_t  s_error_code = 0;          /* 存储闪烁次数代码 */
static volatile uint32_t s_led_ticks = 0;           /* 毫秒计数器 */
static volatile uint8_t  s_refresh_needed = 1;

/* 内部基础驱动：把RGB数据填入DMA缓冲区 */
static void LED_SetRGB(RGB_Color_t color)
{
    uint32_t i;
    uint32_t temp = ((uint32_t)color.g << 16) | ((uint32_t)color.r << 8) | color.b;
    
    // 解析24位数据
    for (i = 0; i < 24; i++) {
        if (temp & (1u << (23 - i)))  {
            s_dma_buffer[i] = PWM_CODE_1;
        } else {
            s_dma_buffer[i] = PWM_CODE_0;
        }
    }
    // 后面全部填充0，用于产生 Reset 信号
    for (i = 24; i < DMA_BUF_SIZE; i++) {
        s_dma_buffer[i] = 0;
    }
    s_refresh_needed = 1;
}

/* 硬件底层初始化：PB8 -> TIM16_CH1 -> DMA1_CH3 */
void BSP_LED_Init(void)
{
    /* 1. 开启 GPIOB、TIM16 和 DMA1 时钟 */
    RCC->AHBENR  |= RCC_AHBENR_GPIOBEN;
    RCC->AHBENR  |= RCC_AHBENR_DMA1EN;
    RCC->APB2ENR |= RCC_APB2ENR_TIM16EN;

    /* 2. 配置 PB8 为复用模式 (AF2) */
    GPIOB->MODER   &= ~GPIO_MODER_MODER8_Msk;
    GPIOB->MODER   |= (0x2u << GPIO_MODER_MODER8_Pos);      /* 复用模式 */
    GPIOB->OSPEEDR |= (0x3u << GPIO_OSPEEDR_OSPEEDR8_Pos);  /* 高速 */
    
    GPIOB->AFR[1]  &= ~GPIO_AFRH_AFSEL8_Msk;
    GPIOB->AFR[1]  |= (0x2u << GPIO_AFRH_AFSEL8_Pos);       /* AF2 = TIM16_CH1 */

    /* 3. 配置 TIM16 时基环境：产生 800kHz 载波 */
    TIM16->PSC = 0;                                         /* 不分频 (48MHz) */
    TIM16->ARR = 60 - 1;                                    /* 48MHz / 60 = 800kHz (1.25us) */

    /* 4. 配置 TIM16_CH1 为 PWM1 模式 */
    TIM16->CCMR1 &= ~TIM_CCMR1_OC1M_Msk;
    TIM16->CCMR1 |= (0x6u << TIM_CCMR1_OC1M_Pos);           /* PWM mode 1 */
    TIM16->CCMR1 |= TIM_CCMR1_OC1PE;                        /* 开启预装载 */

    TIM16->CCER  |= TIM_CCER_CC1E;                          /* 使能通道1输出 */
    TIM16->BDTR  |= TIM_BDTR_MOE;                           /* 主输出使能(高级/专用定时器必须) */

    /* 5. 配置 DMA1 通道3 (TIM16_CH1 或 TIM16_UP 触发) */
    DMA1_Channel3->CPAR  = (uint32_t)&(TIM16->CCR1);        /* 目标外设：TIM16 比较寄存器 */
    DMA1_Channel3->CMAR  = (uint32_t)s_dma_buffer;          /* 源地址：内存缓冲区 */
    DMA1_Channel3->CNDTR = DMA_BUF_SIZE;
    DMA1_Channel3->CCR   = DMA_CCR_DIR                      /* 内存至外设 */
                         | DMA_CCR_MINC                     /* 内存自增 */
                         | (0x1u << DMA_CCR_MSIZE_Pos)      /* 内存数据宽度：32位 */
                         | (0x1u << DMA_CCR_PSIZE_Pos);     /* 外设数据宽度：32位 */
    
    /* 6. 开启定时器的 DMA 请求（更新事件或CC1事件均可，这里用 CC1 触发） */
    TIM16->DIER &= ~TIM_DIER_CC1DE; 
	TIM16->DIER |= TIM_DIER_UDE;     // 改为更新事件触发 DMA

    /* 7. 启动定时器 */
    TIM16->CR1 |= TIM_CR1_CEN;
    
    /* 默认初始化状态 */
    LED_SetRGB(COLOR_YELLOW);
}

/**
 * @brief LED设置状态函数
 * 
 * @param 
 * @return 
 */
void BSP_LED_SetStatus(LED_Status_t status, uint8_t code)
{
    if (s_current_status != status || s_error_code != code) {
        s_current_status = status;
        s_error_code = code;
        s_led_ticks = 0; /* 复位时间轴，使状态切换时立即响应 */
    }
}

/* 1ms 时钟心跳 */
void BSP_LED_Tick_1ms(void)
{
    s_led_ticks++;
}

/* 触发 DMA 传输，外发波形 */
static void LED_SubmitDMA(void)
{
    if (DMA1_Channel3->CCR & DMA_CCR_EN) return;
    
    DMA1->IFCR = DMA_IFCR_CTCIF3;    // 清除 DMA 通道 3 传输完成标志
    TIM16->SR  = ~TIM_SR_UIF;        // 清除定时器更新中断标志
    
    DMA1_Channel3->CCR   &= ~DMA_CCR_EN;
    DMA1_Channel3->CNDTR  = DMA_BUF_SIZE;
    DMA1_Channel3->CCR   |= DMA_CCR_EN;
}

/* 状态机核心业务逻辑 */
void BSP_LED_Process(void)
{
    uint32_t period_ticks = s_led_ticks;
    static uint32_t last_exec_time = 0;

    // 防止过于频繁计算，每 5ms 执行一次状态解析
    if (period_ticks - last_exec_time < 5) return;
    last_exec_time = period_ticks;

    switch (s_current_status) 
    {
        case LED_STATUS_INIT:       /* 黄色常亮 */
            LED_SetRGB(COLOR_YELLOW);
            break;

        case LED_STATUS_READY:      /* 绿色常亮 */
            LED_SetRGB(COLOR_GREEN);
            break;

        case LED_STATUS_SET_ADDR:   /* 紫色常亮 */
            LED_SetRGB(COLOR_PURPLE);
            break;

        /* ---- 闪烁状态分类处理 (基于 500ms 翻转周期的复合闪烁) ---- */
        case LED_STATUS_ERR_CURRENT: /* 橙色闪烁 (均匀按秒闪烁) */
            if ((period_ticks / 300) % 2 == 0) {
                LED_SetRGB(COLOR_ORANGE);
            } else {
                LED_SetRGB(COLOR_NONE);
            }
            break;

        case LED_STATUS_ERR_OFFLINE: /* 蓝色闪烁 */
            if ((period_ticks / 300) % 2 == 0) {
                LED_SetRGB(COLOR_BLUE);
            } else {
                LED_SetRGB(COLOR_NONE);
            }
            break;

        case LED_STATUS_ERR_START_FAIL: /* 白色闪烁 */
            if ((period_ticks / 250) % 2 == 0) {
                LED_SetRGB(COLOR_WHITE);
            } else {
                LED_SetRGB(COLOR_NONE);
            }
            break;

        case LED_STATUS_ERR_VOLTAGE: /* 浅蓝闪烁 */
            if ((period_ticks / 300) % 2 == 0) {
                LED_SetRGB(COLOR_CYAN);
            } else {
                LED_SetRGB(COLOR_NONE);
            }
            break;

        case LED_STATUS_ALARM_ALT:  /* 绿蓝交替闪烁 */
            if ((period_ticks / 400) % 2 == 0) {
                LED_SetRGB(COLOR_GREEN);
            } else {
                LED_SetRGB(COLOR_BLUE);
            }
            break;

        /* ---- 带有明确‘闪烁次数代表代码’的特殊状态 ---- */
        /* 设计逻辑：大周期 2.5秒。前段由 code 决定闪烁几次，后段全灭留空作为间隔 */
        case LED_STATUS_ERR_MOS:    /* 红色闪烁 (次数=错误码) */
        {
            uint32_t sub_time = period_ticks % 2500; /* 2.5秒一轮循环 */
            uint32_t flash_index = sub_time / 200;   /* 每次亮灭占 200ms (亮100ms/灭100ms) */
            
            if (flash_index < (uint32_t)(s_error_code * 2)) {
                if (flash_index % 2 == 0) {
                    LED_SetRGB(COLOR_RED);
                } else {
                    LED_SetRGB(COLOR_NONE);
                }
            } else {
                LED_SetRGB(COLOR_NONE); /* 超过次数，在周期剩余时间内保持熄灭 */
            }
            break;
        }

        case LED_STATUS_ADDR_DONE:  /* 紫色闪烁 (次数=电调地址) */
        {
            uint32_t sub_time = period_ticks % 3000; /* 3秒一轮大循环 */
            uint32_t flash_index = sub_time / 250;   /* 每250ms动作一次 (亮125ms/灭125ms) */
            
            if (flash_index < (uint32_t)(s_error_code * 2)) {
                if (flash_index % 2 == 0) {
                    LED_SetRGB(COLOR_PURPLE);
                } else {
                    LED_SetRGB(COLOR_NONE);
                }
            } else {
                LED_SetRGB(COLOR_NONE);
            }
            break;
        }

        default:
            LED_SetRGB(COLOR_NONE);
            break;
    }


    /* 只要数据流被更新或重绘，触发 DMA 输出 */
    if (s_refresh_needed) {
        // 由于 STM32F0 的 DMA 非循环模式传完即止，需先清除通道传输完成标志
        DMA1->IFCR = DMA_IFCR_CTCIF3; 
        LED_SubmitDMA();
        s_refresh_needed = 0;
    }
}
