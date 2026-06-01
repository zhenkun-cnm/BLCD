#include "bldc_pwm.h"


void BSP_PWM_Init(void)
{
    /* ========================================================================== */
    /* 1. 开启时钟：GPIOA、GPIOB、TIM2、TIM3                                       */
    /* ========================================================================== */
    RCC->AHBENR  |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN;
    
    /* === 核心修正 A：强行解除 PA15 的调试引脚占用，释放给普通复用外设 === */
    /* 开启复用系统配置时钟 */
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    /* 对于 STM32F0，确保通过关闭相关调试映射或正常重刷 MODER 来夺回控制权 */

    /* ========================================================================== */
    /* 2. 配置 GPIO 引脚复用映射 (你原本的代码保留)                                 */
    /* ========================================================================== */
    // ------------------ (A) 配置 GPIOA: PA15 ------------------
    GPIOA->MODER &= ~GPIO_MODER_MODER15_Msk;
    GPIOA->MODER |= (0x2u << GPIO_MODER_MODER15_Pos);
    GPIOA->OSPEEDR |= (0x3u << GPIO_OSPEEDR_OSPEEDR15_Pos);
    GPIOA->AFR[1] &= ~GPIO_AFRH_AFSEL15_Msk;
    GPIOA->AFR[1] |= (0x2u << GPIO_AFRH_AFSEL15_Pos);

    // ------------------ (B) 配置 GPIOB: PB3, PB4 ------------------
    GPIOB->MODER &= ~(GPIO_MODER_MODER3_Msk | GPIO_MODER_MODER4_Msk);
    GPIOB->MODER |= (0x2u << GPIO_MODER_MODER3_Pos) | (0x2u << GPIO_MODER_MODER4_Pos);
    GPIOB->OSPEEDR |= (0x3u << GPIO_OSPEEDR_OSPEEDR3_Pos) | (0x3u << GPIO_OSPEEDR_OSPEEDR4_Pos);
    
    GPIOB->AFR[0] &= ~GPIO_AFRL_AFSEL3_Msk;
    GPIOB->AFR[0] |= (0x2u << GPIO_AFRL_AFSEL3_Pos);
    GPIOB->AFR[0] &= ~GPIO_AFRL_AFSEL4_Msk;
    GPIOB->AFR[0] |= (0x1u << GPIO_AFRL_AFSEL4_Pos);

    /* ========================================================================== */
    /* 3. 配置 TIM2 & TIM3 寄存器 (你原本的代码保留)                                */
    /* ========================================================================== */
    TIM2->PSC = 2; TIM2->ARR = 991;
    TIM2->CCMR1 &= ~(TIM_CCMR1_OC1M_Msk | TIM_CCMR1_OC2M_Msk);
    TIM2->CCMR1 |= (0x6u << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;
    TIM2->CCMR1 |= (0x6u << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
    TIM2->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC2P);
    TIM2->CR1 |= TIM_CR1_ARPE;
    TIM2->CCR1 = 0; TIM2->CCR2 = 0;

    TIM3->PSC = 2; TIM3->ARR = 991;
    TIM3->CCMR1 &= ~TIM_CCMR1_OC1M_Msk;
    TIM3->CCMR1 |= (0x6u << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;
    TIM3->CCER &= ~TIM_CCER_CC1P;
    TIM3->CR1 |= TIM_CR1_ARPE;
    TIM3->CCR1 = 0;

    /* 执行一次影子寄存器更新 */
    TIM2->EGR |= TIM_EGR_UG;
    TIM3->EGR |= TIM_EGR_UG;

    /* === 核心修正 B：物理全桥开闸！强行向 BDTR 写入 MOE 主输出使能 === */
    /* 只有这行写进去，TIM2 和 TIM3 的 PWM 发生器产生的波形才能物理穿透到引脚上 */
    TIM2->BDTR |= TIM_BDTR_MOE;
    TIM3->BDTR |= TIM_BDTR_MOE;
}
/**
  * @brief  设置指定通道的 PWM 占空比值 (脉宽)
  * @param  ch: 目标物理通道
  * @param  duty: 占空比数值 (有效范围: 0 ~ 991)
  */
void BSP_PWM_SetDuty(TIM_TypeDef *TIMx, PWM_Channel_t ch, uint16_t duty)
{
    if (duty > TIMx->ARR) duty = TIMx->ARR; /* 越界硬件限幅保护 */
    switch (ch)
    {
        case PWM_Channel1:
            TIMx->CCR1 = duty;
            break;
        case PWM_Channel2:
            TIMx->CCR2 = duty;
            break;
        case PWM_Channel3:
            TIMx->CCR3 = duty;
            break;
        case PWM_Channel4:
            TIMx->CCR4 = duty;
            break;
        default: break;
    }
}

/**
  * @brief  开启对应通道的 PWM 波形输出
  */
void BSP_PWM_Start(TIM_TypeDef *TIMx,PWM_Channel_t ch)
{
    switch (ch)
    {
        case PWM_Channel1:
            TIMx->CCER |= TIM_CCER_CC1E;
            TIMx->CR1  |= TIM_CR1_CEN;
            break;
        case PWM_Channel2:
            TIMx->CCER |= TIM_CCER_CC2E;
            TIMx->CR1  |= TIM_CR1_CEN;
            break;
        case PWM_Channel3:
            TIMx->CCER |= TIM_CCER_CC3E;
            TIMx->CR1  |= TIM_CR1_CEN;
            break;
        case PWM_Channel4:
            TIMx->CCER |= TIM_CCER_CC4E;
            TIMx->CR1  |= TIM_CR1_CEN;
            break;
        default: break;
    }
}

/**
  * @brief  关闭对应通道的 PWM 波形输出
  */
void BSP_PWM_Stop(TIM_TypeDef *TIMx,PWM_Channel_t ch)
{
    switch (ch)
    {
        case PWM_Channel1:
            TIMx->CCER &= ~TIM_CCER_CC1E;
            break;
        case PWM_Channel2:
            TIMx->CCER &= ~TIM_CCER_CC2E;
            break;
        case PWM_Channel3:
            TIMx->CCER &= ~TIM_CCER_CC3E;
            break;
        case PWM_Channel4:
            TIMx->CCER &= ~TIM_CCER_CC4E;
            break;
        default: break;
    }
}

