#include "bldc_clock.h"
#include "bldc_time.h"
#include "stm32f0xx.h"

void SystemClock_Config(void)
{
    // Flash: 1 wait state + prefetch buffer enable (required for 48 MHz)
    FLASH->ACR |= (0x1u << FLASH_ACR_LATENCY_Pos)
                | (1u << FLASH_ACR_PRFTBE_Pos);

    // PLL config: HSI/2 (4 MHz) × 12 = 48 MHz
    // STM32F0xx PLL input must be within 2-4 MHz
    RCC->CFGR &= ~(RCC_CFGR_PLLSRC_Msk | RCC_CFGR_PLLMUL_Msk);
    RCC->CFGR |= RCC_CFGR_PLLSRC_HSI_DIV2                  // HSI/2 = 4 MHz into PLL
               | (0xAu << RCC_CFGR_PLLMUL_Pos);           // PLLMUL = 12, output = 48 MHz

    // Enable PLL and wait for lock
    RCC->CR |= (1u << RCC_CR_PLLON_Pos);
    while (!(RCC->CR & RCC_CR_PLLRDY_Msk)) {}

    // Switch system clock to PLL
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW_Msk) | RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_PLL) {}

    // Update global
    SystemCoreClock = 48000000;
}

void BSP_SysTick_Init_1ms(void)
{
    /* 1. 计算重装载值：48MHz 下 1ms 需要 48000 个周期，寄存器从 LOAD 减到 0，所以填入 48000 - 1 */
    SysTick->LOAD = (SystemCoreClock / 1000u) - 1u;

    /* 2. 清空当前计数值，向 VAL 写入任何值都会清零整个计数器，并清除 CTRL 的 COUNTFLAG 标志 */
    SysTick->VAL = 0u;

    /* 3. 配置控制寄存器 CTRL：
       - Bit 2 (CLKSOURCE) = 1: 使用核心时钟处理器时钟 (AHB)
       - Bit 1 (TICKINT)   = 1: 计数器减到 0 时触发 SysTick 异常中断
       - Bit 0 (ENABLE)    = 1: 开启定时器
    */
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | 
                    SysTick_CTRL_TICKINT_Msk   | 
                    SysTick_CTRL_ENABLE_Msk;
                    
    /* 4. 配置中断优先级 (可选)
       SysTick 属于系统异常，其优先级由 SHP[1] 寄存器控制。
       通过以下 CMSIS 函数可以将 SysTick 优先级设为最高（0）或自定义值
    */
    NVIC_SetPriority(SysTick_IRQn, 3); /* 视项目需求调整，电调中通常设为中等或较低，防止阻塞电机中断 */
}

/**
  * @brief  获取当前系统心跳计数值
  * @return 系统上电以来的毫秒数 (uint32_t, 约 49.7 天回绕)
  */
uint32_t BSP_GetTick(void)
{
    return g_system_ticks;
}
