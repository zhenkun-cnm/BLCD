#include "bldc_time.h"
#include "stm32f0xx.h"

volatile uint32_t g_system_ticks = 0;

/**
  * @brief  初始化系统时间基准 (SysTick 毫秒 + TIM6 微秒)
  * @note   适用于 STM32F051, 主频 48MHz
  */
void BSP_Time_Init(void)
{
    /* ========================================================================== */
    /* 1. 初始化 SysTick 控制器（实现 1ms 周期心跳中断）                             */
    /* ========================================================================== */
    SystemCoreClockUpdate();
    SysTick->LOAD = (SystemCoreClock / 1000u) - 1u; /* 48,000,000 / 1000 - 1 */
    SysTick->VAL  = 0u;                             /* 清空当前计数值 */
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |    /* 使用内核时钟源 */
                    SysTick_CTRL_TICKINT_Msk   |    /* 开启异常中断 */
                    SysTick_CTRL_ENABLE_Msk;        /* 正式开闸运行 */

    /* ========================================================================== */
    /* 2. 初始化 TIM6 基本定时器（实现 1us 级物理计数）                              */
    /* ========================================================================== */
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;             /* 开启 TIM6 外设时钟 */

    /* 配置预分频器 (PSC)：
       TIM6 挂在 APB1 总线上（48MHz）。
       分频系数 = PSC + 1，所以设为 48 - 1 = 47。
       此时定时器时钟频率为 48MHz / 48 = 1MHz，即 1 微秒计数器加 1 */
    TIM6->PSC = 48 - 1;

    /* 配置自动重装载寄存器 (ARR)：
       TIM6 是 16 位定时器，最大计数值为 65535。
       直接设为最大值，让它在 0 ~ 65535 之间自由环形递增 */
    TIM6->ARR = 0xFFFFu;

    /* 开启影子寄存器自动预装载使能 (ARPE) */
    TIM6->CR1 |= TIM_CR1_ARPE;

    /* 触发一次软件更新事件，强行将上面的 PSC 和 ARR 写入硬件底层生效 */
    TIM6->EGR |= TIM_EGR_UG;

    /* 正式使能定时器 6 开始计数 (CEN = 1) */
    TIM6->CR1 |= TIM_CR1_CEN;
}

/**
  * @brief  获取当前系统启动后的总毫秒数 (心跳轴)
  */
uint32_t get_tick(void)
{
    return g_system_ticks;
}

/**
  * @brief  毫秒级阻塞延时
  * @param  ms: 需要延时的毫秒数
  */
void delay_ms(uint32_t ms)
{
    uint32_t start = g_system_ticks;
    /* 考虑到了 uint32_t 溢出回绕问题，减法依然满足时间差计算 */
    while ((g_system_ticks - start) < ms);
}

/**
  * @brief  微秒级精准阻塞延时
  * @param  us: 需要延时的微秒数 (最大不要超过 65535)
  * @note   直接提取硬件寄存器 TIM6->CNT 差值，免中断压栈开销，效率极高
  */
void delay_us(uint32_t us)
{
    /* 捕获当前的硬件计数值 */
    uint16_t start_val = (uint16_t)(TIM6->CNT);
    
    /* 核心硬件防越界算法：
      因为 TIM6->CNT 是 16 位无符号数（0~65535 循环滚动）。
      利用 C 语言强制类型转换 (uint16_t)(当前值 - 初始值)，
      即便在延时期间 CNT 刚好跨越 65535 翻转回 0，做减法后的差值依然完全准确。
    */
    while ((uint16_t)((uint16_t)TIM6->CNT - start_val) < us);
}
