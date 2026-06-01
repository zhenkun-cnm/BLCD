#include "stm32f0xx.h"
#include "bldc_comp.h"
#include "bldc_debug.h"

void BSP_COMP2_Init(void)
{
// ---------------------------------------------------------
    // 1. 开启外设时钟
    // ---------------------------------------------------------
    // 开启 GPIOA 时钟 (PA2 和 PA3)
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN; 
    // 开启 SYSCFG 和 COMP 时钟 (使用官方推荐的严谨宏名称)
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN; 
    // ---------------------------------------------------------
    // 2. 配置 GPIOA 引脚
    // ---------------------------------------------------------
// ---------------------------------------------------------
    // 2. 配置 GPIOA 引脚
    // ---------------------------------------------------------
    // 将 PA2(A相), PA3(中性点), PA4(B相), PA5(C相) 均配置为纯模拟模式
    GPIOA->MODER |= (GPIO_MODER_MODER2 | GPIO_MODER_MODER3 | GPIO_MODER_MODER4 | GPIO_MODER_MODER5);
    // 禁用上下拉电阻
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR2 | GPIO_PUPDR_PUPDR3 | GPIO_PUPDR_PUPDR4 | GPIO_PUPDR_PUPDR5);

    // ---------------------------------------------------------
    // 3. 配置 COMP2 控制状态寄存器 (COMP_CSR)
    // ---------------------------------------------------------
    // 先清零 COMP2 相关配置区，防止残留数据干扰
    COMP->CSR &= ~(COMP_CSR_COMP2INSEL  | 
                   COMP_CSR_COMP2OUTSEL | 
                   COMP_CSR_COMP2POL    | 
                   COMP_CSR_COMP2HYST   | 
                   COMP_CSR_COMP2MODE   | 
                   COMP_CSR_WNDWEN);

    // 【修正处】选择反相输入端 (Vin-) 为 PA2
    // 在F051中，110 = COMP2_INM6 = PA2
    COMP->CSR &= ~COMP_CSR_COMP2INSEL; 

    COMP->CSR |= (COMP_CSR_COMP2INSEL_2 | COMP_CSR_COMP2INSEL_1); //PA2
	// 2. 仅置位第 2 位 (二进制 100)
	//COMP->CSR |= COMP_CSR_COMP2INSEL_2; //PA4
	//COMP->CSR |= (COMP_CSR_COMP2INSEL_2 | COMP_CSR_COMP2INSEL_0);//PA5
	
	
    //COMP->CSR |= COMP_CSR_COMP2HYST_0;  // 写入 01，配置为低迟滞 (Low Hysteresis)
    
// ---------------------------------------------------------
    // 4. 配置 EXTI (外部中断控制器) - 【关键修改：保持彻底静默】
    // ---------------------------------------------------------
    EXTI->IMR  &= ~EXTI_IMR_MR22;  // 【改】屏蔽中断申请，禁止打扰 CPU
    EXTI->RTSR &= ~EXTI_RTSR_TR22; // 【改】关闭上升沿触发检测
    EXTI->FTSR &= ~EXTI_FTSR_TR22; // 【改】关闭下降沿触发检测
    EXTI->PR = EXTI_PR_PR22;       // 清除配置过程中可能产生的虚假挂起标志

    // ---------------------------------------------------------
    // 5. 配置 NVIC
    // ---------------------------------------------------------
    // 即使这里使能了 NVIC，只要 EXTI->IMR 关着，比较器也无法触发中断
    NVIC_SetPriority(ADC1_COMP_IRQn, 0); 
    NVIC_EnableIRQ(ADC1_COMP_IRQn);      

    // ---------------------------------------------------------
    // 6. 使能比较器 - 【关键修改：删掉原本的开启代码】
    // ---------------------------------------------------------
    // 比较器本体 (COMP2EN) 在初始化阶段不应该打开！
    // 一方面是为了省电，另一方面是为了防止比较器瞎跳变默默置位 EXTI_PR。
    // 这里将其保持关闭，等需要开环启动时，由 BSP_COMP2_Start() 去打开。
    COMP->CSR &= ~COMP_CSR_COMP2EN;
}

// 外部声明您编写的纯寄存器版分时复用换相回调函数
extern void BLDC_COMP2_TriggerCallback(void);

void ADC1_COMP_IRQHandler(void)
{
    // 【修改处】：双重校验！
    // 不仅要确认发生了跳变 (PR == 1)
    // 还必须确认软件当前确实开启了这个中断 (IMR == 1)
    if( ((EXTI->PR & EXTI_PR_PR22) != 0) && ((EXTI->IMR & EXTI_IMR_MR22) != 0) ) 
    {   
        // 只有 IMR 和 PR 同时为 1，才去执行闭环换相
        BLDC_COMP2_TriggerCallback();
        // 必须第一时间清除，防止后续被换相尖峰干扰
        EXTI->PR = EXTI_PR_PR22;
    }
    // 如果你有处理 ADC 中断的逻辑，可以接着写在这里
    // 比如：if (ADC1->ISR & ADC_ISR_EOC) { ... }
}

void BSP_COMP2_Start_IT(void)
{
    /* =================================================================================
     * 第一步：开启比较器本体 (先通电)
     * ================================================================================= */
    COMP->CSR |= COMP_CSR_COMP2EN;

    /* =================================================================================
     * 第二步：等待比较器状态稳定 (消除 t_START 建立时间内的毛刺)
     * ================================================================================= */
    // 简单的软件延时，消耗几个 CPU 周期即可（具体视系统主频而定，几微秒足矣）
    for(volatile uint8_t i = 0; i < 50; i++) {
        __NOP(); 
    }

    /* =================================================================================
     * 第三步：恢复边沿检测功能
     * ================================================================================= */
    EXTI->RTSR |= EXTI_RTSR_TR22;
    //EXTI->FTSR |= EXTI_FTSR_TR22;

    /* =================================================================================
     * 第四步：清除由于上电建立时间产生的虚假挂起标志位 (必须在开启 IMR 前清理)
     * ================================================================================= */
    EXTI->PR = EXTI_PR_PR22;       

    /* =================================================================================
     * 第五步：开启 EXTI 外部中断线 (最后接通向 CPU 的报警线)
     * ================================================================================= */
    EXTI->IMR |= EXTI_IMR_MR22;   
}
/**
  * @brief  关闭 COMP2 并关闭其中断 (等效于 HAL_COMP_Stop_IT)
  */
void BSP_COMP2_Stop_IT(void)
{
    /* =================================================================================
     * 第一步：关闭 EXTI 中断线
     * 作用：拔掉比较器连接到 CPU 中断控制器的“报警线”，即使比较器电压翻转，也不会触发中断。
     * ================================================================================= */
    
    // 1. 屏蔽 EXTI 线 22 的中断
    // IMR (Interrupt Mask Register)，清零第 22 位。0 表示屏蔽(Block)，1 表示放行。
    EXTI->IMR &= ~EXTI_IMR_MR22;   
    
    // 2. (可选/推荐) 同时清除上升沿和下降沿触发配置，还原到最干净的状态
    EXTI->RTSR &= ~EXTI_RTSR_TR22; // 关闭上升沿触发
    EXTI->FTSR &= ~EXTI_FTSR_TR22; // 关闭下降沿触发
    
    // 3. 清除可能已经挂起（Pending）的中断标志位，防止下次重新打开中断时直接误触发
    // 注意：EXTI_PR 是“写 1 清零”的寄存器
    EXTI->PR = EXTI_PR_PR22;       

    /* =================================================================================
     * 第二步：关闭比较器本体
     * 作用：真正断开比较器的电源，降低芯片功耗。
     * ================================================================================= */
    
    // 清除 COMP2EN 位 (写 0)，彻底关闭比较器 2
    COMP->CSR &= ~COMP_CSR_COMP2EN;
}

void BSP_COMP2_Start(void)
{
    EXTI->IMR &= ~EXTI_IMR_MR22;   // 屏蔽中断
    EXTI->RTSR &= ~EXTI_RTSR_TR22; // 关上升沿检测
    EXTI->FTSR &= ~EXTI_FTSR_TR22; // 关下降沿检测
    EXTI->PR = EXTI_PR_PR22;       // 清空历史挂起标志
    COMP->CSR |= COMP_CSR_COMP2EN; // 开启比较器
}

/**
  * @brief  仅关闭 COMP2 (等效于 HAL_COMP_Stop)
  * @note   此函数只给比较器断电，不会清除 EXTI (外部中断) 的配置。
  */
void BSP_COMP2_Stop(void)
{
    /* =================================================================================
     * 关闭比较器电源
     * ================================================================================= */

    // 将 COMP_CSR 寄存器中的 COMP2EN (使能位) 清零 (写 0)
    // 这会直接切断比较器的内部电源，比较器停止工作，输出端会被强制拉低或保持无效状态。
    COMP->CSR &= ~COMP_CSR_COMP2EN;
}

/**
  * @brief  COMP2 通道独立诊断 —— 轮流切换 PA2/PA4/PA5，跨时间窗口统计翻转次数
  * @note   【重要】本函数为阻塞式独立诊断，会暂停调用者的执行流约 500ms。
  *         调用前确保电机正在转动（开环换相或外力拖动），且 COMP2 已使能。
  *         每个通道采样约 165ms (300 点 × 550us)，统计 COMP2OUT 的电平翻转次数。
  *         正常工作的通道应能看到多次翻转（BEMF 穿越中性点）；
  *         异常通道翻转次数为 0（引脚始终高于或始终低于中性点电压）。
  */
void BSP_COMP2_Channel_Diagnostic(void)
{
    uint32_t saved_insel = COMP->CSR & COMP_CSR_COMP2INSEL;
    uint32_t saved_imr   = EXTI->IMR  & EXTI_IMR_MR22;

    /* 屏蔽中断，纯轮询模式 */
    EXTI->IMR &= ~EXTI_IMR_MR22;
    COMP->CSR |= COMP_CSR_COMP2EN;

    typedef struct {
        const char *name;
        uint32_t    sel;
    } ch_t;

    const ch_t channels[] = {
        {"PA2", COMP_CSR_COMP2INSEL_2 | COMP_CSR_COMP2INSEL_1},  /* 110 = PA2 */
        {"PA4", COMP_CSR_COMP2INSEL_2},                            /* 100 = PA4 */
        {"PA5", COMP_CSR_COMP2INSEL_2 | COMP_CSR_COMP2INSEL_0},   /* 101 = PA5 */
    };

    LOG_DEBUG("=== COMP2 Channel Edge Diagnostic ===\r\n");
    LOG_DEBUG("    (300 samples/ch, 550us spacing, ~165ms/ch)\r\n");

    for (uint8_t ci = 0; ci < 3; ci++)
    {
        COMP->CSR = (COMP->CSR & ~COMP_CSR_COMP2INSEL) | channels[ci].sel;
        for (volatile uint16_t d = 0; d < 200; d++) { __NOP(); }

        uint8_t  prev  = (COMP->CSR & COMP_CSR_COMP2OUT) ? 1 : 0;
        uint8_t  first = prev;
        uint16_t edges = 0;

        for (uint16_t s = 0; s < 300; s++)
        {
            delay_us(550);   /* 300×550us ≈ 165ms，跨越多个换相周期 */

            uint8_t cur = (COMP->CSR & COMP_CSR_COMP2OUT) ? 1 : 0;
            if (cur != prev) {
                edges++;
            }
            prev = cur;
        }

        LOG_DEBUG("  %s: edges=%u first=%u last=%u\r\n",
                  channels[ci].name, edges, first, prev);
    }

    /* 恢复现场 */
    COMP->CSR = (COMP->CSR & ~COMP_CSR_COMP2INSEL) | saved_insel;
    EXTI->PR  = EXTI_PR_PR22;
    EXTI->IMR |= saved_imr;

    LOG_DEBUG("=== Diagnostic End ===\r\n");
}


