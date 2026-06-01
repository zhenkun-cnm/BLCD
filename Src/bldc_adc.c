#include "bldc_adc.h"
#include "stm32f0xx.h"

/* 显式定义外发原始硬件缓冲区与传输完成标志位 */
volatile uint16_t g_adc_raw_value[ADC_CONVERT_NUM] = {0};
volatile uint8_t  g_adc_new_data_flag = 0;
volatile BLDC_Metrics_t  g_adc_metrics;

/**
  * @brief  纯寄存器级 ADC 硬件自校准程序
  * @note   必须在 ADC 处于【关闭状态 (ADEN=0)】时调用
  * @return 0: 校准成功; -1: 校准超时失败
  */
int32_t BSP_ADC_Calibration_Start(void)
{
    volatile uint32_t timeout = 0u;

    /* 1. 确保 ADC 处于关闭状态，否则触发校准会导致外设锁死 */
    if (ADC1->CR & ADC_CR_ADEN) 
    {
        ADC1->CR |= ADC_CR_ADDIS;
        timeout = 0xFFFFu;
        while (ADC1->CR & ADC_CR_ADEN) 
        {
            if (--timeout == 0u) return -1; /* 硬件关闭超时 */
        }
    }

    /* 2. 隔离数字总线噪声：强切至 ADC 专用内部异步时钟模式 (HSI14) */
    ADC1->CFGR2 &= ~ADC_CFGR2_CKMODE_Msk; 

    /* 3. 正式启动硬件自校准电路 (ADCAL = 1) */
    ADC1->CR |= ADC_CR_ADCAL;

    /* 4. 阻塞等待硬件内部 SAR 电容阵列校准算法执行结束 */
    timeout = 0xFFFFu;
    while (ADC1->CR & ADC_CR_ADCAL) 
    {
        if (--timeout == 0u) return -1; /* 自校准超时失败 */
    }
    
    return 0; /* 校准圆满成功 */
}

/**
  * @brief  ADC1 + DMA1_Channel1 寄存器级全自动初始化
  * @note   配置为：PA0, PA1 模拟输入，开启连续前向扫描与循环 DMA 传输完成中断
  */
void BSP_ADC_Init(void)
{
    /* ========================================================================== */
    /* 1. 开启外设时钟：GPIOA、ADC1、DMA1                                          */
    /* ========================================================================== */
    RCC->AHBENR  |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_DMA1EN;
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    /* ========================================================================== */
    /* 2. 配置引脚：PA0 (ADC_IN0), PA1 (ADC_IN1) 为模拟输入模式                      */
    /* ========================================================================== */
    /* 模拟模式对应的配置码为 0x3 (11) */
    GPIOA->MODER |= (0x3u << GPIO_MODER_MODER0_Pos) | (0x3u << GPIO_MODER_MODER1_Pos);
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR0_Msk | GPIO_PUPDR_PUPDR1_Msk); /* 禁用上下拉 */

    /* ========================================================================== */
    /* 3. 配置 DMA1_Channel1 (ADC1 固定绑定通道) 并开闸【传输完成中断】              */
    /* ========================================================================== */
    DMA1_Channel1->CCR &= ~DMA_CCR_EN; /* 配置前先确保关闭通道 */
    while(DMA1_Channel1->CCR & DMA_CCR_EN);

    /* 刮骨疗毒：免宏定义冲突写法，直接清空通道 1 的所有历史中断状态位 */
    DMA1->IFCR = 0x0000000Fu; 

    DMA1_Channel1->CPAR  = (uint32_t)&(ADC1->DR);               /* 源地址：ADC 原始数据寄存器 */
    DMA1_Channel1->CMAR  = (uint32_t)g_adc_raw_value;           /* 目的地址：SRAM 本地数组 */
    DMA1_Channel1->CNDTR = ADC_CONVERT_NUM;                     /* 每次循环搬运的数量 */

    /* CCR 寄存器硬核配置：
       - MINC : 内存地址递增（搬完 CH0 自动移到下一位存 CH1）
       - CIRC : 循环传输模式（配合 ADC 实现后台无限不间断自动刷新）
       - PSIZE/MSIZE = 01 : 外设与内存数据宽度皆为 16位 (Half-Word)
       - TCIE : 开启传输完成中断，当一帧内所有通道全搬完时向 CPU 发出中断请求
    */
    DMA1_Channel1->CCR = DMA_CCR_MINC 
                       | DMA_CCR_CIRC 
                       | DMA_CCR_TCIE  
                       | (0x1u << DMA_CCR_PSIZE_Pos) 
                       | (0x1u << DMA_CCR_MSIZE_Pos)
                       | (0x1u << DMA_CCR_PL_Pos);            /* 中等优先级 */

    DMA1_Channel1->CCR |= DMA_CCR_EN;                         /* 开闸使能 DMA 通道 1 */

    /* 配置并使能 DMA1_Channel1 的 NVIC 中断通道 */
    NVIC_SetPriority(DMA1_Channel1_IRQn, 1);                  /* 优先级设为 1，仅次于电调核心定时器 */
    NVIC_EnableIRQ(DMA1_Channel1_IRQn);

    /* ========================================================================== */
    /* 4. 配置 ADC1 核心控制寄存器                                                 */
    /* ========================================================================== */
    if (ADC1->CR & ADC_CR_ADEN) {
        ADC1->CR |= ADC_CR_ADDIS;
        while(ADC1->CR & ADC_CR_ADEN);
    }

    /* CONT = 1 (连续转换模式), OVRMOD = 1 (覆盖原有数据，防止总线竞争导致外设死锁) */
    ADC1->CFGR1 = ADC_CFGR1_CONT | ADC_CFGR1_OVRMOD;
    
    /* DMAEN = 1 (使能 DMA 传输), DMACFG = 1 (循环 DMA 模式) */
    ADC1->CFGR1 |= ADC_CFGR1_DMAEN | ADC_CFGR1_DMACFG;

    /* 选择转换通道：勾选 CH0 与 CH1 参与序列扫描 */
    ADC1->CHSELR = ADC_CHSELR_CHSEL0 | ADC_CHSELR_CHSEL1;

    /* 配置采样时间为最快速度 1.5 周期，满足高频电流环响应 */
    ADC1->SMPR = (0x0u << ADC_SMPR_SMP_Pos);

    /* ========================================================================== */
    /* 5. 核心硬件自校准：注入误差系数，彻底扣除零点温漂与晶圆失调电压              */
    /* ========================================================================== */
    BSP_ADC_Calibration_Start();

    /* ========================================================================== */
    /* 6. 正式使能 ADC                                                           */
    /* ========================================================================== */
    ADC1->ISR |= ADC_ISR_ADRDY;               /* 清除准备就绪标志 */
    ADC1->CR  |= ADC_CR_ADEN;                 /* 开启 ADC 供电 */
    while(!(ADC1->ISR & ADC_ISR_ADRDY));      /* 阻塞等待模拟电路彻底稳定就绪 */
}

/**
  * @brief  启动 ADC 开始连续硬件扫描转换
  */
void BSP_ADC_Start(void)
{
    if ((ADC1->CR & ADC_CR_ADEN) && !(ADC1->CR & ADC_CR_ADSTART)) {
        ADC1->CR |= ADC_CR_ADSTART;
    }
}

/**
  * @brief  停止 ADC 转换
  */
void BSP_ADC_Stop(void)
{
    if (ADC1->CR & ADC_CR_ADSTART) {
        ADC1->CR |= ADC_CR_ADSTP;
        while(ADC1->CR & ADC_CR_ADSTP);
    }
}

/**
  * @brief  带超时防护的、可扩展综合数据拉取函数（核心第一种架构）
  * @param  p_metrics: 指向应用层业务数据集结构体的指针
  * @return 0: 成功获取同周期完美对齐数据; -1: 超时失败（底层硬件停摆）
  */
int32_t BSP_ADC_GetMetrics(volatile BLDC_Metrics_t *p_metrics)
{
    if (p_metrics == 0) return -1;

    /* 1. 防飞车超时计数器：48MHz 下，480000 次空转大约等效于 15~20ms */
    /* 如果外部高频噪声将 ADC 寄存器打挂导致中断不再触发，本计数器能强行破锁，防止卡死 */
    volatile uint32_t timeout_cnt = 480000u; 

    /* 2. 同步死等：如果没有触发新一轮的 DMA 中断，CPU 将在此挂起等待最新干净的数据 */
    while (g_adc_new_data_flag == 0u)
    {
        if (--timeout_cnt == 0u)
        {
            return -1; /* 返回硬件故障代码，上层应立即切断 PWM，进入保护模式 */
        }
    }

    /* 3. 数据无撕裂安全拉取：在此处将底层的原始“数组索引”与抽象的“业务成员名”无缝绑定 */
    p_metrics->phase_i = g_adc_raw_value[1]; /* CH0 自动对齐填入相电流 */
    p_metrics->v_bus   = g_adc_raw_value[0]; /* CH1 自动对齐填入母线电压 */
    
    /* 💡 以后硬件升级多加了通道（例如增加了温度或油门）？直接在下方追加赋值即可！ */
    /* p_metrics->temp_mos = g_adc_raw_value[2]; */
    /* p_metrics->throttle = g_adc_raw_value[3]; */

    /* 4. 自动履约：清空软件标志位，关闭闸门，让下一轮调用继续在开头死等 */
    g_adc_new_data_flag = 0u;

    return 0; /* 成功获取 */
}

/* ========================================================================== */
/* 3. DMA1 通道 1 硬件中断服务函数                                            */
/* ========================================================================== */
void DMA1_Channel1_IRQHandler(void)
{
    /* 检查是否是通道 1 的传输完成（TC）中断触发 */
    if (DMA1->ISR & DMA_ISR_TCIF1)
    {
        /* 强行清除中断标志位，防止无限死循环进中断 */
        DMA1->IFCR = DMA_IFCR_CTCIF1;

        /* 数据已全部稳妥写入内存，打上最新标记 */
        g_adc_new_data_flag = 1;
    }
}

