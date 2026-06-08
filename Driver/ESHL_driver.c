//
// Created by E_LJF on 25-6-7.
// Optimized: 加入查找表重构、软件PLL测速、相位超前补偿、抖动超时保护
//

#include "ESHL_driver.h"
#include "tim.h"
#include "comp.h"
#include "adc.h"
#include "Internal_Flash.h"
#include "stdio.h"

#define ESHL_MAX_CURRENT_ADC_MAX	2191		//电调设计最大电流ADC值

uint16_t ESHL_Data[2] = {0};					//电调需要离线保存的数据,[0]为电调地址(默认EC00,可更改),[1]为电调运行电流限制ADC值,默认1158
volatile uint8_t ESHL_step = 0;					//★OPT: 加 volatile,因为在中断和主循环都会访问
static  uint16_t ESHL_run_pwm = ESHL_RUN_MIN_PWM;//电调运行pwm
ESHL_DIRECTION_ENUM_T ESHL_direction;			//电调运行方向
ESHL_STATE_ENUM_T ESHL_state;					//电调状态
ESHL_BAT_ENUM_T ESHL_BatType;					//电调使用的电池类型
float ESHL_InitVbat = 0;						//初始化时测得的电池电压(V)，用于调试输出
uint32_t ESHL_InitAdcSum10 = 0;					//初始化时10次ADC读数原始总和

uint32_t adc_val_buff[5] = {0};  //ADC值缓存
uint16_t ESHL_MOS_Current_ADC_Value = 0;		//测得的MOS电流
uint16_t ESHL_LeakageCurrent_ADC_Value = 0;		//电调总漏电流

/* ★OPT-NEW: 诊断信息全局变量定义,供外部读取转速、错误信息 */
ESHL_DIAG_T ESHL_Diag = {0};

/* ADC DMA 循环缓冲: [0]=PA0 电压(VBAT), [1]=PA1 电流
 * FORWARD 扫描顺序：IN0(PA0)→[0]，IN1(PA1)→[1]
 * DMA 在后台自动刷新，CPU 直接读取即可，加 volatile 防编译器优化 */
volatile uint16_t adc_dma_buf[2] = {0};


/* ============================================================================
 * ★★★ OPT-NEW: 换相查找表 ★★★
 * ----------------------------------------------------------------------------
 * 把原来 12 个 case 分支中重复的"切换比较器输入通道 + 切换 EXTI 触发边沿"
 * 操作集中到一张表中,索引为 ESHL_step (0~11)
 *
 * 表中每个条目含义:
 *   match_sense  -- 当前步等待的比较器输出值 (1=高电平触发换相, 0=低电平触发)
 *   channel_bits -- 这一步要监听的悬空相对应的 COMP CSR 通道值 (已左移到位)
 *                   0x04 → PA4 (B相)
 *                   0x05 → PA5 (C相)
 *                   0x06 → PA2 (A相)
 *   edge_rising  -- EXTI 触发边沿 (1=上升沿, 0=下降沿)
 *
 * 通过查表,原来 240 行的 switch 大块可以压缩到 30 行核心逻辑
 * ============================================================================ */
typedef struct {
    uint8_t  match_sense;	 //此步期望的 sense 值,匹配则换相
    uint32_t channel_bits;	 //此步监听的相对应 COMP CSR 通道掩码
    uint8_t  edge_rising;	 //此步的 EXTI 触发边沿 (1=上升沿)
} commutation_lut_t;

static const commutation_lut_t commutation_table[12] = {
    /* === 正转 (Forward) === */
    /* idx 0 */ {1, (0x06U << 4U), 1},  // Step 0: 监听 A相(PA2),等待上升沿
    /* idx 1 */ {0, (0x05U << 4U), 0},  // Step 1: 监听 C相(PA5),等待下降沿
    /* idx 2 */ {1, (0x04U << 4U), 1},  // Step 2: 监听 B相(PA4),等待上升沿
    /* idx 3 */ {0, (0x06U << 4U), 0},  // Step 3: 监听 A相(PA2),等待下降沿
    /* idx 4 */ {1, (0x05U << 4U), 1},  // Step 4: 监听 C相(PA5),等待上升沿
    /* idx 5 */ {0, (0x04U << 4U), 0},  // Step 5: 监听 B相(PA4),等待下降沿
    /* === 反转 (Reverse) === */
    /* idx 6 */ {1, (0x04U << 4U), 1},  // Step 6 : 监听 B相,等待上升沿
    /* idx 7 */ {0, (0x05U << 4U), 0},  // Step 7 : 监听 C相,等待下降沿
    /* idx 8 */ {1, (0x06U << 4U), 1},  // Step 8 : 监听 A相,等待上升沿
    /* idx 9 */ {0, (0x04U << 4U), 0},  // Step 9 : 监听 B相,等待下降沿
    /* idx10 */ {1, (0x05U << 4U), 1},  // Step 10: 监听 C相,等待上升沿
    /* idx11 */ {0, (0x06U << 4U), 0},  // Step 11: 监听 A相,等待下降沿
};


/* ★OPT: 提取的内联函数: 应用某一步对应的比较器通道+EXTI边沿配置 */
static inline void apply_comp_config(uint8_t step)
{
    const commutation_lut_t *p = &commutation_table[step];
    /* 1. 切换比较器输入通道 (清除原 INMSEL 位,写入新通道) */
    ESHL_COMP.Instance->CSR = (ESHL_COMP.Instance->CSR & ~(0x7U << 4U)) | p->channel_bits;
    /* 2. 配置 EXTI 触发边沿 */
    if (p->edge_rising) {
        SET_BIT(EXTI->RTSR, COMP_EXTI_LINE_COMP2);
        CLEAR_BIT(EXTI->FTSR, COMP_EXTI_LINE_COMP2);
    } else {
        CLEAR_BIT(EXTI->RTSR, COMP_EXTI_LINE_COMP2);
        SET_BIT(EXTI->FTSR, COMP_EXTI_LINE_COMP2);
    }
}


/**
 * @brief  开环切闭环时，强制同步硬件比较器通道与触发边沿
 * @param  step 当前的状态机步数 (ESHL_step)
 * @note   该函数必须在 HAL_COMP_Start_IT() 之前调用
 *
 * ★OPT: 从原来 120 行的大 switch 改为查表实现, 功能完全等价但代码极简
 */
void ESHL_Sync_Comp_Hardware(uint8_t step)
{
    if (step >= 12) return;					 //★OPT: 范围保护

    apply_comp_config(step);				 //查表设置通道和边沿

    /* 【极其关键】切换通道时内部模拟开关跳变会产生假毛刺，必须在此处把产生的中断标志位清掉！
     * 否则一开启中断就会立马误触发一次。*/
    __HAL_COMP_EXTI_CLEAR_FLAG(COMP_EXTI_LINE_COMP2);
}


//us延时,需要传入一个指向我们所配置的定时器的指针变量(句柄)
void delay_us(TIM_HandleTypeDef *usTIMER,uint16_t us)
{
    __HAL_TIM_SET_COUNTER(usTIMER, 0);  // 把TIMER的counter设为0
    while (__HAL_TIM_GET_COUNTER(usTIMER) < us); //读取定时器的counter值
}
//由于定时器的频率是1MHz，所以其counter每次加1的时候，就表示过了1us
//每次调用时都把定时器的counter先置为0


//关闭所有MOS管
static inline void MOS_CloseAll() {
	ESHL_AD_DISABLE();//关闭A相MOS管
	ESHL_AU_DISABLE();

	ESHL_BU_DISABLE();//关闭B相MOS管
	ESHL_BD_DISABLE();

	ESHL_CU_DISABLE();//关闭C相MOS管
	ESHL_CD_DISABLE();

	delay_us(&ESHL_US_TIM,20);
}


static void ESHL_U_D_Ctrl(uint16_t pwm)
{
    switch (ESHL_step) {
//--------------正转-------------
        case 0://BC MOS
			ESHL_AU_DISABLE();
			ESHL_CU_DISABLE();
		   ESHL_AD_DISABLE();
		   ESHL_BD_DISABLE();

			ESHL_BU_ENABLE(pwm);
			ESHL_CD_ENABLE();
            break;

        case 1://BA MOS
			ESHL_AU_DISABLE();
			ESHL_CU_DISABLE();
		   ESHL_BD_DISABLE();
		   ESHL_CD_DISABLE();

			ESHL_BU_ENABLE(pwm);
			ESHL_AD_ENABLE();
            break;

        case 2://CA MOS
			ESHL_AU_DISABLE();
			ESHL_BU_DISABLE();
		   ESHL_BD_DISABLE();
		   ESHL_CD_DISABLE();

			ESHL_CU_ENABLE(pwm);
			ESHL_AD_ENABLE();
            break;

        case 3://CB MOS
			ESHL_AU_DISABLE();
			ESHL_BU_DISABLE();
		   ESHL_AD_DISABLE();
		   ESHL_CD_DISABLE();

			ESHL_CU_ENABLE(pwm);
			ESHL_BD_ENABLE();
            break;

        case 4://AB MOS
			ESHL_BU_DISABLE();
			ESHL_CU_DISABLE();
		   ESHL_AD_DISABLE();
		   ESHL_CD_DISABLE();

			ESHL_AU_ENABLE(pwm);
			ESHL_BD_ENABLE();
            break;

        case 5://AC MOS
			ESHL_BU_DISABLE();
			ESHL_CU_DISABLE();
		   ESHL_AD_DISABLE();
		   ESHL_BD_DISABLE();

			ESHL_AU_ENABLE(pwm);
			ESHL_CD_ENABLE();
            break;


//--------------反转-------------
		  case 6://AC MOS
			ESHL_BU_DISABLE();
			ESHL_CU_DISABLE();
		   ESHL_AD_DISABLE();
		   ESHL_BD_DISABLE();

			ESHL_AU_ENABLE(pwm);
			ESHL_CD_ENABLE();
			  break;

		  case 7://AB MOS
			ESHL_BU_DISABLE();
			ESHL_CU_DISABLE();
		   ESHL_AD_DISABLE();
		   ESHL_CD_DISABLE();

			ESHL_AU_ENABLE(pwm);
			ESHL_BD_ENABLE();
            break;

		  case 8://CB MOS
			ESHL_AU_DISABLE();
			ESHL_BU_DISABLE();
		   ESHL_AD_DISABLE();
		   ESHL_CD_DISABLE();

			ESHL_CU_ENABLE(pwm);
			ESHL_BD_ENABLE();
            break;

		  case 9://CA MOS
			ESHL_AU_DISABLE();
			ESHL_BU_DISABLE();
		   ESHL_BD_DISABLE();
		   ESHL_CD_DISABLE();

			ESHL_CU_ENABLE(pwm);
			ESHL_AD_ENABLE();
            break;

		  case 10://BA MOS
			ESHL_AU_DISABLE();
			ESHL_CU_DISABLE();
		   ESHL_BD_DISABLE();
		   ESHL_CD_DISABLE();

			ESHL_BU_ENABLE(pwm);
			ESHL_AD_ENABLE();
            break;

		  case 11://BC MOS
			ESHL_AU_DISABLE();
			ESHL_CU_DISABLE();
		   ESHL_AD_DISABLE();
		   ESHL_BD_DISABLE();

			ESHL_BU_ENABLE(pwm);
			ESHL_CD_ENABLE();
            break;

        default:
            break;
    }
}


//电调初始化
void ESHL_ESC_Init()
{

	uint16_t read_buff[2] = {0};
	float Vbat = 0;

	ESHL_AD_DISABLE();//关闭A相MOS管
	ESHL_AU_DISABLE();

	ESHL_BU_DISABLE();//关闭B相MOS管
	ESHL_BD_DISABLE();

	ESHL_CU_DISABLE();//关闭C相MOS管
	ESHL_CD_DISABLE();

	ESHL_state = ESHL_STATE_OFF;
	ESHL_direction = ESHL_CLOCKWISE;

    HAL_TIM_PWM_Start(&ESHL_A_TIM,ESHL_A_TIM_CH);//开启MOS上管定时器
    HAL_TIM_PWM_Start(&ESHL_B_TIM,ESHL_B_TIM_CH);
    HAL_TIM_PWM_Start(&ESHL_C_TIM,ESHL_C_TIM_CH);

    HAL_TIM_Base_Start(&ESHL_US_TIM);//开启us延时定时器

	HAL_ADCEx_Calibration_Start(&ESHL_Current_ADC);//ADC校准

	/* 初始化时阻塞读取电压（DMA 尚未启动，用轮询方式）
	 * ADC 扫描顺序: IN0(PA0 电压) → IN1(PA1 电流)，每轮累加 IN0 丢弃 IN1 */
	HAL_ADC_Start(&ESHL_Current_ADC);
	for (uint8_t i = 0; i < 10; i++) {
		if (HAL_ADC_PollForConversion(&ESHL_Current_ADC, 5) == HAL_OK)
			Vbat += (float)HAL_ADC_GetValue(&ESHL_Current_ADC); // 读 IN0 电压
		if (HAL_ADC_PollForConversion(&ESHL_Current_ADC, 5) == HAL_OK)
			(void)HAL_ADC_GetValue(&ESHL_Current_ADC);          // 丢弃 IN1 电流
	}
	HAL_ADC_Stop(&ESHL_Current_ADC);

	Vbat = (Vbat/10) * 3.3f / 4096 * 10;//ADC值转为电压
	ESHL_InitAdcSum10 = (uint32_t)Vbat;  // 保存10次ADC总和(在/10之前)
	Vbat += 2.18f;	//加是因为ADC不准,有偏差
	ESHL_InitVbat = Vbat;//保存到全局变量，用于调试输出
	printf("Vbat:%d\r\n",(uint8_t)Vbat);

 	if (Vbat > 22.2f && Vbat < 25.2f) {
		ESHL_BatType = ESHL_BAT_6S;
	}else if (Vbat > 18.5f && Vbat < 21.0f) {
		ESHL_BatType = ESHL_BAT_5S;
	}else if (Vbat > 14.8f && Vbat < 16.8f) {
		ESHL_BatType = ESHL_BAT_4S;
	}else if (Vbat > 11.1f && Vbat < 14.f) {
		ESHL_BatType = ESHL_BAT_3S;
	} else{
		ESHL_state = ESHL_STATE_BATTERY_VOLTAGE_ERROR;
	}


	InternalFLASH_ReadMore16(ESHL_ADDR_FLASH_ADD,read_buff,2);
	if ((read_buff[0] >> 8) != 0xEC) {//如果没有存储地址
		ESHL_Data[0] = ESHL_DEFALT_ADDR;//默认地址
		ESHL_Data[1] = ESHL_RUN_Current_Limit_ADC_DEFALT;			//默认限制电流ADC值
		InternalFlashWriteMoreUint_16(ESHL_ADDR_FLASH_ADD,ESHL_Data,2);//写入内部Flash
	}
	else {//已经储存地址
		ESHL_Data[0] = read_buff[0];//复制电调地址
		ESHL_Data[1] = read_buff[1];//复制限制电流ADC值
	}

	/* ★OPT-NEW: 初始化诊断信息 */
	ESHL_ResetDiag();

	/* ★ 启动 ADC DMA 循环采样（只启动一次，之后永不停止）
	 * adc_dma_buf[0]=PA0 电流，adc_dma_buf[1]=PA1 电压，DMA 在后台自动刷新 */
	HAL_ADC_Start_DMA(&ESHL_Current_ADC, (uint32_t *)adc_dma_buf, 2);

	/* ★ 开启 ADC 模拟看门狗中断（AWD 硬件过流保护）
	 * 阈值已在 adc.c MX_ADC_Init 中配置，此处只开中断使能 */
	__HAL_ADC_ENABLE_IT(&ESHL_Current_ADC, ADC_IT_AWD);
}
extern uint32_t close_num;
extern uint32_t comp_num;


//电调开环启动
void ESHL_Start(ESHL_DIRECTION_ENUM_T direction)
{
    static uint16_t time = 100;
	uint8_t BMEF_num = 0;//开环启动时累计过零事件数量
	ESHL_direction = direction;
	ESHL_run_pwm = ESHL_START_PWM;

    HAL_COMP_Stop_IT(&ESHL_COMP);//关闭比较器中断
	HAL_COMP_Start(&ESHL_COMP);  //打开比较器
	MOS_CloseAll();

	ESHL_state = ESHL_STATE_OPEN_LOOP_START;//电调状态更新为开环启动状态
	ESHL_step = (ESHL_direction == ESHL_CLOCKWISE) ? 0 : 6;

	/* ===== 三阶段转子定位 ===== */

	// /* 阶段一: 强力预拉合 — 给转子施加 step=0 方向的磁场,
	//  * 让转子开始向该方向运动,消除完全随机初始位置带来的对立面死角 */
	// ESHL_step = (ESHL_direction == ESHL_CLOCKWISE) ? 0 : 6;
	// ESHL_U_D_Ctrl(ESHL_ALIGN_STRONG_PWM);
	// for (uint16_t i = 0; i < ESHL_ALIGN_PREHOLD_MS; i++) {
	// 	if (adc_dma_buf[1] >= ESHL_RotoCurrent_ADC_MAX) {
	// 		MOS_CloseAll();
	// 		HAL_COMP_Stop(&ESHL_COMP);
	// 		ESHL_run_pwm = 0;
	// 		ESHL_state = ESHL_STATE_CURRENT_ERROR;
	// 		return;
	// 	}
	// 	HAL_Delay(1);
	// }

	// /* 阶段二: 顺序慢扫 — 依次切换 step 1~5(或 7~11),
	//  * 无论转子初始在哪里,扫完后必然跟到末位步附近,消除位置不确定性 */
	// uint8_t scan_start = (ESHL_direction == ESHL_CLOCKWISE) ? 1 : 7;
	// uint8_t scan_end   = (ESHL_direction == ESHL_CLOCKWISE) ? 5 : 11;
	// for (uint8_t s = scan_start; s <= scan_end; s++) {
	// 	ESHL_step = s;
	// 	ESHL_U_D_Ctrl(ESHL_ALIGN_SCAN_PWM);
	// 	for (uint16_t i = 0; i < ESHL_ALIGN_STEP_MS; i++) {
	// 		if (adc_dma_buf[1] >= ESHL_RotoCurrent_ADC_MAX) {
	// 			MOS_CloseAll();
	// 			HAL_COMP_Stop(&ESHL_COMP);
	// 			ESHL_run_pwm = 0;
	// 			ESHL_state = ESHL_STATE_CURRENT_ERROR;
	// 			return;
	// 		}
	// 		HAL_Delay(1);
	// 	}
	// }

	// /* 阶段三: 强力吸合回 step=0 — 转子此时在末位步附近(只差一步),
	//  * 强力拉回 step=0 确保加速换相的起点完全对准 */
	// ESHL_step = (ESHL_direction == ESHL_CLOCKWISE) ? 0 : 6;
	// ESHL_U_D_Ctrl(ESHL_ALIGN_STRONG_PWM);
	// for (uint16_t i = 0; i < ESHL_ALIGN_HOLD_MS; i++) {
	// 	if (adc_dma_buf[1] >= ESHL_RotoCurrent_ADC_MAX) {
	// 		MOS_CloseAll();
	// 		HAL_COMP_Stop(&ESHL_COMP);
	// 		ESHL_run_pwm = 0;
	// 		ESHL_state = ESHL_STATE_CURRENT_ERROR;
	// 		return;
	// 	}
	// 	HAL_Delay(1);
	// }
	MOS_CloseAll();
	delay_us(&ESHL_US_TIM, 5000); // 等待转子完全静止
	/* ===== 三阶段定位完成,转子已对准 step=0(或 step=6) ===== */

    while (1)
    {
        for (uint16_t i = 0; i < time; ++i) {
            delay_us(&ESHL_US_TIM,55);
        }

        if (time < 20)
        {
            time = 100;

            MOS_CloseAll();
        	if ((BMEF_num >= 18) && (BMEF_num <= 35))//开环启动成功
        	{
				printf("Open loop start successful, BMEF_num: %d\r\n", BMEF_num);
        		printf("OPEN_CLOSE_NUM: %ld\r\n", close_num);//输出过零事件数量
				printf("COMP_NUM: %ld\r\n", comp_num);//输出比较器触发数量
				HAL_COMP_Stop(&ESHL_COMP);
				// --- 必须加上这段：强制硬件与状态机 Step 1 对齐 ---
				ESHL_Sync_Comp_Hardware(ESHL_step);

				/* ★OPT-NEW: 进入闭环前重置 PLL 计数,保证测速从干净状态开始 */
				ESHL_Diag.last_zc_tick = __HAL_TIM_GET_COUNTER(&ESHL_US_TIM);
				ESHL_Diag.commutation_period_us = 0;
				ESHL_Diag.rpm = 0;

        		HAL_COMP_Start_IT(&ESHL_COMP);
        		ESHL_state = (ESHL_direction == ESHL_CLOCKWISE) ? ESHL_STATE_RUN_CLOCKWISE : ESHL_STATE_RUN_COUNTER_CLOCKWISE;//电调状态更新为对应方向运动状态
            }
        	else {			//开环启动失败
        		printf("Open loop start failed, BMEF_num: %d\r\n", BMEF_num);
				HAL_COMP_Stop(&ESHL_COMP);
        		HAL_COMP_Stop_IT(&ESHL_COMP);
        		MOS_CloseAll();
        		ESHL_run_pwm = 0;
        		ESHL_state = ESHL_STATE_OPEN_LOOP_START_FAIL;//电调状态更新为开环启动失败
        	}
            break;
        }

    	// HAL_ADC_Start_DMA(&ESHL_Current_ADC,adc_val_buff,2);//检测开环过渡期电流值
    	// if (adc_val_buff[1] >= ESHL_OPEN_LOOP_Transition_Period_ADC_MAX) {
    	// 	HAL_COMP_Stop(&ESHL_COMP);
    	// 	MOS_CloseAll();
    	// 	ESHL_run_pwm = 0;
    	// 	ESHL_state = ESHL_STATE_CURRENT_ERROR;//电调状态更新为电流异常
    	// 	return ;
    	// }

        time -= time/85+1;
        ESHL_step++;

        if(ESHL_direction == ESHL_COUNTER_CLOCKWISE)
		  {
        	ESHL_step = (ESHL_step > 11) ? 6 : ESHL_step;
		  }else
		  {
		  	ESHL_step %= 6;
		  }
		  ESHL_U_D_Ctrl(ESHL_run_pwm);

    	if (SENSE_H) {
    		BMEF_num ++;
    	}

    }
}

uint16_t step_num[12];

/* ★OPT-NEW: 内联工具函数 -- 计算两次过零事件之间的微秒差(处理16位计数器溢出) */
static inline uint32_t calc_us_diff(uint32_t now, uint32_t last)
{
    /* htim6 是 1MHz 的 16 位定时器, 每 65536us (65.5ms) 溢出一次
     * 这里假设最多溢出一次。若 motor 太慢导致溢出>1 次,会得到错误结果,
     * 但低于 ~915 RPM (7极对) 的转速本身就不在 PLL 有效范围内 */
    if (now >= last) {
        return now - last;
    } else {
        return (0x10000UL - last) + now;
    }
}


/* ★OPT-NEW: 内联工具函数 -- 更新 PLL 测速 (经一阶低通滤波) */
static inline void pll_update(uint32_t diff_us)
{
    /* 异常过滤:过短(可能是抖动)或过长(可能是启动期/丢步)都不参与滤波 */
   if (diff_us < 100U || diff_us > 750U) return;

    /* 一阶低通滤波: new = (old*(ALPHA-1) + raw) / ALPHA
     * ALPHA 越大,响应越慢,但滤波越平滑 */
    const uint32_t alpha = ESHL_PLL_FILTER_ALPHA;
    uint32_t old = ESHL_Diag.commutation_period_us;
    if (old == 0) {
        /* 首次测量,直接赋值 */
        ESHL_Diag.commutation_period_us = diff_us;
    } else {
        ESHL_Diag.commutation_period_us = (old * (alpha - 1) + diff_us) / alpha;
    }

    /* 计算 RPM:
     *   每电气周期有 6 次换相, 故电气周期 T_e = 6 * commutation_period_us
     *   电气频率 f_e = 1e6 / T_e
     *   机械频率 f_m = f_e / pole_pairs
     *   RPM = f_m * 60 = 60 * 1e6 / (6 * commutation_period_us * pole_pairs)
     *       = 10000000 / (commutation_period_us * pole_pairs)
     */
    if (ESHL_Diag.commutation_period_us > 0) {
        ESHL_Diag.rpm = 10000000UL /
            (ESHL_Diag.commutation_period_us * ESHL_MOTOR_POLE_PAIRS);
    }
}


/* ★OPT-NEW: 内联工具函数 -- 根据当前转速计算相位超前延迟(us)
 *
 * 实现"换相之后等待一定时间再真正切换 MOS"的核心函数:
 *
 *   基础延迟 = 换相周期 / 2  (对应 30° 电角度)
 *   超前补偿 = 基础延迟 * 超前百分比 / 100
 *   实际延迟 = 基础延迟 - 超前补偿
 *
 * 速度越高,超前百分比越大,实际等待时间越短
 * 速度低于阈值时不补偿 (按完整 30° 延迟)
 * 速度高于阈值时用最大补偿
 */
static inline uint32_t calc_phase_advance_delay_us(void)
{
#if ESHL_PHASE_ADV_ENABLE
    uint32_t period = ESHL_Diag.commutation_period_us;
    if (period == 0) return 0;

    uint32_t rpm = ESHL_Diag.rpm;
    if (rpm < ESHL_PHASE_ADV_LOW_RPM) return 0;

    uint32_t pct;
    if (rpm >= ESHL_PHASE_ADV_HIGH_RPM) {
        pct = ESHL_PHASE_ADV_MAX_PCT;
    } else {
        pct = (rpm - ESHL_PHASE_ADV_LOW_RPM) * ESHL_PHASE_ADV_MAX_PCT
            / (ESHL_PHASE_ADV_HIGH_RPM - ESHL_PHASE_ADV_LOW_RPM);
    }

    uint32_t delay = period * pct / 100;

    /* 分段限制：
     * 低速(<8000RPM)  最大 25us
     * 高速(>=8000RPM) 最大 15us
     * 防止高速过补偿 */
    uint32_t max_delay;
    if (rpm < 8000) {
        max_delay = 25;
    } else {
        max_delay = 15;
    }
    if (delay > max_delay) delay = max_delay;

    return delay;
#else
    return 0;
#endif
}

/* ============================================================================
 * ★★★ 优化后的比较器中断回调 (核心 ISR) ★★★
 * ----------------------------------------------------------------------------
 * 改动总览:
 *   1. 用查找表 commutation_table[] 代替 12 个 case 分支
 *   2. 加入软件 PLL: 测量过零事件间隔,估算换相周期和转速
 *   3. 加入相位超前补偿: 检测到过零后等待 (30°-超前角) 才换相
 *   4. 加入抖动循环超时保护: 防止信号异常时卡死中断
 *   5. 记录诊断信息: ISR 调用次数、超时次数等
 *
 * 注意: 当 ESHL_PHASE_ADV_ENABLE=0 时,行为与原代码完全等价
 * ============================================================================ */
void HAL_COMP_TriggerCallback(COMP_HandleTypeDef *hcomp)
{
    /* ★OPT: 防御性参数校验 */
    if (hcomp == NULL) return;

    /* ★OPT-NEW: ISR 进入计数,便于调试 */
    ESHL_Diag.isr_call_count++;

    /* ★OPT-NEW: 步数越界保护 -- 若 ESHL_step 被异常修改,立即停机 */
    if (ESHL_step >= 12) {
        MOS_CloseAll();
        return;
    }

    /* ★OPT-NEW: PLL 测速 -- 记录两次过零事件之间的时间差
     * 注意: 必须在【任何阻塞操作(相位延迟)之前】采样,才能测到真实的换相周期 */
    uint32_t now_us = __HAL_TIM_GET_COUNTER(&ESHL_US_TIM);
    uint32_t diff_us = calc_us_diff(now_us, ESHL_Diag.last_zc_tick);
    ESHL_Diag.last_zc_tick = now_us;

    uint8_t sense = 0;
    uint16_t debounce_loops = 0;	 //★OPT-NEW: 抖动循环计数器

    __disable_irq();
    do
    {
        if(SENSE_H) sense = 1; else sense = 0;

        /* ★OPT-NEW: 抖动循环超时保护 -- 防止无限循环卡死
         * 若信号长时间不稳定(硬件故障/EMI干扰),立即停机并报错 */
        if (++debounce_loops > ESHL_DEBOUNCE_MAX_LOOPS) {
            MOS_CloseAll();
            HAL_COMP_Stop_IT(&ESHL_COMP);
            ESHL_run_pwm = 0;
            ESHL_state = ESHL_STATE_SIGNAL_UNSTABLE;
            ESHL_Diag.debounce_overrun_cnt++;
            __enable_irq();
            return;
        }

        /* ★OPT: 用查找表替代 12 个 case 分支 */
        const commutation_lut_t *lut = &commutation_table[ESHL_step];

        if (sense == lut->match_sense) {
            /*-- 匹配换相条件 ---------------------------------------------*/

            /* ★OPT-NEW: 相位超前补偿 -- 在真正换相前等待计算好的延迟时间
             * 关闭时(ESHL_PHASE_ADV_ENABLE=0)返回 0, 行为与原代码相同 */
            uint32_t adv_delay = calc_phase_advance_delay_us();
            if (adv_delay > 0 && adv_delay < 10000U) {
                /* 注意: 这里仍在中断里短暂忙等,适合 us 级延迟
                 * 若要严格不阻塞中断,需要改为定时器触发换相,代价是要占用一个TIM */
                delay_us(&ESHL_US_TIM, (uint16_t)adv_delay);
            }

            /* 步进 */
            if (ESHL_step >= 6) {
                /* 反转模式: 步数在 6~11 之间循环 */
                ESHL_step = (ESHL_step >= 11) ? 6 : (ESHL_step + 1);
                close_num++;
            } else {
                /* 正转模式: 步数在 0~5 之间循环 */
                ESHL_step = (ESHL_step + 1) % 6;
                step_num[(lut - commutation_table) * 2 % 12]++;
            }

            /* 切换 MOS 管输出 */
            ESHL_U_D_Ctrl(ESHL_run_pwm);

            /* 切换比较器到下一个悬空相,并切换 EXTI 触发边沿 (查表) */
            apply_comp_config(ESHL_step);

            /* ★OPT-NEW: 更新 PLL 测速 (用换相前采样的时间差) */
            pll_update(diff_us);
            /* 仅首次匹配的那次过零参与测速,后续抖动循环不再更新 */
            diff_us = 0;

        } else {
            /*-- 未匹配: 状态未变化,只刷新 PWM ----------------------------*/
            if (ESHL_step >= 6) {
                close_num++;
            } else {
                step_num[((lut - commutation_table) * 2 + 1) % 12]++;
            }
            ESHL_U_D_Ctrl(ESHL_run_pwm);
        }

    } while((SENSE_L && sense) || (SENSE_H && !sense));
    //如果状态不稳定（过零时有抖动），则继续在中断中处理，直到状态稳定

    /* ★OPT-NEW: 记录峰值抖动次数,便于调试硬件信号质量 */
    if (debounce_loops > ESHL_Diag.max_debounce_loops) {
        ESHL_Diag.max_debounce_loops = debounce_loops;
    }

    __enable_irq();
}




//ESHL_Beep(ESHL_BEEP_SHORT);//E
//
//	ESHL_Beep(ESHL_BEEP_SHORT);//S
//	ESHL_Beep(ESHL_BEEP_SHORT);
//	ESHL_Beep(ESHL_BEEP_SHORT);
//
//	ESHL_Beep(ESHL_BEEP_SHORT);//H
//	ESHL_Beep(ESHL_BEEP_SHORT);
//	ESHL_Beep(ESHL_BEEP_SHORT);
//	ESHL_Beep(ESHL_BEEP_SHORT);
//
//	ESHL_Beep(ESHL_BEEP_SHORT);//L
//	ESHL_Beep(ESHL_BEEP_LONG);
//	ESHL_Beep(ESHL_BEEP_SHORT);
//	ESHL_Beep(ESHL_BEEP_SHORT);




//电机发声函数
void ESHL_Beep(ESHL_BEEP_ENUM_T beep)
{
	if (ESHL_state == ESHL_STATE_OFF) {
		MOS_CloseAll();

		ESHL_step = 0;

		switch (beep) {
			case ESHL_BEEP_SHORT:
				for (uint16_t i = 0; i < 250; i++) {
					ESHL_step = 0;
					ESHL_U_D_Ctrl(100);
					delay_us(&ESHL_US_TIM, 20);
					ESHL_step = 1;
					ESHL_U_D_Ctrl(100);
					delay_us(&ESHL_US_TIM, 180);
				}
				break;

			case ESHL_BEEP_LONG:
				for (uint16_t i = 0; i < 500; i++) {
					ESHL_step = 0;
					ESHL_U_D_Ctrl(100);
					delay_us(&ESHL_US_TIM, 50);
					ESHL_step = 1;
					ESHL_U_D_Ctrl(100);
					delay_us(&ESHL_US_TIM, 180);
				}
				break;

				//		case ESHL_BEEP_DOU:
				//			for(uint16_t i = 0;i < 50;i++)
				//			{
				//				ESHL_step = 0;
				//				ESHL_U_D_Ctrl(100);
				//				delay_us(&ESHL_US_TIM,100);
				//				ESHL_step = 1;
				//				ESHL_U_D_Ctrl(100);
				//				delay_us(&ESHL_US_TIM,500);
				//			}
				//			break;

			default:
				break;
		}

		ESHL_U_D_Ctrl(0);

		MOS_CloseAll();
		HAL_Delay(20);
	}
}


//设置电调运行PWM值
void ESHL_SET_PWM(uint16_t pwm) {
	if (ESHL_state == ESHL_STATE_RUN_CLOCKWISE || ESHL_state == ESHL_STATE_RUN_COUNTER_CLOCKWISE) {
		pwm = (pwm <= ESHL_RUN_MAX_PWM) ? pwm : ESHL_RUN_MAX_PWM;//限制最大值
		pwm = (pwm >= ESHL_RUN_MIN_PWM) ? pwm : ESHL_RUN_MIN_PWM;//限制最小值

		if (pwm > ESHL_run_pwm) {
			//printf("Increasing PWM from1 %d to %d\r\n", ESHL_run_pwm, pwm);
			ESHL_run_pwm += ESHL_RUN_PWM_STEP;
			ESHL_run_pwm = (ESHL_run_pwm <= 991) ? ESHL_run_pwm : 991;
			//printf("Increasing PWM from2 %d to %d\r\n", ESHL_run_pwm, pwm);
		}
		else if (pwm < ESHL_run_pwm) {
			//printf("Decreasing PWM from2 %d to %d\r\n", ESHL_run_pwm, pwm);
			if (ESHL_run_pwm <= ESHL_RUN_PWM_STEP) {
				ESHL_run_pwm = 0;
			}
			else {
				ESHL_run_pwm -= ESHL_RUN_PWM_STEP;
			}
		}
		else if (ESHL_run_pwm == pwm){
			ESHL_run_pwm = pwm;
		}
	}
}



/* ================================================================
 * ADC 模式切换辅助函数（供 MOS_SelfTest 独占使用）
 *
 * 正常运行时 ADC 处于 DMA 循环模式，MOS 自检需要手动逐步采样，两者互斥
 * ================================================================ */

/* 自检前：暂停 DMA 循环，关闭 AWD 中断 */
static void ADC_SwitchToSingleMode(void)
{
    HAL_ADC_Stop_DMA(&ESHL_Current_ADC);
    __HAL_ADC_DISABLE_IT(&ESHL_Current_ADC, ADC_IT_AWD);
    HAL_Delay(1);  // 等待 DMA 真正停止
}

/* 自检后：恢复 DMA 循环和 AWD 中断（无论成功失败都必须调用）*/
static void ADC_RestoreContinuousMode(void)
{
    __HAL_ADC_CLEAR_FLAG(&ESHL_Current_ADC, ADC_FLAG_AWD);  // 清自检期间可能产生的 AWD 标志
    __HAL_ADC_ENABLE_IT(&ESHL_Current_ADC, ADC_IT_AWD);
    HAL_ADC_Start_DMA(&ESHL_Current_ADC, (uint32_t *)adc_dma_buf, 2);
}

/* 单次阻塞采样（自检专用）
 * 依赖 CubeMX 配置: EOCSelection = EOC at end of single channel conversion
 * 返回 0=成功, 1=超时 */
static uint8_t ADC_SampleOnce(uint16_t *out_current, uint16_t *out_vbat)
{
    if (HAL_ADC_Start(&ESHL_Current_ADC) != HAL_OK) return 1;
    if (HAL_ADC_PollForConversion(&ESHL_Current_ADC, 5) != HAL_OK) {
        HAL_ADC_Stop(&ESHL_Current_ADC); return 1;
    }
    /* 第一次转换 = IN0 (PA0) = 电压 */
    if (out_vbat) *out_vbat = (uint16_t)HAL_ADC_GetValue(&ESHL_Current_ADC);
    else          (void)HAL_ADC_GetValue(&ESHL_Current_ADC); // 不需要时也必须读走，清 EOC
    /* 第二次转换 = IN1 (PA1) = 电流 */
    if (out_current) {
        if (HAL_ADC_PollForConversion(&ESHL_Current_ADC, 5) != HAL_OK) {
            HAL_ADC_Stop(&ESHL_Current_ADC); return 1;
        }
        *out_current = (uint16_t)HAL_ADC_GetValue(&ESHL_Current_ADC);
    }
    HAL_ADC_Stop(&ESHL_Current_ADC);
    return 0;
}

/* 单管电流测试公共逻辑（消除原 6 个 case 的重复代码）
 * 调用前：已通过宏开启目标 MOS 管
 * 调用后：自动关闭所有管
 * 返回 0=正常, 1=超限（超限时写入 *p_error_id = mos_id）*/
static uint8_t MOS_TestOneTube(uint8_t mos_id, uint16_t threshold, uint8_t *p_error_id)
{
    uint16_t peak_adc = 0;
    uint16_t sample   = 0;
    for (uint8_t i = 0; i < ESHL_MOS_Current_Test_num; i++) {
        if (ADC_SampleOnce(&sample, NULL) == 0 && sample > peak_adc)
            peak_adc = sample;
    }
    MOS_CloseAll();
    if (peak_adc > threshold) {
        *p_error_id = mos_id;
        printf("MOS Self-Test: #%d fault, ADC peak: %d\r\n", mos_id, peak_adc);
        return 1;
    }
    return 0;
}

/* 检测MOS管是否短路
 * 返回问题MOS管编号: 0=无异常, 12=漏电流过大
 * MOS管编号: 1(AD) 2(AU) 3(BD) 4(BU) 5(CD) 6(CU) */
uint8_t MOS_SelfTest(void)
{
    uint8_t  error_mos_id = 0;
    uint16_t peak_adc     = 0;
    uint16_t sample       = 0;

    HAL_COMP_Stop_IT(&ESHL_COMP);
    MOS_CloseAll();
    ADC_SwitchToSingleMode();  // ★ 暂停 DMA 循环和 AWD

    /* Step 0: 测总漏电流（所有管关闭时的底噪基准）*/
    for (uint8_t i = 0; i < ESHL_MOS_Current_Test_num; i++) {
        if (ADC_SampleOnce(&sample, NULL) == 0 && sample > peak_adc)
            peak_adc = sample;
			//printf("MOS Self-Test: Leakage current sample %d: ADC %d\r\n", i, sample);
    }
    ESHL_LeakageCurrent_ADC_Value = peak_adc;
    if (peak_adc >= 15) {
        error_mos_id = 12;
        printf("MOS Self-Test: Leakage current fault, ADC: %d\r\n", peak_adc);
        goto selftest_done;  // 漏电流大可能有管子损坏，不继续测试
    }

    /* Step 1~6: 逐管测试，发现故障立即退出 */
    ESHL_AD_ENABLE();
    if (MOS_TestOneTube(1, ESHL_MOS_Current_ADC_MAX, &error_mos_id)) goto selftest_done;

    ESHL_AU_ENABLE(ESHL_MOS_TestPWM);
    if (MOS_TestOneTube(2, ESHL_MOS_Current_ADC_MAX, &error_mos_id)) goto selftest_done;

    ESHL_BD_ENABLE();
    if (MOS_TestOneTube(3, ESHL_MOS_Current_ADC_MAX, &error_mos_id)) goto selftest_done;

    ESHL_BU_ENABLE(ESHL_MOS_TestPWM);
    if (MOS_TestOneTube(4, ESHL_MOS_Current_ADC_MAX, &error_mos_id)) goto selftest_done;

    ESHL_CD_ENABLE();
    if (MOS_TestOneTube(5, ESHL_MOS_Current_ADC_MAX, &error_mos_id)) goto selftest_done;

    ESHL_CU_ENABLE(ESHL_MOS_TestPWM);
    if (MOS_TestOneTube(6, ESHL_MOS_Current_ADC_MAX, &error_mos_id)) goto selftest_done;

    printf("MOS Self-Test: All passed.\r\n");

selftest_done:
    MOS_CloseAll();
    ADC_RestoreContinuousMode();  // ★ 无论成功失败，必须恢复 DMA 循环和 AWD
    return error_mos_id;
}



//电机运行过程中检测电机是否停转
void ESHL_RuningChack() {
	static uint32_t last_tick = 0;
	static uint8_t last_step = 0;
	if ((ESHL_state == ESHL_STATE_RUN_CLOCKWISE) || (ESHL_state == ESHL_STATE_RUN_COUNTER_CLOCKWISE) /*|| (ESHL_state == ESHL_STATE_MOTOR_RUNING_STOP)*/) {
		if (ESHL_step != last_step) {
			last_step = ESHL_step;
			last_tick = HAL_GetTick();
		}
		else if (HAL_GetTick() - last_tick >= ESHL_MOTOR_TIMEOUT) {
			HAL_COMP_Stop_IT(&ESHL_COMP);
			MOS_CloseAll();
			ESHL_run_pwm = 0;
			last_tick = 0;
			last_step = 0;
			ESHL_state = ESHL_STATE_MOTOR_RUNING_STOP;//电调状态更新为电机运行时停转
		}
	}
}


/* 电压 ADC 低压阈值查找表（整数，无浮点）
 * 计算: adc = (Vbat - 1.18) / 10 * 4096 / 3.3  (反推自原代码偏移公式)
 * ⚠ 以下值需用万用表实测标定，板间 ADC 偏差较大 */
static const uint16_t vbat_low_threshold_adc[4] = {
    1232,  // ESHL_BAT_3S: 11.1V
    1690,  // ESHL_BAT_4S: 14.8V
    2148,  // ESHL_BAT_5S: 18.5V
    2606,  // ESHL_BAT_6S: 22.2V
};

//电机运行时定时检测电流和电池电压
void ESHL_RuningCurrentVBATChack(void)
{
    static uint32_t last_tick    = 0;
    static uint8_t  vbat_low_cnt = 0;

    /* 节流：100ms 执行一次（电压变化慢） */
    if (HAL_GetTick() - last_tick < 100) return;
    last_tick = HAL_GetTick();

    /* 电流：直接读 DMA 最新值，过流已由 AWD 硬件处理 */
    ESHL_Diag.current_avg_adc = adc_dma_buf[1];

#if ESHL_VBAT_CHACK_EN
    /* 电压：读 DMA 缓冲中 PA0 最新值，无需阻塞采样 */
    uint16_t vbat_adc = adc_dma_buf[0];

    if (ESHL_BatType <= ESHL_BAT_6S) {
        if (vbat_adc <= vbat_low_threshold_adc[ESHL_BatType]) {
            if (++vbat_low_cnt >= 3) {  // 连续 3 次(300ms)才停机，防误判
                HAL_COMP_Stop_IT(&ESHL_COMP);
                MOS_CloseAll();
                ESHL_run_pwm = 0;
                ESHL_state = ESHL_STATE_BATTERY_VOLTAGE_ERROR;
                ESHL_Diag.last_lowvbat_adc = vbat_adc;
                vbat_low_cnt = 0;
            }
        } else {
            vbat_low_cnt = 0;
        }
    }
#endif
}


//更改电调运行限制电流
void ESHL_ChangeRuningCurrentLimit(uint16_t current) {
	ESHL_Data[1] = (current >= ESHL_MAX_CURRENT_ADC_MAX) ? ESHL_MAX_CURRENT_ADC_MAX : current;
	InternalFlashWriteMoreUint_16(ESHL_ADDR_FLASH_ADD,ESHL_Data,2);//限制电流ADC值写入内部Flash
}


//外部电调状态设置接口
void ESHL_SetState(ESHL_STATE_ENUM_T state) {
	ESHL_state = state;
}


//获取电调当前状态
ESHL_STATE_ENUM_T ESHL_GetState() {
	return ESHL_state;
}


//电调刹车
void ESHL_Break() {

	static  uint32_t BMEF_NUM = 0;//过零事件计数
	static uint32_t BMEF_NUM_Last = 0;//上一次过零事件计数
	static uint16_t Same_BMEF_NUM = 0;//过零事件计数相同的次数

	if (ESHL_state == ESHL_STATE_BRAKE) {
		ESHL_run_pwm = 0;
		MOS_CloseAll();

		HAL_COMP_Start(&ESHL_COMP);

		if (SENSE_H) {
			BMEF_NUM ++;
		}

		if ((BMEF_NUM - BMEF_NUM_Last) != 0) {//电机还没停转
			BMEF_NUM_Last = BMEF_NUM;
		}else if (BMEF_NUM - BMEF_NUM_Last == 0) {
			Same_BMEF_NUM ++;
		}

		if (Same_BMEF_NUM >= ESHL_BREAK_OK_SAME_BMEF_NUM) {//判定为电机停转
			MOS_CloseAll();
			HAL_COMP_Stop(&ESHL_COMP);
			ESHL_state = ESHL_STATE_BRAKE_OK;//更新状态为刹车成功
			BMEF_NUM = 0;
			BMEF_NUM_Last = 0;
			Same_BMEF_NUM = 0;

			/* ★OPT-NEW: 刹车成功后清零 PLL/转速,防止下次启动用到陈旧数据 */
			ESHL_Diag.rpm = 0;
			ESHL_Diag.commutation_period_us = 0;
			return;//退出函数
		}

#if ESHL_BREAK_MOD == 1//启用三相短路刹车
		ESHL_AU_ENABLE(ESHL_BREAK_PWM);
		ESHL_BU_ENABLE(ESHL_BREAK_PWM);
		ESHL_CU_ENABLE(ESHL_BREAK_PWM);

		for (uint8_t i = 0;i < 25; i++) {
			HAL_ADC_Start_DMA(&ESHL_Current_ADC, adc_val_buff, 2); //检测电调电流
			if (adc_val_buff[1] > ESHL_Data[1]) {//过流保护
				MOS_CloseAll();
				break;
			}
		}
#endif
	}
}


//获取电调地址
uint16_t ESHL_GetAddr() {
	return ESHL_Data[0];
}


//设置电调地址
//addr为接收到的地址,不是要设置的地址
/*主机用广播地址发送更改地址命令,更改地址时,所有已连接的电调地址都需要重新设置
地址设置方法: 快速旋转已经连接上电调的电机,此时电调将自动将自身地址设为EC00
同时电调状态指示灯会闪烁,闪烁次数代表电调地址
此时旋转第二个电调的电机,此时电调将自动将自身地址设为EC01,以此类推,直到所有电调地址设置完毕
电调设置地址后需要重新上电才能使用*/
void ESHL_SetAddr(uint16_t addr) {

	static uint16_t max_addr = 0;//当前最大地址
	static uint8_t BMEF_Num = 0;//过零事件计数

	HAL_COMP_Start(&ESHL_COMP);

	if (SENSE_H) {
		BMEF_Num ++;
	}

	if (((addr >> 8) != 0xEC) && (addr != 0x0000)) {//输入处理
		BMEF_Num = 0;
		return;
	}
	if ((addr & 0xff) > (max_addr & 0xff)) {
		max_addr = addr;//更新地址最大值
	}

	if (BMEF_Num >= 12) {
		if (((max_addr & 0xff) + 1) >= 0xFF) {//达到最大地址
			ESHL_Data[0] = 0xECFF;//最大地址
		}
		else if (addr == 0x0000) {
			ESHL_Data[0] = 0xEC01;//第一个设置地址的电调
		}
		else {
			ESHL_Data[0] = (max_addr + 1);
		}
		InternalFlashWriteMoreUint_16(ESHL_ADDR_FLASH_ADD,ESHL_Data,2);//电调地址写入内部Flash
		ESHL_state = ESHL_STATE_SET_ADDR_OK;//更新电调状态
		HAL_COMP_Stop(&ESHL_COMP);
		max_addr = 0;
		BMEF_Num = 0;
	}
}


//更改电调运行方向
void ESHL_SetDirection(ESHL_DIRECTION_ENUM_T direction) {
	if ((ESHL_state != ESHL_STATE_RUN_CLOCKWISE) && (ESHL_state != ESHL_STATE_RUN_COUNTER_CLOCKWISE)) {//电调非运行状态才能更改方向
		ESHL_direction = direction;
	}
}


//获取电调当前运行方向外部接口
ESHL_DIRECTION_ENUM_T ESHL_GetDirection() {
	return ESHL_direction;
}


//关闭电调,可被通信唤醒
// void ESHL_OFF() {
// 	ESHL_run_pwm = 0;
// 	HAL_COMP_Stop_IT(&ESHL_COMP);
// 	HAL_COMP_Stop(&ESHL_COMP);
// 	MOS_CloseAll();
// 	ESHL_state = ESHL_STATE_OFF;
// }


//关闭所有MOS管并且关闭比较器
void ESHL_CloseMOSComp() {
	ESHL_run_pwm = 0;
	HAL_COMP_Stop_IT(&ESHL_COMP);
	HAL_COMP_Stop(&ESHL_COMP);
	MOS_CloseAll();

	/* ★OPT-NEW: 同时清零转速记录,下次启动从干净状态开始 */
	ESHL_Diag.rpm = 0;
	ESHL_Diag.commutation_period_us = 0;
}


/* ============================================================================
 * ★★★ OPT-NEW: 对外的诊断/转速 API
 * ============================================================================ */

/**
 * @brief  获取当前电机转速 (RPM)
 * @retval RPM 值。返回 0 表示尚未测到有效转速 (静止/刚启动)
 * @note   该值由比较器中断中的软件 PLL 实时更新,经过一阶低通滤波
 */
uint32_t ESHL_GetSpeedRPM(void) {
    return ESHL_Diag.rpm;
}

/**
 * @brief  获取当前换相周期 (us)
 * @retval 周期(us)。一个完整电气周期 = 6 × 该值。返回 0 表示未测到
 */
uint32_t ESHL_GetCommutationPeriod(void) {
    return ESHL_Diag.commutation_period_us;
}

/**
 * @brief  复位诊断信息(转速、错误计数等)
 */
void ESHL_ResetDiag(void) {
    ESHL_Diag.last_zc_tick            = 0;
    ESHL_Diag.commutation_period_us   = 0;
    ESHL_Diag.rpm                     = 0;
    ESHL_Diag.debounce_overrun_cnt    = 0;
    ESHL_Diag.max_debounce_loops      = 0;
    ESHL_Diag.isr_call_count          = 0;
    ESHL_Diag.overcurrent_cnt         = 0;
    ESHL_Diag.last_overcurrent_adc    = 0;
    ESHL_Diag.last_lowvbat_adc        = 0;
    ESHL_Diag.current_avg_adc         = 0;
}


/* ================================================================
 * HAL_ADC_LevelOutOfWindowCallback
 * ADC 模拟看门狗回调（硬件过流保护）
 *
 * 触发条件：PA0(IN0) 电流采样值超过 AWD 高阈值（1158 ≈ 37A）
 * 响应时间：< 5µs（远快于原 30ms 软件轮询）
 * 注意：ADC1_COMP_IRQn 与 COMP2 共享，此回调仅在过流时触发（极低频）
 * ================================================================ */
void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance != ADC1) return;

    if ((ESHL_state != ESHL_STATE_RUN_CLOCKWISE) &&
        (ESHL_state != ESHL_STATE_RUN_COUNTER_CLOCKWISE)) return;

    static uint8_t  overcurrent_cnt   = 0;
    static uint32_t last_trigger_tick = 0;

    /* 直接读 ADC 数据寄存器，这是触发 AWD 时的原始值 */
    uint16_t val = (uint16_t)(hadc->Instance->DR & 0xFFF);
    uint32_t now = HAL_GetTick();

    /* 两次触发间隔超过 1ms 视为非连续，重新计数 */
    if (now - last_trigger_tick > 1) {
        overcurrent_cnt = 0;
    }
    last_trigger_tick = now;

    if (++overcurrent_cnt >= 3) {
        overcurrent_cnt = 0;

        HAL_COMP_Stop_IT(&ESHL_COMP);
        MOS_CloseAll();
        //HAL_ADC_Stop_DMA(&ESHL_Current_ADC);

        ESHL_run_pwm = 0;
        ESHL_state = ESHL_STATE_CURRENT_ERROR;

        printf("Overcurrent! ADC: %d (%.1fA)\r\n",
               val, (float)val / 31.3f);
        ESHL_Diag.last_overcurrent_adc = val;
        ESHL_Diag.overcurrent_cnt++;
    }
}


void ESHL_PrintSpeedAndAdcReport(void)
{
    static uint32_t last_tick = 0;
    static uint8_t print_state = 0; // 0: 打印转速, 1: 打印电气参数

    // 节流：每 1000ms 触发一次
    if (HAL_GetTick() - last_tick < 100) return;
    last_tick = HAL_GetTick();

    if (print_state == 0) {
        // 打印一次转速
        printf("==== SPEED ====\r\nRPM: %lu \tPeriod: %lu us\r\n",
               (unsigned long)ESHL_GetSpeedRPM(),
               (unsigned long)ESHL_GetCommutationPeriod());
        print_state = 1; // 切换状态
    } 
    else {
        printf("==== POWER ====\r\nVBAT ADC: %u \tCurrent ADC: %u\r\n",
               (unsigned)adc_dma_buf[0],
               (unsigned)ESHL_Diag.current_avg_adc);
        print_state = 0; // 切换回转速
    }
}
