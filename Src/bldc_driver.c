/**
  * @file    bldc_driver.c
  * @brief   电调（ESC）控制层核心逻辑实现
  * @note    本文件是电调算法层与硬件驱动层之间的桥梁，实现以下功能：
  *          - ESC 状态机管理（关闭、待机、启动、运行、刹车、异常等状态）
  *          - 六步换相控制（正转 6 步 + 反转 6 步，共 12 步换相表）
  *          - MOS 管上电自检（逐一检测每只 MOS 是否存在短路）
  *          - 电池电压自动识别（3S ~ 6S 锂电池）
  *          - 通过高频 PWM 驱动电机线圈发声（短鸣 / 长鸣）
  *          - 运行中电流与电池电压周期性巡检保护
  */
#include "bldc_driver.h"
#include "bldc_debug.h"
#include "bldc_clock.h"
#include "bldc_comp.h"

/* ========================================================================== */
/* 全局变量：电调运行状态与参数                                                */
/* ========================================================================== */

/* 电调当前运行状态，初始为关闭状态（ESHL_STATE_OFF），由主循环状态机统一调度管理 */
static uint8_t ESHL_state = ESHL_STATE_OFF;

/* 电调旋转方向，默认顺时针（ESHL_CLOCKWISE = 0），可由外部指令修改 */
static uint8_t ESHL_direction = ESHL_CLOCKWISE;

/* 检测到的电池类型，默认 3S（11.1V），上电初始化时根据分压采样电压自动识别 */
uint8_t ESHL_BatType = ESHL_BAT_3S;

/* 母线电压 ADC 原始采样值（0 ~ 4095），用于实时电压监测 */
uint16_t ESHL_vbus = 0;

/* 相电流 ADC 原始采样值（0 ~ 4095），用于运行电流监测 */
uint16_t ESHL_phase_i = 0;

/* MOS 管导通电流 ADC 最大值记录，用于自检时判定 MOS 是否短路损坏 */
uint16_t ESHL_MOS_Current_ADC_Value = 0;

/* 电调总漏电流 ADC 最大值记录，用于判断功率回路是否存在异常漏电 */
uint16_t ESHL_LeakageCurrent_ADC_Value = 0;

/* 六步换相步骤计数器（0 ~ 11），对应 ESHL_U_D_Ctrl() 中的 12 步换相表 */
static uint8_t ESHL_step = 0;

static  uint16_t ESHL_run_pwm = ESHL_RUN_MIN_PWM;//电调运行pwm
/**
  * 电调需要离线持久化存储的数据
  *   [ESHL_DATA_ADDR_IDX]          = 电调通信地址（默认 0xEC00，可在线更改并保存至 Flash）
  *   [ESHL_DATA_CURRENT_LIMIT_IDX] = 运行电流限制 ADC 阈值（默认 1158），超出此值触发过流保护
  */
uint16_t ESHL_Data[2] = {0};

//电调初始化
void ESHL_ESC_Init()
{

	float Vbat = 0;

	ESHL_AD_DISABLE();//关闭A相MOS管
	ESHL_AU_DISABLE();

	ESHL_BU_DISABLE();//关闭B相MOS管
	ESHL_BD_DISABLE();

	ESHL_CU_DISABLE();//关闭C相MOS管
	ESHL_CD_DISABLE();

	ESHL_state = ESHL_STATE_OFF;
	ESHL_direction = ESHL_CLOCKWISE;

    ESHL_AU_START();
    ESHL_BU_START();
    ESHL_CU_START();

    //HAL_TIM_Base_Start(&ESHL_US_TIM);//开启us延时定时器
	//HAL_ADCEx_Calibration_Start(&ESHL_Current_ADC);//ADC校准
    //ESHL_ADC_CALIBRATION_START();

	for (uint8_t i = 0; i < 10; i++) {
		BSP_ADC_GetMetrics(&g_adc_metrics);
		Vbat += g_adc_metrics.v_bus;
        delay_ms(10);
	}

	Vbat = ESHL_VBAT_CalcVoltage((uint16_t)(Vbat / 10));
	LOG_INFO("Vbat = %f\r\n", 	Vbat);
 	if (Vbat > 22.2f && Vbat < 25.2f) {
		ESHL_BatType = ESHL_BAT_6S;
	}else if (Vbat > 18.5f && Vbat < 21.0f) {
		ESHL_BatType = ESHL_BAT_5S;
	}else if (Vbat > 14.8f && Vbat < 16.8f) {
		ESHL_BatType = ESHL_BAT_4S;
	}else if (Vbat > 11.1f && Vbat < 12.6f) {
		ESHL_BatType = ESHL_BAT_3S;
	} else{
		ESHL_state = ESHL_STATE_BATTERY_VOLTAGE_ERROR;
	}

	ESHL_Data[ESHL_DATA_CURRENT_LIMIT_IDX] = 1158;//运行电流限制 ADC 阈值（默认 1158，超出此值触发过流保护）
}


//关闭所有MOS管
static inline void MOS_CloseAll() {
	ESHL_AD_DISABLE();//关闭A相MOS管
	ESHL_AU_DISABLE();

	ESHL_BU_DISABLE();//关闭B相MOS管
	ESHL_BD_DISABLE();

	ESHL_CU_DISABLE();//关闭C相MOS管
	ESHL_CD_DISABLE();

	delay_us(20);
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



//检测MOS管是否短路
//返回问题MOS管编号,0为无异常,10为未知错误
//				 12为漏电流过大
//MOS管编号如下:
//  1(A+)    3(B+)    5(C+)
//  2(A-)    4(B-)    6(C-)
uint8_t MOS_SelfTest() {

    uint8_t error_mos_id = 0;   //异常MOS id
    uint8_t step = 0;           //电流检测步骤
    uint8_t flag = 1;           //电流检测标志

		//HAL_COMP_Stop_IT(&ESHL_COMP);//关闭比较器
		MOS_CloseAll();//关闭所有MOS管
		while (flag){
			switch (step) {

				case 0://测总漏电流
					for (uint8_t i = 0; i < ESHL_MOS_Current_Test_num; i++) {   //测总漏电流
                        BSP_ADC_GetMetrics(&g_adc_metrics);
						ESHL_LeakageCurrent_ADC_Value = (g_adc_metrics.phase_i > ESHL_LeakageCurrent_ADC_Value) ? g_adc_metrics.phase_i : ESHL_LeakageCurrent_ADC_Value;
                        delay_ms(1);
                    }
					if (ESHL_LeakageCurrent_ADC_Value >= 15) {//大于200ma触发
						ESHL_state = ESHL_STATE_CURRENT_ERROR;
						error_mos_id = 12;
						flag = 0;
					}else {
						step = 1;
					}
					break;
				case 1://测A相上管电流
					ESHL_AD_ENABLE();//打开A相上管
					for (uint8_t i = 0; i < ESHL_MOS_Current_Test_num; i++) {   //测A相上管电流
						BSP_ADC_GetMetrics(&g_adc_metrics);
                        ESHL_MOS_Current_ADC_Value = (g_adc_metrics.phase_i > ESHL_MOS_Current_ADC_Value) ? g_adc_metrics.phase_i : ESHL_MOS_Current_ADC_Value;
					}
					if (ESHL_MOS_Current_ADC_Value > ESHL_MOS_Current_ADC_MAX) {
						MOS_CloseAll();//关闭所有MOS管
						error_mos_id = 1;
						ESHL_state = ESHL_STATE_MOS_ERROR;
						flag = 0;
					}
					else{
						MOS_CloseAll(); //关闭所有MOS管
						ESHL_MOS_Current_ADC_Value = 0;
						step = 2;
					}
					break;


				case 2://测A相下管电流
					ESHL_AU_ENABLE(ESHL_MOS_TestPWM);//打开A相下管
					for (uint8_t i = 0; i < ESHL_MOS_Current_Test_num; i++) {   //测A相下管电流
                        BSP_ADC_GetMetrics(&g_adc_metrics);
                        ESHL_MOS_Current_ADC_Value = (g_adc_metrics.phase_i > ESHL_MOS_Current_ADC_Value) ? g_adc_metrics.phase_i : ESHL_MOS_Current_ADC_Value;
                    }
                    if (ESHL_MOS_Current_ADC_Value > ESHL_MOS_Current_ADC_MAX) {
                        MOS_CloseAll();//关闭所有MOS管
                        error_mos_id = 1;
                        ESHL_state = ESHL_STATE_MOS_ERROR;
                        flag = 0;
                    }
                    else{
                        MOS_CloseAll(); //关闭所有MOS管
                        ESHL_MOS_Current_ADC_Value = 0;
                        step = 3;
                    }
                    break;

                case 3://测B相上管电流
                    ESHL_BD_ENABLE();//打开B相下管
                    for (uint8_t i = 0; i < ESHL_MOS_Current_Test_num; i++) {   //测B相上管电流
                        BSP_ADC_GetMetrics(&g_adc_metrics);
                        ESHL_MOS_Current_ADC_Value = (g_adc_metrics.phase_i > ESHL_MOS_Current_ADC_Value) ? g_adc_metrics.phase_i : ESHL_MOS_Current_ADC_Value;
                    }
                    if (ESHL_MOS_Current_ADC_Value > ESHL_MOS_Current_ADC_MAX) {
                        MOS_CloseAll();//关闭所有MOS管
                        error_mos_id = 3;
                        ESHL_state = ESHL_STATE_MOS_ERROR;
                        flag = 0;
                    }
                    else{
                        MOS_CloseAll(); //关闭所有MOS管
                        ESHL_MOS_Current_ADC_Value = 0;
                        step = 4;
                    }
                    break;

                case 4://测B相下管电流
                    ESHL_BU_ENABLE(ESHL_MOS_TestPWM);//打开B相上管
                    for (uint8_t i = 0; i < ESHL_MOS_Current_Test_num; i++) {   //测B相下管电流
                        BSP_ADC_GetMetrics(&g_adc_metrics);
                        ESHL_MOS_Current_ADC_Value = (g_adc_metrics.phase_i > ESHL_MOS_Current_ADC_Value) ? g_adc_metrics.phase_i : ESHL_MOS_Current_ADC_Value;
                    }
                    if (ESHL_MOS_Current_ADC_Value > ESHL_MOS_Current_ADC_MAX) {
                        MOS_CloseAll();//关闭所有MOS管
                        error_mos_id = 3;
                        ESHL_state = ESHL_STATE_MOS_ERROR;
                        flag = 0;
                    }
                    else{
                        MOS_CloseAll(); //关闭所有MOS管
                        ESHL_MOS_Current_ADC_Value = 0;
                        step = 5;
                    }
                    break;

                case 5://测C相上管电流
                    ESHL_CD_ENABLE();//打开C相下管
                    for (uint8_t i = 0; i < ESHL_MOS_Current_Test_num; i++) {   //测C相上管电流
                        BSP_ADC_GetMetrics(&g_adc_metrics);
                        ESHL_MOS_Current_ADC_Value = (g_adc_metrics.phase_i > ESHL_MOS_Current_ADC_Value) ? g_adc_metrics.phase_i : ESHL_MOS_Current_ADC_Value;
                    }
                    if (ESHL_MOS_Current_ADC_Value > ESHL_MOS_Current_ADC_MAX) {
                        MOS_CloseAll();//关闭所有MOS管
                        error_mos_id = 5;
                        ESHL_state = ESHL_STATE_MOS_ERROR;
                        flag = 0;
                    }
                    else{
                        MOS_CloseAll(); //关闭所有MOS管
                        ESHL_MOS_Current_ADC_Value = 0;
                        step = 6;
                    }
                    break;

                case 6://测C相下管电流
                    ESHL_CU_ENABLE(ESHL_MOS_TestPWM);//打开C相上管
                    for (uint8_t i = 0; i < ESHL_MOS_Current_Test_num; i++) {   //测C相下管电流
                        BSP_ADC_GetMetrics(&g_adc_metrics);
                        ESHL_MOS_Current_ADC_Value = (g_adc_metrics.phase_i > ESHL_MOS_Current_ADC_Value) ? g_adc_metrics.phase_i : ESHL_MOS_Current_ADC_Value;
                    }
                    if (ESHL_MOS_Current_ADC_Value > ESHL_MOS_Current_ADC_MAX) {
                        MOS_CloseAll();//关闭所有MOS管
                        error_mos_id = 5;
                        ESHL_state = ESHL_STATE_MOS_ERROR;
                        flag = 0;
                    }
                    else{
                        MOS_CloseAll(); //关闭所有MOS管
                        ESHL_MOS_Current_ADC_Value = 0;
                        flag = 0;
                    }
                    break;
				default:
					MOS_CloseAll();
					error_mos_id = 10;
					break;
			}
		}
	return error_mos_id;
}

//电机发声函数
void ESHL_Beep(ESHL_BEEP_ENUM_T beep)
{
	if (ESHL_state == ESHL_STATE_OFF) {
		ESHL_step = 0;

		switch (beep) {
			case ESHL_BEEP_SHORT:
				for (uint16_t i = 0; i < 250; i++) {
					ESHL_step = 0;
					ESHL_U_D_Ctrl(100);
					delay_us( 20);
					ESHL_step = 1;
					ESHL_U_D_Ctrl(100);
					delay_us(180);
				}
				break;

			case ESHL_BEEP_LONG:
				for (uint16_t i = 0; i < 500; i++) {
					ESHL_step = 0;
					ESHL_U_D_Ctrl(100);
					delay_us(50);
					ESHL_step = 1;
					ESHL_U_D_Ctrl(100);
					delay_us(180);
				}
				break;

			default:
				break;
		}
		ESHL_U_D_Ctrl(0);
		MOS_CloseAll();
		delay_ms(20);
	}
}

//获取电调当前状态
ESHL_STATE_ENUM_T ESHL_GetState() {
	return ESHL_state;
}


//外部电调状态设置接口
void ESHL_SetState(ESHL_STATE_ENUM_T state) {
	ESHL_state = state;
}


//获取电调当前运行方向外部接口
ESHL_DIRECTION_ENUM_T ESHL_GetDirection() {
	return ESHL_direction;
}

//外部设置电调方向接口
void ESHL_SetDirection(ESHL_DIRECTION_ENUM_T direction) {
    ESHL_direction = (uint8_t)direction;
}

void BLDC_COMP2_TriggerCallback(void)
{
    uint8_t sense = 0;

    // 关闭全局中断
    __disable_irq();

    // 【新增】：硬件消隐防抖
    // 换相瞬间MOS管开关会产生巨大的电压毛刺，等几微秒再读比较器，避开尖峰
    delay_us(3); 

    // 采集当前比较器状态 (仅读一次！)
    if(SENSE_H) sense = 1; else sense = 0;

    switch(ESHL_step)
    {
    //=================================================================
    // 顺时针方向 (正转)
    //=================================================================
        case 0:
            if(sense) {
                ESHL_step++; ESHL_step %= 6;
                ESHL_U_D_Ctrl(ESHL_run_pwm);
                COMP2_SEL_PA5();       // 下一步检测C相 (PA5)
                COMP2_EXTI_FALLING();  // 配置为下降沿触发
            } else {
                ESHL_U_D_Ctrl(ESHL_run_pwm);
            }
            break;

        case 1:
            if(!sense) { // 期待过零点下降沿
                ESHL_step++; ESHL_step %= 6;
                ESHL_U_D_Ctrl(ESHL_run_pwm);
                COMP2_SEL_PA4();       // 下一步检测B相 (PA4)
                COMP2_EXTI_RISING();   // 配置为上升沿触发
            } else {
                ESHL_U_D_Ctrl(ESHL_run_pwm);
            }
            break;

        case 2:
            if(sense) {
                ESHL_step++; ESHL_step %= 6;
                ESHL_U_D_Ctrl(ESHL_run_pwm);
                COMP2_SEL_PA2();       // 下一步检测A相 (PA2)
                COMP2_EXTI_FALLING();
            } else {
                ESHL_U_D_Ctrl(ESHL_run_pwm);
            }
            break;

        case 3:
            if(!sense) {
                ESHL_step++; ESHL_step %= 6;
                ESHL_U_D_Ctrl(ESHL_run_pwm);
                COMP2_SEL_PA5();       
                COMP2_EXTI_RISING();
            } else {
                ESHL_U_D_Ctrl(ESHL_run_pwm);
            }
            break;

        case 4:
            if(sense) {
                ESHL_step++; ESHL_step %= 6;
                ESHL_U_D_Ctrl(ESHL_run_pwm);
                COMP2_SEL_PA4();       
                COMP2_EXTI_FALLING();
            } else {
                ESHL_U_D_Ctrl(ESHL_run_pwm);
            }
            break;

        case 5:
            if(!sense) {
                ESHL_step++; ESHL_step %= 6;
                ESHL_U_D_Ctrl(ESHL_run_pwm);
                COMP2_SEL_PA2();       
                COMP2_EXTI_RISING();
            } else {
                ESHL_U_D_Ctrl(ESHL_run_pwm);
            }
            break;

    //=================================================================
    // 逆时针方向 (反转) - 请参照上面的逻辑同步删除 do...while
    //=================================================================
        // ... (保留你原来的 case 6 到 11 的逻辑，但不要包在循环里) ...
        default:
            MOS_CloseAll();
            break;
    }

    // 【极其重要】：因为上面刚刚切换了通道和 EXTI 极性，
    // 硬件必定会立刻产生一个虚假的 EXTI 中断标志，必须在这里将其抹除！
    EXTI->PR = EXTI_PR_PR22;

    // 恢复全局中断
    __enable_irq();
}

//电调开环启动
void ESHL_Start(ESHL_DIRECTION_ENUM_T direction)
{
    static uint16_t time = 100;
	uint8_t BMEF_num = 0;//开环启动时累计过零事件数量
	ESHL_direction = direction;
	ESHL_run_pwm = ESHL_START_PWM;

    BSP_COMP2_Stop_IT();
	BSP_COMP2_Start();
	MOS_CloseAll();

	ESHL_state = ESHL_STATE_OPEN_LOOP_START;//电调状态更新为开环启动状态
	ESHL_step = (ESHL_direction == ESHL_CLOCKWISE) ? 0 : 6;

	ESHL_U_D_Ctrl(19);//转子定位
    for (uint8_t i = 0; i < 200; i++){
    	BSP_ADC_GetMetrics(&g_adc_metrics);
    	if (g_adc_metrics.phase_i >= ESHL_RotoCurrent_ADC_MAX) {
    		MOS_CloseAll();
    		BSP_COMP2_Stop();
    		ESHL_run_pwm = 0;
    		ESHL_state = ESHL_STATE_CURRENT_ERROR;//电调状态更新为电流异常
    		return ;
    	}
    	delay_ms(1);
    }
    while (1)
    {
        for (uint16_t i = 0; i < time; ++i) {
            delay_us(55);
        }
        if (time < 20)
        {
			MOS_CloseAll();
            time = 100;
        	if ((BMEF_num >= 18) && (BMEF_num <= 35))//开环启动成功
        	{
				LOG_INFO("BLDC_CLOSE_LOOP_START_SUCCESS\r\n");
				LOG_INFO("ESHL_step = %d\r\n", ESHL_step);
				LOG_INFO("BMEF_num = %d\r\n", BMEF_num);
                BSP_COMP2_Stop();
				
                BSP_COMP2_Start_IT();
        		ESHL_state = (ESHL_direction == ESHL_CLOCKWISE) ? ESHL_STATE_RUN_CLOCKWISE : ESHL_STATE_RUN_COUNTER_CLOCKWISE;//电调状态更新为对应方向运动状态
			}
        	else {			//开环启动失败
				LOG_INFO("BLDC_CLOSE_LOOP_START_Fail BMEF_num = %d\r\n", BMEF_num);
        		BSP_COMP2_Stop();
				BSP_COMP2_Stop_IT();
        		MOS_CloseAll();
        		ESHL_run_pwm = 0;
        		ESHL_state = ESHL_STATE_OPEN_LOOP_START_FAIL;//电调状态更新为开环启动失败
        	}
            break;
        }

    	BSP_ADC_GetMetrics(&g_adc_metrics);
    	if (g_adc_metrics.phase_i >= ESHL_OPEN_LOOP_Transition_Period_ADC_MAX) {
    		BSP_COMP2_Stop();
    		MOS_CloseAll();
    		ESHL_run_pwm = 0;
			LOG_DEBUG("BLDC_OPEN_LOOP_Transition_Period_Fail_i = %d\r\n", g_adc_metrics.phase_i);
    		ESHL_state = ESHL_STATE_CURRENT_ERROR;//电调状态更新为电流异常
    		return ;
    	}

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

//电调刹车
void ESHL_Break() {

	static  uint32_t BMEF_NUM = 0;//过零事件计数
	static uint32_t BMEF_NUM_Last = 0;//上一次过零事件计数
	static uint16_t Same_BMEF_NUM = 0;//过零事件计数相同的次数

	if (ESHL_state == ESHL_STATE_BRAKE) {
		ESHL_run_pwm = 0;
		MOS_CloseAll();
        BSP_COMP2_Start();

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
			BSP_COMP2_Stop();
			ESHL_state = ESHL_STATE_BRAKE_OK;//更新状态为刹车成功
			BMEF_NUM = 0;
			BMEF_NUM_Last = 0;
			Same_BMEF_NUM = 0;
			return;//退出函数
		}

#if ESHL_BREAK_MOD == 1//启用三相短路刹车
		ESHL_AU_ENABLE(ESHL_BREAK_PWM);
		ESHL_BU_ENABLE(ESHL_BREAK_PWM);
		ESHL_CU_ENABLE(ESHL_BREAK_PWM);

		for (uint8_t i = 0;i < 25; i++) {
			HAL_ADC_Start_DMA(&ESHL_Current_ADC, adc_val_buff, 2); //检测电调电流
			if (adc_val_buff[0] > ESHL_Data[1]) {//过流保护
				MOS_CloseAll();
				break;
			}
		}
#endif
	}
}


//电机运行时定时检测电流和电池电压
void ESHL_RuningCurrentVBATChack() {

	static uint32_t last_tick = 0;
	float current = 0;

#if ESHL_VBAT_CHACK_EN
		float Vbat = 0;
#endif
		if (BSP_GetTick() - last_tick >= ESHL_RUNING_CURRENT_VBAT_CHACK_TIMOUT) {
			for (uint8_t i = 0; i < 10; i++) {
				BSP_ADC_GetMetrics(&g_adc_metrics);
#if ESHL_VBAT_CHACK_EN
				Vbat += g_adc_metrics.v_bus;//获取电池电压
#endif
				current += g_adc_metrics.phase_i;//获取电流
			}
			last_tick = BSP_GetTick();
			current = current / 10;
			if (current >= (float)ESHL_Data[ESHL_DATA_CURRENT_LIMIT_IDX]) {
				//HAL_COMP_Stop_IT(&ESHL_COMP);
				//MOS_CloseAll();
				//ESHL_run_pwm = 0;
				last_tick = 0;
				ESHL_state = ESHL_STATE_CURRENT_ERROR;//电调状态更新为电流异常
			}
#if ESHL_VBAT_CHACK_EN
			Vbat = ESHL_VBAT_CalcVoltage((uint16_t)(Vbat / 10));
			switch (ESHL_BatType) {

				case ESHL_BAT_3S:
					if ((Vbat - 11.1f) <= ESHL_VBAT_LIMIT) {
						//HAL_COMP_Stop_IT(&ESHL_COMP);
						MOS_CloseAll();
						//ESHL_run_pwm = 0;
						last_tick = 0;
						ESHL_state = ESHL_STATE_BATTERY_VOLTAGE_ERROR;//电调状态更新为电池电压异常
					}
					break;

				case ESHL_BAT_4S:
					if ((Vbat - 14.8f) <= ESHL_VBAT_LIMIT) {
						//HAL_COMP_Stop_IT(&ESHL_COMP);
						MOS_CloseAll();
						//ESHL_run_pwm = 0;
						last_tick = 0;
						ESHL_state = ESHL_STATE_BATTERY_VOLTAGE_ERROR;//电调状态更新为电池电压异常
					}
					break;

				case ESHL_BAT_5S:
					if ((Vbat - 18.5f) <= ESHL_VBAT_LIMIT) {
						//HAL_COMP_Stop_IT(&ESHL_COMP);
						MOS_CloseAll();
						//ESHL_run_pwm = 0;
						last_tick = 0;
						ESHL_state = ESHL_STATE_BATTERY_VOLTAGE_ERROR;//电调状态更新为电池电压异常
					}
					break;

				case ESHL_BAT_6S:
					if ((Vbat - 22.2f) <= ESHL_VBAT_LIMIT) {
						//HAL_COMP_Stop_IT(&ESHL_COMP);
						MOS_CloseAll();
						//ESHL_run_pwm = 0;
						last_tick = 0;
						ESHL_state = ESHL_STATE_BATTERY_VOLTAGE_ERROR;//电调状态更新为电池电压异常
					}
					break;

				default:
					break;
			}
#endif
		}
}

// 电机运行过程中检测电机是否停转 (堵转检测)
void ESHL_RuningChack(void)
{
    static uint32_t last_tick = 0;
    // 使用 0xFF 作为初始无效值，防止恰好与 step 0 冲突
    static uint8_t last_step = 0xFF;

    // 仅在电机处于闭环运行状态时进行堵转检测
    if ((ESHL_state == ESHL_STATE_RUN_CLOCKWISE) || (ESHL_state == ESHL_STATE_RUN_COUNTER_CLOCKWISE))
    {
        // 正常情况：电机转动，换相步数发生变化
        if (ESHL_step != last_step)
        {
            last_step = ESHL_step;
            last_tick = get_tick(); // 更新系统时间戳
        }
        // 异常情况：步数没变，检查停留时间是否超时
        else if ((get_tick() - last_tick) >= ESHL_MOTOR_TIMEOUT)
        {
            // 确认为堵转！执行停机保护逻辑
            BSP_COMP2_Stop_IT();    // 纯寄存器版：关闭比较器及外部中断
            MOS_CloseAll();         // 关闭所有 MOS 管，切断输出

            ESHL_run_pwm = 0;

            // 重置静态变量，为下一次启动做准备
            last_tick = get_tick();
            last_step = 0xFF;

            // 更新状态机为堵转停机
            ESHL_state = ESHL_STATE_MOTOR_RUNING_STOP;

            LOG_DEBUG("Motor Stalled! Step stuck at: %d\r\n", ESHL_step);
        }
    }
    else
    {
        // 【核心修复】：当电机处于待机、刹车或开环启动阶段时
        // 必须持续刷新 last_tick 和 last_step
        // 这样可以保证当状态机刚刚切入 RUN (闭环) 的第一瞬间，
        // 不会因为残留的历史时间差而触发误报警。
        last_tick = get_tick();
        last_step = 0xFF;
    }
}

//关闭所有MOS管并且关闭比较器
void ESHL_CloseMOSComp() {

	ESHL_run_pwm = 0;
	BSP_COMP2_Stop();
	BSP_COMP2_Stop_IT();
	MOS_CloseAll();
}

//设置电调运行PWM值
void ESHL_SET_PWM(uint16_t pwm) {
	if (ESHL_state == ESHL_STATE_RUN_CLOCKWISE || ESHL_state == ESHL_STATE_RUN_COUNTER_CLOCKWISE) {
		pwm = (pwm <= ESHL_RUN_MAX_PWM) ? pwm : ESHL_RUN_MAX_PWM;//限制最大值
		pwm = (pwm >= ESHL_RUN_MIN_PWM) ? pwm : ESHL_RUN_MIN_PWM;//限制最小值

		if (pwm > ESHL_run_pwm) {
			ESHL_run_pwm += ESHL_RUN_PWM_STEP;
			ESHL_run_pwm = (ESHL_run_pwm <= 991) ? ESHL_run_pwm : 991;
		}
		else if (pwm < ESHL_run_pwm) {
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
