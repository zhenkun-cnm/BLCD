\
#ifndef __BLDC_DRIVER_H__
#define __BLDC_DRIVER_H__

#include "stdint.h"
#include "stm32f051x8.h"
#include "bldc_pwm.h"
#include "bldc_gpio.h"
#include "bldc_adc.h"
#include "bldc_time.h"

//#define ESHL_COMP hcomp2			//电调使用的比较器

//#define ESHL_Current_ADC   hadc    //MOS电流计使用的ADC

#define ESHL_US_TIM TIM6			//电调us定时器

#define ESHL_A_TIM TIM3			//电调A相定时器
#define ESHL_A_TIM_CH PWM_Channel1	//电调A相定时器通道

#define ESHL_B_TIM TIM2			//电调B相定时器
#define ESHL_B_TIM_CH PWM_Channel2	//电调B相定时器通道

#define ESHL_C_TIM TIM2			//电调C相定时器
#define ESHL_C_TIM_CH PWM_Channel1	//电调A相定时器通道

typedef enum ESHL_BAT_ENUM {
	ESHL_BAT_3S,		//3S锂电池
	ESHL_BAT_4S,		//4S锂电池
	ESHL_BAT_5S,		//5S锂电池
	ESHL_BAT_6S,		//6S锂电池
}ESHL_BAT_ENUM_T;//电调电池枚举

typedef enum ESHL_BEEP_ENUM
{
	ESHL_BEEP_SHORT,
	ESHL_BEEP_LONG,
//	ESHL_BEEP_DOU,
//	ESHL_BEEP_RAI,
//	ESHL_BEEP_MI,
//	ESHL_BEEP_FA,
//	ESHL_BEEP_SOU,
//	ESHL_BEEP_LA,
//	ESHL_BEEP_SI,
//	ESHL_BEEP_DOUi,
}ESHL_BEEP_ENUM_T;//电调声音枚举

//电调A相上管配置
#define ESHL_AU_START()      BSP_PWM_Start(ESHL_A_TIM,ESHL_A_TIM_CH)
#define ESHL_AU_ENABLE(pwm)  BSP_PWM_SetDuty(ESHL_A_TIM,ESHL_A_TIM_CH,pwm)
#define ESHL_AU_DISABLE()    BSP_PWM_SetDuty(ESHL_A_TIM,ESHL_A_TIM_CH,0)
//电调A相下管配置

#define ESHL_AD_DISABLE()	 BSP_GPIO_WriteLow(BLDC_PIN_AD)
#define ESHL_AD_ENABLE()     BSP_GPIO_WriteHigh(BLDC_PIN_AD)

//电调B相上管配置
#define ESHL_BU_START()		 BSP_PWM_Start(ESHL_B_TIM,ESHL_B_TIM_CH)
#define ESHL_BU_ENABLE(pwm)	 BSP_PWM_SetDuty(ESHL_B_TIM,ESHL_B_TIM_CH,pwm)
#define ESHL_BU_DISABLE()	 BSP_PWM_SetDuty(ESHL_B_TIM,ESHL_B_TIM_CH,0)
//电调B相下管配置
#define ESHL_BD_DISABLE()    BSP_GPIO_WriteLow(BLDC_PIN_BD)
#define ESHL_BD_ENABLE()     BSP_GPIO_WriteHigh(BLDC_PIN_BD)

//电调C相上管配置
#define ESHL_CU_START()      BSP_PWM_Start(ESHL_C_TIM,ESHL_C_TIM_CH)
#define ESHL_CU_ENABLE(pwm)  BSP_PWM_SetDuty(ESHL_C_TIM,ESHL_C_TIM_CH,pwm)
#define ESHL_CU_DISABLE() 	 BSP_PWM_SetDuty(ESHL_C_TIM,ESHL_C_TIM_CH,0)

//电调C相下管配置
#define ESHL_CD_DISABLE()     BSP_GPIO_WriteLow(BLDC_PIN_CD)
#define ESHL_CD_ENABLE()      BSP_GPIO_WriteHigh(BLDC_PIN_CD)

#define ESHL_ADC_CALIBRATION_START()			  BSP_ADC_Calibration_Start()

// 切换比较器负极输入通道 (对应 STM32F051 的 PA4, PA5, PA2)
#define COMP2_SEL_PA4()  do { COMP->CSR = (COMP->CSR & ~COMP_CSR_COMP2INSEL) | (COMP_CSR_COMP2INSEL_2); } while(0)
#define COMP2_SEL_PA5()  do { COMP->CSR = (COMP->CSR & ~COMP_CSR_COMP2INSEL) | (COMP_CSR_COMP2INSEL_2 | COMP_CSR_COMP2INSEL_0); } while(0)
#define COMP2_SEL_PA2()  do { COMP->CSR = (COMP->CSR & ~COMP_CSR_COMP2INSEL) | (COMP_CSR_COMP2INSEL_2 | COMP_CSR_COMP2INSEL_1); } while(0)

// 切换 EXTI 22 线的触发极性
#define COMP2_EXTI_FALLING() do { EXTI->RTSR &= ~EXTI_RTSR_TR22; EXTI->FTSR |= EXTI_FTSR_TR22; } while(0)
#define COMP2_EXTI_RISING()  do { EXTI->FTSR &= ~EXTI_FTSR_TR22; EXTI->RTSR |= EXTI_RTSR_TR22; } while(0)

#define SENSE_H  ((COMP->CSR & COMP_CSR_COMP2OUT) != 0)
#define SENSE_L  ((COMP->CSR & COMP_CSR_COMP2OUT) == 0)


typedef enum ESHL_STATE_ENUM {
    ESHL_STATE_OFF = 0,					//电调关闭
	EShl_STATE_READY,					//电调初始化完成,具备启动条件
	ESHL_STATE_START,					//启动电机
	ESHL_STATE_OPEN_LOOP_START,			//电调开环启动中
	ESHL_STATE_OPEN_LOOP_START_FAIL,	//电调开环启动失败
	ESHL_STATE_OPEN_LOOP_START_ERROR,	//电调开环启动重试超过最大次数,无法开环启动
	ESHL_STATE_RUN_CLOCKWISE,			//顺时针方向运动中
	ESHL_STATE_RUN_COUNTER_CLOCKWISE,	//逆时针方向运动中
	ESHL_STATE_MOTOR_RUNING_STOP,		//电机运行时停转
	ESHL_STATE_BRAKE,					//刹车中
	ESHL_STATE_BRAKE_OK,				//刹车完毕
	ESHL_STATE_MOS_ERROR,				//MOS异常
	ESHL_STATE_CURRENT_ERROR,			//电流异常
	ESHL_STATE_BATTERY_VOLTAGE_ERROR,	//电池电压异常
	ESHL_STATE_SET_ADDR,				//电调设置地址中
	ESHL_STATE_SET_ADDR_OK,				//电调地址设置成功
	ESHL_STATE_SET_HOST_TIMEOUT,		//主机离线
}ESHL_STATE_ENUM_T;//电调状态枚举

typedef  enum ESHL_DIRECTION_ENUM {
	ESHL_CLOCKWISE = 0,						//电调顺时针运动标记
	ESHL_COUNTER_CLOCKWISE = 1,				//电调逆时针运动标记
}ESHL_DIRECTION_ENUM_T;


#define ESHL_DEFALT_ADDR    0xEC00	//电调默认地址
#define ESHL_ADDR_FLASH_ADD	0x0800F800 //用于存放电调地址的STM32内部Flash地址
//991 80
#define ESHL_START_PWM    80	//电调启动PWM值
#define ESHL_RUN_PWM_STEP  8	//电调pwm步进值
#define ESHL_RUN_MAX_PWM  991	//电调运行最大PWM值
#define ESHL_RUN_MIN_PWM  80	//电调运行最低PWM值

#define ESHL_BREAK_MOD	  0		//电调刹车方式选择,0为滑动刹车,1为三相短路刹车
#define ESHL_BREAK_PWM	  100	//电调短路刹车时用的PWM值,默认100
#define ESHL_BREAK_OK_SAME_BMEF_NUM  500		//电机停转阈值,刹车时过零事件相同的次数超过此值视为电机停转

#define ESHL_VBAT_CHACK_EN 1	//是否开启电调运行电压检测标志0为关闭,1为开启,(注:目前的策略是电池电压过低直接停转,请慎重考虑是否开启)
#define ESHL_VBAT_LIMIT	     0.1f	//电调电池电压限制阈值(V float),当前电池电压与电池额定电压差值小于此值时,视为电池电压过低

/* VBAT 电压计算参数 */
#define ESHL_VBAT_ADC_REF          3.3f    /* ADC 参考电压 (V) */
#define ESHL_VBAT_ADC_RESOLUTION   4096.0f /* 12 位 ADC 分辨率 */
#define ESHL_VBAT_DIVIDER_RATIO    10.0f   /* 电阻分压比 (R1+R2)/R2 */
#define ESHL_VBAT_ADC_OFFSET       1.18f   /* ADC 采样偏移校正值 (V)，因器件离散性需补偿 */

/**
  * @brief  将 VBUS ADC 原始采样值转换为实际电池电压
  * @param  adc_raw  ADC 原始采样值 (0 ~ 4095)
  * @return 电池实际电压值 (V)
  * @note   转换公式: raw * Vref / 4096 * DIVIDER_RATIO + OFFSET
  */
static inline float ESHL_VBAT_CalcVoltage(uint16_t adc_raw)
{
    return (float)adc_raw * ESHL_VBAT_ADC_REF / ESHL_VBAT_ADC_RESOLUTION * ESHL_VBAT_DIVIDER_RATIO
           + ESHL_VBAT_ADC_OFFSET;
}


//注:ADC值计算公式为 ADC = 目标电流(A)*0.0005(采样电阻阻值Omega)*50(电流计放大倍数)*4096(12位ADC精度)/3.3(单片机参考电压)
//				即 ADC = 目标电流(A)*31.30
#define ESHL_MOS_TestPWM            				20      //MOS管测试时使用的PWM值
#define ESHL_MOS_Current_Test_num   				50      //MOS电流单路测试次数


#define ESHL_MOS_Current_ADC_MAX        			20       //MOS电流阈值,超过此值视为电流不正常
#define ESHL_RotoCurrent_ADC_MAX    				45		//电调转子定位电流阈值，超过此值视为电流不正常
#define ESHL_OPEN_LOOP_Transition_Period_ADC_MAX	140		//电调开环过渡期电流阈值，超过此值视为电流不正常
#define ESHL_RUN_Current_Limit_ADC_DEFALT			1158	//电调运行电流限制ADC值,默认1158


#define ESHL_MOTOR_TIMEOUT							250		//电机换向超时时间(ms),超过此值视为换向失败
#define ESHL_RUNING_CURRENT_VBAT_CHACK_TIMOUT		30		//电调运行时电流和电池电压检测时间间隔(ms)

#define ESHL_OPEN_LOOP_RESTART_MAX_NUM				5		//电调开环启动最大重试次数,超过此值视为无法开环启动
															//	(注:必须大于或等于2,乱填数据后果自负)

/* ESHL_Data[] 索引宏定义 */
#define ESHL_DATA_ADDR_IDX         0   /* 电调离线地址索引 */
#define ESHL_DATA_CURRENT_LIMIT_IDX 1  /* 电调运行电流限制ADC值索引 */

uint8_t MOS_SelfTest();
void ESHL_Beep(ESHL_BEEP_ENUM_T beep);
void ESHL_ESC_Init();
ESHL_STATE_ENUM_T ESHL_GetState();
ESHL_DIRECTION_ENUM_T ESHL_GetDirection();
void ESHL_CloseMOSComp();
void ESHL_SetState(ESHL_STATE_ENUM_T state);
void ESHL_RuningCurrentVBATChack() ;
void ESHL_Start(ESHL_DIRECTION_ENUM_T direction);
void ESHL_RuningChack(void) ;
void ESHL_SET_PWM(uint16_t pwm);
void ESHL_SetDirection(ESHL_DIRECTION_ENUM_T direction);
void ESHL_Break();

#endif
