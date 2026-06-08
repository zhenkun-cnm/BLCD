//
// Created by E_LJF on 25-6-7.
// Optimized: 加入查找表重构、软件PLL测速、相位超前补偿、抖动超时保护
//

#ifndef ESHL_DRIVER_H
#define ESHL_DRIVER_H

#include "main.h"

#define ESHL_COMP hcomp2			//电调使用的比较器

#define ESHL_Current_ADC   hadc    //MOS电流计使用的ADC

#define ESHL_US_TIM htim6			//电调us定时器

#define ESHL_A_TIM htim3			//电调A相定时器
#define ESHL_A_TIM_CH TIM_CHANNEL_1	//电调A相定时器通道

#define ESHL_B_TIM htim2			//电调B相定时器
#define ESHL_B_TIM_CH TIM_CHANNEL_2	//电调B相定时器通道

#define ESHL_C_TIM htim2			//电调C相定时器
#define ESHL_C_TIM_CH TIM_CHANNEL_1	//电调A相定时器通道

#define SENSE_L              (!(hcomp2.Instance->CSR & 0x4000))
#define SENSE_H              ((hcomp2.Instance->CSR & 0x4000))	//比较器输出值,通过寄存器获得

//电调A相上管配置
#define ESHL_AU_ENABLE(pwm)  __HAL_TIM_SetCompare(&ESHL_A_TIM,ESHL_A_TIM_CH,pwm)
#define ESHL_AU_DISABLE()    __HAL_TIM_SetCompare(&ESHL_A_TIM,ESHL_A_TIM_CH,0)
//电调A相下管配置
#define ESHL_AD_DISABLE()	 HAL_GPIO_WritePin(BLDC_AD_GPIO_Port,BLDC_AD_Pin,GPIO_PIN_RESET)
#define ESHL_AD_ENABLE()     HAL_GPIO_WritePin(BLDC_AD_GPIO_Port,BLDC_AD_Pin,GPIO_PIN_SET)

//电调B相上管配置
#define ESHL_BU_ENABLE(pwm)	 __HAL_TIM_SetCompare(&ESHL_B_TIM,ESHL_B_TIM_CH,pwm)
#define ESHL_BU_DISABLE()	 __HAL_TIM_SetCompare(&ESHL_B_TIM,ESHL_B_TIM_CH,0)
//电调B相下管配置
#define ESHL_BD_DISABLE()    HAL_GPIO_WritePin(BLDC_BD_GPIO_Port,BLDC_BD_Pin,GPIO_PIN_RESET)
#define ESHL_BD_ENABLE()     HAL_GPIO_WritePin(BLDC_BD_GPIO_Port,BLDC_BD_Pin,GPIO_PIN_SET)

//电调C相上管配置
#define ESHL_CU_ENABLE(pwm)  __HAL_TIM_SetCompare(&ESHL_C_TIM,ESHL_C_TIM_CH,pwm)
#define ESHL_CU_DISABLE() 	 __HAL_TIM_SetCompare(&ESHL_C_TIM,ESHL_C_TIM_CH,0)
//电调C相下管配置
#define ESHL_CD_DISABLE()    HAL_GPIO_WritePin(BLDC_CD_GPIO_Port,BLDC_CD_Pin,GPIO_PIN_RESET)
#define ESHL_CD_ENABLE()     HAL_GPIO_WritePin(BLDC_CD_GPIO_Port,BLDC_CD_Pin,GPIO_PIN_SET)


#define ESHL_DEFALT_ADDR    0xEC00	//电调默认地址
#define ESHL_ADDR_FLASH_ADD	0x0800F800 //用于存放电调地址的STM32内部Flash地址

#define ESHL_START_PWM     100	//电调启动PWM值
#define ESHL_RUN_PWM_STEP  8	//电调pwm步进值
#define ESHL_RUN_MAX_PWM  991	//电调运行最大PWM值
#define ESHL_RUN_MIN_PWM  80	//电调运行最低PWM值

#define ESHL_BREAK_MOD	  0		//电调刹车方式选择,0为滑动刹车,1为三相短路刹车
#define ESHL_BREAK_PWM	  100	//电调短路刹车时用的PWM值,默认100
#define ESHL_BREAK_OK_SAME_BMEF_NUM  500		//电机停转阈值,刹车时过零事件相同的次数超过此值视为电机停转

#define ESHL_VBAT_CHACK_EN 0	//是否开启电调运行电压检测标志0为关闭,1为开启,(注:目前的策略是电池电压过低直接停转,请慎重考虑是否开启)
#define ESHL_VBAT_LIMIT	   0.1f	//电调电池电压限制阈值(V float),当前电池电压与电池额定电压差值小于此值时,视为电池电压过低


//注:ADC值计算公式为 ADC = 目标电流(A)*0.0005(采样电阻阻值Ω)*50(电流计放大倍数)*4096(12位ADC精度)/3.3(单片机参考电压)
//				即 ADC = 目标电流(A)*31.30
#define ESHL_MOS_TestPWM            				20      //MOS管测试时使用的PWM值
#define ESHL_MOS_Current_Test_num   				50      //MOS电流单路测试次数


#define ESHL_MOS_Current_ADC_MAX        			50       //MOS电流阈值,超过此值视为电流不正常
#define ESHL_RotoCurrent_ADC_MAX    				50		//电调转子定位电流阈值，超过此值视为电流不正常
#define ESHL_OPEN_LOOP_Transition_Period_ADC_MAX	140		//电调开环过渡期电流阈值，超过此值视为电流不正常
#define ESHL_RUN_Current_Limit_ADC_DEFALT			1158	//电调运行电流限制ADC值,默认1158


#define ESHL_MOTOR_TIMEOUT							250		//电机换向超时时间(ms),超过此值视为换向失败
#define ESHL_RUNING_CURRENT_VBAT_CHACK_TIMOUT		30		//电调运行时电流和电池电压检测时间间隔(ms)

#define ESHL_OPEN_LOOP_RESTART_MAX_NUM				5		//电调开环启动最大重试次数,超过此值视为无法开环启动
															//	↑↑↑(注:必须大于或等于2,乱填数据后果自负)↑↑↑

/* ADC 模拟看门狗过流阈值，监控 PA0(IN0) 电流通道
 * 复用运行限流值 1158，对应约 37A */
#define ESHL_AWD_HIGH_THRESHOLD     ESHL_RUN_Current_Limit_ADC_DEFALT


/* ============================================================================
 * ★★★ 优化新增配置（OPT-NEW）★★★
 * ============================================================================ */

/* --- 抖动超时保护配置 ---
 * 原代码 do-while 抖动循环没有出口保护，若硬件长时间抖动会卡死中断
 * DEBOUNCE_MAX_LOOPS 限制最大循环次数，超过则强制退出并停机 */
#define ESHL_DEBOUNCE_MAX_LOOPS		200		//抖动循环最大次数,超过此值视为信号异常

/* --- 软件 PLL 测速配置 ---
 * 通过测量两次过零中断之间的时间间隔来估算转速
 * 使用 us 定时器(htim6)的计数值作为时间基准
 * 公式: 电气周期 T = 6 * 过零间隔
 *      转速 RPM = 60 * 10^6 / (T_us * 极对数) */
#define ESHL_MOTOR_POLE_PAIRS		7		//电机极对数,需要根据实际电机修改(常见外转子无刷:7/12/14)
#define ESHL_PLL_FILTER_ALPHA		3		//PLL 一阶低通滤波系数 (1~7,值越大滤波越强)
											//  新周期 = (旧周期 * (ALPHA-1) + 实测周期) / ALPHA

/* --- 相位超前补偿配置 ---
 * 原理: 标准 BLDC 应在反电动势过零后等 30° 电角度才换相 (最佳转矩点)
 *      高速时由于系统延迟,实际等待角度需小于 30° (即"超前")
 *
 * 实现:  delay_after_zc = (commutation_period >> 1) - advance_offset
 *                       └─ 30°延迟 ─┘   └─ 超前补偿 ─┘
 *
 * 配置: PHASE_ADVANCE_PERCENT 单位为百分比,表示在 30° 基础上超前多少
 *      例:  0 = 完整 30° 延迟 (低速最佳)
 *          25 = 超前 25%, 实际延迟 22.5° (中高速)
 *          50 = 超前 50%, 实际延迟 15° (高速)
 *          100 = 立即换相 (相当于原代码行为) */
#define ESHL_PHASE_ADV_ENABLE		0		//是否启用相位超前补偿 (0=关闭, 与原代码行为相同)
											//建议: 先关闭,等PLL稳定运行后再开启调试
#define ESHL_PHASE_ADV_LOW_RPM		3000	//低速阈值:此速度以下不补偿
#define ESHL_PHASE_ADV_HIGH_RPM		15000	//高速阈值:此速度以上用最大补偿
#define ESHL_PHASE_ADV_MAX_PCT		40		//最大超前百分比(0~99),数值越大超前越多
/* ============================================================================ */


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
	ESHL_STATE_SIGNAL_UNSTABLE,			//★OPT-NEW: 比较器信号异常(抖动超时)

}ESHL_STATE_ENUM_T;//电调状态枚举

typedef  enum ESHL_DIRECTION_ENUM {
	ESHL_CLOCKWISE = 0,						//电调顺时针运动标记
	ESHL_COUNTER_CLOCKWISE = 1,				//电调逆时针运动标记
}ESHL_DIRECTION_ENUM_T;


/* ============================================================================
 * ★★★ OPT-NEW: PLL 测速 & 诊断信息结构体
 * ============================================================================
 * 把测速、诊断信息集中放在一个结构体里,便于调试和导出 */
typedef struct ESHL_DIAG_T {
	/* PLL 测速相关 */
	volatile uint32_t last_zc_tick;			//上次过零事件时的计数 (us)
	volatile uint32_t commutation_period_us;	//换相周期(us),经过低通滤波
	volatile uint32_t rpm;					//当前估算转速(RPM)

	/* 故障诊断 */
	volatile uint32_t debounce_overrun_cnt;	//抖动超时计数
	volatile uint32_t max_debounce_loops;	//最大抖动循环次数(峰值记录)
	volatile uint32_t isr_call_count;		//中断进入次数(用于调试)
	volatile uint32_t overcurrent_cnt;		//AWD 过流触发次数
	volatile uint16_t last_overcurrent_adc;	//最近一次过流时的电流 ADC 值
	volatile uint16_t last_lowvbat_adc;		//最近一次低压触发时的电压 ADC 值
	volatile uint16_t current_avg_adc;		//当前电流最新值(供主循环读取)
} ESHL_DIAG_T;
/* ============================================================================ */



extern float    ESHL_InitVbat;			//初始化时测得的电池电压(V)，用于调试输出
extern uint32_t ESHL_InitAdcSum10;		//初始化时10次ADC读数之和(原始值,未经任何转换)
extern ESHL_DIAG_T ESHL_Diag;			//★OPT-NEW: 诊断信息

void delay_us(TIM_HandleTypeDef *usTIMER,uint16_t us);
void ESHL_ESC_Init();
void ESHL_Start(ESHL_DIRECTION_ENUM_T direction);
void ESHL_Beep(ESHL_BEEP_ENUM_T beep);
void ESHL_SET_PWM(uint16_t pwm);
uint8_t MOS_SelfTest();
void ESHL_RuningChack();
void ESHL_RuningCurrentVBATChack();
void ESHL_ChangeRuningCurrentLimit(uint16_t current);
void ESHL_SetState(ESHL_STATE_ENUM_T state);
ESHL_STATE_ENUM_T ESHL_GetState();
void ESHL_Break();
uint16_t ESHL_GetAddr();
void ESHL_SetAddr(uint16_t addr);
void ESHL_SetDirection(ESHL_DIRECTION_ENUM_T direction);
ESHL_DIRECTION_ENUM_T ESHL_GetDirection();
// void ESHL_OFF();
void ESHL_CloseMOSComp();
void ESHL_PrintSpeedAndAdcReport(void);

/* ★OPT-NEW: 新增 API */
uint32_t ESHL_GetSpeedRPM(void);			//获取当前转速(RPM)
uint32_t ESHL_GetCommutationPeriod(void);	//获取换相周期(us)
void ESHL_ResetDiag(void);					//复位诊断信息

#endif //ESHL_DRIVER_H