#include "main.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "bldc_clock.h"
#include "bldc_debug.h"
#include "bldc_led.h"
#include "bldc_gpio.h"
#include "bldc_pwm.h"
#include "bldc_time.h"
#include "bldc_adc.h"
#include "bldc_driver.h"
#include "bldc_comp.h"
#include "stm32f0xx.h"

/* USER CODE END Includes */
uint32_t testTick = 0;
uint8_t ESHL_OpenLoopRestartNum = 0;		//开环重启计数次数
static uint16_t g_target_pwm = 150;			//目标PWM速度值,可通过串口修改


int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* USER CODE BEGIN SysInit */
  SystemClock_Config();
  BSP_DEBUG_Init(115200u);
  BSP_SysTick_Init_1ms();
  BSP_LED_Init();
  BSP_GPIO_Init();
  BSP_PWM_Init();
  BSP_Time_Init();
  BSP_ADC_Init();
  BSP_ADC_Start();
  BSP_COMP2_Init();
  /* USER CODE END SysInit */

  /* USER CODE BEGIN 2 */
  LOG_INFO("BLDC firmware start\r\n");
  LOG_DEBUG("SystemCoreClock = %u Hz\r\n", SystemCoreClock);
  BSP_LED_SetStatus(LED_STATUS_READY, 0);
	/* USER CODE END 2 */
  uint32_t start_time = g_rx_data_len; /* RX数据长度时间基准 */
  ESHL_ESC_Init();
  uint8_t mos_error_id = MOS_SelfTest();		//MOS自检,返回错误代码
  if(mos_error_id)
  {
      while(1)
      {
          LOG_DEBUG("MOS error code: %d\r\n", mos_error_id);
          delay_ms(1000);
      }
  }
  else
  {
      LOG_DEBUG("MOS self test pass\r\n");
  }

  //   //----------蜂鸣器----------
  // ESHL_Beep(ESHL_BEEP_SHORT);			//E
  delay_ms(100);

  ESHL_Beep(ESHL_BEEP_SHORT);			//S
  ESHL_Beep(ESHL_BEEP_SHORT);
  ESHL_Beep(ESHL_BEEP_SHORT);
  delay_ms(100);

  ESHL_Beep(ESHL_BEEP_LONG);			//C
  ESHL_Beep(ESHL_BEEP_SHORT);
  ESHL_Beep(ESHL_BEEP_LONG);
  ESHL_Beep(ESHL_BEEP_SHORT);
  delay_ms(100);

  if (ESHL_GetState() == ESHL_STATE_OFF) {			//初始化无异常
    ESHL_SetState(EShl_STATE_READY);					//将ESC状态设置为准备就绪
    LOG_INFO("BLDC firmware ready\r\n");
  }

  while (1)
  {
    BSP_LED_Process();
    ESHL_RuningCurrentVBATChack();				//定时检测和电池电压
    BSP_DEBUG_ProcessRx();
	      /* ---- 串口命令派发 ---- */
    BSP_DEBUG_DispatchCmd(&g_target_pwm);
    //LOG_INFO("BATTERY VOLTAGE = %f\r\n", ESHL_VBAT_CalcVoltage(g_adc_metrics.v_bus));
    /* USER CODE BEGIN 3 */
    switch (ESHL_GetState()) {
      case ESHL_STATE_OFF:								//电机关闭
        ESHL_CloseMOSComp();					//关闭所有MOS管并且关闭比较器
        BSP_LED_SetStatus(LED_STATUS_ERR_MOS, 0);
        break;
      case EShl_STATE_READY:							//电机准备就绪
        BSP_LED_SetStatus(LED_STATUS_READY, 0);
        break;
      case ESHL_STATE_START:							//电机启动中
          BSP_LED_SetStatus(LED_STATUS_READY, 0);
          ESHL_Start(ESHL_GetDirection());
        break;
      case ESHL_STATE_OPEN_LOOP_START_FAIL:		//开环启动失败
        if (ESHL_OpenLoopRestartNum > ESHL_OPEN_LOOP_RESTART_MAX_NUM - 2) {
          ESHL_SetState(ESHL_STATE_OPEN_LOOP_START_ERROR);		//设置电机状态为无法启动
        }
        else{
          ESHL_OpenLoopRestartNum++;
          ESHL_Start(ESHL_GetDirection());
        }
        break;
      case ESHL_STATE_OPEN_LOOP_START_ERROR:		//开环启动多次尝试失败,无法正常启动
            ESHL_CloseMOSComp();					//关闭所有MOS管并且关闭比较器LED_STATUS_ERR_START_FAIL
          // BSP_LED_SetStatus(LED_STATUS_ERR_START_FAIL, 4);
          // delay_ms(200);
          // BSP_LED_SetStatus(LED_STATUS_ERR_START_FAIL, 0);
          // delay_ms(200);
          // ESHL_CommunicationSendCode(ESHL_PROTOCOL_CMD_ERROR,0XE2);	//发送错误代码
        break;
      case ESHL_STATE_RUN_CLOCKWISE:			//顺时针方向运动
        //ESHL_CloseMOSComp();					//关闭所有MOS管并且关闭比较器
          ESHL_OpenLoopRestartNum = 0;				//开环重启计数清零
          ESHL_RuningChack();       		//运行检测
          ESHL_SET_PWM(g_target_pwm);					//设置指定PWM值
          BSP_LED_SetStatus(LED_STATUS_READY, 0);
        break;

       case ESHL_STATE_RUN_COUNTER_CLOCKWISE:		//逆时针方向运动
        //ESHL_CloseMOSComp();					//关闭所有MOS管并且关闭比较器
          ESHL_OpenLoopRestartNum = 0;				//开环重启计数清零
          ESHL_RuningChack();       		//运行检测
          ESHL_SET_PWM(g_target_pwm);					//设置指定PWM值
          BSP_LED_SetStatus(LED_STATUS_READY, 0);
        break;

      case ESHL_STATE_MOTOR_RUNING_STOP:		//电机运行时停转
        // ESHL_CommunicationSendCode(ESHL_PROTOCOL_CMD_ERROR,0XE3);	//发送错误代码
        // ESHL_CommunicationStart();			//重新启动
        // ESHL_Start(ESHL_GetDirection());		//重新启动
        break;
      case ESHL_STATE_CURRENT_ERROR:				//电流异常
          // ESHL_CloseMOSComp();					//关闭所有MOS管并且关闭比较器
          // BSP_LED_SetStatus(LED_STATUS_ERR_CURRENT, 4);
          // delay_ms(200);
          // BSP_LED_SetStatus(LED_STATUS_ERR_CURRENT, 0);
          // delay_ms(200);
          // ESHL_CommunicationSendCode(ESHL_PROTOCOL_CMD_ERROR,0XE1);	//发送错误代码
        break;
      case ESHL_STATE_SET_HOST_TIMEOUT:			//主机超时
          // ESHL_CloseMOSComp();					//关闭所有MOS管并且关闭比较器
          // BSP_LED_SetStatus(LED_STATUS_ERR_OFFLINE, 4);
          // delay_ms(200);
          // BSP_LED_SetStatus(LED_STATUS_ERR_OFFLINE,0);
          // delay_ms(200);
        break;

      // case ESHL_STATE_SET_ADDR:					//设置节点地址
      //   ESHL_SetAddr(recv_str.addr_dat);
      //   WS2812_SetAll(&ESHL_StateLed,15,0,15);
      //   break;

      // case ESHL_STATE_SET_ADDR_OK:			//设置地址成功
      //   while (1) {
      //     for (uint8_t i = 0;i < (uint8_t)(ESHL_GetAddr() & 0xff);i++) {
      //       WS2812_SetAll(&ESHL_StateLed,15,0,15);
      //       delay_ms(200);
      //       WS2812_SetAll(&ESHL_StateLed,0,0,0);
      //       delay_ms(200);
      //       ESHL_CommunicationAddressSend();	//广播节点地址
      //     }
      //     delay_ms(800);
      //   }
      //   break;

      case ESHL_STATE_BATTERY_VOLTAGE_ERROR:		//电池电压异常
        // ESHL_CloseMOSComp();					//关闭所有MOS管并且关闭比较器
        // BSP_LED_SetStatus(LED_STATUS_ERR_VOLTAGE, 4);
        // delay_ms(200);
        // BSP_LED_SetStatus(LED_STATUS_ERR_VOLTAGE, 0);
        // delay_ms(200);
        // ESHL_CommunicationSendCode(ESHL_PROTOCOL_CMD_ERROR,0XE4);	//发送错误代码
        break;

      case ESHL_STATE_BRAKE:				//刹车中
          ESHL_Break();
        break;

      case ESHL_STATE_BRAKE_OK:			//刹车完成
        // ESHL_CommunicationSendCode(ESHL_PROTOCOL_CMD_BRAKE,0xA3);
        ESHL_SetState(EShl_STATE_READY);
        LOG_INFO("电机完成刹车，进入准备状态\r\n");
        break;

      default:
        break;
    }
    /* USER CODE END 3 */
  }
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
	BSP_LED_Tick_1ms();
  g_system_ticks++;
}
