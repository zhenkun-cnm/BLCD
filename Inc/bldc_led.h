#ifndef __BLDC_LED_H
#define __BLDC_LED_H

#include <stdint.h>

/* 电调指示灯状态枚举 */
typedef enum {
    LED_STATUS_INIT = 0,         /* 黄色常亮：正在初始化 */
    LED_STATUS_READY,            /* 绿色常亮：准备就绪，正常运行 */
    LED_STATUS_SET_ADDR,         /* 紫色常亮：正在设置地址 */
    LED_STATUS_ADDR_DONE,        /* 紫色闪烁：地址设置完毕，闪烁次数代表地址 */
    LED_STATUS_ERR_MOS,          /* 红色闪烁：MOS异常，闪烁次数代表错误代码 */
    LED_STATUS_ERR_CURRENT,      /* 橙色闪烁：电流异常 */
    LED_STATUS_ERR_OFFLINE,      /* 蓝色闪烁：主机离线，数据接收超时 */
    LED_STATUS_ERR_START_FAIL,   /* 白色闪烁：开环启动失败 */
    LED_STATUS_ERR_VOLTAGE,      /* 浅蓝闪烁：电池电压异常 */
    LED_STATUS_ALARM_ALT         /* 绿色与蓝色交替闪烁 */
} LED_Status_t;

/* 外部调用接口 */
void BSP_LED_Init(void);
void BSP_LED_SetStatus(LED_Status_t status, uint8_t code);
void BSP_LED_Tick_1ms(void);    /* 需在 1ms SysTick 中断或定时器中断中调用 */
void BSP_LED_Process(void);     /* 在 main 循环中调用 */

#endif /* __BLDC_LED_H */