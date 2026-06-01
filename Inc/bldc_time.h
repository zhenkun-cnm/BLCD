#ifndef __BLDC_TIME_H
#define __BLDC_TIME_H

#include <stdint.h>

/* 毫秒心跳计数器（由 SysTick_Handler 在内核中断中自动累加） */
extern volatile uint32_t g_system_ticks;

/* 公共驱动接口 */
void BSP_Time_Init(void);
void delay_ms(uint32_t ms);
void delay_us(uint32_t us);
uint32_t get_tick(void);

#endif /* __BLDC_TIME_H */

