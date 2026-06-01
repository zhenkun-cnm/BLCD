#ifndef __BLDC_PWM_H
#define __BLDC_PWM_H

#include <stdint.h>
#include "stm32f0xx.h"

/* ---- 物理通道枚举定义 ---- */
typedef enum {
    PWM_Channel1 = 0,  /* TIM2 Channel 1 */
    PWM_Channel2,      /* TIM2 Channel 2 */
    PWM_Channel3,       /* TIM3 Channel 1 */
    PWM_Channel4
} PWM_Channel_t;

/* ---- 公共接口声明 ---- */
void BSP_PWM_Init(void);
void BSP_PWM_SetDuty(TIM_TypeDef *TIMx, PWM_Channel_t ch, uint16_t duty);
void BSP_PWM_Start(TIM_TypeDef *TIMx,PWM_Channel_t ch);
void BSP_PWM_Stop(TIM_TypeDef *TIMx,PWM_Channel_t ch);


#endif /* __BLDC_PWM_H */

