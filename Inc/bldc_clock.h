#ifndef __BLDC_CLOCK_H__
#define __BLDC_CLOCK_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void SystemClock_Config(void);
void BSP_SysTick_Init_1ms(void);
uint32_t BSP_GetTick(void);

#ifdef __cplusplus
}
#endif

#endif /* __BLDC_CLOCK_H__ */
