#ifndef __GENERAL_TIMER_H__
#define __GENERAL_TIMER_H__


#include "stm32f4xx_hal.h" /* STM32F4xx HAL库(可根据MCU型号调整) */


void General_Timer_Init(uint32_t ARR, uint32_t PSC);
uint16_t Get_Timer_Count(void);

#endif
