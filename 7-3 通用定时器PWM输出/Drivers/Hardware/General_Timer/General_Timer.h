#ifndef __GENERAL_TIMER_H__
#define __GENERAL_TIMER_H__


#include "stm32f4xx_hal.h" /* STM32F4xx HAL库(可根据MCU型号调整) */

#define START_PWM_IT 0 // 启动PWM中断功能

void General_Timer_PWM_Init(uint8_t duty_cycle);
void Set_Duty_Cycle(uint8_t duty_cycle);
uint16_t Get_Duty_Cycle(void);


#endif
