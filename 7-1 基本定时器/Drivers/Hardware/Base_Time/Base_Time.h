#ifndef __BASE_TIME_H__
#define __BASE_TIME_H__

#include "stm32f4xx_hal.h" /* STM32F4xx HAL库(可根据MCU型号调整) */
#include "./Hardware/LCD/LCD.h" /* LCD驱动接口头文件 */
#include "./Hardware/LCD/LCD_Data.h"
void BaseTime_Init(uint32_t Arr, uint32_t Prc);

#endif
