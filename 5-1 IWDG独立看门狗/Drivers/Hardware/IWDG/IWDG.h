#ifndef __IWDG_H
#define __IWDG_H

#include "stm32f4xx_hal.h" // 根据你的芯片型号修改（f1/f4/f7等）


/* 函数声明 ----------------------------------------------------------*/
/**
 * @brief  初始化独立看门狗
 * @param  timeout_ms: 超时时间(毫秒)
 *         有效范围: 1ms ~ 26214ms (LSI=32kHz, 最大分频256, reload=4095)
 * @retval 0: 成功
 *         1: 参数超出范围
 *         2: 初始化失败
 */
void IWDG_Init(uint16_t xms);

/**
 * @brief  喂狗函数（重载计数器）
 * @param  无
 * @retval 无
 */
void IWDG_Feed(void);



#endif /* __IWDG_H */
