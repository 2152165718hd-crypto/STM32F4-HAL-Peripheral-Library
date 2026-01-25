#include "./Hardware/LED/LED.h"
#include "stm32f4xx_hal.h"

void LED_Init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();  /* 使能GPIOB时钟 */

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_8; /* 选择引脚 */
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;    /* 推挽输出模式 */
    GPIO_InitStruct.Pull = GPIO_NOPULL;            /* 不使用上拉或下拉电阻 */
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;   /* 低速 */
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);       /* 初始化GPIOB */
}

void LED_Off(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET); /* 点亮LED */
}

void LED_On(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET); /* 熄灭LED */
}

void LED_Toggle(void)
{
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_8); /* 切换LED状态 */
}
