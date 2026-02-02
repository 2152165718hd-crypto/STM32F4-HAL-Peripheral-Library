#include ".\Hardware\Active_Buzzer\Active_Buzzer.h"

void Active_Buzzer_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE(); /* 使能GPIOA时钟 */

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_6;            /* 选择引脚 */
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;  /* 推挽输出模式 */
    GPIO_InitStruct.Pull = GPIO_NOPULL;          /* 不使用上拉或下拉电阻 */
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM; /* 低速 */
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);      /* 初始化GPIOA */
}

void Buzzer_On(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET); 
}

void Buzzer_Off(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET); 
}
