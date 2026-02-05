#include ".\Hardware\General_Timer\General_Timer.h"

TIM_HandleTypeDef htim;
TIM_ClockConfigTypeDef sClockSourceConfig;
static uint16_t number = 0;

void General_Timer_Init(uint32_t ARR, uint32_t PSC)
{
    htim.Instance = TIM2;                                        /* 选择TIM2定时器 */
    htim.Init.Prescaler = PSC;                                   /* 设置预分频值 */
    htim.Init.CounterMode = TIM_COUNTERMODE_UP;                  /* 向上计数模式 */
    htim.Init.Period = ARR;                                      /* 设置自动重装载值 */
    htim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;            /* 时钟分频设置为不分频 */
    htim.Init.RepetitionCounter = 0;                             /* 重复计数器设置为0（仅高级定时器适用） */
    htim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE; /* 自动重装载预装载使能 */
    HAL_TIM_Base_Init(&htim);

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_ETRMODE2; /* 外部时钟模式2 */
    sClockSourceConfig.ClockPolarity = TIM_CLOCKPOLARITY_NONINVERTED;
    sClockSourceConfig.ClockPrescaler = TIM_CLOCKPRESCALER_DIV1;
    sClockSourceConfig.ClockFilter = 0x02;

    HAL_TIM_ConfigClockSource(&htim, &sClockSourceConfig);
    HAL_TIM_Base_Start_IT(&htim); /* 启动TIM2定时器并使能中断 */
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        __HAL_RCC_GPIOA_CLK_ENABLE(); /* 使能GPIOA时钟 */
        GPIO_InitTypeDef GPIO_InitStruct = {0};
        GPIO_InitStruct.Pin = GPIO_PIN_0;            /* PA0引脚 */
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;      /* 复用推挽输出 */
        GPIO_InitStruct.Pull = GPIO_NOPULL;          /* 不使用上拉或下拉 */
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; /* 低速 */
        GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;   /* 复用为TIM2功能 */
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);      /* 初始化PA0引脚 */

        __HAL_RCC_TIM2_CLK_ENABLE();           /* 使能TIM2时钟 */
        HAL_NVIC_SetPriority(TIM2_IRQn, 2, 2); /* 设置TIM2中断优先级 */
        HAL_NVIC_EnableIRQ(TIM2_IRQn);         /* 使能TIM2中断 */
    }
}

void TIM2_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim); /* 调用HAL库的中断处理函数 */
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        number++;
    }
}

uint16_t Get_Timer_Count(void)
{
    return number;
}
