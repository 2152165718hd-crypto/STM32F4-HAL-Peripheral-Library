/* 全局定时器句柄（用于 TIM4） */
#include ".\Hardware\General_Timer\General_Timer.h"
TIM_HandleTypeDef htim;

/* PWM 与呼吸更新参数 */
#define PWM_PSC (84 - 1)     /* TIM4 计数频率 1 MHz */
#define PWM_PERIOD (500 - 1) /* PWM 周期 500 us，对应频率 2 kHz */
#define BREATH_UPDATE_DIV 50 /* 每 50 个更新中断调整一次亮度(约 5 ms) */
#define BREATH_STEP 1        /* 占空比步进(%) */

/**
 * @brief  初始化 TIM4 为 PWM 输出（通道 3，PB8）
 * @param  duty_cycle: 初始占空比，取值范围 0 - 100（单位：%）
 * @note   本函数完成 TIM4 的基本配置、PWM 模式配置并启动通道 3 的 PWM 输出。
 * @retval 无
 */

void General_Timer_PWM_Init(uint8_t duty_cycle)
{
    htim.Instance = TIM4;
    htim.Init.Prescaler = PWM_PSC;
    htim.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim.Init.Period = PWM_PERIOD;
    htim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim.Init.RepetitionCounter = 0;
    htim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_PWM_Init(&htim);
    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = duty_cycle * PWM_PERIOD / 100; // 占空比
    sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim, &sConfigOC, TIM_CHANNEL_3);
    __HAL_TIM_ENABLE_OCxPRELOAD(&htim, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim, TIM_CHANNEL_3);
#if START_PWM_IT
    HAL_TIM_Base_Start_IT(&htim);
#else
    HAL_TIM_Base_Start(&htim);
#endif
}

/**
 * @brief  PWM 硬件初始化回调（MSP）
 * @param  htim: 指向定时器句柄的指针
 * @note   该函数由 HAL 在初始化 TIM PWM 时调用，用于使能时钟并配置 GPIO 引脚复用。
 * @retval 无
 */

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
    __HAL_RCC_TIM4_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

#if START_PWM_IT
    HAL_NVIC_SetPriority(TIM4_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM4_IRQn);
#endif
}

/**
 * @brief  设置 PWM 占空比
 * @param  duty_cycle: 占空比（0 - 100）
 * @retval 无
 */

void Set_Duty_Cycle(uint8_t duty_cycle)
{
    __HAL_TIM_SET_COMPARE(&htim, TIM_CHANNEL_3, duty_cycle * PWM_PERIOD / 100);
}

/**
 * @brief  获取当前 PWM 占空比
 * @param  无
 * @retval 当前占空比（0 - 100）
 */

uint16_t Get_Duty_Cycle(void)
{
    uint32_t compare = __HAL_TIM_GET_COMPARE(&htim, TIM_CHANNEL_3);
    return compare * 100 / PWM_PERIOD;
}

void TIM4_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    static int8_t dir = 1;
    static uint8_t duty = 0;
    static uint16_t update_div = 0;
    if (htim->Instance == TIM4)
    {
        if (++update_div < BREATH_UPDATE_DIV)
        {
            return;
        }
        update_div = 0;
        if (dir > 0)
        {
            if (duty >= 100)
            {
                dir = -1;
                duty = 100 - BREATH_STEP;
            }
            else
                duty += BREATH_STEP;
        }
        else
        {
            if (duty == 0)
            {
                dir = 1;
                duty = BREATH_STEP;
            }
            else
                duty -= BREATH_STEP;
        }
        Set_Duty_Cycle(duty);
    }
}
