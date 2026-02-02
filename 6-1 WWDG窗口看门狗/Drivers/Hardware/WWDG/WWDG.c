#include "./Hardware/WWDG/WWDG.h"

typedef struct
{
    uint16_t prescaler_val; // 分频系数数值（4/8/16...）
    uint32_t prescaler_hal; // HAL库WWDG分频枚举宏（关键：用于配置hWwdg.Init.Prescaler）
    uint16_t counter;        // 计算后的重载值
    uint8_t window;
} WWDG_TimeTypeDef;

// 初始化分频表：匹配HAL库枚举，顺序不变
WWDG_TimeTypeDef WWDG_Time[4] =
    {
        {1, WWDG_PRESCALER_1},
        {2, WWDG_PRESCALER_2},
        {4, WWDG_PRESCALER_4},
        {8, WWDG_PRESCALER_8}
    };

WWDG_HandleTypeDef hwwdg;

void WWDG_Init(uint16_t timeout_us, uint8_t rate)
{
    uint8_t i = 0;
    uint64_t Max_timeout = 0;                    // 使用 uint64_t 防止溢出
    uint32_t pclk_freq = HAL_RCC_GetPCLK1Freq(); // 动态获取PCLK1频率，比宏定义更安全
    uint32_t calculated_counter = 0;

    // 1. 查找合适的分频系数
    for (i = 0; i < 4; i++)
    {
        // 公式：T = 4096 * 2^WDGTB * (T[5:0] + 1) / PCLK1
        // 最大计数值为 64 (0x7F - 0x40 + 1)
        // 使用 uint64_t 进行计算
        Max_timeout = (uint64_t)64 * 4096 * WWDG_Time[i].prescaler_val * 1000000;
        Max_timeout /= pclk_freq;

        if (Max_timeout >= timeout_us)
            break;
    }

    if (i == 4)
        i = 3; // 如果都无法满足，使用最大分频

    // 2. 计算 Counter (重载值)
    // Counter = (Timeout * PCLK1) / (4096 * Prescaler) + 64 - 1
    // 注意：这里需要再次使用 uint64_t 避免 timeout_us * pclk_freq 溢出
    uint64_t temp_clocks = (uint64_t)timeout_us * pclk_freq;
    uint32_t div_factor = 4096 * WWDG_Time[i].prescaler_val * 1000000;

    calculated_counter = (uint32_t)(temp_clocks / div_factor) + 64;

    // 3. 边界限制 (STM32 WWDG Counter 只有7位，最大127)
    if (calculated_counter > 127)
        calculated_counter = 127;
    if (calculated_counter < 64)
        calculated_counter = 64; // 理论上不应发生，除非timeout极短

    WWDG_Time[i].counter = (uint16_t)calculated_counter;

    // 4. 计算窗口值 Window
    // Window 必须大于 63 (0x3F) 且 小于等于 Counter
    uint16_t temp_window = (WWDG_Time[i].counter * rate) / 100;
    if (temp_window < 64)
        temp_window = 64; // 硬件限制
    if (temp_window > WWDG_Time[i].counter)
        temp_window = WWDG_Time[i].counter;

    WWDG_Time[i].window = (uint8_t)temp_window;

    // 5. HAL库初始化
    hwwdg.Instance = WWDG;
    hwwdg.Init.Counter = WWDG_Time[i].counter;
    hwwdg.Init.Prescaler = WWDG_Time[i].prescaler_hal;
    hwwdg.Init.Window = WWDG_Time[i].window;
    hwwdg.Init.EWIMode = WWDG_EWI_ENABLE;

    if (HAL_WWDG_Init(&hwwdg) == HAL_OK)
    {
        printf("WWDG Init OK: Prescaler=%d, Counter=%d, Window=%d\r\n",
               WWDG_Time[i].prescaler_val,
               hwwdg.Init.Counter,
               hwwdg.Init.Window);
    }
    else
    {
        printf("WWDG Init Failed!\r\n");
    }
}

void HAL_WWDG_MspInit(WWDG_HandleTypeDef *hwwdg)
{
    if (hwwdg ->Instance== WWDG)
    {
        __HAL_RCC_WWDG_CLK_ENABLE();
        HAL_NVIC_SetPriorityGrouping(2);
        HAL_NVIC_SetPriority(WWDG_IRQn, 2, 2);
        HAL_NVIC_EnableIRQ(WWDG_IRQn);
    }
}

void WWDG_IRQHandler(void)
{
    HAL_WWDG_IRQHandler(&hwwdg);
}

void HAL_WWDG_EarlyWakeupCallback(WWDG_HandleTypeDef *hwwdg)
{
    HAL_WWDG_Refresh(hwwdg);
}
