#include ".\Hardware\Base_Time\Base_Time.h"

TIM_HandleTypeDef htim;

static uint8_t current_p = 0, i = 0;

void BaseTime_Init(uint32_t Arr, uint32_t Prc)
{
    htim.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    htim.Init.Period = Arr;
    htim.Init.Prescaler = Prc;
    htim.Instance = TIM7;
    HAL_TIM_Base_Init(&htim);
    HAL_TIM_Base_Start_IT(&htim);
}
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM7)
    {
        __HAL_RCC_TIM7_CLK_ENABLE();
        HAL_NVIC_SetPriority(TIM7_IRQn, 2, 2);
        HAL_NVIC_EnableIRQ(TIM7_IRQn);
    }
}

void TIM7_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (i ==0)
    {
        st7789_draw_string(50, 10, " HD LOVE JJY", 0xFFE0, 0x0000, 2);
        st7789_draw_string(10, 35, "Wife is JJY", 0xFFFF, 0x0000, 2);
        st7789_draw_string(10, 60, "Husband is HD", 0xFFFF, 0x0000, 2);
        st7789_draw_progress_bar_border(10, 140, 220, 20, 0x001F);
    }
    st7789_update_progress_bar(10, 140, 220, 20, current_p, i, 0x001F, 0x0000);
    st7789_show_int(180, 120, i, 3, 0xFFFF, 0x0000, 2);
    st7789_draw_char(220, 120, '%', 0xFFFF, 0x0000, 2);
    if (i <= 100)
    {
        current_p = i;
        i++;
    }
    if (i == 101)
    {
		st7789_draw_string(10, 120, "Here comes JJY", 0xF8B2, 0x0000, 2);
        st7789_draw_bitmap565(0, 160, 240, 159, gImage_56);
        HAL_NVIC_DisableIRQ(TIM7_IRQn);
		st7789_draw_string(10,85,"HALTICK:",0xFFE0, 0x0000, 2);
		st7789_show_int(120,85,HAL_GetTick(),6,0xFFFF, 0x0000, 2);
    }
}
