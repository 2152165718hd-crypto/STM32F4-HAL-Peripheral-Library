#include "./Hardware/IWDG/IWDG.h"

IWDG_HandleTypeDef hiwdg;

typedef struct
{
	uint16_t prescaler_val; // 分频系数数值（4/8/16...）
	uint32_t prescaler_hal; // HAL库IWDG分频枚举宏（关键：用于配置hiwdg.Init.Prescaler）
	uint16_t reload;		// 计算后的重载值
} IWDG_TimeTypeDef;

// 初始化分频表：匹配HAL库枚举，顺序不变
IWDG_TimeTypeDef IWDG_Time[7] =
	{
		{4, IWDG_PRESCALER_4},
		{8, IWDG_PRESCALER_8},
		{16, IWDG_PRESCALER_16},
		{32, IWDG_PRESCALER_32},
		{64, IWDG_PRESCALER_64},
		{128, IWDG_PRESCALER_128},
		{256, IWDG_PRESCALER_256}};

void IWDG_Init(uint16_t xms)
{
	uint32_t MaxTime = 0;
	uint8_t i = 0;

	__HAL_RCC_LSI_ENABLE();
	// 参数检查
	if (xms < 1)
		xms = 1;

	// 查找合适的分频系数
	for (i = 0; i < 7; i++)
	{
		MaxTime = (IWDG_Time[i].prescaler_val * 4096 * 1000) / LSI_VALUE;
		if (MaxTime >= xms)
		{
			break;
		}
	}
	if (i == 7)
		i = 6; // 超出最大时间，选择最大分频

	// 计算重载值（改进精度和优先级）
	uint32_t temp_reload = (xms * LSI_VALUE) / (IWDG_Time[i].prescaler_val * 1000);

	if (temp_reload > 0)
	{
		IWDG_Time[i].reload = temp_reload - 1;
	}
	else
	{
		IWDG_Time[i].reload = 0;
	}

	// 限制最大值
	if (IWDG_Time[i].reload > 0x0FFF)
	{
		IWDG_Time[i].reload = 0x0FFF;
	}

	// 配置并启动IWDG
	hiwdg.Instance = IWDG;
	hiwdg.Init.Prescaler = IWDG_Time[i].prescaler_hal;
	hiwdg.Init.Reload = IWDG_Time[i].reload;

	HAL_IWDG_Init(&hiwdg);

	// 立即喂狗一次（重要！）
	HAL_IWDG_Refresh(&hiwdg);
}

void IWDG_Feed(void)
{
	HAL_IWDG_Refresh(&hiwdg); // 喂狗
}
