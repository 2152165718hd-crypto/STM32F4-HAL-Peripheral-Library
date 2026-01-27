/**
 * @file    EXTI.c
 * @brief   外部中断(EXTI)驱动程序
 * @details 用于初始化GPIO外部中断、设置中断优先级和处理中断回调
 * @author  STM32F4
 * @date    2026
 * @version 1.0
 */

#include "./Hardware/EXTI/EXTI.h"
#include "./SYSTEM/delay/delay.h"
#include "./Hardware/LED/LED.h"
#include "./Hardware/LCD/LCD.h" /* LCD驱动头文件 */

/**
 * @brief   外部中断初始化函数
 * @details 初始化GPIOE第8引脚和GPIOA第0引脚为外部中断输入，配置中断优先级
 * @param   void
 * @return  void
 */
void Exti_Init(void)
{
	/* 使能GPIOA和GPIOE时钟 */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();

	/* 配置GPIO初始化结构体 */
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING; /* 下降沿触发 */
	GPIO_InitStruct.Pull = GPIO_PULLUP;			 /* 上拉配置 */
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; /* 低速 */
	GPIO_InitStruct.Pin = GPIO_PIN_8;			 /* PE8引脚 */
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);		 /* 初始化GPIOE */

	/* 配置GPIOA第0引脚为外部中断 */
	GPIO_InitStruct.Pin = GPIO_PIN_0;		/* PA0引脚 */
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); /* 初始化GPIOA */

	/* 使能外部中断 */
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);	  /* 使能EXTI0中断 */
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn); /* 使能EXTI9_5中断 */

	/* 配置NVIC优先级分组和中断优先级 */
	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_2); /* 设置优先级分组 */
	HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 0);				/* 设置EXTI0优先级 */
	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 1);			/* 设置EXTI9_5优先级 */
}

/**
 * @brief   外部中断0处理函数
 * @details 处理EXTI0中断请求
 * @param   void
 * @return  void
 */
void EXTI0_IRQHandler(void)
{
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}

/**
 * @brief   外部中断9-5处理函数
 * @details 处理EXTI9_5中断请求
 * @param   void
 * @return  void
 */
void EXTI9_5_IRQHandler(void)
{
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_8);
}

/**
 * @brief   GPIO外部中断回调函数
 * @details 根据触发的GPIO引脚号进行相应的处理
 * @param   GPIO_Pin - 触发中断的GPIO引脚号
 * @return  void
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	/* 处理PA0引脚中断 - 关闭LED */
	if (GPIO_Pin == GPIO_PIN_0)
	{
		delay_ms(20); /* 消抖延迟 */
		LED_Off();	  /* 关闭LED */
		st7789_draw_string(0, 80, "LED OFF", ST7789_COLOR_WHITE, ST7789_COLOR_RED, 4);
	}

	/* 处理PE8引脚中断 - 打开LED */
	if (GPIO_Pin == GPIO_PIN_8)
	{
		delay_ms(20); /* 消抖延迟 */
		LED_On();	  /* 打开LED */
		st7789_draw_string(0, 160, "LED On", ST7789_COLOR_WHITE, ST7789_COLOR_RED, 4);
	}
}
