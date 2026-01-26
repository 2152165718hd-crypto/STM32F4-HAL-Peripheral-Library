#include "./Hardware/EXTI/EXTI.h"
#include "./SYSTEM/delay/delay.h"
#include "./Hardware/LED/LED.h"
#include "./Hardware/LCD/LCD.h"      /* LCD驱动头文件 */

void Exti_Init(void)
{
	__HAL_RCC_GPIOA_CLK_ENABLE();  							
	__HAL_RCC_GPIOE_CLK_ENABLE();							

    GPIO_InitTypeDef GPIO_InitStruct = {0};					
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;    
    GPIO_InitStruct.Pull = GPIO_PULLUP ;           
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;  
	GPIO_InitStruct.Pin = GPIO_PIN_8; 				
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);       
		
	GPIO_InitStruct.Pin = GPIO_PIN_0;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);    
	
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
	
	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_2);
	HAL_NVIC_SetPriority(EXTI0_IRQn,2,0);
	HAL_NVIC_SetPriority(EXTI0_IRQn,2,1);
}

void EXTI0_IRQHandler(void)
{
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}

void EXTI9_5_IRQHandler(void)
{
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_8);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin==GPIO_PIN_0)
	{
		delay_ms(20);
		LED_Off();
		st7789_draw_string(0,80,"LED OFF",ST7789_COLOR_WHITE, ST7789_COLOR_RED, 4);
	}
	
	if(GPIO_Pin==GPIO_PIN_8)
	{
		delay_ms(20);
		LED_On();
		st7789_draw_string(0,160,"LED On",ST7789_COLOR_WHITE, ST7789_COLOR_RED, 4);
	}
}

