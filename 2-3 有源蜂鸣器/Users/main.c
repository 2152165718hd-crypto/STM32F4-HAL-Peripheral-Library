#include "./SYSTEM/sys/sys.h"     // 系统相关配置头文件（时钟、中断等）
#include "./SYSTEM/usart/usart.h" // 串口相关头文件（本程序未实际使用，保留为工程通用包含）
#include "./SYSTEM/delay/delay.h" // 延时功能头文件（提供delay_us/delay_ms函数）
#include "./Hardware/LED/LED.h"
#include "./Hardware/KEY/Key.h"
#include ".\Hardware\Active_Buzzer\Active_Buzzer.h"

uint8_t Keynum;
int main(void)
{
    HAL_Init();                         /* 初始化HAL库（底层资源、SysTick定时器等） */
    sys_stm32_clock_init(336, 8, 2, 7); /* 配置系统时钟：PLL锁相环参数配置，最终系统主频为168MHz */
    delay_init(168);                    /* 初始化延时函数，参数为系统主频（168MHz），用于校准延时基准 */
    usart_init(115200);
    LED_Init();
    KEY_Init();
    Active_Buzzer_Init();
    printf("Hello，World");
    while (1)
    {
        Keynum = KEY_Scan();
        if (Keynum == 1)
        {
            LED_On();
            printf("LED ON!  Buzzer On!\n");
            printf("Key1 Pressed!\n");
            Buzzer_On();
        }
        else if (Keynum == 2)
        {
            LED_Off();
            printf("LED OFF!  Buzzer Off!\n");
            printf("Key2 Pressed!\n");
            Buzzer_Off();
        }
        else if (Keynum == 3)
        {
            LED_Toggle();
            printf("LED TOGGLE!\n");
            printf("Key3 Pressed!\n");
        }
    }
}
