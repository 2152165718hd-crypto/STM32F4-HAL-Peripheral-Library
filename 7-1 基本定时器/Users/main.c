#include "./SYSTEM/sys/sys.h"     // 系统相关配置头文件（时钟、中断等）
#include "./SYSTEM/usart/usart.h" // 串口相关头文件（本程序未实际使用，保留为工程通用包含）
#include "./SYSTEM/delay/delay.h" // 延时功能头文件（提供delay_us/delay_ms函数）
#include "./Hardware/LCD/LCD.h"
#include "./Hardware/LCD/LCD_Data.h"
#include ".\Hardware\Base_Time\Base_Time.h"

int main(void)
{
    HAL_Init();                         /* 初始化HAL库（底层资源、SysTick定时器等） */
    sys_stm32_clock_init(336, 8, 2, 7); /* 配置系统时钟：PLL锁相环参数配置，最终系统主频为168MHz */
    delay_init(168);                    /* 初始化延时函数，参数为系统主频（168MHz），用于校准延时基准 */
    usart_init(115200);
    printf("Hello，World");
    st7789_init();
    BaseTime_Init(500 - 1, 16800-1);
    while (1)
    {
    }
}
