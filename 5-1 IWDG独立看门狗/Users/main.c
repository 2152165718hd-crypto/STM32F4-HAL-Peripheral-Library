#include "./SYSTEM/sys/sys.h"     // 系统相关配置头文件（时钟、中断等）
#include "./SYSTEM/delay/delay.h" // 延时功能头文件（提供delay_us/delay_ms函数）
#include "./Hardware/LCD/LCD.h"
#include "./Hardware/UART/UART.h"
#include "./Hardware/LCD/LCD_Data.h"
#include "./Hardware/IWDG/IWDG.h"
int main(void)
{
    HAL_Init();                         /* 初始化HAL库（底层资源、SysTick定时器等） */
    sys_stm32_clock_init(336, 8, 2, 7); /* 配置系统时钟：PLL锁相环参数配置，最终系统主频为168MHz */
    delay_init(168);                    /* 初始化延时函数，参数为系统主频（168MHz），用于校准延时基准 */
    UART_Init(115200);
    st7789_init();

    IWDG_Init(1000);
    UART_SendString("Hello, UART!\r\n");

    while (1)
    {
        IWDG_Feed();
        UART_SendString("IWGD FEED!\r\n");
        st7789_draw_string(10, 10, "IWGD FEED!", ST7789_COLOR_GRED, ST7789_COLOR_BLACK, 2);
        delay_ms(500);
        st7789_draw_string(10, 10, "          ", ST7789_COLOR_WHITE, ST7789_COLOR_BLACK, 2);
        delay_ms(500);
    }
}
