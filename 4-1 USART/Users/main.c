#include "./SYSTEM/sys/sys.h"     // 系统相关配置头文件（时钟、中断等）
#include "./SYSTEM/delay/delay.h" // 延时功能头文件（提供delay_us/delay_ms函数）
#include "./Hardware/LCD/LCD.h"
#include "./Hardware/UART/UART.h"
#include "./Hardware/LCD/LCD_Data.h"
int main(void)
{
    uint16_t len = 0,state=0;
    uint8_t buffer[512] = {0};
    HAL_Init();                         /* 初始化HAL库（底层资源、SysTick定时器等） */
    sys_stm32_clock_init(336, 8, 2, 7); /* 配置系统时钟：PLL锁相环参数配置，最终系统主频为168MHz */
    delay_init(168);                    /* 初始化延时函数，参数为系统主频（168MHz），用于校准延时基准 */
    UART_Init(115200);
    st7789_init();
    UART_SendString("Hello, UART!\r\n");
    printf("Printf redirected to UART successfully!\r\n");
    while (1)
    {
       state=UART_IsDataAvailable();
       if(state)
         {
            len=UART_GetAvailableDataCount();
            printf("Available Data Length: %d\r\n",len);
            UART_ReadData(buffer,len);
            printf("Received Data: \r\n");
            for(uint16_t i=0;i<len;i++)
            {
                printf("%c",buffer[i]);
            }
            printf("\r\n");
         }
    }
}
