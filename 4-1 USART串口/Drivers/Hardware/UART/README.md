# UART 串口模块使用说明

## 概述

这是一个基于STM32F4 HAL库的完整UART串口通信模块，支持中断接收、环形缓冲区、帧协议解析等功能。

## 功能特性

- **初始化配置**: 支持自定义波特率、数据位、停止位等参数
- **中断接收**: 使用中断方式接收数据，避免轮询阻塞
- **环形缓冲区**: 256字节环形缓冲区，防止数据丢失
- **多种发送方式**: 支持发送字节、数据数组、字符串
- **帧协议支持**: 内置帧协议解析状态机，支持自定义帧格式
- **缓冲区管理**: 提供数据可用性检查、缓冲区清空等功能

## API 接口

### 初始化函数

```c
void UART_Init(uint32_t baudrate);
```

- 参数: baudrate - 波特率 (如 9600, 115200 等)
- 说明: 初始化UART1，配置为8位数据位、无校验、1位停止位

### 发送函数

```c
void UART_SendByte(uint8_t byte);           // 发送单个字节
void UART_SendData(uint8_t *data, uint16_t length);  // 发送数据数组
void UART_SendString(char *str);            // 发送字符串
```

### 接收函数

```c
uint8_t UART_ReadByte(void);                // 读取单个字节
uint16_t UART_ReadData(uint8_t *buffer, uint16_t length);  // 读取多个字节
uint8_t UART_IsDataAvailable(void);         // 检查是否有数据可用
void UART_FlushRxBuffer(void);              // 清空接收缓冲区
```

### 帧处理

```c
void UART_ProcessReceivedData(void);        // 处理接收到的数据帧（在主循环中调用）
void UART_FrameProcess(UART_FrameTypeDef *frame);  // 帧处理回调函数
```

## 使用示例

### 基本使用

```c
#include "./Hardware/UART/UART.h"

int main(void)
{
    // 系统初始化...
    HAL_Init();
    // 时钟初始化...

    // 初始化UART
    UART_Init(115200);

    // 发送欢迎信息
    UART_SendString("UART Module Initialized!\r\n");

    while (1)
    {
        // 检查是否有数据
        if (UART_IsDataAvailable())
        {
            uint8_t data = UART_ReadByte();
            // 处理接收到的数据
            UART_SendByte(data);  // 回显
        }

        // 处理帧数据（如果使用帧协议）
        UART_ProcessReceivedData();
    }
}
```

### 帧协议使用

模块支持简单的帧协议：

- 帧头: 0xAA
- 数据长度: 1字节
- 数据内容: 长度由数据长度字段决定
- 帧尾: 0x55

发送帧示例:

```c
uint8_t frame_data[] = {0xAA, 0x03, 0x01, 0x02, 0x03, 0x55};
UART_SendData(frame_data, sizeof(frame_data));
```

接收到的帧会在 `UART_FrameProcess` 函数中处理。

## 配置说明

### 缓冲区大小

```c
#define UART_RX_BUFFER_SIZE 256  // 可在UART.h中修改
```

### 帧协议格式

```c
#define UART_FRAME_HEADER 0xAA   // 帧头
#define UART_FRAME_TAIL   0x55   // 帧尾
```

## 注意事项

1. **中断优先级**: UART中断优先级设置为1，可根据需要调整
2. **超时处理**: 发送函数使用HAL_MAX_DELAY，可根据需要修改
3. **缓冲区溢出**: 环形缓冲区满时会覆盖旧数据
4. **线程安全**: 此实现不保证线程安全，如需多线程使用请添加互斥锁

## 硬件连接

- TX: PA9
- RX: PA10
- UART实例: USART1

如需使用其他UART实例或引脚，请修改 `UART_Init` 和 `HAL_UART_MspInit` 函数。
