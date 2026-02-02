#ifndef __UART_H
#define __UART_H

#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdio.h>

// 环形缓冲区大小
#define UART_RX_BUFFER_SIZE 512

// 帧协议定义
#define UART_FRAME_HEADER 'S' // 帧头  
#define UART_FRAME_TAIL 'E'   // 帧尾

// 函数声明
void UART_Init(uint32_t baudrate);
void UART_SendByte(uint8_t byte);
void UART_SendData(uint8_t *data, uint16_t length);
void UART_SendString(char *str);
uint8_t UART_ReadByte(void);
uint16_t UART_ReadData(uint8_t *buffer, uint16_t length);
uint8_t UART_IsDataAvailable(void);
uint16_t UART_GetAvailableDataCount(void);
void UART_FlushRxBuffer(void);
void UART_ResetFrameState(void);
HAL_StatusTypeDef UART_WaitForTransmission(uint32_t timeout);

#endif
