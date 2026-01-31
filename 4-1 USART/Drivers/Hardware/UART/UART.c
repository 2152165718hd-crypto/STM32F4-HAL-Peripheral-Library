#include "./Hardware/UART/UART.h"

UART_HandleTypeDef huart1;

// 环形缓冲区 - 只存储有效数据
uint8_t Rx_Cycle_buffer[UART_RX_BUFFER_SIZE] = {0};
volatile uint16_t Rx_Cycle_buffer_head = 0;
volatile uint16_t Rx_Cycle_buffer_tail = 0;
uint8_t Rx_buffer[1] = {0}; // 单字节接收缓冲区

// 帧结构体 - 作为第一层缓冲区
typedef struct
{
    uint8_t header;    // 帧头
    uint8_t length;    // 数据长度
    uint8_t data[255]; // 第一层缓冲区
    uint8_t tail;      // 帧尾
} UART_FrameTypeDef;

// 帧接收状态机
typedef enum
{
    UART_FRAME_STATE_IDLE,      // 等待帧头
    UART_FRAME_STATE_WAIT_LEN,  // 等待长度
    UART_FRAME_STATE_WAIT_DATA, // 等待数据
    UART_FRAME_STATE_WAIT_TAIL  // 等待帧尾
} UART_FrameStateTypeDef;

// 状态机变量
static volatile UART_FrameStateTypeDef frame_state = UART_FRAME_STATE_IDLE;
static UART_FrameTypeDef current_frame;
static volatile uint8_t frame_data_index = 0;

/**
 * @brief 初始化UART1
 * @param baudrate 波特率
 */
void UART_Init(uint32_t baudrate)
{
    // 使能UART时钟
    __HAL_RCC_USART1_CLK_ENABLE();

    huart1.Instance = USART1;
    huart1.Init.BaudRate = baudrate;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;

    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        while (1)
            ;
    }

    // 配置中断优先级
    HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);

    // 启动接收中断
    HAL_UART_Receive_IT(&huart1, Rx_buffer, 1);
}

/**
 * @brief UART MSP初始化
 */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (huart->Instance == USART1)
    {
        __HAL_RCC_GPIOA_CLK_ENABLE();

        GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
}

/**
 * @brief UART中断处理函数
 */
void USART1_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart1);
}

/**
 * @brief 将有效数据存入环形缓冲区
 * @param data 数据指针
 * @param length 数据长度
 */
static void UART_StoreValidData(uint8_t *data, uint8_t length)
{
    for (uint8_t i = 0; i < length; i++)
    {
        Rx_Cycle_buffer[Rx_Cycle_buffer_head] = data[i];
        Rx_Cycle_buffer_head = (Rx_Cycle_buffer_head + 1) % UART_RX_BUFFER_SIZE;

        // 如果缓冲区满，覆盖旧数据
        if (Rx_Cycle_buffer_head == Rx_Cycle_buffer_tail)
        {
            Rx_Cycle_buffer_tail = (Rx_Cycle_buffer_tail + 1) % UART_RX_BUFFER_SIZE;
        }
    }
}

/**
 * @brief UART接收完成回调 - 在中断中直接处理数据帧
 * @param huart UART句柄
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        uint8_t byte = Rx_buffer[0];

        switch (frame_state)
        {
        case UART_FRAME_STATE_IDLE:
            if (byte == UART_FRAME_HEADER)
            {
                current_frame.header = byte;
                frame_data_index = 0;
                frame_state = UART_FRAME_STATE_WAIT_LEN;
            }
            break;

        case UART_FRAME_STATE_WAIT_LEN:
            current_frame.length = byte;
            if (current_frame.length > 0 && current_frame.length <= 255)
            {
                frame_state = UART_FRAME_STATE_WAIT_DATA;
            }
            else
            {
                // 无效长度，重置状态
                frame_state = UART_FRAME_STATE_IDLE;
            }
            break;

        case UART_FRAME_STATE_WAIT_DATA:
            // 存入第一层缓冲区
            current_frame.data[frame_data_index++] = byte;
            if (frame_data_index >= current_frame.length)
            {
                frame_state = UART_FRAME_STATE_WAIT_TAIL;
            }
            break;

        case UART_FRAME_STATE_WAIT_TAIL:
            if (byte == UART_FRAME_TAIL)
            {
                current_frame.tail = byte;
                // 帧接收完成，将有效数据存入环形缓冲区
                UART_StoreValidData(current_frame.data, current_frame.length);
            }
            // 重置状态机
            frame_state = UART_FRAME_STATE_IDLE;
            frame_data_index = 0;
            break;

        default:
            frame_state = UART_FRAME_STATE_IDLE;
            frame_data_index = 0;
            break;
        }

        // 继续接收下一个字节
        HAL_UART_Receive_IT(&huart1, Rx_buffer, 1);
    }
}

/**
 * @brief 发送单个字节
 */
void UART_SendByte(uint8_t byte)
{
    HAL_UART_Transmit(&huart1, &byte, 1, HAL_MAX_DELAY);
}

/**
 * @brief 发送数据数组
 */
void UART_SendData(uint8_t *data, uint16_t length)
{
    HAL_UART_Transmit(&huart1, data, length, HAL_MAX_DELAY);
}

/**
 * @brief 发送字符串
 */
void UART_SendString(char *str)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
}

/**
 * @brief 从环形缓冲区读取一个字节
 * @return 读取的字节
 */
uint8_t UART_ReadByte(void)
{
    uint8_t data = 0;

    if (UART_IsDataAvailable())
    {
        data = Rx_Cycle_buffer[Rx_Cycle_buffer_tail];
        Rx_Cycle_buffer_tail = (Rx_Cycle_buffer_tail + 1) % UART_RX_BUFFER_SIZE;
    }

    return data;
}

/**
 * @brief 从环形缓冲区读取多个字节
 * @param buffer 目标缓冲区
 * @param length 要读取的长度
 * @return 实际读取的字节数
 */
uint16_t UART_ReadData(uint8_t *buffer, uint16_t length)
{
    uint16_t count = 0;

    while (count < length && UART_IsDataAvailable())
    {
        buffer[count++] = UART_ReadByte();
    }

    return count;
}

/**
 * @brief 检查是否有数据可用
 * @return 1: 有数据, 0: 无数据
 */
uint8_t UART_IsDataAvailable(void)
{
    return (Rx_Cycle_buffer_head != Rx_Cycle_buffer_tail);
}

/**
 * @brief 获取可读取的数据字节数
 * @return 可读取的字节数
 */
uint16_t UART_GetAvailableDataCount(void)
{
    if (Rx_Cycle_buffer_head >= Rx_Cycle_buffer_tail)
    {
        return (Rx_Cycle_buffer_head - Rx_Cycle_buffer_tail);
    }
    else
    {
        return (UART_RX_BUFFER_SIZE - Rx_Cycle_buffer_tail + Rx_Cycle_buffer_head);
    }
}

/**
 * @brief 清空接收缓冲区
 */
void UART_FlushRxBuffer(void)
{
    Rx_Cycle_buffer_head = 0;
    Rx_Cycle_buffer_tail = 0;
}

/**
 * @brief 重置帧接收状态机
 */
void UART_ResetFrameState(void)
{
    frame_state = UART_FRAME_STATE_IDLE;
    frame_data_index = 0;
}

/**
 * @brief 等待传输完成
 */
HAL_StatusTypeDef UART_WaitForTransmission(uint32_t timeout)
{
    uint32_t tickstart = HAL_GetTick();

    while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC) == RESET)
    {
        if ((HAL_GetTick() - tickstart) > timeout)
        {
            return HAL_TIMEOUT;
        }
    }

    return HAL_OK;
}

//printf重定向函数
int fputc(int ch, FILE *f)
{
    UART_SendByte((uint8_t)ch);
    return ch;
}

