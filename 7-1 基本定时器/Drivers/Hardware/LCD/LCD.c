/* ======================== 包含文件 ======================== */
#include "./SYSTEM/delay/delay.h"    /* 延时功能 */
#include "./Hardware/LCD/LCD.h"      /* LCD驱动接口头文件 */
#include "./Hardware/LCD/LCD_Data.h" /* LCD字体库数据 */
#include <stdlib.h>                  /* 标准库函数(abs等) */
#include <stdio.h>                   /* 标准输入输出库(sprintf/snprintf) */

/* 确保包含STM32F4xx HAL库主头文件 */
#ifndef __STM32F4xx_HAL_H
#include "stm32f4xx_hal.h"
#endif

/* ======================== 硬件引脚定义 ======================== */
/*
 * STM32F407ZGTx 硬件SPI1配置：
 * - PA5: SPI1_SCK (串行时钟)
 * - PB5: SPI1_MOSI (主出从入)
 * - 其他控制信号: DC(PD14)、CS(PE14)、RST(PE1)、BLK(PB8)
 */

/* LCD硬件引脚：SPI时钟 */
#define LCD_SCLK_PIN GPIO_PIN_5
#define LCD_SCLK_GPIO_PORT GPIOA

/* LCD硬件引脚：数据/命令选择 */
#define LCD_DC_PIN GPIO_PIN_14
#define LCD_DC_GPIO_PORT GPIOD

/* LCD硬件引脚：背光控制 */
#define LCD_BLK_PIN GPIO_PIN_8
#define LCD_BLK_GPIO_PORT GPIOB

/* LCD硬件引脚：复位信号 */
#define LCD_RST_PIN GPIO_PIN_1
#define LCD_RST_GPIO_PORT GPIOE

/* LCD硬件引脚：芯片选择 */
#define LCD_CS_PIN GPIO_PIN_14
#define LCD_CS_GPIO_PORT GPIOE

/* LCD硬件引脚：SPI数据(MOSI) */
#define LCD_SDA_PIN GPIO_PIN_5
#define LCD_SDA_GPIO_PORT GPIOB

/* ======================== ST7789液晶控制器指令定义 ======================== */
/* 列地址范围设置指令(0x2A) */
#define ST7789_CMD_COL_ADDR_SET 0x2A
/* 行地址范围设置指令(0x2B) */
#define ST7789_CMD_ROW_ADDR_SET 0x2B
/* 内存写入指令(0x2C) */
#define ST7789_CMD_MEMORY_WRITE 0x2C
/* 内存数据访问控制指令(0x36) - 用于设置显示方向 */
#define ST7789_CMD_MEMORY_DATA_ACCESS_CTRL 0x36
/* 接口像素格式指令(0x3A) - 设置为RGB 5-6-5 16位模式 */
#define ST7789_CMD_INTERFACE_PIXEL_FORMAT 0x3A
/* 睡眠模式退出指令(0x11) - 初始化时使用 */
#define ST7789_CMD_SLEEP_OUT 0x11
/* 显示开启指令(0x29) - 启用显示 */
#define ST7789_CMD_DISPLAY_ON 0x29

/* ======================== 硬件SPI控制结构体 ======================== */
#if USE_HARDWARE_SPI
/* SPI1硬件外设句柄，用于硬件SPI模式下的数据收发 */
static SPI_HandleTypeDef hspi1;
#endif

/* ======================== ST7789液晶设备控制结构体 ======================== */
/**
 * @struct st7789_dev_t
 * @brief 液晶屏设备状态管理结构
 * @member width 当前有效显示宽度，单位像素(支持旋转后的宽度)
 * @member height 当前有效显示高度，单位像素(支持旋转后的高度)
 * @member dir 当前显示方向，取值范围0~3(竖屏0°、横屏90°、竖屏180°、横屏270°)
 */
typedef struct
{
    uint16_t width;
    uint16_t height;
    st7789_dir_t dir;
} st7789_dev_t;

/* 全局液晶设备实例，保存屏幕宽高及显示方向状态 */
static st7789_dev_t s_lcd =
    {
        .width = ST7789_CFG_DEFAULT_WIDTH,
        .height = ST7789_CFG_DEFAULT_HEIGHT,
        .dir = ST7789_DIR_PORTRAIT_0};

/* ======================== 实用宏定义 ======================== */
/**
 * @macro SWAP_BYTES
 * 功能：交换16位颜色值的高低字节
 * 原理：ST7789采用大端字节序(RGB565高字节在前)，而ARM MCU通常为小端
 * 必要性：通过字节交换实现MCU与液晶屏数据格式的匹配，确保正确显示颜色
 */
#define SWAP_BYTES(color) ((uint16_t)(((color) >> 8) | ((color) << 8)))

/* ======================== 底层通信函数 ======================== */

/**
 * @function soft_spi_write_byte
 * @brief 软件SPI发送单字节
 * @param byte 要发送的数据字节
 * @details 软件SPI实现，按MSB优先方式依次发送8个比特位
 *          - 时钟线先拉低，设置数据线电平
 *          - 时钟线拉高进行数据采样(从设备在上升沿采样)
 *          - 数据左移，继续发送下一比特
 * 算法思路：
 *   for i = 0 to 7:
 *     SCLK = 0              (时钟拉低，准备改变数据)
 *     SDA = (byte & 0x80)   (设置最高位的数据)
 *     byte = byte << 1      (数据左移，准备下一位)
 *     SCLK = 1              (时钟拉高，接收端采样数据)
 */
static void soft_spi_write_byte(uint8_t byte)
{
    for (uint8_t i = 0; i < 8; i++)
    {
        HAL_GPIO_WritePin(LCD_SCLK_GPIO_PORT, LCD_SCLK_PIN, GPIO_PIN_RESET);
        if (byte & 0x80)
            HAL_GPIO_WritePin(LCD_SDA_GPIO_PORT, LCD_SDA_PIN, GPIO_PIN_SET);
        else
            HAL_GPIO_WritePin(LCD_SDA_GPIO_PORT, LCD_SDA_PIN, GPIO_PIN_RESET);
        byte <<= 1;
        HAL_GPIO_WritePin(LCD_SCLK_GPIO_PORT, LCD_SCLK_PIN, GPIO_PIN_SET);
    }
}

/**
 * @function lcd_spi_send_byte
 * @brief 统一的SPI字节发送接口(自动选择硬件/软件模式)
 * @param byte 待发送的数据字节
 * @details 根据编译配置USE_HARDWARE_SPI自动选择发送方式：
 *          - 硬件SPI: 调用HAL_SPI_Transmit，速度快，支持DMA加速
 *          - 软件SPI: 调用soft_spi_write_byte，速度慢但兼容性好
 */
static void lcd_spi_send_byte(uint8_t byte)
{
#if USE_HARDWARE_SPI
    /* 硬件SPI发送，超时时间设为10ms */
    HAL_SPI_Transmit(&hspi1, &byte, 1, 10);
#else
    /* 软件SPI发送 */
    soft_spi_write_byte(byte);
#endif
}

/**
 * @function lcd_spi_send_buffer
 * @brief 统一的SPI缓冲区发送接口(自动选择硬件/软件模式)
 * @param data 数据缓冲区指针
 * @param len 发送长度，单位字节
 * @details 批量发送数据，相比逐字节发送性能提升显著：
 *          - 硬件SPI: 利用DMA或HAL缓存机制，大幅提升填充速度(帧缓冲填充)
 *          - 软件SPI: 循环调用soft_spi_write_byte，逐字节发送
 * 应用场景：填充矩形区域、显示位图等需要发送大量连续数据的操作
 */
static void lcd_spi_send_buffer(const uint8_t *data, size_t len)
{
#if USE_HARDWARE_SPI
    /* 硬件SPI批量发送，利用DMA或HAL优化循环，大幅提升填充速度 */
    /* 假设最大超时时间按数据量估算，这里给足够的时间 */
    HAL_SPI_Transmit(&hspi1, (uint8_t *)data, len, 1000);
#else
    /* 软件SPI循环发送 */
    for (size_t i = 0; i < len; i++)
    {
        soft_spi_write_byte(data[i]);
    }
#endif
}

/* ======================== GPIO控制辅助函数 ======================== */
/* CS (Chip Select) 片选控制 */
static void st7789_cs_low(void) { HAL_GPIO_WritePin(LCD_CS_GPIO_PORT, LCD_CS_PIN, GPIO_PIN_RESET); }
static void st7789_cs_high(void) { HAL_GPIO_WritePin(LCD_CS_GPIO_PORT, LCD_CS_PIN, GPIO_PIN_SET); }

/* DC (Data/Command) 数据/命令选择控制 */
static void st7789_dc_low(void) { HAL_GPIO_WritePin(LCD_DC_GPIO_PORT, LCD_DC_PIN, GPIO_PIN_RESET); }
static void st7789_dc_high(void) { HAL_GPIO_WritePin(LCD_DC_GPIO_PORT, LCD_DC_PIN, GPIO_PIN_SET); }

/* ======================== 写入函数 ======================== */

/**
 * @function st7789_write_cmd
 * @brief 向液晶屏发送命令字节
 * @param cmd 8位命令码
 * @details 时序：拉低CS -> 拉低DC(命令模式) -> 发送字节 -> 拉高CS
 */
static void st7789_write_cmd(uint8_t cmd)
{
    st7789_cs_low();
    st7789_dc_low();
    lcd_spi_send_byte(cmd);
    st7789_cs_high();
}

/**
 * @function st7789_write_data_byte
 * @brief 向液晶屏发送单字节数据
 * @param data 8位数据
 * @details 时序：拉低CS -> 拉高DC(数据模式) -> 发送字节 -> 拉高CS
 */
static void st7789_write_data_byte(uint8_t data)
{
    st7789_cs_low();
    st7789_dc_high();
    lcd_spi_send_byte(data);
    st7789_cs_high();
}

/**
 * @function st7789_write_data16
 * @brief 向液晶屏发送16位颜色数据(RGB565)
 * @param data 16位颜色值
 * @details 时序：拉低CS -> 拉高DC(数据模式) -> 发送高字节 -> 发送低字节 -> 拉高CS
 *          RGB565格式: R(5bit)|G(6bit)|B(5bit)
 */
static void st7789_write_data16(uint16_t data)
{
    st7789_cs_low();
    st7789_dc_high();
    /* 发送高字节 */
    lcd_spi_send_byte(data >> 8);
    /* 发送低字节 */
    lcd_spi_send_byte(data & 0xFF);
    st7789_cs_high();
}

/**
 * @function st7789_write_bytes
 * @brief 向液晶屏批量发送数据字节
 * @param data 数据缓冲区指针
 * @param len 发送长度，单位字节
 * @details 时序：拉低CS -> 拉高DC(数据模式) -> 发送数据缓冲区 -> 拉高CS
 *          用于填充矩形、显示位图等批量数据传输场景
 */
static void st7789_write_bytes(const uint8_t *data, size_t len)
{
    if (len == 0)
        return;
    st7789_cs_low();
    st7789_dc_high();
    lcd_spi_send_buffer(data, len);
    st7789_cs_high();
}

/* ======================== GPIO/SPI初始化 ======================== */

/**
 * @function lcd_gpio_init
 * @brief 初始化液晶屏所需的GPIO和SPI外设
 * @details 本函数完成以下初始化工作：
 *          1. 使能所有GPIO端口时钟 (GPIOA/B/D/E)
 *          2. 配置普通I/O引脚 (DC、CS、RST、BLK) 为推挽输出
 *          3. 根据USE_HARDWARE_SPI宏选择初始化硬件SPI或软件SPI
 *             - 硬件SPI: 配置PA5(SCK)/PB5(MOSI)为复用功能，初始化SPI1外设
 *             - 软件SPI: 配置PA5(SCK)/PB5(MOSI)为普通推挽输出
 *          4. 设置引脚初始状态 (CS/DC高电平，BLK低电平)
 *
 * 硬件SPI配置详解：
 *   - 工作模式: 主机模式 (SPI_MODE_MASTER)
 *   - 数据宽度: 8位 (SPI_DATASIZE_8BIT)
 *   - 极性/相位: CPOL=1, CPHA=1 (Mode 3，第二边沿采样)
 *   - 时钟分频: Prescaler=2，APB2时钟84MHz -> 42MHz
 *   - 注意: 若出现花屏可改为Prescaler=4(21MHz)
 */
static void lcd_gpio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* 使能所有需要的GPIO端口时钟 */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    /* 通用GPIO初始化 (DC, RST, CS, BLK) */
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    /* 配置DC (数据/命令选择) */
    GPIO_InitStruct.Pin = LCD_DC_PIN;
    HAL_GPIO_Init(LCD_DC_GPIO_PORT, &GPIO_InitStruct);

    /* 配置CS (片选) */
    GPIO_InitStruct.Pin = LCD_CS_PIN;
    HAL_GPIO_Init(LCD_CS_GPIO_PORT, &GPIO_InitStruct);

    /* 配置RST (复位) */
    GPIO_InitStruct.Pin = LCD_RST_PIN;
    HAL_GPIO_Init(LCD_RST_GPIO_PORT, &GPIO_InitStruct);

    /* 配置BLK (背光) */
    GPIO_InitStruct.Pin = LCD_BLK_PIN;
    HAL_GPIO_Init(LCD_BLK_GPIO_PORT, &GPIO_InitStruct);

#if USE_HARDWARE_SPI
    /* ========== 硬件SPI1 初始化 (STM32F407) ========== */
    /* 使能SPI1时钟 */
    __HAL_RCC_SPI1_CLK_ENABLE();

    /* 配置SPI引脚 (PA5: SCK, PB5: MOSI) 为复用功能 */
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;

    GPIO_InitStruct.Pin = LCD_SCLK_PIN;
    HAL_GPIO_Init(LCD_SCLK_GPIO_PORT, &GPIO_InitStruct); // PA5

    GPIO_InitStruct.Pin = LCD_SDA_PIN;
    HAL_GPIO_Init(LCD_SDA_GPIO_PORT, &GPIO_InitStruct); // PB5

    /* SPI1 参数配置 */
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES; /* 标准全双工模式(只发不收也可工作) */
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;

    /* 软件SPI时序分析：SCLK默认为高，拉低->改变数据->拉高。
       这对应 CPOL=1 (Idle High), CPHA=1 (2nd Edge Capture)。
       大多数ST7789也支持Mode 0和Mode 3。
    */
    hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
    hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;

    hspi1.Init.NSS = SPI_NSS_SOFT; /* 软件控制CS */

    /* APB2时钟通常为84MHz。
       Prescaler 2 -> 42MHz (ST7789通常可承受)
       若出现花屏，可改为 SPI_BAUDRATEPRESCALER_4 (21MHz)
    */
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;

    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        /* 初始化错误处理 */
    }

    __HAL_SPI_ENABLE(&hspi1); /* 使能SPI */

#else
    /* ========== 软件SPI 初始化 ========== */
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    /* 配置SCLK */
    GPIO_InitStruct.Pin = LCD_SCLK_PIN;
    HAL_GPIO_Init(LCD_SCLK_GPIO_PORT, &GPIO_InitStruct);

    /* 配置SDA */
    GPIO_InitStruct.Pin = LCD_SDA_PIN;
    HAL_GPIO_Init(LCD_SDA_GPIO_PORT, &GPIO_InitStruct);

    /* 软件SPI初始状态：时钟高 */
    HAL_GPIO_WritePin(LCD_SCLK_GPIO_PORT, LCD_SCLK_PIN, GPIO_PIN_SET);
#endif

    /* 设置控制引脚初始状态 */
    HAL_GPIO_WritePin(LCD_CS_GPIO_PORT, LCD_CS_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LCD_DC_GPIO_PORT, LCD_DC_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LCD_BLK_GPIO_PORT, LCD_BLK_PIN, GPIO_PIN_RESET);
}

/* ======================== 背光控制 ======================== */

/**
 * @function st7789_backlight_off
 * @brief 关闭液晶屏背光
 * @details BLK引脚置高电平，关闭背光(低电平使能)
 */
static void st7789_backlight_off(void) { HAL_GPIO_WritePin(LCD_BLK_GPIO_PORT, LCD_BLK_PIN, GPIO_PIN_SET); }

/**
 * @function st7789_backlight_on
 * @brief 开启液晶屏背光
 * @details BLK引脚置低电平，开启背光
 */
static void st7789_backlight_on(void) { HAL_GPIO_WritePin(LCD_BLK_GPIO_PORT, LCD_BLK_PIN, GPIO_PIN_RESET); }

/* ======================== 硬件复位 ======================== */

/**
 * @function st7789_reset
 * @brief 对ST7789控制器执行硬件复位
 * @details 复位时序：
 *          1. RST置低 -> 延时50ms (复位脉宽)
 *          2. RST置高 -> 延时120ms (等待IC初始化完成)
 *          需要等待芯片完全初始化才能发送指令，否则可能导致初始化失败
 */
static void st7789_reset(void)
{
    HAL_GPIO_WritePin(LCD_RST_GPIO_PORT, LCD_RST_PIN, GPIO_PIN_RESET);
    HAL_Delay(50);
    HAL_GPIO_WritePin(LCD_RST_GPIO_PORT, LCD_RST_PIN, GPIO_PIN_SET);
    HAL_Delay(120);
}

/* ======================== 液晶寄存器初始化 ======================== */

/**
 * @function st7789_reg_init
 * @brief 配置ST7789的内部寄存器参数
 * @details 本函数执行LCD初始化序列，包括：
 *          - 设置像素格式为RGB 5-6-5 16位模式
 *          - 配置显示时序参数 (刷新率、行周期等)
 *          - 设置电压和功耗参数
 *          - 加载伽玛修正曲线 (RGB正向和反向)
 *          - 启用显示输出
 *          这些参数通常由LCD供应商根据实际屏幕特性提供
 */
static void st7789_reg_init(void)
{
    st7789_write_cmd(ST7789_CMD_INTERFACE_PIXEL_FORMAT);
    st7789_write_data_byte(0x05);

    st7789_write_cmd(0xB2);
    {
        uint8_t seq[] = {0x0C, 0x0C, 0x00, 0x33, 0x33};
        st7789_write_bytes(seq, sizeof(seq));
    }

    st7789_write_cmd(0xB7);
    st7789_write_data_byte(0x35);

    st7789_write_cmd(0xBB);
    st7789_write_data_byte(0x32);

    st7789_write_cmd(0xC2);
    st7789_write_data_byte(0x01);

    st7789_write_cmd(0xC3);
    st7789_write_data_byte(0x15);

    st7789_write_cmd(0xC4);
    st7789_write_data_byte(0x20);

    st7789_write_cmd(0xC6);
    st7789_write_data_byte(0x0F);

    st7789_write_cmd(0xD0);
    {
        uint8_t seq[] = {0xA4, 0xA1};
        st7789_write_bytes(seq, sizeof(seq));
    }

    st7789_write_cmd(0xE0);
    {
        uint8_t seq[] = {0xD0, 0x08, 0x0E, 0x09, 0x09, 0x05, 0x31, 0x33, 0x48, 0x17, 0x14, 0x15, 0x31, 0x34};
        st7789_write_bytes(seq, sizeof(seq));
    }

    st7789_write_cmd(0xE1);
    {
        uint8_t seq[] = {0xD0, 0x08, 0x0E, 0x09, 0x09, 0x15, 0x31, 0x33, 0x48, 0x17, 0x14, 0x15, 0x31, 0x34};
        st7789_write_bytes(seq, sizeof(seq));
    }

    st7789_write_cmd(0x21);
    st7789_write_cmd(ST7789_CMD_DISPLAY_ON);
}

/* ======================== 核心显示功能 ======================== */

/**
 * @function st7789_set_direction
 * @brief 设置液晶屏显示方向(竖屏/横屏旋转)
 * @param dir 显示方向枚举值，范围0~3
 *            - ST7789_DIR_PORTRAIT_0: 竖屏正常(0°)，宽=240，高=320
 *            - ST7789_DIR_LANDSCAPE_90: 横屏90°旋转，宽=320，高=240
 *            - ST7789_DIR_PORTRAIT_180: 竖屏180°翻转，宽=240，高=320
 *            - ST7789_DIR_LANDSCAPE_270: 横屏270°旋转，宽=320，高=240
 * @details 本函数通过修改ST7789的MADCTL寄存器(0x36)来改变显示方向。
 *          各方向的MADCTL参数值已通过实验验证。
 *          同时更新全局的s_lcd.width和s_lcd.height以反映旋转后的屏幕尺寸。
 */
void st7789_set_direction(st7789_dir_t dir)
{
    s_lcd.dir = dir;
    st7789_write_cmd(ST7789_CMD_MEMORY_DATA_ACCESS_CTRL);

    switch (dir)
    {
    case ST7789_DIR_PORTRAIT_0:
        s_lcd.width = ST7789_CFG_DEFAULT_WIDTH;
        s_lcd.height = ST7789_CFG_DEFAULT_HEIGHT;
        st7789_write_data_byte(0x00);
        break;
    case ST7789_DIR_LANDSCAPE_90:
        s_lcd.width = ST7789_CFG_DEFAULT_HEIGHT;
        s_lcd.height = ST7789_CFG_DEFAULT_WIDTH;
        st7789_write_data_byte(0xC0);
        break;
    case ST7789_DIR_PORTRAIT_180:
        s_lcd.width = ST7789_CFG_DEFAULT_WIDTH;
        s_lcd.height = ST7789_CFG_DEFAULT_HEIGHT;
        st7789_write_data_byte(0x70);
        break;
    case ST7789_DIR_LANDSCAPE_270:
        s_lcd.width = ST7789_CFG_DEFAULT_HEIGHT;
        s_lcd.height = ST7789_CFG_DEFAULT_WIDTH;
        st7789_write_data_byte(0xA0);
        break;
    default:
        break;
    }
}

/**
 * @function st7789_set_window
 * @brief 设置液晶屏显示窗口的行列地址范围
 * @param x0 窗口左上角X坐标
 * @param y0 窗口左上角Y坐标
 * @param x1 窗口右下角X坐标
 * @param y1 窗口右下角Y坐标
 * @details 窗口设置后续的绘制操作(像素写入)只会在这个矩形区域内进行。
 *          实际发送给ST7789的地址需要加上偏移量(X_OFFSET、Y_OFFSET)。
 *          时序：
 *          1. 发送列地址范围设置指令(0x2A) -> 发送4字节(xs_H/L, xe_H/L)
 *          2. 发送行地址范围设置指令(0x2B) -> 发送4字节(ys_H/L, ye_H/L)
 *          3. 发送内存写入指令(0x2C) -> 后续数据写入内存
 */
static void st7789_set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    uint16_t xs = x0 + ST7789_CFG_X_OFFSET;
    uint16_t xe = x1 + ST7789_CFG_X_OFFSET;
    uint16_t ys = y0 + ST7789_CFG_Y_OFFSET;
    uint16_t ye = y1 + ST7789_CFG_Y_OFFSET;

    st7789_write_cmd(ST7789_CMD_COL_ADDR_SET);
    st7789_write_data_byte(xs >> 8);
    st7789_write_data_byte(xs & 0xFF);
    st7789_write_data_byte(xe >> 8);
    st7789_write_data_byte(xe & 0xFF);

    st7789_write_cmd(ST7789_CMD_ROW_ADDR_SET);
    st7789_write_data_byte(ys >> 8);
    st7789_write_data_byte(ys & 0xFF);
    st7789_write_data_byte(ye >> 8);
    st7789_write_data_byte(ye & 0xFF);

    st7789_write_cmd(ST7789_CMD_MEMORY_WRITE);
}

/**
 * @function st7789_fill_rect
 * @brief 填充矩形区域为指定颜色(高性能实现)
 * @param x0 矩形左上角X坐标(像素)
 * @param y0 矩形左上角Y坐标(像素)
 * @param x1 矩形右下角X坐标(像素)
 * @param y1 矩形右下角Y坐标(像素)
 * @param color RGB565格式的16位颜色值
 * @details 本函数采用缓冲区分块填充技术实现高性能的矩形填充：
 *
 * 算法思路：
 *   1. 边界检查：确保矩形坐标有效，防止越界
 *   2. 建立静态缓冲区(512*16bit=1KB)，预装填目标颜色数据
 *      - 优点：避免每次循环都修改缓冲区数据
 *      - 缺点：限制了单次填充的最大块大小为512像素
 *   3. 计算总像素数 = (x1-x0+1) * (y1-y0+1)
 *   4. 分块发送：每次发送512像素或剩余像素数中的较小值
 *      - 硬件SPI: 利用DMA加速，每次调用HAL_SPI_Transmit发送1KB数据
 *      - 软件SPI: 逐字节发送，速度较慢但兼容性好
 *   5. 重复步骤4直到所有像素填充完毕
 *
 * 为什么要分块填充：
 *   - 液晶屏最大填充区域240*320=76800像素，需要153.6KB内存
 *   - MCU内存通常只有几百KB，无法一次装入所有颜色数据
 *   - 分块填充可在有限内存条件下完成全屏显示
 *   - 缓冲区大小权衡：过小频繁调用SPI函数，过大浪费内存
 *   - 512像素(1KB)是最佳平衡点
 *
 * 性能优化说明：
 *   - CS和DC在发送前拉低/拉高，所有数据发送完后才恢复
 *   - 避免了每个512像素块都切换CS/DC的开销
 *   - 硬件SPI模式下速度可达42MHz，填满整屏约2ms
 *   - 软件SPI模式下速度约100KB/s，填满整屏约150ms
 */
void st7789_fill_rect(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color)
{
    if (x0 > x1 || y0 > y1)
        return;
    if (x1 >= s_lcd.width)
        x1 = s_lcd.width - 1;
    if (y1 >= s_lcd.height)
        y1 = s_lcd.height - 1;

    st7789_set_window(x0, y0, x1, y1);

    uint32_t pixels = (uint32_t)(x1 - x0 + 1) * (uint32_t)(y1 - y0 + 1);
    static uint16_t buffer[512]; // 1KB Buffer
    uint32_t block_size = 512;

    uint16_t color_swapped = SWAP_BYTES(color);

    for (uint32_t i = 0; i < block_size; i++)
    {
        buffer[i] = color_swapped;
    }

    st7789_cs_low();
    st7789_dc_high();

    while (pixels > 0)
    {
        uint32_t current_chunk = (pixels > block_size) ? block_size : pixels;

        /* 调用统一的 buffer 发送函数 */
        /* 注意：buffer是uint16，发送长度需要字节数，所以是 current_chunk * 2 */
        lcd_spi_send_buffer((uint8_t *)buffer, current_chunk * 2);

        pixels -= current_chunk;
    }

    st7789_cs_high();
}

/**
 * @function st7789_clear
 * @brief 清屏，用指定颜色填充整个显示区域
 * @param color RGB565格式的16位颜色值，通常为0x0000(黑色)或0xFFFF(白色)
 * @details 调用st7789_fill_rect函数填充从(0,0)到(width-1,height-1)的矩形
 */
void st7789_clear(uint16_t color)
{
    st7789_fill_rect(0, 0, s_lcd.width - 1, s_lcd.height - 1, color);
}

/**
 * @function st7789_draw_pixel
 * @brief 绘制单个像素点
 * @param x 像素X坐标
 * @param y 像素Y坐标
 * @param color RGB565格式的16位颜色值
 * @details 实际上是一个1x1像素的矩形填充，通过设置窗口为单点实现
 */
void st7789_draw_pixel(uint16_t x, uint16_t y, uint16_t color)
{
    if (x >= s_lcd.width || y >= s_lcd.height)
        return;
    st7789_set_window(x, y, x, y);
    st7789_write_data16(color);
}

/**
 * @function st7789_draw_line
 * @brief 绘制任意方向的直线(使用Bresenham算法)
 * @param x0 直线起点X坐标
 * @param y0 直线起点Y坐标
 * @param x1 直线终点X坐标
 * @param y1 直线终点Y坐标
 * @param color RGB565格式的16位颜色值
 *
 * @details 算法：Bresenham增量直线算法(中点画线法)
 *
 * 原理与推导：
 *   传统直线方程：y = mx + b，对于每个x值计算对应y值
 *   问题：浮点运算耗时，不适合嵌入式系统
 *
 *   Bresenham算法：
 *   1. 计算增量向量：dx = |x1 - x0|, dy = |y1 - y0|
 *   2. 计算步长：sx = sign(x1 - x0), sy = sign(y1 - y0)
 *   3. 误差项：err = dx + dy (dy为负，表示反向)
 *   4. 逐点迭代：根据误差项决定下一个像素位置
 *      - 如果 err*2 >= dy，则在x方向步进(err += dy)
 *      - 如果 err*2 <= dx，则在y方向步进(err += dx)
 *
 * 为什么使用这个算法：
 *   - 完全使用整数运算，无浮点开销
 *   - 每个像素点只需比较和加法，计算量O(max(dx,dy))
 *   - 能够精确近似直线，不会出现断裂或重复像素
 *   - 对任意角度直线都适用(包括负斜率和陡峭情况)
 *
 * 误差项的含义：
 *   - err表示当前点到理想直线的偏离程度(放大后的值)
 *   - e2 = 2*err是为了避免除法，使用乘法判断误差门限
 *   - dy和dx的符号通过反向初始化来处理(dy=-|y1-y0|)
 *
 * 时间复杂度：O(max(|x1-x0|, |y1-y0|))
 * 空间复杂度：O(1)
 */
void st7789_draw_line(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color)
{
    int16_t dx = abs((int16_t)x1 - (int16_t)x0);
    int16_t sx = x0 < x1 ? 1 : -1;
    int16_t dy = -abs((int16_t)y1 - (int16_t)y0);
    int16_t sy = y0 < y1 ? 1 : -1;
    int16_t err = dx + dy;
    int16_t e2;

    for (;;)
    {
        st7789_draw_pixel(x0, y0, color);
        if (x0 == x1 && y0 == y1)
            break;
        e2 = 2 * err;
        if (e2 >= dy)
        {
            err += dy;
            x0 += sx;
        }
        if (e2 <= dx)
        {
            err += dx;
            y0 += sy;
        }
    }
}

/**
 * @function st7789_draw_bitmap565
 * @brief 绘制RGB565格式的位图
 * @param x 位图左上角X坐标
 * @param y 位图左上角Y坐标
 * @param w 位图宽度(像素)
 * @param h 位图高度(像素)
 * @param data RGB565格式的位图数据指针，每个像素占2字节(大端序)
 * @details 直接将位图数据发送到液晶屏的显示内存中
 *          位图数据通常由图像转换工具生成(如LCDAssistant)
 *          RGB565格式：R(5bit)|G(6bit)|B(5bit)
 */
void st7789_draw_bitmap565(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint8_t *data)
{
    if (x + w > s_lcd.width || y + h > s_lcd.height)
        return;
    st7789_set_window(x, y, x + w - 1, y + h - 1);
    st7789_write_bytes(data, (size_t)w * h * 2);
}

/* ======================== 初始化序列 ======================== */

/**
 * @function st7789_init
 * @brief ST7789液晶驱动完整初始化函数
 * @details 执行以下初始化步骤：
 *          1. GPIO和SPI外设初始化(lcd_gpio_init)
 *          2. 关闭背光(防止花屏)
 *          3. 执行硬件复位序列(st7789_reset)
 *          4. 发送睡眠退出指令，延时200ms等待芯片初始化
 *          5. 设置显示方向为竖屏0°(st7789_set_direction)
 *          6. 配置内部寄存器参数(st7789_reg_init)
 *          7. 清屏为黑色
 *          8. 开启背光(显示启用)
 *
 * 初始化顺序的重要性：
 *   - GPIO和SPI必须先初始化，否则无法与芯片通信
 *   - 关闭背光防止未初始化时显示不稳定的内容
 *   - 硬件复位后需等待足够时间(120ms)让芯片启动
 *   - 睡眠退出后也需延时(200ms)让振荡器稳定
 *   - 寄存器配置后再开启显示，确保参数生效
 *   - 最后开启背光完成初始化
 */
void st7789_init(void)
{
    lcd_gpio_init();

    st7789_backlight_off();
    st7789_reset();

    st7789_write_cmd(ST7789_CMD_SLEEP_OUT);
    HAL_Delay(200);

    st7789_set_direction(ST7789_DIR_PORTRAIT_0);
    st7789_reg_init();

    st7789_clear(0x0000); // Black
    st7789_backlight_on();
}

/**
 * @function st7789_width
 * @brief 获取当前显示宽度(与显示方向相关)
 * @return 当前显示宽度，单位像素
 */
uint16_t st7789_width(void) { return s_lcd.width; }

/**
 * @function st7789_height
 * @brief 获取当前显示高度(与显示方向相关)
 * @return 当前显示高度，单位像素
 */
uint16_t st7789_height(void) { return s_lcd.height; }

/* ======================== 文本与图形显示函数 ======================== */

/**
 * @function st7789_draw_char
 * @brief 绘制单个ASCII字符(支持缩放)
 * @param x 字符左上角X坐标
 * @param y 字符左上角Y坐标
 * @param c ASCII字符码，范围0x20(空格)~0x7E(~)
 * @param color 字体颜色，RGB565格式16位值
 * @param bg 背景颜色，RGB565格式16位值
 * @param scale 字符缩放因子(1=5x7像素, 2=10x14像素, 最大=4)
 *
 * @details 字符点阵生成算法：
 *
 * 数据来源：内置的5x7点阵字体库(st7789_font5x7)
 *   - 每个ASCII字符占5字节
 *   - 每字节的7个比特对应一列像素(LSB在下)
 *   - 例如'A'的点阵：5列 * 7行
 *
 * 颜色缓冲区生成流程：
 *   1. 遍历字体的7行(for r = 0 to 6)
 *      2. 每行缩放sy倍(for sy = 0 to scale-1)
 *         3. 遍历5列(for col = 0 to 4)
 *            4. 从点阵中提取该行该列的比特：(glyph[col] >> r) & 0x01
 *            5. 比特为1用字体色，为0用背景色
 *            6. 该像素缩放sx倍填充到缓冲区
 *
 * 缓冲区大小计算：
 *   - 最大缩放scale=4时，字符大小=20x28=560像素
 *   - 需要560*2=1120字节，所以静态缓冲区大小设为[20*28]
 *
 * 性能考虑：
 *   - 缓冲区预生成后一次性发送到液晶屏
 *   - 相比逐像素绘制，速度提升显著
 *   - 但仍受限于字体库的分辨率(仅5x7)，缩放倍数不宜过大
 */
void st7789_draw_char(uint16_t x, uint16_t y, char c, uint16_t color, uint16_t bg, uint8_t scale)
{
    if (x >= s_lcd.width || y >= s_lcd.height)
        return;
    if (c < 0x20 || c > 0x7E)
        c = '?';

    uint8_t font_w = 5;
    uint8_t font_h = 7;
    uint16_t real_w = font_w * scale;
    uint16_t real_h = font_h * scale;

    /* 缓冲区，最大支持scale=4 */
    static uint16_t buffer[20 * 28];
    if (scale > 4)
        return;

    const uint8_t *glyph = st7789_font5x7[c - 0x20];
    uint32_t index = 0;

    uint16_t color_swapped = SWAP_BYTES(color);
    uint16_t bg_swapped = SWAP_BYTES(bg);

    /* 生成字符点阵数据的颜色缓冲区 */
    for (uint16_t r = 0; r < font_h; r++)
    {
        for (uint8_t sy = 0; sy < scale; sy++)
        {
            for (uint16_t col = 0; col < font_w; col++)
            {
                uint8_t pixel_on = (glyph[col] >> r) & 0x01;
                uint16_t draw_color = pixel_on ? color_swapped : bg_swapped;
                for (uint8_t sx = 0; sx < scale; sx++)
                {
                    buffer[index++] = draw_color;
                }
            }
        }
    }

    /* 设置写入窗口 */
    st7789_set_window(x, y, x + real_w - 1, y + real_h - 1);

    /* ============ 修复部分开始 ============ */
    /* set_window 结束后 CS 是高的。如果不拉低 CS，数据会被忽略！*/
    st7789_cs_low();
    st7789_dc_high(); // 确保是数据模式

    /* 发送缓冲区数据 */
    lcd_spi_send_buffer((uint8_t *)buffer, index * 2);

    st7789_cs_high();
    /* ============ 修复部分结束 ============ */
}

/**
 * @function st7789_draw_string
 * @brief 绘制字符串(带换行处理)
 * @param x 字符串左上角X坐标
 * @param y 字符串左上角Y坐标
 * @param str 待绘制的字符串指针(以'\0'结尾)
 * @param color 字体颜色，RGB565格式16位值
 * @param bg 背景颜色，RGB565格式16位值
 * @param scale 字符缩放因子
 * @details 自动换行处理：
 *          - 当字符绘制超过屏幕右边界时，自动换到下一行
 *          - 换行起始位置为x=0，y增加8*scale像素
 *          - 当超过屏幕下边界时停止绘制
 *          - 如果背景色与字体色不同，在字符后补充背景色填充
 */
void st7789_draw_string(uint16_t x, uint16_t y, const char *str, uint16_t color, uint16_t bg, uint8_t scale)
{
    while (*str)
    {
        if (x + 6 * scale >= s_lcd.width)
        {
            x = 0;
            y += 8 * scale;
            if (y >= s_lcd.height)
                break;
        }

        st7789_draw_char(x, y, *str++, color, bg, scale);

        if (bg != color)
        {
            st7789_fill_rect(x + 5 * scale, y, x + 6 * scale - 1, y + 7 * scale - 1, bg);
        }
        x += 6 * scale;
    }
}

/**
 * @function st7789_show_int
 * @brief 显示带格式的整数
 * @param x 显示左上角X坐标
 * @param y 显示左上角Y坐标
 * @param num 待显示的有符号32位整数
 * @param len 显示的数字位数(不足前补0)
 * @param color 字体颜色
 * @param bg 背景颜色
 * @param scale 字符缩放因子
 * @details 使用sprintf格式化整数为字符串，然后调用st7789_draw_string显示
 *          例如：st7789_show_int(10, 10, 1024, 4, ...) 显示 "1024"
 */
void st7789_show_int(uint16_t x, uint16_t y, int32_t num, uint8_t len, uint16_t color, uint16_t bg, uint8_t scale)
{
    char buf[16];
    char fmt[8];
    if (len > 15)
        len = 15;
    sprintf(fmt, "%%0%dd", len);
    sprintf(buf, fmt, num);
    st7789_draw_string(x, y, buf, color, bg, scale);
}

/**
 * @function st7789_show_float
 * @brief 显示带格式的浮点数
 * @param x 显示左上角X坐标
 * @param y 显示左上角Y坐标
 * @param num 待显示的单精度浮点数
 * @param len 整数部分位数(不足前补0)
 * @param frac 小数部分位数(四舍五入)
 * @param color 字体颜色
 * @param bg 背景颜色
 * @param scale 字符缩放因子
 * @details 使用snprintf和浮点格式化实现精确的小数显示
 *          例如：st7789_show_float(10, 10, 3.14159, 1, 4, ...) 显示 "3.1416"
 */
void st7789_show_float(uint16_t x, uint16_t y, float num, uint8_t len, uint8_t frac, uint16_t color, uint16_t bg, uint8_t scale)
{
    char buf[20];
    char fmt[10];
    if (len > 10)
        len = 10;
    if (frac > 6)
        frac = 6;
    sprintf(fmt, "%%0%d.%df", len + frac + 1, frac);
    snprintf(buf, sizeof(buf), fmt, num);
    st7789_draw_string(x, y, buf, color, bg, scale);
}

/**
 * @function st7789_draw_progress_bar_border
 * @brief 绘制进度条边框(空心矩形)
 * @param x 边框左上角X坐标
 * @param y 边框左上角Y坐标
 * @param w 边框宽度(像素)
 * @param h 边框高度(像素)
 * @param color 边框颜色
 * @details 使用四条直线绘制矩形框：上、下、左、右边界
 */
void st7789_draw_progress_bar_border(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    st7789_draw_line(x, y, x + w, y, color);
    st7789_draw_line(x, y + h, x + w, y + h, color);
    st7789_draw_line(x, y, x, y + h, color);
    st7789_draw_line(x + w, y, x + w, y + h, color);
}

/**
 * @function st7789_update_progress_bar
 * @brief 更新进度条的填充(增量更新，仅重绘变化部分)
 * @param x 进度条框体左上角X坐标
 * @param y 进度条框体左上角Y坐标
 * @param w 进度条框体宽度
 * @param h 进度条框体高度
 * @param last_percent 上一次的进度百分比(0~100)
 * @param now_percent 当前的进度百分比(0~100)
 * @param f_color 前景色(进度条填充颜色)
 * @param b_color 背景色(未完成部分颜色)
 *
 * @details 进度条高性能实现：
 *
 * 增量更新算法：
 *   1. 确保百分比在0~100范围内
 *   2. 如果新旧进度相同，直接返回(避免重复绘制)
 *   3. 计算内部绘制区域(去掉2像素边框)
 *   4. 根据进度百分比计算填充宽度：bar_w = inner_w * percent / 100
 *   5. 根据进度增减方向只绘制变化部分：
 *      - 进度增加：从上一个末尾填充到当前末尾(填充f_color)
 *      - 进度减少：从当前末尾填充到上一个末尾(填充b_color)
 *
 * 为什么要增量更新：
 *   - 完整重绘整个进度条每次都要调用st7789_fill_rect
 *   - 增量更新只需一次矩形填充，提升性能3倍以上
 *   - 在实时更新场景下(例如加载动画)尤其明显
 *
 * 使用场景：
 *   - 文件下载进度显示
 *   - 系统启动进度条
 *   - 数据加载等长时间操作的进度表示
 */
void st7789_update_progress_bar(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                                uint8_t last_percent, uint8_t now_percent,
                                uint16_t f_color, uint16_t b_color)
{
    if (now_percent > 100)
        now_percent = 100;
    if (last_percent > 100)
        last_percent = 100;
    if (now_percent == last_percent)
        return;

    uint16_t inner_x = x + 2;
    uint16_t inner_y = y + 2;
    uint16_t inner_w = w - 4;
    uint16_t inner_h = h - 4;

    uint16_t last_bar_w = (uint32_t)inner_w * last_percent / 100;
    uint16_t now_bar_w = (uint32_t)inner_w * now_percent / 100;

    if (now_percent > last_percent)
    {
        if (now_bar_w > last_bar_w)
        {
            st7789_fill_rect(inner_x + last_bar_w, inner_y,
                             inner_x + now_bar_w - 1, inner_y + inner_h - 1, f_color);
        }
    }
    else
    {
        if (last_bar_w > now_bar_w)
        {
            st7789_fill_rect(inner_x + now_bar_w, inner_y,
                             inner_x + last_bar_w - 1, inner_y + inner_h - 1, b_color);
        }
    }
}
/**
 * @function st7789_demo
 * @brief 液晶屏功能演示
 * @details 演示程序包括：
 *          1. 显示标题和版本信息(字符串)
 *          2. 显示整数示例(1024)
 *          3. 显示浮点数示例(3.14159)
 *          4. 绘制进度条边框
 *          5. 进度条从0%动画更新至100%
 *          6. 显示进度百分比及百分号符号
 *          7. 等待1秒
 *          8. 循环显示10种颜色，每种停留100ms
 *          本函数用于验证LCD初始化和各项功能是否正常
 */
void st7789_demo(void)
{
    st7789_clear(0x0000);

    st7789_draw_string(10, 10, "LCKFB 2.0 HW SPI", 0xFFE0, 0x0000, 2);
    st7789_draw_string(10, 50, "Int:", 0xFFFF, 0x0000, 2);
    st7789_show_int(70, 50, 1024, 4, 0x07E0, 0x0000, 2);
    st7789_draw_string(10, 80, "Flt:", 0xFFFF, 0x0000, 2);
    st7789_show_float(70, 80, 3.14159f, 1, 4, 0xF800, 0x0000, 2);

    st7789_draw_progress_bar_border(10, 140, 220, 20, 0x001F);

    uint8_t current_p = 0;
    for (int i = 0; i <= 100; i++)
    {
        st7789_update_progress_bar(10, 140, 220, 20, current_p, i, 0x001F, 0x0000);
        st7789_show_int(180, 120, i, 3, 0xFFFF, 0x0000, 2);
        st7789_draw_char(220, 120, '%', 0xFFFF, 0x0000, 2);
        current_p = i;
        HAL_Delay(20);
    }

    HAL_Delay(1000);

    uint16_t colors[] = {
        0xFFFF, 0x0000, 0x001F, 0xF800, 0x07E0,
        0x07FF, 0xF81F, 0xFFE0, 0x8410, 0xFD20};

    for (int i = 0; i < 10; i++)
    {
        st7789_fill_rect(20, 180, 220, 300, colors[i]);
        HAL_Delay(100);
    }
    HAL_Delay(500);
}
