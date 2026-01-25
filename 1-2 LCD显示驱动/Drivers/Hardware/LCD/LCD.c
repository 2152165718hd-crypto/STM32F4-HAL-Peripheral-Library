/**
 * @file LCD.c
 * @brief ST7789液晶显示驱动程序 - HAL库软件SPI实现
 * @author STM32F4-HAL-Peripheral-Library
 * @version 1.0
 * @date 2024
 *
 * @details 本文件包含ST7789液晶控制器的完整驱动实现，使用STM32 HAL库和软件SPI接口。
 * 支持多种显示方向、文字显示、图形绘制、进度条等功能。
 *
 * @note 该驱动完全基于GPIO软件SPI实现，无需硬件SPI外设
 */

/* ======================== 包含文件 ======================== */
#include "./SYSTEM/delay/delay.h"    /* 延时函数 */
#include "./Hardware/LCD/LCD.h"      /* LCD驱动头文件 */
#include "./Hardware/LCD/LCD_Data.h" /* LCD字体数据 */

/* ======================== 硬件引脚定义 ======================== */
/** @defgroup LCD_Pin_Define 液晶屏硬件引脚定义
 * @details 根据硬件设计定义各个控制信号的GPIO引脚，确保与实际硬件连接一致
 * @{
 */

/* SPI时钟信号SCLK (PA5) - 用于串行数据同步 */
#define LCD_SCLK_PIN GPIO_PIN_5  /* SCLK引脚编号 */
#define LCD_SCLK_GPIO_PORT GPIOA /* SCLK所在端口 */

/* 数据/命令选择信号DC (PD14) - 高电平表示数据，低电平表示命令 */
#define LCD_DC_PIN GPIO_PIN_14 /* DC引脚编号 */
#define LCD_DC_GPIO_PORT GPIOD /* DC所在端口 */

/* 背光控制信号BLK (PB8) - 用于控制液晶屏背光亮度 */
#define LCD_BLK_PIN GPIO_PIN_8  /* 背光引脚编号 */
#define LCD_BLK_GPIO_PORT GPIOB /* 背光所在端口 */

/* 硬件复位信号RST (PE1) - 用于初始化和复位液晶控制器 */
#define LCD_RST_PIN GPIO_PIN_1  /* 复位引脚编号 */
#define LCD_RST_GPIO_PORT GPIOE /* 复位所在端口 */

/* 片选信号CS (PE14) - 用于选中液晶控制器以进行SPI通信 */
#define LCD_CS_PIN GPIO_PIN_14 /* 片选引脚编号 */
#define LCD_CS_GPIO_PORT GPIOE /* 片选所在端口 */

/* SPI数据信号SDA/MOSI (PB5) - 用于发送串行数据至液晶控制器 */
#define LCD_SDA_PIN GPIO_PIN_5  /* SDA引脚编号 */
#define LCD_SDA_GPIO_PORT GPIOB /* SDA所在端口 */

/** @} */

/* ======================== 内部命令定义 ======================== */
/** @defgroup LCD_Command 液晶控制器内部命令定义
 * @details 这些命令用于配置液晶控制器的各种工作模式和显示参数
 * @{
 */

/* 列地址设置命令 - 用于定义水平写入范围 */
#define ST7789_CMD_COL_ADDR_SET 0x2A

/* 行地址设置命令 - 用于定义竖直写入范围 */
#define ST7789_CMD_ROW_ADDR_SET 0x2B

/* 内存写入命令 - 用于向显存中写入像素数据 */
#define ST7789_CMD_MEMORY_WRITE 0x2C

/* 内存数据访问控制命令 - 用于设置显示方向和扫描顺序 */
#define ST7789_CMD_MEMORY_DATA_ACCESS_CTRL 0x36

/* 接口像素格式命令 - 用于设置像素数据格式(如RGB565) */
#define ST7789_CMD_INTERFACE_PIXEL_FORMAT 0x3A

/* 睡眠模式关闭命令 - 用于唤醒液晶控制器 */
#define ST7789_CMD_SLEEP_OUT 0x11

/* 显示开启命令 - 用于打开液晶屏显示 */
#define ST7789_CMD_DISPLAY_ON 0x29

/** @} */

/* ======================== 液晶设备结构体定义 ======================== */
/** @struct st7789_dev_t
 * @brief 液晶显示器设备信息结构体
 * @details 用于保存液晶屏的动态参数(如当前宽度、高度、显示方向等)，
 * 允许在运行时动态改变这些参数
 */
typedef struct
{
    uint16_t width;   /* 当前显示宽度(像素) */
    uint16_t height;  /* 当前显示高度(像素) */
    st7789_dir_t dir; /* 当前显示方向(0~3) */
} st7789_dev_t;

/* 静态全局液晶设备对象 - 保存液晶屏的当前配置状态 */
static st7789_dev_t s_lcd =
    {
        .width = ST7789_CFG_DEFAULT_WIDTH,   /* 初始宽度为240像素 */
        .height = ST7789_CFG_DEFAULT_HEIGHT, /* 初始高度为320像素 */
        .dir = ST7789_DIR_PORTRAIT_0         /* 初始方向为竖屏0度 */
};

/* ======================== 字节交换宏定义 ======================== */
/** @def SWAP_BYTES(color)
 * @brief 交换16位颜色数据的高低字节
 * @param color 输入的16位RGB565颜色值
 * @return 交换后的颜色值
 * @details STM32是小端MCU，而液晶屏数据需要大端格式，
 * 通过此宏实现字节序转换以适配液晶屏要求
 */
#define SWAP_BYTES(color) ((uint16_t)(((color) >> 8) | ((color) << 8)))

/* ======================== 软件SPI底层实现 ======================== */

/**
 * @function soft_spi_write_byte
 * @brief 通过软件SPI发送一个字节数据
 * @param byte 要发送的字节数据
 * @return 无
 *
 * @algorithm
 * - 使用移位寄存器实现，从MSB(最高位)开始逐位发送
 * - 对每一位：先拉低时钟→设置数据→拉高时钟，形成一个SPI周期
 * - 共需8个时钟周期发送完一个字节
 * - MSB-first的发送顺序是SPI标准协议要求
 *
 * @details
 * 本函数采用位并行、时间串行的方式模拟SPI时序。
 * 通过GPIO直接控制，实现对硬件SPI的替代。
 */
static void soft_spi_write_byte(uint8_t byte)
{
    /* 循环8次发送8个bit */
    for (uint8_t i = 0; i < 8; i++)
    {
        HAL_GPIO_WritePin(LCD_SCLK_GPIO_PORT, LCD_SCLK_PIN, GPIO_PIN_RESET); /* SCLK置0(下降沿) */
        if (byte & 0x80)                                                     /* 判断最高位是否为1 */
            HAL_GPIO_WritePin(LCD_SDA_GPIO_PORT, LCD_SDA_PIN, GPIO_PIN_SET); /* 是1则MOSI设为高 */
        else
            HAL_GPIO_WritePin(LCD_SDA_GPIO_PORT, LCD_SDA_PIN, GPIO_PIN_RESET); /* 是0则MOSI设为低 */
        byte <<= 1;                                                            /* 左移一位，准备发送下一位 */
        HAL_GPIO_WritePin(LCD_SCLK_GPIO_PORT, LCD_SCLK_PIN, GPIO_PIN_SET);     /* SCLK置1(上升沿) */
    }
}

/**
 * @function st7789_cs_low
 * @brief 拉低片选信号
 * @return 无
 * @note CS低电平有效，表示选中液晶控制器
 */
static void st7789_cs_low(void) { HAL_GPIO_WritePin(LCD_CS_GPIO_PORT, LCD_CS_PIN, GPIO_PIN_RESET); }

/**
 * @function st7789_cs_high
 * @brief 拉高片选信号
 * @return 无
 * @note CS高电平无效，表示释放液晶控制器
 */
static void st7789_cs_high(void) { HAL_GPIO_WritePin(LCD_CS_GPIO_PORT, LCD_CS_PIN, GPIO_PIN_SET); }

/**
 * @function st7789_dc_low
 * @brief 拉低DC信号，表示发送命令
 * @return 无
 * @note DC低电平表示当前传输的数据是命令而非显示数据
 */
static void st7789_dc_low(void) { HAL_GPIO_WritePin(LCD_DC_GPIO_PORT, LCD_DC_PIN, GPIO_PIN_RESET); }

/**
 * @function st7789_dc_high
 * @brief 拉高DC信号，表示发送数据
 * @return 无
 * @note DC高电平表示当前传输的数据是显示数据而非命令
 */
static void st7789_dc_high(void) { HAL_GPIO_WritePin(LCD_DC_GPIO_PORT, LCD_DC_PIN, GPIO_PIN_SET); }

/**
 * @function st7789_write_cmd
 * @brief 向液晶控制器发送一个命令字节
 * @param cmd 要发送的命令字节
 * @return 无
 *
 * @algorithm
 * - 片选低(开始通信)
 * - DC拉低(告知控制器这是命令)
 * - 通过SPI发送命令字节
 * - 片选高(结束通信)
 */
static void st7789_write_cmd(uint8_t cmd)
{
    st7789_cs_low();          /* 片选低，选中液晶控制器 */
    st7789_dc_low();          /* DC低，表示发送的是命令 */
    soft_spi_write_byte(cmd); /* 通过SPI发送命令字节 */
    st7789_cs_high();         /* 片选高，释放液晶控制器 */
}

/**
 * @function st7789_write_data_byte
 * @brief 向液晶控制器发送一个数据字节
 * @param data 要发送的数据字节
 * @return 无
 *
 * @algorithm
 * - 片选低(开始通信)
 * - DC拉高(告知控制器这是数据)
 * - 通过SPI发送数据字节
 * - 片选高(结束通信)
 */
static void st7789_write_data_byte(uint8_t data)
{
    st7789_cs_low();           /* 片选低，选中液晶控制器 */
    st7789_dc_high();          /* DC高，表示发送的是数据 */
    soft_spi_write_byte(data); /* 通过SPI发送数据字节 */
    st7789_cs_high();          /* 片选高，释放液晶控制器 */
}

/**
 * @function st7789_write_data16
 * @brief 向液晶控制器发送一个16位数据(如颜色值)
 * @param data 要发送的16位数据
 * @return 无
 *
 * @algorithm
 * - 片选低(开始通信)
 * - DC拉高(告知控制器这是数据)
 * - 先发送高字节(data >> 8)
 * - 再发送低字节(data & 0xFF)
 * - 片选高(结束通信)
 *
 * @details
 * SPI是字节序列的通信协议，16位数据需分两个字节发送。
 * 大端格式(MSB first)要求先发高字节。
 */
static void st7789_write_data16(uint16_t data)
{
    st7789_cs_low();                  /* 片选低，选中液晶控制器 */
    st7789_dc_high();                 /* DC高，表示发送的是数据 */
    soft_spi_write_byte(data >> 8);   /* 先发送高字节 */
    soft_spi_write_byte(data & 0xFF); /* 再发送低字节 */
    st7789_cs_high();                 /* 片选高，释放液晶控制器 */
}

/**
 * @function st7789_write_bytes
 * @brief 向液晶控制器发送多个数据字节
 * @param data 指向数据缓冲区的指针
 * @param len 要发送的字节数
 * @return 无
 *
 * @algorithm
 * - 检查长度，如为0则直接返回
 * - 片选低(开始通信)
 * - DC拉高(表示发送的是数据)
 * - 循环发送每一个字节
 * - 片选高(结束通信)
 *
 * @details
 * 这是批量传输接口，用于一次性发送多个数据字节(如图像数据)。
 * 避免逐字节调用的函数开销，提高传输效率。
 */
static void st7789_write_bytes(const uint8_t *data, size_t len)
{
    if (len == 0)
        return;                      /* 如果长度为0，则直接返回避免无意义操作 */
    st7789_cs_low();                 /* 片选低，选中液晶控制器 */
    st7789_dc_high();                /* DC高，表示发送的是数据 */
    for (size_t i = 0; i < len; i++) /* 循环发送每一个字节 */
    {
        soft_spi_write_byte(data[i]); /* 发送缓冲区中的第i个字节 */
    }
    st7789_cs_high(); /* 片选高，释放液晶控制器 */
}

/* ======================== GPIO初始化 ======================== */

/**
 * @function lcd_gpio_init
 * @brief 初始化液晶屏所需的所有GPIO引脚
 * @return 无
 *
 * @algorithm
 * - 使能所有需要的GPIO端口时钟(GPIOA、B、D、E)
 * - 配置所有控制引脚为推挽输出模式
 * - 设置引脚速度为高速(100MHz)
 * - 初始化各引脚的默认状态
 *
 * @details
 * 该函数必须在使用LCD前调用。通过使能GPIO时钟和配置GPIO参数，
 * 使得驱动程序可以正确地控制液晶屏硬件。
 */
static void lcd_gpio_init(void)
{
    /* 使能所有需要的GPIO端口时钟，保证这些端口可以正常工作 */
    __HAL_RCC_GPIOA_CLK_ENABLE(); /* 使能GPIOA时钟(SCLK) */
    __HAL_RCC_GPIOB_CLK_ENABLE(); /* 使能GPIOB时钟(BLK、SDA) */
    __HAL_RCC_GPIOD_CLK_ENABLE(); /* 使能GPIOD时钟(DC) */
    __HAL_RCC_GPIOE_CLK_ENABLE(); /* 使能GPIOE时钟(RST、CS) */

    /* 初始化GPIO配置结构体，设置所有控制引脚的通用参数 */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;   /* 推挽输出模式，便于驱动高低电平 */
    GPIO_InitStruct.Pull = GPIO_NOPULL;           /* 无内部上拉下拉，使用外部电阻 */
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH; /* 高速模式(100MHz)，应对高频SPI时钟 */

    /* ========== 配置SCLK引脚(PA5) ========== */
    GPIO_InitStruct.Pin = LCD_SCLK_PIN;                  /* 设置要初始化的引脚为SCLK */
    HAL_GPIO_Init(LCD_SCLK_GPIO_PORT, &GPIO_InitStruct); /* 对GPIOA执行初始化 */

    /* ========== 配置SDA/MOSI引脚(PB5) ========== */
    GPIO_InitStruct.Pin = LCD_SDA_PIN;                  /* 设置要初始化的引脚为SDA */
    HAL_GPIO_Init(LCD_SDA_GPIO_PORT, &GPIO_InitStruct); /* 对GPIOB执行初始化 */

    /* ========== 配置DC引脚(PD14) ========== */
    GPIO_InitStruct.Pin = LCD_DC_PIN;                  /* 设置要初始化的引脚为DC */
    HAL_GPIO_Init(LCD_DC_GPIO_PORT, &GPIO_InitStruct); /* 对GPIOD执行初始化 */

    /* ========== 配置CS引脚(PE14) ========== */
    GPIO_InitStruct.Pin = LCD_CS_PIN;                  /* 设置要初始化的引脚为CS */
    HAL_GPIO_Init(LCD_CS_GPIO_PORT, &GPIO_InitStruct); /* 对GPIOE执行初始化 */

    /* ========== 配置RST引脚(PE1) ========== */
    GPIO_InitStruct.Pin = LCD_RST_PIN;                  /* 设置要初始化的引脚为RST */
    HAL_GPIO_Init(LCD_RST_GPIO_PORT, &GPIO_InitStruct); /* 对GPIOE执行初始化 */

    /* ========== 配置背光BLK引脚(PB8) ========== */
    GPIO_InitStruct.Pin = LCD_BLK_PIN;                  /* 设置要初始化的引脚为背光 */
    HAL_GPIO_Init(LCD_BLK_GPIO_PORT, &GPIO_InitStruct); /* 对GPIOB执行初始化 */

    /* ========== 设置引脚初始状态 ========== */
    HAL_GPIO_WritePin(LCD_CS_GPIO_PORT, LCD_CS_PIN, GPIO_PIN_SET);     /* CS初始高，未选中 */
    HAL_GPIO_WritePin(LCD_DC_GPIO_PORT, LCD_DC_PIN, GPIO_PIN_SET);     /* DC初始高，准备发数据 */
    HAL_GPIO_WritePin(LCD_SCLK_GPIO_PORT, LCD_SCLK_PIN, GPIO_PIN_SET); /* SCLK初始高，SPI空闲状态 */
    HAL_GPIO_WritePin(LCD_BLK_GPIO_PORT, LCD_BLK_PIN, GPIO_PIN_RESET); /* 背光初始关闭 */
}

/* ======================== 背光控制 ======================== */

/**
 * @function st7789_backlight_off
 * @brief 关闭液晶屏背光
 * @return 无
 * @note 背光引脚置为高电平时背光关闭
 */
static void st7789_backlight_off(void) { HAL_GPIO_WritePin(LCD_BLK_GPIO_PORT, LCD_BLK_PIN, GPIO_PIN_SET); }

/**
 * @function st7789_backlight_on
 * @brief 打开液晶屏背光
 * @return 无
 * @note 背光引脚置为低电平时背光打开
 */
static void st7789_backlight_on(void) { HAL_GPIO_WritePin(LCD_BLK_GPIO_PORT, LCD_BLK_PIN, GPIO_PIN_RESET); }

/* ======================== 硬件复位 ======================== */

/**
 * @function st7789_reset
 * @brief 执行硬件复位序列初始化液晶控制器
 * @return 无
 *
 * @algorithm
 * - RST引脚拉低(复位有效)，保持50ms
 * - RST引脚拉高(复位释放)，等待120ms让控制器完全初始化
 *
 * @details
 * 硬件复位是液晶初始化的必要步骤，通过拉低复位脚可将控制器
 * 复位到已知的初始状态。延时时间需按照芯片数据手册要求。
 */
static void st7789_reset(void)
{
    HAL_GPIO_WritePin(LCD_RST_GPIO_PORT, LCD_RST_PIN, GPIO_PIN_RESET); /* RST拉低，启动复位 */
    HAL_Delay(50);                                                     /* 等待50ms，确保复位电路充分放电 */
    HAL_GPIO_WritePin(LCD_RST_GPIO_PORT, LCD_RST_PIN, GPIO_PIN_SET);   /* RST拉高，释放复位 */
    HAL_Delay(120);                                                    /* 等待120ms，让液晶控制器完成初始化 */
}

/* ======================== 液晶寄存器初始化 ======================== */

/**
 * @function st7789_reg_init
 * @brief 初始化ST7789液晶控制器的内部寄存器参数
 * @return 无
 *
 * @algorithm
 * - 设置像素格式为RGB565(16位真彩色)
 * - 配置帧率与显示时序相关参数
 * - 设置电压参数(VCOM、VCC等)以优化显示效果
 * - 配置伽马曲线(两组曲线用于正向和反向灰度)
 * - 打开色彩反演功能以提高对比度
 * - 打开显示器
 *
 * @details
 * 这些参数直接影响液晶屏的显示效果。不同的液晶屏可能需要
 * 不同的参数调整。这些参数值来自供应商的参考代码。
 */
static void st7789_reg_init(void)
{
    /* 设置像素格式为RGB565(5-6-5真彩色),16位数据表示一个像素 */
    st7789_write_cmd(ST7789_CMD_INTERFACE_PIXEL_FORMAT);
    st7789_write_data_byte(0x05); /* 0x05表示RGB565格式 */

    /* 帧率控制参数(命令0xB2)设置帧频 */
    st7789_write_cmd(0xB2);
    {
        /* 这组参数控制帧率的显示时序 */
        uint8_t seq[] = {0x0C, 0x0C, 0x00, 0x33, 0x33};
        st7789_write_bytes(seq, sizeof(seq));
    }

    /* 栅极电压设置(命令0xB7) */
    st7789_write_cmd(0xB7);
    st7789_write_data_byte(0x35); /* 设置栅极电压值为0x35 */

    /* VCOM电压设置(命令0xBB) - 控制液晶分子的参考电压 */
    st7789_write_cmd(0xBB);
    st7789_write_data_byte(0x32); /* VCOM设为0x32，优化对比度 */

    /* 背光控制参数(命令0xC2) */
    st7789_write_cmd(0xC2);
    st7789_write_data_byte(0x01); /* 背光控制参数 */

    /* 背光调节参数(命令0xC3) */
    st7789_write_cmd(0xC3);
    st7789_write_data_byte(0x15); /* 背光调节值为0x15 */

    /* 背光PWM频率(命令0xC4) */
    st7789_write_cmd(0xC4);
    st7789_write_data_byte(0x20); /* 背光PWM频率设置 */

    /* 显示功能控制(命令0xC6) */
    st7789_write_cmd(0xC6);
    st7789_write_data_byte(0x0F); /* 显示功能控制参数 */

    /* 电源控制参数(命令0xD0) - 设置VRH/VGH/VGL等供电参数 */
    st7789_write_cmd(0xD0);
    {
        /* 这组参数控制液晶屏的供电电压 */
        uint8_t seq[] = {0xA4, 0xA1};
        st7789_write_bytes(seq, sizeof(seq));
    }

    /* 正伽马曲线(命令0xE0) - 用于调节从黑到白的灰度映射关系 */
    st7789_write_cmd(0xE0);
    {
        /* 伽马曲线用于改善显示器的灰度特性，这组参数用于正向显示 */
        uint8_t seq[] = {0xD0, 0x08, 0x0E, 0x09, 0x09, 0x05, 0x31, 0x33, 0x48, 0x17, 0x14, 0x15, 0x31, 0x34};
        st7789_write_bytes(seq, sizeof(seq));
    }

    /* 负伽马曲线(命令0xE1) - 与0xE0相对应，用于反向灰度映射 */
    st7789_write_cmd(0xE1);
    {
        /* 这组参数与0xE0配合使用，形成对称的伽马曲线 */
        uint8_t seq[] = {0xD0, 0x08, 0x0E, 0x09, 0x09, 0x15, 0x31, 0x33, 0x48, 0x17, 0x14, 0x15, 0x31, 0x34};
        st7789_write_bytes(seq, sizeof(seq));
    }

    /* 打开色彩反演(命令0x21) - 反转显示内容的颜色以提高对比度 */
    st7789_write_cmd(0x21); /* 此命令无参数，只是打开反演功能 */

    /* 打开显示器(命令0x29) - 使液晶屏输出可见的图像 */
    st7789_write_cmd(ST7789_CMD_DISPLAY_ON);
}

/* ======================== 核心显示功能 ======================== */

/**
 * @function st7789_set_direction
 * @brief 设置液晶屏的显示方向(旋转显示)
 * @param dir 显示方向枚举值(0~3分别对应0/90/180/270度)
 * @return 无
 *
 * @algorithm
 * - 根据指定方向更新s_lcd的宽高参数
 * - 发送内存访问控制命令(0x36)
 * - 根据方向发送相应的参数值以改变扫描方向
 *
 * @details
 * 该函数允许在运行时改变液晶屏的显示方向。不同的方向需要不同的
 * 扫描模式参数。同时需要交换宽高值以反映实际的显示尺寸变化。
 */
void st7789_set_direction(st7789_dir_t dir)
{
    s_lcd.dir = dir;                                      /* 保存当前方向到全局变量 */
    st7789_write_cmd(ST7789_CMD_MEMORY_DATA_ACCESS_CTRL); /* 发送内存访问控制命令 */

    switch (dir)
    {
    /* 竖屏0度(默认方向) */
    case ST7789_DIR_PORTRAIT_0:
        s_lcd.width = ST7789_CFG_DEFAULT_WIDTH;   /* 宽度为240 */
        s_lcd.height = ST7789_CFG_DEFAULT_HEIGHT; /* 高度为320 */
        st7789_write_data_byte(0x00);             /* 扫描参数0x00 */
        break;
    /* 横屏90度(左转90度) */
    case ST7789_DIR_LANDSCAPE_90:
        s_lcd.width = ST7789_CFG_DEFAULT_HEIGHT; /* 宽度变为320 */
        s_lcd.height = ST7789_CFG_DEFAULT_WIDTH; /* 高度变为240 */
        st7789_write_data_byte(0xC0);            /* 扫描参数0xC0 */
        break;
    /* 竖屏180度(倒转) */
    case ST7789_DIR_PORTRAIT_180:
        s_lcd.width = ST7789_CFG_DEFAULT_WIDTH;   /* 宽度仍为240 */
        s_lcd.height = ST7789_CFG_DEFAULT_HEIGHT; /* 高度仍为320 */
        st7789_write_data_byte(0x70);             /* 扫描参数0x70 */
        break;
    /* 横屏270度(右转90度) */
    case ST7789_DIR_LANDSCAPE_270:
        s_lcd.width = ST7789_CFG_DEFAULT_HEIGHT; /* 宽度变为320 */
        s_lcd.height = ST7789_CFG_DEFAULT_WIDTH; /* 高度变为240 */
        st7789_write_data_byte(0xA0);            /* 扫描参数0xA0 */
        break;
    default: /* 未知方向，保持不变 */
        break;
    }
}

/**
 * @function st7789_set_window
 * @brief 设置矩形显示窗口，后续数据写入将在此区域内
 * @param x0 窗口左上角X坐标
 * @param y0 窗口左上角Y坐标
 * @param x1 窗口右下角X坐标
 * @param y1 窗口右下角Y坐标
 * @return 无
 *
 * @algorithm
 * - 计算实际的硬件坐标(加上偏移值)
 * - 发送列地址范围命令(0x2A)及起止坐标
 * - 发送行地址范围命令(0x2B)及起止坐标
 * - 发送内存写入命令(0x2C)准备写入数据
 *
 * @details
 * ST7789控制器内部有地址自动递增机制。设置好窗口后，
 * 后续的数据写入会自动在该窗口内递增地址。
 */
static void st7789_set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    /* 加上偏移值得到硬件坐标(某些屏幕可能需要坐标偏移) */
    uint16_t xs = x0 + ST7789_CFG_X_OFFSET; /* 起始列坐标加X偏移 */
    uint16_t xe = x1 + ST7789_CFG_X_OFFSET; /* 终止列坐标加X偏移 */
    uint16_t ys = y0 + ST7789_CFG_Y_OFFSET; /* 起始行坐标加Y偏移 */
    uint16_t ye = y1 + ST7789_CFG_Y_OFFSET; /* 终止行坐标加Y偏移 */

    /* 设置列地址窗口(命令0x2A) */
    st7789_write_cmd(ST7789_CMD_COL_ADDR_SET);
    st7789_write_data_byte(xs >> 8);   /* 发送起始列地址高字节 */
    st7789_write_data_byte(xs & 0xFF); /* 发送起始列地址低字节 */
    st7789_write_data_byte(xe >> 8);   /* 发送终止列地址高字节 */
    st7789_write_data_byte(xe & 0xFF); /* 发送终止列地址低字节 */

    /* 设置行地址窗口(命令0x2B) */
    st7789_write_cmd(ST7789_CMD_ROW_ADDR_SET);
    st7789_write_data_byte(ys >> 8);   /* 发送起始行地址高字节 */
    st7789_write_data_byte(ys & 0xFF); /* 发送起始行地址低字节 */
    st7789_write_data_byte(ye >> 8);   /* 发送终止行地址高字节 */
    st7789_write_data_byte(ye & 0xFF); /* 发送终止行地址低字节 */

    /* 发送内存写入命令准备数据传输(命令0x2C) */
    st7789_write_cmd(ST7789_CMD_MEMORY_WRITE);
}

/**
 * @function st7789_fill_rect
 * @brief 用指定颜色填充矩形区域
 * @param x0 矩形左上角X坐标
 * @param y0 矩形左上角Y坐标
 * @param x1 矩形右下角X坐标
 * @param y1 矩形右下角Y坐标
 * @param color 填充颜色(RGB565格式)
 * @return 无
 *
 * @algorithm
 * - 检查坐标有效性，如果超出屏幕范围则进行裁剪
 * - 设置显示窗口为指定矩形
 * - 计算矩形中的总像素数
 * - 使用缓冲区存储待发送的颜色数据(512像素为一批)
 * - 分批发送数据以节省RAM空间
 *
 * @details
 * 该函数是高效的矩形填充实现。通过缓冲区技术可以用较少的RAM
 * 实现快速的大面积填充。颜色值会被字节交换以适配SPI大端格式。
 */
void st7789_fill_rect(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color)
{
    /* 检查坐标的合理性，如果左上角大于右下角则直接返回 */
    if (x0 > x1 || y0 > y1)
        return;

    /* 对超出屏幕范围的坐标进行裁剪 */
    if (x1 >= s_lcd.width)
        x1 = s_lcd.width - 1; /* X不能超过屏幕宽度 */
    if (y1 >= s_lcd.height)
        y1 = s_lcd.height - 1; /* Y不能超过屏幕高度 */

    /* 设置写入窗口为指定矩形区域 */
    st7789_set_window(x0, y0, x1, y1);

    /* 计算矩形中的总像素数 */
    uint32_t pixels = (uint32_t)(x1 - x0 + 1) * (uint32_t)(y1 - y0 + 1);

    /* 定义缓冲区用于存储颜色数据(512个像素) */
    uint16_t buffer[512];
    uint32_t block_size = 512; /* 每次批量发送的像素数 */

    /* 对颜色值进行字节交换以匹配SPI大端格式 */
    uint16_t color_swapped = SWAP_BYTES(color);

    /* 将相同颜色填充到整个缓冲区 */
    for (uint32_t i = 0; i < block_size; i++)
    {
        buffer[i] = color_swapped; /* 每个缓冲区单元存储交换后的颜色 */
    }

    /* 拉低片选，开始批量数据传输 */
    st7789_cs_low();
    st7789_dc_high(); /* DC高表示发送的是显示数据 */

    /* 分批发送缓冲区中的数据直到所有像素都被写入 */
    while (pixels > 0)
    {
        /* 计算本次需要发送的像素数(不超过512) */
        uint32_t current_chunk = (pixels > block_size) ? block_size : pixels;

        /* 每个像素是2个字节，所以要乘以2 */
        for (uint32_t i = 0; i < current_chunk * 2; i++)
        {
            soft_spi_write_byte(((uint8_t *)buffer)[i]); /* 逐字节发送 */
        }
        pixels -= current_chunk; /* 减少剩余待发像素数 */
    }

    st7789_cs_high(); /* 拉高片选，结束数据传输 */
}

/**
 * @function st7789_clear
 * @brief 用指定颜色清空整个液晶屏幕
 * @param color 清空颜色(RGB565格式)
 * @return 无
 *
 * @details
 * 这是对st7789_fill_rect的简化调用，将整个屏幕范围作为矩形进行填充。
 * 常用于初始化屏幕或切换显示内容时清除旧内容。
 */
void st7789_clear(uint16_t color)
{
    st7789_fill_rect(0, 0, s_lcd.width - 1, s_lcd.height - 1, color); /* 填充整个屏幕范围 */
}

/**
 * @function st7789_draw_pixel
 * @brief 在指定坐标处绘制一个像素
 * @param x 像素X坐标
 * @param y 像素Y坐标
 * @param color 像素颜色(RGB565格式)
 * @return 无
 *
 * @algorithm
 * - 检查坐标是否越界，越界则直接返回
 * - 设置1x1的显示窗口为该像素位置
 * - 发送16位颜色数据
 *
 * @details
 * 虽然该函数可用于逐像素绘制，但效率较低。推荐用于特殊场景，
 * 如画布的细节修改。大面积操作应使用fill_rect等高效函数。
 */
void st7789_draw_pixel(uint16_t x, uint16_t y, uint16_t color)
{
    if (x >= s_lcd.width || y >= s_lcd.height)
        return;                    /* 超出屏幕范围则不绘制 */
    st7789_set_window(x, y, x, y); /* 设置窗口为单个像素 */
    st7789_write_data16(color);    /* 发送16位颜色数据 */
}

/**
 * @function st7789_draw_line
 * @brief 使用Bresenham算法绘制一条直线
 * @param x0 起点X坐标
 * @param y0 起点Y坐标
 * @param x1 终点X坐标
 * @param y1 终点Y坐标
 * @param color 直线颜色(RGB565格式)
 * @return 无
 *
 * @algorithm
 * 使用Bresenham线条算法(增量圆周长算法)：
 * - 计算X和Y方向的增量(dx、dy)
 * - 计算方向标志(sx、sy)
 * - 计算初始误差值
 * - 逐点绘制，根据误差值决定下一点位置
 *
 * @details
 * Bresenham算法是光栅化直线绘制的经典算法。通过整数运算避免浮点计算，
 * 提高了效率。该实现支持任意方向的直线绘制。
 */
void st7789_draw_line(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color)
{
    /* 计算X方向的增量(取绝对值) */
    int16_t dx = abs((int16_t)x1 - (int16_t)x0);
    /* 计算X方向的步进符号(+1或-1) */
    int16_t sx = x0 < x1 ? 1 : -1;

    /* 计算Y方向的增量(负数表示向下) */
    int16_t dy = -abs((int16_t)y1 - (int16_t)y0);
    /* 计算Y方向的步进符号(+1或-1) */
    int16_t sy = y0 < y1 ? 1 : -1;

    /* 初始误差值，用于判断何时改变坐标 */
    int16_t err = dx + dy;
    int16_t e2; /* 临时误差值 */

    /* 无限循环绘制直线(通过break在终点时退出) */
    for (;;)
    {
        st7789_draw_pixel(x0, y0, color); /* 绘制当前点 */
        if (x0 == x1 && y0 == y1)
            break; /* 已到达终点，退出循环 */

        e2 = 2 * err; /* 计算两倍的误差值 */

        if (e2 >= dy)
        {
            err += dy;
            x0 += sx;
        } /* X坐标步进 */
        if (e2 <= dx)
        {
            err += dx;
            y0 += sy;
        } /* Y坐标步进 */
    }
}

/**
 * @function st7789_draw_bitmap565
 * @brief 绘制一张RGB565格式的位图图像
 * @param x 图像左上角X坐标
 * @param y 图像左上角Y坐标
 * @param w 图像宽度(像素)
 * @param h 图像高度(像素)
 * @param data 指向RGB565图像数据的指针(2字节/像素)
 * @return 无
 *
 * @algorithm
 * - 检查图像是否超出屏幕范围
 * - 设置显示窗口为图像范围
 * - 一次性发送所有图像字节数据
 *
 * @details
 * 该函数用于高效地显示预制的位图图像(如图标、LOGO等)。
 * 图像数据必须是RGB565格式，每个像素占2个字节。
 */
void st7789_draw_bitmap565(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint8_t *data)
{
    if (x + w > s_lcd.width || y + h > s_lcd.height)
        return;                                    /* 超出屏幕则不绘制 */
    st7789_set_window(x, y, x + w - 1, y + h - 1); /* 设置窗口为图像范围 */
    st7789_write_bytes(data, (size_t)w * h * 2);   /* 发送所有图像字节(w*h*2) */
}

/* ======================== 初始化序列 ======================== */

/**
 * @function st7789_init
 * @brief 初始化ST7789液晶驱动器及其显示
 * @return 无
 *
 * @algorithm
 * 1. 初始化GPIO(使能时钟、配置引脚)
 * 2. 关闭背光(初始黑屏)
 * 3. 执行硬件复位(清除控制器状态)
 * 4. 发送睡眠关闭命令唤醒控制器
 * 5. 等待控制器稳定
 * 6. 设置默认显示方向
 * 7. 配置控制器寄存器(像素格式、伽马、电压等)
 * 8. 清屏为黑色
 * 9. 打开背光
 *
 * @details
 * 这是液晶驱动的标准初始化流程。顺序很重要，每一步都为后续操作
 * 创建必要的条件。初始化后液晶屏可以正常显示。
 *
 * @note 该函数应在main函数启动时调用一次
 */
void st7789_init(void)
{
    lcd_gpio_init(); /* 初始化GPIO引脚(使能时钟、配置引脚模式) */

    st7789_backlight_off(); /* 关闭背光，防止初始化过程中闪烁 */
    st7789_reset();         /* 执行硬件复位，清除控制器内部状态 */

    /* 发送睡眠关闭命令唤醒液晶控制器 */
    st7789_write_cmd(ST7789_CMD_SLEEP_OUT);
    HAL_Delay(200); /* 等待200ms让控制器完全唤醒 */

    st7789_set_direction(ST7789_DIR_PORTRAIT_0); /* 设置默认显示方向为竖屏0度 */
    st7789_reg_init();                           /* 初始化控制器寄存器(像素格式、电压、伽马等) */

    st7789_clear(ST7789_COLOR_BLACK); /* 清屏为黑色，避免垃圾数据显示 */
    st7789_backlight_on();            /* 打开背光，使屏幕可见 */
}

/**
 * @function st7789_width
 * @brief 获取当前显示的宽度
 * @return 当前显示宽度(像素)
 * @note 该值会随着显示方向改变而改变
 */
uint16_t st7789_width(void) { return s_lcd.width; }

/**
 * @function st7789_height
 * @brief 获取当前显示的高度
 * @return 当前显示高度(像素)
 * @note 该值会随着显示方向改变而改变
 */
uint16_t st7789_height(void) { return s_lcd.height; }

/* ======================== 文本与图形显示函数 ======================== */

/**
 * @function st7789_draw_char
 * @brief 在指定位置绘制一个字符
 * @param x 字符左上角X坐标
 * @param y 字符左上角Y坐标
 * @param c 要绘制的字符(ASCII字符)
 * @param color 字符颜色(RGB565格式)
 * @param bg 字符背景颜色(RGB565格式)
 * @param scale 缩放倍数(1~4)
 * @return 无
 *
 * @algorithm
 * 1. 检查坐标有效性和字符有效性
 * 2. 定义字体尺寸(5x7像素点阵)
 * 3. 计算缩放后的实际尺寸
 * 4. 从字体数据中获取该字符的点阵
 * 5. 对每个点阵点进行缩放，填充缓冲区
 * 6. 一次性显示整个字符
 *
 * @details
 * 本函数使用5x7的标准点阵字体。缩放功能通过复制像素实现。
 * 字体数据来自LCD_Data.h中的st7789_font5x7数组。
 */
void st7789_draw_char(uint16_t x, uint16_t y, char c, uint16_t color, uint16_t bg, uint8_t scale)
{
    if (x >= s_lcd.width || y >= s_lcd.height)
        return; /* 起点超出屏幕范围则不绘制 */

    /* 将无效字符替换为问号 */
    if (c < 0x20 || c > 0x7E)
        c = '?';

    uint8_t font_w = 5;               /* 字体点阵宽度为5 */
    uint8_t font_h = 7;               /* 字体点阵高度为7 */
    uint16_t real_w = font_w * scale; /* 缩放后的实际宽度 */
    uint16_t real_h = font_h * scale; /* 缩放后的实际高度 */

    uint16_t buffer[20 * 28] = {0}; /* 缓冲区可容纳最大4倍缩放的字符 */
    if (scale > 4)
        return; /* 缩放倍数过大则退出 */

    /* 从字体数据数组中获取该字符的点阵数据 */
    const uint8_t *glyph = st7789_font5x7[c - 0x20];
    uint32_t index = 0;

    /* 对颜色进行字节交换以匹配SPI格式 */
    uint16_t color_swapped = SWAP_BYTES(color);
    uint16_t bg_swapped = SWAP_BYTES(bg);

    /* 遍历点阵的每一行(7行) */
    for (uint16_t r = 0; r < font_h; r++)
    {
        /* 对每一行进行纵向缩放 */
        for (uint8_t sy = 0; sy < scale; sy++)
        {
            /* 遍历点阵的每一列(5列) */
            for (uint16_t col = 0; col < font_w; col++)
            {
                /* 提取该列在当前行的像素值(0或1) */
                uint8_t pixel_on = (glyph[col] >> r) & 0x01;
                /* 根据像素值选择颜色或背景色 */
                uint16_t draw_color = pixel_on ? color_swapped : bg_swapped;

                /* 对该像素进行横向缩放(scale倍复制) */
                for (uint8_t sx = 0; sx < scale; sx++)
                {
                    buffer[index++] = draw_color; /* 存储缩放后的像素 */
                }
            }
        }
    }

    /* 在屏幕上设置窗口并显示整个字符缓冲区 */
    st7789_set_window(x, y, x + real_w - 1, y + real_h - 1);
    st7789_write_bytes((uint8_t *)buffer, index * 2); /* index表示像素数，乘以2得字节数 */
}

/**
 * @function st7789_draw_string
 * @brief 在指定位置显示一个字符串
 * @param x 字符串起始X坐标
 * @param y 字符串起始Y坐标
 * @param str 指向字符串的指针(C字符串，以\\0结尾)
 * @param color 字符颜色(RGB565格式)
 * @param bg 字符背景颜色(RGB565格式)
 * @param scale 缩放倍数(1~4)
 * @return 无
 *
 * @algorithm
 * 1. 逐字符遍历字符串(直到\\0结尾)
 * 2. 检查当前字符是否会超出屏幕右边界
 * 3. 若超出则换行到下一行
 * 4. 绘制当前字符
 * 5. 如果字符背景色与前景色不同，则在字符右边填充背景
 * 6. 移动到下一个字符位置
 *
 * @details
 * 本函数实现了自动换行功能。每行可显示的字符数取决于屏幕宽度和字符大小。
 * 如果字符串超出屏幕底部，多余的字符将不显示。
 */
void st7789_draw_string(uint16_t x, uint16_t y, const char *str, uint16_t color, uint16_t bg, uint8_t scale)
{
    /* 逐字符处理直到遇到字符串结尾符\\0 */
    while (*str)
    {
        /* 检查当前字符是否会超出屏幕右边界(字符宽度为6*scale) */
        if (x + 6 * scale >= s_lcd.width)
        {
            x = 0;          /* 回到最左边 */
            y += 8 * scale; /* 下移一行(8个像素高度) */
            if (y >= s_lcd.height)
                break; /* 超出屏幕下边界则停止显示 */
        }

        st7789_draw_char(x, y, *str++, color, bg, scale); /* 绘制当前字符 */

        /* 如果背景色与前景色不同，则在字符右边绘制背景(补全字符间距) */
        if (bg != color)
        {
            st7789_fill_rect(x + 5 * scale, y, x + 6 * scale - 1, y + 7 * scale - 1, bg);
        }
        x += 6 * scale; /* 移动到下一个字符位置 */
    }
}

/**
 * @function st7789_show_int
 * @brief 在指定位置显示一个整数
 * @param x 显示的X坐标
 * @param y 显示的Y坐标
 * @param num 要显示的整数值
 * @param len 显示的字符宽度(前面补0)
 * @param color 文字颜色
 * @param bg 背景颜色
 * @param scale 字体缩放倍数
 * @return 无
 *
 * @details
 * 该函数使用sprintf格式化整数，然后调用st7789_draw_string显示。
 * len参数可以在整数前补0实现定长显示。
 */
void st7789_show_int(uint16_t x, uint16_t y, int32_t num, uint8_t len, uint16_t color, uint16_t bg, uint8_t scale)
{
    char buf[16]; /* 缓冲区用于存储格式化后的字符串 */
    char fmt[8];  /* 缓冲区用于存储格式字符串 */
    if (len > 15)
        len = 15;                                    /* 限制宽度不超过15 */
    sprintf(fmt, "%%0%dd", len);                     /* 创建格式字符串如"%05d" */
    sprintf(buf, fmt, num);                          /* 将整数按格式转换为字符串 */
    st7789_draw_string(x, y, buf, color, bg, scale); /* 显示格式化的字符串 */
}

/**
 * @function st7789_show_float
 * @brief 在指定位置显示一个浮点数
 * @param x 显示的X坐标
 * @param y 显示的Y坐标
 * @param num 要显示的浮点值
 * @param len 显示的整数部分宽度
 * @param frac 显示的小数位数
 * @param color 文字颜色
 * @param bg 背景颜色
 * @param scale 字体缩放倍数
 * @return 无
 *
 * @details
 * 该函数使用snprintf格式化浮点数，可以指定小数位数。
 * frac参数限制为最多6位小数。
 */
void st7789_show_float(uint16_t x, uint16_t y, float num, uint8_t len, uint8_t frac, uint16_t color, uint16_t bg, uint8_t scale)
{
    char buf[20]; /* 缓冲区用于存储格式化后的字符串 */
    char fmt[10]; /* 缓冲区用于存储格式字符串 */
    if (len > 10)
        len = 10; /* 限制整数部分宽度 */
    if (frac > 6)
        frac = 6;                                    /* 限制小数位数不超过6 */
    sprintf(fmt, "%%0%d.%df", len + frac + 1, frac); /* 创建格式字符串 */
    snprintf(buf, sizeof(buf), fmt, num);            /* 将浮点数按格式转换为字符串 */
    st7789_draw_string(x, y, buf, color, bg, scale); /* 显示格式化的字符串 */
}

/**
 * @function st7789_draw_progress_bar_border
 * @brief 绘制进度条的矩形边框
 * @param x 进度条左上角X坐标
 * @param y 进度条左上角Y坐标
 * @param w 进度条宽度
 * @param h 进度条高度
 * @param color 边框颜色
 * @return 无
 *
 * @algorithm
 * 绘制矩形的四条边界线：
 * - 上边：从(x,y)到(x+w,y)
 * - 下边：从(x,y+h)到(x+w,y+h)
 * - 左边：从(x,y)到(x,y+h)
 * - 右边：从(x+w,y)到(x+w,y+h)
 */
void st7789_draw_progress_bar_border(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    st7789_draw_line(x, y, x + w, y, color);         /* 绘制上边框 */
    st7789_draw_line(x, y + h, x + w, y + h, color); /* 绘制下边框 */
    st7789_draw_line(x, y, x, y + h, color);         /* 绘制左边框 */
    st7789_draw_line(x + w, y, x + w, y + h, color); /* 绘制右边框 */
}

/**
 * @function st7789_update_progress_bar
 * @brief 更新进度条的显示(只重绘改变的部分)
 * @param x 进度条左上角X坐标
 * @param y 进度条左上角Y坐标
 * @param w 进度条宽度
 * @param h 进度条高度
 * @param last_percent 上一次的百分比值(0~100)
 * @param now_percent 当前的百分比值(0~100)
 * @param f_color 前景色(进度填充颜色)
 * @param b_color 背景色(未填充部分颜色)
 * @return 无
 *
 * @algorithm
 * 1. 限制百分比值在0~100范围内
 * 2. 如果百分比未改变则直接返回(优化性能)
 * 3. 计算内部区域(去掉2像素边框)的坐标
 * 4. 根据百分比计算对应的填充宽度
 * 5. 如果新进度>旧进度，则在扩展部分填充前景色
 * 6. 如果新进度<旧进度，则在缩减部分填充背景色
 *
 * @details
 * 这个实现只重绘改变的部分，提高了更新效率。
 * 适合在循环中频繁更新进度条的场景。
 */
void st7789_update_progress_bar(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                                uint8_t last_percent, uint8_t now_percent,
                                uint16_t f_color, uint16_t b_color)
{
    if (now_percent > 100)
        now_percent = 100; /* 限制新进度不超过100% */
    if (last_percent > 100)
        last_percent = 100; /* 限制旧进度不超过100% */
    if (now_percent == last_percent)
        return; /* 如果进度未改变则无需重绘 */

    /* 计算内部填充区域的坐标(去掉2像素的边框) */
    uint16_t inner_x = x + 2; /* 内部区域左边界 */
    uint16_t inner_y = y + 2; /* 内部区域上边界 */
    uint16_t inner_w = w - 4; /* 内部区域宽度 */
    uint16_t inner_h = h - 4; /* 内部区域高度 */

    /* 根据百分比计算对应的填充宽度 */
    uint16_t last_bar_w = (uint32_t)inner_w * last_percent / 100; /* 旧进度对应的宽度 */
    uint16_t now_bar_w = (uint32_t)inner_w * now_percent / 100;   /* 新进度对应的宽度 */

    /* 如果新进度大于旧进度，则填充新增的部分 */
    if (now_percent > last_percent)
    {
        if (now_bar_w > last_bar_w) /* 确保新宽度确实大于旧宽度 */
        {
            /* 填充扩展部分为前景色 */
            st7789_fill_rect(inner_x + last_bar_w, inner_y,
                             inner_x + now_bar_w - 1, inner_y + inner_h - 1, f_color);
        }
    }
    /* 如果新进度小于旧进度，则清除减少的部分 */
    else
    {
        if (last_bar_w > now_bar_w) /* 确保旧宽度确实大于新宽度 */
        {
            /* 填充缩减部分为背景色 */
            st7789_fill_rect(inner_x + now_bar_w, inner_y,
                             inner_x + last_bar_w - 1, inner_y + inner_h - 1, b_color);
        }
    }
}

/* ======================== 演示函数 ======================== */

/**
 * @function st7789_demo
 * @brief LCD驱动功能演示程序
 * @return 无
 *
 * @algorithm
 * 1. 清屏为黑色
 * 2. 显示标题字符串(黄色)
 * 3. 显示整数值示例(绿色)
 * 4. 显示浮点数示例(红色)
 * 5. 绘制进度条边框(蓝色)
 * 6. 动画演示进度条填充(0~100%)，每次增1%，延迟20ms
 * 7. 全屏颜色滚动显示(10种颜色轮流显示)
 * 8. 每种颜色显示100ms
 *
 * @details
 * 这个函数演示了本驱动支持的主要功能：
 * - 清屏功能
 * - 字符串显示
 * - 整数和浮点数显示
 * - 线条和矩形绘制
 * - 动画效果(进度条和颜色滚动)
 */
void st7789_demo(void)
{
    st7789_clear(ST7789_COLOR_BLACK); /* 清屏为黑色背景 */

    /* ========== 显示标题和演示内容 ========== */
    st7789_draw_string(10, 10, "LCKFB 2.0 in screen", ST7789_COLOR_YELLOW, ST7789_COLOR_BLACK, 2);
    st7789_draw_string(10, 50, "Int:", ST7789_COLOR_WHITE, ST7789_COLOR_BLACK, 2);
    st7789_show_int(70, 50, 1024, 4, ST7789_COLOR_GREEN, ST7789_COLOR_BLACK, 2);
    st7789_draw_string(10, 80, "Flt:", ST7789_COLOR_WHITE, ST7789_COLOR_BLACK, 2);
    st7789_show_float(70, 80, 3.14159f, 1, 4, ST7789_COLOR_RED, ST7789_COLOR_BLACK, 2);

    /* ========== 绘制进度条边框并演示动画 ========== */
    st7789_draw_progress_bar_border(10, 140, 220, 20, ST7789_COLOR_BLUE); /* 绘制边框 */

    uint8_t current_p = 0; /* 当前进度百分比 */
    /* 进度条从0%渐进到100% */
    for (int i = 0; i <= 100; i++)
    {
        st7789_update_progress_bar(10, 140, 220, 20, current_p, i, ST7789_COLOR_BLUE, ST7789_COLOR_BLACK);
        st7789_show_int(180, 120, i, 3, ST7789_COLOR_WHITE, ST7789_COLOR_BLACK, 2); /* 显示百分比 */
        st7789_draw_char(220, 120, '%', ST7789_COLOR_WHITE, ST7789_COLOR_BLACK, 2); /* 显示百分号 */
        current_p = i;                                                              /* 记录当前进度 */
        HAL_Delay(20);                                                              /* 延迟20ms实现动画效果 */
    }

    HAL_Delay(1000); /* 停留1秒让用户看到最终结果 */

    /* ========== 全屏颜色滚动演示 ========== */
    /* 定义10种颜色用于演示 */
    uint16_t colors[] = {
        ST7789_COLOR_WHITE,   /* 白色 */
        ST7789_COLOR_BLACK,   /* 黑色 */
        ST7789_COLOR_BLUE,    /* 蓝色 */
        ST7789_COLOR_RED,     /* 红色 */
        ST7789_COLOR_GREEN,   /* 绿色 */
        ST7789_COLOR_CYAN,    /* 青色 */
        ST7789_COLOR_MAGENTA, /* 品红 */
        ST7789_COLOR_YELLOW,  /* 黄色 */
        ST7789_COLOR_GRAY,    /* 灰色 */
        ST7789_COLOR_ORANGE   /* 橙色 */
    };

    /* 循环显示10种颜色，每种100ms */
    for (int i = 0; i < 10; i++)
    {
        st7789_fill_rect(20, 180, 220, 300, colors[i]); /* 填充矩形显示颜色 */
        HAL_Delay(100);                                 /* 每种颜色显示100ms */
    }
    HAL_Delay(500); /* 最后停留500ms */
}
