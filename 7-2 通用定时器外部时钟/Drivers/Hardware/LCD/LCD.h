/**
 * @file LCD.h
 * @brief ST7789 液晶显示驱动接口定义头文件
 * @author STM32F4-HAL-Peripheral-Library
 * @version 1.0
 * @date 2024
 *
 * @details 本头文件定义了 ST7789 液晶控制器的完整 API 接口，包括
 * 初始化、图形绘制、文本显示、进度条等功能的函数声明和类型定义。
 *
 * @note 驱动基于GPIO软件SPI实现，无需硬件SPI外设
 */

#ifndef __LCD_H__
#define __LCD_H__

#include "stm32f4xx_hal.h" /* STM32F4xx HAL库(可根据MCU型号调整) */
#include <stdint.h>        /* 标准整数类型 */
#include <stddef.h>        /* 标准定义(size_t等) */
#include <stdio.h>         /* 标准输入输出(sprintf等) */
#include <string.h>        /* 字符串操作函数 */

#define USE_HARDWARE_SPI 1
/* ======================== 配置宏定义 ======================== */
/** @defgroup LCD_Config 液晶屏配置参数
 * @{
 */

/** @def ST7789_CFG_DEFAULT_WIDTH
 * @brief 液晶屏默认宽度(像素)
 * @note 可根据实际屏幕参数修改
 */
#ifndef ST7789_CFG_DEFAULT_WIDTH
#define ST7789_CFG_DEFAULT_WIDTH 240
#endif

/** @def ST7789_CFG_DEFAULT_HEIGHT
 * @brief 液晶屏默认高度(像素)
 * @note 可根据实际屏幕参数修改
 */
#ifndef ST7789_CFG_DEFAULT_HEIGHT
#define ST7789_CFG_DEFAULT_HEIGHT 320
#endif

/** @def ST7789_CFG_X_OFFSET
 * @brief X轴坐标偏移值(用于适配不同屏幕)
 */
#ifndef ST7789_CFG_X_OFFSET
#define ST7789_CFG_X_OFFSET 0
#endif

/** @def ST7789_CFG_Y_OFFSET
 * @brief Y轴坐标偏移值(用于适配不同屏幕)
 */
#ifndef ST7789_CFG_Y_OFFSET
#define ST7789_CFG_Y_OFFSET 0
#endif

/** @} */

/*================ 方向枚举 ================*/

/* ======================== 方向枚举定义 ======================== */
/** @enum st7789_dir_t
 * @brief 液晶屏显示方向枚举
 */
typedef enum
{
    ST7789_DIR_PORTRAIT_0 = 0,   /**< 竖屏  0度(默认) */
    ST7789_DIR_LANDSCAPE_90 = 1, /**< 横屏 90度(左转) */
    ST7789_DIR_PORTRAIT_180 = 2, /**< 竖屏180度(倒转) */
    ST7789_DIR_LANDSCAPE_270 = 3 /**< 横屏270度(右转) */
} st7789_dir_t;

/* ======================== 颜色定义 (RGB565格式) ======================== */
/** @defgroup LCD_Color 液晶屏颜色定义(RGB565)
 * @details RGB565格式: 5位红色 + 6位绿色 + 5位蓝色 = 16位颜色
 * @{
 */

#define ST7789_COLOR_WHITE 0xFFFF     /**< 白色 */
#define ST7789_COLOR_BLACK 0x0000     /**< 黑色 */
#define ST7789_COLOR_BLUE 0x001F      /**< 蓝色 */
#define ST7789_COLOR_BRED 0xF81F      /**< 棕红色 */
#define ST7789_COLOR_GRED 0xFFE0      /**< 绿红色 */
#define ST7789_COLOR_GBLUE 0x07FF     /**< 绿蓝色 */
#define ST7789_COLOR_RED 0xF800       /**< 红色 */
#define ST7789_COLOR_MAGENTA 0xF81F   /**< 品红色 */
#define ST7789_COLOR_GREEN 0x07E0     /**< 绿色 */
#define ST7789_COLOR_CYAN 0x7FFF      /**< 青色 */
#define ST7789_COLOR_YELLOW 0xFFE0    /**< 黄色 */
#define ST7789_COLOR_GRAY 0x8430      /**< 灰色 */
#define ST7789_COLOR_ORANGE 0xFD20    /**< 橙色 */
#define ST7789_COLOR_PINK 0xF8B2      /**< 粉红色 */
#define ST7789_COLOR_PURPLE 0x8010    /**< 紫色 */
#define ST7789_COLOR_BROWN 0xA145     /**< 棕色 */
#define ST7789_COLOR_DARKBLUE 0x01CF  /**< 深蓝色 */
#define ST7789_COLOR_LIGHTBLUE 0x7D7C /**< 浅蓝色 */

/** @} */

/* ======================== 公开 API 函数声明 ======================== */

/**
 * @brief 初始化液晶显示驱动器
 * @param 无
 * @return 无
 * @note 该函数应在main函数启动时调用一次，初始化GPIO、硬件复位、寄存器配置
 */
void st7789_init(void);

/**
 * @brief 设置液晶屏显示方向
 * @param dir 显示方向枚举值(ST7789_DIR_PORTRAIT_0/90/180/270)
 * @return 无
 * @note 支持竖屏和横屏显示，会自动交换宽高参数
 */
void st7789_set_direction(st7789_dir_t dir);

/**
 * @brief 用指定颜色清空整个液晶屏幕
 * @param color RGB565格式颜色值(如ST7789_COLOR_BLACK)
 * @return 无
 * @note 常用于初始化屏幕或清除旧内容
 */
void st7789_clear(uint16_t color);

/**
 * @brief 在指定坐标处绘制一个像素点
 * @param x 像素X坐标(0~屏幕宽度-1)
 * @param y 像素Y坐标(0~屏幕高度-1)
 * @param color RGB565格式颜色值
 * @return 无
 * @note 效率较低，不建议用于大面积绘制
 */
void st7789_draw_pixel(uint16_t x, uint16_t y, uint16_t color);

/**
 * @brief 用指定颜色填充矩形区域
 * @param x0 矩形左上角X坐标
 * @param y0 矩形左上角Y坐标
 * @param x1 矩形右下角X坐标
 * @param y1 矩形右下角Y坐标
 * @param color RGB565格式填充颜色
 * @return 无
 * @note 高效的矩形填充，支持自动边界裁剪
 */
void st7789_fill_rect(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color);

/**
 * @brief 使用Bresenham算法绘制直线
 * @param x0 直线起点X坐标
 * @param y0 直线起点Y坐标
 * @param x1 直线终点X坐标
 * @param y1 直线终点Y坐标
 * @param color RGB565格式直线颜色
 * @return 无
 * @note 支持任意方向的直线，算法高效
 */
void st7789_draw_line(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color);

/**
 * @brief 绘制RGB565格式的位图图像
 * @param x 图像左上角X坐标
 * @param y 图像左上角Y坐标
 * @param w 图像宽度(像素)
 * @param h 图像高度(像素)
 * @param data 指向RGB565格式图像数据的指针(2字节/像素)
 * @return 无
 * @note 用于显示预制的位图图像(LOGO、图标等)
 */
void st7789_draw_bitmap565(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint8_t *data);

/**
 * @brief 在指定位置绘制单个ASCII字符
 * @param x 字符左上角X坐标
 * @param y 字符左上角Y坐标
 * @param c 要绘制的ASCII字符
 * @param color RGB565格式字符颜色
 * @param bg RGB565格式背景颜色
 * @param scale 字符缩放倍数(1~4，1倍=5x7像素)
 * @return 无
 * @note 使用5x7点阵字体，支持1~4倍缩放
 */
void st7789_draw_char(uint16_t x, uint16_t y, char c, uint16_t color, uint16_t bg, uint8_t scale);

/**
 * @brief 在指定位置显示字符串(支持自动换行)
 * @param x 字符串起始X坐标
 * @param y 字符串起始Y坐标
 * @param str 指向C字符串的指针(以\\0结尾)
 * @param color RGB565格式字符颜色
 * @param bg RGB565格式背景颜色
 * @param scale 字符缩放倍数(1~4)
 * @return 无
 * @note 超出屏幕右边界时自动换行，超出下边界时停止显示
 */
void st7789_draw_string(uint16_t x, uint16_t y, const char *str, uint16_t color, uint16_t bg, uint8_t scale);

/**
 * @brief 在指定位置显示整数(支持定长显示、前置补零)
 * @param x 显示的X坐标
 * @param y 显示的Y坐标
 * @param num 要显示的整数值(int32_t范围)
 * @param len 显示宽度(不足时前置补零，最大15位)
 * @param color RGB565格式文字颜色
 * @param bg RGB565格式背景颜色
 * @param scale 字符缩放倍数(1~4)
 * @return 无
 * @note 例：st7789_show_int(10, 20, 42, 3, ...) 显示为"042"
 */
void st7789_show_int(uint16_t x, uint16_t y, int32_t num, uint8_t len, uint16_t color, uint16_t bg, uint8_t scale);

/**
 * @brief 在指定位置显示浮点数(支持指定小数位数)
 * @param x 显示的X坐标
 * @param y 显示的Y坐标
 * @param num 要显示的浮点数值(float类型)
 * @param len 整数部分显示宽度(最大10位)
 * @param frac 小数位数(最多6位，如3表示显示3位小数)
 * @param color RGB565格式文字颜色
 * @param bg RGB565格式背景颜色
 * @param scale 字符缩放倍数(1~4)
 * @return 无
 * @note 例：st7789_show_float(10, 20, 3.14159, 1, 4, ...) 显示为"3.1416"
 */
void st7789_show_float(uint16_t x, uint16_t y, float num, uint8_t len, uint8_t frac, uint16_t color, uint16_t bg, uint8_t scale);

/**
 * @brief 绘制进度条的矩形边框(不含填充)
 * @param x 进度条左上角X坐标
 * @param y 进度条左上角Y坐标
 * @param w 进度条宽度(像素)
 * @param h 进度条高度(像素)
 * @param color RGB565格式边框颜色
 * @return 无
 * @note 通常与st7789_update_progress_bar()配合使用
 */
void st7789_draw_progress_bar_border(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);

/**
 * @brief 更新进度条的填充长度(只重绘改变部分，高效)
 * @param x 进度条左上角X坐标
 * @param y 进度条左上角Y坐标
 * @param w 进度条宽度(像素)
 * @param h 进度条高度(像素)
 * @param last_percent 上一次的进度百分比(0~100%)
 * @param now_percent 当前的进度百分比(0~100%)
 * @param f_color RGB565格式前景色(填充颜色)
 * @param b_color RGB565格式背景色(未填充颜色)
 * @return 无
 * @note 例：从0%更新到50%时，只重绘0~50%范围，性能优化
 */
void st7789_update_progress_bar(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                                uint8_t last_percent, uint8_t now_percent,
                                uint16_t f_color, uint16_t b_color);

/**
 * @brief 获取当前液晶屏显示宽度
 * @param 无
 * @return 当前显示宽度(像素)
 * @note 该值随显示方向改变而改变(竖屏240，横屏320)
 */
uint16_t st7789_width(void);

/**
 * @brief 获取当前液晶屏显示高度
 * @param 无
 * @return 当前显示高度(像素)
 * @note 该值随显示方向改变而改变(竖屏320，横屏240)
 */
uint16_t st7789_height(void);

/**
 * @brief LCD驱动功能演示程序(测试用)
 * @param 无
 * @return 无
 * @note 演示清屏、文本、数字、进度条、颜色等所有功能，约需3秒完成
 */
void st7789_demo(void);

#endif
