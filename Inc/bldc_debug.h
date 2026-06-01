#ifndef __BLDC_DEBUG_H
#define __BLDC_DEBUG_H

#include <stdint.h>

/* ========================================================================== */
/* 1. 编译配置与日志级别定义                                                   */
/* ========================================================================== */
#define LOG_LEVEL_NONE  0
#define LOG_LEVEL_ERROR 1
#define LOG_LEVEL_WARN  2
#define LOG_LEVEL_INFO  3
#define LOG_LEVEL_DEBUG 4

/* 当前全局日志级别配置 (可根据开发阶段修改，例如最终发布改为 LOG_LEVEL_WARN) */
#define LOG_LEVEL       LOG_LEVEL_DEBUG

/* ========================================================================== */
/* 2. 调试串口相关参数及外部声明                                               */
/* ========================================================================== */
#define RX_BUF_SIZE     128  /* 串口接收缓冲区大小 */

extern uint8_t g_rx_buffer[RX_BUF_SIZE];
extern volatile uint8_t g_rx_ready_flag;
extern volatile uint16_t g_rx_data_len;

/* 串口公共接口 */
void BSP_DEBUG_Init(uint32_t baudrate);
void BSP_DEBUG_PutChar(char c);
void logxxx(const char *fmt, ...);
void BSP_DEBUG_ProcessRx(void);  /* 主循环帧解析函数 */
void BSP_DEBUG_DispatchCmd(uint16_t *p_target_pwm);  /* 命令派发 */

/* ========================================================================== */
/* 4. 串口命令解析接口                                                         */
/* ========================================================================== */

#define FRAME_HEADER    0xAA    /* 二进制帧帧头 */

typedef enum {
    CMD_NONE = 0,       /* 无待处理命令 */
    CMD_START,          /* 启动电机 (0x01) */
    CMD_STOP,           /* 停止电机 (0x02) */
    CMD_SET_SPEED,      /* 设置速度 (0x03) */
    CMD_STATUS,         /* 查询状态 (0x04) */
    CMD_DIRECTION,      /* 切换方向 (0x05) */
    CMD_HELP            /* 帮助列表 (0x06) */
} CMD_ID_T;

typedef struct {
    volatile CMD_ID_T id;   /* 命令ID (volatile 防止编译器优化) */
    int32_t          param; /* 命令参数 (如速度值) */
} CMD_Result_T;

extern volatile CMD_Result_T g_cmd_result;

/* ========================================================================== */
/* 3. 保存的分级日志宏定义块                                                   */
/* ========================================================================== */
#if LOG_LEVEL >= LOG_LEVEL_ERROR
#define LOG_ERROR(fmt, ...) logxxx("[E] " fmt, ##__VA_ARGS__)
#else
#define LOG_ERROR(fmt, ...) ((void)0)
#endif

#if LOG_LEVEL >= LOG_LEVEL_WARN
#define LOG_WARN(fmt, ...)  logxxx("[W] " fmt, ##__VA_ARGS__)
#else
#define LOG_WARN(fmt, ...)  ((void)0)
#endif

#if LOG_LEVEL >= LOG_LEVEL_INFO
#define LOG_INFO(fmt, ...)  logxxx("[I] " fmt, ##__VA_ARGS__)
#else
#define LOG_INFO(fmt, ...)  ((void)0)
#endif

#if LOG_LEVEL >= LOG_LEVEL_DEBUG
#define LOG_DEBUG(fmt, ...) logxxx("[D] " fmt, ##__VA_ARGS__)
#else
#define LOG_DEBUG(fmt, ...) ((void)0)
#endif

#endif /* __BLDC_DEBUG_H */
