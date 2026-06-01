#include "bldc_debug.h"
#include "bldc_driver.h"
#include "bldc_adc.h"
#include "stm32f0xx.h"
#include <stdarg.h>

/* 全局变量定义 */
uint8_t g_rx_buffer[RX_BUF_SIZE];
volatile uint8_t g_rx_ready_flag = 0;
volatile uint16_t g_rx_data_len = 0;

/* 内部静态指针：记录当前中断收到了第几个字节 */
static volatile uint16_t s_rx_cnt = 0;

/* ====================================================================== */
/* 5. 串口命令解析全局变量                                                  */
/* ====================================================================== */
volatile CMD_Result_T g_cmd_result = {CMD_NONE, 0};

/* ========================================================================== */
/* 1. 内部 Printf 底层：数值/字符串/浮点数处理                                 */
/* ========================================================================== */
static void print_int(int32_t val, int base, int is_signed)
{
    char buf[12];
    char *p = buf + sizeof(buf) - 1;
    *p = '\0';

    uint32_t uval = (is_signed && val < 0) ? (uint32_t)(-val) : (uint32_t)val;
    int abs_base = (base < 0) ? -base : base;
    const char *digits = (base < 0) ? "0123456789ABCDEF" : "0123456789abcdef";

    do {
        *(--p) = digits[uval % (uint32_t)abs_base];
        uval /= (uint32_t)abs_base;
    } while (uval != 0u);

    if (is_signed && val < 0) *(--p) = '-';
    while (*p) BSP_DEBUG_PutChar(*p++);
}

static void print_str(const char *s)
{
    if (!s) s = "(null)";
    while (*s) BSP_DEBUG_PutChar(*s++);
}

static void print_float(float val, int precision)
{
    if (val < 0.0f)
    {
        BSP_DEBUG_PutChar('-');
        val = -val;
    }
    int32_t int_part = (int32_t)val;
    print_int(int_part, 10, 1);
    BSP_DEBUG_PutChar('.');
    float frac_part = val - (float)int_part;
    for (int i = 0; i < precision; i++)
    {
        frac_part *= 10.0f;
        int32_t digit = (int32_t)frac_part;
        BSP_DEBUG_PutChar((char)('0' + digit));
        frac_part -= (float)digit;
    }
}

void logxxx(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    while (*fmt) {
        if (*fmt != '%') {
            BSP_DEBUG_PutChar(*fmt++);
            continue;
        }
        fmt++;
        if (*fmt == '\0') break;

        switch (*fmt++) {
            case 'd': print_int(va_arg(args, int32_t),  10, 1); break;
            case 'u': print_int(va_arg(args, uint32_t), 10, 0); break;
            case 'x': print_int((uint32_t)va_arg(args, int32_t), 16, 0); break;
            case 'X': print_int((uint32_t)va_arg(args, int32_t), -16, 0); break;
            case 'c': BSP_DEBUG_PutChar((char)va_arg(args, int)); break;
            case 's': print_str(va_arg(args, const char*)); break;
            case 'f': print_float((float)va_arg(args, double), 4); break;
            case '%': BSP_DEBUG_PutChar('%'); break;
            default:  BSP_DEBUG_PutChar('?'); break;
        }
    }
    va_end(args);
}

/* ========================================================================== */
/* 2. 硬件底层配置：USART1 中断接收 (非DMA)                                     */
/* ========================================================================== */
void BSP_DEBUG_Init(uint32_t baudrate)
{
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    RCC->AHBENR  |= RCC_AHBENR_GPIOAEN;

    GPIOA->MODER &= ~(GPIO_MODER_MODER9_Msk | GPIO_MODER_MODER10_Msk);
    GPIOA->MODER |= (0x2u << GPIO_MODER_MODER9_Pos) | (0x2u << GPIO_MODER_MODER10_Pos);

    GPIOA->AFR[1] &= ~(GPIO_AFRH_AFSEL9_Msk | GPIO_AFRH_AFSEL10_Msk);
    GPIOA->AFR[1] |= (0x1u << GPIO_AFRH_AFSEL9_Pos) | (0x1u << GPIO_AFRH_AFSEL10_Pos);

    GPIOA->OSPEEDR |= (0x3u << GPIO_OSPEEDR_OSPEEDR9_Pos);
    GPIOA->PUPDR   &= ~(GPIO_PUPDR_PUPDR9_Msk | GPIO_PUPDR_PUPDR10_Msk);
    GPIOA->PUPDR   |= (0x1u << GPIO_PUPDR_PUPDR10_Pos);

    USART1->BRR = (SystemCoreClock + (baudrate / 2u)) / baudrate;
    USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE | USART_CR1_IDLEIE | USART_CR1_UE;
    USART1->CR3 &= ~USART_CR3_DMAR;
    NVIC_SetPriority(USART1_IRQn, 2);
    NVIC_EnableIRQ(USART1_IRQn);
}

void BSP_DEBUG_PutChar(char c)
{
    while (!(USART1->ISR & USART_ISR_TXE_Msk)) {}
    USART1->TDR = (uint8_t)c;
}

/* ========================================================================== */
/* 3. 双中断混合方案 (RXNE 逐字节 + IDLE 帧结束)                                */
/* ========================================================================== */
void USART1_IRQHandler(void)
{
    uint32_t isr_status = USART1->ISR;

    /* ---- 事件 1：接收非空中断 ---- */
    if (isr_status & USART_ISR_RXNE)
    {
        uint8_t received_byte = (uint8_t)(USART1->RDR);

        if (g_rx_ready_flag == 0)
        {
            if (s_rx_cnt < RX_BUF_SIZE)
            {
                g_rx_buffer[s_rx_cnt++] = received_byte;
            }
            else
            {
                s_rx_cnt = 0;
            }
        }
    }

    /* ---- 事件 2：总线空闲中断 ---- */
    if (isr_status & USART_ISR_IDLE)
    {
        USART1->ICR = USART_ICR_IDLECF | USART_ICR_ORECF;

        if (s_rx_cnt > 0)
        {
            g_rx_data_len = s_rx_cnt;
            g_rx_ready_flag = 1;
            s_rx_cnt = 0;
        }
    }

    /* ---- 事件 3：溢出错误处理 ---- */
    if (isr_status & USART_ISR_ORE)
    {
        USART1->ICR = USART_ICR_ORECF;
    }
}

/* ========================================================================== */
/* 4a. 辅助函数：十六进制字符转半字节                                          */
/* ========================================================================== */
static uint8_t hex_nibble(char c)
{
    if (c >= '0' && c <= '9') return (uint8_t)(c - '0');
    if (c >= 'A' && c <= 'F') return (uint8_t)(c - 'A' + 10);
    if (c >= 'a' && c <= 'f') return (uint8_t)(c - 'a' + 10);
    return 0xFF; /* 非法字符 */
}

/* ========================================================================== */
/* 4b. 二进制帧解析 + 文本校验计算器                                          */
/* ========================================================================== */
void BSP_DEBUG_ProcessRx(void)
{
    CMD_ID_T cmd_id = CMD_NONE;
    int32_t  param  = 0;
    uint8_t  cmd;

    /* 没有新数据 */
    if (!g_rx_ready_flag) return;

    /* 上一个命令还没被 main.c 消费，丢弃新数据 */
    if (g_cmd_result.id != CMD_NONE) {
        g_rx_ready_flag = 0;
        g_rx_data_len = 0;
        return;
    }

    /* ---- 文本模式 XOR 校验计算器 ---- */
    /* 以 $ 开头的帧视为校验计算请求，例如: "$ AA 03 01 F4" */
    if (g_rx_data_len >= 2 && g_rx_buffer[0] == '$') {
        uint8_t xor_sum = 0;
        uint16_t i = 1;
        while (i < g_rx_data_len) {
            /* 跳过空白符 */
            while (i < g_rx_data_len && (g_rx_buffer[i] == ' '  || g_rx_buffer[i] == '\t'
                                      || g_rx_buffer[i] == '\r' || g_rx_buffer[i] == '\n'))
                i++;
            if (i + 1 >= g_rx_data_len) break;
            uint8_t hi = hex_nibble((char)g_rx_buffer[i]);
            uint8_t lo = hex_nibble((char)g_rx_buffer[i + 1]);
            if (hi <= 0x0F && lo <= 0x0F) {
                xor_sum ^= (uint8_t)((hi << 4) | lo);
                i += 2;
            } else {
                i++;
            }
        }
        LOG_INFO("校验和: 0x%X\r\n", xor_sum);
        goto clear;
    }

    /* 最小帧: 帧头(1) + 命令(1) + 校验(1) = 3 字节 */
    if (g_rx_data_len < 3) goto clear;

    /* XOR 校验：所有字节异或结果应为 0 */
    {
        uint8_t xor_sum = 0;
        for (uint16_t i = 0; i < g_rx_data_len; i++)
            xor_sum ^= g_rx_buffer[i];
        if (xor_sum != 0) goto clear;
    }

    /* 检查帧头 */
    if (g_rx_buffer[0] != FRAME_HEADER) goto clear;

    cmd = g_rx_buffer[1];

    /* 根据命令字解析 */
    switch (cmd) {
        case 0x01:
            cmd_id = CMD_START;
            LOG_INFO("收到命令: 启动\r\n");
            break;

        case 0x02:
            cmd_id = CMD_STOP;
            LOG_INFO("收到命令: 停止\r\n");
            break;

        case 0x03:  /* 设置速度: 2字节参数, 大端序 */
            if (g_rx_data_len >= 5) {
                param = (g_rx_buffer[2] << 8) | g_rx_buffer[3];
                cmd_id = CMD_SET_SPEED;
                LOG_INFO("收到命令: 设置速度 %d\r\n", (int32_t)param);
            }
            break;

        case 0x04:
            cmd_id = CMD_STATUS;
            LOG_INFO("收到命令: 查询状态\r\n");
            break;

        case 0x05:
            cmd_id = CMD_DIRECTION;
            LOG_INFO("收到命令: 切换方向\r\n");
            break;

        case 0x06:
            /* 帮助命令直接回应，不经过 main.c */
            LOG_INFO("\r\n命令列表:\r\n");
            LOG_INFO("  0x01 启动电机\r\n");
            LOG_INFO("  0x02 停止电机\r\n");
            LOG_INFO("  0x03 + 2B 设置速度(80~991)\r\n");
            LOG_INFO("  0x04 查询状态\r\n");
            LOG_INFO("  0x05 切换方向\r\n");
            LOG_INFO("  '$' + hex 计算校验和\r\n");
            LOG_INFO("  例: $ AA 03 01 F4\r\n");
            goto clear;

        default:
            LOG_INFO("未知命令: 0x%02X\r\n", cmd);
            goto clear;
    }

    g_cmd_result.id    = cmd_id;
    g_cmd_result.param = param;

clear:
    g_rx_ready_flag = 0;
    g_rx_data_len   = 0;
}

/* ========================================================================== */
/* 4c. 命令派发：根据解析结果控制电机状态机                                    */
/* ========================================================================== */
void BSP_DEBUG_DispatchCmd(uint16_t *p_target_pwm)
{
    ESHL_STATE_ENUM_T cur_state;

    if (g_cmd_result.id == CMD_NONE) {
        return;
    }

    cur_state = ESHL_GetState();

    switch (g_cmd_result.id) {
        case CMD_START:
            if (cur_state == EShl_STATE_READY) {
                ESHL_SetState(ESHL_STATE_START);
                LOG_INFO("电机启动中...\r\n");
            } else if (cur_state == ESHL_STATE_RUN_CLOCKWISE
                    || cur_state == ESHL_STATE_RUN_COUNTER_CLOCKWISE) {
                LOG_INFO("电机已在运行中\r\n");
            } else {
                LOG_INFO("当前状态无法启动电机\r\n");
            }
            break;
        case CMD_STOP:
            if (cur_state == ESHL_STATE_RUN_CLOCKWISE
                    || cur_state == ESHL_STATE_RUN_COUNTER_CLOCKWISE) {
                ESHL_CloseMOSComp();
                ESHL_SetState(ESHL_STATE_BRAKE);
                LOG_INFO("电机开始停止\r\n");
            } else {
                LOG_INFO("电机未在运行\r\n");
            }
            break;

        case CMD_SET_SPEED:
            if (g_cmd_result.param >= ESHL_RUN_MIN_PWM
                    && g_cmd_result.param <= ESHL_RUN_MAX_PWM) {
                if (p_target_pwm) {
                    *p_target_pwm = (uint16_t)g_cmd_result.param;
                }
                LOG_INFO("速度已设为 %d\r\n", (int32_t)g_cmd_result.param);
            } else {
                LOG_INFO("速度值无效，范围 %d~%d\r\n",
                         ESHL_RUN_MIN_PWM, ESHL_RUN_MAX_PWM);
            }
            break;

        case CMD_STATUS: {
                const char *state_str = "未知";
                switch (cur_state) {
                    case ESHL_STATE_OFF:              state_str = "关闭"; break;
                    case EShl_STATE_READY:            state_str = "就绪"; break;
                    case ESHL_STATE_START:            state_str = "启动中"; break;
                    case ESHL_STATE_OPEN_LOOP_START:  state_str = "开环启动"; break;
                    case ESHL_STATE_RUN_CLOCKWISE:    state_str = "顺时针运行"; break;
                    case ESHL_STATE_RUN_COUNTER_CLOCKWISE: state_str = "逆时针运行"; break;
                    case ESHL_STATE_BRAKE:            state_str = "刹车中"; break;
                    case ESHL_STATE_MOS_ERROR:        state_str = "MOS异常"; break;
                    case ESHL_STATE_CURRENT_ERROR:    state_str = "电流异常"; break;
                    case ESHL_STATE_BATTERY_VOLTAGE_ERROR: state_str = "电压异常"; break;
                    case ESHL_STATE_MOTOR_RUNING_STOP: state_str = "停转"; break;
                    default: break;
                }
                const char *dir_str = (ESHL_GetDirection() == ESHL_CLOCKWISE)
                                     ? "顺时针" : "逆时针";
                float vbat = ESHL_VBAT_CalcVoltage(g_adc_metrics.v_bus);
                LOG_INFO("状态: %s, 方向: %s, 目标速度: %d, 电压: %dV\r\n",
                         state_str, dir_str,
                         p_target_pwm ? (int32_t)*p_target_pwm : 0,
                         (int32_t)vbat);
            }
            break;

        case CMD_DIRECTION:
            if (cur_state == ESHL_STATE_RUN_CLOCKWISE
                    || cur_state == ESHL_STATE_RUN_COUNTER_CLOCKWISE) {
                LOG_INFO("请先停止电机再切换方向\r\n");
            } else {
                ESHL_DIRECTION_ENUM_T new_dir =
                    (ESHL_GetDirection() == ESHL_CLOCKWISE)
                    ? ESHL_COUNTER_CLOCKWISE : ESHL_CLOCKWISE;
                ESHL_SetDirection(new_dir);
                LOG_INFO("方向已切换为 %s\r\n",
                         (new_dir == ESHL_CLOCKWISE) ? "顺时针" : "逆时针");
            }
            break;

        default:
            break;
    }
    g_cmd_result.id = CMD_NONE; /* 命令已消费 */
}
