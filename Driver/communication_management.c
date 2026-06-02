//
// Created by E_LJF on 25-7-18.
//

#include "communication_management.h"

#include "ESHL_driver.h"
#include "ESHL_protocol.h"
#include "usart.h"
#include "crc.h"
#include <stdio.h>

/* ── 全局变量 ─────────────────────────────────────────── */

ESHL_PROTOCOL_PACK_ANALYSIS_T send_str =//电调数据发送结构体
{
    .head = 0,
    .addr = 0,
    .cmd = 0,
    .data = 0,
    .addr_dat = 0,
    .current_limit = 0,
    .throttle = 0
};

ESHL_PROTOCOL_PACK_ANALYSIS_T recv_str =//电调数据接收结构体
{
    .head = 0,
    .addr = 0,
    .cmd = 0,
    .data = 0,
    .addr_dat = 0,
    .current_limit = 0,
    .throttle = 0
};

uint8_t rx_buff[ESHL_RX_PACK_LEN] = "";     //电调数据接收缓存
uint8_t ESHL_RXPack[ESHL_RX_PACK_LEN] = ""; //电调数据接收包
uint8_t ESHL_TXPack[ESHL_TX_PACK_LEN] = ""; //电调数据发送包

uint16_t ESHL_RunPWMBuff = 800;   //电调运行pwm缓存

uint8_t uart_updated_flag = 0;//串口是否收到数据标志,1为收到数据

/* ── 解码上下文 ────────────────────────────────────────── */
// 处理当前包时,记录原始数据指针,供0xC9等命令读取payload
static uint8_t *recv_raw_pkt = NULL;

/* ── 下行: 清空接收结构体 ─────────────────────────────── */

static void ESHL_ResetRecvStr(ESHL_PROTOCOL_PACK_ANALYSIS_T *p)
{
    p->head          = 0;
    p->addr          = 0;
    p->len           = 0;
    p->cmd           = 0;
    p->data          = 0;
    p->addr_dat      = 0;
    p->current_limit = 0;
    p->throttle      = 0;
}

/* ── ISR 回调 ─────────────────────────────────────────── */

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART1) {
        uart_updated_flag = 1;

        for (uint16_t i = 0; i < Size; i++) {
            ESHL_RXPack[i] = rx_buff[i]; //复制数据
        }

        if (ESHL_RX_PACK_LEN > Size) {//数据包数组还有空位
            for (uint16_t i = Size; i < ESHL_RX_PACK_LEN; i++) {
                ESHL_RXPack[i] = 0;//空位填0,防止读到上一次数据
            }
        }

        // 全双工模式下，直接重新开启接收，不用管发送状态
        HAL_UARTEx_ReceiveToIdle_DMA(&ESHL_UART, (uint8_t *)rx_buff, ESHL_RX_PACK_LEN);
        __HAL_DMA_DISABLE_IT(ESHL_UART.hdmarx, DMA_IT_HT);//关闭DMA半满中断
    }
}


void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        uart_updated_flag = 0;
        HAL_UARTEx_ReceiveToIdle_DMA(&ESHL_UART, (uint8_t *)rx_buff, ESHL_RX_PACK_LEN);//开启串口通信
        __HAL_DMA_DISABLE_IT(ESHL_UART.hdmarx, DMA_IT_HT);//关闭DMA半满中断
    }
}


//开始通信
void ESHL_CommunicationStart() {
    uart_updated_flag = 0;//清除标志
    // 全双工不需要 HAL_HalfDuplex_EnableReceiver，直接开启 DMA 即可
    HAL_UARTEx_ReceiveToIdle_DMA(&ESHL_UART, (uint8_t *)rx_buff, ESHL_RX_PACK_LEN);
    __HAL_DMA_DISABLE_IT(ESHL_UART.hdmarx, DMA_IT_HT);//关闭DMA半满中断
}


//停止通信
void ESHL_CommunicationStop() {
    uart_updated_flag = 0; // 清除标志
    // 仅使用 HAL_UART_Abort，它会安全地中止串口接收和对应的 DMA 传输
    HAL_UART_Abort(&ESHL_UART);
}

/* ── 下行: 从包中解析各字段 ───────────────────────────── */

static uint8_t ESHL_ParsePayload(uint8_t *pack, ESHL_PROTOCOL_PACK_ANALYSIS_T *p)
{
    p->cmd = pack[4];//命令码

    switch (p->cmd) {

        case ESHL_PROTOCOL_CMD_THR://油门命令
            uint8_to_float(&pack[5], &p->throttle);
            break;

        case ESHL_PROTOCOL_CMD_ESC_OFF://关闭电调
            p->data = pack[5];
            break;

        case ESHL_PROTOCOL_CMD_ESC_ON://启动电调
            p->data = pack[5];
            break;

        case ESHL_PROTOCOL_CMD_BREAK://电调刹车
            p->data = pack[5];
            break;

        case ESHL_PROTOCOL_CMD_CURRENT_LIMIT://更改电调运行电流限制ADC值
            p->current_limit = (pack[5] << 8) | pack[6];
            break;

        case ESHL_PROTOCOL_CMD_ERROR://电调异常
            p->data = pack[5];
            break;

        case ESHL_PROTOCOL_CMD_CHANGE_ADDR://更改电调地址
            p->addr_dat = (pack[5] << 8) | pack[6];
            break;

        case ESHL_PROTOCOL_CMD_HELP://帮助 — 无payload
        case ESHL_PROTOCOL_CMD_CHECKSUM://算合校验 — payload原样
            break;

        default:
            return 0;
    }
    return 1;
}

/* ── 命令处理及响应 ───────────────────────────────────── */

static void ESHL_CMDProcessing(ESHL_PROTOCOL_PACK_ANALYSIS_T *pkt)
{
    ESHL_STATE_ENUM_T state = ESHL_GetState();

    switch (pkt->cmd) {

        case ESHL_PROTOCOL_CMD_THR://油门命令
            if (state == ESHL_STATE_RUN_CLOCKWISE || state == ESHL_STATE_RUN_COUNTER_CLOCKWISE) {
                ESHL_RunPWMBuff = (uint16_t)(pkt->throttle * ESHL_RUN_MAX_PWM);
            }
            break;

        case ESHL_PROTOCOL_CMD_ESC_OFF: //关闭电调
            ESHL_SetState(ESHL_STATE_OFF);
            break;

        case ESHL_PROTOCOL_CMD_ESC_ON: //启动电调
            if ((state == ESHL_STATE_OFF) || (state == EShl_STATE_READY)) {
                if (pkt->data == 0xA1) {
                    ESHL_RunPWMBuff = ESHL_RUN_MIN_PWM;
                    ESHL_SetDirection(ESHL_CLOCKWISE);
                    ESHL_SetState(ESHL_STATE_START);
                }
                else if (pkt->data == 0xA2) {
                    ESHL_RunPWMBuff = ESHL_RUN_MIN_PWM;
                    ESHL_SetDirection(ESHL_COUNTER_CLOCKWISE);
                    ESHL_SetState(ESHL_STATE_START);
                }
            }
            break;

        case ESHL_PROTOCOL_CMD_BREAK://电调刹车
            if (state == ESHL_STATE_RUN_CLOCKWISE || state == ESHL_STATE_RUN_COUNTER_CLOCKWISE) {
                ESHL_CloseMOSComp();
                ESHL_SetState(ESHL_STATE_BRAKE);
            }
            break;

        case ESHL_PROTOCOL_CMD_CURRENT_LIMIT://更改电调运行电流限制ADC值
            if ((state == ESHL_STATE_OFF) || (state == EShl_STATE_READY)) {
                ESHL_ChangeRuningCurrentLimit(pkt->current_limit);
                ESHL_CommunicationSendCode(ESHL_PROTOCOL_CMD_CURRENT_LIMIT, 0xAA);
            }
            break;

        case ESHL_PROTOCOL_CMD_CHANGE_ADDR: //更改电调地址
            if ((state == ESHL_STATE_OFF) || (state == EShl_STATE_READY)) {
                ESHL_CloseMOSComp();
                ESHL_SetState(ESHL_STATE_SET_ADDR);
            }
            break;

        case ESHL_PROTOCOL_CMD_HELP: // 0xC8 — 帮助 (printf 输出)
            printf("\r\n=== ESHL CMD ===\r\n");
            printf("C1 thr(float)\r\n");
            printf("C2 off\r\n");
            printf("C3 on A1=CW A2=CCW\r\n");
            printf("C4 brake\r\n");
            printf("C5 curlim(u16)\r\n");
            printf("C6 err\r\n");
            printf("C7 setaddr(bcast)\r\n");
            printf("C8 help\r\n");
            printf("C9 chksum(raw)\r\n");
            break;

        case ESHL_PROTOCOL_CMD_CHECKSUM: // 0xC9 — 算合校验 (printf 返回)
            {
                if (recv_raw_pkt != NULL && pkt->len >= 6)
                {
                    uint8_t payload_len = pkt->len - 6;
                    if (payload_len >= 1)
                    {
                        uint8_t sum = get_checksum(&recv_raw_pkt[5], payload_len);
                        printf("checksum: [");
                        for (uint8_t i = 0; i < payload_len; i++)
                        {
                            printf("%02X ", recv_raw_pkt[5 + i]);
                        }
                        printf("] = 0x%02X\r\n", sum);
                    }
                }
            }
            break;

        default:
            break;
    }
}

/* ── 通信数据处理 (新—状态机扫描) ─────────────────────── */

void ESHL_CommunicationDataProcessing()
{
    uint16_t myAddr = ESHL_GetAddr();
    uint8_t *buf = ESHL_RXPack;

    // 如果没有收到新数据, 跳过
    if (!uart_updated_flag) {
        return;
    }
    uart_updated_flag = 0; // 消费标志
    
    // 【修改】：注释掉了这部分 printf，防止破坏二进制通信总线导致乱码
    /*
    printf("Received data: ");
    for (int i = 0; i < ESHL_RX_PACK_LEN; i++) {
        printf("%02X ", buf[i]);
    }
    printf("\r\n");
    */

    uint16_t pos = 0;
    while (pos <= ESHL_RX_PACK_LEN - 5) // 至少: 帧头1+地址2+长度1+命令1 = 5
    {
        // ① 帧头比对 (最廉价)
        if (buf[pos] != ESHL_PROTOCOL_HEAD_HOST_TO_ESC) {
            pos++;
            continue;
        }

        // ② 读取长度并校验合法性
        uint8_t pkt_len = buf[pos + 3];
        if (pkt_len < 5 || pkt_len > ESHL_DATA_PACK_MAX_LEN) {
            pos++;
            continue;
        }

        // ③ 缓冲区越界检查 (不完整包, 等待更多数据)
        if (pos + pkt_len > ESHL_RX_PACK_LEN) {
            break;
        }

        // ④ 地址匹配 (广播或本机)
        uint16_t pkt_addr = ((uint16_t)buf[pos + 2] << 8) | buf[pos + 1];
        uint8_t cmd = buf[pos + 4]; // 提前读取命令码

        if (pkt_addr != ESHL_PROTOCOL_BROADCAST_ADDR && pkt_addr != myAddr) {
            pos++;
            continue;
        }

        // ⑤ 记录原始包指针 (供0xC9等命令使用)
        recv_raw_pkt = &buf[pos];

        // ⑥ 读命令码, 判断是否为调试命令
        if (cmd == ESHL_PROTOCOL_CMD_HELP || cmd == ESHL_PROTOCOL_CMD_CHECKSUM) {
            // 调试命令 — 跳过合校验, 直接处理
            ESHL_ResetRecvStr(&recv_str);
            recv_str.head = buf[pos];
            recv_str.addr = pkt_addr;
            recv_str.len  = pkt_len;
            ESHL_ParsePayload(&buf[pos], &recv_str);
            ESHL_CMDProcessing(&recv_str);
            pos += pkt_len;
        }
        // ⑦ 普通命令 — 必须通过合校验
        else if (verify_checksum(&buf[pos], pkt_len)) {
            ESHL_ResetRecvStr(&recv_str);
            recv_str.head = buf[pos];
            recv_str.addr = pkt_addr;
            recv_str.len  = pkt_len;
            ESHL_ParsePayload(&buf[pos], &recv_str);
            ESHL_CMDProcessing(&recv_str);
            pos += pkt_len;
        }
        else {
            // 合校验失败, 跳过这一字节继续扫描
            pos++;
        }
    }
}

/* ── 发送函数 (极简全双工版本) ──────────────────────────────────── */

//广播电调地址
void ESHL_CommunicationAddressSend() {
    uint8_t packet[10] = "";
    ESHL_PROTOCOL_PACK_ANALYSIS_T send_str;

    send_str.cmd = ESHL_PROTOCOL_CMD_CHANGE_ADDR;
    send_str.addr = ESHL_PROTOCOL_BROADCAST_ADDR;
    send_str.data = 0;
    send_str.addr_dat = ESHL_GetAddr();

    ESHL_ProtocolPackMake(packet, &send_str, ESHL_DATA_ESC_TO_HOST);//数据打包
    
    // 【修改】：全双工模式下直接发送，不需要停掉接收，互不干扰！
    HAL_UART_Transmit(&ESHL_UART, (uint8_t*)packet, 10, 100); 
}

//向主机发送一字节数据
void ESHL_CommunicationSendCode(uint8_t cmd, uint8_t code) {
    uint8_t packet[10] = "";
    ESHL_PROTOCOL_PACK_ANALYSIS_T send_str;

    send_str.cmd = cmd;
    send_str.addr = ESHL_GetAddr();
    send_str.data = code;
    send_str.addr_dat = 0;

    ESHL_ProtocolPackMake(packet, &send_str, ESHL_DATA_ESC_TO_HOST); 
    
    // 【修改】：全双工模式下直接发送，不需要停掉接收，互不干扰！
    HAL_UART_Transmit(&ESHL_UART, (uint8_t*)packet, 10, 100); 
}