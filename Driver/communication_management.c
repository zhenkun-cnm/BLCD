//
// Created by E_LJF on 25-7-18.
//

#include "communication_management.h"

#include "ESHL_driver.h"
#include "ESHL_protocol.h"
#include "usart.h"

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

uint8_t data_updated_flag = 0;  //串口数据是否已更新标志,1为已更新
uint8_t uart_updated_flag = 0;//串口是否收到数据标志,1为收到数据


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {

    uart_updated_flag = 1;

    for (uint16_t i = 0; i < Size; i++) {
        ESHL_RXPack[i] = rx_buff[i]; //复制数据
    }

    if (ESHL_RX_PACK_LEN > Size) {//数据包数组还有空位
        for (uint16_t i = Size; i < ESHL_RX_PACK_LEN; i++) {
            ESHL_RXPack[i] = 0;//空位填0,防止读到上一次数据
        }
    }

    HAL_UARTEx_ReceiveToIdle_DMA(&ESHL_UART, (uint8_t *)rx_buff, ESHL_RX_PACK_LEN);//开启串口通信
    __HAL_DMA_DISABLE_IT(ESHL_UART.hdmarx,DMA_IT_HT);//关闭DMA半满中断
}


void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    uart_updated_flag = 0;
    HAL_UARTEx_ReceiveToIdle_DMA(&ESHL_UART, (uint8_t *)rx_buff, ESHL_RX_PACK_LEN);//开启串口通信
    __HAL_DMA_DISABLE_IT(ESHL_UART.hdmarx,DMA_IT_HT);//关闭DMA半满中断
}


//开始通信
void ESHL_CommunicationStart() {
    uart_updated_flag = 0;//清除标志
    HAL_HalfDuplex_EnableReceiver(&ESHL_UART);
    HAL_UARTEx_ReceiveToIdle_DMA(&ESHL_UART, (uint8_t *)rx_buff, ESHL_RX_PACK_LEN);//开启串口通信
    __HAL_DMA_DISABLE_IT(huart1.hdmarx,DMA_IT_HT);//关闭DMA半满中断
}


//停止通信
void ESHL_CommunicationStop() {
    uart_updated_flag = 0;//清除标志
    HAL_UART_Abort(&ESHL_UART);
    HAL_DMA_Abort_IT(ESHL_UART.hdmarx);
}


//命令处理及响应
static void ESHL_CMDProcessing() {

     ESHL_STATE_ENUM_T state = ESHL_GetState();

     switch (recv_str.cmd) {

         case ESHL_PROTOCOL_CMD_THR://油门命令

             if (state == ESHL_STATE_RUN_CLOCKWISE || state == ESHL_STATE_RUN_COUNTER_CLOCKWISE) {//运行状态中修改
                 ESHL_RunPWMBuff = (uint16_t) (recv_str.throttle * ESHL_RUN_MAX_PWM); //将油门百分比转为油门PWM值
             }
             break;

         case ESHL_PROTOCOL_CMD_ESC_OFF: //关闭电调
             ESHL_SetState(ESHL_STATE_OFF);//更新电调状态
             break;

         case ESHL_PROTOCOL_CMD_ESC_ON: //启动电调,数据0XA1表示顺时针启动,0XA2表示逆时针启动
             if ((state == ESHL_STATE_OFF) || (state == EShl_STATE_READY)) {
                 if (recv_str.data == 0XA1) {
                     ESHL_RunPWMBuff = ESHL_RUN_MIN_PWM;
                     ESHL_SetDirection(ESHL_CLOCKWISE);//方向设为顺时针
                     ESHL_SetState(ESHL_STATE_START);
                 }
                 else if (recv_str.data == 0XA2) {
                     ESHL_RunPWMBuff = ESHL_RUN_MIN_PWM;
                     ESHL_SetDirection(ESHL_COUNTER_CLOCKWISE);//方向设为逆时针
                     ESHL_SetState(ESHL_STATE_START);
                 }
             }
             break;

         case ESHL_PROTOCOL_CMD_BREAK://电调刹车
             if (state == ESHL_STATE_RUN_CLOCKWISE || state == ESHL_STATE_RUN_COUNTER_CLOCKWISE) {//运行状态生效
                 ESHL_CloseMOSComp();
                 ESHL_SetState(ESHL_STATE_BRAKE);//更新电调状态
             }
             break;

         case ESHL_PROTOCOL_CMD_CURRENT_LIMIT://更改电调运行电流限制ADC值
             if ((state == ESHL_STATE_OFF) || (state == EShl_STATE_READY)) {//电调非运动状态才能修改电流限制
                 ESHL_ChangeRuningCurrentLimit(recv_str.current_limit);
                 ESHL_CommunicationSendCode(ESHL_PROTOCOL_CMD_CURRENT_LIMIT,0xAA);
             }
             break;

         case ESHL_PROTOCOL_CMD_CHANGE_ADDR: //更改电调地址
             if ((state == ESHL_STATE_OFF) || (state == EShl_STATE_READY)) {
                 ESHL_CloseMOSComp();
                 ESHL_SetState(ESHL_STATE_SET_ADDR);//更新电调状态
             }
             break;

         default:
             break;
     }
}


//通信数据处理
void ESHL_CommunicationDataProcessing() {

    ESHL_STATE_ENUM_T state = ESHL_GetState();

    for (uint16_t i = 0; i < (ESHL_RX_PACK_LEN - ESHL_DATA_PACK_MAX_LEN);i++) {//遍历接收缓存
        if(ESHL_ProtocolAnalysisAddr(&ESHL_RXPack[i],&recv_str)) {//提取帧头,地址,数据包长
            if ((recv_str.addr == ESHL_GetAddr()) || (recv_str.addr == ESHL_PROTOCOL_BROADCAST_ADDR)) {//地址匹配
                ESHL_ProtocolAnalysisData(ESHL_RXPack,&recv_str);//提取数据
                ESHL_CMDProcessing();//命令处理及响应
                i += recv_str.len;//跳过已经读取数据的部分
            }
        }
    }
}

//广播电调地址
void ESHL_CommunicationAddressSend() {
    uint8_t packet[10] = "";
    ESHL_PROTOCOL_PACK_ANALYSIS_T send_str;

    send_str.cmd = ESHL_PROTOCOL_CMD_CHANGE_ADDR;
    send_str.addr = ESHL_PROTOCOL_BROADCAST_ADDR;
    send_str.data = 0;
    send_str.addr_dat = ESHL_GetAddr();

    ESHL_CommunicationStop();//停止通信
    HAL_HalfDuplex_EnableTransmitter(&ESHL_UART);//启用发送
    ESHL_ProtocolPackMake(packet,&send_str,ESHL_DATA_ESC_TO_HOST);//数据打包
    HAL_UART_Transmit_DMA(&ESHL_UART,(uint8_t*)packet,10);//发送数据
    HAL_Delay(1);
    ESHL_CommunicationStop();//停止通信
}


//向主机发送一字节数据
//cmd为要发送的命令
//code为要发送的1字节数据
void ESHL_CommunicationSendCode(uint8_t cmd,uint8_t code) {

    uint8_t packet[10] = "";
    ESHL_PROTOCOL_PACK_ANALYSIS_T send_str;

    send_str.cmd = cmd;
    send_str.addr = ESHL_GetAddr();
    send_str.data = code;
    send_str.addr_dat = 0;

    ESHL_CommunicationStop();//停止通信
    HAL_HalfDuplex_EnableTransmitter(&ESHL_UART);//启用发送
    ESHL_ProtocolPackMake(packet,&send_str,ESHL_DATA_ESC_TO_HOST);//数据打包
    HAL_UART_Transmit_DMA(&ESHL_UART,(uint8_t*)packet,10);//发送数据
    HAL_Delay(10);
    ESHL_CommunicationStop();//停止通信
}
