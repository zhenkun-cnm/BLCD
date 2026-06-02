//
// Created by E_LJF on 25-7-18.
//

#ifndef COMMUNICATION_MANAGEMENT_H
#define COMMUNICATION_MANAGEMENT_H

#include "main.h"

#define ESHL_UART      huart1   //电调通信使用的串口

#define ESHL_RX_PACK_LEN 50         //电调接收缓存大小
#define ESHL_TX_PACK_LEN 20         //电调发送缓存大小
#define ESHL_DATA_PACK_MAX_LEN 16   //电调数据包最大长度(合校验1字节替换CRC16的2字节)



void ESHL_CommunicationStart();
void ESHL_CommunicationStop();
void ESHL_CommunicationDataProcessing();
void ESHL_CommunicationAddressSend();
void ESHL_CommunicationSendCode(uint8_t cmd,uint8_t code);

#endif //COMMUNICATION_MANAGEMENT_H
