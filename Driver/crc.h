#ifndef __CRC_H__
#define __CRC_H__

/* 包含头文件 ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdio.h>

/* 类型定义 ------------------------------------------------------------------*/

/* 宏定义 --------------------------------------------------------------------*/

/* 扩展变量 ------------------------------------------------------------------*/

/* 函数声明 ------------------------------------------------------------------*/
uint8_t verify_crc8_check_sum(uint8_t* pchMessage, uint16_t dwLength);
uint8_t verify_crc16_check_sum(uint8_t* pchMessage, uint32_t dwLength);

void append_crc8_check_sum(uint8_t* pchMessage, uint16_t dwLength);
void append_crc16_check_sum(uint8_t* pchMessage, uint32_t dwLength);

/* 合校验 (累加和, 取低8位) — 替代CRC16用于ESHL协议 */
uint8_t get_checksum(uint8_t *pchMessage, uint16_t dwLength);
uint8_t verify_checksum(uint8_t *pchMessage, uint16_t dwLength);
void    append_checksum(uint8_t *pchMessage, uint16_t dwLength);

#endif  // __CRC_H__

