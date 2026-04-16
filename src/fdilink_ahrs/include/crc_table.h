#ifndef CRC_TABLE_H
#define CRC_TABLE_H

#include <stdint.h>

// 协议校验工具：
// - CRC8：帧头校验
// - CRC16：载荷校验
// - CRC32：当前项目中预留接口，主流程并未重点使用
uint8_t CRC8_Table(uint8_t* p, uint8_t counter);
uint16_t CRC16_Table(uint8_t *p, uint8_t counter);
uint32_t CRC32_Table(uint8_t *p, uint8_t counter);

#endif // CRC_TABLE_H
