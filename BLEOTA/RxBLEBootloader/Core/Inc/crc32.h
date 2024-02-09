/*
 * crc32.h
 *
 *  Created on: Oct 18, 2023
 *      Author: Hamza
 */

#ifndef INC_CRC32_H_
#define INC_CRC32_H_

#include <stdint.h>


extern uint32_t m_crc;
void crc32();

void start(int type);

void update(uint8_t *data,uint32_t len);
void int_flash_vCalculateCRC(uint8_t *data, uint32_t len, uint32_t *ulCRC);
uint32_t finish(void);

#endif /* INC_CRC32_H_ */



