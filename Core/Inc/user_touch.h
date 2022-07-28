/*
 * user_touch.h
 *
 *  Created on: May 30, 2021
 *      Author: sangho
 */

#ifndef INC_USER_TOUCH_H_
#define INC_USER_TOUCH_H_

#include "main.h"


#define TOUCH_CMD_RDX	0xd1
#define TOUCH_CMD_RDY	0x91
#define TOUCH_CMD_RDZ1	0xb1
#define TOUCH_CMD_RDZ2	0xc1

#define Z_THRESHOLD 3000

void Touch_read();
uint16_t Touch_read_ADX();
uint16_t Touch_read_ADY();
void Touch_spi(uint8_t *dout, uint8_t *din, uint8_t size);
void Touch_delay();
#endif /* INC_USER_TOUCH_H_ */
