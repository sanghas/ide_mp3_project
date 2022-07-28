/*
 * user_eeprom.h
 *
 *  Created on: Apr 29, 2021
 *      Author: sangho
 */

#ifndef INC_USER_EEPROM_H_
#define INC_USER_EEPROM_H_

#include "main.h"


#define EEPROM_R	0xa1
#define EEPROM_W	0xa0


void EEPROM_write(I2C_HandleTypeDef* hi2c, uint8_t addr, uint8_t data);
uint8_t EEPROM_read(I2C_HandleTypeDef* hi2c, uint8_t addr);


#endif /* INC_USER_EEPROM_H_ */
