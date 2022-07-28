/*
 * user_eeprom.c
 *
 *  Created on: Apr 29, 2021
 *      Author: sangho
 */


#include "user_eeprom.h"

void EEPROM_write(I2C_HandleTypeDef* hi2c, uint8_t addr, uint8_t data)
{
	uint8_t pData[2];
	pData[0] = addr;
	pData[1] = data;

	HAL_I2C_Master_Transmit(hi2c, EEPROM_W , pData, 2, 100);
	HAL_Delay(1);
}

uint8_t EEPROM_read(I2C_HandleTypeDef* hi2c, uint8_t addr)
{
	uint8_t pData[2];
	pData[0] = addr;

	HAL_I2C_Master_Transmit(hi2c, EEPROM_W , pData, 1, 100);
	HAL_I2C_Master_Receive(hi2c, EEPROM_R, pData, 1, 100);

	HAL_Delay(1);
	return pData[0];
}

//void EEPROM_SQ_write(I2C_HandleTypeDef* hi2c, uint8_t addr, uint8_t* data, uint8_t size)

//void EEPROM_SQ_read(I2C_HandleTypeDef* hi2c, uint8_t addr, uint8_t* data, uint8_t size)

