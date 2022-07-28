/*
 * user_joypad.c
 *
 *  Created on: May 31, 2021
 *      Author: sangho
 */


#include "user_joypad.h"


uint8_t Joypad_read()
{
	int i, t;
	uint8_t temp = 0;
	HAL_GPIO_WritePin(JOYPAD_LAT_GPIO_Port, JOYPAD_LAT_Pin, GPIO_PIN_SET);

	t = 90;
	while(t--);

	HAL_GPIO_WritePin(JOYPAD_LAT_GPIO_Port, JOYPAD_LAT_Pin, GPIO_PIN_RESET);

	for(i = 0 ; i < 8 ; i++)
	{

		temp <<= 1;
		temp |= HAL_GPIO_ReadPin(JOYPAD_DAT_GPIO_Port, JOYPAD_DAT_Pin);

		HAL_GPIO_WritePin(JOYPAD_CLK_GPIO_Port, JOYPAD_CLK_Pin, GPIO_PIN_SET);

		t = 90;
		while(t--);

		HAL_GPIO_WritePin(JOYPAD_CLK_GPIO_Port, JOYPAD_CLK_Pin, GPIO_PIN_RESET);

		t = 90;
		while(t--);
	}

	return ~temp;
}
