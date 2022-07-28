/*
 * user_touch.c
 *
 *  Created on: May 30, 2021
 *      Author: sangho
 */

#include "user_touch.h"
#include "MP3_Player.h"

uint16_t zraw, xraw, yraw;
extern unsigned int t_cnt;
uint8_t two_touch=0;
extern uint8_t vol_check;
extern uint8_t vol_flag;

void Touch_read()
{
	uint8_t dout[3] = {0};
	uint8_t din[3];
	uint16_t z1, z2;
	int32_t z;
	uint16_t data[6] = {0};

	HAL_GPIO_WritePin(T_CS_GPIO_Port, T_CS_Pin, GPIO_PIN_RESET);

	dout[0] = TOUCH_CMD_RDZ1;
	Touch_spi(dout, din, 3);

	dout[0] = TOUCH_CMD_RDZ2;
	Touch_spi(dout, din, 3);
	z1 = ((din[1]<<8) | din[2])>>3;
	z = z1 + 4095;

	dout[0] = TOUCH_CMD_RDY;
	Touch_spi(dout, din, 3);
	z2 = ((din[1]<<8) | din[2])>>3;
	z -= z2;

	if(z >= Z_THRESHOLD)
	{
		dout[0] = TOUCH_CMD_RDY;
		Touch_spi(dout, din, 3); //dummy

		dout[0] = TOUCH_CMD_RDX;
		Touch_spi(dout, din, 3);
		data[0] = ((din[1]<<8) | din[2])>>3;

		dout[0] = TOUCH_CMD_RDY;
		Touch_spi(dout, din, 3);
		data[1] = ((din[1]<<8) | din[2])>>3;

		dout[0] = TOUCH_CMD_RDX;
		Touch_spi(dout, din, 3);
		data[2] = ((din[1]<<8) | din[2])>>3;

		dout[0] = TOUCH_CMD_RDY;
		Touch_spi(dout, din, 3);
		data[3] = ((din[1]<<8) | din[2])>>3;
	}

	dout[0] = 0xd0;
	Touch_spi(dout, din, 3);
	data[4] = ((din[1]<<8) | din[2])>>3;

	dout[0] = 0x00;
	Touch_spi(dout, din, 3);
	data[5] = ((din[1]<<8) | din[2])>>3;

	HAL_GPIO_WritePin(T_CS_GPIO_Port, T_CS_Pin, GPIO_PIN_SET);

	if( z < 0 ) z = 0;
	if( z < Z_THRESHOLD)
	{
		zraw = 0;
		return;
	}

	if( z > 6000)
	{
		return;
	}
	zraw = z;

	xraw = (data[0] + data[2])/2;
	yraw = (data[1] + data[3])/2;

	if(xraw>1650&&xraw<2500&&yraw>2700&&yraw<3250)//스탑,재생
	{
		SWAP_PLAY();
		HAL_Delay(50);
		if(!isPlaying&&!isFileOpen)
			Sel_Music();
	}
	if(xraw>2900&&xraw<3400&&yraw>2600&&yraw<3300)//다음곡
	{
		HAL_Delay(50);
		Next_Music();
	}
	if(xraw>500&&xraw<1250&&yraw>2600&&yraw<3300)//이전곡
	{
		HAL_Delay(50);
		Before_Music();
	}
	if(xraw>3300&&xraw<3700&&yraw>1000&&yraw<1650)	//볼륨업
	{
		if(vol_flag>=2)
			LCD_Vol_Display(true,true);
		else
			LCD_Vol_Display(true,false);
		vol_check=0;
		vol_flag=0;

	}
	if(xraw>2200&&xraw<2700&&yraw>1000&&yraw<1650)	//볼륨다운
	{
		if(vol_flag==2)
			LCD_Vol_Display(false,true);
		else
			LCD_Vol_Display(false,false);
		vol_check=0;
		vol_flag=0;
	}
	if(xraw>800&&xraw<1800&&yraw>670&&yraw<1500)	// 투 터치를 위한 보조수단
	{
		two_touch++;
	}


}


void Touch_spi(uint8_t *dout, uint8_t *din, uint8_t size)
{
	int i, j;


	for(j = 0 ; j < size ; j++)
	{
		for(i = 7 ; i >= 0 ; i--)
		{
			HAL_GPIO_WritePin(T_MOSI_GPIO_Port, T_MOSI_Pin, ((dout[j] >> i) & 0x01));

			//HAL_Delay(1);
			Touch_delay();
			HAL_GPIO_WritePin(T_SCL_GPIO_Port, T_SCL_Pin, GPIO_PIN_SET);
			din[j] <<= 1;
			din[j] |= HAL_GPIO_ReadPin(T_MISO_GPIO_Port, T_MISO_Pin);

			//HAL_Delay(1);
			Touch_delay();
			HAL_GPIO_WritePin(T_SCL_GPIO_Port, T_SCL_Pin, GPIO_PIN_RESET);

		}
	}

	//HAL_Delay(1);
	Touch_delay();




}




void Touch_delay()
{
	int i;
	for(i = 0 ; i < 100 ; i++);
}
