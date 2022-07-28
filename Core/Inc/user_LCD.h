/*
 * user_LCD.h
 *
 *  Created on: May 13, 2021
 *      Author: sangho
 */

#ifndef INC_USER_LCD_H_
#define INC_USER_LCD_H_

#include "main.h"
#include "stdbool.h"


#define BANK1_index		*((uint16_t *)0x6c000000)
#define BANK1_A10_data	*((uint16_t *)0x6c000800)

#define LCD_wram_cmd	0x2c
#define LCD_xpos_cmd	0x2a
#define LCD_ypos_cmd	0x2b

#define LCD_width	240
#define LCD_height	320


#define COLOR_BLACK	0x0005
#define COLOR_WHITE	0xffff
#define COLOR_BLUE	0x001f
#define COLOR_GREEN	0x07e0
#define COLOR_RED	0xf800

void LCD_BL_on();
void LCD_BL_off();


void LCD_Check_ID();

void LCD_init();
void LCD_SetPos(uint16_t xPos, uint16_t yPos);
void LCD_Clear(uint16_t Color);

void LCD_Draw_Point(uint16_t x, uint16_t y, uint16_t Color);
void LCD_Draw_Line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,uint16_t Color);
void LCD_Draw_Filled_Circle(uint16_t x, uint16_t y, uint16_t r, uint16_t Color);
void LCD_Draw_Char(uint16_t x, uint16_t y, uint8_t Char, uint16_t Color, uint16_t font_size);
void LCD_Draw_Char1(uint16_t x, uint16_t y,uint16_t Color,uint16_t width ,uint16_t height);
void LCD_Draw_Str(uint16_t x, uint16_t y, uint8_t* str, uint16_t Color, uint16_t font_size);
void LCD_Menu_Set();
void LCD_Draw_Base();
void LCD_Draw_Resume();
void LCD_Draw_Pause();
void LCD_Draw_Box(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,uint16_t Color);
void LCD_Play_Flow();
void LCD_Vol_Display(bool change,bool voloff);
void LCD_Out_Time();
void LCD_PlayBar(int per);
void LCD_Type(int type);
#endif /* INC_USER_LCD_H_ */
