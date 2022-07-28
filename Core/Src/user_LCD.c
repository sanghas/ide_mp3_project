/*
 * user_LCD.c
 *
 *  Created on: May 13, 2021
 *      Author: sangho
 */

#include "user_LCD.h"
#include "user_font.h"
#include "math.h"

extern uint16_t musiccnt;
extern uint16_t musicfil[100];
extern char filname[100][20];
extern uint16_t playstate;
int a=-1;
extern char playingtime[10];
int vol_state=0;
extern int timeout;
extern uint16_t randnum;
char menu[15];

void LCD_BL_on()
{
	HAL_GPIO_WritePin(LCD_BL_GPIO_Port, LCD_BL_Pin, GPIO_PIN_SET);
}
void LCD_BL_off()
{
	HAL_GPIO_WritePin(LCD_BL_GPIO_Port, LCD_BL_Pin, GPIO_PIN_RESET);
}
void LCD_Check_ID()
{
	uint16_t temp;
	BANK1_index = 0xd3;
	temp = BANK1_A10_data; //dummy
	temp = BANK1_A10_data; //0x00
	temp = BANK1_A10_data; //0x93
	temp <<= 8;
	temp |= BANK1_A10_data; //0x41
	temp = 0;
}
void LCD_init()
{
	BANK1_index = 0xCF;
	BANK1_A10_data = 0x00;
	BANK1_A10_data = 0xC1;
	BANK1_A10_data = 0X30;
	BANK1_index = 0xED;
	BANK1_A10_data = 0x64;
	BANK1_A10_data = 0x03;
	BANK1_A10_data = 0X12;
	BANK1_A10_data = 0X81;
	BANK1_index = 0xE8;
	BANK1_A10_data = 0x85;
	BANK1_A10_data = 0x10;
	BANK1_A10_data = 0x7A;
	BANK1_index = 0xCB;
	BANK1_A10_data = 0x39;
	BANK1_A10_data = 0x2C;
	BANK1_A10_data = 0x00;
	BANK1_A10_data = 0x34;
	BANK1_A10_data = 0x02;
	BANK1_index = 0xF7;
	BANK1_A10_data = 0x20;
	BANK1_index = 0xEA;
	BANK1_A10_data = 0x00;
	BANK1_A10_data = 0x00;
	BANK1_index = 0xC0;    //Power control
	BANK1_A10_data = 0x1B;   //VRH[5:0]
	BANK1_index = 0xC1;    //Power control
	BANK1_A10_data = 0x01;   //SAP[2:0];BT[3:0]
	BANK1_index = 0xC5;    //VCM control
	BANK1_A10_data = 0x30; 	 //3F
	BANK1_A10_data = 0x30; 	 //3C
	BANK1_index = 0xC7;    //VCM control2
	BANK1_A10_data = 0XB7;
	BANK1_index = 0x36;    // Memory Access Control
	BANK1_A10_data = 0x48;
	BANK1_index = 0x3A;
	BANK1_A10_data = 0x55;
	BANK1_index = 0xB1;
	BANK1_A10_data = 0x00;
	BANK1_A10_data = 0x1A;
	BANK1_index = 0xB6;    // Display Function Control
	BANK1_A10_data = 0x0A;
	BANK1_A10_data = 0xA2;
	BANK1_index = 0xF2;    // 3Gamma Function Disable
	BANK1_A10_data = 0x00;
	BANK1_index = 0x26;    //Gamma curve selected
	BANK1_A10_data = 0x01;
	BANK1_index = 0xE0;    //Set Gamma
	BANK1_A10_data = 0x0F;
	BANK1_A10_data = 0x2A;
	BANK1_A10_data = 0x28;
	BANK1_A10_data = 0x08;
	BANK1_A10_data = 0x0E;
	BANK1_A10_data = 0x08;
	BANK1_A10_data = 0x54;
	BANK1_A10_data = 0XA9;
	BANK1_A10_data = 0x43;
	BANK1_A10_data = 0x0A;
	BANK1_A10_data = 0x0F;
	BANK1_A10_data = 0x00;
	BANK1_A10_data = 0x00;
	BANK1_A10_data = 0x00;
	BANK1_A10_data = 0x00;
	BANK1_index = 0XE1;    //Set Gamma
	BANK1_A10_data = 0x00;
	BANK1_A10_data = 0x15;
	BANK1_A10_data = 0x17;
	BANK1_A10_data = 0x07;
	BANK1_A10_data = 0x11;
	BANK1_A10_data = 0x06;
	BANK1_A10_data = 0x2B;
	BANK1_A10_data = 0x56;
	BANK1_A10_data = 0x3C;
	BANK1_A10_data = 0x05;
	BANK1_A10_data = 0x10;
	BANK1_A10_data = 0x0F;
	BANK1_A10_data = 0x3F;
	BANK1_A10_data = 0x3F;
	BANK1_A10_data = 0x0F;
	BANK1_index = 0x2B;   //Page Address Set
	BANK1_A10_data = 0x00;
	BANK1_A10_data = 0x00;
	BANK1_A10_data = 0x01;
	BANK1_A10_data = 0x3f;
	BANK1_index = 0x2A;   //Column Address Set
	BANK1_A10_data = 0x00;
	BANK1_A10_data = 0x00;
	BANK1_A10_data = 0x00;
	BANK1_A10_data = 0xef;
	BANK1_index = 0x11; //Exit Sleep
	HAL_Delay(120);
	BANK1_index = 0x29; //display on
}
void LCD_SetPos(uint16_t xPos, uint16_t yPos)
{
	BANK1_index = LCD_xpos_cmd;
	BANK1_A10_data = xPos>>8;
	BANK1_A10_data = xPos&0xff;
	BANK1_index = LCD_ypos_cmd;
	BANK1_A10_data = yPos>>8;
	BANK1_A10_data = yPos&0xff;
}
void LCD_Clear(uint16_t Color)
{
	int i;
	LCD_SetPos(0,0);
	//LCD_buf[10][11] = Color;
	BANK1_index = LCD_wram_cmd;
	for(i = 0; i < LCD_width * LCD_height ; i++)
	{
		BANK1_A10_data = Color;
	}
}
void LCD_Draw_Point(uint16_t x, uint16_t y, uint16_t Color)
{
	LCD_SetPos(x,y);
	BANK1_index = LCD_wram_cmd;
	BANK1_A10_data = Color;
}
void LCD_Draw_Line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,uint16_t Color)
{
		LCD_SetPos(x1,y1);
		BANK1_index = LCD_wram_cmd;

		float dx, dy, slope, xz;			//xz: x절편
		uint16_t rx1,rx2,ry1,ry2;

		rx1=x1<=x2?x1:x2;					//x값 정렬
		rx2=x1<=x2?x2:x1;
		ry1=y1<=y2?y1:y2;					//y값 정렬
		ry2=y1<=y2?y2:y1;

		dx=(float)x2-(float)x1;
		dy= (float)y2-(float)y1;

		if(dx==0)
		{
			for(;ry1<ry2;ry1++)
				LCD_Draw_Point(rx1, ry1, Color);
			return 0;
		}

		if(dy==0)
		{
			for(;rx1<rx2;rx1++)
				LCD_Draw_Point(rx1, ry1, Color);
			return 0;
		}

		slope=(float)dy/(float)dx;			//기울기

		if(slope>=0)
			xz=(float)rx1*slope;
		else
			xz=(float)rx2*slope;

		for(;rx1<=rx2;rx1++)
		{
			y1=slope*(float)rx1-xz;
			if(fabs(slope)>=1)
				for(int j=0;j<=fabs(slope)+1;j++)		// 점 잇기
					LCD_Draw_Point(rx1, y1+j, Color);
			else
				for(int j=0;j<=fabs(slope)+1;j++)		// 점 잇기
					LCD_Draw_Point(rx1+j, y1, Color);
		}
}
void LCD_Draw_Filled_Circle(uint16_t x, uint16_t y, uint16_t r, uint16_t Color)
{
	int i=0,j=0,px=x-r,py=y-r,size=2*r,temp=r*r;
	uint16_t nowcolor;
	LCD_SetPos(px,py);
	BANK1_index = LCD_wram_cmd;
	for(j=0;j<size;j++)
	{
		for(i=0;i<size;i++)
		{
			int det=(x-px-i-1)*(x-px-i-1)+(y-py-j-1)*(y-py-j-1);
			if(det<temp)	nowcolor=Color;
			else	nowcolor=COLOR_BLACK;
			BANK1_A10_data=nowcolor;
		}
		LCD_SetPos(px, py+j+1);
		BANK1_index = LCD_wram_cmd;
	}
}
void LCD_Draw_Char(uint16_t x, uint16_t y, uint8_t Char, uint16_t Color, uint16_t font_size)
{
	uint8_t temp;
	int i,j,dx = 0, dy = 0;

	for(i = 0 ; i < asc_length[font_size] ; i++)
	{
		if(font_size == 0) 			temp = asc2_1206[Char - ' '][i];
		else if(font_size == 1) 	temp = asc2_1608[Char - ' '][i];
		else if(font_size == 2) 	temp = asc2_2412[Char - ' '][i];
		else if(font_size == 3) 	temp = asc2_3216[Char - ' '][i];

		for(j = 0 ; j < 8 ; j++, temp <<= 1)
		{
			if((temp&0x80) == 0x80) LCD_Draw_Point(x + dx, y + dy, Color);

			dy++;
			if(dy == asc_height[font_size])
			{
				dy = 0;
				dx--;
			}
		}
	}
}
void LCD_Draw_Char1(uint16_t x, uint16_t y,uint16_t Color,uint16_t width ,uint16_t height)
{
	int temp;
	int i,dx = 0, dy = 0;

	for(i = 0 ; i < 620 ; i++)
	{
		temp=image[i];
		if(temp==1)
		{
			for(int j = 0 ; j < 2 ; j++)
			{
				for(int k=0;k<2;k++)
					LCD_Draw_Point(x-dx+j,y-dy+k,Color);
			}
		}
		dx+=2;

		if(dx==width*2)
		{
			dx=0;
			dy-=2;
		}
	}
}
void LCD_Draw_Str(uint16_t x, uint16_t y, uint8_t* str, uint16_t Color, uint16_t font_size)
{
	int dx = 0;

	while(*str != '\0')
	{
		LCD_Draw_Char(x + dx, y, *str,Color,font_size);
		str++;
		dx -= asc_width[font_size];
	}
}

void LCD_Draw_Box(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,uint16_t Color)
{
	int i;
	LCD_SetPos(x1,y1);
	BANK1_index = LCD_wram_cmd;
	for(i = x1; i < x2; i++)
	{
		for(int j=y1;j<y2;j++)
			LCD_Draw_Point(i,j,Color);
	}

}
void LCD_Menu_Set()			//초기 메뉴 표시
{
	//버튼식 메뉴 번호 설정
	sprintf(menu,"%d/%d",1,musiccnt);
	LCD_Draw_Str(200, 24,menu,COLOR_BLACK,0);

	//버튼식 메뉴 지정표시
	LCD_Draw_Str(210, 40,"-",COLOR_BLACK,1);

	//메뉴를 MENUCNT만큼 표시
	for(int i=0;i<MENUCNT;i++)
		LCD_Draw_Str(200, 40+16*i, filname[musicfil[i]],COLOR_BLACK,1);
}
void LCD_Draw_Base()		//초기 화면 세팅
{
	LCD_Clear(COLOR_WHITE);

	//테두리 세팅
	LCD_Draw_Box(3,3,237,8,COLOR_BLACK);
	LCD_Draw_Box(3,3,8,317,COLOR_BLACK);
	LCD_Draw_Box(3,312,237,317,COLOR_BLACK);
	LCD_Draw_Box(232,3,237,317,COLOR_BLACK);
	LCD_Draw_Box(3,172,237,312,COLOR_BLACK);

	//재생 버튼
	LCD_Draw_Filled_Circle(120, 241, 25, COLOR_WHITE);
	for(int i=0;i<18;i++)
	{
		LCD_Draw_Line(125, 258-i , 125-(1*i), 258-(1*i) ,COLOR_BLACK);
		LCD_Draw_Line(125, 223+i , 125-i, 223+i ,COLOR_BLACK);
	}

	//재생상태
	LCD_Draw_Str(230, 174,"CYCLE",COLOR_RED,0);

	//재생 바 및 재생 시간 표시
	LCD_Draw_Box(60,298,180,300,COLOR_WHITE);
	LCD_Draw_Str(48, 294, "00:00", COLOR_WHITE, 0);
	LCD_Draw_Str(220, 294, "00:00",COLOR_WHITE, 0);

	//볼륨 바 세팅
	LCD_Draw_Box(48, 26,70,150, COLOR_BLACK);
	LCD_Draw_Box(50, 28,68,148, COLOR_WHITE);
	LCD_Draw_Str(46, 148, "+", COLOR_BLACK, 0);
	LCD_Draw_Str(74, 148, "-", COLOR_BLACK, 0);

	//다음 곡 (>)
	for(int i=0;i<23;i++)
	{
		LCD_Draw_Line(60, 263-i , 60-(1*i), 263-i ,COLOR_WHITE);
		LCD_Draw_Line(60, 218+i , 60-i, 218+i ,COLOR_WHITE);
	}
	for(int i=0;i<15;i++)
	{
			LCD_Draw_Line(60, 255-i , 60-(1*i), 255-(1*i) ,COLOR_BLACK);
			LCD_Draw_Line(60, 226+i , 60-i, 226+i ,COLOR_BLACK);
	}

	//이전 곡 (<)
	for(int i=0;i<23;i++)
	{
		LCD_Draw_Line(180, 263-i , 180+i, 263-i ,COLOR_WHITE);
		LCD_Draw_Line(180, 218+i , 180+i, 218+i ,COLOR_WHITE);
	}
	for(int i=0;i<15;i++)
	{
			LCD_Draw_Line(180, 255-i , 180+i, 255-i ,COLOR_BLACK);
			LCD_Draw_Line(180, 226+i , 180+i, 226+i ,COLOR_BLACK);
	}
}
void LCD_Draw_Resume()		//재생표시
{
	//재생버튼 제거
	for(int i=0;i<18;i++)
	{
		LCD_Draw_Line(125, 258-i , 125-(1*i), 258-(1*i) ,COLOR_WHITE);
		LCD_Draw_Line(125, 223+i , 125-i, 223+i ,COLOR_WHITE);
	}
	//정지버튼 생성
	LCD_Draw_Box(123,230,128,250,COLOR_BLACK);
	LCD_Draw_Box(112,230,117,250,COLOR_BLACK);
}
void LCD_Draw_Pause()		//정지표시
{
	//정지버튼 제거
	LCD_Draw_Box(123,230,128,250,COLOR_WHITE);
	LCD_Draw_Box(112,230,117,250,COLOR_WHITE);

	//재생버튼 추가
	for(int i=0;i<18;i++)
	{
		LCD_Draw_Line(125, 258-i , 125-(1*i), 258-(1*i) ,COLOR_BLACK);
		LCD_Draw_Line(125, 223+i , 125-i, 223+i ,COLOR_BLACK);
	}

}
void LCD_Play_Flow()		//현재 재생곡 표시
{
	//이전 파일의 정보 삭제
	LCD_Draw_Box(20, 287,50,306, COLOR_BLACK);
	LCD_Draw_Box(40,187,200,205,COLOR_BLACK);

	//재생 될 노래의 제목 표시
	LCD_Draw_Str(157, 187,filname[musicfil[playstate]],COLOR_WHITE,1);

	//재생 될 노래의 길이 표시
	playTime();

	//재생 될 노래의 번호 표시
	char playpoint[8];
	LCD_Draw_Box(100,172,160,183,COLOR_BLACK);
	sprintf(playpoint,"%d/%d",playstate+1,musiccnt);
	LCD_Draw_Str(159, 171,playpoint,COLOR_WHITE,0);

}
void LCD_Vol_Display(bool change,bool voloff)	//볼륨 바 변경
{
	//볼륨 바 간격 조정
	int vol_bluck[16]={148,140,132,124,116,108,100,92,84,76,68,60,52,44,36,28};


	//볼륨 바가 존재하는 경우
	if(!voloff)
	{
		if(change==1&&vol_state!=15)	//볼륨바를 한칸 증가
		{
			Codex_Vol_up();
			vol_state++;
			LCD_Draw_Box(50, vol_bluck[vol_state],68,vol_bluck[vol_state-1], COLOR_BLACK);
		}
		if(change==0&&vol_state!=0)		//볼륨바를 한칸 감소
		{
			Codex_Vol_down();
			vol_state--;
			LCD_Draw_Box(50, vol_bluck[vol_state+1],68,vol_bluck[vol_state], COLOR_WHITE);
		}
	}

	//음표가 존재하는 경우
	else
	{
		//음표 제거
		LCD_Draw_Char1(80, 60,COLOR_WHITE,20 ,31);

		//볼륨바 생성
		LCD_Draw_Box(48, 26,70,150, COLOR_BLACK);
		LCD_Draw_Str(46, 148, "+", COLOR_BLACK, 0);
		LCD_Draw_Str(74, 148, "-", COLOR_BLACK, 0);

		//위쪽부터 볼륨까지 흰색으로 비움
		if(change==1&&vol_state!=15)	//볼륨업
		{
			Codex_Vol_up();
			vol_state++;
			LCD_Draw_Box(50, 28,68,vol_bluck[vol_state], COLOR_WHITE);
		}
		else if(change==0&&vol_state!=0)//볼륨다운
		{
			Codex_Vol_down();
			vol_state--;
			LCD_Draw_Box(50, 28,68,vol_bluck[vol_state], COLOR_WHITE);
		}

		//볼륨바가 존재한다고 알림
		voloff=0;
	}


}
void LCD_PlayBar(int per)	//노래 재생량 표시 (크기가 작아 처음부터 재생 위치 까지 그림)
{
	if(per!=0)
		LCD_Draw_Box(180-(per*12/10),298,180,300,COLOR_RED);
}
void LCD_Out_Time()	//마지막으로 작동시킨 시간 초기화
{
	timeout=0;
	LCD_BL_on();
}
void LCD_Type(int type)		//재생타입 표시
{
	//사이클이 일정하여 타입이 바뀌는 경우 이전 글자 제거 후 표시
	if(type==MUSIC_STOP)
	{

		LCD_Draw_Str(230, 174,"REPLAY",COLOR_BLACK,0);
		LCD_Draw_Str(230, 174,"STOP",COLOR_RED,0);
	}
	if(type==MUSIC_CYCLE)
	{
		LCD_Draw_Str(230, 174,"STOP",COLOR_BLACK,0);
		LCD_Draw_Str(230, 174,"CYCLE",COLOR_RED,0);
	}
	if(type==MUSIC_SHUFFLE)
	{
		LCD_Draw_Str(230, 174,"CYCLE",COLOR_BLACK,0);
		LCD_Draw_Str(230, 174,"RAMDOM",COLOR_RED,0);
	}
	if(type==MUSIC_RECYCLE)
	{
		LCD_Draw_Str(230, 174,"RAMDOM",COLOR_BLACK,0);
		LCD_Draw_Str(230, 174,"REPLAY",COLOR_RED,0);
	}
}
