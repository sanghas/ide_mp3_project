/*
 * user_codex.c
 *
 *  Created on: May 4, 2021
 *      Author: sangho
 */

#include "user_codex.h"
#include "user_LCD.h"
#include "MP3_Player.h"
#include "stdio.h"

extern SPI_HandleTypeDef hspi1;
extern uint16_t q;
extern uint16_t musiccnt;
extern uint16_t musicfil[100];
extern char filname[100][20];
uint16_t vol_set=0xf0f0;
uint16_t playstate;
extern unsigned int t_cnt;
extern int t_ctrl;
char playingtime[10];
extern unsigned long filsize[100];
int mp3sec=0;
int bef_per=0;
extern uint16_t randnum;
extern char menu[15];
void Codex_temp()
{
	Codex_reg_write(CODEX_REG_AICTRL0,16000);
	Codex_reg_write(CODEX_REG_AICTRL1,0);
	Codex_reg_write(CODEX_REG_AICTRL2,4096);
	Codex_reg_write(CODEX_REG_AICTRL3,0);

	Codex_reg_write(CODEX_REG_MODE,0x5004);
}


void Codex_on()
{
	HAL_GPIO_WritePin(VS_RST_GPIO_Port, VS_RST_Pin, GPIO_PIN_SET);
	HAL_Delay(100);
}

void Codex_off()
{
	HAL_GPIO_WritePin(VS_RST_GPIO_Port, VS_RST_Pin, GPIO_PIN_RESET);
	HAL_Delay(100);
}

void Codex_reg_read(uint8_t addr, uint16_t* data)
{
	uint8_t pTxData[4];
	uint8_t pRxData[4];

	pTxData[0] = CODEX_READ;
	pTxData[1] = addr;


	while(HAL_GPIO_ReadPin(VS_DERQ_GPIO_Port, VS_DERQ_Pin) ==  GPIO_PIN_RESET);

	HAL_GPIO_WritePin(VS_XCS_GPIO_Port, VS_XCS_Pin, GPIO_PIN_RESET);  //CS low
	HAL_SPI_TransmitReceive(&hspi1, pTxData, pRxData, 4, 100);
	HAL_GPIO_WritePin(VS_XCS_GPIO_Port, VS_XCS_Pin, GPIO_PIN_SET);  //CS high


	*data = (pRxData[2]<<8) | pRxData[3];
}

void Codex_reg_write(uint8_t addr, uint16_t data)
{
	uint8_t pTxData[4];
	uint8_t pRxData[4];

	pTxData[0] = CODEX_WRITE;
	pTxData[1] = addr;
	pTxData[2] = 0xff & (data >> 8);
	pTxData[3] = 0xff & data;


	while(HAL_GPIO_ReadPin(VS_DERQ_GPIO_Port, VS_DERQ_Pin) ==  GPIO_PIN_RESET);

	HAL_GPIO_WritePin(VS_XCS_GPIO_Port, VS_XCS_Pin, GPIO_PIN_RESET);  //CS low
	HAL_SPI_TransmitReceive(&hspi1, pTxData, pRxData, 4, 100);
	HAL_GPIO_WritePin(VS_XCS_GPIO_Port, VS_XCS_Pin, GPIO_PIN_SET);  //CS high

}

void Codex_Vol_off()
{
	Codex_reg_write(CODEX_REG_VOL, 0xffff);
}

void Codex_Vol(uint16_t vol)
{
	Codex_reg_write(CODEX_REG_VOL, vol);
}

void Codex_Speaker_on()
{
	Codex_reg_write(CODEX_REG_WRAMADDR, CODEX_MEM_DDR);
	Codex_reg_write(CODEX_REG_WRAM, 0x10);

	Codex_reg_write(CODEX_REG_WRAMADDR, CODEX_MEM_ODATA);
	Codex_reg_write(CODEX_REG_WRAM, 0x10);
}

void Codex_Speaker_off()
{
	Codex_reg_write(CODEX_REG_WRAMADDR, CODEX_MEM_DDR);
	Codex_reg_write(CODEX_REG_WRAM, 0x10);

	Codex_reg_write(CODEX_REG_WRAMADDR, CODEX_MEM_ODATA);
	Codex_reg_write(CODEX_REG_WRAM, 0x00);
}

void downclick()		//버튼식 이전곡으로 변경
{


	//버튼식 커서 값 다운
	q-=1;

	//uint로 선언하여 65535(-1)로 이동할 경우 파일의 마지막 값으로 이동
	if(q==65535)
	{
		q=musiccnt-1;

		//이전 메뉴 제거
		for(int i=0;i<MENUCNT;i++)
			LCD_Draw_Str(200, 40+16*i, filname[musicfil[i]],COLOR_WHITE,1);
		LCD_Draw_Str(210, 40,"-",COLOR_WHITE,1);

		//현재 메뉴 표시
		for(int i=0;i<(musiccnt%MENUCNT);i++)
			LCD_Draw_Str(200, 40+16*i, filname[musicfil[musiccnt-(musiccnt%MENUCNT)+i]],COLOR_BLACK,1);
		LCD_Draw_Str(210, 40+16*((musiccnt-1)%MENUCNT),"-",COLOR_BLACK,1);
	}

	//버튼식 커서의 위치를 표시하는  '-'위치만 이동
	LCD_Draw_Str(210, 40+16*((q+1)%MENUCNT),"-",COLOR_WHITE,1);
	LCD_Draw_Str(210, 40+16*(q%MENUCNT),"-",COLOR_BLACK,1);

	//MENUCNT칸의 메뉴를 벗어간 경우
	if((q%MENUCNT)==(MENUCNT-1)&&(int)q!=(65535))
	{
		for(int i=MENUCNT-1;i>=0;i--)
		{
			LCD_Draw_Str(200, 40+16*i, filname[musicfil[q+1+i]],COLOR_WHITE,1);
			LCD_Draw_Str(200, 40+16*i, filname[musicfil[q+i-MENUCNT+1]],COLOR_BLACK,1);
		}
	}

	//커서의 위치를 표현
	LCD_Draw_Str(200, 24,menu,COLOR_WHITE,0);
	sprintf(menu,"%d/%d",q+1,musiccnt);
	LCD_Draw_Str(200, 24,menu,COLOR_BLACK,0);
}
void upclick()			//한칸 위로 커서 이동
{
	if(q==musiccnt-1)
	{
		//이전 메뉴 제거
		for(int i=0;i<MENUCNT;i++)
		{
			LCD_Draw_Str(200, 40+16*i, filname[musicfil[i+q-q%MENUCNT]],COLOR_WHITE,1);
		 	LCD_Draw_Str(210, 40+16*((q)%MENUCNT),"-",COLOR_WHITE,1);
		}

		//현재 메뉴 표시
		for(int i=0;i<MENUCNT;i++)
		{
			LCD_Draw_Str(210, 40,"-",COLOR_BLACK,1);
			LCD_Draw_Str(200, 40+16*i, filname[musicfil[i]],COLOR_BLACK,1);
		}
		q=0;
	}

	q+=1;
	LCD_Draw_Str(210, 40+16*((q-1)%MENUCNT),"-",COLOR_WHITE,1);
	LCD_Draw_Str(210, 40+16*(q%MENUCNT),"-",COLOR_BLACK,1);
	if((q%MENUCNT)==0&&q!=0)
	{
		for(int i=0;i<MENUCNT;i++)
		{
			LCD_Draw_Str(200, 40+16*i, filname[musicfil[i+q-MENUCNT]],COLOR_WHITE,1);
			LCD_Draw_Str(200, 40+16*i, filname[musicfil[i+q]],COLOR_BLACK,1);
		}
	}

	//MENUCNT칸의 메뉴를 벗어간 경우
	LCD_Draw_Str(200, 24,menu,COLOR_WHITE,0);
	sprintf(menu,"%d/%d",q+1,musiccnt);
	LCD_Draw_Str(200, 24,menu,COLOR_BLACK,0);
}
void Sel_Music()		//커서 위치의 노래 재생
{
	playstate=q;
	MP3_Play(filname[musicfil[q]]);
	LCD_Draw_Resume();
}
void Next_Music()		//다음곡 재생
{
	//playstate를 증가시켜 노래 재생
	playstate++;
	if(playstate==musiccnt)
		playstate=0;
	MP3_Play(filname[musicfil[playstate]]);

	//노래 재생 디스플레이
	LCD_Draw_Resume();
}
void Before_Music()		//이전곡 재생
{
	playstate--;
	if(playstate==65535)
		playstate=musiccnt-1;
	MP3_Play(filname[musicfil[playstate]]);
	LCD_Draw_Resume();
}
void Codex_Vol_up()		//볼륨 한칸 업 0x1c1c-0x7676를 max min로 설정
{
	if(vol_set==0xf0f0)
		vol_set=0x7676;
	if(vol_set==0x1c1c)	//0x0000이 가장 큰 소리지만 0x0000의 경우 전류를 많이 먹어 디스플레이에 문제 발생
		return ;
	vol_set-=0x0606;

	Codex_reg_write(CODEX_REG_VOL, vol_set);
}
void Codex_Vol_down()	//볼륨 한칸 다운 0x1c1c-0x7676를 min max로 설정	(볼륨의 범위는 0x0000부터 0xffff까지지만 적당한 영역만 지정해도 충분하다 생각)
{
	if(vol_set==0xf0f0)	//0xffff가 아닌 이유는 0xffff로 설정될 경우 오디오가 켜지고 꺼지는 소리가 크게남
		return ;
	vol_set+=0x0606;

	if(vol_set==0x7676)	//0x7676도 매우 작은 소리
		vol_set=0xf0f0;

	Codex_reg_write(CODEX_REG_VOL, vol_set);
}
void SWAP_PLAY()		//재생 정지 스왑
{
	if(isPlaying&&isFileOpen)
	{
		MP3_Pause();
		LCD_Draw_Pause();
	}
	else if(!isPlaying&&isFileOpen)
	{
		LCD_Draw_Resume();
		MP3_Resume();
	}
}
void upTime()			//노래 현재 재생시간 표시 (헤더파일을 읽지 못해 가장 기본인 128kbps로 고정 따라서 다른 bitrate를 가진 파일의 경우 재생시간이 크게 달라짐)
{
	int per_play = t_cnt*10 / mp3sec;	//퍼센트
	if(per_play>bef_per)
	{
		LCD_PlayBar(per_play);
	}
	bef_per=per_play;
	char beftime[10];	//화면 출력을 위한 변수
	if(t_cnt==10)
	{
		LCD_Draw_Str(220, 294, "00:00", COLOR_BLACK, 0);
		sprintf(playingtime,"%d%d:%d%d",(t_cnt/6000)%6,(t_cnt/600)%10,(t_cnt/100)%6,(t_cnt/10)%10);
	}
	else
	{
		sprintf(beftime,"%s",playingtime);
		sprintf(playingtime,"%d%d:%d%d",(t_cnt/6000)%6,(t_cnt/600)%10,(t_cnt/100)%6,(t_cnt/10)%10);
	}
	LCD_Draw_Str(220, 294, &beftime, COLOR_BLACK, 0);
	LCD_Draw_Str(220, 294, &playingtime, COLOR_WHITE, 0);
}
void time_Reset()		//이전 시간 초기화
{
	LCD_Draw_Str(220, 294, &playingtime, COLOR_BLACK, 0);
}
void playTime()			//현재 노래 전체 길이 표시
{
	//시간 측정 후 화면 출력
	char playtime[10];
	mp3sec=filsize[musicfil[playstate]]*8/MP3_BITRATE;
	sprintf(playtime,"%d%d:%d%d",(mp3sec/600)%6,(mp3sec/60)%10,(mp3sec/10)%6,(mp3sec/1)%10);
	LCD_Draw_Str(48, 294, playtime, COLOR_WHITE, 0);
}
void Codex_type(int type)//타입에 따른 재생
{
	//노래를 정지
	if(type==MUSIC_STOP)
	{
		MP3_Stop();
		LCD_Draw_Pause();
		LCD_Draw_Box(40,187,200,205,COLOR_BLACK);
		LCD_Draw_Box(60,298,180,300,COLOR_WHITE);
		LCD_Draw_Str(48, 294, "00:00", COLOR_WHITE, 0);
		LCD_Draw_Str(220, 294, "00:00",COLOR_WHITE, 0);
		LCD_Draw_Box(100,172,160,183,COLOR_BLACK);
	}

	//노래 이어서 재생
	if(type==MUSIC_CYCLE)
	{
		Next_Music();
	}

	//랜덤 노래 재생 (셔플구현은 못했음 + 별도의 조작이 없을 시 노래에 따라 randnum이 정해져 정확한 랜덤값은 아님)
	if(type==MUSIC_SHUFFLE)
	{
		playstate=(rand()+randnum)%musiccnt;
		Next_Music();
	}

	//한 곡 반복 재생시 playstate를 증가하고 재생하기 떄문에 -1을 해줌
	if(type==MUSIC_RECYCLE)
	{
		playstate--;
		Next_Music();
	}
}

