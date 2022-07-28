/*
 * user_codex.h
 *
 *  Created on: May 4, 2021
 *      Author: sangho
 */

#ifndef INC_USER_CODEX_H_
#define INC_USER_CODEX_H_

#include "main.h"


#define CODEX_READ	0x03
#define CODEX_WRITE	0x02

#define CODEX_REG_MODE		0x00
#define CODEX_REG_VOL		0x0b
#define CODEX_REG_WRAM		0x06
#define CODEX_REG_WRAMADDR	0x07
#define CODEX_REG_AICTRL0	0x0c
#define CODEX_REG_AICTRL1	0x0d
#define CODEX_REG_AICTRL2	0x0e
#define CODEX_REG_AICTRL3	0x0f

#define CODEX_MEM_DDR		0xc017
#define CODEX_MEM_ODATA		0xc019


void Codex_Sin_Test();

void Codex_on();
void Codex_off();
void Codex_reg_read(uint8_t addr, uint16_t* data);
void Codex_reg_write(uint8_t addr, uint16_t data);

void Codex_Vol_off();
void Codex_Vol(uint16_t vol);
void Codex_Speaker_on();
void Codex_Speaker_off();
void downclick();
void upclick();
void Sel_Music();
void Codex_Vol_up();
void Codex_Vol_down();
uint16_t Codex_read_Dir(char* fil);
void SWAP_PLAY();
void Next_Music();
void upTime();
void time_Reset();
void playTime();
void Codex_type(int type);

#endif /* INC_USER_CODEX_H_ */
