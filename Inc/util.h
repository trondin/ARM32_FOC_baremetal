/**
  * This file is part of the hoverboard-firmware-hack project.
  *
  * Copyright (C) 2020-2021 Emanuel FERU <aerdronix@gmail.com>
  *
  * This program is free software: you can redistribute it and/or modify
  * it under the terms of the GNU General Public License as published by
  * the Free Software Foundation, either version 3 of the License, or
  * (at your option) any later version.
  *
  * This program is distributed in the hope that it will be useful,
  * but WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  * GNU General Public License for more details.
  *
  * You should have received a copy of the GNU General Public License
  * along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef UTIL_H
#define UTIL_H

#include <stdint.h>


// Rx Structures USART
typedef struct{
  uint16_t  start;
  int16_t   steer;
  int16_t   speed;
  uint16_t  checksum;
} SerialCommand;


// Input Structure
typedef struct {
  int16_t   raw;    // raw input
  int16_t   cmd;    // command
  uint8_t   typ;    // type
  uint8_t   typDef; // type Defined
  int16_t   min;    // minimum
  int16_t   mid;    // middle
  int16_t   max;    // maximum
  int16_t   dband;  // deadband
} InputStruct;

// Initialization Functions
void BLDC_Init(void);
void Input_Lim_Init(void);
void Input_Init(void);
//void UART_DisableRxErrors(UART_HandleTypeDef *huart);

// General Functions
void poweronMelody(void);
void beepCount(uint8_t cnt, uint8_t freq, uint8_t pattern);
void beepLong(uint8_t freq);
void beepShort(uint8_t freq);
void beepShortMany(uint8_t cnt, int8_t dir);
void calcAvgSpeed(void);
//void adcCalibLim(void);
void updateCurSpdLim(void);
//void standstillHold(void);
int  checkInputType(int16_t min, int16_t mid, int16_t max);

// Input Functions
void calcInputCmd(InputStruct *in, int16_t out_min, int16_t out_max);
void handleTimeout(void);
void readCommand(void);

// Poweroff Functions
void poweroff(void);
void poweroffPressCheck(void);

// Filtering Functions
void filtLowPass32(int32_t u, uint16_t coef, int32_t *y);
void rateLimiter16(int16_t u, int16_t rate, int16_t *y);

#endif

