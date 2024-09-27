/*
* This file is part of the hoverboard-firmware-hack project.
*
* Copyright (C) 2017-2018 Rene Hopf <renehopf@mac.com>
* Copyright (C) 2017-2018 Nico Stute <crinq@crinq.de>
* Copyright (C) 2017-2018 Niklas Fauth <niklas.fauth@kit.fail>
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

// Define to prevent recursive inclusion
#ifndef DEFINES_H
#define DEFINES_H

#include "stm32f1xx_hal.h"
#include "config.h"


#define HALL_U_PIN 4
#define HALL_V_PIN 5
#define HALL_W_PIN 0

#define HALL_U_PORT GPIOB
#define HALL_V_PORT GPIOB
#define HALL_W_PORT GPIOB




#define TIM_UH_PIN 8
#define TIM_UH_PORT GPIOA
#define TIM_UL_PIN 13
#define TIM_UL_PORT GPIOB
#define TIM_VH_PIN 9
#define TIM_VH_PORT GPIOA
#define TIM_VL_PIN 14
#define TIM_VL_PORT GPIOB
#define TIM_WH_PIN 10
#define TIM_WH_PORT GPIOA
#define TIM_WL_PIN 15
#define TIM_WL_PORT GPIOB


#define DC_CUR_PIN 3
#define U_CUR_PIN 4

#define DC_CUR_PORT GPIOA
#define U_CUR_PORT GPIOA
#define V_CUR_PORT GPIOA

#define LED_PIN 1
#define LED_PORT GPIOD

#define BUZZER_PIN 15
#define BUZZER_PORT GPIOA


#define OFF_PIN 15
#define OFF_PORT GPIOC


#define BUTTON_PIN 14
#define BUTTON_PORT GPIOC

//#define CHARGER_PIN 12
//#define CHARGER_PORT GPIOA



#define DELAY_TIM_FREQUENCY_US 1000000

#define MILLI_R (R * 1000)
#define MILLI_PSI (PSI * 1000)
#define MILLI_V (V * 1000)

#define NO 0
#define YES 1
#define ABS(a) (((a) < 0) ? -(a) : (a))
#define LIMIT(x, lowhigh) (((x) > (lowhigh)) ? (lowhigh) : (((x) < (-lowhigh)) ? (-lowhigh) : (x)))
#define SAT(x, lowhigh) (((x) > (lowhigh)) ? (1.0f) : (((x) < (-lowhigh)) ? (-1.0f) : (0.0f)))
#define SAT2(x, low, high) (((x) > (high)) ? (1.0f) : (((x) < (low)) ? (-1.0f) : (0.0f)))
#define STEP(from, to, step) (((from) < (to)) ? (MIN((from) + (step), (to))) : (MAX((from) - (step), (to))))
#define DEG(a) ((a)*M_PI / 180.0f)
#define RAD(a) ((a)*180.0f / M_PI)
#define SIGN(a) (((a) < 0) ? (-1) : (((a) > 0) ? (1) : (0)))
#define CLAMP(x, low, high) (((x) > (high)) ? (high) : (((x) < (low)) ? (low) : (x)))
#define IN_RANGE(x, low, high) (((x) >= (low)) && ((x) <= (high)))
#define SCALE(value, high, max) MIN(MAX(((max) - (value)) / ((max) - (high)), 0.0f), 1.0f)
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#define MIN3(a, b, c) MIN(a, MIN(b, c))
#define MAX3(a, b, c) MAX(a, MAX(b, c))
#define ARRAY_LEN(x) (uint32_t)(sizeof(x) / sizeof(*(x)))
#define MAP(x, in_min, in_max, out_min, out_max) (((((x) - (in_min)) * ((out_max) - (out_min))) / ((in_max) - (in_min))) + (out_min))

/*
typedef struct {
  uint16_t dcr; 
  uint16_t rrB; 
  uint16_t rrC;
  uint16_t batt1;  
  uint16_t temp;
} adc_buf_t;
*/

typedef struct {
  uint16_t phA; 
  uint16_t phB; 
  uint16_t phC;
  uint16_t batt1;  
  uint16_t temp;
} adc_buf_t;

#endif // DEFINES_H

