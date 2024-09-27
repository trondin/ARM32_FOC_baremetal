
/*
* This file is part of the hoverboard-firmware-hack project.
*
* Copyright (C) 2017-2018 Rene Hopf <renehopf@mac.com>
* Copyright (C) 2017-2018 Nico Stute <crinq@crinq.de>
* Copyright (C) 2017-2018 Niklas Fauth <niklas.fauth@kit.fail>
* Copyright (C) 2019-2020 Emanuel FERU <aerdronix@gmail.com>
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

#include <stdio.h>
#include <stdlib.h> // for abs()
#include "stm32f1xx_hal.h"
#include "defines.h"
#include "setup.h"
#include "config.h"
#include "util.h"
#include "BLDC_controller.h"      /* BLDC's header file */
#include "rtwtypes.h"



void SystemClock_Config(void);

//------------------------------------------------------------------------
// Global variables set externally
//------------------------------------------------------------------------
extern volatile adc_buf_t adc_buffer;

// Matlab defines - from auto-code generation
//---------------
extern ExtY rtY;                  /* External outputs */

//---------------

extern InputStruct input;            // input structure

extern int16_t speedAvg;                // Average measured speed
extern int16_t speedAvgAbs;             // Average measured speed in absolute
extern uint8_t timeoutFlgSerial;        // Timeout Flag for Rx Serial command: 0 = OK, 1 = Problem detected (line disconnected or wrong Rx data)

extern volatile int pwmr;               // global variable for pwm right. -1000 to 1000
extern uint8_t enable;                  // global variable for motor enable
extern int16_t batVoltage;              // global variable for battery voltage



//------------------------------------------------------------------------
// Global variables set here in main.c
//------------------------------------------------------------------------
extern volatile uint32_t buzzerTimer;
volatile uint32_t main_loop_counter;
int16_t batVoltageCalib;         // global variable for calibrated battery voltage
int16_t board_temp_deg_c;        // global variable for calibrated temperature in degrees Celsius


//------------------------------------------------------------------------
// Local variables
//------------------------------------------------------------------------
typedef struct
{
  uint16_t  start;

  int16_t   cmd;
  int16_t   cmd1;
  int16_t   speed_meas;
  int16_t   speedR_meas;

  int16_t   batVoltage;
  int16_t   boardTemp;
  uint16_t cmdLed;
  uint16_t  checksum;
} SerialFeedback;

static SerialFeedback Feedback;

static int16_t  speedRateFixdt;       // local fixed-point variable for speed rate limiter
static int32_t  speedFixdt;           // local fixed-point variable for speed low-pass filter

static uint32_t    inactivity_timeout_counter;


static uint16_t rate = RATE; // Adjustable rate to support multiple drive modes on startup

// https://www.sebulli.com/ntc/index.php?lang=en
// The NTC table has 33 interpolation points.
// Unit:0.1 °C
const int16_t NTC_table[33] = {
  1835, 1493, 1151, 969, 847, 754, 678, 615, 
  560, 511, 466, 425, 386, 350, 316, 282, 250, 
  218, 187, 156, 125, 93, 61, 28, -6, -43, 
  -83, -126, -176, -235, -311, -428, -545
};

/**
* \brief    Converts the ADC result into a temperature value.
*
*           P1 and p2 are the interpolating point just before and after the
*           ADC value. The function interpolates between these two points
*           The resulting code is very small and fast.
*           Only one integer multiplication is used.
*           The compiler can replace the division by a shift operation.
*
*           In the temperature range from -20°C to 100°C the error
*           caused by the usage of a table is 0.652°C
*
* \param    adc_value  The converted ADC result
* \return              The temperature in 0.1 °C
*
*/
int16_t NTC_ADC2Temperature(uint16_t adc_value)
{
  int16_t p1,p2;
  // Estimate the interpolating point before and after the ADC value. 
  p1 = NTC_table[ (adc_value >> 7)  ];
  p2 = NTC_table[ (adc_value >> 7)+1];
  // Interpolate between both points. 
  return p1 - ( (p1-p2) * (adc_value & 0x007F) ) / 128;
};

uint8_t * TXponter;
uint8_t TXcouter;
void UART_tx(void)
{
  TXcouter = sizeof(Feedback);
  TXponter = (uint8_t *)&Feedback; 
  USART3->CR1 |= USART_CR1_TXEIE;

  while (!(USART3->SR & USART_SR_TXE));
  USART3->DR = *TXponter++;
  TXcouter--;
}


SerialCommand command_raw;
extern uint16_t timeoutCntSerial_R;
extern uint8_t  timeoutFlgSerial_R;
extern SerialCommand commandR; 
void USART3_IRQHandler(void)
{
  if (USART3->SR & USART_SR_TXE) // tx is not empty
  {
    USART3->DR = *TXponter++;
    TXcouter--;
    if (TXcouter==0)   USART3->CR1 &= ~USART_CR1_TXEIE;  // disable interrupt
  }
  else if(USART3->SR & USART_SR_RXNE)  // rx is not empty
  {
    static uint8_t state=0;
    static uint8_t * buffPointer;
    USART3->SR &= ~USART_SR_RXNE; // just in case
    uint8_t data;
    data = USART3->DR;
    if (state==0)
    {
      if (data == (SERIAL_START_FRAME & 0xFF)) state++;
    }
    else if (state==1)
    {
      if (data == (SERIAL_START_FRAME >> 8))
      {
        state++;
        buffPointer = (uint8_t *)&command_raw + 2;
        command_raw.start = SERIAL_START_FRAME;
      }
      else state=0;
    }
    else if(state<8)
    {
      *buffPointer = data;
      buffPointer++;
      if (state==7)
      {
        state = 0;  
        uint16_t checksum = (uint16_t)(command_raw.start ^ command_raw.steer ^ command_raw.speed);
        //uint16_t checksum = (uint16_t)(command_raw.start ^ command_raw.speed);
        if (command_raw.checksum == checksum) 
        {
          commandR = command_raw;
          timeoutFlgSerial_R = 0;         // Clear timeout flag
          timeoutCntSerial_R = 0;         // Reset timeout counter           
        }
      }
      else state++;
    }
  }
}


int main(void) 
{

  uint32_t    buzzerTimer_prev = 0;
  int16_t speed=0;

  HAL_Init();

  RCC->APB2ENR |=  RCC_APB2ENR_AFIOEN;
  AFIO->MAPR |= AFIO_MAPR_PD01_REMAP;    // remap PD0, PD1
  AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_JTAGDISABLE; // disable jtag

  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
  // System interrupt init
  /* PendSV_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(PendSV_IRQn, 0, 0);
  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

  SystemClock_Config();

  RCC->AHBENR &= ~RCC_AHBENR_DMA1EN;   // DMA1CLK_DISABLE();
  MX_GPIO_Init();
  MX_TIM_Init();
  MX_ADC1_Init();
  BLDC_Init();        // BLDC Controller Init

  OFF_PORT->BSRR = 1<<OFF_PIN;   // Activate Latch  
  Input_Lim_Init();   // Input Limitations Init
  Input_Init();       // Input Init

  // Start ADC conversion on regular group with external trigger 
  ADC1->CR2 |= ADC_CR2_EXTTRIG;  // 

  poweronMelody();
  LED_PORT->BRR = 1<<LED_PIN;
  
  // Loop until button is released
  while(BUTTON_PORT->IDR & (1<<BUTTON_PIN))  HAL_Delay(10); 

  while(1) 
  {
    if (buzzerTimer - buzzerTimer_prev > 16*DELAY_IN_MAIN_LOOP) // 1 ms = 16 ticks buzzerTimer
    {  
      readCommand();                        // Read Command: input1[0].cmd, input[0].cmd
      calcAvgSpeed();                       // Calculate average measured speed: speedAvg, speedAvgAbs

      // ####### MOTOR ENABLING: Only if the initial input is very small (for SAFETY) #######
      if (enable == 0 && !rtY.z_errCode && ABS(input.cmd) < 50)
      {
        beepShort(6);                     // make 2 beeps indicating the motor enable
        beepShort(4); 
        HAL_Delay(100);
        speedFixdt = 0;      // reset filters
        enable = 1;          // enable motors
      }


      // ####### LOW-PASS FILTER #######
      rateLimiter16(input.cmd, rate, &speedRateFixdt);
      filtLowPass32(speedRateFixdt >> 4, FILTER, &speedFixdt);
      speed = (int16_t)(speedFixdt >> 16);  // convert fixed-point to integer
      pwmr = speed;

      // ####### CALC BOARD TEMPERATURE #######
      board_temp_deg_c = NTC_ADC2Temperature(adc_buffer.temp);
      
      // ####### CALC CALIBRATED BATTERY VOLTAGE #######
      batVoltageCalib = batVoltage * BAT_CALIB_REAL_VOLTAGE / BAT_CALIB_ADC;

      // ####### FEEDBACK SERIAL OUT #######
      if (main_loop_counter % 2 == 0)     // Send data periodically every 10 ms
      {
        if (!(USART3->CR1 & USART_CR1_TXEIE)) // tx is empty
        {
          Feedback.start	      = SERIAL_START_FRAME;
          Feedback.cmd          = input.cmd;
          Feedback.cmd1         = 0;
          Feedback.speed_meas	  = rtY.n_mot;
          Feedback.speedR_meas	= 0;
          Feedback.batVoltage	  = batVoltageCalib;
          Feedback.boardTemp	  = board_temp_deg_c;    
          Feedback.cmdLed       = 0;
          Feedback.checksum     = Feedback.start ^ Feedback.cmd ^ Feedback.cmd1 
                                                 ^ Feedback.speed_meas  ^ Feedback.speedR_meas
                                                 ^ Feedback.batVoltage ^ Feedback.boardTemp
                                                 ^ Feedback.cmdLed;
          UART_tx();
        }
      }



      // ####### POWEROFF BY POWER-BUTTON #######
      poweroffPressCheck();

      // ####### BEEP AND EMERGENCY POWEROFF #######
      if (TEMP_POWEROFF_ENABLE && board_temp_deg_c >= TEMP_POWEROFF && speedAvgAbs < 20)  // poweroff before mainboard burns OR low bat 3
        poweroff();
      else if ( BAT_DEAD_ENABLE && batVoltage < BAT_DEAD && speedAvgAbs < 20)
        poweroff();
      else if (rtY.z_errCode)    // 1 beep (low pitch): Motor error, disable motors
      {
        enable = 0;
        beepCount(1, 24, 1);
      }
      else if (timeoutFlgSerial)        // 3 beeps (low pitch): Serial timeout
        beepCount(3, 24, 1);
      else if (TEMP_WARNING_ENABLE && board_temp_deg_c >= TEMP_WARNING)     // 5 beeps (low pitch): Mainboard temperature warning
        beepCount(5, 24, 1);
      else if (BAT_LVL1_ENABLE && batVoltage < BAT_LVL1)      // 1 beep fast (medium pitch): Low bat 1
        beepCount(0, 10, 6);
      else if (BAT_LVL2_ENABLE && batVoltage < BAT_LVL2)     // 1 beep slow (medium pitch): Low bat 2 
        beepCount(0, 10, 30);
      else if (BEEPS_BACKWARD && (((speed < -50) && speedAvg < 0) )) // 1 beep fast (high pitch): Backward spinning motors
        beepCount(0, 5, 1); 
      else beepCount(0, 0, 0);  // do not beep
      

      // ####### INACTIVITY TIMEOUT #######
      inactivity_timeout_counter++;
      if ( abs(speed) > 50) inactivity_timeout_counter = 0;

      if (inactivity_timeout_counter > (INACTIVITY_TIMEOUT * 60 * 1000) / (DELAY_IN_MAIN_LOOP + 1))  // rest of main loop needs maybe 1ms
         poweroff();
      

      // Update states
      buzzerTimer_prev = buzzerTimer;
      main_loop_counter++;
    }
  }
}


// ===========================================================
/** System Clock Configuration
*/
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL          = RCC_PLL_MUL16;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType           = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource        = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider       = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider      = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider      = RCC_HCLK_DIV1;

  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  PeriphClkInit.PeriphClockSelection    = RCC_PERIPHCLK_ADC;
  // PeriphClkInit.AdcClockSelection    = RCC_ADCPCLK2_DIV8;  // 8 MHz
  PeriphClkInit.AdcClockSelection       = RCC_ADCPCLK2_DIV4;  // 16 MHz
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  /**Configure the Systick interrupt time
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

  /**Configure the Systick
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}
