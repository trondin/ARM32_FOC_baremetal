/*
* This file implements FOC motor control.
* This control method offers superior performanace
* compared to previous cummutation method. The new method features:
* ► reduced noise and vibrations
* ► smooth torque output
* ► improved motor efficiency -> lower energy consumption
*/

#include "stm32f1xx_hal.h"
#include "defines.h"
#include "setup.h"
#include "config.h"
#include "util.h"

// Matlab includes and defines - from auto-code generation
// ###############################################################################
#include "BLDC_controller.h"           /* Model's header file */
#include "rtwtypes.h"

extern RT_MODEL *const rtM;

extern P    rtP;  // why only left?

extern DW   rtDW;                 /* Observable states */
extern ExtU rtU;                  /* External inputs */
extern ExtY rtY;                  /* External outputs */
// ###############################################################################

static int16_t pwm_margin;              /* This margin allows to have a window in the PWM signal for proper FOC Phase currents measurement */

extern uint8_t ctrlModReq;
static int16_t curDC_max = (I_DC_MAX * A2BIT_CONV);
int16_t cur_phaB = 0, cur_phaC = 0, cur_DC = 0;

volatile int pwmr = 0;

extern volatile adc_buf_t adc_buffer;

uint8_t buzzerFreq          = 0;
uint8_t buzzerPattern       = 0;
uint8_t buzzerCount         = 0;
volatile uint32_t buzzerTimer = 0;
static uint8_t  buzzerPrev  = 0;
static uint8_t  buzzerIdx   = 0;

uint8_t        enable       = 0;        // initially motors are disabled for SAFETY
static uint8_t enableFin    = 0;

//static const uint16_t pwm_res  = 64000000 / 2 / PWM_FREQ; // = 2000
#define  PWM_RES   (64000000 / 2 / PWM_FREQ)          // = 2000


int16_t        batVoltage       = (400 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE;
static int32_t batVoltageFixdt  = (400 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE << 16;  // Fixed-point filter output initialized at 400 V*100/cell = 4 V/cell converted to fixed-point

// =================================
// DMA interrupt frequency =~ 16 kHz
// =================================
void DMA1_Channel1_IRQHandler(void) 
{
  DMA1->IFCR = DMA_IFCR_CTCIF1;
  
  //(GPIOB->ODR & (1<<6)) ? (GPIOB->BRR = 1<<6) : (GPIOB->BSRR = 1<<6);   //test

  // calibrate ADC offsets
  static uint8_t DummyCnt=100;
  if (DummyCnt >0) { DummyCnt--; return; }

  static int16_t offsetcount = 1024;
  static int16_t offsetrrA    = 0;
  static int16_t offsetrrB    = 0;
  static int16_t offsetrrC    = 0;
  static int32_t summaA=0;
  static int32_t summaB=0;
  static int32_t summaC=0;  

  if(offsetcount>0)
  {
    summaA += adc_buffer.phA;
    summaB += adc_buffer.phB;    
    summaC += adc_buffer.phC;
    offsetcount--;
    if(offsetcount!=0) return;
    else
    {
      offsetrrA = (int16_t)(summaA/1024);
      offsetrrB = (int16_t)(summaB/1024);
      offsetrrC = (int16_t)(summaC/1024);
    }
  }

  // Filter battery voltage at a slower sampling rate
  if (buzzerTimer % 1000 == 0) 
  {  
    filtLowPass32(adc_buffer.batt1, BAT_FILT_COEF, &batVoltageFixdt);
    batVoltage = (int16_t)(batVoltageFixdt >> 16);  // convert fixed-point to integer
  }

  // Get Right motor currents
  cur_phaB = (int16_t)(offsetrrB - adc_buffer.phB);
  cur_phaC = (int16_t)(offsetrrC - adc_buffer.phC);
  cur_DC   = (int16_t)(offsetrrA - adc_buffer.phA) + cur_phaB + cur_phaC;

  // Disable PWM when current limit is reached (current chopping)
  // This is the Level 2 of current protection. The Level 1 should kick in first given by I_MOT_MAX
  if(ABS(cur_DC)  > curDC_max || enable == 0) TIM1->BDTR &= ~TIM_BDTR_MOE;
  else                                         TIM1->BDTR |= TIM_BDTR_MOE;


  // Create square wave for buzzer
  buzzerTimer++;
  if (buzzerFreq != 0 && (buzzerTimer / 5000) % (buzzerPattern + 1) == 0) 
  {
    if (buzzerPrev == 0)
    {
      buzzerPrev = 1;
      // pause 2 periods
      if (++buzzerIdx > (buzzerCount + 2)) buzzerIdx = 1;
    }
    if (buzzerTimer % buzzerFreq == 0 && (buzzerIdx <= buzzerCount || buzzerCount == 0))
    {
      (BUZZER_PORT->ODR & (1<<BUZZER_PIN)) ? (BUZZER_PORT->BRR = 1<<BUZZER_PIN) : (BUZZER_PORT->BSRR = 1<<BUZZER_PIN);
    }
  }
  else
    if (buzzerPrev)
    {
      BUZZER_PORT->BRR = 1<<BUZZER_PIN;
      buzzerPrev = 0;
    }

  // Adjust pwm_margin depending on the selected Control Type
  if (rtP.z_ctrlTypSel == FOC_CTRL) pwm_margin = 110;
  else                                   pwm_margin = 0;
  
  // ############################### MOTOR CONTROL ###############################
  int ur, vr, wr;
  static boolean_T OverrunFlag = false;

  // Check for overrun 
  if (OverrunFlag) return;
  OverrunFlag = true;
  // Make sure to stop BOTH motors in case of an error 
  enableFin = enable && !rtY.z_errCode;

  // ========================= RIGHT MOTOR ===========================  
  // Get hall sensors values
  uint8_t hall_u = !(HALL_U_PORT->IDR & (1<<HALL_U_PIN));
  uint8_t hall_v = !(HALL_V_PORT->IDR & (1<<HALL_V_PIN));
  uint8_t hall_w = !(HALL_W_PORT->IDR & (1<<HALL_W_PIN));
  //(hall_w !=0) ? (LED_PORT->BSRR = 1<<LED_PIN) : (LED_PORT->BRR = 1<<LED_PIN);  //test


  // Set motor inputs here 
  rtU.b_motEna      = enableFin;
  rtU.z_ctrlModReq  = ctrlModReq;
  rtU.r_inpTgt      = pwmr;
  rtU.b_hallA       = hall_u;
  rtU.b_hallB       = hall_v;
  rtU.b_hallC       = hall_w;
  rtU.i_phaAB       = cur_phaB;
  rtU.i_phaBC       = cur_phaC;
  rtU.i_DCLink      = cur_DC;

  // Step the controller 
  BLDC_controller_step(rtM);

  // Get motor outputs here 
  ur            = rtY.DC_phaA;
  vr            = rtY.DC_phaB;
  wr            = rtY.DC_phaC;

  // Apply commands 
  TIM1->CCR1  = (uint16_t)CLAMP(ur + PWM_RES / 2, pwm_margin, PWM_RES-pwm_margin);
  TIM1->CCR2  = (uint16_t)CLAMP(vr + PWM_RES / 2, pwm_margin, PWM_RES-pwm_margin);
  TIM1->CCR3  = (uint16_t)CLAMP(wr + PWM_RES / 2, pwm_margin, PWM_RES-pwm_margin);
  // =================================================================

  /* Indicate task complete */
  OverrunFlag = false;
 
 // ###############################################################################

}
