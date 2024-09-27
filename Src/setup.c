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

/*
tim1 master, enable -> trgo
tim8, gated slave mode, trgo by tim1 trgo. overflow -> trgo
adc1,adc2 triggered by tim8 trgo
adc 1,2 dual mode

ADC1             ADC2
R_Blau PC4 CH14  R_Gelb PC5 CH15
L_Grün PA0 CH01  L_Blau PC3 CH13
R_DC PC1 CH11    L_DC PC0 CH10
BAT   PC2 CH12   L_TX PA2 CH02
BAT   PC2 CH12   L_RX PA3 CH03

pb10 usart3 dma1 channel2/3
*/

#include "defines.h"
#include "config.h"
#include "setup.h"


volatile adc_buf_t adc_buffer;



/* USART3 init function */
void UART3_Init(void)
{

  RCC->AHBENR |= RCC_AHBENR_DMA1EN; 
  RCC->APB1ENR  |=  RCC_APB1ENR_USART3EN;
  RCC->APB2ENR |=  RCC_APB2ENR_IOPBEN; 
  // PB10     ------> USART3_TX
  GPIOB->CRH &= ~((0xF << (10-8)*4));
  GPIOB->CRH |= (0xB << (10-8)*4);
  // PB11     ------> USART3_RX pullup input 
  GPIOB->CRH &= ~((0xF << (11-8)*4));
  GPIOB->CRH |= (0x8 << (11-8)*4);
  GPIOB->ODR |= 1 << 11;

  uint16_t uartdiv = 64000000 /2 / USART3_BAUD;  
  USART3->BRR = ((( uartdiv / 16 ) << USART_BRR_DIV_Mantissa_Pos) |
                 (( uartdiv % 16 ) << USART_BRR_DIV_Fraction_Pos));
  USART3->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE | USART_CR1_RXNEIE;

  NVIC_SetPriority (USART3_IRQn, 0);
  NVIC_EnableIRQ(USART3_IRQn);  
}


void MX_GPIO_Init(void) 
{

  // GPIO Ports Clock Enable 
  RCC->APB2ENR |=  RCC_APB2ENR_AFIOEN |  RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN | RCC_APB2ENR_IOPDEN; 

  // float inpits  Hall sensors
  HALL_U_PORT->CRL &= ~((0xF << HALL_U_PIN*4));
  HALL_U_PORT->CRL |= (4 << HALL_U_PIN*4);
  HALL_V_PORT->CRL &= ~((0xF << HALL_V_PIN*4));
  HALL_V_PORT->CRL |= (4 << HALL_V_PIN*4);
  HALL_W_PORT->CRL &= ~((0xF << HALL_W_PIN*4));
  HALL_W_PORT->CRL |= (4 << HALL_W_PIN*4);


  // on/off button - float inpit
  BUTTON_PORT->CRH &= ~((0xF << (BUTTON_PIN-8)*4));  
  BUTTON_PORT->CRH |= (4 << (BUTTON_PIN-8)*4);

  // buzzer pushpull output
  BUZZER_PORT->CRH &= ~((0xF << (BUZZER_PIN-8)*4));  
  BUZZER_PORT->CRH |= (2 << (BUZZER_PIN-8)*4);
  // LED  pushpull output
  LED_PORT->CRL &= ~((0xF << LED_PIN*4));  
  LED_PORT->CRL |= (2 << LED_PIN*4);
  LED_PORT->BSRR = 1<<LED_PIN;
  // off  pushpull output
  OFF_PORT->CRH &= ~((0xF << (OFF_PIN-8)*4));  
  OFF_PORT->CRH |= (2 << (OFF_PIN-8)*4);
  // test PB6
  GPIOB->CRL &= ~(0xF << 6*4);  
  GPIOB->CRL |= (2 << 6*4);

}

//#define TIM1_ONLY

void MX_TIM_Init(void) 
{
  RCC->APB2ENR |=  RCC_APB2ENR_TIM1EN;
  RCC->APB2ENR |=   RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN;

  //AFIO->MAPR |= AFIO_MAPR_TIM1_REMAP_PARTIALREMAP;    // remap tim1
  AFIO->MAPR &= ~AFIO_MAPR_TIM1_REMAP_FULLREMAP_Msk;

  // alternative PWM pullpush output ports
  TIM_UH_PORT->CRH &= ~((0xF << (TIM_UH_PIN-8)*4));
  TIM_UH_PORT->CRH |= (0xA << (TIM_UH_PIN-8)*4);

  TIM_VH_PORT->CRH &= ~((0xF << (TIM_VH_PIN-8)*4));
  TIM_VH_PORT->CRH |= (0xA << (TIM_VH_PIN-8)*4);  

  TIM_WH_PORT->CRH &= ~((0xF << (TIM_WH_PIN-8)*4));
  TIM_WH_PORT->CRH |= (0xA << (TIM_WH_PIN-8)*4);    

  TIM_UL_PORT->CRH &= ~((0xF << (TIM_UL_PIN-8)*4));
  TIM_UL_PORT->CRH |= (0xA << (TIM_UL_PIN-8)*4);   
  
  TIM_VL_PORT->CRH &= ~((0xF << (TIM_VL_PIN-8)*4));
  TIM_VL_PORT->CRH |= (0xA << (TIM_VL_PIN-8)*4); 

  TIM_WL_PORT->CRH &= ~((0xF << (TIM_WL_PIN-8)*4));
  TIM_WL_PORT->CRH |= (0xA << (TIM_WL_PIN-8)*4);  
  // -------------

  TIM1->CR1 = TIM_CR1_CMS_0; // Center-aligned mode 1
  TIM1->PSC = 0;
  TIM1->ARR = 64000000 / 2 / PWM_FREQ;
  TIM1->RCR = 1; 
  // master/slave
  TIM1->CR2 = TIM_CR2_MMS_0;  // master TRGO
  // ConfigChannel 
  TIM1->CCMR1  = TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1;   
  TIM1->CCMR2  = TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1;  
  //TIM1->CCMR2 |= TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_0;  // PWM2 for CCR4 ????????????
  // break
  TIM1->BDTR = TIM_BDTR_OSSR | TIM_BDTR_OSSI | DEAD_TIME;
  // enables
  TIM1->CCER = TIM_CCER_CC1E | TIM_CCER_CC1NE 
             | TIM_CCER_CC2E | TIM_CCER_CC2NE 
             | TIM_CCER_CC3E | TIM_CCER_CC3NE;  
  //TIM1->CCER |= TIM_CCER_CC1NP | TIM_CCER_CC2NP |  TIM_CCER_CC3NP; 
  TIM1->BDTR |= TIM_BDTR_MOE;

  // start  
  TIM1->CR1 |= TIM_CR1_CEN; 


  // timer2
  RCC->APB1ENR |=  RCC_APB1ENR_TIM3EN; 
  TIM3->CR1 = TIM_CR1_CMS_0;  //Center-aligned mode 1
  TIM3->PSC = 1;
  TIM3->ARR = 64000000 / 2/ PWM_FREQ;  
  // master/slave
  TIM3->CR2 = TIM_CR2_MMS_1;
  // Slave Config Synchronization
  TIM3->SMCR = TIM_SMCR_SMS_2  | TIM_SMCR_SMS_0;   // Gated Mode 
  TIM3->RCR = 1;  
  // start
  TIM3->CR1 |= TIM_CR1_CEN;  
}


void MX_ADC1_Init(void)
{
  RCC->APB2ENR |=  RCC_APB2ENR_ADC1EN | RCC_APB2ENR_IOPAEN;
  // analog input current sensor pins
  // ADC_CHANNEL 0(PA0), 2(PA2), 3(PA3), 4(PA4), 5(PA5),  
  GPIOA->CRL &= ~((0xF << 0*4) | (0xF << 2*4) | (0xF << 3*4) | (0xF << 4*4)  | (0xF << 5*4));

  ADC1->CR1 = ADC_CR1_SCAN ;     // ADC SCAN ENABLE  
  ADC1->CR2 = ADC_CR2_DMA | ADC_CR2_TSVREFE | ADC_CR2_ADON;
  // Enable or disable the remapping of ADC1_ETRGREG:
  // ADC1 External Event regular conversion is connected to TIM8 TRG0
  ADC1->CR2 |= 4 << ADC_CR2_EXTSEL_Pos;    //  TIMER3

  // number of convertson
  ADC1->SQR1 = 4 << ADC_SQR1_L_Pos;  

  // 1 - ADC_CHANNEL_3 phase A   
  //ADC1->SMPR2 = ADC_SAMPLETIME_7CYCLES_5 << ADC_SMPR2_SMP3_Pos;
  ADC1->SQR3 = 3 << ADC_SQR3_SQ1_Pos;
  
  // 2 - ADC_CHANNEL_4 phase B   
  //ADC1->SMPR2 = ADC_SAMPLETIME_7CYCLES_5 << ADC_SMPR2_SMP4_Pos;
  ADC1->SQR3 |= 4 << ADC_SQR3_SQ2_Pos;

  // 3 -  ADC_CHANNEL_5 phase C 
  //ADC1->SMPR2 |= ADC_SAMPLETIME_7CYCLES_5 << ADC_SMPR2_SMP5_Pos;
  ADC1->SQR3 |= 5 << ADC_SQR3_SQ3_Pos;

  // 4 ADC_CHANNEL_2;  // pa2 vbat
  ADC1->SMPR2 |= ADC_SAMPLETIME_7CYCLES_5 << ADC_SMPR2_SMP2_Pos;
  ADC1->SQR3 |= 2 << ADC_SQR3_SQ4_Pos;

  //5 - ADC_CHANNEL_0 TEMPSENSOR;  // NTC
  ADC1->SMPR2 |= ADC_SAMPLETIME_7CYCLES_5 << ADC_SMPR2_SMP0_Pos;
  ADC1->SQR3 |= 0 << ADC_SQR3_SQ5_Pos;

  //ADC1->CR2 |= ADC_CR2_ADON;
  // calibration	
  HAL_Delay(1);
  ADC1->CR2 |= ADC_CR2_CAL;
  while ((ADC1->CR2 & ADC_CR2_CAL) != 0) { };
  //


  RCC->AHBENR |=  RCC_AHBENR_DMA1EN;

  DMA1_Channel1->CCR   = 0;
  DMA1_Channel1->CNDTR = 5;
  DMA1_Channel1->CPAR  = (uint32_t)&(ADC1->DR);
  DMA1_Channel1->CMAR  = (uint32_t)&adc_buffer;
  DMA1_Channel1->CCR     = DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0 | DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_TCIE; 
  DMA1_Channel1->CCR |= DMA_CCR_EN;

  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}
