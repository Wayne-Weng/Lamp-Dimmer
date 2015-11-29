/* #include "experiment_stm32f3.h" */

#ifndef __EXPERIMENT_STM32F3_H
#define __EXPERIMENT_STM32F3_H

#include "stm32f30x.h"
/*====================================================================================================*/
/*====================================================================================================*/
#define LED_R_Set     GPIOC->BSRR = GPIO_Pin_15
#define LED_R_Reset   GPIOC->BRR  = GPIO_Pin_15
#define LED_R_Toggle  GPIOC->ODR ^= GPIO_Pin_15
#define LED_G_Set     GPIOC->BSRR = GPIO_Pin_14
#define LED_G_Reset   GPIOC->BRR  = GPIO_Pin_14
#define LED_G_Toggle  GPIOC->ODR ^= GPIO_Pin_14
#define LED_B_Set     GPIOC->BSRR = GPIO_Pin_13
#define LED_B_Reset   GPIOC->BRR  = GPIO_Pin_13
#define LED_B_Toggle  GPIOC->ODR ^= GPIO_Pin_13

#define KEY_WU_Read   ((GPIOA->IDR & GPIO_Pin_0) == GPIO_Pin_0)
#define KEY_BO_Read   ((GPIOB->IDR & GPIO_Pin_2) == GPIO_Pin_2)
/*====================================================================================================*/
/*====================================================================================================*/
void Main_Mode_Change( void );
void Func_Mode_Toggle( void );
void Mode_Task( void );
void GPIO_Config( void );
void EXTI_Config( void );
void TIM_Config( void );
void ADC_Config( void );
void ZCDelayConfig( void );
void PhaseModConfig( void );
void PulseConfigL( void );
void PulseConfigR( void );
uint16_t ADC_ReadData( ADC_TypeDef* ADCx );
void SamplingDSP( void );
void ManualDSP( void );
void FuncGenDSP( void );
/*====================================================================================================*/
/*====================================================================================================*/
extern volatile uint8_t MainMode;

#define MAIN_MODE_AUDIO 0
#define MAIN_MODE_MANUAL 1
#define MAIN_MODE_BREATH 2

/*====================================================================================================*/
/*====================================================================================================*/
#endif
