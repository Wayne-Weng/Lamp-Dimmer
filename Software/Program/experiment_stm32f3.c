/*====================================================================================================*/
/*====================================================================================================*/
#include "stm32f3_system.h"
#include "experiment_stm32f3.h"

#include <math.h>
#include "arm_math.h"

/*====================================================================================================*/
/*====================================================================================================*/
#define OUT_LEVEL_MAX 5000
#define ENV_LEVEL_MAX 2048
#define ENV_LOG_MIN_THRES 20.0

#define ATTACK_TIME_MS   10.0
#define RELEASE_TIME_MS  500.0
#define SAMPLING_FREQ 2500.0/*45454.4545454545*/
#define ATTACK_TIME_MS_MAX 1000.0
#define ATTACK_TIME_MS_MIN_THRES 0.1
#define RELEASE_TIME_MS_MAX 2000.0
#define RELEASE_TIME_MS_MIN_THRES 0.1

volatile float LevelOutL = 0;
volatile float LevelOutR = 0;

volatile float AttackCoef = 1;
volatile float ReleaseCoef = 1;
volatile float LogCoef = 0;

#define AUDIO_LEVEL_BIAS 2047

volatile uint16_t ADC_Data_AudioL = AUDIO_LEVEL_BIAS;
volatile uint16_t ADC_Data_AudioR = AUDIO_LEVEL_BIAS;
volatile uint16_t ADC_Data_POT[2] = {0, 0};

#define POT_LEVEL_MAX_VELOCITY 4095
#define POT_LEVEL_MAX_FREQ 4095
//#define VEL_LOG_MIN_THRES 500.0

//volatile float PowCoefVelocity = 0;

#define POT_LEVEL_MAX_ATTACK 4095
#define POT_LEVEL_MAX_RELEASE 4095

volatile float AttackTimeMS = 0.0;
volatile float ReleaseTimeMS = 0.0;

volatile uint16_t POTLevelAttack = 0;
volatile uint16_t POTLevelRelease = 0;

volatile float PowCoefAttack = 0;
volatile float PowCoefRelease = 0;

/*=====================================================================================================*/
/*=====================================================================================================*/
#define FUNC_MODE_CLOSED 0
#define FUNC_MODE_A 1
#define FUNC_MODE_B 2

volatile uint8_t MainMode = MAIN_MODE_AUDIO;
volatile uint8_t FuncMode = FUNC_MODE_CLOSED;

int main( void )
{
  GPIO_Config();
	ADC_Config();
	TIM_Config();
	EXTI_Config();
	
	//Pre-calculate the AttackCoef and ReleaseCoef from pre-set ATTACK_TIME_MS and RELEASE_TIME_MS as initial value
	AttackCoef = pow(0.01, 1.0/((double)ATTACK_TIME_MS*(double)SAMPLING_FREQ*0.001));
	ReleaseCoef = pow(0.01, 1.0/((double)RELEASE_TIME_MS*(double)SAMPLING_FREQ*0.001));
	
	//Pre-calculate the LogCoef for Audio Envelope Velocity Log-Mapping Calculation 
	LogCoef = (float)OUT_LEVEL_MAX/log10((double)ENV_LEVEL_MAX/(double)ENV_LOG_MIN_THRES);
	
	//Pre-calculate the PowCoefAttack and PowCoefRelease for Linear-to-Log mapping of POT value to AttackTimeMS and ReleaseTimeMS
	PowCoefAttack = log10((double)ATTACK_TIME_MS_MAX/(double)ATTACK_TIME_MS_MIN_THRES)/(float)POT_LEVEL_MAX_ATTACK;
	PowCoefRelease = log10((double)RELEASE_TIME_MS_MAX/(double)RELEASE_TIME_MS_MIN_THRES)/(float)POT_LEVEL_MAX_RELEASE;
	//PowCoefVelocity = log10((double)OUT_LEVEL_MAX/(double)VEL_LOG_MIN_THRES)/(float)POT_LEVEL_MAX_VELOCITY;
	
  while(1) {
		
		//Change mode of operation when MODE_SELECT is pressed
		if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) != RESET){
			//Reset FuncMode to FUNC_MODE_CLOSED and clear indication LEDs
			GPIO_SetBits(GPIOC, GPIO_Pin_15);
			GPIO_SetBits(GPIOC, GPIO_Pin_13);
			FuncMode = FUNC_MODE_CLOSED;
			//Change MainMode
			Main_Mode_Change();
			//De-Noise
			Delay_10ms(2);
			//Prevent multiple button triggers while MODE_SELECT is held
			while(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) != RESET)	Mode_Task();
			//De-Noise
			Delay_10ms(2);
		}
		
		//Change mode of operation when MODE_FUNCTION is pressed
		if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_2) != RESET){
			//Toggle FuncMode
			Func_Mode_Toggle();
			//De-Noise
			Delay_10ms(2);
			//Prevent multiple button triggers while MODE_FUNCTION is held
			while(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_2) != RESET)	Mode_Task();
			//De-Noise
			Delay_10ms(2);
		}
		
		Mode_Task();

  }
}

void Main_Mode_Change( void )
{
	switch(MainMode)
	{
		case MAIN_MODE_AUDIO:
			
			//Stop(Pause) Sampling, therefore stop the LevelOutL and LevelOutR varying by DSP Envelope Follower
			TIM_Cmd(TIM4, DISABLE);
			MainMode = MAIN_MODE_MANUAL;
			break;
		
		case MAIN_MODE_MANUAL:
			
			//Change the MainMode first then start TIM4, to ensure its MainMode select routine doesn't go to wrong path
			MainMode = MAIN_MODE_BREATH;
			//Start/Resume Sampling, therefore proceed the DSP Function Generator
			TIM_Cmd(TIM4, ENABLE);
			break;
		
		case MAIN_MODE_BREATH:
			
			//Change the MainMode directly, therefore the MainMode select routine of TIM4 will proceed the DSP Envelope Follower soon
			MainMode = MAIN_MODE_AUDIO;
			break;
	}
}

void Func_Mode_Toggle( void )
{
	switch(FuncMode)
	{
		case FUNC_MODE_CLOSED:
			GPIO_ResetBits(GPIOC, GPIO_Pin_15);
			FuncMode = FUNC_MODE_A;
			break;
		
		case FUNC_MODE_A:
			GPIO_SetBits(GPIOC, GPIO_Pin_15);
			GPIO_ResetBits(GPIOC, GPIO_Pin_13);
			FuncMode = FUNC_MODE_B;
			break;	
		
		case FUNC_MODE_B:
			GPIO_SetBits(GPIOC, GPIO_Pin_13);
			GPIO_SetBits(GPIOC, GPIO_Pin_15);
			FuncMode = FUNC_MODE_CLOSED;
			break;	
	}
}

void Mode_Task( void )
{
	switch(MainMode)
	{
		case MAIN_MODE_AUDIO:
			
			if(FuncMode == FUNC_MODE_A)	{
				
				POTLevelAttack = ADC_Data_POT[0];
				POTLevelRelease = ADC_Data_POT[1];
				
				//Logarithmic-Mapped POT value to AttackTimeMS and ReleaseTimeMS
				AttackTimeMS = pow(10.0,(double)POTLevelAttack*(double)PowCoefAttack) * ATTACK_TIME_MS_MIN_THRES;
				ReleaseTimeMS = pow(10.0,(double)POTLevelRelease*(double)PowCoefRelease) * RELEASE_TIME_MS_MIN_THRES;
				
				//Calculate corresponding AttackCoef and ReleaseCoef from the tuned AttackTimeMS and ReleaseTimeMS times
				AttackCoef = pow(0.01, 1.0/((double)AttackTimeMS*(double)SAMPLING_FREQ*0.001));
				ReleaseCoef = pow(0.01, 1.0/((double)ReleaseTimeMS*(double)SAMPLING_FREQ*0.001));
				
			}
			
			break;
		
		case MAIN_MODE_MANUAL:
			
			ManualDSP();
			break;
		
		case MAIN_MODE_BREATH:
		
		
			break;
	}
}
/*====================================================================================================*/
/*====================================================================================================*/
void GPIO_Config( void )
{
  GPIO_InitTypeDef GPIO_InitStruct;
	
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA |  RCC_AHBPeriph_GPIOB |  RCC_AHBPeriph_GPIOC, ENABLE);
	
	// PA6 = PULSE_OUT_LEFT, PA7 = PULSE_OUT_RIGHT
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	// PA0 = MODE_SELECT
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	// PB2 = MODE_FUNCTION
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	// PC15 = FUNC_MODE_A Red LED indicator
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_15;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIO_InitStruct);
	GPIO_SetBits(GPIOC, GPIO_Pin_15);
	
	// PC15 = FUNC_MODE_A Red LED indicator
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIO_InitStruct);
	GPIO_SetBits(GPIOC, GPIO_Pin_13);
}
/*====================================================================================================*/
/*====================================================================================================*/
void ADC_Config( void )
{
	volatile uint32_t calibration_value_ADC1 = 0;
	volatile uint32_t calibration_value_ADC2 = 0;
	volatile uint32_t calibration_value_ADC3 = 0;
	
	DMA_InitTypeDef DMA_InitStruct;
	ADC_InitTypeDef ADC_InitStruct;
	ADC_CommonInitTypeDef ADC_CommonInitStruct;
	GPIO_InitTypeDef GPIO_InitStruct;
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1 | RCC_AHBPeriph_DMA2, ENABLE);
	
	// ADC1_IN2 -> DMA1_CH1 -> ADC_Data_AudioL
	DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
	DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t)&ADC_Data_AudioL;
	DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStruct.DMA_BufferSize = 1;
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStruct.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStruct.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStruct);
	
	// ADC2_IN1 -> DMA2_CH1 -> ADC_Data_AudioR
	DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&ADC2->DR;
	DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t)&ADC_Data_AudioR;
	DMA_Init(DMA2_Channel1, &DMA_InitStruct);
	
	// ADC3_IN12, ADC3_CH1 -> DMA2_CH5 -> ADC_Data_POT[0], ADC_Data_POT[1]
	DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&ADC3->DR;
	DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t)&ADC_Data_POT[0];
	DMA_InitStruct.DMA_BufferSize = 2;
	DMA_Init(DMA2_Channel5, &DMA_InitStruct);
	
	RCC_ADCCLKConfig(RCC_ADC12PLLCLK_Div1);
	RCC_ADCCLKConfig(RCC_ADC34PLLCLK_Div1);
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB, ENABLE);
	
	// PA1 = ADC_IN_LEFT, PA4 = ADC_IN_RIGHT
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_4;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	// PB0 = ATTACK, PB1 = RELEASE
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ADC12 | RCC_AHBPeriph_ADC34, ENABLE);
	
	ADC_CommonInitStruct.ADC_Mode = ADC_Mode_Independent;                                                                    
  ADC_CommonInitStruct.ADC_Clock = ADC_Clock_AsynClkMode;                    
  ADC_CommonInitStruct.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;             
  ADC_CommonInitStruct.ADC_DMAMode = ADC_DMAMode_OneShot;                  
  ADC_CommonInitStruct.ADC_TwoSamplingDelay = 0;
  ADC_CommonInit(ADC1, &ADC_CommonInitStruct);
	ADC_CommonInit(ADC2, &ADC_CommonInitStruct);
	ADC_CommonInit(ADC3, &ADC_CommonInitStruct);
	
	ADC_InitStruct.ADC_ContinuousConvMode = ADC_ContinuousConvMode_Enable;
  ADC_InitStruct.ADC_Resolution = ADC_Resolution_12b; 
  ADC_InitStruct.ADC_ExternalTrigConvEvent = ADC_ExternalTrigConvEvent_0;         
  ADC_InitStruct.ADC_ExternalTrigEventEdge = ADC_ExternalTrigEventEdge_None;
  ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStruct.ADC_OverrunMode = ADC_OverrunMode_Disable;   
  ADC_InitStruct.ADC_AutoInjMode = ADC_AutoInjec_Disable;  
  ADC_InitStruct.ADC_NbrOfRegChannel = 1;
  ADC_Init(ADC1, &ADC_InitStruct);
	ADC_Init(ADC2, &ADC_InitStruct);
	
	ADC_InitStruct.ADC_NbrOfRegChannel = 2;
	ADC_Init(ADC3, &ADC_InitStruct);
	
	ADC_VoltageRegulatorCmd(ADC1, ENABLE);
	ADC_VoltageRegulatorCmd(ADC2, ENABLE);
	ADC_VoltageRegulatorCmd(ADC3, ENABLE);
  Delay_10us(1);
  ADC_SelectCalibrationMode(ADC1, ADC_CalibrationMode_Single);
	ADC_SelectCalibrationMode(ADC2, ADC_CalibrationMode_Single);
	ADC_SelectCalibrationMode(ADC3, ADC_CalibrationMode_Single);
	ADC_StartCalibration(ADC1);
  while(ADC_GetCalibrationStatus(ADC1) != RESET );
  calibration_value_ADC1 = ADC_GetCalibrationValue(ADC1);
	ADC_StartCalibration(ADC2);
  while(ADC_GetCalibrationStatus(ADC2) != RESET );
  calibration_value_ADC2 = ADC_GetCalibrationValue(ADC2);
	ADC_StartCalibration(ADC3);
  while(ADC_GetCalibrationStatus(ADC3) != RESET );
  calibration_value_ADC3 = ADC_GetCalibrationValue(ADC3);
	
	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 1, ADC_SampleTime_7Cycles5);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_1, 1, ADC_SampleTime_7Cycles5);
	ADC_RegularChannelConfig(ADC3, ADC_Channel_12, 1, ADC_SampleTime_7Cycles5);
	ADC_RegularChannelConfig(ADC3, ADC_Channel_1, 2, ADC_SampleTime_7Cycles5);
	
	ADC_DMAConfig(ADC1, ADC_DMAMode_Circular);
	ADC_DMAConfig(ADC2, ADC_DMAMode_Circular);
	ADC_DMAConfig(ADC3, ADC_DMAMode_Circular);
	
	ADC_DMACmd(ADC1, ENABLE);
	ADC_DMACmd(ADC2, ENABLE);
	ADC_DMACmd(ADC3, ENABLE);

  ADC_Cmd(ADC1, ENABLE);
	ADC_Cmd(ADC2, ENABLE);
	ADC_Cmd(ADC3, ENABLE);
	
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_RDY));
	while(!ADC_GetFlagStatus(ADC2, ADC_FLAG_RDY));
	while(!ADC_GetFlagStatus(ADC3, ADC_FLAG_RDY));
	
	DMA_Cmd(DMA1_Channel1, ENABLE);
	DMA_Cmd(DMA2_Channel1, ENABLE);
	DMA_Cmd(DMA2_Channel5, ENABLE);
	
  ADC_StartConversion(ADC1);
	ADC_StartConversion(ADC2);
	ADC_StartConversion(ADC3);
}
/*====================================================================================================*/
/*====================================================================================================*/
void EXTI_Config( void )
{
  GPIO_InitTypeDef GPIO_InitStruct;
	EXTI_InitTypeDef EXTI_InitStruct;
  NVIC_InitTypeDef NVIC_InitStruct;
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	
	// PA5 = ZC_IN
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource5);

  EXTI_InitStruct.EXTI_Line = EXTI_Line5;
  EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStruct.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStruct);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  NVIC_InitStruct.NVIC_IRQChannel = EXTI9_5_IRQn;
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStruct);
}
/*====================================================================================================*/
/*====================================================================================================*/
#define ZC_DELAY 310

void TIM_Config( void )
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
	NVIC_InitTypeDef NVIC_InitStruct;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4 | RCC_APB1Periph_TIM6, ENABLE);
	
	//Zero Cross Timing
	TIM_TimeBaseStruct.TIM_Period = (uint16_t)(ZC_DELAY-1);															//ZC_DELAY value
	TIM_TimeBaseStruct.TIM_Prescaler = (uint16_t)((SystemCoreClock / 1000000)-1);				//1MHZ
	TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStruct.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStruct);
	
	//Pulse Timing TIM3:L, TIM6:R
	TIM_TimeBaseStruct.TIM_Period = (uint16_t)(2000-1);																	//Not Important Value
	TIM_TimeBaseStruct.TIM_Prescaler = (uint16_t)((SystemCoreClock / 1000000)-1);				//1MHZ
	TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStruct.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStruct);
	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStruct);
	
	//Audio Sampling
	TIM_TimeBaseStruct.TIM_Period = (uint16_t)(400/*22*/-1);																		//About 45kHZ Sampling Freq
	TIM_TimeBaseStruct.TIM_Prescaler = (uint16_t)((SystemCoreClock / 1000000)-1);				//1MHZ
	TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStruct.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStruct);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitStruct.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStruct);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitStruct.NVIC_IRQChannel = TIM3_IRQn;
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStruct);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitStruct.NVIC_IRQChannel = TIM6_DAC_IRQn;
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStruct);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitStruct.NVIC_IRQChannel = TIM4_IRQn;
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 3;
  NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStruct);
	
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
	TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);
	
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
	
	TIM_Cmd(TIM4, ENABLE);
}
/*====================================================================================================*/
/*====================================================================================================*/
void ZCDelayConfig( void )
{
	TIM_SetCounter(TIM2, 0);
	TIM_Cmd(TIM2, ENABLE);
}
/*=====================================================================================================*/
/*=====================================================================================================*/
#define PHASE_DURATION 8333
#define MOD_DELAY_OFFSET 20
#define MOD_DELAY_MIN 50
#define MOD_DELAY_MAX (PHASE_DURATION - MOD_DELAY_OFFSET-600)

#define OUT_LEVEL_THRES_H OUT_LEVEL_MAX
#define OUT_LEVEL_THRES_L 0

void PhaseModConfig( void )
{
	static uint16_t ModDelayL = 0;
	static uint16_t ModDelayR = 0;
	
	TIM_Cmd(TIM2, DISABLE);
	
	//calculate Level & Timing
	
	//Clip Level to High or Low limiting level
	if(LevelOutL>OUT_LEVEL_THRES_H)	LevelOutL = OUT_LEVEL_THRES_H;
	else if(LevelOutL<OUT_LEVEL_THRES_L)	LevelOutL = OUT_LEVEL_THRES_L;
	if(LevelOutR>OUT_LEVEL_THRES_H)	LevelOutR = OUT_LEVEL_THRES_H;
	else if(LevelOutR<OUT_LEVEL_THRES_L)	LevelOutR = OUT_LEVEL_THRES_L;
	
	//Phase Area Lineraization
	ModDelayL = (float)PHASE_DURATION - (float)PHASE_DURATION*acos( 1.0 - 2.0*(double)LevelOutL / (double)OUT_LEVEL_MAX )/PI;
	ModDelayR = (float)PHASE_DURATION - (float)PHASE_DURATION*acos( 1.0 - 2.0*(double)LevelOutR / (double)OUT_LEVEL_MAX )/PI;
	
	//Time Offset Correction
	if(ModDelayL<=MOD_DELAY_OFFSET)	ModDelayL = 0;
	else	ModDelayL = ModDelayL - MOD_DELAY_OFFSET;
	if(ModDelayR<=MOD_DELAY_OFFSET)	ModDelayR = 0;
	else	ModDelayR = ModDelayR - MOD_DELAY_OFFSET;
	
	//Stability Limiting threshold
	if(ModDelayL<MOD_DELAY_MIN)	ModDelayL = MOD_DELAY_MIN;
	else if(ModDelayL>MOD_DELAY_MAX)	ModDelayL = MOD_DELAY_MAX;
	if(ModDelayR<MOD_DELAY_MIN)	ModDelayR = MOD_DELAY_MIN;
	else if(ModDelayR>MOD_DELAY_MAX)	ModDelayR = MOD_DELAY_MAX;
	
	TIM_SetAutoreload(TIM3, ModDelayL);
	TIM_SetAutoreload(TIM6, ModDelayR);
	TIM_SetCounter(TIM3, 0);
	TIM_SetCounter(TIM6, 0);
	TIM_Cmd(TIM3, ENABLE);
	TIM_Cmd(TIM6, ENABLE);
}
/*=====================================================================================================*/
/*=====================================================================================================*/
#define PULSE_START 0
#define PULSE_STOP  1
#define PULSE_WIDTH  10

void PulseConfigL( void )
{
	static unsigned char PulseState = PULSE_START;
	
	TIM_Cmd(TIM3, DISABLE);
	
	switch(PulseState)
	{
		case PULSE_START:
			
			PulseState = PULSE_STOP;
			GPIO_SetBits(GPIOA, GPIO_Pin_6);
		
			TIM_SetAutoreload(TIM3, PULSE_WIDTH);
			TIM_SetCounter(TIM3, 0);
			TIM_Cmd(TIM3, ENABLE);
			break;
		
		case PULSE_STOP:
			
			PulseState = PULSE_START;
			GPIO_ResetBits(GPIOA, GPIO_Pin_6);
			break;
		
	}
}

void PulseConfigR( void )
{
	static unsigned char PulseState = PULSE_START;
	
	TIM_Cmd(TIM6, DISABLE);
	
	switch(PulseState)
	{
		case PULSE_START:
			
			PulseState = PULSE_STOP;
			GPIO_SetBits(GPIOA, GPIO_Pin_7);
		
			TIM_SetAutoreload(TIM6, PULSE_WIDTH);
			TIM_SetCounter(TIM6, 0);
			TIM_Cmd(TIM6, ENABLE);
			break;
		
		case PULSE_STOP:
			
			PulseState = PULSE_START;
			GPIO_ResetBits(GPIOA, GPIO_Pin_7);
			break;
		
	}
}
/*=====================================================================================================*/
/*=====================================================================================================*/
uint16_t ADC_ReadData( ADC_TypeDef* ADCx )
{
  while(ADC_GetFlagStatus(ADCx, ADC_FLAG_EOC) == RESET);
  return ADC_GetConversionValue(ADCx);
}
/*=====================================================================================================*/
/*=====================================================================================================*/
#define ENV_LEVEL_GAIN 1.0

#define POT_LEVEL_BIAS_PAN 2047
#define POT_LEVEL_THRES_PANSW 2047
#define PAN_LEVEL_MAX 2048

void SamplingDSP( void )
{
	static uint16_t AudLevelL = AUDIO_LEVEL_BIAS;
	static uint16_t AudLevelR = AUDIO_LEVEL_BIAS;
	static float AudLevAbsL = 0;
	static float AudLevAbsR = 0;
	static float EnvLevelL = 0;
	static float EnvLevelR = 0;
	
	static uint16_t POTLevelA = 0;
	static uint16_t POTLevelB = 0;
	//static uint16_t POTLevelAttack = 0;
	//static uint16_t POTLevelRelease = 0;
	static uint16_t POTLevelPanSW = 0;
	static uint16_t POTLevelPan = 0;
	
	//static float AttackTimeMS = 0.0;
	//static float ReleaseTimeMS = 0.0;
	
	static float Pan = 0.0;
	static float PanAttL = 1.0;
	static float PanAttR = 1.0;
	
	//Get ADC Data for AudLevelL and AudLevelR
	AudLevelL = ADC_Data_AudioL;
	AudLevelR = ADC_Data_AudioR;
	//Get ADC Data for POTLevelA and POTLevelB
	POTLevelA = ADC_Data_POT[0];
	POTLevelB = ADC_Data_POT[1];
	
	//Process POT actions according to state of FuncMode
	switch(FuncMode)
	{
		case FUNC_MODE_CLOSED:
			
			break;
		
		case FUNC_MODE_A:
	/*		
			POTLevelAttack = POTLevelA;
			POTLevelRelease = POTLevelB;
			
			//Linear Mapping of POT value to AttackTimeMS and ReleaseTimeMS
			//AttackTimeMS = (ATTACK_TIME_MS_MAX - ATTACK_TIME_MS_MIN_THRES)*((float)POTLevelAttack/(float)POT_LEVEL_MAX) + ATTACK_TIME_MS_MIN_THRES;
			//ReleaseTimeMS = (RELEASE_TIME_MS_MAX - RELEASE_TIME_MS_MIN_THRES)*((float)POTLevelRelease/(float)POT_LEVEL_MAX) + RELEASE_TIME_MS_MIN_THRES;
			
			//Logarithmic-Mapped POT value to AttackTimeMS and ReleaseTimeMS
			AttackTimeMS = pow(10.0,(double)POTLevelAttack*(double)PowCoefAttack) * ATTACK_TIME_MS_MIN_THRES;
			ReleaseTimeMS = pow(10.0,(double)POTLevelRelease*(double)PowCoefRelease) * RELEASE_TIME_MS_MIN_THRES;
			
			//Calculate corresponding AttackCoef and ReleaseCoef from the tuned AttackTimeMS and ReleaseTimeMS times
			AttackCoef = pow(0.01, 1.0/((double)AttackTimeMS*(double)SAMPLING_FREQ*0.001));
			ReleaseCoef = pow(0.01, 1.0/((double)ReleaseTimeMS*(double)SAMPLING_FREQ*0.001));
	*/		
			break;
		
		case FUNC_MODE_B:
			
			POTLevelPanSW = POTLevelA;
			POTLevelPan = POTLevelB;
			
			if(POTLevelPanSW>POT_LEVEL_THRES_PANSW){
				//Determine Pan direction
				Pan = (float)POTLevelPan - (float)POT_LEVEL_BIAS_PAN;
				//Obtain L and R Velocity attenuation ratio
				if(Pan>0)	{
					PanAttL = ((float)PAN_LEVEL_MAX - fabs((double)Pan))/(float)PAN_LEVEL_MAX;
					PanAttR = 1.0;
				}else	{
					PanAttL = 1.0;
					PanAttR = ((float)PAN_LEVEL_MAX - fabs((double)Pan))/(float)PAN_LEVEL_MAX;
				}
			}else	{
				PanAttL = 1.0;
				PanAttR = 1.0;
			}
			
			break;
	}
	
	//Calculate Absolute value(Amplitude) of Audio Level from AUDIO_LEVEL_BIAS value
	AudLevAbsL = fabs((double)AudLevelL - (double)AUDIO_LEVEL_BIAS);
	AudLevAbsR = fabs((double)AudLevelR - (double)AUDIO_LEVEL_BIAS);
	
	//Obtain Envelope value from Audio Amplitude value with AttackCoef and ReleaseCoef
	if(AudLevAbsL>EnvLevelL)	{
		EnvLevelL = AttackCoef*(EnvLevelL - AudLevAbsL) + AudLevAbsL;
	}else	{
		EnvLevelL = ReleaseCoef*(EnvLevelL - AudLevAbsL) + AudLevAbsL;
	}
	
	if(AudLevAbsR>EnvLevelR)	{
		EnvLevelR = AttackCoef*(EnvLevelR - AudLevAbsR) + AudLevAbsR;
	}else	{
		EnvLevelR = ReleaseCoef*(EnvLevelR - AudLevAbsR) + AudLevAbsR;
	}
	
	//LED_B_Toggle;
	
	//Set Output Level with Logarithmic-Mapped Envelope Velocity with software Gain as well as corresponding Pan Attenuation for each channel
	LevelOutL = LogCoef * log10((double)EnvLevelL*(double)ENV_LEVEL_GAIN/(double)ENV_LOG_MIN_THRES) * PanAttL;
	LevelOutR = LogCoef * log10((double)EnvLevelR*(double)ENV_LEVEL_GAIN/(double)ENV_LOG_MIN_THRES) * PanAttR;
}
/*=====================================================================================================*/
/*=====================================================================================================*/
#define MANUAL_VELOCITY 1000.0

void ManualDSP( void )
{
	static uint16_t POTLevelVel = 0;
	static uint16_t POTLevelPan = 0;
	static float Velocity = MANUAL_VELOCITY;
	static float Pan = 0.0;
	static float PanAttL = 1.0;
	static float PanAttR = 1.0;
	
	//Get ADC Data for POTLevelVel and POTLevelPan
	POTLevelVel = ADC_Data_POT[0];
	POTLevelPan = ADC_Data_POT[1];
	
	//Process POT actions according to state of FuncMode
	switch(FuncMode)
	{
		case FUNC_MODE_CLOSED:
			
			break;
		
		case FUNC_MODE_A:
			
			//Calculate Logarithmic-Mapped Velocity from POTLevelVel
			//Velocity = pow(10.0,(double)POTLevelVel*(double)PowCoefVelocity)*(float)VEL_LOG_MIN_THRES;
			
			//Calculate Linear mapping of Velocity from POTLevelVel
			Velocity = (float)OUT_LEVEL_MAX*((float)POTLevelVel/(float)POT_LEVEL_MAX_VELOCITY);
			
			//Determine Pan direction
			Pan = (float)POTLevelPan - (float)POT_LEVEL_BIAS_PAN;
			
			//Obtain L and R Velocity attenuation ratio
			if(Pan>0)	{
				PanAttL = ((float)PAN_LEVEL_MAX - fabs((double)Pan))/(float)PAN_LEVEL_MAX;
				PanAttR = 1.0;
			}else	{
				PanAttL = 1.0;
				PanAttR = ((float)PAN_LEVEL_MAX - fabs((double)Pan))/(float)PAN_LEVEL_MAX;
			}
			
			break;
		
		case FUNC_MODE_B:
			
			break;
	}
	
	//Set Output Level with Velocity and corresponding Pan Attenuation for each channel
	LevelOutL = Velocity*PanAttL;
	LevelOutR = Velocity*PanAttR;
}
/*=====================================================================================================*/
/*=====================================================================================================*/
#define BREATH_VELOCITY 2500.0
#define BREATH_FREQ_HZ 1.0
#define BREATH_FREQ_HZ_MAX 8.0
#define BREATH_FREQ_HZ_MIN 0.02

#define WAVE_LEVEL_MAX 1.0
#define WAVE_LEVEL_OFFSET_MAX (WAVE_LEVEL_MAX/2.0)

#define COUNT_UP 1
#define COUNT_DOWN 0

#define POT_LEVEL_BIAS_PHASE 2047
#define PHASE_LEVEL_MAX 2048

void FuncGenDSP( void )
{
	static uint16_t POTLevelA = 0;
	static uint16_t POTLevelB = 0;
	
	static uint16_t POTLevelVel = 0;
	static uint16_t POTLevelFreq = 0;
	static uint16_t POTLevelPhase = 0;
	static uint16_t POTLevelPan = 0;
	
	static float Velocity = BREATH_VELOCITY;
	static float Freq = BREATH_FREQ_HZ;
	static float Increment = 0.0;
	static float Remainder = 0.0;
	static float Phase = 0.0;
	
	static float WaveLevelMain = 0.0;
	static float WaveLevelL = 0.0;
	static float WaveLevelR = 0.0;
	
	//Offset refers to the Vertical Offset
	static float WaveLevelOffset = 0.0;
	
	static uint8_t CountDir = COUNT_UP;
	
	static float Pan = 0.0;
	static float PanAttL = 1.0;
	static float PanAttR = 1.0;
	
	//Get ADC Data for POTLevelA and POTLevelB
	POTLevelA = ADC_Data_POT[0];
	POTLevelB = ADC_Data_POT[1];
	
	switch(FuncMode)
	{                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             
		case FUNC_MODE_CLOSED:
			
			break;
		
		case FUNC_MODE_A:
			
			POTLevelVel = POTLevelA;
			POTLevelFreq = POTLevelB;
			
			Velocity = (float)OUT_LEVEL_MAX*((float)POTLevelVel/(float)POT_LEVEL_MAX_VELOCITY);
			Freq = (BREATH_FREQ_HZ_MAX - BREATH_FREQ_HZ_MIN)*((float)POTLevelFreq/(float)POT_LEVEL_MAX_FREQ) + BREATH_FREQ_HZ_MIN;
			
			break;
		
		case FUNC_MODE_B:
			
			POTLevelPhase = POTLevelA;
			POTLevelPan = POTLevelB;
			
			//Calculate phase drift direction and amount, Ranging from -1 to 1 corresponding to -180deg(Lagging) to +180deg(Leading) 
			Phase = (float)POTLevelPhase - (float)POT_LEVEL_BIAS_PHASE;
			
			//Calculate the level offset amount and direction needed to represent phase drift amount
			//In TRIANGLE wave form, time and level are linear proportional
			//Thus by setting level offset, it is in other words setting the demanded phase drift amount
			//Setting phase difference by varying both L and R phase drift oppositly
			//For 180, only need 90 for each wave level, vice versa for -180
			WaveLevelOffset = (float)WAVE_LEVEL_OFFSET_MAX*(Phase/(float)PHASE_LEVEL_MAX);
			
			//Determine Pan direction
			Pan = (float)POTLevelPan - (float)POT_LEVEL_BIAS_PAN;
			//Obtain L and R Velocity attenuation ratio
			if(Pan>0)	{
				PanAttL = ((float)PAN_LEVEL_MAX - fabs((double)Pan))/(float)PAN_LEVEL_MAX;
				PanAttR = 1.0;
			}else	{
				PanAttL = 1.0;
				PanAttR = ((float)PAN_LEVEL_MAX - fabs((double)Pan))/(float)PAN_LEVEL_MAX;
			}
			
			break;
	}
	
	//Calculate Sampling Increment value corresponding to Freq and WAVE_LEVEL_MAX under the SAMPLING_FREQ
	Increment = (float)(2.0*WAVE_LEVEL_MAX*(double)Freq/SAMPLING_FREQ);
	Remainder = fmod((double)Increment,2.0*WAVE_LEVEL_MAX);
	
	//
	switch(CountDir)
	{
		case COUNT_UP:
			
			if(Remainder<(float)WAVE_LEVEL_MAX)	{
				WaveLevelMain = WaveLevelMain + Remainder;
				if(WaveLevelMain>(float)WAVE_LEVEL_MAX)	{
					WaveLevelMain = (float)WAVE_LEVEL_MAX - (WaveLevelMain - (float)WAVE_LEVEL_MAX);
					CountDir = COUNT_DOWN;
				}
			}else	{
				WaveLevelMain = WaveLevelMain - ((float)(2.0*WAVE_LEVEL_MAX) - Remainder);
				if(WaveLevelMain<0)	WaveLevelMain = 0 - (WaveLevelMain - 0);
			}
			
			WaveLevelL = WaveLevelMain - WaveLevelOffset;
			WaveLevelR = WaveLevelMain + WaveLevelOffset;
			
			if(WaveLevelL>(float)WAVE_LEVEL_MAX)	WaveLevelL = (float)WAVE_LEVEL_MAX - (WaveLevelL - (float)WAVE_LEVEL_MAX);
			else if(WaveLevelL<0)	WaveLevelL = 0 - (WaveLevelL - 0);
			if(WaveLevelR>(float)WAVE_LEVEL_MAX)	WaveLevelR = (float)WAVE_LEVEL_MAX - (WaveLevelR - (float)WAVE_LEVEL_MAX);
			else if(WaveLevelR<0)	WaveLevelR = 0 - (WaveLevelR - 0);
			
			break;
		
		case COUNT_DOWN:
			
			if(Remainder<(float)WAVE_LEVEL_MAX)	{
				WaveLevelMain = WaveLevelMain - Remainder;
				if(WaveLevelMain<0)	{
					WaveLevelMain = 0 - (WaveLevelMain - 0);
					CountDir = COUNT_UP;
				}
			}else	{
				WaveLevelMain = WaveLevelMain + ((float)(2.0*WAVE_LEVEL_MAX) - Remainder);
				if(WaveLevelMain>(float)WAVE_LEVEL_MAX)	WaveLevelMain = (float)WAVE_LEVEL_MAX - (WaveLevelMain - (float)WAVE_LEVEL_MAX);
			}
			
			WaveLevelL = WaveLevelMain + WaveLevelOffset;
			WaveLevelR = WaveLevelMain - WaveLevelOffset;
			
			if(WaveLevelL>(float)WAVE_LEVEL_MAX)	WaveLevelL = (float)WAVE_LEVEL_MAX - (WaveLevelL - (float)WAVE_LEVEL_MAX);
			else if(WaveLevelL<0)	WaveLevelL = 0 - (WaveLevelL - 0);
			if(WaveLevelR>(float)WAVE_LEVEL_MAX)	WaveLevelR = (float)WAVE_LEVEL_MAX - (WaveLevelR - (float)WAVE_LEVEL_MAX);
			else if(WaveLevelR<0)	WaveLevelR = 0 - (WaveLevelR - 0);
			
			break;
	}
	
	//LED_B_Toggle;
	
	LevelOutL = WaveLevelL*Velocity*PanAttL;
	LevelOutR = WaveLevelR*Velocity*PanAttR;
}
