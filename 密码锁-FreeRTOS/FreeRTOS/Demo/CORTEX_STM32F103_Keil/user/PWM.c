#include "stm32f10x_it.h"                  // Device header
#include "FreeRTOS.h"
#include "stm32f10x_tim.h"

void PWM_Init(void)
{
	TIM_TimeBaseInitTypeDef TimeBaseStruct;
	TIM_OCInitTypeDef OCInitStruct;
	GPIO_InitTypeDef GPIO_InitStruct;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	
	TIM_InternalClockConfig(TIM2);
	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TimeBaseStruct.TIM_Period = 20000-1;			//ARR
	TimeBaseStruct.TIM_Prescaler = 72-1;		//PSC
	TIM_TimeBaseInit(TIM2,&TimeBaseStruct);
	
	TIM_OCStructInit(&OCInitStruct);
	OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
	OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
	OCInitStruct.TIM_Channel = TIM_Channel_3;
	OCInitStruct.TIM_Pulse = 0;

	TIM_OCInit(TIM2, &OCInitStruct);

	TIM_Cmd(TIM2,ENABLE);
}



void PWM_setCompare3(uint16_t Comparel)		//500-2500
{
	TIM_SetCompare3(TIM2, Comparel);
}

uint16_t PWM_To_Angle(uint16_t PWM)		//将PWM转为角度
{
	uint16_t angle;
	angle = (int)((float)(PWM - 500) * (0.09));
	return angle;
}

uint16_t Angle_To_PWM(uint16_t Angle)		//将PWM转为角度
{
	uint16_t PWM;
	PWM = (int)((100/9) * (float)Angle + 500);
	return PWM;
}

