#include "key.h"
#include "stm32f10x_it.h"
#include "stm32f10x_exti.h"

void Key_Init(void)
{
	GPIO_InitTypeDef Key_Struct;
	EXTI_InitTypeDef Key_Exti_Struct;
	NVIC_InitTypeDef NVIC_Key_Struct;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	
	Key_Struct.GPIO_Mode = GPIO_Mode_IPU;
	Key_Struct.GPIO_Pin = GPIO_Pin_0;
	Key_Struct.GPIO_Speed = GPIO_Speed_50MHz;
	
	Key_Exti_Struct.EXTI_Line = EXTI_Line0;
	Key_Exti_Struct.EXTI_LineCmd = ENABLE;
	Key_Exti_Struct.EXTI_Mode = EXTI_Mode_Interrupt;
	Key_Exti_Struct.EXTI_Trigger = EXTI_Trigger_Rising;
	
	NVIC_Key_Struct.NVIC_IRQChannel = EXTI0_IRQChannel;
	NVIC_Key_Struct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Key_Struct.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_Key_Struct.NVIC_IRQChannelSubPriority = 1;
	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);
	
	GPIO_Init(GPIOA, &Key_Struct);
	NVIC_Init(&NVIC_Key_Struct);
	EXTI_Init(&Key_Exti_Struct);
}

void Led_Init(void)
{
	GPIO_InitTypeDef Led_Struct;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	Led_Struct.GPIO_Mode = GPIO_Mode_Out_PP;
	Led_Struct.GPIO_Pin = GPIO_Pin_8;
	Led_Struct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &Led_Struct);
	GPIO_WriteBit(GPIOA, GPIO_Pin_8, Bit_SET);
}

