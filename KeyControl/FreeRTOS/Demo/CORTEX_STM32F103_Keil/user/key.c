#include "stm32f10x_lib.h"

void Key_Init()
{
	GPIO_InitTypeDef GPIOStruct;
	EXTI_InitTypeDef EXTIInitStruct;
	NVIC_InitTypeDef NVICInitStruct;
	
	/*ʱ�ӳ�ʼ��*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO, ENABLE);
	/*GPIO����*/
	GPIOStruct.GPIO_Mode = GPIO_Mode_IPD;
	GPIOStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIOStruct.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_1 | GPIO_Pin_6 | GPIO_Pin_0;		//PA146
	GPIO_Init(GPIOA, &GPIOStruct);
	
	/*PA1�ж����ȼ�����*/
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);		/* ����NVICΪ���ȼ���1 */
	NVICInitStruct.NVIC_IRQChannel = EXTI1_IRQChannel;	//EXTI1_IRQn;
	NVICInitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVICInitStruct.NVIC_IRQChannelPreemptionPriority = 1;
	NVICInitStruct.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVICInitStruct);
	
	/*PA1�ж�ģʽ����*/
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource1);	/* ѡ��EXTI���ź�Դ */
	EXTIInitStruct.EXTI_Line = EXTI_Line1;
	EXTIInitStruct.EXTI_LineCmd = ENABLE;
	EXTIInitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTIInitStruct.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_Init(&EXTIInitStruct);
	
	/*PA4�ж����ȼ�����*/
	NVICInitStruct.NVIC_IRQChannel = EXTI4_IRQChannel;	//EXTI4_IRQn;
	NVIC_Init(&NVICInitStruct);
	/*PA4�ж�ģʽ����*/
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource4);
	EXTIInitStruct.EXTI_Line = EXTI_Line4;
	EXTI_Init(&EXTIInitStruct);
	
	/*PA6�ж����ȼ�����*/
	NVICInitStruct.NVIC_IRQChannel = EXTI9_5_IRQChannel;	//EXTI9_5_IRQn;
	NVIC_Init(&NVICInitStruct);
	/*PA6�ж�ģʽ����*/
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource6);
	EXTIInitStruct.EXTI_Line = EXTI_Line6;
	EXTI_Init(&EXTIInitStruct);
	
	/*PA0�ж����ȼ�����*/
	NVICInitStruct.NVIC_IRQChannel = EXTI0_IRQChannel;		//EXTI0_IRQn;
	NVIC_Init(&NVICInitStruct);
	/*PA0�ж�ģʽ����*/
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);
	EXTIInitStruct.EXTI_Line = EXTI_Line0;
	EXTI_Init(&EXTIInitStruct);
}

