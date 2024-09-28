#include "Keyboard.h"

void delay_ms(u16 time)		
{    
   u16 i=0;  
   while(time--)
   {
      i=12000;  //自己定义
      while(i--) ;    
   }
}
void KeyBoard_Init()
{
	GPIO_InitTypeDef GPIOStruct;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO, ENABLE);
	/*GPIO配置*/
	GPIOStruct.GPIO_Mode = GPIO_Mode_IPD;
	GPIOStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIOStruct.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_1 | GPIO_Pin_6 | GPIO_Pin_0;		//PA146
	GPIO_Init(GPIOA, &GPIOStruct);
}

uint16_t KeyBoard_Read()
{
	uint16_t Key = 0;
	if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6))
	{
		delay_ms(20);
		while(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6));
		Key = 1;
	}
	if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_4))
	{
		delay_ms(20);
		while(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_4));
		Key = 2;
	}
	if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1))
	{
		delay_ms(20);
		while(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1));
		Key = 3;
	}
	if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0))
	{
		delay_ms(20);
		while(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0));
		Key = 4;
	}
	return Key;
}
