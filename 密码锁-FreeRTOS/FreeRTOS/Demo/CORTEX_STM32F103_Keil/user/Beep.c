#include "stm32f10x_it.h"
#include "Beep.h"

#define Beep_GPIO_Pin	GPIO_Pin_3
#define Beep_GPIO_Port	GPIOA

void Beep_delay(u16 time)		
{    
   u16 i=0;  
   while(time--)
   {
      i=1200;  //自己定义
      while(i--) ;    
   }
}

void Beep_Init()
{
	GPIO_InitTypeDef GPIOInitStruct;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIOInitStruct.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIOInitStruct.GPIO_Pin = Beep_GPIO_Pin;
	GPIOInitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(Beep_GPIO_Port, &GPIOInitStruct);
	GPIO_WriteBit(Beep_GPIO_Port, Beep_GPIO_Pin, Bit_SET);
	
}
void Beep_Ring()
{
	GPIO_WriteBit(Beep_GPIO_Port, Beep_GPIO_Pin, Bit_RESET);
}

void Beep_Close()
{
	GPIO_WriteBit(Beep_GPIO_Port, Beep_GPIO_Pin, Bit_SET);
}
