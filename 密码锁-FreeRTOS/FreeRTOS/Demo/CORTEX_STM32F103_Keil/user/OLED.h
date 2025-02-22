#ifndef __OLED_H
#define __OLED_H 

#include "FREERTOS.h"                  // Device header
#include "stdlib.h"	
#include "stm32f10x_it.h"


//-----------------测试LED端口定义---------------- 

#define LED_ON GPIO_WriteBit(GPIOC,GPIO_Pin_12,Bit_RESET)
#define LED_OFF GPIO_WriteBit(GPIOC,GPIO_Pin_12,Bit_SET)

//-----------------OLED端口定义---------------- 

#define OLED_SCL_Clr() GPIO_WriteBit(GPIOB,GPIO_Pin_4,Bit_RESET)//SCL
#define OLED_SCL_Set() GPIO_WriteBit(GPIOB,GPIO_Pin_4,Bit_SET)

#define OLED_SDA_Clr() GPIO_WriteBit(GPIOB,GPIO_Pin_5,Bit_RESET)//DIN
#define OLED_SDA_Set() GPIO_WriteBit(GPIOB,GPIO_Pin_5,Bit_SET)

#define OLED_RES_Clr() GPIO_WriteBit(GPIOD,GPIO_Pin_2,Bit_RESET)//RES
#define OLED_RES_Set() GPIO_WriteBit(GPIOD,GPIO_Pin_2,Bit_SET)


#define OLED_CMD  0	//写命令
#define OLED_DATA 1	//写数据

void OLED_ClearPoint(u8 x,u8 y);
void OLED_ColorTurn(u8 i);
void OLED_DisplayTurn(u8 i);
void I2C_Start(void);
void I2C_Stop(void);
void I2C_WaitAck(void);
void Send_Byte(u8 dat);
void OLED_WR_Byte(u8 dat,u8 mode);
void OLED_DisPlay_On(void);
void OLED_DisPlay_Off(void);
void OLED_Refresh(void);
void OLED_Clear(void);
void OLED_DrawPoint(u8 x,u8 y,u8 t);
void OLED_DrawLine(u8 x1,u8 y1,u8 x2,u8 y2,u8 mode);
void OLED_DrawCircle(u8 x,u8 y,u8 r);
void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 size1,u8 mode);
void OLED_ShowChar6x8(u8 x,u8 y,u8 chr,u8 mode);
void OLED_ShowString(u8 x,u8 y,char *chr,u8 size1,u8 mode);
void OLED_ShowNum(u8 x,u8 y,u32 num,u8 len,u8 size1,u8 mode);
void OLED_ShowChinese(u8 x,u8 y,u8 num,u8 size1,u8 mode);
void OLED_ScrollDisplay(u8 num,u8 space,u8 mode);
void OLED_ShowPicture(u8 x,u8 y,u8 sizex,u8 sizey,u8 BMP[],u8 mode);
void OLED_Init(void);
void main_delay_ms(u16 time);	
#endif

