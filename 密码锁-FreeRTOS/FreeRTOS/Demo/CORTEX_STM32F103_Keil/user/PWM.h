#ifndef __PWM_H
#define __PWM_H

void PWM_Init(void);
void PWM_setCompare3(uint16_t Comparel);
uint16_t PWM_To_Angle(uint16_t PWM);		//将PWM转为角度
uint16_t Angle_To_PWM(uint16_t Angle);		//将PWM转为角度

#endif
