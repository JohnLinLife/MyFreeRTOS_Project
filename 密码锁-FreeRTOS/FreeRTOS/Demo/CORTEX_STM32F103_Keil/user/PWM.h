#ifndef __PWM_H
#define __PWM_H

void PWM_Init(void);
void PWM_setCompare3(uint16_t Comparel);
uint16_t PWM_To_Angle(uint16_t PWM);		//��PWMתΪ�Ƕ�
uint16_t Angle_To_PWM(uint16_t Angle);		//��PWMתΪ�Ƕ�

#endif
