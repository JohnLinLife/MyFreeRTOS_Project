#ifndef __KEYBOARD_H
#define __KEYBOARD_H

#include "stm32f10x_it.h"
#include "stdint.h"

void KeyBoard_Init(void);
uint16_t KeyBoard_Read(void);

#endif
