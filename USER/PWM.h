#ifndef __PWM_H
#define	__PWM_H
 
#include "stm32f10x.h"
#include "stm32f10x_tim.h"
 
 
void TIM4_PWM_Init(u16 arr,u16 psc);			//初始化 定时器4_PWM
 
#endif