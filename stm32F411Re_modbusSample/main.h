#ifndef _MAIN_H
#define _MAIN_H

#include "stm32f4xx.h"

#include "usart.h"

#define NULL ((void *)0)

void RCC_init(void);
void DMA_init(void);

//BOOLEAN TYPE DEFINE
typedef enum Bool{
	FALSE = 0,
	TRUE = 1
}Bool;

#endif //_MAIN_H
