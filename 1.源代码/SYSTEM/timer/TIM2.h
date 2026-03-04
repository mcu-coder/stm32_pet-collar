#ifndef __TIM2_H
#define __TIM2_H

#include "sys.h"

void TIM2_Init(u16 Prescaler, u16 Period);
uint32_t TIM2_GetInterruptCount(void);

#endif

