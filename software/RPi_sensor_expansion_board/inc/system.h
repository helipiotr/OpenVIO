/*
 * system.h
 * This file describes system-related configuration for the reuse across the project
 */

#ifndef SYSTEM_H_
#define SYSTEM_H_
#include "stm32f4xx.h"

extern uint32_t PLL_clock_freq;          /*!< System Clock Frequency (Core Clock) */

void RCC_Conf();

#endif /* SYSTEM_H_ */
