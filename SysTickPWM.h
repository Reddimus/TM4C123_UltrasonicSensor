// SysTickPWM.h
// Runs on LM4F120/TM4C123
// Kevin Martinez
// Feb 2023

#include <stdint.h>

void SysTick_Init(void);
void SysTick_Handler(void);
void SysPWM_RGB_LED(uint32_t distance);
