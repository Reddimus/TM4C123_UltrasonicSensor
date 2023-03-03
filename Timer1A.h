// Timer1.h
// Runs on LM4F120/TM4C123
// Use Timer1 in 32-bit periodic mode to request interrupts
// Kevin Martinez
// Feb 2023

#include <stdint.h>

#ifndef __TIMER1INTS_H__ // do not include more than once
#define __TIMER1INTS_H__

void Timer1A_Init(void);
void Timer1A_start(void);
void Timer1A_stop (void);
void Timer1A_Wait1us(unsigned long delay);
uint32_t Timer1A_Get_MC_Elapsed(void);

#endif // __TIMER2INTS_H__
