// Timer1.c
// Runs on LM4F120/TM4C123
// Use TIMER1 in 32-bit periodic mode to request interrupts at a periodic rate
// Kevin Martinez
// Feb 2023

#include <stdint.h>
#include "tm4c123gh6pm.h"

void (*PeriodicTask1)(void);   // user function

void Timer1A_Init(void){
  SYSCTL_RCGCTIMER_R |= 0x02;   // 0) activate TIMER1
  TIMER1_CTL_R = 0x00000000;    // 1) disable TIMER1A during setup
  TIMER1_CFG_R = 0x00000000;    // 2) configure for 32-bit mode
  TIMER1_TAMR_R = 0x00000002;   // 3) configure for periodic down-count mode
  TIMER1_TAPR_R = 0;            // 5) bus clock prescale
}

void Timer1A_start(void){
	TIMER1_TAILR_R = TIMER_TAILR_M - 1; //reload value; using timer len (0xFFFFFFFF)
	TIMER1_CTL_R = 0x00000001;    // 10) enable TIMER1A
}

void Timer1A_stop(void){
	TIMER1_CTL_R &= ~0x00000001; // 1) disable TIMER1A
}

void Timer1A_Wait1us(unsigned long delay){
	unsigned long period = delay * 40; // period / 40Mhz = delay us -> period = delay us * 40 Mhz
	TIMER1_TAILR_R = period-1; //reload value
	TIMER1_CTL_R = 0x00000001;  //enable timer
	while(TIMER1_TAR_R > 0){}; // while current value is greater than 0 timer is counting down(32 bit count down)
	Timer1A_stop();
}

// Calculate number of machine cycles elapsed
	uint32_t Timer1A_Get_MC_Elapsed(void){
	return TIMER1_TAILR_R - TIMER1_TAR_R; //load value - current value
}