// SysTickPWM.c
// Runs on TM4C123
// By Kevin Martinez
// Feb 2023
// Use SysTick interrupts to implement a timer-based PWM: this PWM signal can be used 
// to control the brightness of an LED

#include <stdint.h>
#include "tm4c123gh6pm.h"

#define PERIOD_W 					400000 					// num of machine cycles for 10ms, value is based on 40MHz system clock
#define PERIOD_R 					20000000				// 2hz red period
#define HALF_DUTY   			PERIOD_W/2 			// 50% duty cycle
#define DUTY_STEP					PERIOD_W/10 		// duty cycle change for each button press
#define RED								0x02
#define WHITE 						0x0E

// Global variables: 
// H: number of clocks cycles for duty cycle
// L: number of clock cycles for non-duty cycle
volatile unsigned long H,L,global_dist;

void SysTick_Init(void){
	H = L = HALF_DUTY;            			// 50% duty cycle, @ system clock 40MHz, 
  NVIC_ST_CTRL_R = 0;           			// disable SysTick during setup
  NVIC_ST_RELOAD_R = HALF_DUTY-1;     // reload value for 50% duty cycle
  NVIC_ST_CURRENT_R = 0;        			// any write to current clears it
  NVIC_SYS_PRI3_R = (NVIC_SYS_PRI3_R&0x1FFFFFFF)|0x40000000; // bit 31-29 for SysTick, set priority to 2
  NVIC_ST_CTRL_R |= NVIC_ST_CTRL_CLK_SRC + NVIC_ST_CTRL_INTEN + NVIC_ST_CTRL_ENABLE;  // enable with core clock and interrupts, start systick timer
}

// SysTick ISR:
// 1. Implement timing control for duty cycle and non-duty cycle
// 2. Output a waveform based on current duty cycle
void SysTick_Handler(void){
	NVIC_ST_CTRL_R &= ~NVIC_ST_CTRL_ENABLE; 	// turn off SysTick 
	// toggle RGB LED
	// Red LED range
	if(global_dist < 10){
		GPIO_PORTF_DATA_R ^= RED;
		
		if(GPIO_PORTF_DATA_R & RED){ // previous cycle is duty cycle
			H = L = PERIOD_R/2;
			NVIC_ST_RELOAD_R = L-1;     // switch to non-duty cycle
		}
		else{
			GPIO_PORTF_DATA_R &= ~0x0E;	// turn off
			NVIC_ST_RELOAD_R = H-1;     // switch to duty cycle
		}
	}
	// white LED range
	else if((10 <= global_dist) && (global_dist <= 70)){
		GPIO_PORTF_DATA_R ^= WHITE;
		
		if((GPIO_PORTF_DATA_R & WHITE) && (GPIO_PORTF_DATA_R & ~RED)){   // previous cycle is duty cycle
			NVIC_ST_RELOAD_R = L-1;     // switch to non-duty cycle
		}
		else{
			GPIO_PORTF_DATA_R &= ~0x0E;
			NVIC_ST_RELOAD_R = H-1;     // switch to duty cycle
		}
	}
	// Out of range (off)
	else{
		GPIO_PORTF_DATA_R &= ~0x0E;
		NVIC_ST_RELOAD_R = H-1;
	}
	
	NVIC_ST_CURRENT_R = 0;
	NVIC_ST_CTRL_R |= NVIC_ST_CTRL_ENABLE; // turn on systick to continue
}

void SysPWM_RGB_LED(uint32_t distance){
	global_dist = distance;
	// dynamically change white with distance eq
	if((10 <= distance) && (distance <= 70)){
		double dist_perc = (double)(distance - 10) / (double)(70 - 10);
		if (dist_perc < .1) dist_perc = .1;
		else if (dist_perc > .9) dist_perc = .9;
		H = PERIOD_W * dist_perc;
		L = PERIOD_W-H; // constant period of 10ms, variable duty cycle
	}
}
