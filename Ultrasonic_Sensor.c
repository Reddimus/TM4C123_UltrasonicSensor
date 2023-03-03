// Ultrasonic_Sensor.c
// This program runs on TM4C123.
// This is a program to show how to interface HC-SR04 Ultrasonic sensor and demonstrate useful Embeded System features.
// PB4 connects to echo pin to generate edge-triggered interrupt.
// PB5 connects to Ultrasonic sensor trigger pin.
// General Purpose timer 1A is used to generate the required timing for trigger pin and measure echo pulse width.
// Global variable "distance" is used to hold the distance in cemtimeter for the obstacles
// in front of the sensor. 
// By Kevin Martinez
// March 2023

/*
This C program interfaces with an HC-SR04 ultrasonic sensor. The program runs on the 
TM4C123 microcontroller and uses the SysTick timer to generate the required timing for the trigger 
pin and measure the echo pulse width. The distance to obstacles in front of the sensor is held in 
a global variable and calculated based on the width of the echo pulse and the speed of sound. The 
program initializes the PB4 and PB5 pins, sets up the interrupt on PB4, and runs an infinite loop 
that measures the distance and updates the global variable.
*/

#include <stdint.h>
#include "PLL.h"
#include "Timer1A.h"
#include "SysTickPWM.h"
#include "tm4c123gh6pm.h"

#define TRIGGER_PIN 		(*((volatile unsigned long *)0x40005080))  // PB5 is the trigger pin
#define TRIGGER_VALUE 	0x20   // trigger at port 5
#define ECHO_PIN 				(*((volatile unsigned long *)0x40005040))  // PB4 is the echo pin	
#define ECHO_VALUE 			0x10   // echo at port 4
#define MC_LEN 					.025 // length of one machine cycle in microsecond for 40Mhz clock 	1 / 40 Mhz = .025
#define SOUND_SPEED 		0.0343 // centimeter per micro-second

extern void DisableInterrupts(void);
extern void EnableInterrupts(void);
extern void WaitForInterrupt(void);  // low power mode
void PortB_Init(void);
void PortF_Init(void);

static volatile uint8_t done = 0;
static volatile uint32_t distance = 0;

int main(void){
	DisableInterrupts();
	PLL_Init(); 	// bus clock changed 16 Mhz -> 40 MHz
  PortB_Init();
	PortF_Init();
	Timer1A_Init();
	SysTick_Init();
  EnableInterrupts();
  while(1){
		done = 0;
		distance = 0;
		TRIGGER_PIN &= ~TRIGGER_VALUE; // send low to trigger
		Timer1A_Wait1us(2);		// 2us
		TRIGGER_PIN |= TRIGGER_VALUE; // send high to trigger
		Timer1A_Wait1us(10);	// 10 us
		TRIGGER_PIN &= ~TRIGGER_VALUE; // send low to trigger
		
		while(!done);
  }
}

// intialize onboard RGB LED (PF3,2,1) outputs
void PortF_Init(void){
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOF;     	// activate F clock
	while ((SYSCTL_RCGC2_R & SYSCTL_RCGC2_GPIOF)!= SYSCTL_RCGC2_GPIOF){} // wait for the clock to be ready
  GPIO_PORTF_LOCK_R = GPIO_LOCK_KEY; 	// unlock GPIO Port F
  GPIO_PORTF_CR_R |= 0x0E;         		// allow changes to PF3,2,1
  GPIO_PORTF_AMSEL_R &= ~0x0E;  			// disable analog functionality on PF3,2,1
  GPIO_PORTF_PCTL_R &= ~0x0000FFF0; 	// configure PF3,2,1 as GPIO
  GPIO_PORTF_DIR_R |= 0x0E;     			// make PF3,2,1 out
  GPIO_PORTF_DATA_R &= ~0x0E;					// Turn off color to start duty cycle.
  GPIO_PORTF_AFSEL_R &= ~0x0E;  			// disable alt funct on PF3,2,1
  GPIO_PORTF_DEN_R |= 0x0E;     			// enable digital I/O on PF3,2,1
}

void PortB_Init(void){ 
  SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOB;           // 1) activate clock for Port B
	while ((SYSCTL_RCGC2_R&SYSCTL_RCGC2_GPIOB)!=SYSCTL_RCGC2_GPIOB){}; // wait for clock to start
                                    // 2) no need to unlock PA2
	GPIO_PORTB_PCTL_R &= ~0x00FF0000;
  GPIO_PORTB_AMSEL_R &= (uint32_t)~0x30;      // 4) disable analog function on PB4&5
  GPIO_PORTB_DIR_R &= ~0x10;        // 5) PB4:echo pin, input
  GPIO_PORTB_DIR_R |= 0x20;         // 5) PB5:trigger pin, output
  GPIO_PORTB_AFSEL_R &= ~0x30;      // 6) regular port function
  GPIO_PORTB_DEN_R |= 0x30;         // 7) enable digital port
		
  GPIO_PORTB_IS_R &= ~0x10;         // PB4 is edge-sensitive
  GPIO_PORTB_IBE_R |= 0x10;         // PB4 is both edges
  GPIO_PORTB_IEV_R &= ~0x10;        // PB4 both edge event
  GPIO_PORTB_ICR_R = 0x10;          // clear flag 6
  GPIO_PORTB_IM_R |= 0x10;          // arm interrupt on PB4
  NVIC_PRI0_R = (NVIC_PRI0_R&0xFFFF1FFF)|0x0004000; // (g) GPIO Port B priority 2
  NVIC_EN0_R = 0x00000002;          // (h) enable Port B edge interrupt
}

void GPIOPortB_Handler(void){
	if (ECHO_PIN==ECHO_VALUE){  // echo pin rising edge is detected, start timing
		Timer1A_start();
	}
	else { // echo pin falling edge is detected, end timing and calculate distance.
    // The following code is based on the fact that the HCSR04 ultrasonic sensor 
    // echo pin will always go low after a trigger with bouncing back
    // or after a timeout. The maximum distance can be detected is 400cm.
		// The speed of sound is approximately 340 meters per second, 
		// or  .0343 c/µS.
		Timer1A_stop();
		distance = (uint32_t)(Timer1A_Get_MC_Elapsed()*MC_LEN*SOUND_SPEED)/2;
		SysPWM_RGB_LED(distance);	// dynamically change white brightness and or send distance to SysTickPWM module
		done = 1;
	}
	GPIO_PORTB_ICR_R = 0x10;      // acknowledge flag 6
}
