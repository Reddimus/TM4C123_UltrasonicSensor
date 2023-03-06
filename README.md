
# An Ultrasonic Ranger Finder


Embedded Systems Project - GPIO, SysTick timer, Edge Interrupt, PLL, GPTM, Ultrasonic Sensor, and Software PWM

Description:

This project aims to review and apply concepts related to embedded systems by interfacing with various hardware components such as the HC-SR04 ultrasonic sensor and LEDs. 
The project utilizes GPIO, SysTick timer, and Edge Interrupt to control the hardware components. 
Additionally, it introduces new concepts such as PLL, GPTM, and Software PWM to generate a system clock, generate periodic timing, and control the brightness of LEDs.

Technologies Used:

Embedded C Keil v5, 
TM4C LaunchPad,
HC-SR04 Ultrasonic Sensor,
2K Resistor,
1K Resistor,
Jumper wires,
Breadboard



Connection

Connect Power rail to VBUS on the TM4C. The ultrasonic sensor requires 5 volts.
Create a voltage divider between Trigger Pin and Echo pin using a 1k and 2k resistor. Connect Trigger pin to PB7 and echo pin to PB6 on the TM4C. Connect Ground Pin to ground pin on the TM4C.
 
 


Installation


Connect the HC-SR04 Ultrasonic Sensor to the TM4C Launch Pad. Connect the TM4C LaunchPad to a computer using a USB cable. Compile and flash the code onto the TM4C. 
Open the debugger and click run. It will then read the centimeters between the Ultra Sonic Sensor and any object in front of it


Credits

This project was created by Kevin Martinez and Jesus Perez.


Video Link

https://youtu.be/2qrtmd1UbZQ


Contact

For questions or comments about this project, please contact: 
Kevin.Martinez03@student.csulb.edu or Jesus.Perez04@student.csulb.edu

