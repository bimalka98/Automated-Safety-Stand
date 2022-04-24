/*
 * AutomatedSS.c
 *
 * Created			: 4/17/2022 8:38:56 PM
 * Author @github	: bimalka98; ViduraErandika; 
 * 
 * Pin Configuration:
 * PC6 - reset
 * PD0 - key on determinant switch( pin will be high if key is on)
 * PD2 - IR digital input
 * PD4 - lift down pin(if this pin is high,stand should be lift down)
 * PB6,PB7- oscillator
 * PD5 - red led(while stand is lifting down and lifting up, this led should on)
 * PD6 - green led (if the stand is in right position, this led is on)
 * PC5 - SCL
 * PC4 - SDL - I2C communication with gyroscope
 * PC3,PC2,PC1,PC0 - stepper outs
 */ 

 // define the clock frequency to be 4MHz
 // also declared in i2c.c remember to change there too!
#ifndef F_CPU
#define F_CPU 4000000UL
#endif

// define the baud rate to be 9600 bps (for the UART)
#ifndef BAUD
#define BAUD 9600
#endif


// Include required built-in header files
#include <avr/io.h>
#include <util/delay.h>
#include <util/setbaud.h>
#include <avr/interrupt.h>
#include <math.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>

// Include user defined header files
#include "MPU6050/mpu6050.h"
#include "MPU6050/mpu6050_reg.h"
#include "I2C/i2c.h"
#include "stepperMotor/stepper.h"

// define the macros
#define RESTING_ANGLE_OF_BIKE 30.0 // angle in degrees of the bike when it is resting (parked)

// define the global variables
bool KeyONDetected = false; // if key is on, this variable will be made true
bool CurrentAngleOfBikeZero = false;	//if angle of the bike 0, variable be made true
float CurrentAngleOfBike = 0; // angle of the bike



int main(void)
{
   
	initialize();
	

    while (1) 
    {
        // read the PIND0 and store its value to the variable KeyONDetected
        KeyONDetected = (PIND & (1 << PIND0)); // if key is on, this variable will be made true 
		  
		if (KeyONDetected & CurrentAngleOfBikeZero)
			stepperUp(KeyONDetected,CurrentAngleOfBikeZero);
		
		  
        // get the angle of the bike from the gyroscope
        //CurrentAngleOfBike = getCurrentAngleOfBike();

        if ((PIND&0x08) == 0) //check whether lift down switch is ground
			stepperDown();


    }
}

