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

 // define the clock frequency to be 8MHz
#ifndef F_CPU
#define F_CPU 4000000UL
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
float CurrentAngleOfBike = 0; // angle of the bike



int main(void)
{
    /* Data Direction Definitions: input low, output high */

    DDRD |= (1 << PIND5) | (1 << PIND6); // make pins related to LEDs as output pins
    DDRC |= (1 << PINC3) | (1 << PINC2) | (1 << PINC1) | (1 << PINC0); // make pins related to stepper as output pins
	

	



    while (1) 
    {
        // read the PIND0 and store its value to the variable KeyONDetected
        KeyONDetected = (PIND & (1 << PIND0)); // if key is on, this variable will be made true   

        // get the angle of the bike from the gyroscope
        //CurrentAngleOfBike = getCurrentAngleOfBike();

        

    }
}

