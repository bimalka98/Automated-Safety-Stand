/*
 * utilities.h
 *
 * Created: 4/24/2022 7:33:36 PM
 *  Author: bimalka98
 */ 


#ifndef UTILITIES_H_
#define UTILITIES_H_

// include header files 
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <inttypes.h>


// variable declarations
volatile double count;
const double unit_t = 8/F_CPU;

void setUpTimer();

void getTime(double * deltat);



#endif /* UTILITIES_H_ */