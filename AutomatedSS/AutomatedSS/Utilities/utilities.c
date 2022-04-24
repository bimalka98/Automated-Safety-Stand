/*
 * utilities.c
 *
 * Created: 4/24/2022 7:33:17 PM
 *  Author: bimalka98
 */ 

#include "utilities.h"


void setUpTimer(){
	TCCR1A = 0x00;
	TIMSK1 |= (1 << TOIE1);
	TCCR1B |= (1 << CS11);
	TCCR1B &= ~( (1 << CS12)  | (1 << CS10)); // pre scaler=8
}

void getTime(double * deltat){
	cli();
	uint8_t l = TCNT1L;
	uint8_t h = TCNT1H;
	uint16_t step = h<<8 | l;
	*deltat = (double)step*5e-7 + count*0.032768;
	count = 0;
	sei();
}