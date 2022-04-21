/*
 * stepper.c
 *
 * Created: 4/21/2022 11:15:08 PM
 *  Author: vidur
 */ 

#include <avr/io.h>
#include <util/delay.h>

unsigned int delay = 100;
unsigned int count = 0;

int stepperUp( ){
	DDRD = ~(1<<1);  //PA1 as input switch
	PORTD |= (1<<1);  //activate pull up
	DDRC = 0xFF;	//PORT C as an output port
	
	while((PIND&0x02)==1){
		count += 1;
		PORTC = 0x66;
		_delay_ms(delay);
		PORTC = 0xCC;
		_delay_ms(delay);
		PORTC = 0x99;
		_delay_ms(delay);
		PORTC = 0x33;
		_delay_ms(delay);
	}
			
	return 0;
}

int stepperDown(){
	while (count>=0){
		count -= 1;
		PORTC = 0x66;
		_delay_ms(delay);
		PORTC = 0x33;
		_delay_ms(delay);
		PORTC = 0x99;
		_delay_ms(delay);
		PORTC = 0xCC;
		_delay_ms(delay);
	}
	
	return 0;
}