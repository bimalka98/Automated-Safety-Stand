/*
 * stepper.c
 *
 * Created: 4/21/2022 11:15:08 PM
 *  Author: vidura
 */ 

#include <avr/io.h>
#include <util/delay.h>

#define  OUTPUT_RED_LED (5) //PD5 >> red led(while stand is lifting down and lifting up, this led should on)
#define  OUTPUT_GREEN_LED (6)  //PD6 >> green led (if the stand is in right position, this led is on)
#define  IR_INPUT (2)		//PD2	>> IR digital input
#define  KEY_SWITCH (0)		//PD0	>> key on determinant switch( pin will be high if key is on)
#define  LIFT_DOWN_PIN (4)	//PD4	>> lift down pin(if this pin is high,stand should be lift down)

#define STEP_A (3)
#define STEP_B (2)
#define STEP_C (1)
#define STEP_D (0)


unsigned int delay = 500;
unsigned int count = 0;

int initialize(){
	DDRD |= (1<<OUTPUT_RED_LED)|(1<<OUTPUT_GREEN_LED); // PD5 and PD6 as outputs				
	DDRD &= ~(1<<KEY_SWITCH)|~(1<<IR_INPUT)|~(1<<LIFT_DOWN_PIN);  // PD0 PD2 PD4 as inputs  				
	DDRC |= (1<<STEP_A)|(1<<STEP_B)|(1<<STEP_C)|(1<<STEP_D);	//steppers as an output port
	
	PORTD |= (1<<LIFT_DOWN_PIN);  //activate pull up
}

int stepperUp( ){
	
	
	while((PIND&0x02)==1){
		count += 1;
		PORTD |= (1<<OUTPUT_RED_LED); //switch on the RED LED
		
		PORTC |= (1<<STEP_A)|(1<<STEP_D); //1001
		PORTC &= ~(1<<STEP_B)|~(1<<STEP_C);
		_delay_ms(delay);
		
		PORTC |= (1<<STEP_A);	//1000
		PORTC &= ~(1<<STEP_B)|~(1<<STEP_C)|~(1<<STEP_D);
		_delay_ms(delay);
		
		PORTC |= (1<<STEP_A)|(1<<STEP_B); //1100
		PORTC &= ~(1<<STEP_C)|~(1<<STEP_D);
		_delay_ms(delay);
		
		PORTC |= (1<<STEP_B);	//0100
		PORTC &= ~(1<<STEP_A)|~(1<<STEP_C)|~(1<<STEP_D);
		_delay_ms(delay);
		
		PORTC |= (1<<STEP_B)|(1<<STEP_C); //0110
		PORTC &= ~(1<<STEP_A)|~(1<<STEP_D);
		_delay_ms(delay);
		
		PORTC |= (1<<STEP_C);	//0010
		PORTC &= ~(1<<STEP_A)|~(1<<STEP_B)|~(1<<STEP_D);
		_delay_ms(delay);
		
		PORTC |= (1<<STEP_C)|(1<<STEP_D); //0011
		PORTC &= ~(1<<STEP_A)|~(1<<STEP_B);
		_delay_ms(delay);
		
		PORTC |= (1<<STEP_D);	//0001
		PORTC &= ~(1<<STEP_A)|~(1<<STEP_B)|~(1<<STEP_C);
		_delay_ms(delay);
	}
	
	PORTD &= ~(1<<OUTPUT_RED_LED);		//switch off the RED LED
	return 0;
}

int stepperDown(){
	
	if ((PIND&0x08) == 0) //check whether lift down switch is ground
	{
		while (count>=0){
			count -= 1;
			PORTD |= (1<<OUTPUT_RED_LED); //switch on the RED LED
			
			PORTC |= (1<<STEP_D);	//0001
			PORTC &= ~(1<<STEP_A)|~(1<<STEP_B)|~(1<<STEP_C);
			_delay_ms(delay);
			
			PORTC |= (1<<STEP_C)|(1<<STEP_D); //0011
			PORTC &= ~(1<<STEP_A)|~(1<<STEP_B);
			_delay_ms(delay);
			
			PORTC |= (1<<STEP_C);	//0010
			PORTC &= ~(1<<STEP_A)|~(1<<STEP_B)|~(1<<STEP_D);
			_delay_ms(delay);
			
			PORTC |= (1<<STEP_B)|(1<<STEP_C); //0110
			PORTC &= ~(1<<STEP_A)|~(1<<STEP_D);
			_delay_ms(delay);
			
			PORTC |= (1<<STEP_B);	//0100
			PORTC &= ~(1<<STEP_A)|~(1<<STEP_C)|~(1<<STEP_D);
			_delay_ms(delay);
			
			PORTC |= (1<<STEP_A)|(1<<STEP_B); //1100
			PORTC &= ~(1<<STEP_C)|~(1<<STEP_D);
			_delay_ms(delay);
			
			PORTC |= (1<<STEP_A);	//1000
			PORTC &= ~(1<<STEP_B)|~(1<<STEP_C)|~(1<<STEP_D);
			_delay_ms(delay);
			
			PORTC |= (1<<STEP_A)|(1<<STEP_D); //1001
			PORTC &= ~(1<<STEP_B)|~(1<<STEP_C);
			_delay_ms(delay);
			
		}
	}
	
	PORTD &= ~(1<<OUTPUT_RED_LED);		//switch off the RED LED
	return 0;
}