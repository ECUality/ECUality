#include "TimerOne.h"
 
unsigned char led1 = 13;
unsigned char led2 = 12;
 
void Delay_Ms( unsigned int );
 
void setup() 
{
  // Initialize the digital pin as an output.
  // Pin 13 has an LED connected on most Arduino boards
  pinMode(13, OUTPUT);    
  pinMode(12, OUTPUT);
  pinMode(11, OUTPUT);
  
  
  Timer1.initialize(5000000); // set half-period = 100000 microseconds (or 0.1 sec)
  Timer1.attachInterrupt( Timer_isr ); // attach the service routine here
  
  attachInterrupt(4, Toggle_led, RISING);  
}
 
void loop()
{
	// Blink the led 5 times a second. 
	Delay_Ms(500);
	digitalWrite(12, HIGH);
	Delay_Ms(500);
	digitalWrite(12, LOW);
	
}

void Delay_Ms( unsigned int d ) {
	unsigned int  i, k;
	for ( i=0; i<d; i++ )
	{
		for ( k=0; k<2000; k++) 
		{
			asm volatile ("NOP");
		}
			
	}
}

 
/// --------------------------
/// Custom ISR Timer Routine
/// --------------------------
void Timer_isr()
{
	sei();
	// Just spin here for a bit to block main code.  LED should blink slowly because most of the time will be spent here.
    //delay(900);
	Delay_Ms(4800);
	
	// Toggle LED
    digitalWrite( 13, digitalRead( 13 ) ^ 1 );
}

void Toggle_led() {
	// Toggle LED
    digitalWrite( 11, digitalRead( 11 ) ^ 1 );
}