#include "TimerOne.h"
 
const char air_flow_pin =	A12;
const char air_temp_pin =	A14;
const char o2_pin		=	A9;
const char coolant_temp_pin = A13;
const char oil_pressure_pin = A11;

unsigned int ms_between_doing_task[10];
unsigned char timer_for_task[10] = { 0 };
void (*task[10]) (void);
unsigned char n_tasks;

unsigned char air_flow, air_temp, o2, coolant_temp, oil_pressure;

 
void Delay_Ms( unsigned int );
 
void setup() 
{
	task[0] = readAirFlow;		ms_between_doing_task[0] = 300;
	task[1] = readO2Sensor;		ms_between_doing_task[1] = 100;
	task[2] = readAirTemp;		ms_between_doing_task[2] = 250;
	task[3] = readCoolantTemp;	ms_between_doing_task[3] = 250;
	task[4] = readOilPressure;	ms_between_doing_task[4] = 250;
	n_tasks = 5;

  // Initialize the digital pin as an output.
  // Pin 13 has an LED connected on most Arduino boards
  pinMode(13, OUTPUT);    
  pinMode(12, OUTPUT);
  pinMode(11, OUTPUT);
  
  Timer1.initialize(1000);				// set half-period = 1000 microseconds (1 ms)
  Timer1.attachInterrupt( Timer1_isr ); // attach the service routine here
  
  attachInterrupt(4, Toggle_led, RISING);  
}
 
void loop()
{

	Poll_Serial();

	
}

void Poll_Serial()
{
	static char c[10];
	static unsigned int receivedNum; 

	if (!Serial.available())
		return;
	
	Serial.readBytes(c, 1);
	receivedNum = Serial.parseInt();
	Serial.readBytesUntil('\n', &c[2], 8);

	switch (c[0])
	{
	case 'a':		// 'a' for setting air %
		receivedNum = constrain(receivedNum, 0, 255);
		//Set_Air((char)receivedNum);
		Serial.print("Air set to: ");
		Serial.println(receivedNum);
		break;

	case 'o':		// 'o' for setting O2

		break;

	case 'r':		// 'r' for setting RPM

		break;

	case 'c':
		
		break;

	case 'i':		// 'i' to sample the Injector duration
		
		break;

	case 't':		// 'i' to sample the Injector duration
		
		break;

	default:
		Serial.println("not understood");
	}
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

void readAirFlow()
{
	//air_flow = analogRead(air_flow_pin);
	digitalWrite(13, 1 ^ digitalRead(13));
}

void readO2Sensor()
{
	digitalWrite(12, 1 ^ digitalRead(12));
}

void readAirTemp() 
{

}

void readCoolantTemp() 
{

}

void readOilPressure()
{

}

void Timer1_isr()
{
	sei();
	static unsigned char i;

	for (i = 0; i < n_tasks; i++)
	{
		timer_for_task[i]++;
		if (timer_for_task[i] >= ms_between_doing_task[i])
		{
			(*task[i])();			// call task i.
			timer_for_task[i] = 0;		// reset this task's timer
		}
	}

}





void Toggle_led() {
	// Toggle LED
    digitalWrite( 11, digitalRead( 11 ) ^ 1 );
}