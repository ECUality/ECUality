#include "TimerOne.h"
 
const char air_flow_pin		=	A12;
const char air_temp_pin		=	A14;
const char o2_pin			=	A9;
const char coolant_temp_pin =	A13;
const char oil_pressure_pin =	A11;

unsigned int ms_between_doing_task[10];
unsigned int timer_for_task[10] = { 0 };
void (*task[10]) (void);
unsigned char n_tasks;

unsigned char air_flow, air_temp, o2, coolant_temp, oil_pressure;

unsigned int engine_map[9][9];
 
void setup() 
{
	task[0] = readAirFlow;		ms_between_doing_task[0] = 50;
	task[1] = readO2Sensor;		ms_between_doing_task[1] = 300;
	task[2] = readAirTemp;		ms_between_doing_task[2] = 250;
	task[3] = readCoolantTemp;	ms_between_doing_task[3] = 250;
	task[4] = readOilPressure;	ms_between_doing_task[4] = 250;
	n_tasks = 5;
	
	loadMap(engine_map);

  // Initialize the digital pin as an output.
  // Pin 13 has an LED connected on most Arduino boards
  pinMode(13, OUTPUT);    
  pinMode(12, OUTPUT);
  pinMode(11, OUTPUT);
  
  Timer1.initialize(1000);				// set half-period = 1000 microseconds (1 ms)
  Timer1.attachInterrupt( Timer1_isr ); // attach the service routine here
  
  //attachInterrupt(4, Toggle_led, RISING);  
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
	toggle(12);
	//air_flow = analogRead(air_flow_pin);
}

void readO2Sensor()
{
	toggle(13);
	//o2 = digitalRead(o2_pin);
}

void readAirTemp() 
{
	air_temp = analogRead(air_temp_pin);
}

void readCoolantTemp() 
{
	coolant_temp = analogRead(coolant_temp_pin);
}

void readOilPressure()
{
	oil_pressure = analogRead(oil_pressure_pin);
}

void loadMap(unsigned int engine_map[][9])
{
	
	unsigned int map1[9][9] = {	
		{ 0,	220,	180,	140,	100,	80, 	60, 	40, 	20 },
		{ 500,	2473,	2160,	1899,	1789,	1537,	1301,	1185,	1129 },
		{ 695,	2473,	2161,	1900,	1744,	1484,	1166,	988,	925 },
		{ 965,	2471,	2171,	1922,	1619,	1176,	945,	775,	776 },
		{ 1341,	2472,	2170,	1898,	1229,	940,	774,	757,	827 },
		{ 1863,	2471,	2338,	1632,	949,	749,	635,	676,	710 },
		{ 2589,	2464,	2006,	1223,	749,	618,	635,	645,	624 },
		{ 3589,	2476,	1596,	944,	615,	587,	587,	588,	901 },
		{ 5000,	1931,	1203,	766,	596,	564,	561,	560,	589  }	
		};
			
	engine_map = map1;			
	
	/*
	for (r=0; r < rows; r++)
	{
		for (c=0; c < cols; c++)
		{
			
			
		}
		
	} */
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

void toggle(unsigned char pin) {
	// Toggle LED
    digitalWrite( pin, digitalRead( pin ) ^ 1 );
}