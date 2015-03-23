#include "TimerOne.h"
#include <eeprom.h>
#include "ECUality.h"
#include "EEPROMAnything.h"
 
#define MAX_MAP_SIZE	10
#define MIN_MAP_SIZE	3

const char air_flow_pin		=	A12;
const char air_temp_pin		=	A14;
const char o2_pin			=	A9;
const char coolant_temp_pin =	A13;
const char oil_pressure_pin =	A11;
const char tach_pin = 19;

unsigned int ms_between_doing_task[10];
unsigned int timer_for_task[10] = { 0 };
void (*task[10]) (void);
unsigned char n_tasks;

unsigned char air_flow, air_temp, o2, coolant_temp, oil_pressure;
unsigned int tach_period;

// map variables
unsigned int n_air_gridlines, n_rpm_gridlines;
unsigned int air_gridline[MAX_MAP_SIZE], rpm_gridline[MAX_MAP_SIZE];
unsigned int engine_map[MAX_MAP_SIZE * MAX_MAP_SIZE];
unsigned int map_correction[MAX_MAP_SIZE * MAX_MAP_SIZE];
unsigned int map_volatility[MAX_MAP_SIZE * MAX_MAP_SIZE];
 
void setup() 
{
	task[0] = readAirFlow;		ms_between_doing_task[0] = 50;
	task[1] = readO2Sensor;		ms_between_doing_task[1] = 300;
	task[2] = readAirTemp;		ms_between_doing_task[2] = 250;
	task[3] = readCoolantTemp;	ms_between_doing_task[3] = 250;
	task[4] = readOilPressure;	ms_between_doing_task[4] = 250;
	n_tasks = 5;
	
	loadMapFromEE();

	// input interrupt pin
	pinMode(tach_pin, INPUT);					// This gets attached to an interrupt
	attachInterrupt(4, tachRisingEdgeISR, RISING);	// interrupt 4 maps to pin 19. 

	// Initialize the digital pin as an output.
	// Pin 13 has an LED connected on most Arduino boards
	pinMode(13, OUTPUT);    
	pinMode(12, OUTPUT);
	pinMode(11, OUTPUT);
  
	Timer1.initialize(1000);				// set half-period = 1000 microseconds (1 ms)
	Timer1.attachInterrupt( Timer1_isr ); // attach the service routine here

	// Configure Timer3 for measuring injector pulse duration (using interrupt)
	TCCR3A = 0;							// clear control register A 
	TCCR3B = _BV(CS31) | _BV(CS30);		// start the timer at clk/64 prescale. 

	// Configure Timer0 for generating a fast PWM.  16us period, 62.5kHz
	// CONFIG BITS:
	// WGM0<3:0>	= 0 (0000)	normal mode, counts up to 0xFFFF.  
	// COM0A<1:0>	= 2 (10)	clears OC0A on compare match so OCR0A represents "high" time. 
	// COM0B<1:0>	= 2 (10)	same as A.
	// CS0<2:0>		= 1 (001)	1:1 pre-scaling, timer running. 
  
	Serial.begin(115200);
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
	//Serial.write(c, 1);		// echo

	//receivedNum = Serial.parseInt();
	//Serial.readBytesUntil('\n', &c[2], 8);

	switch (c[0])
	{
	case 'a':		// 'a' for setting air %
		receivedNum = constrain(receivedNum, 0, 255);
		//Set_Air((char)receivedNum);
		Serial.print("Air set to: ");
		Serial.println(receivedNum);
		break;

	case 'm':		// 'o' for setting O2
		receiveMap();
		break;

	case 'r':		// 'r' for setting RPM
		reportMap();
		break;

	case 'l':		// 'i' to sample the Injector duration
		loadMapFromEE();
		break;

	case 's':		// 'r' for setting RPM
		saveMapToEE();
		break;

	default:
		Serial.println("not understood");
	}
	dumpLine();
}
void dumpLine(void)
{
	char str[5];
	while (Serial.readBytesUntil('\n', str, 4));
}

// Serial to data functions
int receiveMap()
{
	char str[3] = "";	// all zeros.
	unsigned int new_engine_map[MAX_MAP_SIZE * MAX_MAP_SIZE];
	unsigned int new_air_gridline[MAX_MAP_SIZE];
	unsigned int new_rpm_gridline[MAX_MAP_SIZE];
	unsigned int new_n_air_gridlines, new_n_rpm_gridlines;
	unsigned int new_map_size;

	Serial.readBytes(str, 2);

	if (strcmp(str, "ap") != 0)
	{
		Serial.println("spell 'map' please");
		return -1;
	}

	if (!receiveUIntBetween(&new_n_air_gridlines, MIN_MAP_SIZE, MAX_MAP_SIZE, "air_gridlines"))
		return- 1;

	if (!receiveUIntBetween(&new_n_rpm_gridlines, MIN_MAP_SIZE, MAX_MAP_SIZE, "rpm_gridlines"))
		return -1;

	if (!receiveUIntArray(new_air_gridline, new_n_air_gridlines, "air gridline"))
		return -1;
	
	if (!receiveUIntArray(new_rpm_gridline, new_n_rpm_gridlines, "rpm gridline"))
		return -1;

	new_map_size = new_n_air_gridlines * new_n_rpm_gridlines;
	if (!receiveUIntArray(new_engine_map, new_map_size, "map data"))
		return -1;


	n_air_gridlines = new_n_air_gridlines;
	n_rpm_gridlines = new_n_rpm_gridlines;
	copyArray(new_air_gridline, air_gridline, n_air_gridlines);
	copyArray(new_rpm_gridline, rpm_gridline, n_rpm_gridlines);
	copyArray(new_engine_map, engine_map, new_map_size);

	dumpLine();		// dump any additional characters. 

	reportMap();
	
}
void reportMap()
{
	int i;
	reportArray("air_gridline:\n", air_gridline, n_air_gridlines);
	reportArray("rpm_gridline:\n", rpm_gridline, n_rpm_gridlines);
	Serial.println("engine_map:");
	for (i = 0; i < n_rpm_gridlines; ++i)
	{
		reportArray(" ", &engine_map[n_air_gridlines*i], n_air_gridlines);
	}
	Serial.print("\n");
}
void reportArray(char str[], unsigned int *data, unsigned int n)
{
	int i;
	Serial.print(str);
	for (i = 0; i < n; ++i)
	{
		Serial.print(data[i]);
		Serial.print("\t");
	}
	Serial.print("\n");
}
char receiveUIntArray(unsigned int *new_array, unsigned int n_array, char str[])
{
	unsigned int i;

	Serial.setTimeout(50);	// set the timeout to receive each number to 50ms

	for (i = 0; i < n_array; ++i)
	{
		new_array[i] = Serial.parseInt();
		//Serial.print(new_array[i]);

		if (!new_array[i])
		{
			Serial.print("timed out while reading ");
			Serial.println(str);
			return 0;
		}

	}
	return 1; 
}
char receiveUIntBetween(unsigned int *var, unsigned int lower, unsigned int upper, char var_name[])
{
	unsigned int new_value = Serial.parseInt();
	if (new_value > upper)
	{
		Serial.print("too many ");
		Serial.println(var_name);
		return 0;
	}
	if (new_value < lower)
	{
		Serial.print("too few");
		Serial.println(var_name);
		return 0;
	}
	*var = new_value;
	return 1;
}

// EE access functions
void loadMapFromEE()
{
	// eeprom addresses: 0 = n_air_gridlines, n_rpm_gridlines
	unsigned int address, map_size, value;
	address = 0;

	address += EEPROM_readAnything(address, value);
	if (!assignIfBetween(value, n_air_gridlines, MAX_MAP_SIZE, MIN_MAP_SIZE, "n_air_gridlines"))
		return;

	address += EEPROM_readAnything(address, value);
	if (!assignIfBetween(value, n_rpm_gridlines, MAX_MAP_SIZE, MIN_MAP_SIZE, "n_rpm_gridlines"))
		return;

	address += EEPROM_readAnything(address, air_gridline);
	address += EEPROM_readAnything(address, rpm_gridline);
	address += EEPROM_readAnything(address, engine_map);
	Serial.println("map loaded from EE");
	
}
void saveMapToEE()
{
	// eeprom addresses: 0 = n_air_gridlines, n_rpm_gridlines
	unsigned int address, map_size;
	address = 0;

	address += EEPROM_writeAnything(address, n_air_gridlines);
	address += EEPROM_writeAnything(address, n_rpm_gridlines);
	address += EEPROM_writeAnything(address, air_gridline);
	address += EEPROM_writeAnything(address, rpm_gridline);
	address += EEPROM_writeAnything(address, engine_map);
	Serial.println("saved map to EE");
}
char assignIfBetween(const unsigned int source, unsigned int &destination, unsigned int max, unsigned int min, char var_name[])
{
	if ((source > MAX_MAP_SIZE) || (source < MIN_MAP_SIZE))
	{
		Serial.print(var_name);
		Serial.println(" out of range");
		return 0;
	}
	destination = source;
	return 1;
}

void addArrays(unsigned int source_array[], unsigned int destination_array[], unsigned int n)
{
	unsigned int i;

	for (i = 0; i < n; i++)
		destination_array[i] += source_array[i];
}
void copyArray(unsigned int source_array[], unsigned int destination_array[], unsigned int n)
{
	unsigned int i;

	for (i = 0; i < n; i++)
	{
		destination_array[i] = source_array[i];
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

// Sensor reading functions
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

void tachRisingEdgeISR()
{
	tach_period = TCNT3; 		// record the timer setting 
	TCNT3 = 0;					// start the timer over. 

	// set all injectors high (in same instruction)
	PORTL |= 0xAA;				// 0b10101010;
	PORTG |= PG1;
}

ISR(TIMER3_COMPA_vect)			// this runs when TCNT3 == OCR3A. 
{
	// set all injectors low (in same instruction)
	PORTL &= 0x55;				// 0b01010101; 
	PORTG &= ~PG1;
	
}
void toggle(unsigned char pin) {
	// Toggle LED
    digitalWrite( pin, digitalRead( pin ) ^ 1 );
}