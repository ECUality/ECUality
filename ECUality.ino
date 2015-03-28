#include "TimerOne.h"
#include <eeprom.h>
#include "ECUality.h"
#include "EEPROMAnything.h"
 
#define MAX_MAP_SIZE	10
#define MIN_MAP_SIZE	3

// Pin mappings
const char air_flow_pin		=	A12;
const char air_temp_pin		=	A14;
const char o2_pin			=	A9;
const char coolant_temp_pin =	A13;
const char oil_pressure_pin =	A11;
const char tach_pin = 19;

const char inj1_pin = 42;
const char inj2_pin = 44;
const char inj3_pin = 46;
const char inj4_pin = 48;

// operating control variables
unsigned char air_stabilize_rate;	// the rate at which accelerator pump transient decays.
unsigned int cold_enrich_rate;		// how aggressive the enrichment is with respect to temperature. 
unsigned int cold_threshold;		// the temperature below which enrichment kicks in. 

unsigned int ms_between_doing_task[10];
unsigned int timer_for_task[10] = { 0 };
void (*task[10]) (void);
unsigned char n_tasks;

// sensor input variables
int air_flow, o2, air_temp, coolant_temp, oil_pressure;
unsigned int tach_period;

// variables that capture dynamic aspects of sensor input
int air_flow_d, air_flow_snap, o2_d;	

// map variables
unsigned int n_air_gridlines, n_rpm_gridlines;
unsigned int air_gridline[MAX_MAP_SIZE], rpm_gridline[MAX_MAP_SIZE];
unsigned int engine_map[MAX_MAP_SIZE * MAX_MAP_SIZE];
unsigned int map_correction[MAX_MAP_SIZE * MAX_MAP_SIZE];
unsigned int map_volatility[MAX_MAP_SIZE * MAX_MAP_SIZE];
 
void setup() 
{
	task[0] = readAirFlow;		ms_between_doing_task[0] = 250;
	task[1] = readO2Sensor;		ms_between_doing_task[1] = 300;
	task[2] = readAirTemp;		ms_between_doing_task[2] = 250;
	task[3] = readCoolantTemp;	ms_between_doing_task[3] = 250;
	task[4] = readOilPressure;	ms_between_doing_task[4] = 250;
	task[5] = calcStuff;		ms_between_doing_task[5] = 20;
	n_tasks = 5;
	
	loadMapFromEE();

	// input interrupt pin
	pinMode(tach_pin, INPUT);					// This gets attached to an interrupt

	pinMode(inj1_pin, OUTPUT);
	pinMode(inj2_pin, OUTPUT);
	pinMode(inj3_pin, OUTPUT);
	pinMode(inj4_pin, OUTPUT);

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
	
	TIMSK3 |= _BV(OCIE3A);				// enable output compare A interrupt on timer 3
	// disable the timer 0 interrupt.  This breaks millis() but prevents interference with pulse timing.
	//TIMSK0 &= 0x00;					

	setInjectorDuration(5);

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


	switch (c[0])
	{
	case 'a':		// 'a' for setting air %
		receivedNum = constrain(receivedNum, 0, 255);
		//Set_Air((char)receivedNum);
		Serial.print("Air set to: ");
		Serial.println(receivedNum);
		break;

	case 'm':		// MAP update
		receiveMap();
		break;

	case 'r':		// REPORT map
		reportMap();
		break;

	case 'l':		// LOAD map from ee
		loadMapFromEE();
		break;

	case 's':		// SAVE map
		saveMapToEE();
		break;

	case 'd':		// DURATION manual control
		receivedNum = Serial.parseInt();
		setInjectorDuration(receivedNum);
		Serial.print("Injector dur set to: ");
		Serial.println(receivedNum);
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

// Sensor reading functions
void readAirFlow()
{
	toggle(13);

	air_flow_d = -air_flow;
	air_flow = analogRead(air_flow_pin);	// takes 100us.  Should shorten
	air_flow_d += air_flow;

	air_flow_snap += air_flow_d;

	if (air_flow_snap)
	{
		if (air_flow_snap > air_stabilize_rate)
			air_flow_snap -= air_stabilize_rate;
		else
			air_flow_snap = 0;
	}


}
void readO2Sensor()
{
	o2_d = -o2;
	o2 = analogRead(o2_pin);		// this takes 100us.  Should shorten
	o2_d += o2;
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
	// .5V = 0psi  4.5V = 100psi.  25psi/V 
	// 25psi/V * 5V/1023tic = .1222 psi/tic
	// 1 psi ~ 8 tics. 
	oil_pressure = analogRead(oil_pressure_pin);
}

unsigned int interpolateMap(unsigned char air, unsigned int rpm)
{
	int i_air, i_rpm;
	unsigned int inj_r0a0, inj_r0a1, inj_r1a0, inj_r1a1, inj_r0ak, inj_r1ak;  // 
	i_air = findIndexJustAbove(air_gridline, air, n_air_gridlines);
	i_rpm = findIndexJustAbove(air_gridline, air, n_air_gridlines);

	// check if both i's are -1 (key was higher than all array elements)
	// check if any of them are 0 (key was lower than lowest element)
	// handle those cases
	
	inj_r0a0 = engine_map[(i_rpm - 1)*n_air_gridlines+i_air - 1];		// use pointer math instead? 
	inj_r0a1 = engine_map[(i_rpm - 1)*n_air_gridlines+i_air];
	inj_r1a0 = engine_map[i_rpm*n_air_gridlines+i_air - 1];
	inj_r1a1 = engine_map[i_rpm*n_air_gridlines+i_air];

	// here we do bilinear interpolation
	inj_r0ak = linearInterp(air, air_gridline[i_air - 1], air_gridline[i_air], inj_r0a0, inj_r0a1);
	inj_r1ak = linearInterp(air, air_gridline[i_air - 1], air_gridline[i_air], inj_r1a0, inj_r1a1);
	return linearInterp(rpm, rpm_gridline[i_rpm - 1], rpm_gridline[i_rpm], inj_r0ak, inj_r1ak);

}
int linearInterp(int x_key, int x1, int x2, int y1, int y2)
{
	int dx, Dx, Dy;
	Dx = x2 - x1;
	dx = x_key - x1; 
	Dy = y2 - y1;
	return y1 + (Dy*dx) / Dx;
}
int findIndexJustAbove(unsigned int array[], int key, int length)
{
	int i;
	for (i = 0; i < length; i++)
	{
		if (array[i] > key)
			return i;
	}
	// didn't find a value above key.  Return -1 to indicate an error. 
	return -1;
}

int adjustForTakeoff( int air_flow )
{
	return 0;
}
int adjustForColdEngine(unsigned int &duration, int coolant_temp, int air_flow)
{
	static unsigned int adjustment;
	// applies a positive offset to injector duration.  
	// offset depends linearly on coolant temperature.  
	// offset depends linearly on injector duration (works like a % of normal)
	// Parameters: 
	// takes a threshold temp and a rate (mx +b) for linear dependence on  coolant temperature

	if (coolant_temp >= cold_threshold)		// is engine already warm? 
		return 0; 
	adjustment = cold_enrich_rate * (cold_threshold - coolant_temp) * duration;
	adjustment >>= 6;			// divide by 64; 
	return adjustment;
}
int adjustForCoasting( int rpm, int air_flow )
{
	return 0;
}

void setInjectorDuration(unsigned int new_inj_duration)	// sets the injector duration
{
	OCR3A = new_inj_duration;
}

void Delay_Ms(unsigned int d) {
	unsigned int  i, k;
	for (i = 0; i<d; i++)
	{
		for (k = 0; k<2000; k++)
		{
			asm volatile ("NOP");
		}

	}
}
void toggle(unsigned char pin) 
{
	// Toggle LED
	digitalWrite(pin, digitalRead(pin) ^ 1);
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
	GTCCR |= _BV(PSRSYNC);		// clear the prescaler. 
	tach_period = TCNT3; 		// record the timer setting 
	TCNT3 = 0;					// start the timer over. 

	// set all injectors high (in same instruction)
	PORTL |= 0xAA;				// 0b10101010;
	//PORTG |= PG1;				// this can be dropped if I move a pin over
}
ISR(TIMER3_COMPA_vect)			// this runs when TCNT3 == OCR3A. 
{
	// set all injectors low (in same instruction)
	PORTL &= 0x55;				// 0b01010101; 
	//PORTG &= ~PG1;
}

