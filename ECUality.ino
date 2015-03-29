//#include "TimerOne.h"
#include <eeprom.h>
#include "ECUality.h"
#include "EEPROMAnything.h"
 
#define MAX_MAP_SIZE	10
#define MIN_MAP_SIZE	3

char inspect;

// Pin mappings
uint8_t air_flow_pin		= A12;
uint8_t air_temp_pin		= A14;
uint8_t o2_pin				= A9;
uint8_t coolant_temp_pin	= A13;
uint8_t oil_pressure_pin	= A11;
uint8_t tach_pin			= 19;
uint8_t idl_full_pin		= A10;

const char inj1_pin = 42;
const char inj2_pin = 44;
const char inj3_pin = 46;
const char inj4_pin = 48;

// operating control variables
unsigned char air_stabilize_rate;	// the rate at which accelerator pump transient decays.
unsigned int cold_eng_enrich_rate;		// how aggressive the enrichment is with respect to temperature. 
unsigned int cold_threshold;		// the temperature below which enrichment kicks in. 

// task variables
unsigned int ms_freq_of_task[10];
unsigned int task_runtime[10];
unsigned int ms_since_last_task[10] = { 0 };
void (*task[10]) (void);
unsigned char n_tasks;

// sensor input variables
int air_flow, rpm, o2, air_temp, coolant_temp, oil_pressure;
unsigned int tach_period, inj_duration;

// variables that capture dynamic aspects of sensor input
int air_flow_d, air_flow_snap, o2_d;	

// map variables
unsigned int n_air, n_rpm;
unsigned int air_gridline[MAX_MAP_SIZE], rpm_gridline[MAX_MAP_SIZE];
unsigned int engine_map[MAX_MAP_SIZE * MAX_MAP_SIZE];
unsigned int map_correction[MAX_MAP_SIZE * MAX_MAP_SIZE];
unsigned int map_volatility[MAX_MAP_SIZE * MAX_MAP_SIZE];
 
void setup() 
{
	Serial.begin(115200);

	inspect = 0;
	
	task[0] = readAirFlow;			ms_freq_of_task[0] = 50;
	task[1] = readO2Sensor;			ms_freq_of_task[1] = 50;
	task[2] = calcRPM;				ms_freq_of_task[2] = 50;
	task[3] = updateInjDuration;	ms_freq_of_task[3] = 50;
	task[4] = readAirTemp;			ms_freq_of_task[4] = 250;
	task[5] = readOilPressure;		ms_freq_of_task[5] = 250;
	task[6] = readCoolantTemp;		ms_freq_of_task[6] = 250;
	n_tasks = 7;
	
	loadMapFromEE();  

	// input interrupt pin
	pinMode(tach_pin, INPUT);					// This gets attached to an interrupt
	pinMode(inj1_pin, OUTPUT);
	pinMode(inj2_pin, OUTPUT);
	pinMode(inj3_pin, OUTPUT);
	pinMode(inj4_pin, OUTPUT);
	pinMode(13, OUTPUT);

	inj_duration = 5;

	// TIMERS
	//Timer1.initialize(2000);				// set half-period = 1000 microseconds (1 ms)
	//Timer1.attachInterrupt( isrTimer1 ); // attach the service routine here
	
	TCCR1A = 0;				// disables all output compare modules and clears WGM1<0-1> 
	TCCR1B = _BV(CS10);		// sets prescaler to 1:1, and turns on timer, clears WGM1<3:2>
	TCCR1B |= _BV(WGM12) | _BV(WGM13);	// 
	// WGM1<3:0> = 1,1,0,0 = mode 12 "CTC" with top = ICR.   positive slope only, 
	ICR1 = 32000;			// 16 tics / us * 2000us = 32000tics in 2ms.
	TIMSK1 = _BV(ICIE1);

	// Configure Timer3 for measuring injector pulse duration (using interrupt)
	TCCR3A = 0;							// clear control register A 
	TCCR3B = _BV(CS31) | _BV(CS30);		// start the timer at clk/64 prescale. 
	
	TIMSK3 |= _BV(OCIE3A);				// enable output compare A interrupt on timer 3

	// disable the timer 0 interrupt.  This breaks millis() but prevents interference with pulse timing.
	//TIMSK0 &= 0x00;					

	// Configure Timer0 for generating a fast PWM.  16us period, 62.5kHz
	// CONFIG BITS:
	// WGM0<3:0>	= 0 (0000)	normal mode, counts up to 0xFFFF.  
	// COM0A<1:0>	= 2 (10)	clears OC0A on compare match so OCR0A represents "high" time. 
	// COM0B<1:0>	= 2 (10)	same as A.
	// CS0<2:0>		= 1 (001)	1:1 pre-scaling, timer running. 

	attachInterrupt(4, isrTachRisingEdge, RISING);	// interrupt 4 maps to pin 19. 
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
		Serial.print("Air flow: ");
		Serial.println(air_flow);
		break;

	case 'r':
		Serial.print("rpm: ");
		Serial.println(rpm);
		break;

	case 'M':		// MAP update
		receiveMap();
		break;

	case 'm':		// REPORT map
		reportMap();
		break;

	case 'L':		// LOAD map from ee
		loadMapFromEE();
		break;

	case 'S':		// SAVE map
		saveMapToEE();
		break;

	case 'i':
		Serial.print("inj duration: ");
		Serial.println(inj_duration); 
		break;

	case 't':
		reportArray("Task runtimes in us: ", task_runtime, n_tasks);
		break;

	default:
		Serial.println("no comprendo");
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
	unsigned int new_n_air, new_n_rpm;
	unsigned int new_map_size;

	Serial.readBytes(str, 2);

	if (strcmp(str, "ap") != 0)
	{
		Serial.println("spell 'map' please");
		return -1;
	}

	if (!receiveUIntBetween(&new_n_air, MIN_MAP_SIZE, MAX_MAP_SIZE, "air_gridlines"))
		return- 1;

	if (!receiveUIntBetween(&new_n_rpm, MIN_MAP_SIZE, MAX_MAP_SIZE, "rpm_gridlines"))
		return -1;

	if (!receiveUIntArray(new_air_gridline, new_n_air, "air gridline"))
		return -1;
	
	if (!receiveUIntArray(new_rpm_gridline, new_n_rpm, "rpm gridline"))
		return -1;

	new_map_size = new_n_air * new_n_rpm;
	if (!receiveUIntArray(new_engine_map, new_map_size, "map data"))
		return -1;


	n_air = new_n_air;
	n_rpm = new_n_rpm;
	copyArray(new_air_gridline, air_gridline, n_air);
	copyArray(new_rpm_gridline, rpm_gridline, n_rpm);
	copyArray(new_engine_map, engine_map, new_map_size);

	dumpLine();		// dump any additional characters. 

	reportMap();
	
}
void reportMap()
{
	int i;
	reportArray("air_gridline:\n", air_gridline, n_air);
	reportArray("rpm_gridline:\n", rpm_gridline, n_rpm);
	Serial.println("engine_map:");
	for (i = 0; i < n_rpm; ++i)
	{
		reportArray(" ", &engine_map[n_air*i], n_air);
	}
	Serial.print("\n");
}
void reportArray(char str[], unsigned int *data, unsigned int n)
{
	int i;
	Serial.print(str);
	Serial.print("\t");
	for (i = 0; i < n; ++i)
	{
		Serial.print(data[i]);
		Serial.print("\t");
	}
	Serial.println();
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
	// eeprom addresses: 0 = n_air, n_rpm
	unsigned int address, map_size, value;
	address = 0;

	address += EEPROM_readAnything(address, value);
	if (!assignIfBetween(value, n_air, MAX_MAP_SIZE, MIN_MAP_SIZE, "n_air"))
		return;

	address += EEPROM_readAnything(address, value);
	if (!assignIfBetween(value, n_rpm, MAX_MAP_SIZE, MIN_MAP_SIZE, "n_rpm"))
		return;

	address += EEPROM_readAnything(address, air_gridline);
	address += EEPROM_readAnything(address, rpm_gridline);
	address += EEPROM_readAnything(address, engine_map);
	Serial.println("map loaded from EE");
	
}
void saveMapToEE()
{
	// eeprom addresses: 0 = n_air, n_rpm
	unsigned int address, map_size;
	address = 0;

	address += EEPROM_writeAnything(address, n_air);
	address += EEPROM_writeAnything(address, n_rpm);
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
	air_flow = analogRead(air_flow_pin) >> 2;	// takes 100us.  Should shorten
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
void calcRPM()
{
	// (60e6 us/min) / (4 us/tic)				= 15e6 tics/min
	// (tics/min) / (tach_period tics/pulse)	= pulses/min
	// pulses/min * (1 rev/pulse)				= 15e6 / tach_period 
	// we're pulsing the injectors (and measuring) every other tach input pulse, so it's 1:1 with crankshaft. 
	rpm = 15000000 / tach_period;
}

void updateInjDuration()
{
	static int new_inj_duration, accel_offset, cold_eng_offset, air_temp_offset;

	if (areWeCoasting(rpm, air_flow))
	{
		inj_duration = 0;
		return;
	}
	
	// find nominal
	new_inj_duration = interpolateMap(rpm, air_flow); 
	
	// adjust for stuff
	accel_offset = adjustForSuddenAccel(air_flow);		// these should not be order dependent.  
	cold_eng_offset = adjustForColdEngine(new_inj_duration, coolant_temp, air_flow);
	air_temp_offset = adjustForAirTemp(new_inj_duration, air_temp);

	new_inj_duration = new_inj_duration + accel_offset + cold_eng_offset + air_temp_offset;

	// output that shit
	inj_duration = new_inj_duration;
}

unsigned int interpolateMap(unsigned int rpm, unsigned char air)
{
	int i_air, i_rpm;
	int inj_r0a0, inj_r0a1, inj_r1a0, inj_r1a1, inj_r0ak, inj_r1ak, inj_rkak;  // 
	i_air = findIndexJustAbove(air_gridline, air, n_air);
	i_rpm = findIndexJustAbove(rpm_gridline, rpm, n_rpm);

	// check if both i's are -1 (key was higher than all array elements)
	// check if any of them are 0 (key was lower than lowest element)
	// handle those cases
	

	if (i_air > 0)
	{
		if (i_rpm > 0)			// both rpm and air are on the map, so do bilinear interpolation
		{
			inj_r0a0 = engine_map[(i_rpm - 1)*n_air + i_air - 1];	 
			inj_r0a1 = engine_map[(i_rpm - 1)*n_air + i_air];
			inj_r1a0 = engine_map[i_rpm*n_air + i_air - 1];
			inj_r1a1 = engine_map[i_rpm*n_air + i_air];

			inj_r0ak = linearInterp(air, air_gridline[i_air - 1], air_gridline[i_air], inj_r0a0, inj_r0a1);
			inj_r1ak = linearInterp(air, air_gridline[i_air - 1], air_gridline[i_air], inj_r1a0, inj_r1a1);
			return linearInterp(rpm, rpm_gridline[i_rpm - 1], rpm_gridline[i_rpm], inj_r0ak, inj_r1ak);
		}
		else if (i_rpm == 0)	// rpm below the map, air is on the map  (1d interpolate along edge)
		{
			return linearInterp(air, air_gridline[i_air - 1], air_gridline[i_air], mapPoint(0, i_air - 1), mapPoint(0, i_air));
		}
		else 					// rpm above map, air on the map (1d interpolate along edge)
		{
			return linearInterp(air, air_gridline[i_air - 1], air_gridline[i_air], mapPoint(n_rpm - 1, i_air - 1), mapPoint(n_rpm - 1, i_air));
		}
	}
	else if (i_air == 0)	// air below	
	{ 
		if (i_rpm > 0)			// rpm on, air below (1d interpolate along edge)
		{
			return linearInterp(rpm, rpm_gridline[i_rpm - 1], rpm_gridline[i_rpm], mapPoint(i_rpm - 1, 0), mapPoint(i_rpm, 0));
		}
		else if (i_rpm == 0)	// rpm below, air below (corner) 
		{
			return mapPoint(0, 0);
		}
		else					// rpm above, air below (corner) 
		{
			return mapPoint(n_rpm - 1, 0);
		}
	}
	else	// air above
	{
		if (i_rpm > 0)			// rpm on, air above (1d interpolate along edge)
		{
			return linearInterp(rpm, rpm_gridline[i_rpm - 1], rpm_gridline[i_rpm], mapPoint(i_rpm - 1, n_air - 1), mapPoint(i_rpm, n_air - 1));
		}
		else if (i_rpm == 0)	// rpm below, air above (corner) 
		{
			return mapPoint(0, n_air - 1);
		}
		else					// rpm above, air above (corner) 
		{
			return mapPoint(n_rpm - 1, n_air - 1);
		}
	}
}

unsigned int mapPoint(uint8_t i_rpm, uint8_t i_air)
{
	if (i_rpm > n_rpm)
	{
		Serial.println("i_rpm out of bounds");
		return 0;
	}
	if (i_air > n_air)
	{
		Serial.println("i_air out of bounds");
		return 0;
	}

	return engine_map[i_rpm * n_air + i_air];
}
int linearInterp(int x_key, int x1, int x2, int y1, int y2)
{
	float dx;
	int Dx, Dy;
	Dx = x2 - x1;
	dx = x_key - x1; 
	Dy = y2 - y1;
	return y1 + int(Dy*(dx / Dx));
}
int findIndexJustAbove(unsigned int array[], int key, int length)
{		// assumes the input array is sorted high to low. 
	int i;
	for (i = 0; i < length; i++)
	{
		if (array[i] < key)
			return i;
	}
	// didn't find a value above key.  Return -1 to indicate an error. 
	return -1;
}

int adjustForSuddenAccel( int air_flow )
{
	return 0;
}
int adjustForColdEngine( int nominal_duration, int coolant_temp, int air_flow)
{
	static unsigned int adjustment;
	// applies a positive offset to injector duration.  
	// offset depends linearly on coolant temperature.  
	// offset depends linearly on injector duration (works like a % of normal)
	// Parameters: 
	// takes a threshold temp and a rate (mx +b) for linear dependence on  coolant temperature

	if (coolant_temp >= cold_threshold)		// is engine already warm? 
		return 0; 
	adjustment = cold_eng_enrich_rate * (cold_threshold - coolant_temp) * nominal_duration;
	adjustment >>= 6;			// divide by 64; 
	return 0;	// adjustment;		XXX
}
int adjustForAirTemp(int nominal_duration, int air_temp)
{
	return 0;
}
char areWeCoasting( unsigned int rpm, unsigned char air_flow )
{
	if (air_flow < 80)
		return digitalRead(idl_full_pin);
	else
		return 0;
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

ISR( TIMER1_CAPT_vect )
{
	sei();
	TIMSK1 &= ~_BV(TOIE1);		// disable this interrupt while we're in it. (no good to interrupt ourselves)
	static unsigned char i;
	static unsigned int task_start_time = 0; 
	
	for (i = 0; i < n_tasks; i++)
	{
		ms_since_last_task[i] += 2;				// tic eahc task's betwee-calls timer
		if (ms_since_last_task[i] >= ms_freq_of_task[i])	// is it time to call the task?
		{
			ms_since_last_task[i] = 0;			// reset this task's between-calls timer.

			task_start_time = TCNT1 >> 4;		// record start in us
			(*task[i])();						// call task i.
			task_runtime[i] = TCNT1 >> 4;		// record finish in us
			task_runtime[i] -= task_start_time;	// subtract to find runtime in us. 
		}
	}

	TIMSK1 |= _BV(TOIE1);	// this interrup can (and will) interrupt itself here, but we're worried about the reporting, 
							// which is only a problem if we don't get to the part where it stops.  This ensures it does get that far. 
}
void isrTachRisingEdge()
{
	static char pulse_divider = 0;
	if (!pulse_divider)
	{
		GTCCR |= _BV(PSRSYNC);		// clear the prescaler. 
		tach_period = TCNT3; 		// record the timer setting 
		TCNT3 = 0;					// start the timer over. 

		// update the injector turn off timing. 
		OCR3A = inj_duration;

		// set all injectors high (in same instruction)
		PORTL |= 0xAA;				// 0b10101010;
		pulse_divider = 1;
	}
	else
		--pulse_divider;

	//PORTG |= PG1;				// this can be dropped if I move a pin over
}
ISR(TIMER3_COMPA_vect)			// this runs when TCNT3 == OCR3A. 
{
	// set all injectors low (in same instruction)
	PORTL &= 0x55;				// 0b01010101; 
	//PORTG &= ~PG1;
}

