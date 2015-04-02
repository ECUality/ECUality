
#include "ECUality.h"
#include <eeprom.h>
#include "EEPROMAnything.h"
#include "ECUality.h"

#define PARAM_EE_ADR	0
#define MAP_EE_ADR		100
#define OFFSET_EE_ADR	600

#define MAX_MAP_SIZE	10U
#define MIN_MAP_SIZE	3U
#define N_CHOKE_SCALE	4U

// Pin mappings
const uint8_t air_flow_pin = A12;
const uint8_t air_temp_pin = A14;
const uint8_t o2_pin = A9;
const uint8_t coolant_temp_pin = A13;
const uint8_t oil_pressure_pin = A11;
const uint8_t tach_pin = 19;
const uint8_t idl_full_pin = A10;
const uint8_t cranking_pin = 10;

const uint8_t inputs[] = { air_flow_pin, air_temp_pin, o2_pin, coolant_temp_pin, oil_pressure_pin, tach_pin,
	idl_full_pin, cranking_pin, '\0'};

// Output pins
const uint8_t inj2_pin = 42;
const uint8_t inj3_pin = 44;
const uint8_t inj4_pin = 46;
const uint8_t inj1_pin = 48;

const uint8_t coil1_pin = 28;
const uint8_t coil2_pin = 26;
const uint8_t coil3_pin = 24;
const uint8_t coil4_pin = 22;

const uint8_t fuel_pin = A8;
const uint8_t drv_en_pin = 39;
const uint8_t o2_pwr_pin = A1;
const uint8_t cs_knock = 9;
const uint8_t cs_sd = 12;
const uint8_t cs_inj = 36;
const uint8_t inj_led = 25;

uint8_t outputs[] = { inj1_pin, inj2_pin, inj3_pin, inj4_pin, 
	coil1_pin, coil2_pin, coil3_pin, coil4_pin,
	fuel_pin, drv_en_pin, o2_pwr_pin, 
	cs_knock, cs_sd, cs_inj, inj_led, '\0'};


extern int inspect[5] = {};

// alternate paramater data structure
//const unsigned char n_params = 4;
//unsigned int params[n_params] = {0};
//char *param_names[] = { "air_stabilize_rate", "choke_rate", "cold_threshold", "cranking_dur" };

// operating control variables
char good_ee_loads;

unsigned int air_stabilize_rate;		// the rate at which accelerator pump transient decays.
unsigned int cold_threshold;			// the temperature below which enrichment kicks in.  (100F to 150F)
unsigned int cranking_dur;				// the constant duration that gets sent while cranking (500 - 2000) 

// task variables
unsigned int ms_freq_of_task[10];
unsigned int task_runtime[10];
unsigned int ms_since_last_task[10] = { 0 };
void(*task[10]) (void);
unsigned char n_tasks;

// sensor input variables
int air_flow, rpm, o2, air_temp, coolant_temp, oil_pressure;
unsigned int tach_period, inj_duration;

const int temp_scale_x[] = { 563, 454, 356, 278, 208, 157, 116, 88, 66, 52, 41};
const int temp_scale_z[] = {32, 50, 68, 86, 104, 122, 140, 158, 176, 194, 212};
const char n_temp_scale = 11;

int choke_scale_x[] = { 150, 100, 50, 20 };
int choke_scale_z[] = { 20, 200, 700, 1000 };	// these are actually fractions z/1024


// variables that capture dynamic aspects of sensor input
int air_flow_d, air_flow_snap, o2_d;

// map variables
unsigned int n_air, n_rpm;
int air_gridline[MAX_MAP_SIZE], rpm_gridline[MAX_MAP_SIZE];
int engine_map[MAX_MAP_SIZE * MAX_MAP_SIZE];
int correction_map[MAX_MAP_SIZE * MAX_MAP_SIZE];
int map_volatility[MAX_MAP_SIZE * MAX_MAP_SIZE];
int global_offset; 

 //////////////////// pogram ///////////////////
void setup()
{
	Serial.begin(115200);

	task[0] = readAirFlow;			ms_freq_of_task[0] = 50;
	task[1] = readO2Sensor;			ms_freq_of_task[1] = 50;
	task[2] = calcRPM;				ms_freq_of_task[2] = 50;
	task[3] = calcInjDuration;		ms_freq_of_task[3] = 50;
	task[4] = updateInjectors;		ms_freq_of_task[4] = 50;
	task[5] = readAirTemp;			ms_freq_of_task[5] = 250;
	task[6] = readOilPressure;		ms_freq_of_task[6] = 250;
	task[7] = readCoolantTemp;		ms_freq_of_task[7] = 250;
	n_tasks = 8;

	// input interrupt pin
	digitalWrite(cs_knock, HIGH);
	digitalWrite(cs_inj, HIGH);
	digitalWrite(cs_sd, HIGH);

	setPinModes(inputs, INPUT);
	setPinModes(outputs, OUTPUT);
	pinMode(40, INPUT);		// the other hooked-up inj1_pin. 
	//pinMode(drv_en_pin, OUTPUT);

	//pinMode(coil1_pin,)
	//pinMode(13, OUTPUT);

	inj_duration = 5;

	good_ee_loads = 1;
	good_ee_loads &= loadParamsFromEE();
	good_ee_loads &= loadMapFromEE();
	good_ee_loads &= loadOffsetsFromEE();

	// TIMERS
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
void setPinModes(const uint8_t pins[], const uint8_t direction)
{
	char i = 0;
	while (1)
	{
		if (pins[i] == '\0')
			return;
		pinMode(pins[i], direction);
		++i;
	}
}

void loop()
{
	Poll_Serial();
}

// Serial functions
void Poll_Serial()
{
	static char c[10];
	static int gain;

	if (!Serial.available())
		return;

	Serial.readBytes(c, 1);
	//Serial.write(c, 1);		// echo


	switch (c[0])
	{
	case 'a':					// report air flow
		Serial.print("air flow: ");
		Serial.println(air_flow);
		break;

	case 'r':					// report rpm
		Serial.print("rpm: ");
		Serial.println(rpm);
		break;

	case 'o':					// report O2 signal
		Serial.print("O2: ");
		Serial.println(o2);
		break;

	case 't':					// report O2 signal
		Serial.print("coolant temp: ");
		Serial.println(coolant_temp);
		break;

	case 'q':					// report O2 signal
		Serial.print("air temp: ");
		Serial.println(air_temp);
		break;

	case'p':					// preport params
		reportParams();
		break;

	case 'm':					// report map
		reportMap();
		break;

	case 'i':					// report injector duration
		Serial.print("inj duration: ");
		Serial.println(inj_duration);
		break;

	case 'I':					// report Inspection
		reportArray("inspect array", inspect, 5);
		break;

	case 'z':					// report task runtimes
		reportArray("Task runtimes in us: ", task_runtime, n_tasks);
		break;

	case 'x':
		reportOffsets();
		break;

	case 'L':					// increase fuel locally
		localOffset(getGain());
		Serial.print(".");
		break;

	case 'l':					// decrease fuel locally 
		localOffset(-getGain());
		Serial.print(".");
		break;

	case 'G':					// increase fuel globally 
		global_offset += getGain();
		Serial.print(".");
		break;

	case 'g':					// decrease fuel globally
		global_offset -= getGain();
		Serial.print(".");
		break;

	case 'C':					// clear all local offsets
		clearArray(correction_map, MAX_MAP_SIZE*MAX_MAP_SIZE);
		break;
		
	case 'c':					// clear global offset
		global_offset = 0;
		break;

	case 'P':					// Parameter update
		receiveParams();
		break;

	case 'M':					// MAP update
		receiveMap();
		break;

	case 'S':					// SAVE params, map to EE
		saveParamsToEE();
		saveMapToEE();
		break;

	case'E':					// Load params, map, offsets from EEPROM
		loadParamsFromEE();
		loadMapFromEE();
		loadOffsetsFromEE();
		break;

	case 'O':
		saveOffsetsToEE();
		break;

	case '+':
		disableDrive();
		break;

	case '-':
		enableDrive();
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
int getGain()		// this is just the number of characters before the newline '\n' character times a constant (16)
{
	int gain = 1; 
	char c[10];
	gain += Serial.readBytesUntil('\n', &c[1], 9);
	gain <<= 4;		// multiply by 16
	return gain;
}

// Serial to data functions
char receiveMap()
{
	char str[3] = "";	// all zeros.
	int new_engine_map[MAX_MAP_SIZE * MAX_MAP_SIZE];
	int new_air_gridline[MAX_MAP_SIZE];
	int new_rpm_gridline[MAX_MAP_SIZE];
	unsigned int new_n_air, new_n_rpm;
	unsigned int new_map_size;

	Serial.readBytes(str, 2);

	if (strcmp(str, "ap") != 0)
	{
		Serial.println("spell 'map' please");
		return -1;
	}

	if (!receiveNumberBetween(&new_n_air, MIN_MAP_SIZE, MAX_MAP_SIZE, "air_gridlines"))
		return-1;

	if (!receiveNumberBetween(&new_n_rpm, MIN_MAP_SIZE, MAX_MAP_SIZE, "rpm_gridlines"))
		return -1;

	if (!receiveArray(new_air_gridline, new_n_air, "air gridline"))
		return -1;

	if (!receiveArray(new_rpm_gridline, new_n_rpm, "rpm gridline"))
		return -1;

	new_map_size = new_n_air * new_n_rpm;
	if (!receiveArray(new_engine_map, new_map_size, "map data"))
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
char receiveParams()
{
	char str[3] = "";	// all zeros.

	unsigned int new_air_stabilize_rate;		// the rate at which accelerator pump transient decays.
	unsigned int new_cold_threshold;			// the temperature below which enrichment kicks in. 
	unsigned int new_cranking_dur;				// the constant duration that gets sent while cranking
	int new_choke_scale_x[N_CHOKE_SCALE];
	int new_choke_scale_z[N_CHOKE_SCALE];


	//receiveArray(params, n_params, "param array");

	if (!receiveNumberBetween(&new_air_stabilize_rate, 0, 200, "air_stabilize_rate"))
		return-1;
	if (!receiveNumberBetween(&new_cold_threshold, 90, 195, "cold_threshold"))
		return-1;
	if (!receiveNumberBetween(&new_cranking_dur, 500, 2000, "cranking_dur"))
		return-1;

	if (!receiveArray(new_choke_scale_x, N_CHOKE_SCALE, "choke_scale_x"))
		return -1;
	if (!receiveArray(new_choke_scale_z, N_CHOKE_SCALE, "choke_scale_z"))
		return -1;

	air_stabilize_rate = new_air_stabilize_rate;
	cold_threshold = new_cold_threshold;
	cranking_dur = new_cranking_dur;
	copyArray(new_choke_scale_x, choke_scale_x, N_CHOKE_SCALE);
	copyArray(new_choke_scale_z, choke_scale_z, N_CHOKE_SCALE);

	dumpLine();			// dump any additional characters. 
	reportParams();
}
void reportParams()
{
	Serial.print("air_stabilize_rate: ");
	Serial.print(air_stabilize_rate);
	Serial.print("   cold_threshold: ");
	Serial.print(cold_threshold);
	Serial.print("   cranking_dur: ");
	Serial.println(cranking_dur);

	reportArray("choke scale Temps:  ", choke_scale_x, N_CHOKE_SCALE);
	reportArray("choke scale values: ", choke_scale_z, N_CHOKE_SCALE);

}
void reportOffsets()
{
	int i;

	Serial.print("global_offset: ");
	Serial.println(global_offset);

	for (i = 0; i < n_rpm; ++i)
	{
		reportArray(" ", &correction_map[n_air*i], n_air);
	}
	Serial.println();
}

// EE access functions
char loadParamsFromEE()
{
	unsigned int address;
	address = PARAM_EE_ADR;

	unsigned int new_air_stabilize_rate;		// the rate at which accelerator pump transient decays.
	unsigned int new_cold_threshold;			// the temperature below which enrichment kicks in. 
	unsigned int new_cranking_dur;				// the constant duration that gets sent while cranking
	int new_choke_scale_x[N_CHOKE_SCALE];
	int new_choke_scale_z[N_CHOKE_SCALE];

	address += EEPROM_readAnything(address, new_air_stabilize_rate);	// 2 bytes unsigned int
	address += EEPROM_readAnything(address, new_cold_threshold);		// 2 bytes unsigned int
	address += EEPROM_readAnything(address, new_cranking_dur);			// 2 bytes unsigned int	
	address += EEPROM_readAnything(address, new_choke_scale_x);				// 8 bytes int x4
	address += EEPROM_readAnything(address, new_choke_scale_z);				// 8 bytes int x4		TOTAL: 22 of 32

	char good = 1;
	good = good && (new_air_stabilize_rate >= 0) && (new_air_stabilize_rate <= 100);
	good = good && (new_cold_threshold >= 90) && (new_cold_threshold <= 195);
	good = good && (new_cranking_dur >= 500) && (new_cranking_dur <= 2000);
	good = good && (new_choke_scale_x[0] >= new_cold_threshold) && (new_choke_scale_x[0] <= 230);
	good = good && (new_choke_scale_z[N_CHOKE_SCALE - 1] >= 200) && (new_choke_scale_z[N_CHOKE_SCALE - 1] <= 2000);
	
	if (!good)
	{
		Serial.println("invalid parameter.");
		return 0;
	}

	air_stabilize_rate = new_air_stabilize_rate;
	cold_threshold = new_cold_threshold;
	cranking_dur = new_cranking_dur;
	copyArray(new_choke_scale_x, choke_scale_x, N_CHOKE_SCALE);
	copyArray(new_choke_scale_z, choke_scale_z, N_CHOKE_SCALE);

	Serial.println("loaded params from EE.");
	return 1;
}
void saveParamsToEE()
{
	unsigned int address;
	address = PARAM_EE_ADR;

	address += EEPROM_writeAnything(address, air_stabilize_rate);	// 2 bytes unsigned char
	address += EEPROM_writeAnything(address, cold_threshold);		// 2 bytes unsigned int
	address += EEPROM_writeAnything(address, cranking_dur);			// 2 bytes unsigned int	
	address += EEPROM_writeAnything(address, choke_scale_x);		// 8 bytes int x4
	address += EEPROM_writeAnything(address, choke_scale_z);		// 8 bytes int x4		TOTAL: 22 of 32
	Serial.println("saved params to EE.");
}
char loadMapFromEE()
{
	// eeprom addresses: 0 = n_air, n_rpm
	unsigned int address, value;
	address = MAP_EE_ADR; //XXX

	address += EEPROM_readAnything(address, value);				// 2 bytes
	if (!assignIfBetween(value, n_air, MAX_MAP_SIZE, MIN_MAP_SIZE, "n_air"))
	{
		detachInterrupt(4);
		return 0;
	}

	address += EEPROM_readAnything(address, value);				// 2 bytes
	if (!assignIfBetween(value, n_rpm, MAX_MAP_SIZE, MIN_MAP_SIZE, "n_rpm"))
	{
		detachInterrupt(4);
		return 0;
	}

	address += EEPROM_readAnything(address, air_gridline);		// 20 bytes
	address += EEPROM_readAnything(address, rpm_gridline);		// 20 bytes
	address += EEPROM_readAnything(address, engine_map);		// 200 bytes  TOTAL: 244 of 496
	Serial.println("loaded map from EE.");
	return 1;
}
void saveMapToEE()
{
	// eeprom addresses: 0 = n_air, n_rpm
	unsigned int address, map_size;
	address = MAP_EE_ADR;

	address += EEPROM_writeAnything(address, n_air);
	address += EEPROM_writeAnything(address, n_rpm);
	address += EEPROM_writeAnything(address, air_gridline);
	address += EEPROM_writeAnything(address, rpm_gridline);
	address += EEPROM_writeAnything(address, engine_map);

	Serial.println("saved map to EE");
}
char loadOffsetsFromEE()
{
	unsigned int address;
	address = OFFSET_EE_ADR;

	address += EEPROM_readAnything(address, global_offset);
	address += EEPROM_readAnything(address, correction_map);
	return 1;
}
void saveOffsetsToEE()
{
	unsigned int address;
	address = OFFSET_EE_ADR;

	address += EEPROM_writeAnything(address, global_offset);
	address += EEPROM_writeAnything(address, correction_map);
	Serial.println("offsets saved.");
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
	//o2_d = -o2;
	o2 = analogRead(o2_pin);		// this takes 100us.  Should shorten
	o2 *= 5; 
	//o2_d += o2;
}
void readAirTemp() 
{
	air_temp = analogRead(air_temp_pin);
	air_temp = scaleRead(air_temp, temp_scale_x, temp_scale_z, n_temp_scale);
}
void readCoolantTemp() 
{
	coolant_temp = analogRead(coolant_temp_pin);
	coolant_temp = scaleRead(coolant_temp, temp_scale_x, temp_scale_z, n_temp_scale);
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

// Pulse duration business
void calcInjDuration()
{
	static unsigned int new_inj_duration;
	static int accel_offset, choke_offset, air_temp_offset;

	if (digitalRead(cranking_pin))
	{
		choke_offset = adjustForColdEngine(cranking_dur, coolant_temp);
		inj_duration = cranking_dur + choke_offset;
		return;
	}
	if (areWeCoasting(rpm, air_flow))
	{
		inj_duration = 0;
		return;
	}
	
	// find nominal
	new_inj_duration = interpolateMap(rpm, air_flow); 
	
	// adjust for stuff
	accel_offset = adjustForSuddenAccel(air_flow);		// these should not be order dependent.  
	choke_offset = adjustForColdEngine(new_inj_duration, coolant_temp);
	air_temp_offset = adjustForAirTemp(new_inj_duration, air_temp);

	new_inj_duration = new_inj_duration + accel_offset + choke_offset + air_temp_offset;

	// output that shit
	inj_duration = new_inj_duration;
}
int adjustForSuddenAccel(int air_flow)
{
	return 0;
}
int adjustForColdEngine(unsigned int nominal_duration, int coolant_temp)
{
	// calculates a positive offset as a percentage of nominal injector duration. 
	// % increase depends linearly on coolant temperature.  
	// takes a threshold temp and a rate (mx +b) for linear dependence on  coolant temperature
	unsigned long adjustment;
	
	if (coolant_temp >= cold_threshold)		// is engine already warm? 
		return 0;

	adjustment = scaleRead(coolant_temp, choke_scale_x, choke_scale_z, N_CHOKE_SCALE);
	adjustment *= nominal_duration;
	adjustment >>= 10;		// divide by 1024
	return int(adjustment);	// adjustment;		XXX
}
int adjustForAirTemp(int nominal_duration, int air_temp)
{
	return 0;
}
char areWeCoasting(unsigned int rpm, unsigned char air_flow)
{
	return (!digitalRead(idl_full_pin) && (air_flow < 80) && (rpm > 1200) );
}
void updateInjectors()
{
	if (inj_duration > 10)
		OCR3A = inj_duration;		// normal operation
	else
		OCR3A = 10;					// if inj_duration = 0, we just drop it. 

	// if we just set jumped the compare register below the timer, should stop the injector 
	// because if we don't the timer will have to wrap all the way around before it stops. 
	if (inj_duration <= TCNT3)		
	{
		// turn off the injectors
		PORTL &= ~0xAA;			// 0b01010101; 
		PORTA &= ~0x08;				// 
	}
}

// map interpolation
unsigned int interpolateMap(unsigned int rpm, unsigned char air)
{
	int i_air, i_rpm;
	int inj_r0a0, inj_r0a1, inj_r1a0, inj_r1a1, inj_r0ak, inj_r1ak, inj_rkak;  // 
	i_air = findIndexJustAbove(air_gridline, air, n_air);
	i_rpm = findIndexJustAbove(rpm_gridline, rpm, n_rpm);

	// check if both i's are -1 (key was higher than all array elements)
	// check if any of them are 0 (key was lower than lowest element)
	// handle those cases
	//Serial.print("r");
	//Serial.print(i_rpm);
	//Serial.print("a");
	//Serial.println(i_air);

	if (i_air > 0)
	{
		if (i_rpm > 0)			// both rpm and air are on the map, so do bilinear interpolation
		{
			inj_r0a0 = mapPoint(i_rpm - 1, i_air - 1);
			inj_r0a1 = mapPoint(i_rpm - 1, i_air);
			inj_r1a0 = mapPoint(i_rpm, i_air - 1);
			inj_r1a1 = mapPoint(i_rpm, i_air);

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
unsigned int mapPoint(char i_rpm, char i_air)
{
	static int raw, local;
	if ((i_rpm >= n_rpm) || (i_rpm < 0) || (i_air >= n_air) || (i_air < 0) )
	{
		//Serial.print("ir: ");
		//Serial.print(int(i_rpm));
		//Serial.print("   ia: ");
		//Serial.println(int(i_air));
		return 0;
	}
	raw = engine_map[i_rpm * n_air + i_air];
	local = correction_map[i_rpm * n_air + i_air];
	return (raw + local + global_offset);
}
int linearInterp(int x_key, int x1, int x2, int y1, int y2)
{
	int dx, Dx;
	long Dy;
	Dx = x2 - x1;
	dx = x_key - x1; 
	Dy = y2 - y1;

	return y1 + (Dy*dx)/Dx;
}
char findIndexJustAbove(const int array[], int key, int length)
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

// map modification
void localOffset(long offset)
{
	// here, we apply an offset to the map correction in the locale of our current operation (air_flow and rpm) 
	char i_air, i_rpm;
	int D_rpm, D_air, d_rpm_lowside, d_air_rightside;
	long dz_r0, dz_r1;
	int dz_r0a0, dz_r0a1, dz_r1a0, dz_r1a1;
		
	// find the location on the map.
	i_air = findIndexJustAbove(air_gridline, air_flow, n_air);
	i_rpm = findIndexJustAbove(rpm_gridline, rpm, n_rpm);

	if ((i_air <= 0) || (i_rpm <= 0))		// if we're not operating on the map, fuggedaboutit
		return;								// note: -1 means we're above, 0 means we're below so <= 0 captures both

	// 	offset gets divided into 4 the 4 nearest grid intersections based on where we're currently operating. 
	//  more of the offset goes to the intersections we're closest to. 

	D_rpm = rpm_gridline[i_rpm - 1] - rpm_gridline[i_rpm];	// positive
	D_air = air_gridline[i_air - 1] - air_gridline[i_air];	// positive

	d_rpm_lowside = rpm - rpm_gridline[i_rpm];			// delta above lower neighbors
	d_air_rightside = air_flow - air_gridline[i_air];		// delta to left of neighbors. 

	if ((d_rpm_lowside * 2) > D_rpm)		// closer to the upper neighbors.  Use lower delta for calculations.
	{
		dz_r0 = (offset * d_rpm_lowside) / D_rpm;
		dz_r1 = offset - dz_r0;
	}
	else									// closer to the lower neighbors.  Use upper delta for calculations.
	{
		dz_r1 = (offset * (D_rpm - d_rpm_lowside)) / D_rpm;
		dz_r0 = offset - dz_r1;
	}

	if ((d_air_rightside * 2) > D_air)		// closer to the left neighbors, use right side deltas (larger) 
	{
		dz_r0a0 = (dz_r0 * d_air_rightside) / D_air;
		dz_r0a1 = dz_r0 - dz_r0a0;

		dz_r1a0 = (dz_r1 * d_air_rightside) / D_air;
		dz_r1a1 = dz_r1 - dz_r1a0;
	}
	else									// closer to right neighbors, use left side deltas (larger) 
	{
		dz_r0a1 = (dz_r0 * (D_air - d_air_rightside)) / D_air;
		dz_r0a0 = dz_r0 - dz_r0a1;

		dz_r1a1 = (dz_r1 * (D_air - d_air_rightside)) / D_air;
		dz_r1a0 = dz_r1 - dz_r1a1;
	}

	editMapPoint(i_rpm - 1, i_air - 1,	dz_r0a0);
	editMapPoint(i_rpm - 1, i_air,		dz_r0a1);
	editMapPoint(i_rpm,		i_air - 1,	dz_r1a0);
	editMapPoint(i_rpm,		i_air,		dz_r1a1);
}
void editMapPoint(char i_rpm, char i_air, int offset)
{
	if ((i_rpm >= n_rpm) || (i_rpm < 0) || (i_air >= n_air) || (i_air < 0))
	{
		Serial.println("index out of bounds");
		return;
	}
	correction_map[n_air*i_rpm + i_air] += offset; 
}

// analog reading scaling
int scaleRead(int x, const int x_array[], const int y_array[], char n)
{
	char i;
	i = findIndexJustAbove(x_array, x, n);

	if (i > 0)	// reading is on the graph
		return linearInterp(x, x_array[i-1], x_array[i], y_array[i-1], y_array[i]);
	
	else if (i == 0)	// x is to left of graph.  Use leftmost points
		return linearInterp(x, x_array[0], x_array[1], y_array[0], y_array[1]);

	else				// x is to right of graph. Use rightmost points.
		return linearInterp(x, x_array[n-1], x_array[n], y_array[n-1], y_array[n]);
}

// important commands
void enableDrive()
{
	if (good_ee_loads)
	{
		digitalWrite(fuel_pin, HIGH);
		digitalWrite(drv_en_pin, LOW);
		//attachInterrupt(4, isrTachRisingEdge, RISING);	// interrupt 4 maps to pin 19. 
		Serial.println("Armed.");
	}
	else
		Serial.println("bad EE data, no go.");
}
void disableDrive()
{
	digitalWrite(fuel_pin, LOW);
	digitalWrite(drv_en_pin, HIGH);		// this turns off the injectors and ignition
	//detachInterrupt(4);
	Serial.println("inj, fuel disabled.");
}

// simple stuff
void toggle(unsigned char pin) 
{
	// Toggle LED
	digitalWrite(pin, digitalRead(pin) ^ 1);
}

// interrupt routines
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
	static char pulse_divider = 0;	// the actions happen every other pulse.  We use pulse_divider for that.
	if (!pulse_divider)
	{
		GTCCR |= _BV(PSRSYNC);		// clear the prescaler. 
		tach_period = TCNT3; 		// record the timer setting 
		TCNT3 = 0;					// start the timer over. 

		// set all injectors high (in same instruction)
		if (inj_duration)
		{
			PORTL |= 0xAA;				// 0b10101010;
			PORTA |= 0x08;				// 0b00001000  (A3 is turned on) 
		}
		pulse_divider = 1;
	}
	else
		--pulse_divider;
}
ISR(TIMER3_COMPA_vect)			// this runs when TCNT3 == OCR3A. 
{
	// set all injectors low (in same instruction)
	PORTL &= ~0xAA;			// 0b01010101; 
	PORTA &= ~0x08;				// 
}
