
#include "Arrays.h"
#include "Map.h"
#include "Scale.h"
#include "Parameter.h"
#include "ECUSerial.h"
#include "Interpolation.h"
#include "EEIndex.h"
#include <eeprom.h>
#include "EEPROMAnything.h"

#define PARAM_EE_ADR	0
#define MAP_EE_ADR		100
#define OFFSET_EE_ADR	600

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
const uint8_t drv_en_pin = 38;
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

// task variables
unsigned int ms_freq_of_task[20];
unsigned int task_runtime[20];
unsigned int ms_since_last_task[20] = { 0 };
void(*task[20]) (void);
unsigned char n_tasks;

// sensor input variables
int air_flow, rpm, o2, air_temp, coolant_temp, oil_pressure;
unsigned int tach_period, inj_duration;

// variables that capture dynamic aspects of sensor input
int air_flow_d, air_flow_snap, o2_d;

// EE-Backed Parameters //////////////////////////////////

//unsigned int air_stabilize_rate;		// the rate at which accelerator pump transient decays.
//unsigned int cold_threshold;			// the temperature below which enrichment kicks in.  (100F to 150F)
//unsigned int cranking_dur;				// the constant duration that gets sent while cranking (500 - 2000) 

//const int temp_scale_x[] = { 563, 454, 356, 278, 208, 157, 116, 88, 66, 52, 41};
//const int temp_scale_z[] = {32, 50, 68, 86, 104, 122, 140, 158, 176, 194, 212};
//const char n_temp_scale = 11;

//int choke_scale_x[] = { 150, 100, 50, 20 };
//int choke_scale_z[] = { 20, 200, 700, 1000 };	// these are actually fractions z/1024


// map variables
//unsigned int n_air, n_rpm;
//int air_gridline[MAX_MAP_SIZE], rpm_gridline[MAX_MAP_SIZE];
//int engine_map[MAX_MAP_SIZE * MAX_MAP_SIZE];
//int correction_map[MAX_MAP_SIZE * MAX_MAP_SIZE];
//int map_volatility[MAX_MAP_SIZE * MAX_MAP_SIZE];

Parameter air_stabilize_rate(5, 500);
Parameter cold_threshold (80, 190); 
Parameter cranking_dur (800, 3000);
Scale choke_scale (4);
Scale temp_scale(11);
Scale air_rpm_scale (8);
Map inj_map (&air_rpm_scale);
Map correction_map (&air_rpm_scale);

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
	task[6] = readCoolantTemp;		ms_freq_of_task[6] = 250;
	task[7] = readOilPressure;		ms_freq_of_task[7] = 250;
	task[8] = statusReport;			ms_freq_of_task[8] = 250;
	n_tasks = 9;

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
	loadData();
	
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
		Map::report(&inj_map);
		break;

	case 'i':					// report injector duration
		Serial.print("inj duration: ");
		Serial.println(inj_duration);
		break;

	case 'I':					// report Inspection
		ESerial.reportArray("inspect array", inspect, 5);
		break;

	case 'z':					// report task runtimes
		ESerial.reportArray("Task runtimes in us: ", task_runtime, n_tasks);
		break;

	case 'x':
		Serial.print("Global offset: ");
		Serial.println(global_offset);
		Map::report(&correction_map);
		break;

	case 'L':					// increase fuel locally
		inj_map.localOffset(rpm, air_flow, getGain());
		Serial.print(".");
		break;

	case 'l':					// decrease fuel locally 
		inj_map.localOffset(rpm, air_flow, -getGain());
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
		Map::clear(&correction_map);
		break;
		
	case 'c':					// clear global offset
		global_offset = 0;
		break;

	case 'v':					// Parameter update
		Parameter::receive(&cold_threshold);
		break;

	case 'b':
		Parameter::receive(&cranking_dur);
		break;

	case 'n':
		Scale::receive(&temp_scale);
		break;

	case 'u':
		Scale::receive(&choke_scale);
		break;

	case 'h':
		Scale::receive(&air_rpm_scale);
		break;

	case 'M':					// MAP update
		Map::receive(&inj_map);
		break;

	case 'S':					// SAVE params, map to EE
		saveData();
		break;

	case'E':					// Load params, map, offsets from EEPROM
		loadData();
		break;

	case 'O':
		Map::save(&correction_map);
		break;

	case '+':
		disableDrive();
		break;

	case '-':
		enableDrive();
		break;

	case 'e':
		ESerial.reportArray("EE addresses: ", EE_index.addresses, MAX_EE_ADDRESSES);

	default:
		Serial.println("no comprendo");
	}
	ESerial.dumpLine();
}

int getGain()		// this is just the number of characters before the newline '\n' character times a constant (16)
{
	int gain = 1; 
	char c[10];
	gain += Serial.readBytesUntil('\n', &c[1], 9);
	gain <<= 4;		// multiply by 16
	return gain;
}
void statusReport()
{
	if (digitalRead(cranking_pin))
	{
		Serial.print(" ");
		Serial.print(inj_duration);
	}
}
const char loadData()
{
	char good = 1;
	good &= Parameter::load(&cranking_dur);
	good &= Parameter::load(&cold_threshold);
	good &= Scale::load(&choke_scale);
	good &= Scale::load(&temp_scale);
	good &= Scale::load(&air_rpm_scale);
	good &= Map::load(&inj_map);
	good &= Map::load(&correction_map);

	if (!good)
		Serial.println(F("not all data loaded"));
	return good; 
}
void saveData()
{
	Parameter::save(&cranking_dur);
	Parameter::save(&cold_threshold);
	Scale::save(&choke_scale);
	Scale::save(&temp_scale);
	Scale::save(&air_rpm_scale);
	Map::save(&inj_map);
	Map::save(&correction_map);
}
void reportParams()
{
	Serial.println(F("cold threshold, cranking duration : "));
	Parameter::report(&cold_threshold);
	Parameter::report(&cranking_dur);
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
		if (air_flow_snap > air_stabilize_rate.value)
			air_flow_snap -= air_stabilize_rate.value;
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
	air_temp = temp_scale.interpolate(air_temp);
}
void readCoolantTemp() 
{
	coolant_temp = analogRead(coolant_temp_pin);
	coolant_temp = temp_scale.interpolate(coolant_temp);
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
		choke_offset = adjustForColdEngine(cranking_dur.value, coolant_temp);
		inj_duration = cranking_dur.value + choke_offset;
		return;
	}
	if (areWeCoasting(rpm, air_flow))
	{
		inj_duration = 0;
		return;
	}
	
	// find nominal
	new_inj_duration = inj_map.interpolate(rpm, air_flow, &correction_map); 
	
	// adjust for stuff
	accel_offset = adjustForAccel(air_flow);		// these should not be order dependent.  
	choke_offset = adjustForColdEngine(new_inj_duration, coolant_temp);
	air_temp_offset = adjustForAirTemp(new_inj_duration, air_temp);

	new_inj_duration = new_inj_duration + accel_offset + choke_offset + air_temp_offset;

	// output that shit
	inj_duration = new_inj_duration;
}
int adjustForAccel(int air_flow)
{
	return 0;
}
int adjustForColdEngine(unsigned int nominal_duration, int coolant_temp)
{
	// calculates a positive offset as a percentage of nominal injector duration. 
	// % increase depends linearly on coolant temperature.  
	// takes a threshold temp and a rate (mx +b) for linear dependence on  coolant temperature
	unsigned long adjustment;
	
	if (coolant_temp >= cold_threshold.value)		// is engine already warm? 
		return 0;

	adjustment = choke_scale.interpolate(coolant_temp);
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
ISR( TIMER1_CAPT_vect )			// this is the scheduler interrupt 
{
	sei();

	// Some of these tasks take a while, and they all run within this interrupt's scope.
	// The timer might have enough time to reach the another input compare match, which 
	// would trigger another of these interrupts.  We have to keep global interrupts enabled
	// so the tach pulses CAN interrupt this interrupt, but if the task scheduler interrupts 
	// itself before finishing, we enter an endless loop and crash the stack.   
	// So we turn off the TIMER1 interrupts while we're in this ISR to prevent this. 
	TIMSK1 = 0;		
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

	// We re-enable this interrupt now that we've gotten through the task list. 
	// as we do this, this interrupt may interrupt itself here, but 
	// all the tasks we were doing have had their "ms_since_last_task[i]"s cleared, 
	// so we'll blaze through the list and exit both nested ISRs. 
	TIMSK1 |= _BV(ICIE1);	
							
					
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
