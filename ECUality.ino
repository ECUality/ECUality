
#include "Defines.h"
#include "Arrays.h"
#include "Map.h"
#include "Scale.h"
#include "Parameter.h"
#include "ECUSerial.h"
#include "Interpolation.h"
#include "EEIndex.h"
#include <eeprom.h>
#include "EEPROMAnything.h"
#include "ECUPins.h"
#include "FuelTweaker.h"
#include "MovingAverage.h"


// operating control variables
char good_ee_loads;
uint8_t auto_stat;
unsigned char run_condition;

// task variables
unsigned int ms_freq_of_task[20];
unsigned int task_runtime[20];
unsigned int ms_since_last_task[20] = { 0 };
void(*task[20]) (void);
unsigned char n_tasks;

// sensor input variables
int air_flow, rpm, o2, air_temp, coolant_temp, oil_pressure;
unsigned int tach_period, inj_duration;
MovingAverage avg_rpm(4);

// variables that capture dynamic aspects of sensor input
int air_flow_d, air_flow_snap, o2_d;

// EE-Backed Parameters //////////////////////////////////

//int choke_scale_x[] = { 150, 100, 50, 20 };
//int choke_scale_z[] = { 20, 200, 700, 1000 };	// these are actually fractions z/1024


Parameter global_offset("global_offset",-1000, 1000);
Parameter coasting_rpm("coasting_rpm", 800, 2500);
Parameter idling_rpm("idling_rpm", 400, 2200);
Parameter air_thresh("air_thresh", 50, 125);
Parameter accel_stabilize_rate("accel_stabilize_rate", 5, 500);		// the rate at which accelerator pump transient decays.
Parameter cold_threshold ("cold_thresh", 80, 190);			// the temperature below which enrichment kicks in.  (100F to 150F)
Parameter cranking_dur ("cranking_dur", 800, 3000);			// the injector duration while cranking not counting choke (500 - 2000) 
Scale choke_scale (-40, 200, 0, 2000, 0);	// scales engine temperature to "per 1024" (think percent) increase of injector itme
Scale temp_scale (0, 1023, -40, 250, 12);	// scales measured voltage on temp sensors to actual temp. in F
Scale air_rpm_scale (0, 255, 200, 6000, 8);	// the x and y gridlines (air-flow and rpm) for the injector map.  (not a scaling function) 
Map inj_map (&air_rpm_scale, 8, 400, 3000);	// the 2d map defining injector durations with respect to air-flow and rpm
Map offset_map (&air_rpm_scale, 8, -1500, 1500); // local modifications to the inj_map, applied by the optimizer. 
Map change_map (&air_rpm_scale, 8, -1500, 1500);	// map that contains the changes made since power-up. 
FuelTweaker boss(run_condition, air_flow, rpm, avg_rpm.average, o2, global_offset.value, offset_map, change_map);


 //////////////////// pogram ///////////////////
void setup()
{
	ESerial.begin(9600);
	//Serial.begin(115200);

	task[0] = readAirFlow;			ms_freq_of_task[0] = 50;
	task[1] = readO2Sensor;			ms_freq_of_task[1] = 50;
	task[2] = calcRPM;				ms_freq_of_task[2] = 50;
	task[3] = calcInjDuration;		ms_freq_of_task[3] = 50;
	task[4] = updateInjectors;		ms_freq_of_task[4] = 50;
	task[5] = readAirTemp;			ms_freq_of_task[5] = 250;
	task[6] = readCoolantTemp;		ms_freq_of_task[6] = 250;
	task[7] = readOilPressure;		ms_freq_of_task[7] = 250;
	task[8] = autoReport;			ms_freq_of_task[8] = 1000;
	task[9] = updateRunCondition;	ms_freq_of_task[9] = 100;
	task[10] = tweakFuel;			ms_freq_of_task[10] = TWEAK_FREQ_MS;
	n_tasks = 11;

	// input interrupt pin
	digitalWrite(cs_knock_pin, HIGH);
	digitalWrite(cs_inj_pin, HIGH);
	digitalWrite(cs_sd_pin, HIGH);

	setPinModes(inputs, INPUT);
	setPinModes(outputs, OUTPUT);
	pinMode(40, INPUT);		// the other hooked-up inj1_pin. 
	//pinMode(drv_en_pin, OUTPUT);

	//pinMode(coil1_pin,)
	//pinMode(13, OUTPUT);

	inj_duration = 5;
	
	Map::clear(&change_map);
	good_ee_loads = loadData(NULL);		// load data from EE.

	initProtocol();						// set up protocol. 
	
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
	TIMSK3 |= _BV(TOIE3);

	// disable the timer 0 interrupt.  This breaks millis() but prevents interference with pulse timing.
	//TIMSK0 &= 0x00;					

	// Configure Timer0 for generating a fast PWM.  16us period, 62.5kHz
	// CONFIG BITS:
	// WGM0<3:0>	= 0 (0000)	normal mode, counts up to 0xFFFF.  
	// COM0A<1:0>	= 2 (10)	clears OC0A on compare match so OCR0A represents "high" time. 
	// COM0B<1:0>	= 2 (10)	same as A.
	// CS0<2:0>		= 1 (001)	1:1 pre-scaling, timer running. 

	attachInterrupt(4, isrTachRisingEdge, RISING);	// interrupt 4 maps to pin 19. 
	enableDrive(NULL);
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
	/*if (Serial.available())
		Serial3.write(Serial.read());
	if (Serial3.available())
		Serial.write(Serial3.read());*/
	if (auto_stat == 2)
	{
		auto_stat = 1;
		reportStatus(NULL);
	}

	if (Serial3.available())
	{
		ESerial.executeCommand();
		Serial3.print(auto_stat);
	}
	
}


// ESerial functions
const char saveData(void* obj_ptr)
{
	Parameter::save(&coasting_rpm);
	Parameter::save(&idling_rpm);
	Parameter::save(&air_thresh);
	Parameter::save(&cold_threshold);
	Parameter::save(&cranking_dur);
	Scale::save(&choke_scale);
	Scale::save(&temp_scale);
	Scale::save(&air_rpm_scale);
	Map::save(&inj_map);
}
const char loadData(void* obj_ptr)
{
	char good = 1;
	good &= Parameter::load(&coasting_rpm);
	good &= Parameter::load(&idling_rpm);
	good &= Parameter::load(&air_thresh);
	good &= Parameter::load(&cold_threshold);
	good &= Parameter::load(&cranking_dur);
	good &= Parameter::load(&global_offset);
	good &= Scale::load(&choke_scale);
	good &= Scale::load(&temp_scale);
	good &= Scale::load(&air_rpm_scale);
	good &= Map::load(&inj_map);
	good &= Map::load(&offset_map);
	good &= FuelTweaker::load(&boss);

	if (!good)
		ESerial.println(F("not all data loaded"));
	else
		ESerial.println(F("all data loaded"));
	return good;
}

const char reportStatus(void* obj_ptr)
{
	ESerial.print(F("air: "));
	ESerial.print(air_flow);
	ESerial.print(F("  avg_rpm: "));
	ESerial.print(avg_rpm.average);
	ESerial.print(F("  inj: "));
	ESerial.print(inj_duration);
	ESerial.print(F("  O2: "));
	ESerial.print(o2);
	ESerial.print(F("  temp: "));
	ESerial.print(coolant_temp);
	ESerial.print(F("  glo_offset: "));
	ESerial.print(global_offset.value);
	ESerial.print("  bossmode: ");
	ESerial.println(boss.mode);
}
const char reportMode(void* obj_ptr)
{
	if (run_condition & _BV(NOT_RUNNING))
		ESerial.print(F("not running,  "));
	else if (run_condition & _BV(CRANKING))
		ESerial.print(F("cranking,  "));
	else if (run_condition & _BV(COASTING))
		ESerial.print(F("coasting,  "));
	else if (run_condition & _BV(IDLING))
		ESerial.print(F("idling,  "));
	else if (run_condition & _BV(WIDE_OPEN))
		ESerial.print(F("hopefully hauling ass,  "));
	else
		ESerial.print(F("pulling,  "));

	if (run_condition & _BV(WARM))
		ESerial.println(F("warm"));
	else
		ESerial.println(F("cold"));
}
const char reportParams(void* obj_ptr)
{
	Parameter::read(&coasting_rpm);
	Parameter::read(&idling_rpm);
	Parameter::read(&air_thresh);
	Parameter::read(&cold_threshold);
	Parameter::read(&cranking_dur);
}
const char reportEEAddresses(void* obj_ptr)
{
	ESerial.reportArray("EE addresses: ", EE_index.addresses, MAX_EE_ADDRESSES);
}
const char reportTaskTimes(void* obj_ptr)
{
	ESerial.reportArray("Task runtimes in us: ", task_runtime, n_tasks);
}
const char toggleAutoStatus(void* obj_ptr)
{
	if (!auto_stat)
		auto_stat = 1;
	else
		auto_stat = 0;
	return 0;
	//auto_stat = 0x01 & !auto_stat;
}
void autoReport()
{
	if (auto_stat == 1)
		auto_stat = 2;
	return;
}

const char increaseGlobal(void* obj_ptr)
{
	global_offset.value += 10;
	ESerial.print("+");
}
const char decreaseGlobal(void* obj_ptr)
{
	global_offset.value -= 10;
	ESerial.print("-");
}
const char enableDrive(void* obj_ptr)
{
	if (good_ee_loads)
	{
		digitalWrite(fuel_pin, HIGH);
		digitalWrite(drv_en_pin, LOW);
		//attachInterrupt(4, isrTachRisingEdge, RISING);	// interrupt 4 maps to pin 19. 
		ESerial.println(F("Armed."));
	}
	else
		ESerial.println(F("bad EE data, no go."));
}
const char disableDrive(void* obj_ptr)
{
	digitalWrite(fuel_pin, LOW);
	digitalWrite(drv_en_pin, HIGH);		// this turns off the injectors and ignition
	//detachInterrupt(4);
	ESerial.println(F("inj, fuel disabled."));
}
const char memory(void* obj_ptr)
{
	extern int __heap_start, *__brkval;
	int free_ram, v;
	free_ram = (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);

	ESerial.print(F("Free ram: "));
	ESerial.println(free_ram);
	return 0;
}


void initProtocol()
{
	ESerial.addCommand(F("x"), disableDrive, NULL);
	ESerial.addCommand(F("arm"), enableDrive, NULL);
	ESerial.addCommand(F("auto"), toggleAutoStatus, NULL);
	ESerial.addCommand(F("lock"), FuelTweaker::lock, &boss);
	ESerial.addCommand(F("mode"), reportMode, NULL);
	ESerial.addCommand(F("+"), increaseGlobal, NULL);
	ESerial.addCommand(F("-"), decreaseGlobal, NULL);
	ESerial.addCommand(F("stat"), reportStatus, NULL);
	ESerial.addCommand(F("para"), reportParams, NULL);
	ESerial.addCommand(F("save"), saveData, NULL);
	ESerial.addCommand(F("load"), loadData, NULL);
	ESerial.addCommand(F("ee"), reportEEAddresses, NULL);
	ESerial.addCommand(F("task"), reportTaskTimes, NULL);
	ESerial.addCommand(F("mem"), memory, NULL);


	ESerial.addCommand(F("rbos"), FuelTweaker::status, &boss);
	ESerial.addCommand(F("pbos"), FuelTweaker::reportParams, &boss);
	ESerial.addCommand(F("Sbos"), FuelTweaker::save, &boss);
	
	ESerial.addCommand(F("Wolt"), Parameter::write, &(boss.o2_lower_thresh));
	ESerial.addCommand(F("Wout"), Parameter::write, &(boss.o2_upper_thresh));
	ESerial.addCommand(F("Wowi"), Parameter::write, &(boss.time_warming_o2_thresh));
	ESerial.addCommand(F("Wlsl"), Parameter::write, &(boss.local_sum_limit));
	ESerial.addCommand(F("Wtsz"), Parameter::write, &(boss.step_size));
	ESerial.addCommand(F("Wewi"), Parameter::write, &(boss.time_eng_warm_thresh));
	ESerial.addCommand(F("Wrph"), Parameter::write, &(boss.rpm_hyst));
	ESerial.addCommand(F("Weri"), Parameter::write, &(boss.time_running_thresh));
	ESerial.addCommand(F("Wibs"), Parameter::write, &(boss.idle_backstep));
	ESerial.addCommand(F("Wiaf"), Parameter::write, &(boss.idle_adjust_freq));
	
	ESerial.addCommand(F("Wcrp"), Parameter::write, &coasting_rpm);

	ESerial.addCommand(F("Wirp"), Parameter::write, &idling_rpm);
	
	ESerial.addCommand(F("Winj"), Map::write, &inj_map);	// write a new injector map from what I send you next
	ESerial.addCommand(F("rinj"), Map::read, &inj_map);		// report the injector map to the serial port
	ESerial.addCommand(F("Sinj"), Map::save, &inj_map);		// save the injector map to EEPROM
	ESerial.addCommand(F("linj"), Map::load, &inj_map);		// load theh injector map from EEPROM

	// don't want write access to correction map.  Optimizer handles this.
	ESerial.addCommand(F("rloc"), Map::read, &offset_map);
	ESerial.addCommand(F("Sloc"), Map::save, &offset_map);
	ESerial.addCommand(F("lloc"), Map::load, &offset_map);
	ESerial.addCommand(F("Cloc"), Map::clear, &offset_map);
	// don't want manual + or - control over correction map.  Optimizer handles this

	ESerial.addCommand(F("rchg"), Map::read, &change_map);
	ESerial.addCommand(F("Cchg"), Map::clear, &change_map);

	ESerial.addCommand(F("Wglo"), Parameter::write, &global_offset);
	ESerial.addCommand(F("rglo"), Parameter::read, &global_offset);
	ESerial.addCommand(F("Sglo"), Parameter::save, &global_offset);
	ESerial.addCommand(F("lglo"), Parameter::load, &global_offset);
	ESerial.addCommand(F("Cglo"), Parameter::clear, &global_offset);

	ESerial.addCommand(F("Wcho"), Scale::write, &choke_scale);
	ESerial.addCommand(F("rcho"), Scale::read, &choke_scale);
	ESerial.addCommand(F("Scho"), Scale::save, &choke_scale);
	ESerial.addCommand(F("lcho"), Scale::load, &choke_scale);

	ESerial.addCommand(F("Wtem"), Scale::write, &temp_scale);
	ESerial.addCommand(F("rtem"), Scale::read, &temp_scale);
	ESerial.addCommand(F("Stem"), Scale::save, &temp_scale);
	ESerial.addCommand(F("ltem"), Scale::load, &temp_scale);

	ESerial.addCommand(F("Wgri"), Scale::write, &air_rpm_scale);
	ESerial.addCommand(F("rgri"), Scale::read, &air_rpm_scale);
	ESerial.addCommand(F("Sgri"), Scale::save, &air_rpm_scale);
	ESerial.addCommand(F("lgri"), Scale::load, &air_rpm_scale);

	ESerial.addCommand(F("Wcrp"), Parameter::write, &coasting_rpm);
	ESerial.addCommand(F("rcrp"), Parameter::read, &coasting_rpm);
	ESerial.addCommand(F("Scrp"), Parameter::save, &coasting_rpm);
	ESerial.addCommand(F("lcrp"), Parameter::load, &coasting_rpm);

	ESerial.addCommand(F("Wirp"), Parameter::write, &idling_rpm);
	ESerial.addCommand(F("rirp"), Parameter::read, &idling_rpm);
	ESerial.addCommand(F("Sirp"), Parameter::save, &idling_rpm);
	ESerial.addCommand(F("lirp"), Parameter::load, &idling_rpm);

	ESerial.addCommand(F("Wath"), Parameter::write, &air_thresh);
	ESerial.addCommand(F("rath"), Parameter::read, &air_thresh);
	ESerial.addCommand(F("Sath"), Parameter::save, &air_thresh);
	ESerial.addCommand(F("lath"), Parameter::load, &air_thresh);

	ESerial.addCommand(F("Wcld"), Parameter::write, &cold_threshold);
	ESerial.addCommand(F("rcld"), Parameter::read, &cold_threshold);
	ESerial.addCommand(F("Scld"), Parameter::save, &cold_threshold);
	ESerial.addCommand(F("lcld"), Parameter::load, &cold_threshold);

	ESerial.addCommand(F("Wcld"), Parameter::write, &cold_threshold);
	ESerial.addCommand(F("rcld"), Parameter::read, &cold_threshold);
	ESerial.addCommand(F("Scld"), Parameter::save, &cold_threshold);
	ESerial.addCommand(F("lcld"), Parameter::load, &cold_threshold);

	ESerial.addCommand(F("Wcra"), Parameter::write, &cranking_dur);
	ESerial.addCommand(F("rcra"), Parameter::read, &cranking_dur);
	ESerial.addCommand(F("Scra"), Parameter::save, &cranking_dur);
	ESerial.addCommand(F("lcra"), Parameter::load, &cranking_dur);

	ESerial.addCommand(F("Wacc"), Parameter::write, &accel_stabilize_rate);
	ESerial.addCommand(F("racc"), Parameter::read, &accel_stabilize_rate);
	ESerial.addCommand(F("Sacc"), Parameter::save, &accel_stabilize_rate);
	ESerial.addCommand(F("lacc"), Parameter::load, &accel_stabilize_rate);
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
		if (air_flow_snap > accel_stabilize_rate.value)
			air_flow_snap -= accel_stabilize_rate.value;
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
	avg_rpm.addSample(rpm);
}

// Pulse duration business
void calcInjDuration()
{
	static unsigned int new_inj_duration;
	static int accel_offset, choke_offset, air_temp_offset;

	if (digitalRead(cranking_pin))
	{
		choke_offset = adjustForColdEngine(cranking_dur.value, coolant_temp) >> 1;	
		//choke_offset >>= 1;									// halve this for cranking. 
		inj_duration = cranking_dur.value + choke_offset;
		return;
	}
	if (run_condition == COASTING)
	{
		inj_duration = 0;
		return;
	}
	
	// find nominal
	new_inj_duration = inj_map.interpolate(rpm, air_flow, &offset_map); 
	
	// adjust for stuff
	accel_offset = adjustForAccel(air_flow);		// these should not be order dependent.  
	choke_offset = adjustForColdEngine(new_inj_duration, coolant_temp);
	air_temp_offset = adjustForAirTemp(new_inj_duration, air_temp);

	new_inj_duration = new_inj_duration + accel_offset + choke_offset + air_temp_offset + global_offset.value;

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
	
	if (run_condition & _BV(WARM))		// is engine already warm? 
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

void tweakFuel()
{
	boss.tweak();
}
void updateRunCondition()
{
	char idl_or_full = !digitalRead(idl_full_pin);
	run_condition = 0;

	if (rpm < RUNNING_RPM)	
		run_condition |= _BV(NOT_RUNNING);

	else if (digitalRead(cranking_pin))
		run_condition |= _BV(CRANKING);

	else if (idl_or_full && (air_flow < air_thresh.value) && (rpm > coasting_rpm.value))
		run_condition |= _BV(COASTING);

	else if (idl_or_full && (air_flow < air_thresh.value) && (rpm < idling_rpm.value))
		run_condition |= _BV(IDLING);

	else if (idl_or_full && (air_flow > air_thresh.value))
		run_condition |= _BV(WIDE_OPEN);

	if (coolant_temp >= cold_threshold.value)
		run_condition |= _BV(WARM);
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
ISR(TIMER3_OVF_vect)
{
	tach_period = 0xFFFF;	// this makes it obvious the engine is not running
}