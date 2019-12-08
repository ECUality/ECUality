
#include "Defines.h"
#include "Globals.h"
#include "Arrays.h"
#include "Map.h"
#include "Scale.h"
#include "Parameter.h"
#include "ECUSerial.h"
#include "ECUProtocol.h"
#include "Interpolation.h"
#include "EEIndex.h"
#include "EEPROMAnything.h"
#include "ECUPins.h"
#include "FuelTweaker.h"
#include "MovingAverage.h"
#include "SPICommands.h"
#include <EEPROM.h>
#include <SPI.h>

// Data: 60e6 us/min   16us/timer tic (256 prescale, 16Mhz)   
// quarter-rev of dizzy is measured 1/2 rev of crank
// us/min / us/tic  = tics/min
// tics / min * rev/sample = rev/min * tics/sample
// rev/min * tics/sample / tics/sample = rev/min 
// 60e6 / 16 / 2 = 1,875,000 rpm * tics/sample
// so rpm = 1875000/(spark_period)

#define TICS_PER_TACH 1875000

void addTask(void(*task)(), unsigned int ms);	// for some reason, this needs a prototype before it is used where others don't. 

void setup()
{
	char enable = 0;
	simulate_tach_en = 0;					// default to debug resources off. 
	fuel_pump_en = injector_en = coil_en = 1;	// default to having driving resources on. 

	ESerial.begin(9600);
	//Serial.begin(115200);

	InitSPI();

	NameParams();

	// There are 3 levels of priority in this program.  
	// Every event falls into one of these categories
	// 1: Injector duration timing (highest)
	// 2: Tasks that happen periodically (middle)
	// 3: Responses to the user (lowest)
	//
	// The following "task" array contains the periods of the level-2 events. 
	// Execution of these events can be interrupted by a level-1 event. 
	// These events will interrupt execution of a level-3 event. 
	n_tasks = 0;
	addTask(simulatedTachEdge, 20);
	addTask(readAirFlow, 50);
	addTask(readO2Sensor, 50);
	addTask(calcRPMandDwell, 50);
	addTask(calcInjDuration, 50);
	addTask(calcIgnitionMarks, 50);
	addTask(readAirTemp, 250);
	addTask(readCoolantTemp, 250);
	addTask(readOilPressure, 250);
	addTask(autoReport, 1000);
	addTask(updateRunCondition, 100);
	addTask(tweakFuel, TWEAK_FREQ_MS);

	i_autoreport = 8;		// this is referenced to delay next autoreport message.  

	// input interrupt pin
	digitalWrite(cs_knock_pin, HIGH);
	digitalWrite(cs_inj_pin, HIGH);
	digitalWrite(cs_sd_pin, HIGH);

	setPinModes(inputs, INPUT);
	setPinModes(outputs, OUTPUT);
	pinMode(40, INPUT);		// the other hooked-up inj1_pin. 
	pinMode(index_pin, INPUT_PULLUP);	// enable pullup on index pin. 

	digitalWrite(cs_inj_pin, HIGH);
	digitalWrite(cs_knock_pin, HIGH);
	digitalWrite(cs_sd_pin, HIGH);


	inj_duration = 5;
	tach_period = 0;

	Map::clear(&change_map);
	good_ee_loads = loadData(NULL);		// load data from EE.

	initProtocol();						// set up protocol. 

	// TIMERS
	// Configure Timer1 for task scheduling.  Hits ICR1 every 2ms and runs interrupt. 
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
	//TIMSK3 |= _BV(TOIE3);				// enable timer overflow interrupt on timer 3 (just-in-case-of-something-weird)

	// Configure Timers 4 and 5 for measuring ignition dwell (using interrupt)
	TCCR4A = TCCR5A = 0;				// clear control register A 
	TCCR4B = _BV(CS42);					// start the timers at clk/256 prescale. = 16us per tic. 
	TCCR5B = _BV(CS52);
	//TCCR4B = _BV(CS42) | _BV(CS40);		// start the timer at clk/1024 prescale. = 64us per tic. 

	// enable timer overflow interrupt for non-running and slow-cranking issues
	TIMSK4 |= _BV(TOIE4);	// this affects rpm calculation for low RPM
	TIMSK5 |= _BV(TOIE5);


	// Timer4 NOTE: updateDwell() enables or disables the ignition system by 
	// setting or clearing OCIE4A, the output compare interrupt enable bit. 
	// The Output compare interrupt is what starts the dwells.
	// It is important for engine-start that we NOT set OCIE4A here. 


	// disable the timer 0 interrupt.  This breaks millis() but prevents interference with pulse timing.
	// note: ECUSerial.cpp uses millis() to time communication-stream events.  
	//TIMSK0 &= 0x00;		

	// Configure Timer0 for generating a fast PWM.  16us period, 62.5kHz
	// CONFIG BITS:
	// WGM0<3:0>	= 0 (0000)	normal mode, counts up to 0xFFFF.  
	// COM0A<1:0>	= 2 (10)	clears OC0A on compare match so OCR0A represents "high" time. 
	// COM0B<1:0>	= 2 (10)	same as A.
	// CS0<2:0>		= 1 (001)	1:1 pre-scaling, timer running. 

	// next line: the input circuit inverts the signal, so a rising edge from the distributor produces a falling
	// edge at the Arduino.  We trigger when the tab enters the Hall sensor.  The hall sensor produces a high signal
	// when it is looking at a tab, so we want to trigger here on a FALLING edge. 

	attachInterrupt(tach_interrupt, isrTachEdge, FALLING);	// interrupt 4 maps to pin 19. 	
	attachInterrupt(coil_current_interrupt, isrCoilNominalCurrent, RISING);	// interrupt 3 maps to pin 20. 

	EEPROM_readAnything(ENABLE_ADDY, enable);
	if (enable)
		enableDrive();
	else
		ESerial.println(F("engine disabled"));		// said to either my forgetful self, or a van-thief. 

	EEPROM_readAnything(TWEAK_LOCK_ADDY, boss.lockout);
	if (boss.lockout)
		ESerial.println(F("tweaks locked"));		// lock bit set, no tuning. 

	auto_stat = 1; // turn on auto-reporting. 

	// following sets an 8ms timeout on dwell with soft shutdown. 
	// it must come some significant time after SPIInit().  Hence this placement. 
	SPIInitSparkMode();	
}

void addTask(void(*task)(), unsigned int ms)
{
	task_list[n_tasks] = task;
	ms_freq_of_task[n_tasks] = ms;
	n_tasks++;
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
	// reportStatus is queued for action in the task scheduler but called here because serial output
	// takes forever and doesn't have as high a priority as running the engine. 
	if (auto_stat == 2)
	{
		auto_stat = 1;
		reportStatus(status_mode);
	}

	// check for any available serial input, then parse and run the commands. 
	ESerial.executeCommand();	
}

void autoReport()
{
	if (auto_stat == 1)
		auto_stat = 2;
	return;
}

// Sensor reading functions
void readAirFlow()
{

	air_flow_d = -air_flow;
	air_flow = analogRead(air_flow_pin) >> 2;	// takes 100us.  Should shorten
	air_flow_d += air_flow;

	if (air_flow_d > 20)
	{
		n_air_faults++;
		//ESerial.print(F("!air flt: "));
		//ESerial.println(n_air_faults);
	}

	/*
	air_flow_snap += air_flow_d;

	if (air_flow_snap)
	{
		if (air_flow_snap > accel_stabilize_rate.value)
			air_flow_snap -= accel_stabilize_rate.value;
		else
			air_flow_snap = 0;
	} */
	

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
void calcRPMandDwell()
{
	static uint16_t old_rpm = 0;
	static uint16_t l_dwell;
	static uint16_t l_tach_period; // a local to this function copy of the volatile tach-period
	char cSREG;

	cSREG = SREG; // store SREG value - we don't assume GIE is set, we just copy it. 
	cli(); //disable interrupts during copy
	l_tach_period = tach_period;
	SREG = cSREG; /* restore SREG value (GIE-bit) to what it was*/

	// (60e6 us/min) / (4 us/tic)				= 15e6 tics/min
	// (tics/min) / (tach_period tics/pulse)	= pulses/min
	// pulses/min * (1 rev/pulse)				= 15e6 / tach_period 
	// we're pulsing the injectors (and measuring) every other tach input pulse, so it's 1:1 with crankshaft. 
	old_rpm = rpm;

	rpm = TICS_PER_TACH / l_tach_period;		// tach_period_copy doesn't get touched by interrupts, so this is safe. 
	rpm_d = old_rpm - rpm;
	avg_rpm.addSample(rpm);
	
	// This section says if we're running, and the rpm makes an abrupt change, 
	// it's a fault.  Log it.  
	if (!(run_condition & _BV(NOT_RUNNING) ) && (abs(rpm_d) > 1000))
	{
		if (rpm > avg_rpm.average)
		{
			n_rpm_hi_faults++;
			//ESerial.print("!RPM xtra ");
			//ESerial.println(n_rpm_hi_faults);
		}
		else
		{
			n_rpm_lo_faults++;
			//ESerial.print(F("!RPM miss "));
			//ESerial.println(n_rpm_lo_faults);
		}
	}
	
	// DWELL: we only start charging coil if RPM is at least 200
	if ((rpm > MIN_IGN_RPM) && (abs(rpm_d) < (rpm >> 2)))
	{
		// clear the output compare flag. 
		TIFR4 &= ~(_BV(OCF4A) | _BV(OCF4B));	// clear the output compare flags. 
		TIFR5 &= ~(_BV(OCF5A) | _BV(OCF5B));	

		TIMSK4 |= (_BV(OCIE4A) | _BV(OCIE4B));	// enable the out.comp. interrupt (ignition system)
		TIMSK5 |= (_BV(OCIE5A) | _BV(OCIE5B));	

		digitalWrite(drv_en_pin, LOW);		// enable the driver chip (active low)
	}
	else
	{
		// disable ignition system. 
		TIMSK4 &= ~_BV(OCIE4A);			// this prevents the coil pin from being set
		TIMSK5 &= ~_BV(OCIE5A);	

		digitalWrite(drv_en_pin, HIGH);	// this starts a soft-shutdown - this pin active low
		PORTA &= 0xAA;			//  &= 1010_1010 turns off bits 0, 2, 4, and 6 (turns off all coils). 
	}

	if (rpm < 800)
	{
		if (digitalRead(cranking_pin))
			l_dwell = starting_dwell.value;		// High dwell during cranking
		else
			l_dwell = low_speed_dwell.value;	//this is a longer dwell
	}
	else
		l_dwell = hi_speed_dwell.value;

	cli(); //disable interrupts during copy
	g_dwell = l_dwell;
	SREG = cSREG; /* restore SREG value (GIE-bit) to what it was*/
}

// Pulse duration business
void calcInjDuration()
{
	static unsigned int l_inj_duration;
	static int accel_offset, choke_offset, air_temp_offset, global_offset;

	if (digitalRead(cranking_pin))
	{
		map_inj_duration = cranking_dur.value;
		choke_offset = adjustForColdEngine(cranking_dur.value, coolant_temp);	
		l_inj_duration = map_inj_duration + choke_offset;
	}

	else if (run_condition & _BV(IDLING))	// only consider cold engine enrichment for idle.  No global correction. 
	{
		map_inj_duration = idle_offset.value + idle_scale.interpolate(rpm);
		choke_offset = adjustForColdEngine(map_inj_duration, coolant_temp);
		l_inj_duration = map_inj_duration + choke_offset;
	}

	else if (run_condition & _BV(COASTING))
	{
		map_inj_duration = 0;
		l_inj_duration = 0;
	}

	else
	{
		// find nominal
		map_inj_duration = inj_map.interpolate(rpm, air_flow, &offset_map);

		// adjust for stuff
		accel_offset = adjustForAccel(air_flow);		// these should not be order dependent.  
		choke_offset = adjustForColdEngine(map_inj_duration, coolant_temp);
		air_temp_offset = adjustForAirTemp(map_inj_duration, air_temp);
		global_offset = (map_inj_duration >> 8)*global_correction.value;

		l_inj_duration = map_inj_duration + accel_offset + choke_offset + air_temp_offset + global_offset;
	}

	// output that shit
	char cSREG;
	cSREG = SREG; // store SREG value - we don't assume GIE is set, we just copy it. 
	cli(); //disable interrupts during copy
	inj_duration = l_inj_duration;
	SREG = cSREG; /* restore SREG value (GIE-bit) to what it was*/
}

// Ignition advance business - executes immediately after calcInjDuration()
void calcIgnitionMarks() {
	static uint16_t dwell_mark, spark_mark;	// these are the timer settings for the dwell start and spark events.  
	static uint16_t l_tach_period;			// this is a local copy of the volatile tach_period.
	char cSREG;

	const uint16_t edge_to_post = 256;
	static uint16_t spark_position;

	// here we calculate the centrifugal advance (rpm based) 
	// the units are 1/512ths of a revolution to facilitate division speeds. 
	// the original timing  
	advance = rpm_adv.interpolate(rpm);
	/*if (rpm < 1100) advance = 0;
	else if (rpm < 2124) advance = (((rpm - 1100) * 5) >> 8);
	else if (rpm < 2380) advance = 20 + (((rpm - 2124) * 3) >> 7);
	else if (rpm < 4428) advance = 26 + (((rpm - 2380) * 5) >> 10);
	else advance = 35;*/

	// now we do the vacuum advance.  (light-load advance)
	// The engine load is indicated by the injector duration (how much fuel enters the cylinder every cycle) 
	// At sea-level, the maximum injector duration is about 1,800.  The minimum while coasting is about 500.  
	// Assuming 1800 = 1bar (absolute) and a linear scale, then 500 = .278 bar(abs) = .722 bar vacuum (21inHg)
	// The manual calls for vacuum advance to begin at .21 bar and max out at .36 bar, 14 degrees (= 20/512'ths)
	// Using the injector duration scale, .21 bar vacuum = .79 bar absolute = .79*1800 = 1422 inj.
	// Similarly, .36 bar vacuum = .64 bar absolute = 1152 injector.  
	advance += vac_adv.interpolate(map_inj_duration);

	/*if (map_inj_duration < 1200) advance += 20;
	else if (map_inj_duration < 1456) advance += (((1456 - map_inj_duration) * 5) >> 6);*/

	// That's it.  The advance can vary from 0 to 55.  1/512ths =  0 - 38.7 degrees. 
	// At 3500 rpm and inj_duration of 1200, we'll have 31(rpm) and 20(load) = 51/512 = 36 degrees. 


	spark_position = edge_to_post + 28 - advance;	// advance varies from 0 to 55, so we center it with + 28. 

	cSREG = SREG; // store SREG value - we don't assume GIE is set, we just copy it. 
	cli(); //disable interrupts during copy to the volatile variables. 
	l_tach_period = tach_period; 
	SREG = cSREG; /* restore SREG value (GIE-bit) to what it was*/

	spark_mark = ( (unsigned long)spark_position * l_tach_period ) >> 8;	// mark = (bdeg of event) * (timer tics per 256 bdeg) / (256 bdeg). 

	dwell_mark = spark_mark - g_dwell;

	// disable interrupts while Output Compares are set. 
	cSREG = SREG; // store SREG value - we don't assume GIE is set, we just copy it. 
	cli(); //disable interrupts during copy to the volatile variables. 
	v_spark_mark = spark_mark;
	v_dwell_mark = dwell_mark;
	SREG = cSREG; /* restore SREG value (GIE-bit) to what it was*/

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

	if (digitalRead(cranking_pin))
		run_condition |= _BV(CRANKING);

	else if ((rpm < RUNNING_RPM) || !digitalRead(fuel_pin))
		run_condition |= _BV(NOT_RUNNING); 

	else if (idl_or_full && (air_flow < air_thresh.value))
		//(air_flow < coasting_air.interpolate(rpm))
	{
		if (rpm > coasting_rpm.value)
			run_condition |= _BV(COASTING);
		else
			run_condition |= _BV(IDLING);
	}

	else if (idl_or_full && (air_flow > air_thresh.value))
		run_condition |= _BV(WIDE_OPEN);

	if (coolant_temp >= cold_threshold.value)
		run_condition |= _BV(WARM);
}

/*void updateInjectors()
{
	if (inj_duration > 100)
		OCR3A = inj_duration;		// normal operation
	else
		OCR3A = 100;				// if inj_duration = 0, we just drop it. 

	// if we just set the compare register below the timer, we need to close the injector 
	// because if we don't the timer will have to wrap all the way around before it stops. 
	if (inj_duration <= TCNT3)		
	{
		// turn off the injectors
		PORTL &= ~0xAA;			// 0b01010101; 
		PORTA &= ~0x08;				// 
	}
}*/


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

/*********************** interrupt routines ***********************/
// Task scheduler interrupt 
ISR( TIMER1_CAPT_vect )			
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
			(*task_list[i])();						// call task i.
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

// Injectors open and spark events
void simulatedTachEdge() {
	// this function lets the ECU trigger its own tach based on the task scheduler timer (TIMER1)
	// useful for debugging electrical noise and such - the ECU can produce outputs without the engine running. 
	if (simulate_tach_en)
		isrTachEdge();
}
void isrTachEdge()
{
	static uint8_t next_active_coil = 0;

	if ( (PINA & _BV(PA7)) == 0 ) {	// check if index pin is low (triggered) digitalRead(index_pin) == LOW
		next_active_coil = 0x40;	// if index pin is low, set next active coil 
	}
	else {
		next_active_coil = next_active_coil >> 2;
	}

	// these fuel injection actions happen every other pulse.  We use pulse_divider to track which pulse we're on.
	static char pulse_divider = 0;	
	if (!pulse_divider)
	{
		// Ignition operations - we flip which counter we use based on the pulse_divider 
		tach_period = TCNT5;	// this was reset last edge, so this measures the most recent tab duration. 

		TCNT4 = 0;	// set a timer for when we begin dwell again
		OCR4A = v_dwell_mark;		// Set the Output compare value for dwell and sparking events.  
		OCR4B = v_spark_mark;
		timer4_coil = next_active_coil;

		// Fuel operations //
		GTCCR |= _BV(PSRSYNC);		// clear the prescaler. 
		TCNT3 = 0;					// start the timer over. 

		// set all injectors high (in same instruction)
		if (inj_duration && injector_en)
		{
			PORTL |= 0xAA;				// 0b 1010 1010;
			OCR3A = inj_duration; 
		}
		else
			OCR3A = 100;				// if inj_duration = 0, we keep OCR3A positive. 

		pulse_divider = 1;
	}
	else
	{
		// Ignition operations //
		tach_period = TCNT4;		// set last edge, so this measures only one tab 

		TCNT5 = 0;	// set a timer for when we begin dwell again
		OCR5A = v_dwell_mark;		// Set the Output compare value for dwell and sparking events.  
		OCR5B = v_spark_mark;
		timer5_coil = next_active_coil;
		--pulse_divider;
	}

}

// Injectors close event
ISR(TIMER3_COMPA_vect)			// this runs when TCNT3 == OCR3A. 
{
	// set all injectors low (in same instruction)
	PORTL &= ~0xAA;			// 0b01010101; 
}

// Begin dwell event
// Coil1: Arduino 28: Mega A6: 0b0100_0000, 0x40, 64
// Coil2: Arduino 26: Mega A4: 0b0001_0000, 0x10, 16
// Coil3: Arduino 24: Mega A2: 0b0000_0100, 0x04, 4
// Coil4: Arduino 22: Mega A0: 0b0000_0001, 0x01, 1

ISR(TIMER4_COMPA_vect)			// runs when TCNT4 == OCR4A. 
{
	StartDwell(timer4_coil);
}
ISR(TIMER5_COMPA_vect)			// same as for timer 4 but uses timer 5 so timer doesn't get reset before both compares happen
{
	StartDwell(timer5_coil);
}
void StartDwell(uint8_t active_coil)
{
	if (coil_en)
	{
		PORTA |= active_coil;		// coil is turned on; starts ign.coil dwell, in prep for spark
		//PORTA |= 0x08;			//(A3 is turned on for LED indication of function) 
		PORTA |= 0x08;				// turn on the LED
	}
}

// Spark event
ISR(TIMER4_COMPB_vect)
{
	Spark(timer4_coil);
}
ISR(TIMER5_COMPB_vect)
{
	Spark(timer5_coil);
}

void Spark(uint8_t active_coil)
{
	PORTA &= ~active_coil;	// Opens coil circuit, causing spark. 
	//PORTA &= ~0x40;		// 0b 1011 1111 (A6 is turned off) Opens the ignition coil circuit, causing spark
	PORTA &= ~0x08;				//  turn off LED 
}

// De-energize ignition coil without spark (dwell exceeded maximum duration)
ISR(TIMER4_OVF_vect)
{
	tach_period = 65000;	// this makes it obvious the engine is not running 
							// rpm will be 1,875,000 / 65,000 = 28 rpm

	digitalWrite(drv_en_pin, HIGH);	// disables drive (active low).  This causes soft-shutdown of coil (no spark). 
}
ISR(TIMER5_OVF_vect)	// same as directly above. Different timer
{
	tach_period = 65000;	
	digitalWrite(drv_en_pin, HIGH);	// disables drive (active low).  This causes soft-shutdown of coil (no spark). 
}


// Monitor nominal current pin on ignition driver 
void isrCoilNominalCurrent() {
	coil_current_flag = 1;
}