
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

// Data: 60e6 us/min   16us/timer tic (256 prescale, 16Mhz)   2 tach pulses per rev.
// 60e6 / 16 / 2 = 468750 timer tics/pulse * rpm 
// so rpm = 1875000/(spark_period)

#define TICS_PER_TACH 1875000

void setup()
{
	char enable = 0;

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
	task[0] = readAirFlow;			ms_freq_of_task[0] = 50;
	task[1] = readO2Sensor;			ms_freq_of_task[1] = 50;
	task[2] = calcRPMandDwell;		ms_freq_of_task[2] = 50;
	task[3] = calcInjDuration;		ms_freq_of_task[3] = 50;
	task[4] = updateInjectors;		ms_freq_of_task[4] = 50;
	task[5] = readAirTemp;			ms_freq_of_task[5] = 250;
	task[6] = readCoolantTemp;		ms_freq_of_task[6] = 250;
	task[7] = readOilPressure;		ms_freq_of_task[7] = 250;
	task[8] = autoReport;			ms_freq_of_task[8] = 1000;
	task[9] = updateRunCondition;	ms_freq_of_task[9] = 100;
	task[10] = tweakFuel;			ms_freq_of_task[10] = TWEAK_FREQ_MS;
	n_tasks = 11;

	i_autoreport = 8;		// this is referenced to delay next autoreport message.  

	// input interrupt pin
	digitalWrite(cs_knock_pin, HIGH);
	digitalWrite(cs_inj_pin, HIGH);
	digitalWrite(cs_sd_pin, HIGH);

	setPinModes(inputs, INPUT);
	setPinModes(outputs, OUTPUT);
	pinMode(40, INPUT);		// the other hooked-up inj1_pin. 
	
	digitalWrite(cs_inj_pin, HIGH);
	digitalWrite(cs_knock_pin, HIGH);
	digitalWrite(cs_sd_pin, HIGH);

	//pinMode(coil1_pin,)
	//pinMode(13, OUTPUT);

	inj_duration = 5;
	tach_period_i = 0;
	tach_period[tach_period_i] = old_tach_period = 0;
	
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

	// Configure Timer4 for measuring ignition dwell (using interrupt)
	TCCR4A = 0;							// clear control register A 
	TCCR4B = _BV(CS42);					// start the timer at clk/256 prescale. = 16us per tic. 
	//TCCR4B = _BV(CS42) | _BV(CS40);		// start the timer at clk/1024 prescale. = 64us per tic. 

	// enable timer overflow interrupt for non-running and slow-cranking issues
	TIMSK4 |= _BV(TOIE4);	// this affects rpm calculation for low RPM

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

	attachInterrupt(tach_interrupt, isrTachRisingEdge, RISING);	// interrupt 4 maps to pin 19. 
	attachInterrupt(coil_current_interrupt, isrCoilNominalCurrent, RISING);	// interrupt 3 maps to pin 20. 

	EEPROM_readAnything(ENABLE_ADDY, enable);
	if (enable)
		enableDrive();
	else
		ESerial.println(F("fuck off asshole"));		// said to either my forgetful self, or a van-thief. 

	auto_stat = 1; // turn on auto-reporting. 

	// following sets an 8ms timeout on dwell with soft shutdown. 
	// it must come some significant time after SPIInit().  Hence this placement. 
	SPIInitSparkMode();	
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
	if (auto_stat == 2)
	{
		auto_stat = 1;
		reportStatus(NULL);
	}

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
	static unsigned int old_rpm = 0;
	// (60e6 us/min) / (4 us/tic)				= 15e6 tics/min
	// (tics/min) / (tach_period tics/pulse)	= pulses/min
	// pulses/min * (1 rev/pulse)				= 15e6 / tach_period 
	// we're pulsing the injectors (and measuring) every other tach input pulse, so it's 1:1 with crankshaft. 
	old_rpm = rpm;
	rpm = TICS_PER_TACH / tach_period[tach_period_i];
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
	
	////// DWELL SECTION ////////////
	// we only start charging coil if RPM is at least 200

	if ((rpm > MIN_IGN_RPM) && (abs(rpm_d) < (rpm >> 2)))
	{
		// clear the output compare flag. 
		TIFR4 &= ~_BV(OCF4A);	// clear the output compare flag. 
		TIMSK4 |= _BV(OCIE4A);	// enable the out.comp. interrupt (ignition system)
		digitalWrite(drv_en_pin, LOW);
	}
	else
	{
		// disable ignition system. 
		TIMSK4 &= ~_BV(OCIE4A);			// this prevents the coil pin from being set
		digitalWrite(drv_en_pin, HIGH);	// this starts a soft-shutdown
		digitalWrite(coil1_pin, LOW);	// turn off the coil. 
	}

	if (rpm < 800)
	{
		if (digitalRead(cranking_pin))
			g_dwell = starting_dwell.value;		// High dwell during cranking
		else
			g_dwell = low_speed_dwell.value;	//this is a longer dwell
	}
	else
		g_dwell = hi_speed_dwell.value;
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

	if (run_condition & _BV(IDLING))
	{
		new_inj_duration = idle_offset.value + idle_scale.interpolate(rpm);
		inj_duration = (new_inj_duration + adjustForColdEngine(new_inj_duration, coolant_temp));
		return; 
	}

	if (run_condition & _BV(COASTING))
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
void updateInjectors()
{
	if (inj_duration > 10)
		OCR3A = inj_duration;		// normal operation
	else
		OCR3A = 10;					// if inj_duration = 0, we just drop it. 

	// if we just set the compare register below the timer, we need to close the injector 
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
	// Spark!  (time-sensitive, hence placement at top of this ISR)
	PORTA &= ~0x40;		// 0b 1011 1111 (A6 is turned off) Opens the ignition coil circuit, causing spark


	// these fuel injection actions happen every other pulse.  We use pulse_divider to track which pulse we're on.
	static char pulse_divider = 0;	
	if (!pulse_divider)
	{
		GTCCR |= _BV(PSRSYNC);		// clear the prescaler. 

		//tach_period = TCNT3; 		// record the timer setting 
		TCNT3 = 0;					// start the timer over. 

		// set all injectors high (in same instruction)
		if (inj_duration)
		{
			PORTL |= 0xAA;				// 0b 1010 1010;
			PORTA |= 0x08;				// 0b 0000 1000  (A3 is turned on) 
		}
		pulse_divider = 1;
	}
	else
		--pulse_divider;

	// Finish the ignition operations here.  (Dwell is not as time-sensitive)
	old_tach_period = tach_period[tach_period_i];
	// spark period measured separately to handle low-rpm and update every pulse. 
	if (++tach_period_i >= TACH_PERIOD_N)
		tach_period_i = 0;
	tach_period[tach_period_i] = TCNT4;
	

	// set a timer for when we begin dwell again
	TCNT4 = 0;
	// here we set the on-time for charging the coil (dwell) to be 
	// dwell.value * 4us before the next predicted tach rising edge.
	OCR4A = tach_period[tach_period_i] - g_dwell;
	

}
ISR(TIMER3_COMPA_vect)			// this runs when TCNT3 == OCR3A. 
{
	// set all injectors low (in same instruction)
	PORTL &= ~0xAA;			// 0b01010101; 
	PORTA &= ~0x08;				// 
}

ISR(TIMER4_COMPA_vect)			// runs when TCNT4 == OCR4A. 
{
	PORTA |= 0x40;		// 0b 0100 0000 (A6 is turned on) starts ign. coil dwell, in prep for spark
}
ISR(TIMER4_OVF_vect)
{
	/*if (++tach_period_i >= TACH_PERIOD_N)
		tach_period_i = 0;*/
	tach_period[tach_period_i] = 65000;	// this makes it obvious the engine is not running 
							// rpm will be 468,750 / 65,000 = 7 rpm

	digitalWrite(drv_en_pin, HIGH);	// disables drive.  This causes soft-shutdown of coil (no spark). 

}
void isrCoilNominalCurrent() {
	coil_current_flag = 1;
}