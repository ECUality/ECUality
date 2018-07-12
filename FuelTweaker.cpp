#include "FuelTweaker.h"
#include "EEIndex.h"
#include "ECUSerial.h"


FuelTweaker::FuelTweaker(
	const unsigned char& run_condition_, 
	const int& air_flow_, 
	const int& rpm_, 
	const int& avg_rpm_,
	const int& o2_, int& global_correction_, 
	Map& offset_map_, Map& change_map_) :

	run_condition(run_condition_), o2(o2_),					// extermal references
	air_flow(air_flow_), rpm(rpm_), avg_rpm(avg_rpm_),		// external refs. 
	global_correction(global_correction_),					// external refs. 
	offset_map(offset_map_), change_map(change_map_),		// maps

	lockout(0), mode(0), o2_open(0), idle_adjustment(0),	// internal chars
	time_warming_o2(0), time_eng_warm(0), rpm_old(0),		// internal ints
	time_waiting(0),

	o2_upper_thresh("out", 300, 1000),			// Internal Parameters
	o2_lower_thresh("olt", 0, 700),
	step_size("tsz", 0, 10),
	local_sum_limit("lsl", 10, 300),	

	time_warming_o2_thresh("owi", 3, 90),
	time_eng_warm_thresh("ewi", 10, 3600),
	time_running_thresh("eri", 10, 3600),
	
	ee_addy(EE_index.getNewAddress(50))
{
	o2_upper_thresh.setName(F("O2_upper"));
	o2_lower_thresh.setName(F("O2_lower"));
	step_size.setName(F("tweak_step_sz"));
	local_sum_limit.setName(F("loc_sum_limit"));
	time_warming_o2_thresh.setName(F("time_warming_o2"));
	time_eng_warm_thresh.setName(F("time_eng_warm"));
	time_running_thresh.setName(F("time_glo_only"));

	/* This commented because these values initialized from EEPRPOM */
	
	o2_upper_thresh.value = 550;		
	o2_lower_thresh.value = 370;

	//  the size of tweak steps 
	step_size.value = 0;		// start with it zeroed, so there is no tweaking until loaded from EE. 

	local_sum_limit.value = 40;		// the limit to the sum of the local offset map (governs local vs global changes) 

	//  time from when engine stops coasting until it starts tweaking again.  
	time_warming_o2_thresh.value = 20;	// (20 sec) * (1000 ms/s) / (ms/tic)

	//  time from when engine is warm until local tweaks allowed. 
	time_eng_warm_thresh.value = 3 * 60;		// (5 min) * (60 s/min) * (1000 ms/s) / (ms/tic)

	// time from when engine starts until O2-based tweaks are allowed. 
	time_running_thresh.value = 3 * 60;
	
}


FuelTweaker::~FuelTweaker()
{
}

void FuelTweaker::addToLocal(int adjustment)
{
	offset_map.localOffset(rpm, air_flow, adjustment);
	change_map.localOffset(rpm, air_flow, adjustment);
	local_sum += adjustment;
}

void FuelTweaker::tweakGlobalvO2()
{
	mode = GLOBAL_MODE;
	if (o2 > o2_upper_thresh.value)
		global_correction--;		// here we make changes of only 1 because 1 on global_correction = 1/256 = .4% change
	else if (o2 < o2_lower_thresh.value)
		global_correction++; 
}

void FuelTweaker::tweakLocalAndGlobalvO2()
{
	mode = LOCAL_MODE;
	static uint8_t local_step_size;
	local_step_size = step_size.value;

	if (o2 > o2_upper_thresh.value)					// rich:  reduce offsets
	{
		global_correction--;
		addToLocal(-local_step_size);
	}
	else if (o2 < o2_lower_thresh.value)			// lean:  increase offsets
	{
		global_correction++;
		addToLocal(local_step_size);
	}
}

void FuelTweaker::tweak()
{
	// count time that we've been operating warm
	// count time that we have not been coasting. (O2 sensor warm) 
	time_eng_warm++;		// yeah, this wraps at ~4.5 hours.  That'll stop local optimizations for a few min. bite me. 
	time_warming_o2++;
	time_running++;
	if (run_condition & _BV(NOT_RUNNING))
		time_running = 0;

	if ( !(run_condition & _BV(WARM)) )
	{
		time_eng_warm = 0;
		time_warming_o2 = 0;
		//ESerial.print(F(";"));
	}
	

	// Set the weak pullup resistor on the O2 sensor input. 
	// todo! 


	// determine mode. ////////////////////
	
	// if the engine isn't pulling  on it's own fire, don't try to tweak. 
	if ( lockout || 
		(run_condition & (_BV(NOT_RUNNING) | _BV(CRANKING) | _BV(COASTING) | _BV(IDLING))) || 
		(time_running < (time_running_thresh.value * TWEAKS_PER_SEC))   )
	{
		mode = 0;	// the default, "not tuning" state.
		time_warming_o2 = 0;
		return;
	}

	
	if (time_warming_o2 < (time_warming_o2_thresh.value * TWEAKS_PER_SEC))
	{
		mode = 0;	// O2 can't be trusted, and we're not idling, so no tweaks can be reliably done. 
		return;
	}

	// If we're warm, we only make global changes at first.  Then we do both. 
	if ( time_eng_warm < (time_eng_warm_thresh.value * TWEAKS_PER_SEC) )
		tweakGlobalvO2();
	else
		tweakLocalAndGlobalvO2();
}

const char FuelTweaker::status(void* obj_ptr)
{
	FuelTweaker* self = (FuelTweaker *)obj_ptr;
	ESerial.print(F("locked: "));
	ESerial.print(self->lockout);
	ESerial.print(F("   mode: "));
	ESerial.print(self->mode);
	ESerial.print(F("   time_waiting: "));
	ESerial.println(self->time_waiting);
	ESerial.print(F("time_warming_o2: "));
	ESerial.print(self->time_warming_o2);
	ESerial.print(F("   time_eng_warm: "));
	ESerial.print(self->time_eng_warm);
	ESerial.print(F("   local_sum: "));
	ESerial.println(self->local_sum);
}

const char FuelTweaker::reportParams(void* obj_ptr)
{
	FuelTweaker* self = (FuelTweaker *)obj_ptr;
	
	ESerial.print(F("o2_upper_thr: "));
	ESerial.print(self->o2_upper_thresh.value);

	ESerial.print(F("   o2_lower_thr: "));
	ESerial.print(self->o2_lower_thresh.value);

	ESerial.print(F("   step: "));
	ESerial.print(self->step_size.value);

	ESerial.print(F("   local_sum_limit: "));
	ESerial.print(self->local_sum_limit.value);

	ESerial.println();

	ESerial.print(F("time thresholds->  warming_o2: "));
	ESerial.print(self->time_warming_o2_thresh.value);

	ESerial.print(F("   warm: "));
	ESerial.print(self->time_eng_warm_thresh.value);

	ESerial.print(F("   running: "));
	ESerial.print(self->time_running_thresh.value);


}
