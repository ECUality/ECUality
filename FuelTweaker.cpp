#include "FuelTweaker.h"
#include "EEIndex.h"
#include "ECUSerial.h"



FuelTweaker::FuelTweaker(const unsigned char& run_condition_, const int& air_flow_, const int& rpm_, const int& avg_rpm_,
	const int& o2_, int& global_offset_, int& idle_dur_, Map& offset_map_, Map& change_map_) :
	o2_upper_thresh("", 300, 1000),			// Param: 
	o2_lower_thresh("", 0, 700),			// Param: 
	time_warming_o2_thresh("", 3, 90),		// Param: seconds after coasting before O2 tweaks resume
	step_size("", 0, 100),					// Param: 
	local_sum_limit("", 10, 2000),			// Param: 
	time_eng_warm_thresh("", 30, 600),		// Param: seconds engine has to be warm before local tweaks start. 
	rpm_hyst("", 5, 150),					// Param: how sensitive the rpm optimizer is.  Smaller # = more sensitive. 
	time_running_thresh("", 30, 600),		// Param: seconds engine has to be running before any O2 tweaks start.
	idle_backstep("", 20, 500),				// Param: how far back we turn the idle enrichment screw after we see rpm fall. 
	idle_adjust_freq("", 5, 600),			// Param:  seconds between idle adjustments

	lockout(0), mode(0), o2_open(0), idle_adjustment(0),			// internal chars
	time_warming_o2(0), time_eng_warm(0), rpm_old(0),		// internal ints
	time_waiting(0),
	run_condition(run_condition_), o2(o2_),					// extermal references
	air_flow(air_flow_), rpm(rpm_), avg_rpm(avg_rpm_),		// external refs. 
	global_offset(global_offset_),							// external refs. 
	idle_dur(idle_dur_),
	offset_map(offset_map_), change_map(change_map_),		// maps
	ee_addy(EE_index.getNewAddress(50))
{
	o2_upper_thresh.value = 550;		// the o2 value above which we begin to trim leaner. 
	o2_lower_thresh.value = 370;		// the o2 value below which we begin to trim richer. 

	//  the size of tweak steps 
	step_size.value = 0;		// start with it zeroed, so there is no tweaking until loaded from EE. 

	local_sum_limit.value = 40;			// the limit to the sum of the local offset map (governs local vs global changes) 
	rpm_hyst.value = 50;
	idle_backstep.value = 50;

	//  time from when engine stops coasting until it starts tweaking again.  
	time_warming_o2_thresh.value = 20;	// (20 sec) * (1000 ms/s) / (ms/tic)

	//  time from when engine is warm until local tweaks allowed. 
	time_eng_warm_thresh.value = 3 * 60;		// (5 min) * (60 s/min) * (1000 ms/s) / (ms/tic)

	// time from when engine starts until O2-based tweaks are allowed. 
	time_running_thresh.value = 3 * 60;

	// time between adjustments to idle fuel mixture. 
	idle_adjust_freq.value = 45;		// seconds.
	
}


FuelTweaker::~FuelTweaker()
{
}

void FuelTweaker::tweakvRPM()
{
	// this is the overall tweaking mode.  Can be 0 (not tweaking), LOCAL_MODE, GLOBAL_MODE, IDLE_ADJ_MODE, IDLE_WAIT_MODE
	// it is only set to IDLE_ADJ_MODE or IDLE_WAIT_MODE inside this function.  
	if (mode != IDLE_ADJ_MODE )		
	{
		// if we just came down to idle from pulling or coasting, add 4 secs to timer. 
		// so we don't happen to start adjusting while the rpm is still settling down. 
		if (mode != IDLE_WAIT_MODE)				
			time_waiting -= (4 * TWEAKS_PER_SEC);	

		mode = IDLE_WAIT_MODE;				// record that we're now waiting. 
		time_waiting++;
		global_offset -= idle_adjustment;		// reverse changes from any interrupted previous adjustments. *IDLE or GLOBAL*
		idle_adjustment = 0;				// and zero out the tracker to reflect this reversal.

		// check if it's time to adjust. 
		if (time_waiting < (idle_adjust_freq.value * TWEAKS_PER_SEC))	// it's not time yet. 
			return;

		else									// it's totally time. 
		{
			mode = IDLE_ADJ_MODE;
			rpm_old = avg_rpm;
		}
	}
	// here, we know we're in Adjust mode (if we weren't we would have hit the <return> above)

	// check if RPM has decreased.  stop adjusting and (carburetor terminology) "turn the screw back 1/4 turn."
	if (avg_rpm < (rpm_old - rpm_hyst.value))
	{
		mode = IDLE_WAIT_MODE;				// say we're now waiting.
		time_waiting = 0;					// reset the waiting timer.

		// the following 1 line is from when I played with a single-value idle tune. (needed scale for stability)
		//idle_dur += idle_backstep.value;	// set the screw back a 1/4 turn. *IDLE or GLOBAL*

		// the following 2 lines are from when we were doing local/global distribution of changes.
		global_offset -= idle_adjustment;	// set global back then re-distribute adjustment to local and global. 
		apportionLocalGlobal(idle_adjustment + idle_backstep.value);

		idle_adjustment = 0;			// zero out the adjustment tracker (since we completed without interruption) 
	}
	else if (avg_rpm > rpm_old)			// if rpm goes up, compare to the new, higher rpm.  
		rpm_old = avg_rpm; 

	global_offset--;
	idle_adjustment--;
	
		
}

void FuelTweaker::apportionLocalGlobal(int adjustment)
{
	int max_local_adj = 0;

	if (adjustment >= 0)
	{
		max_local_adj = local_sum_limit.value - local_sum;
		
		if (max_local_adj >= adjustment)
			addToLocal(adjustment);

		else
		{
			addToLocal(max_local_adj);
			global_offset += (adjustment - max_local_adj);
		}
	}
	else
	{
		max_local_adj = -local_sum_limit.value - local_sum;

		if (max_local_adj <= adjustment)
			addToLocal(adjustment);

		else
		{
			addToLocal(max_local_adj);
			global_offset += (adjustment - max_local_adj);
		}
	}
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
		global_offset -= step_size.value;
	else if (o2 < o2_lower_thresh.value)
		global_offset += step_size.value; 
}

void FuelTweaker::tweakLocalAndGlobalvO2()
{
	mode = LOCAL_MODE;
	static uint8_t local_step_size;
	local_step_size = step_size.value << 1;			// local step size = step size * 2; 

	if (o2 > o2_upper_thresh.value)					// rich:  reduce offsets
	{
		if (local_sum <= -local_sum_limit.value)
			global_offset -= step_size.value;

		else
			addToLocal(-local_step_size);

	}
	else if (o2 < o2_lower_thresh.value)			// lean:  increase offsets
	{
		if (local_sum >= local_sum_limit.value)
			global_offset += step_size.value;

		else
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
	if (run_condition & _BV(COASTING))
		time_warming_o2 = 0;
	

	// Set the weak pullup resistor on the O2 sensor input. 
	// todo! 


	// determine mode. ////////////////////
	
	// if the engine isn't running on it's own fire, don't try to tweak. 
	if ( lockout || 
		(run_condition & (_BV(NOT_RUNNING) | _BV(CRANKING) | _BV(COASTING))) || 
		(time_running < (time_running_thresh.value * TWEAKS_PER_SEC))   )
	{
		mode = 0;	// the default, "not tuning" state.
		time_warming_o2 = 0;
		return;
	}

	// if we're idling, tweak to maximize RPM
	
	if (run_condition & _BV(IDLING))
	{
		tweakvRPM();
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
	//ESerial.print(F("  o2open:"));
	//ESerial.print(o2_open);
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

	ESerial.print(F("   rpm_hyst: "));
	ESerial.print(self->rpm_hyst.value);

	ESerial.print(F("   idle_backstep: "));
	ESerial.println(self->idle_backstep.value);

	ESerial.print(F("time thresholds->  warming_o2: "));
	ESerial.print(self->time_warming_o2_thresh.value);

	ESerial.print(F("   warm: "));
	ESerial.print(self->time_eng_warm_thresh.value);

	ESerial.print(F("   running: "));
	ESerial.print(self->time_running_thresh.value);

	ESerial.print(F("   idle_adjust_freq: "));
	ESerial.println(self->idle_adjust_freq.value);


}

const char FuelTweaker::load(void*obj_ptr)
{
	FuelTweaker* self = (FuelTweaker *)obj_ptr;
	char good = 1;

	good &= Parameter::load(&self->o2_lower_thresh);
	good &= Parameter::load(&self->o2_upper_thresh);
	good &= Parameter::load(&self->time_warming_o2_thresh);
	good &= Parameter::load(&self->step_size);
	good &= Parameter::load(&self->local_sum_limit);
	good &= Parameter::load(&self->time_eng_warm_thresh);
	good &= Parameter::load(&self->rpm_hyst);
	good &= Parameter::load(&self->time_running_thresh);
	good &= Parameter::load(&self->idle_backstep);
	good &= Parameter::load(&self->idle_adjust_freq);

	return good; 
}

const char FuelTweaker::save(void* obj_ptr)
{
	FuelTweaker* self = (FuelTweaker *)obj_ptr;
	
	Parameter::save(&self->o2_lower_thresh);
	Parameter::save(&self->o2_upper_thresh);
	Parameter::save(&self->time_warming_o2_thresh);
	Parameter::save(&self->step_size);
	Parameter::save(&self->local_sum_limit);
	Parameter::save(&self->time_eng_warm_thresh);
	Parameter::save(&self->rpm_hyst);
	Parameter::save(&self->time_running_thresh);
	Parameter::save(&self->idle_backstep);
	Parameter::save(&self->idle_adjust_freq);

	ESerial.print(F("saved Tweaker."));
}

const char FuelTweaker::lock(void* obj_ptr)
{
	FuelTweaker* self = (FuelTweaker *)obj_ptr;
	self->lockout = !self->lockout;
	if (self->lockout)
		ESerial.println(F("tweaks locked"));
	else
		ESerial.println(F("tweaks unlocked"));
}