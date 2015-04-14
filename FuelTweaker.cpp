#include "FuelTweaker.h"
#include "EEIndex.h"
#include "ECUSerial.h"


FuelTweaker::FuelTweaker(const unsigned char& run_condition_, const int& air_flow_, const int& rpm_, const int& o2_,
	int& global_offset_, Map& offset_map_, Map& change_map_) :
	o2_upper_thresh("", 300, 1000),			// Parameter: 
	o2_lower_thresh("", 0, 700),			// Parameter: 
	time_warming_o2_thresh("", 5, 240),		// Parameter: 
	step_size("", 0, 100),					// Parameter: 
	local_sum_limit("", 10, 300),			// Parameter: 
	time_eng_warm_thresh("", 60, 3600),		// Parameter: 

	lockout(0), mode(0), o2_open(0), direction(0),			// internal chars
	time_warming_o2(0), time_eng_warm(0), rpm_old(0),		// internal ints
	run_condition(run_condition_), o2(o2_),					// extermal references
	air_flow(air_flow_), rpm(rpm_), 						// external refs. 
	global_offset(global_offset_),							// external refs. 
	offset_map(offset_map_), change_map(change_map_),		// maps
	ee_addy(EE_index.getNewAddress(50))
{

	//  time from when engine is warm until local tweaks allowed. 
	time_eng_warm_thresh.value = 5 * 60 * (1000 / TWEAK_FREQ_MS);		// (5 min) * (60 s/min) * (1000 ms/s) / (ms/tic)

	//  time from when engine stops coasting until it starts tweaking again.  
	time_warming_o2_thresh.value = 20 * (1000 / TWEAK_FREQ_MS);	// (20 sec) * (1000 ms/s) / (ms/tic)

	//  the size of tweak steps 
	step_size.value = 0;		// start with it zeroed, so there is no tweaking until loaded from EE. 

	o2_upper_thresh.value = 550;		// the o2 value above which we begin to trim leaner. 
	o2_lower_thresh.value = 370;		// the o2 value below which we begin to trim richer. 

	local_sum_limit.value = 40;			// the limit to the sum of the local offset map (governs local vs global changes) 
}


FuelTweaker::~FuelTweaker()
{
}

void FuelTweaker::tweakGlobalvRPM()
{
	mode = 1;
	// check if RPM has decreased.  If it has, change the direction we're tweaking in.
	if (rpm < rpm_old)
		direction = !direction;
	rpm_old = rpm; 

	if (direction)
		global_offset += step_size.value;
	else
		global_offset -= step_size.value; 	
}

void FuelTweaker::tweakGlobalvO2()
{
	mode = 2;
	if (o2 > o2_upper_thresh.value)
		global_offset -= step_size.value;
	else if (o2 < o2_lower_thresh.value)
		global_offset += step_size.value; 
}

void FuelTweaker::tweakLocalAndGlobalvO2()
{
	mode = 3;
	if (o2 > o2_upper_thresh.value)					// rich:  reduce offsets
	{
		if (local_sum <= -local_sum_limit.value)
			global_offset -= step_size.value;
		
		else
		{
			offset_map.localOffset(rpm, air_flow, -step_size.value);
			change_map.localOffset(rpm, air_flow, -step_size.value);
			local_sum -= step_size.value;
		}
	}
	else if (o2 < o2_lower_thresh.value)			// lean:  increase offsets
	{
		if (local_sum >= local_sum_limit.value)
			global_offset += step_size.value;

		else
		{
			offset_map.localOffset(rpm, air_flow, step_size.value);
			change_map.localOffset(rpm, air_flow, step_size.value);
			local_sum += step_size.value;
		}
	}
}

void FuelTweaker::tweak()
{
	// count time that we've been operating warm
	// count time that we have not been coasting. (O2 sensor warm) 
	time_eng_warm++;		// yeah, this wraps at ~4.5 hours.  That'll stop local optimizations for a few min. bite me. 
	time_warming_o2++;

	if ( !(run_condition & _BV(WARM)) )
	{
		time_eng_warm = 0;
		time_warming_o2 = 0;
		//Serial.print(F(";"));
	}
	if (run_condition & _BV(COASTING))
	{
		time_warming_o2 = 0;
		//Serial.print(F("*"));
	}

	// Set the weak pullup resistor on the O2 sensor input. 
	// todo! 


	// determine mode. ////////////////////

	mode = 0;	// the default, "not tuning" state.
	if (lockout)	// I told yout not to tweak!
		return;

	// if the engine isn't running on it's own fire, don't try to tweak. 
	if (run_condition & (_BV(NOT_RUNNING) | _BV(CRANKING) | _BV(COASTING)))
	{
		time_warming_o2 = 0;
		return;
	}

	// if we're idling, but the O2 sensor or the engine is cold, tweak to maximize RPM
	if ( !(run_condition & _BV(WARM)) || (time_warming_o2 < time_warming_o2_thresh.value))
	{
		if (run_condition & _BV(IDLING))
		{
			tweakGlobalvRPM();
			return;
		}
		else
			return;		// if we're not idling, we can't use RPM as feedback, so don't tweak. 
	}

	// If we're warm, we only make global changes at first.  Then we do both. 
	if (time_eng_warm < time_eng_warm_thresh.value)
		tweakGlobalvO2();
	else
		tweakLocalAndGlobalvO2();
}

const char FuelTweaker::status(void* obj_ptr)
{
	FuelTweaker* self = (FuelTweaker *)obj_ptr;
	Serial.print(F("locked: "));
	Serial.print(self->lockout);
	Serial.print(F("   mode: "));
	Serial.print(self->mode);
	//Serial.print(F("  o2open:"));
	//Serial.print(o2_open);
	Serial.print(F("   direction: "));
	Serial.println(self->direction);
	Serial.print(F("time_warming_o2: "));
	Serial.print(self->time_warming_o2);
	Serial.print(F("   time_eng_warm: "));
	Serial.print(self->time_eng_warm);
	Serial.print(F("   local_sum: "));
	Serial.println(self->local_sum);
}

const char FuelTweaker::reportParams(void* obj_ptr)
{
	FuelTweaker* self = (FuelTweaker *)obj_ptr;
	
	Serial.print(F("o2_upper_thresh: "));
	Serial.print(self->o2_upper_thresh.value);

	Serial.print(F("   o2_lower_thresh: "));
	Serial.print(self->o2_lower_thresh.value);

	Serial.print(F("   time_warming_o2_thresh: "));
	Serial.println(self->time_warming_o2_thresh.value);

	Serial.print(F("step_size: "));
	Serial.print(self->step_size.value);

	Serial.print(F("   local_sum_limit: "));
	Serial.print(self->local_sum_limit.value);

	Serial.print(F("   time_eng_warm_thresh: "));
	Serial.println(self->time_eng_warm_thresh.value);
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

	Serial.print(F("saved Tweaker."));
}

const char FuelTweaker::lock(void* obj_ptr)
{
	FuelTweaker* self = (FuelTweaker *)obj_ptr;
	self->lockout = !self->lockout;
	if (self->lockout)
		Serial.println(F("tweaks locked"));
	else
		Serial.println(F("tweaks unlocked"));
}