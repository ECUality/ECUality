// ESerial functions

#include "Globals.h"
#include "EEIndex.h"
#include "ECUPins.h"

template<typename T>
void interject(T input)
{
	ESerial.print(input);

	// delay the next autoreport 1 sec to ensure this message is noticeable
	ms_since_last_task[i_autoreport] = 0;
}

template<typename T>
void interjectln(T input)
{
	ESerial.println(input);

	// delay the next autoreport 1 sec to ensure this message is noticeable
	ms_since_last_task[i_autoreport] = 0;
}

const char saveData(void* obj_ptr)
{
	Parameter::save(&coasting_rpm);
	Parameter::save(&idling_rpm);
	Parameter::save(&air_thresh);
	Parameter::save(&cold_threshold);
	Parameter::save(&cranking_dur);
	Parameter::save(&idle_dur);
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
	good &= Parameter::load(&idle_dur);
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

const char reportMode(void* obj_ptr)
{
	if (run_condition & _BV(NOT_RUNNING))
		ESerial.print(F("stop,  "));
	else if (run_condition & _BV(CRANKING))
		ESerial.print(F("crank,  "));
	else if (run_condition & _BV(COASTING))
		ESerial.print(F("coast,  "));
	else if (run_condition & _BV(IDLING))
		ESerial.print(F("idle,  "));
	else if (run_condition & _BV(WIDE_OPEN))
		ESerial.print(F("wide"));
	else
		ESerial.print(F("pull,  "));

	/*
	if (run_condition & _BV(WARM))
	ESerial.println(F("warm"));
	else
	ESerial.println(F("cold"));
	*/
}

void padLastWord(int n, int f)
{
	while (n < f)
	{
		ESerial.print(" ");
		n++;
	}
}
const char reportStatus(void* obj_ptr)
{
	size_t n;
	switch (status_mode) 
	{
	case 1: 
		ESerial.print(F("r"));
		n = ESerial.print(avg_rpm.average);
		padLastWord(n, 5);

		ESerial.print(F("O"));
		n = ESerial.print(o2);
		padLastWord(n, 4);

		ESerial.print(F("i"));
		n = ESerial.println(inj_duration);

		break;

	case 2:
		ESerial.print(F("glo "));
		ESerial.print(global_offset.value);
		ESerial.print(F(" loc "));
		ESerial.println(offset_map.interpolate(rpm, air_flow, NULL));
		break;

	case 3:
		ESerial.print(F("temp "));
		ESerial.print(coolant_temp);
		ESerial.print(F(" Oil "));
		ESerial.println(oil_pressure);
		break;

	case 4:
		ESerial.print(F("Twk "));
		ESerial.print(boss.mode);
		ESerial.print(" ");
		reportMode(NULL);
		ESerial.println();
		break;

	case 5:
		ESerial.print(F("air "));
		ESerial.println(air_flow);
		break;

	case 6:
		ESerial.print(F("A"));
		ESerial.print(n_air_faults);
		ESerial.print(F(" R+"));
		ESerial.print(n_rpm_hi_faults);
		ESerial.print(F(" R-"));
		ESerial.println(n_rpm_lo_faults);
		break;

	case 7:
		break;

	case 8:
		break;

	case 9:
		break;

	default: 
		break;

	}	
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
const char setReportMode(void* obj_ptr)
{
	int new_mode;
	if (ESerial.timedParseInt(new_mode) && new_mode > 0 && new_mode <= 9)
		status_mode = new_mode; 
		
}


const char increaseGlobal(void* obj_ptr)
{
	global_offset.value += 10;
	interject("+");	
}
const char decreaseGlobal(void* obj_ptr)
{
	global_offset.value -= 10;
	interject("-");
}
const char enableDrive(void* obj_ptr)
{
	if (good_ee_loads)
	{
		digitalWrite(fuel_pin, HIGH);
		digitalWrite(drv_en_pin, LOW);
		//attachInterrupt(4, isrTachRisingEdge, RISING);	// interrupt 4 maps to pin 19. 
		interjectln(F("Armed."));
	}
	else
		interjectln(F("bad data, no go"));

}
const char disableDrive(void* obj_ptr)
{
	digitalWrite(fuel_pin, LOW);
	digitalWrite(drv_en_pin, HIGH);		// this turns off the injectors and ignition
	//detachInterrupt(4);
	interjectln(F("disarmed"));
}
const char memory(void* obj_ptr)
{
	extern int __heap_start, *__brkval;
	int free_ram, v;
	free_ram = (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);

	ESerial.print(F("Free ram: "));
	interjectln(free_ram);
	return 0;
}


void initProtocol()
{
	ESerial.addCommand(F("x"), disableDrive, NULL);
	ESerial.addCommand(F("arm"), enableDrive, NULL);
	ESerial.addCommand(F("auto"), setReportMode, NULL);
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

	ESerial.addCommand(F("Widl"), Parameter::write, &idle_dur);
	ESerial.addCommand(F("ridl"), Parameter::read, &idle_dur);
	ESerial.addCommand(F("Sidl"), Parameter::save, &idle_dur);
	ESerial.addCommand(F("lidl"), Parameter::load, &idle_dur);

	ESerial.addCommand(F("Wids"), Parameter::write, &idle_slope);
	ESerial.addCommand(F("rids"), Parameter::read, &idle_slope);
	ESerial.addCommand(F("Sids"), Parameter::save, &idle_slope);
	ESerial.addCommand(F("lids"), Parameter::load, &idle_slope);

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