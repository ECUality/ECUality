// ESerial functions

#include "Globals.h"
#include "EEIndex.h"
#include "EEPROMAnything.h"
#include "ECUPins.h"


char param_to_edit;				// the parameter or scale selected for adjustment 
char index_to_edit;				// the index of the scale Y-point selected for adjustment by dash terminal 
unsigned int adjustment_size;		// how much a parameter changes from one up/down button press.

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
	unsigned char i;

	for (i = 0; i < Parameter::n_params; i++)
	{
		Parameter::save(Parameter::params[i]);
	}

	for (i = 0; i < Scale::n_scales; i++)
	{
		Scale::save(Scale::scales[i]);
	}

	Map::save(&inj_map);
}
const char loadData(void* obj_ptr)
{
	char good = 1;
	unsigned char i;

	for (i = 0; i < Parameter::n_params; i++)
	{
		good &= Parameter::load(Parameter::params[i]);
	}

	for (i = 0; i < Scale::n_scales; i++)
	{
		good &= Scale::load(Scale::scales[i]);
	}
	
	good &= Map::load(&inj_map);
	good &= Map::load(&offset_map);

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
	unsigned char i;

	// print all Parameter objects
	for (i = 0; i < Parameter::n_params; i++)
	{
		Parameter::read(Parameter::params[i]);
	}
	// print all Scale objects
	for (i = 0; i < Scale::n_scales; i++)
	{
		Scale::read(Scale::scales[i]);
	}

	// print inj Map
	Map::read(&inj_map);

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


void enableDrive()
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
void disableDrive()
{
	digitalWrite(fuel_pin, LOW);
	digitalWrite(drv_en_pin, HIGH);		// this turns off the injectors and ignition
	//detachInterrupt(4);
	interjectln(F("disarmed"));
}
const char toggleEnable(void* obj_ptr) {
	if (digitalRead(fuel_pin))		// if the fuel pump is on, turn it off. 
	{
		disableDrive();
		EEPROM_writeAnything(ENABLE_ADDY, 0);		// this way, we wake in the same state for theft prevention 
	}
	else
	{
		enableDrive();
		EEPROM_writeAnything(ENABLE_ADDY, 1);
	}
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

void displayScalePoint(Scale* selected_scale) {
	ESerial.print("!");			// bottom row. 
	ESerial.print(selected_scale->handle);
	ESerial.print(F(" x"));
	ESerial.print(selected_scale->x[index_to_edit]);
	ESerial.print(F(" y"));
	ESerial.println(selected_scale->y[index_to_edit]);
}
void displaySelectedParam() {
	if (param_to_edit < Parameter::n_params)
	{
		ESerial.print("!");
		ESerial.println(Parameter::params[param_to_edit]->name);
	}
	else
		displayScalePoint(Scale::scales[param_to_edit - Parameter::n_params]);
}

const char selectNextParamToEdit(void* obj_ptr) {

	index_to_edit = 0;

	if (++param_to_edit >= (Parameter::n_params + Scale::n_scales))
		param_to_edit = 0;
	
	displaySelectedParam();
}
const char selectPrevParamToEdit(void* obj_ptr) {

	index_to_edit = 0;

	if (--param_to_edit < 0)
		param_to_edit = (Parameter::n_params + Scale::n_scales - 1);

	displaySelectedParam();
}
const char selectNextPointToEdit(void* obj_ptr) {

	if (param_to_edit >= Parameter::n_params)		// only increment if we have selected a scale
	{
		if (++index_to_edit >= Scale::scales[param_to_edit - Parameter::n_params]->n)
			index_to_edit = 0;

		displayScalePoint(Scale::scales[param_to_edit - Parameter::n_params]);
	}

}

void adjustParameter(unsigned char going_up) {
	
	if (param_to_edit < Parameter::n_params)		// we're editing a parameter
	{
		if (going_up)
			Parameter::params[param_to_edit]->value += adjustment_size; 
		else 
			Parameter::params[param_to_edit]->value -= adjustment_size;

		ESerial.print("!");			// bottom row. 
		ESerial.print(Parameter::params[param_to_edit]->handle);
		ESerial.print(F(": "));
		ESerial.println(Parameter::params[param_to_edit]->value);
	}
	else	// we're editing a scale 
	{
		if (going_up)
			Scale::scales[param_to_edit-Parameter::n_params]->y[index_to_edit] += adjustment_size;
		else 
			Scale::scales[param_to_edit - Parameter::n_params]->y[index_to_edit] -= adjustment_size;

		displayScalePoint(Scale::scales[param_to_edit - Parameter::n_params]);
	}
}
const char increaseParameter(void* obj_ptr) {
	adjustParameter(1);
}
const char decreaseParameter(void* obj_ptr) {
	adjustParameter(0);
}

const char increaseAdjustmentSize(void* obj_ptr) {
	if (adjustment_size <= 32)
		adjustment_size *= 4;	// adjustment_size *= 4;
	ESerial.print(F("!step-size: "));
	ESerial.println(adjustment_size);
}
const char decreaseAdjustmentSize(void *obj_ptr) {
	if (adjustment_size >= 4)
		adjustment_size /= 4;	
	ESerial.print(F("!step-size: "));
	ESerial.println(adjustment_size);
}

void initProtocol()
{
	char cmd[5];
	unsigned char i;

	param_to_edit = 0;
	index_to_edit = 0;
	adjustment_size = 1; 

	ESerial.addCommand(F("arm"), toggleEnable, NULL);
	ESerial.addCommand(F("auto"), setReportMode, NULL);
	ESerial.addCommand(F("lock"), FuelTweaker::lock, &boss);
	ESerial.addCommand(F("mode"), reportMode, NULL);			// 4

	ESerial.addCommand(F("+"), increaseParameter, NULL);
	ESerial.addCommand(F("-"), decreaseParameter, NULL);
	ESerial.addCommand(F("stat"), reportStatus, NULL);
	ESerial.addCommand(F("para"), reportParams, NULL);
	ESerial.addCommand(F("save"), saveData, NULL);				// 9

	ESerial.addCommand(F("load"), loadData, NULL);
	ESerial.addCommand(F("ee"), reportEEAddresses, NULL);
	ESerial.addCommand(F("task"), reportTaskTimes, NULL);
	ESerial.addCommand(F("mem"), memory, NULL);					//14

	ESerial.addCommand(F("nxtp"), selectNextParamToEdit, NULL);
	ESerial.addCommand(F("prep"), selectPrevParamToEdit, NULL);
	ESerial.addCommand(F("nxti"), selectNextPointToEdit, NULL);
	ESerial.addCommand(F("dbl"), increaseAdjustmentSize, NULL);
	ESerial.addCommand(F("hlf"), decreaseAdjustmentSize, NULL);		// 19


	ESerial.addCommand(F("rbos"), FuelTweaker::status, &boss);
	ESerial.addCommand(F("pbos"), FuelTweaker::reportParams, &boss);	// 21

	ESerial.addCommand(F("Winj"), Map::write, &inj_map);	// write a new injector map from what I send you next
	ESerial.addCommand(F("rinj"), Map::read, &inj_map);		// report the injector map to the serial port
	ESerial.addCommand(F("Sinj"), Map::save, &inj_map);		// save the injector map to EEPROM
	ESerial.addCommand(F("linj"), Map::load, &inj_map);		// load theh injector map from EEPROM - 25

	ESerial.addCommand(F("Wloc"), Map::write, &offset_map);
	ESerial.addCommand(F("rloc"), Map::read, &offset_map);
	ESerial.addCommand(F("Sloc"), Map::save, &offset_map);
	ESerial.addCommand(F("lloc"), Map::load, &offset_map);
	ESerial.addCommand(F("Cloc"), Map::clear, &offset_map);		// - 30

	ESerial.addCommand(F("rchg"), Map::read, &change_map);
	ESerial.addCommand(F("Cchg"), Map::clear, &change_map);
	ESerial.addCommand(F("Cglo"), Parameter::clear, &global_offset);	// - 33  Currently max is 50

}