#include "Parameter.h"
#include "ECUSerial.h"
#include "EEPROMAnything.h"
#include "EEIndex.h"

Parameter::Parameter(int minimum, int maximum)
{
	min_ = minimum;
	max_ = maximum;
	value = 0;
	ee_address = getEEAddy(4);
}

Parameter::~Parameter()
{
}

const char Parameter::receive(void* obj_ptr)		// Note: this uses parseInt(), so it only really works for int. 
{
	Parameter* self = (Parameter *)obj_ptr;
	char good = 1;
	int new_value;

	good &= ESerial.timedParseInt(new_value);

	if (!good)
	{
		Serial.println(F("not received"));
		return 0;
	}

	if (!self->verify(new_value))
	{
		Serial.println(F("Out of bounds"));
		return 0;
	}

	self->value = new_value;

	ESerial.dumpLine();		// dump any additional characters. 

	report(obj_ptr);
	return 1;
}

const char Parameter::load(void* obj_ptr)
{
	Parameter* self = (Parameter *)obj_ptr;
	char good = 1;
	int new_value;
	unsigned int address;

	address = self->ee_address;
	if (!address)			// address 0 is code for "not a valid address" 
	{
		Serial.println("no EE address for Parameter");
		return 0;
	}

	address += EEPROM_readAnything(address, new_value);

	if (!self->verify(new_value))
	{
		Serial.println(F("Invalid parameter - not loaded"));
		return 0;
	}

	self->value = new_value;

	report(obj_ptr);
	return 1;
}

void Parameter::report(void* obj_ptr)
{
	Serial.println(((Parameter*)obj_ptr)->value);
}

const char Parameter::save(void* obj_ptr)
{
	Parameter* self = (Parameter *)obj_ptr;
	unsigned int address;

	address = self->ee_address;
	if (!address)			// address 0 is code for "not a valid address" 
	{
		Serial.println(F("no EE address for Parameter"));
		return 0;
	}

	address += EEPROM_writeAnything(address, self->value);

	Serial.println(F("saved parameter to EE"));
}

const unsigned int Parameter::getEEAddy(unsigned int size)
{
	if (size < sizeof(*this))		// make certain the default is large enough
		size = sizeof(*this);
	return EE_index.getNewAddress(size);
}

const bool Parameter::verify(const int new_value)
{
	return ((new_value >= min_) && (new_value <= max_));
}