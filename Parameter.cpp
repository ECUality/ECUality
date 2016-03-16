#include "Parameter.h"
#include "ECUSerial.h"
#include "EEPROMAnything.h"
#include "EEIndex.h"

Parameter::Parameter(const char handle_[4], int minimum, int maximum)
{
	handle = handle_;
	setName(F("?"));		// for now, put something here to make dependent functions work. 
	min = minimum;
	max = maximum;
	value = 0;

	ee_address = getEEAddy(4);

	// params[] is an array of pointers to Parameter objects.  Here we add a new pointer to the list.
	params[n_params] = this;		
	n_params++;					// now we advance the number of objects.

}

Parameter::~Parameter()
{
}

const char Parameter::write(void* obj_ptr)		// Note: this uses parseInt(), so it only really works for int. 
{
	Parameter* self = (Parameter *)obj_ptr;
	char good = 1;
	int new_value;

	good &= ESerial.timedParseInt(new_value);

	if (!good)
	{
		ESerial.println(F("not received"));
		return 0;
	}

	if (!self->verify(new_value))
	{
		ESerial.println(F("Out of bounds"));
		return 0;
	}

	self->value = new_value;

	read(obj_ptr);
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
		ESerial.print(F("no EE adress for "));
		ESerial.println(self->name);
		return 0;
	}

	address += EEPROM_readAnything(address, new_value);

	if (!self->verify(new_value))
	{
		ESerial.print(F("Invalid value encountered loading "));
		ESerial.println(self->name);
		return 0;
	}

	self->value = new_value;

	return 1;
}

const char Parameter::read(void* obj_ptr)
{
	Parameter* self = (Parameter *)obj_ptr;
	ESerial.print("W");
	ESerial.print(self->handle);
	ESerial.print(" ");
	ESerial.print(self->name);
	ESerial.print(": ");
	ESerial.println(self->value);
	return 0;
}

const char Parameter::save(void* obj_ptr)
{
	Parameter* self = (Parameter *)obj_ptr;
	unsigned int address;

	address = self->ee_address;
	if (!address)			// address 0 is code for "not a valid address" 
	{
		ESerial.println(F("no EE adr Parameter"));
		return 0;
	}

	address += EEPROM_writeAnything(address, self->value);

	ESerial.print(F("!"));
	ESerial.print(self->name);
	ESerial.println(F(" saved"));
}

const char Parameter::clear(void* obj_ptr)
{
	Parameter* self = (Parameter *)obj_ptr;
	if (self->min < 0)
		self->value = 0;
	else
		self->value = self->min;
}

void Parameter::setName(const __FlashStringHelper* name_) {
	name = name_;
}

const unsigned int Parameter::getEEAddy(unsigned int size)
{
	if (size < sizeof(*this))		// make certain the default is large enough
		size = sizeof(*this);
	return EE_index.getNewAddress(size);
}

const bool Parameter::verify(const int new_value)
{
	return ((new_value >= min) && (new_value <= max));
}

Parameter* Parameter::params[] = {};		// an array of pointers to all Parameter objects
unsigned char Parameter::n_params = 0;		// the total number of Parameter objects
