#include "Scale.h"
#include "ECUSerial.h"
#include "Interpolation.h"
#include "EEIndex.h"
#include "EEPROMAnything.h"
#include "Arrays.h"


Scale::Scale(int x_lower_, int x_upper_, int y_lower_, int y_upper_, unsigned char n_p)
{
	n = n_p;
	x_lower = x_lower_;
	x_upper = x_upper_;
	y_lower = y_lower_;
	y_upper = y_upper_;
	
	// 100 bytes for easy reading.  Should need SCALE_SIZE_MAX * 2(bytes per int) * 2(two arrays) = 80
	ee_address = getEEAddy(100);
}


Scale::~Scale()
{
}

const char Scale::write(void* obj_ptr)
{
	Scale* self = (Scale *)obj_ptr;
	char good = 1;
	int new_n, new_x[SCALE_SIZE_MAX], new_y[SCALE_SIZE_MAX];

	good &= ESerial.timedParseInt(new_n);
	good &= ESerial.timedReceiveArray(new_x, new_n, "x gridline");
	good &= ESerial.timedReceiveArray(new_y, new_n, "y gridline");

	if (!good)
		return 0;

	self->n = new_n;
	copyArray(new_x, self->x, new_n);
	copyArray(new_y, self->y, new_n);

	ESerial.dumpLine();		// dump any additional characters. 

	read(obj_ptr);
	return 1;
}

const char Scale::load(void* obj_ptr)
{
	Scale* self = (Scale *)obj_ptr;
	int new_n, new_x[SCALE_SIZE_MAX], new_y[SCALE_SIZE_MAX];
	unsigned int address;

	address = self->ee_address;
	if (!address)			// address 0 is code for "not a valid address" 
	{
		Serial.println(F("no EE address for Scale"));
		return 0;
	}
	
	address += EEPROM_readAnything(address, new_n);
	address += EEPROM_readAnything(address, new_x);
	address += EEPROM_readAnything(address, new_y);

	if (!self->verify(new_x, new_y, new_n))
	{
		Serial.println(F("Invalid scale - not loaded"));
		return 0;
	}

	self->n = new_n;
	copyArray(new_x, self->x, new_n);
	copyArray(new_y, self->y, new_n);

	return 1;
}

const char Scale::save(void * obj_ptr)
{
	Scale* self = (Scale *)obj_ptr;
	unsigned int address;

	address = self->ee_address;
	if (!address)			// address 0 is code for "not a valid address" 
	{
		Serial.println(F("no EE address for Scale"));
		return 0;
	}

	address += EEPROM_writeAnything(address, self->n);
	address += EEPROM_writeAnything(address, self->x);
	address += EEPROM_writeAnything(address, self->y);

	Serial.println(F("saved scale to EE"));
}

const char Scale::read(void* obj_ptr)
{
	Scale* self = (Scale *)obj_ptr;
	ESerial.reportArray("x: ", self->x, self->n);
	ESerial.reportArray("y: ", self->y, self->n);
	Serial.println();
}

int Scale::interpolate(int x_key)
{
	char i;
	i = findIndexJustAbove(x, x_key, n);

	if (i > 0)	// reading is on the graph
		return linearInterp(x_key, x[i - 1], x[i], y[i - 1], y[i]);

	else if (i == 0)	// x_key is to left of graph.  Use leftmost points
		return linearInterp(x_key, x[0], x[1], y[0], y[1]);

	else				// x_key is to right of graph. Use rightmost points.
		return linearInterp(x_key, x[n - 1], x[n], y[n - 1], y[n]);
}

const bool Scale::verify(const int new_x[], const int new_y[], const int new_n)
{
	int i;
	bool valid = 1;

	valid &= ((new_n >= SCALE_SIZE_MIN) && (new_n <= SCALE_SIZE_MAX));

	for (i = 0; i < new_n; i++)
	{
		valid &= ((new_x[i] >= x_lower) && (new_x[i] <= x_upper));
		valid &= ((new_y[i] >= y_lower) && (new_y[i] <= y_upper));
	}

	return valid;
}

const unsigned int Scale::getEEAddy(unsigned int size)
{
	if (size < sizeof(*this))
		size = sizeof(*this);
	return EE_index.getNewAddress(size);
}