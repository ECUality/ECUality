#include "Map.h"
#include "Scale.h"
#include "ECUSerial.h"
#include "Interpolation.h"
#include "EEIndex.h"
#include "EEPROMAnything.h"
#include "Arrays.h"


Map::Map(const Scale* scale_p, unsigned char size, int z_lower_, int z_upper_)
{
	scale = scale_p;
	n = size;
	z_upper = z_upper_;
	z_lower = z_lower_;
	ee_start_address = getEEAddy(300);	
}

Map::~Map()
{
}

const char Map::write(void* obj_ptr)
{
	Map* self = (Map *)obj_ptr;
	int new_z[MAX_MAP_SIZE*MAX_MAP_SIZE];
	int new_n;
	char good = 1;
	
	good &= ESerial.timedParseInt(new_n);
	good &= ESerial.timedReceiveArray(new_z, new_n*new_n, "map data");

	if (!good)
		return 0;

	if (!self->verify(new_z, new_n))
	{
		Serial.println(F("Invalid map - not loaded"));
		return 0;
	}
	self->n = new_n;
	copyArray(new_z, self->z, new_n*new_n);

	ESerial.dumpLine();		// dump any additional characters. 

	read(obj_ptr);
	return 1;
}

const char Map::load(void* obj_ptr)
{
	Map* self = (Map *)obj_ptr;
	int new_z[MAX_MAP_SIZE*MAX_MAP_SIZE];
	int new_n;
	unsigned int address;						// 

	address = self->ee_start_address;
	if (!address)			// address 0 is code for "not a valid address" 
	{
		Serial.println(F("no EE address for Map"));
		return 0;
	}

	address += EEPROM_readAnything(address, new_n);
	address += EEPROM_readAnything(address, new_z);

	if (!self->verify(new_z, new_n))
	{
		Serial.println(F("Invalid map - not loaded"));
		return 0;
	}

	self->n = new_n;
	copyArray(new_z, self->z, new_n*new_n);

	return 1;
}

const char Map::read(void* obj_ptr)
{
	Map* self = (Map *)obj_ptr;
	int i;
	ESerial.reportArray("air_gridline:\n", self->scale->x, self->n);
	ESerial.reportArray("rpm_gridline:\n", self->scale->y, self->n);
	Serial.println(F("map:"));
	for (i = 0; i < self->n; ++i)
	{
		ESerial.reportArray(" ", &self->z[self->n*i], self->n);
	}
	Serial.print("\n");
}

const char Map::save(void* obj_ptr)
{
	Map* self = (Map *)obj_ptr;
	unsigned int address;
	
	address = self->ee_start_address;
	if (!address)			// address 0 is code for "not a valid address" 
	{
		Serial.println(F("no EE address for Map"));
		return 0;
	}

	address += EEPROM_writeAnything(address, self->n);
	address += EEPROM_writeAnything(address, self->z);

	Serial.println(F("saved map to EE"));
}

const char Map::clear(void * obj_ptr)
{
	Map* self = (Map *)obj_ptr;
	clearArray(self->z, MAX_MAP_SIZE*MAX_MAP_SIZE);
}

const int Map::interpolate(unsigned int rpm, unsigned char air, const Map* correction_map)
{
	
	const int* z2;
	int i_air, i_rpm;
	int inj_r0a0, inj_r0a1, inj_r1a0, inj_r1a1, inj_r0ak, inj_r1ak, inj_rkak;  // 
	i_air = findIndexJustAbove(scale->x, air, n); 
	i_rpm = findIndexJustAbove(scale->y, rpm, n);

	if (correction_map)				// if there is a correction map provided, pass it's z to getPoint()
		z2 = correction_map->z;
	else							// otherwise, put pass getPoint() a null so it doesn't add shit. 
		z2 = NULL;				

	// check if both i's are -1 (key was higher than all array elements)
	// check if any of them are 0 (key was lower than lowest element)
	// handle those cases
	//Serial.print("r");
	//Serial.print(i_rpm);
	//Serial.print("a");
	//Serial.println(i_air);

	if (i_air > 0)
	{
		if (i_rpm > 0)			// both rpm and air are on the map, so do bilinear interpolation
		{
			inj_r0a0 = getPoint(i_rpm - 1, i_air - 1, z2);
			inj_r0a1 = getPoint(i_rpm - 1, i_air, z2);
			inj_r1a0 = getPoint(i_rpm, i_air - 1, z2);
			inj_r1a1 = getPoint(i_rpm, i_air, z2);

			inj_r0ak = linearInterp(air, scale->x[i_air - 1], scale->x[i_air], inj_r0a0, inj_r0a1);
			inj_r1ak = linearInterp(air, scale->x[i_air - 1], scale->x[i_air], inj_r1a0, inj_r1a1);
			return linearInterp(rpm, scale->y[i_rpm - 1], scale->y[i_rpm], inj_r0ak, inj_r1ak);
		}
		else if (i_rpm == 0)	// rpm below the map, air is on the map  (1d interpolate along edge)
		{
			return linearInterp(air, scale->x[i_air - 1], scale->x[i_air], getPoint(0, i_air - 1, z2), getPoint(0, i_air, z2));
		}
		else 					// rpm above map, air on the map (1d interpolate along edge)
		{
			return linearInterp(air, scale->x[i_air - 1], scale->x[i_air], getPoint(n - 1, i_air - 1, z2), getPoint(n - 1, i_air, z2));
		}
	}
	else if (i_air == 0)	// air below	
	{
		if (i_rpm > 0)			// rpm on, air below (1d interpolate along edge)
		{
			return linearInterp(rpm, scale->y[i_rpm - 1], scale->y[i_rpm], getPoint(i_rpm - 1, 0, z2), getPoint(i_rpm, 0, z2));
		}
		else if (i_rpm == 0)	// rpm below, air below (corner) 
		{
			return getPoint(0, 0, z2);
		}
		else					// rpm above, air below (corner) 
		{
			return getPoint(n - 1, 0, z2);
		}
	}
	else	// air above
	{
		if (i_rpm > 0)			// rpm on, air above (1d interpolate along edge)
		{
			return linearInterp(rpm, scale->y[i_rpm - 1], scale->y[i_rpm], getPoint(i_rpm - 1, n - 1, z2), getPoint(i_rpm, n - 1, z2));
		}
		else if (i_rpm == 0)	// rpm below, air above (corner) 
		{
			return getPoint(0, n - 1, z2);
		}
		else					// rpm above, air above (corner) 
		{
			return getPoint(n - 1, n - 1, z2);
		}
	}
}

const int Map::getPoint(const char i_rpm, const char i_air, const int z2[])
{
	if ((i_rpm >= n) || (i_rpm < 0) || (i_air >= n) || (i_air < 0))
		return 0;

	if (!z2)	// if z2 is NULL
		return z[i_rpm * n + i_air];
	else
		return (z[i_rpm * n + i_air] + z2[i_rpm * n + i_air]);	// XXX should we be checking z2 == NULL somehow? 
}

// map modification
void Map::localOffset(unsigned int rpm, unsigned int air, long offset)
{
	// here, we apply an offset to the map correction in the locale of our current operation (air_flow and rpm) 
	char i_air, i_rpm;
	int D_rpm, D_air, d_rpm_lowside, d_air_rightside;
	long dz_r0, dz_r1;
	int dz_r0a0, dz_r0a1, dz_r1a0, dz_r1a1;

	// find the location on the map.
	i_air = findIndexJustAbove(scale->x, air, n);
	i_rpm = findIndexJustAbove(scale->y, rpm, n);

	if ((i_air <= 0) || (i_rpm <= 0))		// if we're not operating on the map, fuggedaboutit
		return;								// note: -1 means we're above, 0 means we're below so <= 0 captures both

	// 	offset gets divided into 4 the 4 nearest grid intersections based on where we're currently operating. 
	//  more of the offset goes to the intersections we're closest to. 

	D_rpm = scale->y[i_rpm - 1] - scale->y[i_rpm];	// positive
	D_air = scale->x[i_air - 1] - scale->x[i_air];	// positive

	d_rpm_lowside = rpm - scale->y[i_rpm];			// delta above lower neighbors
	d_air_rightside = air - scale->x[i_air];		// delta to left of neighbors. 

	if ((d_rpm_lowside * 2) > D_rpm)		// closer to the upper neighbors.  Use lower delta for calculations.
	{
		dz_r0 = (offset * d_rpm_lowside) / D_rpm;
		dz_r1 = offset - dz_r0;
	}
	else									// closer to the lower neighbors.  Use upper delta for calculations.
	{
		dz_r1 = (offset * (D_rpm - d_rpm_lowside)) / D_rpm;
		dz_r0 = offset - dz_r1;
	}

	if ((d_air_rightside * 2) > D_air)		// closer to the left neighbors, use right side deltas (larger) 
	{
		dz_r0a0 = (dz_r0 * d_air_rightside) / D_air;
		dz_r0a1 = dz_r0 - dz_r0a0;

		dz_r1a0 = (dz_r1 * d_air_rightside) / D_air;
		dz_r1a1 = dz_r1 - dz_r1a0;
	}
	else									// closer to right neighbors, use left side deltas (larger) 
	{
		dz_r0a1 = (dz_r0 * (D_air - d_air_rightside)) / D_air;
		dz_r0a0 = dz_r0 - dz_r0a1;

		dz_r1a1 = (dz_r1 * (D_air - d_air_rightside)) / D_air;
		dz_r1a0 = dz_r1 - dz_r1a1;
	}

	offsetZPoint(i_rpm - 1, i_air - 1, dz_r0a0);
	offsetZPoint(i_rpm - 1, i_air, dz_r0a1);
	offsetZPoint(i_rpm, i_air - 1, dz_r1a0);
	offsetZPoint(i_rpm, i_air, dz_r1a1);
}

void Map::offsetZPoint(char i_rpm, char i_air, int offset)
{
	if ((i_rpm >= n) || (i_rpm < 0) || (i_air >= n) || (i_air < 0))
	{
		Serial.println(F("index out of bounds"));
		return;
	}
	z[n*i_rpm + i_air] += offset;
}

const bool Map::verify(const int new_z[], const int new_n)
{
	int i;
	bool valid = 1;

	valid &= ((new_n >= MIN_MAP_SIZE) && (new_n <= MAX_MAP_SIZE));
	valid &= (new_n == scale->n);

	for (i = 0; i < (new_n*new_n); i++)
		valid &= ((new_z[i] >= z_lower) && (new_z[i] <= z_upper));

	return valid;
}

unsigned int Map::getEEAddy(unsigned int size)
{

	if (size < sizeof(*this))
		size = sizeof(*this);
	return EE_index.getNewAddress(size);
}

