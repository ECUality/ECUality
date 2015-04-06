#include "Map.h"
#include "ECUSerial.h"
#include "Interpolation.h"
#include "EEIndex.h"
#include "EEPROMAnything.h"
#include "Arrays.h"


Map::Map(unsigned char size, int air_upper_p, int air_lower_p,
	int rpm_upper_p, int rpm_lower_p, int z_upper_p, int z_lower_p)
	: ee_start_address(getLastEEAddy()),
	air_upper(air_upper_p),
	air_lower(air_lower_p),
	rpm_lower(rpm_lower_p),
	rpm_upper(rpm_upper_p),
	z_upper(z_upper_p),
	z_lower(z_lower_p)

{
	n_air = size;
	n_rpm = size;
}

Map::~Map()
{
}

const char Map::receive(void* obj_ptr)
{
	Map* self = (Map *)obj_ptr;
	char str[3] = "";	// all zeros.
	char good = 1;
	Table2d buff; 
	
	unsigned int new_map_size;

	Serial.readBytes(str, 2);

	if (strcmp(str, "ap") != 0)
	{
		Serial.println("spell 'map' please");
		return -1;
	}
	
	good &= ESerial.timedParseInt(buff.n_air);
	good &= ESerial.timedParseInt(buff.n_rpm);
	good &= ESerial.timedReceiveArray(buff.air_gridline, buff.n_air, "air gridline");
	good &= ESerial.timedReceiveArray(buff.rpm_gridline, buff.n_rpm, "rpm gridline");
	new_map_size = buff.n_air * buff.n_rpm;
	good &= ESerial.timedReceiveArray(buff.z, new_map_size, "map data");

	if (!good)
		return 0;

	self->copy(buff);
	ESerial.dumpLine();		// dump any additional characters. 

	report(obj_ptr);
	return 1;
}

const char Map::report(const void* obj_ptr)
{
	Map* self = (Map *)obj_ptr;
	int i;
	ESerial.reportArray("air_gridline:\n", air_gridline, n_air);
	ESerial.reportArray("rpm_gridline:\n", rpm_gridline, n_rpm);
	Serial.println("engine_map:");
	for (i = 0; i < n_rpm; ++i)
	{
		ESerial.reportArray(" ", &self->z[n_air*i], n_air);
	}
	Serial.print("\n");
}

const char Map::load(void* obj_ptr)
{
	Map* self = (Map *)obj_ptr;
	unsigned int address;						// 

	Table2d buff;		// 242 bytes. 

	address = self->ee_start_address;			// sets the beginnning. 

	address += EEPROM_readAnything(address, buff.n_air);				
	address += EEPROM_readAnything(address, buff.n_rpm);				
	address += EEPROM_readAnything(address, buff.air_gridline);		
	address += EEPROM_readAnything(address, buff.rpm_gridline);		
	address += EEPROM_readAnything(address, buff.z);		

	if (!self->verify(buff))
	{
		Serial.println(F("Invalid map - not loaded"));
		return 0;
	}

	self->copy(buff);

	Serial.println("loaded map from EE.");
	return 1;
}

const char Map::save(const void* obj_ptr)
{
	Map* self = (Map *)obj_ptr;
	// eeprom addresses: 0 = n_air, n_rpm
	unsigned int address, map_size;
	address = self->ee_start_address;

	address += EEPROM_writeAnything(address, n_air);
	address += EEPROM_writeAnything(address, n_rpm);
	address += EEPROM_writeAnything(address, air_gridline);
	address += EEPROM_writeAnything(address, rpm_gridline);
	address += EEPROM_writeAnything(address, self->z);

	Serial.println("saved map to EE");
}

const int Map::interpolate(unsigned int rpm, unsigned char air, const int z2[])
{
	int i_air, i_rpm;
	int inj_r0a0, inj_r0a1, inj_r1a0, inj_r1a1, inj_r0ak, inj_r1ak, inj_rkak;  // 
	i_air = findIndexJustAbove(air_gridline, air, n_air);
	i_rpm = findIndexJustAbove(rpm_gridline, rpm, n_rpm);

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

			inj_r0ak = linearInterp(air, air_gridline[i_air - 1], air_gridline[i_air], inj_r0a0, inj_r0a1);
			inj_r1ak = linearInterp(air, air_gridline[i_air - 1], air_gridline[i_air], inj_r1a0, inj_r1a1);
			return linearInterp(rpm, rpm_gridline[i_rpm - 1], rpm_gridline[i_rpm], inj_r0ak, inj_r1ak);
		}
		else if (i_rpm == 0)	// rpm below the map, air is on the map  (1d interpolate along edge)
		{
			return linearInterp(air, air_gridline[i_air - 1], air_gridline[i_air], getPoint(0, i_air - 1, z2), getPoint(0, i_air, z2));
		}
		else 					// rpm above map, air on the map (1d interpolate along edge)
		{
			return linearInterp(air, air_gridline[i_air - 1], air_gridline[i_air], getPoint(n_rpm - 1, i_air - 1, z2), getPoint(n_rpm - 1, i_air, z2));
		}
	}
	else if (i_air == 0)	// air below	
	{
		if (i_rpm > 0)			// rpm on, air below (1d interpolate along edge)
		{
			return linearInterp(rpm, rpm_gridline[i_rpm - 1], rpm_gridline[i_rpm], getPoint(i_rpm - 1, 0, z2), getPoint(i_rpm, 0, z2));
		}
		else if (i_rpm == 0)	// rpm below, air below (corner) 
		{
			return getPoint(0, 0, z2);
		}
		else					// rpm above, air below (corner) 
		{
			return getPoint(n_rpm - 1, 0, z2);
		}
	}
	else	// air above
	{
		if (i_rpm > 0)			// rpm on, air above (1d interpolate along edge)
		{
			return linearInterp(rpm, rpm_gridline[i_rpm - 1], rpm_gridline[i_rpm], getPoint(i_rpm - 1, n_air - 1, z2), getPoint(i_rpm, n_air - 1, z2));
		}
		else if (i_rpm == 0)	// rpm below, air above (corner) 
		{
			return getPoint(0, n_air - 1, z2);
		}
		else					// rpm above, air above (corner) 
		{
			return getPoint(n_rpm - 1, n_air - 1, z2);
		}
	}
}

const int Map::getPoint(const char i_rpm, const char i_air, const int z2[])
{
	if ((i_rpm >= n_rpm) || (i_rpm < 0) || (i_air >= n_air) || (i_air < 0))
		return 0;

	return (z[i_rpm * n_air + i_air] + z2[i_rpm * n_air + i_air]);	// XXX should we be checking z2 == NULL somehow? 
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
	i_air = findIndexJustAbove(air_gridline, air, n_air);
	i_rpm = findIndexJustAbove(rpm_gridline, rpm, n_rpm);

	if ((i_air <= 0) || (i_rpm <= 0))		// if we're not operating on the map, fuggedaboutit
		return;								// note: -1 means we're above, 0 means we're below so <= 0 captures both

	// 	offset gets divided into 4 the 4 nearest grid intersections based on where we're currently operating. 
	//  more of the offset goes to the intersections we're closest to. 

	D_rpm = rpm_gridline[i_rpm - 1] - rpm_gridline[i_rpm];	// positive
	D_air = air_gridline[i_air - 1] - air_gridline[i_air];	// positive

	d_rpm_lowside = rpm - rpm_gridline[i_rpm];			// delta above lower neighbors
	d_air_rightside = air - air_gridline[i_air];		// delta to left of neighbors. 

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
	if ((i_rpm >= n_rpm) || (i_rpm < 0) || (i_air >= n_air) || (i_air < 0))
	{
		Serial.println("index out of bounds");
		return;
	}
	z[n_air*i_rpm + i_air] += offset;
}

void Map::copy(const Table2d &buff)
{
	n_air = buff.n_air;
	n_rpm = buff.n_rpm;
	copyArray(buff.air_gridline, air_gridline, n_air);
	copyArray(buff.rpm_gridline, rpm_gridline, n_rpm);
	copyArray(buff.z, z, n_air*n_rpm);
}

const bool Map::verify(Table2d &data)
{
	int i;
	bool valid = 1;

	valid &= ((data.n_air >= MIN_MAP_SIZE) && (data.n_air <= MAX_MAP_SIZE));
	valid &= ((data.n_rpm >= MIN_MAP_SIZE) && (data.n_rpm <= MAX_MAP_SIZE));

	for (i = 0; i < data.n_air; i++)
		valid &= ((data.air_gridline[i] >= air_lower) && (data.air_gridline[i] <= air_upper));

	for (i = 0; i < data.n_rpm; i++)
		valid &= ((data.rpm_gridline[i] >= rpm_lower) && (data.rpm_gridline[i] <= rpm_upper));

	for (i = 0; i < (data.n_air*data.n_rpm); i++)
		valid &= ((data.z[i] >= z_lower) && (data.z[i] <= z_upper));

	return valid;
}

unsigned int Map::getLastEEAddy()
{
	unsigned int size = sizeof(*this);
	n_ee_addresses++;
	start_addresses[n_ee_addresses] = start_addresses[n_ee_addresses-1] + size;
	return start_addresses[n_ee_addresses - 1];
}

int Map::n_air = 8;
int Map::n_rpm = 8;
int Map::air_gridline[MAX_MAP_SIZE] = { 0 };
int Map::rpm_gridline[MAX_MAP_SIZE] = { 0 };