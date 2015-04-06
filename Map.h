#pragma once

#include "Defines.h"
#include "ECUSerial.h"
#include "Scale.h"


class Map
{
public:
	explicit Map(const Scale* scale_p, unsigned char size = 8, int z_upper = 3500, int z_lower = 300);
	~Map();
	
	static const char receive(void * obj_ptr);
	static const char report(const void* obj_ptr);
	static const char save(const void * obj_ptr);
	static const char load(void * obj_ptr);
	static const char clear(void * obj_ptr);

	const int interpolate(unsigned int rpm, unsigned char air, const Map* = NULL);		// last param for correction maps
	const int getPoint(const char i_rpm, const char i_air, const int z2[] = NULL);
	void localOffset(unsigned int rpm, unsigned int air, long offset);
	void offsetZPoint(char i_rpm, char i_air, int offset);


	int z[MAX_MAP_SIZE*MAX_MAP_SIZE];			// 100 ints 
	int z_upper, z_lower, n;					// 3 ints
	unsigned char ee_address_reg_index;		// the index of the ee address 
	const Scale* scale;

private:
	unsigned int ee_start_address; 

	const bool verify(const int new_z[], const int new_n);
	unsigned int getEEAddy(unsigned int size);					// returns the last eeprom address stored in EEIndex
};
// number of ints = MAX_MAP_SIZE*MAX_MAP_SIZE + 8
// number of chars = 1; 

