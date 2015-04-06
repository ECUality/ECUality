#pragma once

#include "Defines.h"
#include "ECUSerial.h"

#define MAX_MAP_SIZE 10
#define MIN_MAP_SIZE 3
#define NIL -3000

struct Table2d {
	int n_rpm, n_air;
	int air_gridline[MAX_MAP_SIZE];
	int rpm_gridline[MAX_MAP_SIZE];
	int z[MAX_MAP_SIZE*MAX_MAP_SIZE];
};

class Map
{
public:
	explicit Map(unsigned char size = 8, int air_upper = 255, int air_lower = 0, 
		int rpm_upper = 7000, int rpm_lower = 60, int z_upper = 3500, int z_lower = 300);
	~Map();
	
	static const char receive(void * obj_ptr);
	static const char report(const void* obj_ptr);
	static const char save(const void * obj_ptr);
	static const char load(void * obj_ptr);
	const int interpolate(unsigned int rpm, unsigned char air, const int z2[] = NULL);		// default value permits adding a variable map
	const int getPoint(const char i_rpm, const char i_air, const int z2[] = NULL);
	void localOffset(unsigned int rpm, unsigned int air, long offset);
	void offsetZPoint(char i_rpm, char i_air, int offset);
	void registerCommand(const char c[]);
	void copy(const Table2d &buff); 


	int z[MAX_MAP_SIZE*MAX_MAP_SIZE];

	int air_upper;
	int air_lower;
	int rpm_upper;
	int rpm_lower;
	int z_upper;
	int z_lower;
	static int n_rpm, n_air;
	unsigned char ee_address_reg_index;		// the index of the ee address 

private:
	unsigned int ee_start_address; 

	static int air_gridline[MAX_MAP_SIZE], rpm_gridline[MAX_MAP_SIZE];

	const bool verify(Table2d &temp_buffer);
	unsigned int getLastEEAddy();					// returns the last eeprom address stored in the global variable ee_addresses
};


