#pragma once

#include "Defines.h"

class Scale
{
public:
	Scale(unsigned char n_p = 8);
	~Scale();

	static const char receive(void * obj_ptr);
	static const char load(void * obj_ptr);
	static const char save(const void * obj_ptr);
	static const char report(const void* obj_ptr);

	int interpolate( int x_key);	

	int x[MAX_MAP_SIZE], y[MAX_MAP_SIZE];		// arrays that define a line for scaling inputs to outputs. 
	int x_upper, x_lower;						// upper/lower limits for elements of x
	int y_upper, y_lower;						// upper/lower limits for elements of y
	int n;										// the number elements in each array x, y
	unsigned char ee_address_reg_index;			// the index of the ee address 
	unsigned int ee_start_address;

private:
	const bool verify(const int new_x[], const int new_y[], const int new_n);
	unsigned int getEEAddy(unsigned int size);					// returns the last eeprom address 
};

