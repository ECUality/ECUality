#pragma once

#include "ECUSerial.h"

class Parameter
{
public:
	Parameter(const char* name_, const char handle_[4], int minimum, int maximum);
	~Parameter();

	static const char write(void* obj_ptr);
	static const char read(void* obj_ptr);
	static const char load(void* obj_ptr);
	static const char save(void* obj_ptr);
	static const char clear(void* obj_ptr);

	static Parameter* params[40];		// an array of pointers to Parameter objects
	static unsigned char n_params;		// number of Parameter objects

	int value;

	const char* name;			// Used for referring to this object in serial messages.
	const char* handle;		// Used to access Read, Write, Save functions in protocol. 

private:
	const unsigned int getEEAddy(unsigned int size);
	const bool verify(const int new_value);

	int max;
	int min;
	unsigned int ee_address;
	
};

