#pragma once

#include "ECUSerial.h"

class Parameter
{
public:
	Parameter(int minimum, int maximum);
	~Parameter();

	static const char receive(void* obj_ptr);
	static void report(void* obj_ptr);
	static const char load(void* obj_ptr);
	static const char save(void* obj_ptr);

	int value;

private:
	const unsigned int getEEAddy(unsigned int size);
	const bool verify(const int new_value);

	int max_;
	int min_;
	unsigned int ee_address;
};

