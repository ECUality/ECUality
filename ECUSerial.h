#pragma once
#include "HardwareSerial.h"

#define SERIAL_TIMEOUT 50
#define MAX_COMMANDS 30
#define N_CMD_CHARS 5		// so we can use 4 characters

class ECUSerial
{
	// a pointer to a function that takes, as an argument, a pointer to an object.  (super generic) 
	// these are used to call static member functions belonging to classes which perform various 
	// actions on objects belonging to their respective classes.  For example, to call "receive", 
	// which lives inside the "Map" class, to read new map data from the serial port into a specific
	// Map object. 
	const void (*fun_ptr[MAX_COMMANDS])(void *);	

	// the pointers to the objects that are passed to the above functions. 
	void *obj_ptr[MAX_COMMANDS];

	char command_str[N_CMD_CHARS];

	char n_commands;

public:
	ECUSerial();
	~ECUSerial();

	void executeCommand();

	char addCommand(const void(*function_ptr)(void*), void* object, const char name[N_CMD_CHARS]);

	char timedParseInt(int &value, unsigned char timeout = 55);

	char timedReceiveArray(int new_array[], const unsigned int n_array, const char str[] = "");

	void dumpLine(void)
	{
		char str[5];
		while (Serial.readBytesUntil('\n', str, 4));
	}

	template <typename T>
	void reportArray(char str[], T data[], unsigned int n)
	{
		int i;
		Serial.print(str);
		Serial.print("\t");
		for (i = 0; i < n; ++i)
		{
			Serial.print(data[i]);
			Serial.print("\t");
		}
		Serial.println();
	}

	template <typename T>
	char receiveNumberBetween(T *var, const int lower, const int upper, char var_name[])
	{
		unsigned int new_value = Serial.parseInt();
		if (new_value > upper)
		{
			Serial.print("too many ");
			Serial.println(var_name);
			return 0;
		}
		if (new_value < lower)
		{
			Serial.print("too few");
			Serial.println(var_name);
			return 0;
		}
		*var = new_value;
		return 1;
	}
};

extern ECUSerial ESerial;

