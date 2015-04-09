#pragma once
#include "HardwareSerial.h"

#define SERIAL_TIMEOUT 50
#define N_COMMANDS_MAX 99
#define N_CMD_CHARS 5		// so we can use 4 characters

class ECUSerial
{
	// class contains 3 lists.  1: list of function pointers, 2: list of object pointers, 3: list of strings.
	// the function pointers and object pointers together define an action happening to a specific object.  
	// the function takes, as an argument, a pointer to the object the function is operating on.  
	// These listed functions are static member functions, which lets them be called this way.  
	// For example, to call "write" on the Map map1, the static member fuction "write" belonging to "Map" class
	// is called and passed a pointer to map1.  

	const char (*fun_ptr[N_COMMANDS_MAX])(void *);	// the function pointers
	void* obj_ptr[N_COMMANDS_MAX];					// the object pointers that are passed to the above functions. 
	char command_str[N_COMMANDS_MAX][N_CMD_CHARS];	// the command strings (5 bytes) for each command
	char n_commands;

public:
	ECUSerial();
	~ECUSerial();

	void executeCommand();

	char addCommand(const __FlashStringHelper* new_command_str, const char(*function_ptr)(void*), void* object);

	char timedParseInt(int &value, unsigned char timeout = 55);

	char timedReceiveArray(int new_array[], const unsigned int n_array, const char str[] = "");

	void dumpLine(void)
	{
		char str[5];
		while (Serial.readBytesUntil('\n', str, 4));
	}

	template <typename T>
	void reportArray(const char str[], T data[], unsigned int n)
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
	char receiveNumberBetween(T *var, const int lower, const int upper, const char var_name[])
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

