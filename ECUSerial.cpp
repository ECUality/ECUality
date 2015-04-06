
#include "ECUSerial.h"
#include "Arduino.h"
#include "Arrays.h"



ECUSerial::ECUSerial() 
{
	n_commands = 0;
}

ECUSerial::~ECUSerial() { }

void ECUSerial::executeCommand()
{
	char i;
	char c[N_CMD_CHARS];
	
	Serial.readBytesUntil('\n', c, N_CMD_CHARS - 1);

	for (i = 0; i < n_commands; i++)
	{
		if (strcmp(c, command_str))		// if the command matches
		{
			// call attached function on the attached object. 
			(*fun_ptr[i])(obj_ptr);
			return;
		}
	}
}

char ECUSerial::timedParseInt(int &value, unsigned char timeout)
{
	unsigned int time = millis();
	value = Serial.parseInt();
	time = millis() - time;
	if (time > timeout)
		return 0;
	return 1;
}

char ECUSerial::timedReceiveArray(int new_array[], const unsigned int n_array, const char str[])
{	// parses sequential integers from serial into new_array.  Returns 0 if any integer takes longer than 50 ms.
	unsigned int i;

	Serial.setTimeout(SERIAL_TIMEOUT);	// set the timeout to receive each number to 50ms

	for (i = 0; i < n_array; ++i)
	{
		if (!timedParseInt(new_array[i]))
		{
			Serial.print("timed out while reading ");
			Serial.println(str);
			return 0;
		}

	}
	return 1;
}

char ECUSerial::addCommand(const void(*function_ptr)(void*), void* object, const char name[N_CMD_CHARS])
{
	fun_ptr[n_commands] = function_ptr;
	obj_ptr[n_commands] = object;
	copyArray(name, command_str, N_CMD_CHARS);
	n_commands++;
}

ECUSerial ESerial;