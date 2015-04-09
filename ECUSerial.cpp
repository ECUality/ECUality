
#include "ECUSerial.h"
#include "Arduino.h"
#include "Arrays.h"

void Ftochar(const __FlashStringHelper *ifsh, char* c, uint8_t n)
{
	PGM_P p = reinterpret_cast<PGM_P>(ifsh);
	uint8_t i;

	for (i = 0; i < n; i++)
	{
		c[i] = pgm_read_byte(p++);
		if (c == 0) break;
	}
}

ECUSerial::ECUSerial() 
{
	n_commands = 0;
}

ECUSerial::~ECUSerial() { }

void ECUSerial::executeCommand()
{
	char i;
	char c[N_CMD_CHARS] = "";
	
	if (!Serial.available())
		return;

	Serial.setTimeout(20);		// give it just a little time to give us a string (avoids delay for non-numerical commands)
	Serial.readBytesUntil(' ', c, N_CMD_CHARS - 1);
	Serial.setTimeout(SERIAL_TIMEOUT);

	for (i = 0; i < n_commands; i++)
	{
		if (!strncmp(c, command_str[i], strlen(command_str[i])))		// if the strings match, strncmp returns 0
		{
			// call attached function on the attached object. 
			(*fun_ptr[i])(obj_ptr[i]);
			ESerial.dumpLine();
			return;
		}
	}
	Serial.println("no comprendo");		// if we don't return by now, there was no match. 
	ESerial.dumpLine();
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

char ECUSerial::addCommand(const __FlashStringHelper* new_command_str_F, const char(*function_ptr)(void*), void* object)
{
	char new_command_str[N_CMD_CHARS];
	Ftochar(new_command_str_F, new_command_str, N_CMD_CHARS);
	fun_ptr[n_commands] = function_ptr;
	obj_ptr[n_commands] = object;
	copyArray(new_command_str, command_str[n_commands], N_CMD_CHARS);
	n_commands++;
}


ECUSerial ESerial;