
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

	// passes up any non-letters in the Serial buffer.  Waits 20ms for each. 
	if (dumpNonLetters() == -1)		// if only non-letters are showing up, fuggadaboutit. 
		return;

	// reads at most 4 letters into string c.  The first non-letter encountered stops the reading. 
	// that first non-letter remains in the buffer.  Only the letters are eaten.  
	readNLetters(c, N_CMD_CHARS - 1);

	
	for (i = 0; i < n_commands; i++)
	{
		if (!strncmp(c, command_str[i], strlen(command_str[i])))		// if the strings match, strncmp returns 0
		{
			// call attached function on the attached object. 
			(*fun_ptr[i])(obj_ptr[i]);
			return;
		}
	}
	// if we didn't interpret a valid command, ignore the rest until we see '\n'
	Serial.print(F("no comprendo: "));		// if we don't return by now, there was no match. 
	Serial.println(c);
	dumpLine();
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

int ECUSerial::timedPeek(unsigned int timeout )
{
	unsigned long time = millis();
	while (Serial.peek() == -1)
	{
		if ((millis() - time) > timeout)
			break;
	}
	return Serial.peek();
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
			Serial.print(F("timed out while reading "));
			Serial.println(str);
			return 0;
		}

	}
	return 1;
}

char ECUSerial::dumpNonLetters()
{
	char c = Serial.peek();
	while (!(  ((c >= 'a') && (c <= 'z'))  ||  ((c >= 'A') && (c <= 'Z'))  ||  ((c >= '*') && (c <= '/'))  ))  // c not a letter or +-*/.,./;'
	{
		Serial.read();			// advance the buffer
		c = timedPeek();		// waits (default of) 20ms for a character to show up. returns -1 on timeout.
	}
	return c;
}

char ECUSerial::readNLetters(char c[], unsigned int n)
{
	unsigned int i;
	for (i = 0; i < n; i++)
	{
		c[i] = timedPeek();
		if (!(((c[i] >= 'a') && (c[i] <= 'z')) || ((c[i] >= 'A') && (c[i] <= 'Z')) || ((c[i] >= '*') && (c[i] <= '/'))))	// if c[i] not a letter nor /*-+.
			break;
		Serial.read();		// advance the buffer. 
	}
}

void ECUSerial::dumpLine()
{
	char str[5];
	while (Serial.readBytesUntil('\n', str, 4));
}

ECUSerial ESerial;