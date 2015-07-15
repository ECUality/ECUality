
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

ECUSerial::ECUSerial() :
HSerial(&Serial3),
DefaultSerial(HSerial)
{
	n_commands = 0;
}

ECUSerial::~ECUSerial() { }



void ECUSerial::executeCommand()
{
	char i;
	char c[N_CMD_CHARS] = "";
	
	//if (!HSerial->available())
	//	return;

	// check both serial ports for a received command. Switch active port to whichever has data. 
	if (Serial.available())
		HSerial = &Serial;
	else if (Serial3.available())
		HSerial = &Serial3;
	else
		return;

	// passes up any non-letters in the HSerial buffer.  Waits 20ms for each. 
	if (dumpNonLetters() == -1)		// if only non-letters are showing up, fuggadaboutit. 
	{
		HSerial = DefaultSerial;
		return;
	}
	

	// reads at most 4 letters into string c.  The first non-letter encountered stops the reading. 
	// that first non-letter remains in the buffer.  Only the letters are eaten.  
	readNLetters(c, N_CMD_CHARS - 1);

	
	for (i = 0; i < n_commands; i++)
	{
		if (!strncmp(c, command_str[i], strlen(command_str[i])))		// if the strings match, strncmp returns 0
		{
			// call attached function on the attached object. 
			(*fun_ptr[i])(obj_ptr[i]);
			//dumpNonLetters();
			HSerial = DefaultSerial;
			return;
		}
	}
	// if we didn't interpret a valid command, ignore the rest until we see '\n'
	HSerial->print(F("no comprendo: "));		// if we don't return by now, there was no match. 
	HSerial->println(c);
	dumpLine();

	HSerial = DefaultSerial;
	return;
}

char ECUSerial::addCommand(const __FlashStringHelper* new_command_str_F, const char(*function_ptr)(void*), void* object)
{
	Ftochar(new_command_str_F, command_str[n_commands], N_CMD_CHARS);
	fun_ptr[n_commands] = function_ptr;
	obj_ptr[n_commands] = object;
	n_commands++;
}

int ECUSerial::timedPeek(unsigned int timeout )
{
	unsigned long time = millis();
	while (HSerial->peek() == -1)
	{
		if ((millis() - time) > timeout)
			break;
	}
	return HSerial->peek();
}

char ECUSerial::timedParseInt(int &value, unsigned char timeout)
{
	unsigned int time = millis();
	value = HSerial->parseInt();
	time = millis() - time;
	if (time > timeout)
		return 0;
	return 1;
}

char ECUSerial::timedReceiveArray(int new_array[], const unsigned int n_array, const char str[])
{	// parses sequential integers from serial into new_array.  Returns 0 if any integer takes longer than 50 ms.
	unsigned int i;

	HSerial->setTimeout(SERIAL_TIMEOUT);	// set the timeout to receive each number to 50ms

	for (i = 0; i < n_array; ++i)
	{
		if (!timedParseInt(new_array[i]))
		{
			HSerial->print(F("timed out while reading "));
			HSerial->println(str);
			return 0;
		}

	}
	return 1;
}

char ECUSerial::dumpNonLetters()
{
	char c = HSerial->peek();	// don't need to time this because we know there is at least one from available() check. 
	while (!(  ((c >= 'a') && (c <= 'z'))  ||  ((c >= 'A') && (c <= 'Z'))  ||  ((c >= '*') && (c <= '/'))  ))  // c not a letter or +-*/.,./;'
	{
		HSerial->read();			// advance the buffer
		c = timedPeek();		// waits (default of) 20ms for a character to show up. returns -1 on timeout.
		if (c == -1)			// if there are no more characters, step out of the loop. 
			break;
	}
	return c;		// will be a -1 if there are no letters.  If there is a letter, it will be the first letter encountered. 
}

char ECUSerial::readNLetters(char c[], unsigned int n)
{
	unsigned int i;
	for (i = 0; i < n; i++)
	{
		c[i] = timedPeek();
		HSerial->read();		// advance the buffer. 
		if (!(((c[i] >= 'a') && (c[i] <= 'z')) || ((c[i] >= 'A') && (c[i] <= 'Z'))))	// if c[i] not a letter nor /*-+.
			break;
	}
}

void ECUSerial::dumpLine()
{
	char str[5];
	while (HSerial->readBytesUntil('\n', str, 4));
}

ECUSerial ESerial;