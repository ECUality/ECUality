
#include "ECUSerial.h"
#include "Arduino.h"
#include "Arrays.h"
#include "Parameter.h"
#include "Scale.h"

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
HSerial(&Serial),		// Selects either Serial (USB) and Serial3 (Xbee) for primary comm. 
DefaultSerial(HSerial)
{
	n_commands = 0;
}

ECUSerial::~ECUSerial() { }



void ECUSerial::executeCommand()
{
	char i, j;
	char c[N_CMD_CHARS] = "";
	
	//if (!HSerial->available())
	//	return;

	// check both serial ports for a received command. Switch active port to whichever has data. 
	if (Serial3.available()) {
		if (HSerial == &Serial) {	// let us know on USB if we're swtching to Serial3 
			Serial.println(F("switching to Serial3"));
			HSerial = &Serial3;
		}
	}
	else if (Serial.available()) {
		HSerial = &Serial;
	}
	else
		return;

	// passes up any non-letters in the HSerial buffer.  Waits 20ms for each. 
	if (dumpNonLetters() == -1)		// if only non-letters are showing up, fuggadaboutit. 
		return;
	

	// reads at most 4 letters into string c.  The first non-letter encountered stops the reading. 
	// that first non-letter remains in the buffer.  Only the letters are eaten.  
	readNLetters(c, N_CMD_CHARS - 1);

	
	// Is first letter one of W, R, S?
	if ( (c[0] == 'W') || (c[0] == 'R') || (c[0] == 'S') )
	{
		// does suffix match known parameter handle? 
		for (j = 0; j < Parameter::n_params; j++)
		{
			if (!strncmp(&c[1], Parameter::params[j]->handle, 3)) 
			{
				// suffix matches, so do the Write, Read or Save. 
				if (c[0] == 'W')
				{
					Parameter::write(Parameter::params[j]);
					return;
				}
				else if (c[0] == 'R')
				{
					Parameter::read(Parameter::params[j]);
					return;
				}
				else if (c[0] == 'S')
				{
					Parameter::save(Parameter::params[j]);
					return;
				}
			}
		}

		// does suffix match known scale handle? 
		for (j = 0; j < Scale::n_scales; j++)
		{
			
			if (!strncmp(&c[1], Scale::scales[j]->handle, 3))
			{
				// suffix matches, so do the Write, Read or Save. 
				if (c[0] == 'W')
				{
					Scale::write(Scale::scales[j]);
					return;
				}
				else if (c[0] == 'R')
				{
					Scale::read(Scale::scales[j]);
					return;
				}
				else if (c[0] == 'S')
				{
					Scale::save(Scale::scales[j]);
					return;
				}
			}
		}
	}
	
	// We didn't match the incoming command to a Write, Read or Save of a parameter or scale object.
	// so now we check the misc. command list by just iterating through. 
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
	HSerial->print(F("no comprendo: "));		// if we don't return by now, there was no match. 
	HSerial->println(c);
	dumpLine();

	//HSerial = DefaultSerial;
	return;
}

char ECUSerial::addCommand(const __FlashStringHelper* new_command_str_F, const char(*function_ptr)(void*), void* object)
{
	Ftochar(new_command_str_F, command_str[n_commands], N_CMD_CHARS);
	fun_ptr[n_commands] = function_ptr;
	obj_ptr[n_commands] = object;
	n_commands++;
}

char ECUSerial::addCommand(const char* new_command_str, const char(*function_ptr)(void*), void* object)
{
	for (unsigned char i = 0; i < N_CMD_CHARS; i++)
	{
		command_str[n_commands][i] = new_command_str[i];
	}
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