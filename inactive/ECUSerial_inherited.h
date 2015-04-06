#pragma once
#include "HardwareSerial.h"

#define SERIAL_TIMEOUT 50

class ECUSerial : public HardwareSerial
{
public:
	ECUSerial();
	~ECUSerial();

	void checkForCommand();

	char addCommandToList();

	char timedParseInt(int &value, unsigned char timeout = 55);

	char timedReceiveArray(int new_array[], const unsigned int n_array, const char str[] = "");

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

#if defined(UBRRH) || defined(UBRR0H)
extern ECUSerial ESerial;
#endif
