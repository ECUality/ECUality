
#ifndef ECUALITY_H
#define ECUALITY_H

#include "Arduino.h"


template <typename T> 
void reportArray(char str[], T *data, unsigned int n)
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

#endif	// ECUALITY_H