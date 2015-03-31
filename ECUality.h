
#ifndef ECUALITY_H
#define ECUALITY_H

#include "Arduino.h"


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
void addArrays(T source_array[], T destination_array[], unsigned int n)
{
	unsigned int i;

	for (i = 0; i < n; i++)
		destination_array[i] += source_array[i];
}

template <typename T>
void copyArray(T source_array[], T destination_array[], unsigned int n)
{
	unsigned int i;

	for (i = 0; i < n; i++)
	{
		destination_array[i] = source_array[i];
	}
}

template <typename T>
void clearArray(T destination_array[], unsigned int n)
{
	unsigned int i;

	for (i = 0; i < n; i++)
	{
		destination_array[i] = 0;
	}
}

template <typename T>
char receiveArray(T new_array[], unsigned int n_array, char str[])
{
	unsigned int i;

	Serial.setTimeout(50);	// set the timeout to receive each number to 50ms

	for (i = 0; i < n_array; ++i)
	{
		new_array[i] = Serial.parseInt();
		//Serial.print(new_array[i]);

		if (!new_array[i])
		{
			Serial.print("timed out while reading ");
			Serial.println(str);
			return 0;
		}

	}
	return 1;
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

#endif	// ECUALITY_H