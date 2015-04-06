#pragma once 


template <typename T>
void addArrays(T source_array[], T destination_array[], unsigned int n)
{
	unsigned int i;

	for (i = 0; i < n; i++)
		destination_array[i] += source_array[i];
}

template <typename T>
void copyArray(const T source_array[], T destination_array[], const unsigned int n)
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


