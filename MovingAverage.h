#pragma once

#include "Arduino.h"

#define	LOG2_BUFFER_SIZE_MAX		4	// means maximum size = 16 (2^4). 
class MovingAverage
{
	int buffer[1 << LOG2_BUFFER_SIZE_MAX];
	volatile unsigned int head;
	uint8_t log2_size;
	uint8_t size;

public:
	MovingAverage(uint8_t log2_size_ = LOG2_BUFFER_SIZE_MAX);
	~MovingAverage();

	void addSample(int new_sample);

	int average;
};

