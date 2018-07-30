#include "MovingAverage.h"


MovingAverage::MovingAverage(uint8_t log2_size_) :
head(0),
average(0)
{
	if (log2_size_ < LOG2_BUFFER_SIZE_MAX)
		log2_size = log2_size_;
	else
		log2_size = LOG2_BUFFER_SIZE_MAX;
	
	size = 0x01 << log2_size;
}


MovingAverage::~MovingAverage()
{
}

void MovingAverage::addSample(int new_sample)
{
	long sum = 0; 
	// store new sample in buffer
	buffer[head] = new_sample; 

	// re-compute average
	for (uint8_t i = 0; i < size; i++)
	{
		sum += buffer[i];
	}
	average = sum >> log2_size;		// divide by the number of elements in the buffer. 

	// advance the head
	if (++head >= size)
		head = 0;
}