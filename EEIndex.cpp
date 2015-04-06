#include "EEIndex.h"
#include "HardwareSerial.h"


EEIndex::EEIndex()
{
	n_addresses = 1;		// we skip 0 so we can use that as an indicator of a failed address registration
}


EEIndex::~EEIndex()
{
}

const unsigned int EEIndex::getNewAddress(const unsigned int size)
{
	// this is a good place to check that n_addresses is not above MAX_ADDRESSES 
	if (n_addresses >= MAX_EE_ADDRESSES)
	{
		Serial.println("too many EE addresses");
		return 0;
	}
	if ((addresses[n_addresses] + size) >= EE_AVAILABLE)
	{
		Serial.println("Overflowed EE space");
		return 0;
	}
	n_addresses++;
	addresses[n_addresses] = addresses[n_addresses - 1] + size;
	return addresses[n_addresses-1];
}

EEIndex EE_index;