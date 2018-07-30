#include "EEIndex.h"
#include "ECUSerial.h"


EEIndex::EEIndex()
	: n_addresses(0),
	addresses()
{ 
	addresses[0] = 1;	// we skip 0 so we can use that as an indicator of a failed address registration
}



EEIndex::~EEIndex()
{
}

const unsigned int EEIndex::getNewAddress(const unsigned int size)
{
	// EE address 0, although valid, is considered invalid for the purposes of this class. 
	// any calling function should view a return value of 0 from this function 
	// as an indicator that the process of getting an EE address has failed, and should
	// not try to read from that address.

	if (n_addresses >= MAX_EE_ADDRESSES)			// check that our array is not full 
	{
		ESerial.println(F("too many EE addresses"));
		return 0;
	}
	if ((addresses[n_addresses] + size) >= EE_AVAILABLE)	// check that we still have EE space
	{
		ESerial.println(F("Overflowed EE space"));
		return 0;
	}
	n_addresses++;
	addresses[n_addresses] = addresses[n_addresses - 1] + size;
	return addresses[n_addresses-1];
}

EEIndex EE_index;