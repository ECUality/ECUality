#pragma once

#define MAX_EE_ADDRESSES 20
#define EE_AVAILABLE     2000

class EEIndex
{
public:

	EEIndex();
	~EEIndex();
	const unsigned int getNewAddress(const unsigned int size);

	unsigned int addresses[MAX_EE_ADDRESSES];
	int n_addresses;
};


extern EEIndex EE_index; 