
#include "ECUSerial.h"
#include "Arduino.h"


ECUSerial::ECUSerial() :
HardwareSerial(&rx_buffer, &tx_buffer, &UBRRH, &UBRRL, &UCSRA, &UCSRB, &UCSRC, &UDR, RXEN, TXEN, RXCIE, UDRIE, U2X);
{
}


ECUSerial::~ECUSerial()
{
}

char ECUSerial::timedParseInt(int &value, unsigned char timeout)
{
	unsigned int time = millis();
	value = Serial.parseInt();
	time = millis() - time;
	if (time > timeout)
		return 0;
	return 1;
}

char ECUSerial::timedReceiveArray(int new_array[], const unsigned int n_array, const char str[] = "")
{	// parses sequential integers from serial into new_array.  Returns 0 if any integer takes longer than 50 ms.
	unsigned int i;

	Serial.setTimeout(SERIAL_TIMEOUT);	// set the timeout to receive each number to 50ms

	for (i = 0; i < n_array; ++i)
	{
		if (!timedParseInt(new_array[i]))
		{
			Serial.print("timed out while reading ");
			Serial.println(str);
			return 0;
		}

	}
	return 1;
}