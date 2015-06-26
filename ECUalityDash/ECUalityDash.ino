/*
LiquidCrystal Library - Serial Input

* LCD RS pin to digital pin 12
* LCD Enable pin to digital pin 11
* LCD D4 pin to digital pin 5
* LCD D5 pin to digital pin 4
* LCD D6 pin to digital pin 3
* LCD D7 pin to digital pin 2
* LCD R/W pin to ground
* 

Library originally added 18 Apr 2008
by David A. Mellis
library modified 5 Jul 2009
by Limor Fried (http://www.ladyada.net)
example added 9 Jul 2009
by Tom Igoe
modified 22 Nov 2010
by Tom Igoe

http://arduino.cc/en/Tutorial/LiquidCrystalSerial
*/

// include the library code:
#include <LiquidCrystal.h>
#include <IRLib.h>


// initialize Infrared receiver (measures pulses) and decoder (interprets pulses)
IRdecode My_Decoder;
IRrecv My_Receiver(42);	 // pin assigned

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(12, 11, 10, 9, 8, 7, 6);
HardwareSerial& ESerial = Serial1;
//LiquidCrystal& display = lcd; 

const unsigned long code[] = { 0xFFA25D, 0xFFE21D, 0xFF22DD, 0xFF02FD, 0xFFC23D, 0xFFE01F, 0xFFA857, 0xFF906F, 0xFF6897,
0xFF9867, 0xFFB04F, 0xFF30CF, 0xFF18E7, 0xFF7A85, 0xFF10EF, 0xFF38C7, 0xFF5AA5, 0xFF42BD, 0xFF4AB5, 0xFF52AD };

const char command[][6] = { "arm", "x", "lock", "Sglo", "Sloc", "adji", " ", " ", " ", "+", "-", 
	"auto1", "auto2", "auto3", "auto4", "auto5", "auto6", "auto7", "auto8", "auto9" };

char text_line[16] = "";

#define IRpin_PIN PIND
#define IRpin 2

// for MEGA use these!
//#define IRpin_PIN PINE
//#define IRpin 4

void setup(){
	// set up the LCD's number of columns and rows: 
	lcd.begin(16, 2);
	// initialize the serial communications:
	ESerial.begin(9600);

	My_Receiver.enableIRIn();
}

void loop()
{
	char c; 
	uint8_t n_codes = (sizeof(code) / sizeof(code[0]));
	int i;

	// when characters arrive over the serial port...
	if (ESerial.available()) {
		// wait a bit for the entire message to arrive
		delay(100);

		// check if it's a fault (bottom row)
		if (ESerial.peek() == '!')
		{
			lcd.setCursor(0, 1);	// change to bottom row
			ESerial.read();				// eat the '!' character
		}

		else
			lcd.setCursor(0, 0);

		// read all the available characters
		for (int i = 0; i < 16; i++)
		{
			c = ESerial.peek();
			if (c < ' ')
				lcd.write(' ');
			else
				lcd.write(ESerial.read());

		}

		// dump the rest of the regular-character stream until we get special characters
		while (1)
		{
			c = ESerial.read();
			if (c < ' ')
				break;
		}
		// now dump the remaining characters (which will hopefully all be special)
		while (ESerial.available())
		{
			ESerial.read();
		}
			
	}

	if (My_Receiver.GetResults(&My_Decoder))
	{
		My_Decoder.decode();
		//My_Decoder.DumpResults();
		My_Receiver.resume();
		
		for (i = 0; i < n_codes; i++)
		{
			if (code[i] == My_Decoder.value)
			{
				ESerial.println(command[i]);
				break;
			} 
		}

	}
}