#include "TimerOne.h"
#include <eeprom.h>
#include "ECUality.h"
#include "EEPROMAnything.h"
 
#define MAX_MAP_SIZE	10
#define MIN_MAP_SIZE	3

const char air_flow_pin		=	A12;
const char air_temp_pin		=	A14;
const char o2_pin			=	A9;
const char coolant_temp_pin =	A13;
const char oil_pressure_pin =	A11;

unsigned int ms_between_doing_task[10];
unsigned int timer_for_task[10] = { 0 };
void (*task[10]) (void);
unsigned char n_tasks;

unsigned char air_flow, air_temp, o2, coolant_temp, oil_pressure;

// map variables
unsigned int n_air_gridlines, n_rpm_gridlines;
unsigned int air_gridline[MAX_MAP_SIZE], rpm_gridline[MAX_MAP_SIZE];
unsigned int engine_map[MAX_MAP_SIZE * MAX_MAP_SIZE];
 
void setup() 
{
	task[0] = readAirFlow;		ms_between_doing_task[0] = 50;
	task[1] = readO2Sensor;		ms_between_doing_task[1] = 300;
	task[2] = readAirTemp;		ms_between_doing_task[2] = 250;
	task[3] = readCoolantTemp;	ms_between_doing_task[3] = 250;
	task[4] = readOilPressure;	ms_between_doing_task[4] = 250;
	n_tasks = 5;
	
	//loadMapFromEE();

	// Initialize the digital pin as an output.
	// Pin 13 has an LED connected on most Arduino boards
	pinMode(13, OUTPUT);    
	pinMode(12, OUTPUT);
	pinMode(11, OUTPUT);
  
	Timer1.initialize(1000);				// set half-period = 1000 microseconds (1 ms)
	Timer1.attachInterrupt( Timer1_isr ); // attach the service routine here
  
	Serial.begin(115200);
	//attachInterrupt(4, Toggle_led, RISING);  
}
 
void loop()
{

	Poll_Serial();

	
}

void Poll_Serial()
{
	static char c[10];
	static unsigned int receivedNum; 

	if (!Serial.available())
		return;
	
	Serial.readBytes(c, 1);
	//Serial.write(c, 1);		// echo

	//receivedNum = Serial.parseInt();
	//Serial.readBytesUntil('\n', &c[2], 8);

	switch (c[0])
	{
	case 'a':		// 'a' for setting air %
		receivedNum = constrain(receivedNum, 0, 255);
		//Set_Air((char)receivedNum);
		Serial.print("Air set to: ");
		Serial.println(receivedNum);
		break;

	case 'm':		// 'o' for setting O2
		receiveMap();
		break;

	case 'r':		// 'r' for setting RPM
		reportMap();
		break;

	case 'l':		// 'i' to sample the Injector duration
		loadMapFromEE();
		Serial.println("loaded from EE");
		break;

	case 's':		// 'r' for setting RPM
		saveMapToEE();
		Serial.println("saved map to EE");
		break;

	default:
		Serial.println("not understood");
	}
	dumpLine();
}

// Serial to data functions
int receiveMap()
{
	char str[3] = "";	// all zeros.
	unsigned int new_engine_map[MAX_MAP_SIZE * MAX_MAP_SIZE];
	unsigned int new_air_gridline[MAX_MAP_SIZE];
	unsigned int new_rpm_gridline[MAX_MAP_SIZE];
	unsigned int new_n_air_gridlines, new_n_rpm_gridlines;
	unsigned int new_map_size;

	Serial.readBytes(str, 2);

	if (strcmp(str, "ap") != 0)
	{
		Serial.println("spell 'map' please");
		return -1;
	}

	if (!receiveUIntBetween(&new_n_air_gridlines, MIN_MAP_SIZE, MAX_MAP_SIZE, "air_gridlines"))
		return- 1;

	if (!receiveUIntBetween(&new_n_rpm_gridlines, MIN_MAP_SIZE, MAX_MAP_SIZE, "rpm_gridlines"))
		return -1;

	if (!receiveUIntArray(new_air_gridline, new_n_air_gridlines, "air gridline"))
		return -1;
	
	if (!receiveUIntArray(new_rpm_gridline, new_n_rpm_gridlines, "rpm gridline"))
		return -1;

	new_map_size = new_n_air_gridlines * new_n_rpm_gridlines;
	if (!receiveUIntArray(new_engine_map, new_map_size, "map data"))
		return -1;


	n_air_gridlines = new_n_air_gridlines;
	n_rpm_gridlines = new_n_rpm_gridlines;
	copyArray(new_air_gridline, air_gridline, n_air_gridlines);
	copyArray(new_rpm_gridline, rpm_gridline, n_rpm_gridlines);
	copyArray(new_engine_map, engine_map, new_map_size);

	dumpLine();		// dump any additional characters. 

	reportMap();
	
}

void dumpLine(void)
{
	char str[5];
	while (Serial.readBytesUntil('\n', str, 4));
}
void reportMap()
{
	int i;
	reportArray("air_gridline:\n", air_gridline, n_air_gridlines);
	reportArray("rpm_gridline:\n", rpm_gridline, n_rpm_gridlines);
	Serial.println("engine_map:");
	for (i = 0; i < n_rpm_gridlines; ++i)
	{
		reportArray(" ", &engine_map[n_air_gridlines*i], n_air_gridlines);
	}
	Serial.print("\n");
}
void reportArray(char str[], unsigned int *data, unsigned int n)
{
	int i;
	Serial.print(str);
	for (i = 0; i < n; ++i)
	{
		Serial.print(data[i]);
		Serial.print("\t");
	}
	Serial.print("\n");
}
char receiveUIntArray(unsigned int *new_array, unsigned int n_array, char str[])
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
char receiveUIntBetween(unsigned int *var, unsigned int lower, unsigned int upper, char var_name[])
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
void copyArray(unsigned int source_array[], unsigned int destination_array[], unsigned int n)
{
	unsigned int i;

	for (i = 0; i < n; i++)
	{
		destination_array[i] = source_array[i];
	}
}

// EE access functions
void loadMapFromEE()
{
	// eeprom addresses: 0 = n_air_gridlines, n_rpm_gridlines
	unsigned int address, map_size, value;
	address = 0;

	address += EEPROM_readAnything(address, value);
	if (!assignIfBetween(value, n_air_gridlines, MAX_MAP_SIZE, MIN_MAP_SIZE, "n_air_gridlines"))
		return;

	address += EEPROM_readAnything(address, value);
	if (!assignIfBetween(value, n_rpm_gridlines, MAX_MAP_SIZE, MIN_MAP_SIZE, "n_rpm_gridlines"))
		return;

	address += EEPROM_readAnything(address, air_gridline);
	address += EEPROM_readAnything(address, rpm_gridline);
	address += EEPROM_readAnything(address, engine_map);
	
}
char assignIfBetween(const unsigned int source, unsigned int &destination, unsigned int max, unsigned int min, char var_name[])
{
	if ((source > MAX_MAP_SIZE) || (source < MIN_MAP_SIZE))
	{
		Serial.print(var_name);
		Serial.println(" out of range");
		return 0;
	}
	destination = source;
	return 1;
}
void saveMapToEE()
{
	// eeprom addresses: 0 = n_air_gridlines, n_rpm_gridlines
	unsigned int address, map_size;
	address = 0;

	address += EEPROM_writeAnything(address, n_air_gridlines);
	address += EEPROM_writeAnything(address, n_rpm_gridlines);
	address += EEPROM_writeAnything(address, air_gridline);
	address += EEPROM_writeAnything(address, rpm_gridline);
	address += EEPROM_writeAnything(address, engine_map);
}
//template <class T> int EEPROM_writeAnything(int ee, const T& value)
//{
//	const byte* p = (const byte*)(const void*)&value;
//	unsigned int i;
//	for (i = 0; i < sizeof(value); i++)
//		EEPROM.write(ee++, *p++);
//	return i;
//}
//template <class T> int EEPROM_readAnything(int ee, T& value)
//{
//	byte* p = (byte*)(void*)&value;
//	unsigned int i;
//	for (i = 0; i < sizeof(value); i++)
//		*p++ = EEPROM.read(ee++);
//	return i;
//}

//unsigned int readUIntFromEE(unsigned int address)
//{
//	unsigned int value = 0;
//	value = EEPROM.read(address);
//	value <<= 8;
//	value |= EEPROM.read(address + 1);
//	return value;
//}

//void readUIntArrayFromEE(unsigned int *destination_array, unsigned int start_address, unsigned int length)
//{
//	unsigned int i;
//	for (i = 0; i < length; i++)
//	{
//		destination_array[i] = readUIntFromEE(start_address);
//		start_address += 2; 
//	}
//}

//void writeUIntArrayToEE(unsigned int *source_array, unsigned int start_address, unsigned int length )
//{
//	unsigned int i;
//	for (i = 0; i < length; i++)
//	{
//		source_array[i] = writeUIntToEE(start_address);
//		start_address += 2;
//	}
//}






void Delay_Ms( unsigned int d ) {
	unsigned int  i, k;
	for ( i=0; i<d; i++ )
	{
		for ( k=0; k<2000; k++) 
		{
			asm volatile ("NOP");
		}
			
	}
}

void readAirFlow()
{
	toggle(12);
	//air_flow = analogRead(air_flow_pin);
}

void readO2Sensor()
{
	toggle(13);
	//o2 = digitalRead(o2_pin);
}

void readAirTemp() 
{
	air_temp = analogRead(air_temp_pin);
}

void readCoolantTemp() 
{
	coolant_temp = analogRead(coolant_temp_pin);
}

void readOilPressure()
{
	oil_pressure = analogRead(oil_pressure_pin);
}

//void loadMap(unsigned int engine_map[][9])
//{
//	
//	unsigned int map1[9][9] = {	
//		{ 0,	220,	180,	140,	100,	80, 	60, 	40, 	20 },
//		{ 500,	2473,	2160,	1899,	1789,	1537,	1301,	1185,	1129 },
//		{ 695,	2473,	2161,	1900,	1744,	1484,	1166,	988,	925 },
//		{ 965,	2471,	2171,	1922,	1619,	1176,	945,	775,	776 },
//		{ 1341,	2472,	2170,	1898,	1229,	940,	774,	757,	827 },
//		{ 1863,	2471,	2338,	1632,	949,	749,	635,	676,	710 },
//		{ 2589,	2464,	2006,	1223,	749,	618,	635,	645,	624 },
//		{ 3589,	2476,	1596,	944,	615,	587,	587,	588,	901 },
//		{ 5000,	1931,	1203,	766,	596,	564,	561,	560,	589  }	
//		};
//			
//	engine_map = map1;			
//	
//}

void Timer1_isr()
{
	sei();
	static unsigned char i;

	for (i = 0; i < n_tasks; i++)
	{
		timer_for_task[i]++;
		if (timer_for_task[i] >= ms_between_doing_task[i])
		{
			(*task[i])();			// call task i.
			timer_for_task[i] = 0;		// reset this task's timer
		}
	}

}

void toggle(unsigned char pin) {
	// Toggle LED
    digitalWrite( pin, digitalRead( pin ) ^ 1 );
}