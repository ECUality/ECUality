#include "TimerOne.h"
 
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
	
	//receiveMap();

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

		break;

	case 'c':
		
		break;

	case 'i':		// 'i' to sample the Injector duration
		
		break;

	case 't':		// 'i' to sample the Injector duration
		
		break;

	default:
		Serial.println("not understood");
	}
}

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

	while (Serial.readBytesUntil('\n', str, 2));			// dump any additional characters. 

	reportMap();
	
}
void reportMap()
{
	int i;
	report("air_gridline:\n", air_gridline, n_air_gridlines);
	report("rpm_gridline:\n", rpm_gridline, n_rpm_gridlines);
	Serial.println("engine_map:");
	for (i = 0; i < n_rpm_gridlines; ++i)
	{
		report(" ", &engine_map[n_air_gridlines*i], n_air_gridlines);
	}
	Serial.print("\n");
}
void report(char str[], unsigned int *data, unsigned int n)
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
void copyArray(unsigned int source_array[], unsigned int destination_array[], unsigned int n)
{
	unsigned int i;

	for (i = 0; i < n; i++)
	{
		destination_array[i] = source_array[i];
	}
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

void loadMap(unsigned int engine_map[][9])
{
	
	unsigned int map1[9][9] = {	
		{ 0,	220,	180,	140,	100,	80, 	60, 	40, 	20 },
		{ 500,	2473,	2160,	1899,	1789,	1537,	1301,	1185,	1129 },
		{ 695,	2473,	2161,	1900,	1744,	1484,	1166,	988,	925 },
		{ 965,	2471,	2171,	1922,	1619,	1176,	945,	775,	776 },
		{ 1341,	2472,	2170,	1898,	1229,	940,	774,	757,	827 },
		{ 1863,	2471,	2338,	1632,	949,	749,	635,	676,	710 },
		{ 2589,	2464,	2006,	1223,	749,	618,	635,	645,	624 },
		{ 3589,	2476,	1596,	944,	615,	587,	587,	588,	901 },
		{ 5000,	1931,	1203,	766,	596,	564,	561,	560,	589  }	
		};
			
	engine_map = map1;			
	
	/*
	for (r=0; r < rows; r++)
	{
		for (c=0; c < cols; c++)
		{
			
			
		}
		
	} */
}

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