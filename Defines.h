
#pragma once 

#define MAX_MAP_SIZE 10
#define MIN_MAP_SIZE 3
#define NIL -3000

#define RUNNING_RPM	300

#define NOT_RUNNING	0
#define	CRANKING	1
#define COASTING	2
#define IDLING		3
#define WIDE_OPEN	4
#define WARM		5

#define TWEAK_FREQ_MS 200
#define TWEAKS_PER_SEC 5
#define ENABLE_ADDY 3001

// Data: 60e6 us/min   4us/timer tic   2 tach pulses per rev.
// 60e6 / 4 / 2 = 75e5 timer tics/pulse * rpm 
// so rpm = 75e5/(spark_period)
#define TICS_PER_TACH 7500000
#define MIN_IGN_RPM  200