#pragma once

#include <Arduino.h>
#include "Parameter.h"
#include "Scale.h"
#include "Map.h"
#include "FuelTweaker.h"
#include "MovingAverage.h"


// operating control variables
char good_ee_loads;
uint8_t auto_stat;
unsigned char run_condition;
uint8_t status_mode;

// task variables
unsigned int ms_freq_of_task[20];
unsigned int task_runtime[20];
unsigned int ms_since_last_task[20] = { 0 };
void(*task[20]) (void);
uint8_t n_tasks;
uint8_t i_autoreport; 

// sensor input variables
int air_flow, o2, air_temp, coolant_temp, oil_pressure;
unsigned int inj_duration, rpm;

// Ignition variables
unsigned int old_tach_period, g_dwell;	
unsigned char coil_current_flag;
MovingAverage avg_rpm(4);

#define TACH_PERIOD_N 64
unsigned char tach_period_i;
unsigned int tach_period[TACH_PERIOD_N];

// variables that capture dynamic aspects of sensor input
int air_flow_d, air_flow_snap, o2_d, rpm_d;
//unsigned int last_tach_period;

// fault counters
unsigned int n_rpm_hi_faults, n_rpm_lo_faults, n_air_faults;

// selected parameter to change

// EE-Backed Parameters //////////////////////////////////

Parameter idle_offset("iof", -1000, 1000);			// Just like global offset, but only used during idle conditions
Parameter global_offset("glo", -1000, 1000);
Parameter coasting_rpm("crp", 800, 2500);
Parameter idling_rpm("irp", 400, 2200);
Parameter air_thresh("ath", 50, 125);
Parameter cold_threshold("cld", 80, 190);			// the temperature below which enrichment kicks in.  (100F to 150F)
Parameter cranking_dur("cra", 800, 3000);			// the injector duration while cranking not counting choke (500 - 2000) 
Scale choke_scale("cho", -40, 200, 0, 2000, 0);		// scales engine temperature to "per 1024" (think percent) increase of injector itme
Scale temp_scale("tem", 0, 1023, -40, 250, 12);	// scales measured voltage on temp sensors to actual temp. in F
Scale air_rpm_scale("gri", 0, 255, 200, 6000, 8);	// the x and y gridlines (air-flow and rpm) for the injector map.  (not a scaling function) 
Map inj_map("inj_map", "inj", &air_rpm_scale, 8, 400, 3000);	// the 2d map defining injector durations with respect to air-flow and rpm
Map offset_map("offset_map", "loc", &air_rpm_scale, 8, -1500, 1500); // local modifications to the inj_map, applied by the optimizer. 
Map change_map("change_map", "chg", &air_rpm_scale, 8, -1500, 1500);	// map that contains the changes made since power-up. 

Scale idle_scale("isc", 200, 2000, 300, 2000, 2);		// Linear "map" RPM v inj.Dur at idle.  Higher dur at lower RPM. 

FuelTweaker boss(run_condition, air_flow, rpm, avg_rpm.average, o2,
	global_offset.value, offset_map, change_map);				// presently contains 7 parameters

Parameter low_speed_dwell("lsd", 16, 800);		// the ignition dwell in 4us units. 1ms - 10ms valid range. 
Parameter hi_speed_dwell("hsd", 16, 156);
Parameter starting_dwell("std", 16, 800);			// dwell during starting
//Parameter min_cranking_rpm("mrp, ")

void NameParams() {
	// We need this step to be separate from the constructor only because the F() macro has to live inside a function.  
	global_offset.setName(F("global_offset"));
	coasting_rpm.setName(F("coasting_rpm"));
	idling_rpm.setName(F("idling_rpm"));
	air_thresh.setName(F("air_thresh"));
	cold_threshold.setName(F("cold_threshold"));
	cranking_dur.setName(F("cranking_dur"));
	idle_offset.setName(F("idle_offset"));
	low_speed_dwell.setName(F("ls_dwell"));
	hi_speed_dwell.setName(F("hs_dwell"));
	starting_dwell.setName(F("start_dwell"));


	// Scales
	choke_scale.setName(F("choke_scale"));
	temp_scale.setName(F("temp_scale"));
	air_rpm_scale.setName(F("air_rpm_grid"));
	idle_scale.setName(F("idle_scale"));
}