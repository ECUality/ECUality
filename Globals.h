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
int air_flow, rpm, o2, air_temp, coolant_temp, oil_pressure;
unsigned int tach_period, inj_duration;
MovingAverage avg_rpm(4);

// variables that capture dynamic aspects of sensor input
int air_flow_d, air_flow_snap, o2_d;

// fault counters
unsigned int n_rpm_hi_faults, n_rpm_lo_faults, n_air_faults;

// EE-Backed Parameters //////////////////////////////////

Parameter global_offset("global_offset", -1000, 1000);
Parameter coasting_rpm("coasting_rpm", 800, 2500);
Parameter idling_rpm("idling_rpm", 400, 2200);
Parameter air_thresh("air_thresh", 50, 125);
Parameter accel_stabilize_rate("accel_stabilize_rate", 5, 500);		// the rate at which accelerator pump transient decays.
Parameter cold_threshold("cold_thresh", 80, 190);			// the temperature below which enrichment kicks in.  (100F to 150F)
Parameter cranking_dur("cranking_dur", 800, 3000);			// the injector duration while cranking not counting choke (500 - 2000) 
Parameter idle_dur("idle_dur", 500, 2000);					// the injector duration while idling. 
Scale choke_scale(-40, 200, 0, 2000, 0);	// scales engine temperature to "per 1024" (think percent) increase of injector itme
Scale temp_scale(0, 1023, -40, 250, 12);	// scales measured voltage on temp sensors to actual temp. in F
Scale air_rpm_scale(0, 255, 200, 6000, 8);	// the x and y gridlines (air-flow and rpm) for the injector map.  (not a scaling function) 
Map inj_map(&air_rpm_scale, 8, 400, 3000);	// the 2d map defining injector durations with respect to air-flow and rpm
Map offset_map(&air_rpm_scale, 8, -1500, 1500); // local modifications to the inj_map, applied by the optimizer. 
Map change_map(&air_rpm_scale, 8, -1500, 1500);	// map that contains the changes made since power-up. 
FuelTweaker boss(run_condition, air_flow, rpm, avg_rpm.average, o2, global_offset.value, idle_dur.value, offset_map, change_map);
Parameter idle_slope("idle_slope", 200, 4000);	// a slope that determines how fast the injector duration increases with decreasing rpm at idle.


