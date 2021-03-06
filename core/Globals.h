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
uint8_t simulate_tach_en, fuel_pump_en, injector_en, coil_en; 

// task variables
unsigned int ms_freq_of_task[20];
unsigned int task_runtime[20];
unsigned int ms_since_last_task[20] = { 0 };
void(*task_list[20]) (void);
uint8_t n_tasks;
uint8_t i_autoreport; 

// sensor input variables
int air_flow, o2, air_temp, coolant_temp, oil_pressure;
uint16_t inj_duration, map_inj_duration, rpm;

// Ignition variables
uint16_t g_dwell, advance;	
volatile uint16_t v_spark_mark, v_dwell_mark; 
uint8_t coil_current_flag;
volatile uint8_t timer4_coil, timer5_coil;
MovingAverage avg_rpm(4);

volatile uint16_t tach_period;

// variables that capture dynamic aspects of sensor input
int air_flow_d, air_flow_snap, o2_d, rpm_d;


// fault counters
unsigned int n_rpm_hi_faults, n_rpm_lo_faults, n_air_faults;

// selected parameter to change

// EE-Backed Parameters //////////////////////////////////

Parameter idle_offset("iof", -1000, 1000);			// Just like global offset, but only used during idle conditions

Parameter global_correction("glo", -50, 50);		// a 1/256 correction that scales with pre-corrected injector duration (acts as a "percentage") 
// limited to be +-50/256 ~ approximately +-20%.  

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
	global_correction.value, offset_map, change_map);				// presently contains 7 parameters

Parameter nom_coil_current("ncc", 0, 31);	// nominal coil current.  0 = 3.0A 16 = 7.0A 31 = 10.75A
Parameter hi_speed_dwell("hsd", 50, 300);	// the ignition dwell in 4us units. 1ms - 10ms valid range. 
Parameter starting_dwell("std", 50, 400);	// dwell during starting

// ignition timing parameters
Parameter initial_retard("ire", 0, 512);	// initial retard (1/512ths of a crank rev. with dizzy, it would be ) 
Scale rpm_advance("rad", 200, 6000, 0, 70, 4);
Scale vacuum_advance("vad", 100, 2000, 0, 50, 4);
//Parameter min_cranking_rpm("mrp, ")

void NameParams() {
	// We need this step to be separate from the constructor only because the F() macro has to live inside a function.  
	global_correction.setName(F("global_corr"));
	coasting_rpm.setName(F("coasting_rpm"));
	idling_rpm.setName(F("idling_rpm"));
	air_thresh.setName(F("air_thresh"));
	cold_threshold.setName(F("cold_threshold"));
	cranking_dur.setName(F("cranking_dur"));
	idle_offset.setName(F("idle_offset"));
	nom_coil_current.setName(F("nom_curr"));
	hi_speed_dwell.setName(F("hs_dwell"));
	starting_dwell.setName(F("start_dwell"));


	// Scales
	choke_scale.setName(F("choke_scale"));
	temp_scale.setName(F("temp_scale"));
	air_rpm_scale.setName(F("air_rpm_grid"));
	idle_scale.setName(F("idle_scale"));
	initial_retard.setName(F("init_retard"));
	rpm_advance.setName(F("rpm_adv"));
	vacuum_advance.setName(F("vac_adv"));

}