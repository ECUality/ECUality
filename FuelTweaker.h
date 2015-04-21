#pragma once

#include "Map.h"
#include "Parameter.h"


#define NO_TWEAK_MODE	0
#define IDLE_WAIT_MODE	1
#define GLOBAL_MODE		2
#define LOCAL_MODE		3
#define IDLE_ADJ_MODE	4



class FuelTweaker
{
	uint8_t lockout;
	uint8_t o2_open;
	unsigned int idle_adjustment;
	unsigned int time_warming_o2; 
	unsigned int time_eng_warm; 
	unsigned int time_running;
	unsigned int time_waiting;
	unsigned int rpm_old;
	int local_sum;
	unsigned int ee_addy;
	
	const unsigned char& run_condition;
	const int& o2;
	const int& air_flow;
	const int& rpm;
	const int& avg_rpm; 
	int& global_offset;

	Map& offset_map;
	Map& change_map; 

	void tweakvRPM();
	void tweakGlobalvO2();
	void tweakLocalAndGlobalvO2();
	void apportionLocalGlobal(int adjustment);
	void addToLocal(int adjustment);
	
public:
	uint8_t mode;

	// Adding a parameter happsne: 
	// 1) here  2) constructor initialization list  3) constructor body for initial value  4) load func. 5) save func. 
	// 6) report func.	7) ECUality.ino - "initProtocol" function adding write.  8) where it's used	

	Parameter	o2_upper_thresh;
	Parameter	o2_lower_thresh;	
	Parameter	step_size;
	Parameter	local_sum_limit;	
	Parameter	rpm_hyst;
	Parameter	idle_backstep;
	
	// times in seconds
	Parameter	time_warming_o2_thresh;
	Parameter	time_eng_warm_thresh;
	Parameter	time_running_thresh;
	Parameter	idle_adjust_freq; 



	FuelTweaker(const unsigned char& run_condition, const int& air_flow, const int& rpm, const int& avg_rpm, 
		const int& o2, int& global_correction, Map& offset_map, Map& change_map);
	~FuelTweaker();
	void tweak();
	static const char status(void* obj_ptr);
	static const char reportParams(void* obj_ptr);
	static const char load(void* obj_ptr);
	static const char save(void* obj_ptr);
	static const char lock(void* obj_ptr);

};

