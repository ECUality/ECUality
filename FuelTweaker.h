#pragma once

#include "Map.h"
#include "Parameter.h"


class FuelTweaker
{
	uint8_t lockout;
	uint8_t mode;
	uint8_t o2_open;
	uint8_t direction;
	unsigned int time_warming_o2; 
	unsigned int time_eng_warm; 
	unsigned int rpm_old;
	unsigned int local_sum;
	unsigned int ee_addy;
	
	const unsigned char& run_condition;
	const int& o2;
	const int& air_flow;
	const int& rpm;
	int& global_offset;

	Map& offset_map;
	Map& change_map; 

	void tweakGlobalvRPM();
	void tweakGlobalvO2();
	void tweakLocalAndGlobalvO2();
	
public:
	Parameter	o2_upper_thresh;
	Parameter	o2_lower_thresh;	
	Parameter	time_warming_o2_thresh;
	Parameter	step_size;
	Parameter	local_sum_limit;	
	Parameter	time_eng_warm_thresh;


	FuelTweaker(const unsigned char& run_condition, const int& o2_ptr, const int& air_flow_, const int& rpm_, 
		int& global_correction, Map& offset_map, Map& change_map);
	~FuelTweaker();
	void tweak();
	static const char status(void* obj_ptr);
	static const char reportParams(void* obj_ptr);
	static const char load(void* obj_ptr);
	static const char save(void* obj_ptr);
	static const char lock(void* obj_ptr);

};

