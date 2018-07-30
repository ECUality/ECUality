#pragma once

#include <SPI.h>


void InitSPI() {
	SPI.begin();
	SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
}


uint16_t InterrogateMC(uint16_t spi_send) {
	// Sends a 16-bit value, <to-send>, and reads back 16-bit response. 
	// Send is bracketed by a SPI-check on each side (before and after)
	uint16_t result = 0;

	digitalWrite(cs_inj_pin, LOW);
	_delay_us(100);
	SPI.transfer16(0x0F00);		// SPI check -ignore any previously buffered response
	digitalWrite(cs_inj_pin, HIGH);
	_delay_us(100);

	digitalWrite(cs_inj_pin, LOW);
	_delay_us(100);
	if (SPI.transfer16(spi_send) != 3338) 	// First byte sent
		ESerial.println(F("SPI Error"));	// expect SPI check reply.
	digitalWrite(cs_inj_pin, HIGH);
	_delay_us(100);

	digitalWrite(cs_inj_pin, LOW);
	_delay_us(100);
	result = SPI.transfer16(0x0F00);		// SPI check (reads response to previous)
	digitalWrite(cs_inj_pin, HIGH);
	_delay_us(100);

	return result;
}

void SPIInitSparkMode() {

	// 0x493D = 0100 1001 0011 1101
	//0100 (spark cmd)  
	//100 1 (max dwell = 32ms, max dwell protect enabled)  
	//0 0 1 1 (overlap dwell disabled, gain for 40mohm, soft shutdown enabled, open secondary clamp enabled) 
	//11 01 (open 2nd fault = 100us, end spark threshold = Vpwr + 5.5V) 

	// 0x4B3D = 0100 1011 0011 1101
	//0100 (spark cmd)  
	//101 1 (max dwell = 64ms, max dwell protect enabled)  
	//0 0 1 1 (overlap dwell disabled, gain for 40mohm, soft shutdown enabled, open secondary clamp enabled) 
	//11 01 (open 2nd fault = 100us, end spark threshold = Vpwr + 5.5V) 

	InterrogateMC(0x493D);	// Set max dwell = 32ms and enable soft shutdown
	_delay_us(1000);

	// 0x6A84 = 0110 1010 1000 0100
	//0110 (DAC cmd)  
	//1010 (Max current = 16A )  
	//100 (overlap setting: 50% ) 
	//00100 (Nominal current = 4A) 
	InterrogateMC(0x6A84);	// set Max current to 10A, Nominal current to 4.0A.
}


void SPIAllStatus() {
	uint16_t all_stat_reg, ignition_reg, out12_reg, out34_reg;

	all_stat_reg = InterrogateMC(0x0A00);	// read all-status register
	ignition_reg = InterrogateMC(0x0A40);	// read ignition status register
	out12_reg = InterrogateMC(0x0A10);		// read injector-drive 1,2 status 
	out34_reg = InterrogateMC(0x0A20);		// read injector-drive 3,4 status

	if (all_stat_reg)
	{
		ESerial.print(F("FAULTS: "));

		if (all_stat_reg & (1 << 15))
			ESerial.print(F("reset "));
		if (all_stat_reg & (1 << 14))
			ESerial.print(F("command "));
		if (all_stat_reg & (1 << 13))
			ESerial.print(F("supply "));
		if (all_stat_reg & (1 << 12))
			ESerial.print(F("NMF "));
		if (all_stat_reg & (1 << 11))
			ESerial.print(F("IGN3 "));
		if (all_stat_reg & (1 << 10))
			ESerial.print(F("IGN2 "));
		if (all_stat_reg & (1 << 9))
			ESerial.print(F("IGN1 "));
		if (all_stat_reg & (1 << 8))
			ESerial.print(F("IGN0 "));
		if (all_stat_reg & (1 << 7))
			ESerial.print(F("GP3 "));
		if (all_stat_reg & (1 << 6))
			ESerial.print(F("GP2 "));
		if (all_stat_reg & (1 << 5))
			ESerial.print(F("GP1 "));
		if (all_stat_reg & (1 << 4))
			ESerial.print(F("GP0 "));
		if (all_stat_reg & (1 << 3))
			ESerial.print(F("OUT3 "));
		if (all_stat_reg & (1 << 2))
			ESerial.print(F("OUT2 "));
		if (all_stat_reg & (1 << 1))
			ESerial.print(F("OUT1 "));
		if (all_stat_reg & (1 << 0))
			ESerial.print(F("OUT0 "));
		ESerial.println();
	}
	else 
	{
		ESerial.println(F("no faults"));
		return;
	}
	
	if (ignition_reg)
	{
		ESerial.print(F("IGN FAULTS: "));

		if (ignition_reg & (1 << 13))
			ESerial.print(F("OverV "));
		if (ignition_reg & (1 << 12))
			ESerial.print(F("LowV "));
		if (ignition_reg & (1 << 11))
			ESerial.print(F("3MaxI "));
		if (ignition_reg & (1 << 10))
			ESerial.print(F("3Dwell "));
		if (ignition_reg & (1 << 9))
			ESerial.print(F("3Open "));
		if (ignition_reg & (1 << 8))
			ESerial.print(F("2MaxI "));
		if (ignition_reg & (1 << 7))
			ESerial.print(F("2Dwell "));
		if (ignition_reg & (1 << 6))
			ESerial.print(F("2Open "));
		if (ignition_reg & (1 << 5))
			ESerial.print(F("1MaxI "));
		if (ignition_reg & (1 << 4))
			ESerial.print(F("1Dwell "));
		if (ignition_reg & (1 << 3))
			ESerial.print(F("1Open "));
		if (ignition_reg & (1 << 2))
			ESerial.print(F("0MaxI "));
		if (ignition_reg & (1 << 1))
			ESerial.print(F("0Dwell "));
		if (ignition_reg & (1 << 0))
			ESerial.print(F("0Open "));
		ESerial.println();
	}
	if (out12_reg || out34_reg)
	{
		ESerial.print(F("INJ FAULTS: "));

		if (out34_reg & (1 << 7))
			ESerial.print(F("3Temp "));
		if (out34_reg & (1 << 6))
			ESerial.print(F("3Short "));
		if (out34_reg & (1 << 5))
			ESerial.print(F("3OffOpen "));
		if (out34_reg & (1 << 4))
			ESerial.print(F("3OnOpen "));
		if (out34_reg & (1 << 3))
			ESerial.print(F("2Temp "));
		if (out34_reg & (1 << 2))
			ESerial.print(F("2Short "));
		if (out34_reg & (1 << 1))
			ESerial.print(F("2OffOpen "));
		if (out34_reg & (1 << 0))
			ESerial.print(F("2OnOpen "));

		if (out12_reg & (1 << 7))
			ESerial.print(F("1Temp "));
		if (out12_reg & (1 << 6))
			ESerial.print(F("1Short "));
		if (out12_reg & (1 << 5))
			ESerial.print(F("1OffOpen "));
		if (out12_reg & (1 << 4))
			ESerial.print(F("1OnOpen "));
		if (out12_reg & (1 << 3))
			ESerial.print(F("0Temp "));
		if (out12_reg & (1 << 2))
			ESerial.print(F("0Short "));
		if (out12_reg & (1 << 1))
			ESerial.print(F("0OffOpen "));
		if (out12_reg & (1 << 0))
			ESerial.print(F("0OnOpen "));

		if (digitalRead(coil1_pin))
		{
			ESerial.println();
			ESerial.print(F("Coil Pin High"));
		}
		ESerial.println();
	}
}

void SPISetNOMI(uint8_t nomi) {
	// start with 0110 0000 1000 0000  putting command (6), MaxI = 16A, and overlap setting (50%) in place. 
	//uint16_t data = 0x6080;  // for future feature when updating MaxI
	uint16_t data = 0x6A80;


	nomi &= 0x001F; // toss all bits but lowest 5
	//maxi &= 0x000F; // toss all bits but lowest 4
	//maxi <<= 8;		// rotate maxi to bits 11-8
	data += nomi;
	//data += maxi;
	InterrogateMC(data);
}