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
	// 0x473D = 
	// <11-8> max dwell = 16ms, max dwell protect enabled, 
	// <7-6> overlap dwell disabled, feedback gain for 40mohm, 
	// <5-4> soft shutdown enabled, open 2nd clamp enabled, 
	// <3-0> open 2nd fault = 100us, end spark threshold = Vpwr + 5.5V. 
	InterrogateMC(0x473D);	// Set max dwell = 16ms and enable soft shutdown
}


void SPIAllStatus() {
	uint16_t all_stat_reg, ignition_reg, out12_reg, out34_reg;

	all_stat_reg = InterrogateMC(0x0A00);	// read all-status register
	ignition_reg = InterrogateMC(0x0A40);	// read ignition status register
	out12_reg = InterrogateMC(0x0A10);		// read injector-drove 1,2 status 
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
		return;
	
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

		ESerial.println();
	}
}

void SPISetNOMI(uint8_t nomi) {
	uint16_t data = 0x6880;

	nomi &= 0x001F;
	data += nomi;
	InterrogateMC(data);
}