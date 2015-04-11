#pragma once

// Pin mappings
const uint8_t air_flow_pin = A12;
const uint8_t air_temp_pin = A14;
const uint8_t o2_pin = A9;
const uint8_t coolant_temp_pin = A13;
const uint8_t oil_pressure_pin = A11;
const uint8_t tach_pin = 19;
const uint8_t idl_full_pin = A10;
const uint8_t cranking_pin = 10;

const uint8_t inputs[] = { air_flow_pin, air_temp_pin, o2_pin, coolant_temp_pin, oil_pressure_pin, tach_pin,
idl_full_pin, cranking_pin, '\0' };

// Output pins
const uint8_t inj2_pin = 42;
const uint8_t inj3_pin = 44;
const uint8_t inj4_pin = 46;
const uint8_t inj1_pin = 48;

const uint8_t coil1_pin = 28;
const uint8_t coil2_pin = 26;
const uint8_t coil3_pin = 24;
const uint8_t coil4_pin = 22;

const uint8_t fuel_pin = A8;
const uint8_t drv_en_pin = 38;
const uint8_t o2_pwr_pin = A1;
const uint8_t cs_knock_pin = 9;
const uint8_t cs_sd_pin = 12;
const uint8_t cs_inj_pin = 36;
const uint8_t inj_led_pin = 25;

uint8_t outputs[] = { inj1_pin, inj2_pin, inj3_pin, inj4_pin,
coil1_pin, coil2_pin, coil3_pin, coil4_pin,
fuel_pin, drv_en_pin, o2_pwr_pin,
cs_knock_pin, cs_sd_pin, cs_inj_pin, inj_led_pin, '\0' };
