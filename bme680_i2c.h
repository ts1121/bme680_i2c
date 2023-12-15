#ifndef BME680_i2c_H_
#define BME680_i2c_H_

#include "pico/stdlib.h"
#include "hardware/i2c.h"

/*
i2c sensor module slave addr. Addr depends on SDO connection.
If SDO connected to GND -> addr is 0x76.
Or, if SDO connected to VDDIO -> addr is 0x77.
SDO pin cannot be left floating.
If left floating, addr will be undefined. 
*/
#define BME680_ADDRESS 0x76

/*
Mode Ctrl (mode)/Pressure Oversampling (osrs_p)/Temperature Oversampling (osrs_t) register addr. 
mode<1:0>: the first 2 bits of this register contains the mode value. 
bit-1: MSB of mode value, and bit-0: LSB of mode value.
    mode value 0x00 -> sleep mode.
    mode value 0x01 -> forced mode.
osrs_p<4:2>: the bits-2,3 and 4 of this register contains the 3 bit value for pressure oversampling.
osrs_t<7:5>: the bits-5,6 and 7 of this register contains the 3 bit value for temperature oversampling.
*/
#define CTRL_MEAS 0x74

/*
Humidity Oversampling (osrs_h) register addr.
osrs_h<2:0>: the bits-0,1, and 2 of this register contains the 3 bit value for humidity oversampling.
*/
#define CTRL_HUM 0x72

/*
IIR Filter Config register addr.
filter<4:2>: the bits-2,3, and 4 of this register contains the 3 bit value for iir filter temperature coefficient.
*/
#define IIR_FILTER_CONFIG 0x75

/*
CTRL_GAS_1 register addr.
The fifth bit of this register is run_gas.
run_gas needs to be set as 1 to enable gas conversion. 
*/
#define CTRL_GAS_1 0x71

/*
CTRL_GAS_0 register addr. Register to turn off heater.
Bit 3 (4th Bit) needs to be set to 1 to turn off heater.
*/
#define CTRL_GAS_0 0x70

/*
GAS_WAIT_X register addr, where X = 0 (X is an integer & 0<= X <=9). 
*/
#define GAS_WAIT_0 0x64

/*
Parameter register addr for RES_HEAT_X calculation. 
*/
#define RES_HEAT_0 0x5A

#define PAR_G1 0xED

#define PAR_G2_LSB 0xEB

#define PAR_G2_MSB 0xEC

#define PAR_G3 0xEE

#define RES_HEAT_RANGE 0x02

#define RES_HEAT_VAL 0x00

#define RESET_ADDR 0xE0

/*Status Register Addr.*/
#define MEAS_STATUS_0 0x1D 

/*Data Registers*/
#define TEMP_MSB_ADDR 0x22
#define TEMP_LSB_ADDR 0x23
#define TEMP_XLSB_ADDR 0x23
#define PRESS_MSB_ADDR 0x1F
#define PRESS_LSB_ADDR 0x20
#define PRESS_XLSB_ADDR 0x20
#define HUM_MSB_ADDR 0x25
#define HUM_LSB_ADDR 0x26
#define GAS_R_MSB_ADDR 0x2A
#define GAS_R_LSB_ADDR 0x2B

#define BME680_CALIB_ADDR_1 0x89  // 25 bytes of calibration data for I2C
#define BME680_CALIB_ADDR_2 0xE1  // 16 bytes of calibration data for I2C

/*Initialization Function.
Sets up i2c instance.
*/
void bme680_init(i2c_inst_t *i2c, int i2c_sda_pin, int i2c_scl_pin);

/*
Sets up oversampling for pressure, temperature, humidity.
*/
void set_oversampling_settings(i2c_inst_t *i2c, uint8_t press_os, uint8_t temp_os, uint8_t hum_os);

/*
Sets up mode settings.
*/
void set_mode_settings(i2c_inst_t *i2c, uint8_t mode);

void set_iir_filter_settings(i2c_inst_t *i2c, uint8_t iir_filter);

void enable_gas_conversion(i2c_inst_t *i2c);

void set_heater_set_point(i2c_inst_t *i2c, uint8_t set_point_index);

void set_gas_wait_time(i2c_inst_t *i2c, uint8_t heater_set_point, uint8_t base_time, uint8_t multiplier);

void set_heater_off(i2c_inst_t *i2c);

void set_heater_temperature(i2c_inst_t *i2c, uint8_t heater_set_point, uint8_t target_temp, double amb_temp);

double get_calib_temp_data(i2c_inst_t *i2c, uint8_t raw_temp_msb, uint8_t raw_temp_lsb, uint8_t raw_temp_xlsb);

double get_calib_press_data(i2c_inst_t *i2c, uint8_t raw_press_msb, uint8_t raw_press_lsb, uint8_t raw_press_xlsb);

double get_calib_hum_data(i2c_inst_t *i2c, uint8_t raw_hum_msb, uint8_t raw_hum_lsb);

double get_calib_gas_res_data(i2c_inst_t *i2c, uint8_t raw_gas_r_msb, uint8_t raw_gas_r_lsb);

uint8_t get_reg_val(i2c_inst_t *i2c, uint8_t reg_addr);

void printBinary(uint8_t value);

bool isNewDataAvailable(i2c_inst_t *i2c);

void set_soft_reset(i2c_inst_t *i2c);

#endif
