#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "bme680_i2c.h"

#include "bme680_i2c.h"

// Helper function prototypes
//static void set_mode_oversampling_settings(i2c_inst_t *i2c, uint8_t mode, uint8_t temp_os, uint8_t press_os, uint8_t hum_os);
static void get_res_heat_x_calibr_parameters(i2c_inst_t *i2c, uint8_t par_g1, uint16_t par_g2, uint8_t par_g3, uint8_t res_heat_val, uint8_t res_heat_range);
static void get_calib_reg_data(i2c_inst_t *i2c);

//Parameters for converting raw data
static uint8_t  dig_P10, dig_H6;
static uint16_t dig_T1, dig_P1, dig_H1, dig_H2;
static int16_t  dig_T2, dig_P2, dig_P4, dig_P5, dig_P8, dig_P9, dig_GH2;
static int8_t   dig_T3, dig_P3, dig_P6, dig_P7, dig_H3, dig_H4, dig_H5, dig_H7, dig_GH1, dig_GH3;
static double t_fine;
// Data arrays for conversion of raw gas measurements into resistance
float const_array1[16] = {1, 1, 1, 1, 1, 0.99, 1, 0.992, 1, 1, 0.998, 0.995, 1, 0.99, 1, 1};
double const_array2[16] = {8000000.0, 4000000.0, 2000000.0, 1000000.0, 499500.4995, 248262.1648, 125000.0, 
63004.03226, 31281.28128, 15625.0, 7812.5, 3906.25, 1953.125, 976.5625, 488.28125, 244.140625};

void bme680_init(i2c_inst_t *i2c, int i2c_sda_pin, int i2c_scl_pin) {
    // Initialize the I2C port
    printf("I am here-1\n");
    i2c_init(i2c, 100 * 1000); // Initialize I2C at 100 kHz
    gpio_set_function(i2c_sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(i2c_scl_pin, GPIO_FUNC_I2C);
    gpio_pull_up(i2c_sda_pin);
    gpio_pull_up(i2c_scl_pin);

    // Configure the sensor
    printf("I am here-2\n");
    //set_mode_oversampling_settings(i2c, mode, press_os, temp_os, hum_os);
    printf("Success!\n");
}

void set_oversampling_settings(i2c_inst_t *i2c, uint8_t press_os, uint8_t temp_os, uint8_t hum_os) {
    // Define the register addresses
    uint8_t ctrl_meas_address = CTRL_MEAS; // ctrl_meas register for mode, temp, and press
    uint8_t ctrl_hum_address = CTRL_HUM;  // ctrl_hum register for humidity

    // Read the current value of the ctrl_meas register
    uint8_t read_value;
    uint8_t read_command[] = {ctrl_meas_address};
    i2c_write_blocking(i2c, BME680_ADDRESS, read_command, 1, true); // send register address
    i2c_read_blocking(i2c, BME680_ADDRESS, &read_value, 1, false); // read current value

    // Modify only the necessary bits for temperature and pressure oversampling
    read_value = (read_value & 0x81) | (temp_os << 5) | (press_os << 2); // Preserve mode bits [1:0] and modify [7:5] for temp and [4:2] for press

    // Write the updated configuration to the ctrl_meas register
    uint8_t ctrl_meas_command[] = {ctrl_meas_address, read_value};
    i2c_write_blocking(i2c, BME680_ADDRESS, ctrl_meas_command, 2, false);

    // Read, modify, and write for the humidity oversampling in the ctrl_hum register
    read_command[0] = ctrl_hum_address;
    i2c_write_blocking(i2c, BME680_ADDRESS, read_command, 1, true);
    i2c_read_blocking(i2c, BME680_ADDRESS, &read_value, 1, false);

    // Modify only the lower three bits for humidity oversampling
    read_value = (read_value & 0xF8) | (hum_os & 0x07);

    // Write the updated humidity configuration to the ctrl_hum register
    uint8_t ctrl_hum_command[] = {ctrl_hum_address, read_value};
    i2c_write_blocking(i2c, BME680_ADDRESS, ctrl_hum_command, 2, false);
}

void set_mode_settings(i2c_inst_t *i2c, uint8_t mode) {
    uint8_t ctrl_meas_address = CTRL_MEAS; // ctrl_meas register address

    // Read the current value of the ctrl_meas register
    uint8_t read_value;
    uint8_t read_command[] = {ctrl_meas_address};
    i2c_write_blocking(i2c, BME680_ADDRESS, read_command, 1, true); // send register address
    i2c_read_blocking(i2c, BME680_ADDRESS, &read_value, 1, false); // read current value

    // Modify only bits [0:1] for mode
    // Clear bits [0:1] and then set them to 'mode'
    read_value = (read_value & 0xFC) | (mode & 0x03);

    // Write the updated mode configuration to the ctrl_meas register
    uint8_t ctrl_meas_command[] = {ctrl_meas_address, read_value};
    i2c_write_blocking(i2c, BME680_ADDRESS, ctrl_meas_command, 2, false);
}

void set_iir_filter_settings(i2c_inst_t *i2c, uint8_t iir_filter) {
    uint8_t config_address = IIR_FILTER_CONFIG; // Assuming 0x75 is the address for the IIR filter configuration

    // Read the current value of the config register
    uint8_t read_value;
    uint8_t read_command[] = {config_address};
    i2c_write_blocking(i2c, BME680_ADDRESS, read_command, 1, true); // send register address
    i2c_read_blocking(i2c, BME680_ADDRESS, &read_value, 1, false); // read current value

    // Modify only bits [4:2] for IIR filter
    // Clear bits [4:2] and then set them to 'iir_filter'
    // Mask for clearing bits [4:2] is 0xE3 (binary 11100011)
    read_value = (read_value & 0xE3) | ((iir_filter & 0x07) << 2);

    // Write the updated IIR filter configuration to the config register
    uint8_t config_command[] = {config_address, read_value};
    i2c_write_blocking(i2c, BME680_ADDRESS, config_command, 2, false);
}

void enable_gas_conversion(i2c_inst_t *i2c) {
    uint8_t ctrl_gas_1_address = CTRL_GAS_1; // ctrl_gas_1 register address

    // Read the current value of the ctrl_gas_1 register
    uint8_t read_value;
    uint8_t read_command[] = {ctrl_gas_1_address};
    i2c_write_blocking(i2c, BME680_ADDRESS, read_command, 1, true); // send register address
    i2c_read_blocking(i2c, BME680_ADDRESS, &read_value, 1, false); // read current value

    // Set the run_gas bit to '1'
    read_value |= 0x10; // 0x10 is the hexadecimal representation of 00010000, which sets bit 4

    // Write the updated configuration to the ctrl_gas_1 register
    uint8_t ctrl_gas_1_command[] = {ctrl_gas_1_address, read_value};
    i2c_write_blocking(i2c, BME680_ADDRESS, ctrl_gas_1_command, 2, false);
}

void set_heater_set_point(i2c_inst_t *i2c, uint8_t set_point_index) {
    uint8_t ctrl_gas_1_address = CTRL_GAS_1; // ctrl_gas_1 register address

    // Ensure the set_point_index is within the 4-bit range
    set_point_index &= 0x0F; // Mask to keep only the lower 4 bits

    // Read the current value of the ctrl_gas_1 register
    uint8_t read_value;
    uint8_t read_command[] = {ctrl_gas_1_address};
    i2c_write_blocking(i2c, BME680_ADDRESS, read_command, 1, true); // send register address
    i2c_read_blocking(i2c, BME680_ADDRESS, &read_value, 1, false); // read current value

    // Modify only the nb_conv<3:0> bits
    read_value = (read_value & 0xF0) | set_point_index; // Clear the lower 4 bits and set them to set_point_index

    // Write the updated configuration to the ctrl_gas_1 register
    uint8_t ctrl_gas_1_command[] = {ctrl_gas_1_address, read_value};
    i2c_write_blocking(i2c, BME680_ADDRESS, ctrl_gas_1_command, 2, false);
}

void set_gas_wait_time(i2c_inst_t *i2c, uint8_t heater_set_point, uint8_t base_time, uint8_t multiplier) {
    if (heater_set_point > 9) {
        printf("Invalid Heater Set Point Index\n");
        return;
    }

    uint8_t gas_wait_address = GAS_WAIT_0 + heater_set_point; // Calculate the correct register address

    // Ensure the base_time is within the 6-bit range (0 to 63)
    base_time &= 0x3F; // Mask to keep only the lower 6 bits

    // Encode the multiplier
    uint8_t encoded_multiplier;
    switch (multiplier) {
        case 1:  encoded_multiplier = 0; break; // '00' in bits [7:6]
        case 4:  encoded_multiplier = 1; break; // '01' in bits [7:6]
        case 16: encoded_multiplier = 2; break; // '10' in bits [7:6]
        case 64: encoded_multiplier = 3; break; // '11' in bits [7:6]
        printf("Invalid Multiplier\n");
        default: return; // Invalid multiplier
    }

    // Combine base time and multiplier
    uint8_t gas_wait_value = (encoded_multiplier << 6) | base_time;

    // Write the heater-on time to the gas_wait_x register
    uint8_t gas_wait_command[] = {gas_wait_address, gas_wait_value};
    i2c_write_blocking(i2c, BME680_ADDRESS, gas_wait_command, 2, false);
}

void set_heater_off(i2c_inst_t *i2c) {
    uint8_t ctrl_register_address = CTRL_GAS_0; // Assuming 0x70 is the address for the control register

    // Read the current value of the control register
    uint8_t read_value;
    uint8_t read_command[] = {ctrl_register_address};
    i2c_write_blocking(i2c, BME680_ADDRESS, read_command, 1, true); // send register address
    i2c_read_blocking(i2c, BME680_ADDRESS, &read_value, 1, false); // read current value

    // Set bit 3 to '1'
    read_value |= 0x08; // 0x08 is the hexadecimal representation of 00001000, which sets bit 3

    // Write the updated configuration to the control register
    uint8_t ctrl_command[] = {ctrl_register_address, read_value};
    i2c_write_blocking(i2c, BME680_ADDRESS, ctrl_command, 2, false);
}

void set_heater_temperature(i2c_inst_t *i2c, uint8_t heater_set_point, uint8_t target_temp, double amb_temp) {
    if (heater_set_point > 9) {
        printf("Invalid heater set-point index\n");
        return;
    }

    // Retrieve calibration parameters (par_g1, par_g2, par_g3, res_heat_range, res_heat_val)
    // These should be read from the sensor's relevant registers or memory
    uint8_t par_g1; // Replace with actual value from sensor
    uint16_t par_g2; // Replace with actual value from sensor
    uint8_t par_g3; // Replace with actual value from sensor
    uint8_t res_heat_range; // Replace with actual value from sensor
    uint8_t res_heat_val; // Replace with actual value from sensor

    get_res_heat_x_calibr_parameters(i2c, par_g1, par_g2, par_g3, res_heat_range, res_heat_val);

    // Calculate the heater resistance code
    double var1 = ((double)par_g1 / 16.0) + 49.0;
    double var2 = (((double)par_g2 / 32768.0) * 0.0005) + 0.00235;
    double var3 = ((double)par_g3) / 1024.0;
    double var4 = var1 * (1.0 + (var2 * ((double)target_temp)));
    double var5 = var4 + (var3 * amb_temp);
    uint8_t heater_resistance_code = (uint8_t)(3.4 * ((var5 * (4.0 / (4.0 + (double)res_heat_range)) * (1.0 / (1.0 + ((double)res_heat_val * 0.002)))) - 25));

    uint8_t res_heat_address = RES_HEAT_0 + heater_set_point; // Calculate the correct register address

    // Write the heater resistance code to the res_heat_x register
    uint8_t res_heat_command[] = {res_heat_address, heater_resistance_code};
    i2c_write_blocking(i2c, BME680_ADDRESS, res_heat_command, 2, false);
}

void set_soft_reset(i2c_inst_t *i2c) {
    uint8_t reset_register_address = RESET_ADDR; // Reset register address
    uint8_t reset_command = 0xB6; // Reset command

    // Write the reset command to the reset register
    uint8_t reset_command_array[] = {reset_register_address, reset_command};
    i2c_write_blocking(i2c, BME680_ADDRESS, reset_command_array, 2, false);

    // Wait for the reset to take effect
    sleep_ms(10); // 10 milliseconds should be sufficient for the reset to complete
}

double get_calib_temp_data(i2c_inst_t *i2c, uint8_t raw_temp_msb, uint8_t raw_temp_lsb, uint8_t raw_temp_xlsb) {
    get_calib_reg_data(i2c);
    // Extract and combine the raw temperature values into temp_adc
    uint32_t temp_adc = ((uint32_t)raw_temp_msb << 12) | ((uint32_t)raw_temp_lsb << 4) | ((uint32_t)raw_temp_xlsb >> 4);

    // Apply the conversion formula
    double var1 = ((((double)temp_adc / 16384.0) - (dig_T1 / 1024.0)) * dig_T2);
    double var2 = (((((double)temp_adc / 131072.0) - (dig_T1 / 8192.0)) * (((double)temp_adc / 131072.0) - (dig_T1 / 8192.0))) * (dig_T3 * 16.0));
    t_fine = var1 + var2;
    double temperature = t_fine / 5120.0;

    return temperature; // Return the temperature in degrees Celsius
}

double get_calib_press_data(i2c_inst_t *i2c, uint8_t raw_press_msb, uint8_t raw_press_lsb, uint8_t raw_press_xlsb) {
    
    //get_calib_reg_data(i2c);
    // Extract and combine the raw temperature values into temp_adc
    uint32_t adc_P = ((uint32_t)raw_press_msb << 12) | ((uint32_t)raw_press_lsb << 4) | ((uint32_t)raw_press_xlsb >> 4);
    
    int32_t var1 = 0, var2 = 0, var3 = 0, var4 = 0, P = 0;
    var1 = (((int32_t) t_fine) >> 1) - 64000;
    var2 = ((((var1 >> 2) * (var1 >> 2)) >> 11) * (int32_t) dig_P6) >> 2;
    var2 = var2 + ((var1 * (int32_t)dig_P5) << 1);
    var2 = (var2 >> 2) + ((int32_t) dig_P4 << 16);
    var1 = (((((var1 >> 2) * (var1 >> 2)) >> 13) * ((int32_t) dig_P3 << 5)) >> 3) + (((int32_t) dig_P2 * var1) >> 1);
    var1 = var1 >> 18;
    var1 = ((32768 + var1) * (int32_t) dig_P1) >> 15;
    P = 1048576 - adc_P;
    P = (int32_t)((P - (var2 >> 12)) * ((uint32_t)3125));
    var4 = (1 << 31);
    
    if(P >= var4)
        P = (( P / (uint32_t) var1) << 1);
    else
        P = ((P << 1) / (uint32_t) var1);
        
    var1 = ((int32_t) dig_P9 * (int32_t) (((P >> 3) * (P >> 3)) >> 13)) >> 12;
    var2 = ((int32_t)(P >> 2) * (int32_t) dig_P8) >> 13;
    var3 = ((int32_t)(P >> 8) * (int32_t)(P >> 8) * (int32_t)(P >> 8) * (int32_t)dig_P10) >> 17;
    P = (int32_t)(P) + ((var1 + var2 + var3 + ((int32_t)dig_P7 << 7)) >> 4);
    
    return (double)P;

    //return (double)press_adc; // Return the temperature in degrees Celsius
}

double get_calib_hum_data(i2c_inst_t *i2c, uint8_t raw_hum_msb, uint8_t raw_hum_lsb) {
    //get_calib_reg_data(i2c);
    uint16_t adc_H = ((uint16_t)raw_hum_msb << 8) | raw_hum_lsb;
    int32_t var1 = 0, var2 = 0, var3 = 0, var4 = 0, var5 = 0, var6 = 0, H = 0, T = 0;

    T = (((int32_t) t_fine * 5) + 128) >> 8;
    var1 = (int32_t) adc_H  - ((int32_t) ((int32_t)dig_H1 << 4)) - (((T * (int32_t) dig_H3) / ((int32_t)100)) >> 1);
    var2 = ((int32_t)dig_H2 * (((T * (int32_t)dig_H4) / 
            ((int32_t)100)) + (((T * ((T * (int32_t)dig_H5) / 
            ((int32_t)100))) >> 6) / ((int32_t)100)) + (int32_t)(1 << 14))) >> 10;
    var3 = var1 * var2;
    var4 = ((((int32_t)dig_H6) << 7) + ((T * (int32_t) dig_H7) / ((int32_t)100))) >> 4;
    var5 = ((var3 >> 14) * (var3 >> 14)) >> 10;
    var6 = (var4 * var5) >> 1;

    H = (var3 + var6) >> 12;

    if (H > 102400) H = 102400; // check for over- and under-flow
    else if(H < 0) H = 0;

    return ((double)H / 1024.0);
}

double get_calib_gas_res_data(i2c_inst_t *i2c, uint8_t raw_gas_r_msb, uint8_t raw_gas_r_lsb) {
    uint8_t gas_range = raw_gas_r_lsb & 0x0F;
    // Extract bits [1:0] from raw_gas_r_lsb<7:6> and shift them to [1:0] position
    uint32_t gas_adc_lower = (raw_gas_r_lsb >> 6) & 0x03;

    // Extract bits [9:2] from raw_gas_r_msb<7:0> and shift them to [9:2] position
    uint32_t gas_adc_upper = (uint32_t)raw_gas_r_msb << 2;

    // Combine the upper and lower parts to form the complete gas_adc value
    uint32_t gas_adc = gas_adc_upper | gas_adc_lower;

    double var1; 
    double gas_switch_error = 1.0;
    var1 =  (1340.0 + 5.0 * gas_switch_error) * const_array1[gas_range];
    double gas_res = var1 * const_array2[gas_range] / (gas_adc - 512.0 + var1);
    return gas_res;

}

static void get_calib_reg_data(i2c_inst_t *i2c) {
    uint8_t calib[41];

    int i = 0;
    while(i < 25) {
        uint8_t reg_addr = BME680_CALIB_ADDR_1 + i;
        uint8_t read_command_1[] = {reg_addr};
        i2c_write_blocking(i2c, BME680_ADDRESS, read_command_1, 1, true); // send register address
        i2c_read_blocking(i2c, BME680_ADDRESS, &calib[i], 1, false); // read current value
        i++;
    }

    int j = 25;
    int k = 0;
    while(j < 41) {
        uint8_t reg_addr_2 = BME680_CALIB_ADDR_2 + k;
        uint8_t read_command_2[] = {reg_addr_2};
        i2c_write_blocking(i2c, BME680_ADDRESS, read_command_2, 1, true); // send register address
        i2c_read_blocking(i2c, BME680_ADDRESS, &calib[j], 1, false); // read current value
        k++;
        j++;
    }

    /*
    uint8_t read_command_1[] = {BME680_CALIB_ADDR_1};
    i2c_write_blocking(i2c, BME680_ADDRESS, read_command_1, 1, true); // send register address
    i2c_read_blocking(i2c, BME680_ADDRESS, &calib[0], 1, false); // read current value

    uint8_t read_command_2[] = {BME680_CALIB_ADDR_2};
    i2c_write_blocking(i2c, BME680_ADDRESS, read_command_2, 1, true); // send register address
    i2c_read_blocking(i2c, BME680_ADDRESS, &calib[25], 1, false); // read current value
    */

    //readBytes(BME680_ADDRESS, BME680_CALIB_ADDR_1, 25, &calib[0]);
    //readBytes(BME680_ADDRESS, BME680_CALIB_ADDR_2, 16, &calib[25]);
    // temperature compensation parameters
    dig_T1 = (uint16_t)(((uint16_t) calib[34] << 8) | calib[33]);
    dig_T2 = ( int16_t)((( int16_t) calib[2] << 8) | calib[1]);
    dig_T3 = (  int8_t)             (calib[3]);
    // pressure compensation parameters
    dig_P1 = (uint16_t)(((uint16_t) calib[6] << 8) | calib[5]);
    dig_P2 = ( int16_t)((( int16_t) calib[8] << 8) | calib[7]);
    dig_P3 =  ( int8_t)             (calib[9]);
    dig_P4 = ( int16_t)((( int16_t) calib[12] << 8) | calib[11]);
    dig_P5 = ( int16_t)((( int16_t) calib[14] << 8) | calib[13]);
    dig_P6 =  ( int8_t)             (calib[16]);
    dig_P7 =  ( int8_t)             (calib[15]);  
    dig_P8 = ( int16_t)((( int16_t) calib[20] << 8) | calib[19]);
    dig_P9 = ( int16_t)((( int16_t) calib[22] << 8) | calib[21]);
    dig_P10 = (uint8_t)             (calib[23]);
    // humidity compensation parameters
    dig_H1 =  (uint16_t)(((uint16_t) calib[27] << 4) | (calib[26] & 0x0F));
    dig_H2 =  (uint16_t)(((uint16_t) calib[25] << 4) | (calib[26] >> 4));
    dig_H3 =  (int8_t) calib[28];
    dig_H4 =  (int8_t) calib[29];
    dig_H5 =  (int8_t) calib[30];
    dig_H6 = (uint8_t) calib[31];
    dig_H7 =  (int8_t) calib[32];
    // gas sensor compensation parameters
    dig_GH1 =  (int8_t) calib[37];
    dig_GH2 = ( int16_t)((( int16_t) calib[36] << 8) | calib[35]);
    dig_GH3 =  (int8_t) calib[38];
}


uint8_t get_reg_val(i2c_inst_t *i2c, uint8_t reg_addr) {
    // Read the current value of the ctrl_meas register
    uint8_t read_value;
    uint8_t read_command[] = {reg_addr};
    i2c_write_blocking(i2c, BME680_ADDRESS, read_command, 1, true); // send register address
    i2c_read_blocking(i2c, BME680_ADDRESS, &read_value, 1, false); // read current value

    return read_value;
}

void printBinary(uint8_t value) {
    for (int i = 7; i >= 0; i--) {
        putchar((value & (1 << i)) ? '1' : '0');
    }
}

static void get_res_heat_x_calibr_parameters(i2c_inst_t *i2c, uint8_t par_g1, uint16_t par_g2, uint8_t par_g3, uint8_t res_heat_val, uint8_t res_heat_range) {
    
    //get par_g1 value from register
    uint8_t par_g1_addr[] = {PAR_G1};
    i2c_write_blocking(i2c, BME680_ADDRESS, par_g1_addr, 1, true);
    i2c_read_blocking(i2c, BME680_ADDRESS, &par_g1, 1, false);

    //get par_g2 value from register
    uint8_t buffer[2];
    uint8_t par_g2_addr[] = {PAR_G2_LSB};
    i2c_write_blocking(i2c, BME680_ADDRESS, par_g2_addr, 1, true);
    i2c_read_blocking(i2c, BME680_ADDRESS, buffer, 2, false);
    par_g2 = ((uint16_t)buffer[1] << 8) | buffer[0];

    //get par_g3 value from register
    uint8_t par_g3_addr[] = {PAR_G3};
    i2c_write_blocking(i2c, BME680_ADDRESS, par_g3_addr, 1, true);
    i2c_read_blocking(i2c, BME680_ADDRESS, &par_g3, 1, false);

    //get res_heat_range value from register
    uint8_t buffer_2[1];
    uint8_t res_heat_range_addr[] = {RES_HEAT_RANGE};
    i2c_write_blocking(i2c, BME680_ADDRESS, res_heat_range_addr, 1, true);
    i2c_read_blocking(i2c, BME680_ADDRESS, buffer_2, 1, false);
    res_heat_range = (buffer_2[0] & 0x30) >> 4;

    //get res_heat_val value from register
    uint8_t res_heat_val_addr[] = {RES_HEAT_VAL};
    i2c_write_blocking(i2c, BME680_ADDRESS, res_heat_val_addr, 1, true);
    i2c_read_blocking(i2c, BME680_ADDRESS, &res_heat_val, 1, false);

}

bool isNewDataAvailable(i2c_inst_t *i2c) {
    uint8_t status;
    uint8_t status_reg_addr[] = {MEAS_STATUS_0};
    i2c_write_blocking(i2c, BME680_ADDRESS, status_reg_addr, 1, true);
    i2c_read_blocking(i2c, BME680_ADDRESS, &status, 1, false);
    return (status & 0x80) != 0; // Check if the 7th bit is set
}


/*
static void set_mode_settings(i2c_inst_t *i2c, uint8_t mode, uint8_t press_os, uint8_t temp_os, uint8_t hum_os) {
    // Define the register addresses
    printf("I am here-3\n");
    uint8_t ctrl_meas_address = CTRL_MEAS; // ctrl_meas register for mode, temp, and press
    uint8_t ctrl_hum_address = CTRL_HUM;  // ctrl_hum register for humidity

    printf("I am here-4\n");
    // Construct the configuration byte for ctrl_meas
    // mode is in bits [1:0], press_os is in bits [4:2], temp_os is in bits [7:5] 
    uint8_t ctrl_meas_config = (temp_os << 5) | (press_os << 2) | mode;

    printf("I am here-5\n");
    // Write the configuration byte to the ctrl_meas register
    uint8_t ctrl_meas_command[] = {ctrl_meas_address, ctrl_meas_config};
    i2c_write_blocking(i2c, BME680_ADDRESS, ctrl_meas_command, 2, false);

    //printf("I am here-6\n");
    // Write the humidity oversampling setting to the ctrl_hum register
    //uint8_t ctrl_hum_command[] = {ctrl_hum_address, hum_os};
    //i2c_write_blocking(i2c, BME680_ADDRESS, ctrl_hum_command, 2, false);

    printf("I am here-6\n");
    uint8_t read_value;
    uint8_t read_command[] = {ctrl_hum_address};
    i2c_write_blocking(i2c, BME680_ADDRESS, read_command, 1, true); // send register address
    i2c_read_blocking(i2c, BME680_ADDRESS, &read_value, 1, false); // read current value

    printf("I am here-7\n");
    // Modify only the lower three bits
    read_value = (read_value & 0xF8) | (hum_os & 0x07);

    uint8_t ctrl_hum_command[] = {ctrl_hum_address, read_value};
    i2c_write_blocking(i2c, BME680_ADDRESS, ctrl_hum_command, 2, false);
    printf("I am here-8\n");
}
*/





