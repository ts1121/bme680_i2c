#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "bme680_i2c.h"

//Helper function prototype to print register values for verification
static void test_print_reg_values(uint8_t ctrl_meas_settings, uint8_t ctrl_hum_settings, uint8_t iir_filter_settings,  
                                    uint8_t ctrl_gas_1_val, uint8_t res_heat_0_val, uint8_t par_g1_val, 
                                    uint8_t par_g2_lsb_val, uint8_t par_g2_msb_val, uint8_t par_g3_val, 
                                    uint8_t res_heat_range_val, uint8_t res_heat_val_val, uint8_t gas_wait_x,
                                     uint8_t res_heat_addr);
const uint LED_PIN = 25;

int main() {

   bi_decl(bi_program_description("This is a test binary."));
   bi_decl(bi_1pin_with_name(LED_PIN, "On-board LED"));

    //initialize stdio
   stdio_init_all();

   //Set up LED Pin
   gpio_init(LED_PIN);
   gpio_set_dir(LED_PIN, GPIO_OUT);

   uint8_t mode = 0x01;
   uint8_t press_os = 0x01;
   uint8_t temp_os = 0x01;
   uint8_t hum_os = 0x01;
   uint8_t iir_filter = 0x02;
   uint8_t set_point_index = 0x02;
   uint8_t heater_set_point = 0x01;
   uint8_t base_time = 0x02;
   uint8_t multiplier = 0x01;
   uint8_t gas_wait_x = GAS_WAIT_0 + heater_set_point;
   uint8_t target_temp = 200;
   double amb_temp = 25;
   uint8_t res_heat_addr = RES_HEAT_0 + heater_set_point;


   bme680_init(i2c0,4,5);
   set_oversampling_settings(i2c0,press_os,temp_os,hum_os);
   set_iir_filter_settings(i2c0,iir_filter);
   enable_gas_conversion(i2c0);
   set_heater_set_point(i2c0,set_point_index);
   set_gas_wait_time(i2c0,heater_set_point,base_time,multiplier);
   set_heater_temperature(i2c0,heater_set_point,target_temp,amb_temp);
   set_mode_settings(i2c0,mode);

   //Register values To be used as parameters for the function test_print_reg_values() 
   /*
   uint8_t ctrl_meas_settings = get_reg_val(i2c0,CTRL_MEAS);
   uint8_t ctrl_hum_settings = get_reg_val(i2c0,CTRL_HUM);
   uint8_t iir_filter_settings = get_reg_val(i2c0,IIR_FILTER_CONFIG);
   uint8_t ctrl_gas_1_val = get_reg_val(i2c0,CTRL_GAS_1);
   uint8_t res_heat_0_val = get_reg_val(i2c0,RES_HEAT_0);
   uint8_t par_g1_val = get_reg_val(i2c0,PAR_G1);
   uint8_t par_g2_lsb_val = get_reg_val(i2c0,PAR_G2_LSB);
   uint8_t par_g2_msb_val = get_reg_val(i2c0,PAR_G2_MSB);
   uint8_t par_g3_val = get_reg_val(i2c0,PAR_G3);
   uint8_t res_heat_range_val = get_reg_val(i2c0,RES_HEAT_RANGE);
   uint8_t res_heat_val_val = get_reg_val(i2c0,RES_HEAT_VAL);
   */

    while(1) {
        //LED Blink
        printf("Blinking\n");
        //test_print_reg_values(ctrl_meas_settings, ctrl_hum_settings, iir_filter_settings, ctrl_gas_1_val, res_heat_0_val, par_g1_val, par_g2_lsb_val, par_g2_msb_val, par_g3_val, 
        //                            res_heat_range_val, res_heat_val_val, gas_wait_x, res_heat_addr);

        uint8_t raw_temp_msb = get_reg_val(i2c0,TEMP_MSB_ADDR);
        uint8_t raw_temp_lsb = get_reg_val(i2c0,TEMP_LSB_ADDR);
        uint8_t raw_temp_xlsb = get_reg_val(i2c0,TEMP_XLSB_ADDR);
        double temperature = get_calib_temp_data(i2c0, raw_temp_msb, raw_temp_lsb, raw_temp_xlsb);

        uint8_t raw_press_msb = get_reg_val(i2c0,PRESS_MSB_ADDR);
        uint8_t raw_press_lsb = get_reg_val(i2c0,PRESS_LSB_ADDR);
        uint8_t raw_press_xlsb = get_reg_val(i2c0,PRESS_XLSB_ADDR);
        double pressure = get_calib_press_data(i2c0, raw_press_msb,raw_press_lsb,raw_press_xlsb);

        uint8_t raw_hum_msb = get_reg_val(i2c0,HUM_MSB_ADDR);
        uint8_t raw_hum_lsb = get_reg_val(i2c0,HUM_LSB_ADDR);
        double humidity = get_calib_hum_data(i2c0,raw_hum_msb,raw_hum_lsb);

        uint8_t raw_gas_msb = get_reg_val(i2c0,GAS_R_MSB_ADDR);
        uint8_t raw_gas_lsb = get_reg_val(i2c0,GAS_R_LSB_ADDR);
        double gas_resistance = get_calib_gas_res_data(i2c0,raw_gas_msb,raw_gas_lsb);

        printf("Temperature (*C): %f\n",(float)temperature);
        printf("Pascal (KPa): %f\n",(float)(pressure/1000));
        printf("Humidity (%%RH): %f\n",(float)humidity);
        printf("Gas Resistance (KOhms): %f\n",(float)(gas_resistance/1000));
        printf("\n");
        
        
        gpio_put(LED_PIN,0);
        sleep_ms(500);
        gpio_put(LED_PIN, 1);
        sleep_ms(500);
        
        while(!isNewDataAvailable(i2c0)) {
            //printf("Waiting for new data!\n");
            set_mode_settings(i2c0,mode);
        }
    }
    return 0;
}

static void test_print_reg_values(uint8_t ctrl_meas_settings, uint8_t ctrl_hum_settings, uint8_t iir_filter_settings,  
                                    uint8_t ctrl_gas_1_val, uint8_t res_heat_0_val, uint8_t par_g1_val, 
                                    uint8_t par_g2_lsb_val, uint8_t par_g2_msb_val, uint8_t par_g3_val, 
                                    uint8_t res_heat_range_val, uint8_t res_heat_val_val, uint8_t gas_wait_x,
                                     uint8_t res_heat_addr) {
    
    printf("ctrl_meas_settings: ");
    printBinary(ctrl_meas_settings);
    printf("\n");
    printf("ctrl_hum_settings: ");
    printBinary(ctrl_hum_settings);
    printf("\n");
    printf("iir_filter_settings: ");
    printBinary(iir_filter_settings);
    printf("\n");
    printf("ctrl_gas_1_val: ");
    printBinary(ctrl_gas_1_val);
    printf("\n");
    printf("gas_wait_x: ");
    printBinary(gas_wait_x);
    printf("\n");
    printf("res_heat_addr: ");
    printBinary(res_heat_addr);
    printf("\n");
    printf("res_heat_0_val: ");
    printBinary(res_heat_0_val);
    printf("\n");
    printf("par_g1_val: ");
    printBinary(par_g1_val);
    printf("\n");
    printf("par_g2_lsb_val: ");
    printBinary(par_g2_lsb_val);
    printf("\n");
    printf("par_g2_msb_val: ");
    printBinary(par_g2_msb_val);
    printf("\n");
    printf("par_g3_val: ");
    printBinary(par_g3_val);
    printf("\n");
    printf("res_heat_range_val: ");
    printBinary(res_heat_range_val);
    printf("\n");
    printf("res_heat_val_val: ");
    printBinary(res_heat_val_val);
    printf("\n");

}