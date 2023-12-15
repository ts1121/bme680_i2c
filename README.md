# bme680_i2c
C/C++ library for interfacing a BME680 sensor module with RP2040, leveraging the I2C protocol and pico-sdk.

Prerequisite: pico-sdk.

To build and run the example code in the main.cpp file, as is:

-The Raspberry Pi Pico board comprises two I2C channels (I2C0 and I2C1), and the example code uses the I2C0 channel. Use the following I2C pin connections with the Raspberry Pi Pico board:
1) (BME680) SCL --> (Pico) GP5/Pin# 7
2) (BME680) SDA --> (Pico) GP4/Pin# 6
3) (BME680) SDO --> (Pico) GND (This assigns the BME680 module with an address of 0x76)
4) (BME680) VCC --> (Pico) VCC
5) (BME680) GND --> (Pico) GND

-The example code in the main.cpp file, executes the following basic steps:
1) Establish I2C connection.
2) Set oversampling for T, P and H.
3) Set IIR filter for the temperature sensor.
4) Enable gas conversion.
5) Set index of heater set point.
6) Define heater-on time.
7) Set heater temperature.
8) Set mode to forced mode.
9) Read and print calibrated TPHG data in (Degree Celsius, Kilo Pascal, % of Relative Humidilty, Kilo Ohms).



