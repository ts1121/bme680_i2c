# bme680_i2c
C/C++ library for I2C connection of BME680 with RP2040, leveraging the pico-sdk. Reads and prints TPHG data in (Degree Celsius, Kilo Pascal, % of Relative Humidilty, Kilo Ohms).

Raspberry Pi Pico board comprises two I2C channels (I2C0 and I2C1).
The example code in the main.cpp file, uses the I2C0 channel. To compile and run the example main.cpp file as is, use the following I2C pin connections with the Raspberry Pi Pico boards:
1) (BME680) SCL --> (Pico) GP5/Pin# 7
2) (BME680) SDA --> (Pico) GP4/Pin# 6
3) (BME680) SDO --> (Pico) GND (This assigns the BME680 with an address of 0x76)
4) (BME680) VCC --> (Pico) VCC
5) (BME680) GND --> (Pico) GND


