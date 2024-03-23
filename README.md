# esp-idf-i2c-benchmark
Benchmark for esp-idf i2c driver.   
A new i2c driver is now available in ESP-IDF V5.2.   
The previous i2c driver is still available as a Legacy driver.   
We measured the performance of New i2c driver and Legacy i2c driver.   

I used [this](https://github.com/UncleRus/esp-idf-lib/tree/master/components/mcp23x17) library as a reference.   
Thank you UncleRus.   


# Software requirements
ESP-IDF V5.2 or later.   


# Hardware requirements
MCP23017 16-Bit I/O Expander with i2c Interface.   


# Test circuit
We measured the time required to turn the GPA0 port ON/OFF 10,000 times using the test circuit below.   
![ESP32-MCP23017](https://github.com/nopnop2002/esp-idf-i2c-benchmark/assets/6020549/bf49dc49-062a-444b-8779-e3440bc0d5ca)


# Installation

```
git clone https://github.com/nopnop2002/esp-idf-i2c-benchmark
cd esp-idf-i2c-benchmark/
idf.py set-target {esp32/esp32s2/esp32s3/esp32c2/esp32c3/esp32c6/esp32h2}
idf.py menuconfig
idf.py flash
```

# Benchmark with Og compile option
The maximum I2C Clock frequency for the ESP32 i2c driver is 1.0MHz, but the ESP32C2, C3, and H3 do not work properly at 1MHz.   

|SoC@CPU Freq|i2c Clock Freq[MHz]|Legacy Driver[MillSec]|New Driver[MillSec]||
|:-:|:-:|:-:|:-:|:-:|
|ESP32@160|1.0|9420|6540||
|ESP32@240|1.0|7060|5290||
|ESP32S2@160|1.0|7420|4450||
|ESP32S2@240|1.0|5790|3830||
|ESP32S3@160|1.0|8700|5240||
|ESP32S3@240|1.0|6720|4450||
|ESP32C2@120|0.6|8890|5520|26MHz XTAL|
|ESP32C3@160|0.9|6550|3880||
|ESP32C6@160|1.0|6490|3850||
|ESP32H2@96|0.8|10150|5430||

ESP32C6 gave the fastest results when the CPU frequency was 160MHz (120MHz or 96MHz for some SoCs).   
![i2c-160Mhz](https://github.com/nopnop2002/esp-idf-i2c-benchmark/assets/6020549/eb2efe7f-e546-4cad-8cae-4605d0ca2af0)

ESP32S2 gave the fastest results when the CPU frequency was 240MHz.   
![i2c-240Mhz](https://github.com/nopnop2002/esp-idf-i2c-benchmark/assets/6020549/c97060c4-1414-4429-bfbc-e5b69c362982)