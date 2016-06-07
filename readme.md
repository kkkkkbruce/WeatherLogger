#WeatherLogger

----------
This sketch shows Date & Time & current sensor readings to LCD
And logs sensor readings to CVC file on SD card every 2 secconds
----------

Hardware:
* LCD screen 20x4 using Adafruit i2c/SPI LCD backpack
* DHT 22 Temperature and Humidity sensor
* Adafruit MPL115A2 Pressure sensor breakout
* Adafruit Data Logger Shield


The circuit:
The LCD connected via i2c
* 5V to Arduino 5V pin
* GND to Arduino GND pin
* CLK to Analog #5
* DAT to Analog #4

The DHT 22
* VCC to Arduino 5V pin
* GND to Arduino GND pin
* DAT to Digital #2
* 10K resistor between VCC & DAT as pull up

The MPL115A2 via i2c
* VCC to Arduino 5V pin
* GND to Arduino GND pin
* SCL to Analog #5
* SDA to Analog #4
