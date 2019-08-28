![enter image description here](https://github.com/EasySensors/ButtonSizeNode3/blob/master/pics/BS3_LITE_TOP.jpg?raw=true)
![enter image description here](https://github.com/EasySensors/ButtonSizeNode3/blob/master/pics/BS3_LITE_BOTTOM_RADIO.jpg?raw=true)
![enter image description here](https://github.com/EasySensors/ButtonSizeNode3/blob/master/pics/BS3_LITE_BOTTOM.jpg?raw=true)


**The Button Size Node 3 is a low cost wireless Arduino IDE compatible (the Atmel ATMega328P 8MHz) microcontroller with LoRa RFM 95 or RFM 69 HW(CW) radio on board and few other nice additions.** 
------------------------------------------------------------------------

Best sutable for Home Automation, IOT. Could be used as core board for radio controlling any DIY project. You may think of it as Arduino Pro Mini plus all the items in the picture below::

![](https://github.com/EasySensors/ButtonSizeNode/blob/master/pics/replceA.jpg?raw=true)

## Specification: ##
 - Dimensions XXmm x XXmm
 - Wide operating temperature range. Tested -20 +40 Celsius
 - Sleep current consumption 9 - 12 uA
 - Temperature and humidity sensor Si7021 
 - High Accuracy Temperature Sensor ±0.4 °C (max), –10 to 85 °C
 - Precision Relative Humidity Sensor ± 3% RH (max), 0–80% RH
 - Light sensor BH1750,  spectral responsibility is approximately human eye response.
 - Authentication security - Atmel ATSHA204A Crypto Authentication Chip
 - External JDEC EPROM
 - Dualoptiboot bootloader. Implements over the air (OTA) firmware update ability
 - LoRa RFM 95 or RFM 69-HW (high power version) or CW (low power consumption version) 915, 868 or 433 MHz Radio transceivers
 - Battery voltage sensor (via divider)
 - Supply voltage  3.5-10 Volts
 - The Digital and Analog pins are 3.3 volts
 - Powered by two CR2032 batteries in series with high-efficiency power converter (3.5-10V). 
 - FTDI  header for programming
 - Footprints 
 - LED connected to pin XX
 - Reset button


**Pin out:** 


Arduino Pins|	Description
------------|--------------
A0, A1 |	Available ARDUINO analog GPIO / DIGITAL GPIO
A6 |	Connected to Battery voltage sensor (via divider)
A4 |	Connected to si1132 and bh1750 SDA 
A5 |	Connected to si1132 and bh1750 SCL
A3 |	Connected to  ATSHA204A
D3, D4, D5, D6,D7, D9 |	Available ARDUINO digital GPIO
D8 |	Connected to CS FLASH chip (OTA) M25P40
MISO, MOSI, SCK, RST |	Connected to ISP header
ANT |	RFM69 antenna
Vcc and Bat+ | Unregulated power up to 6.5 Volts
Gnd | Ground


**Arduino IDE Settings**

![Arduino IDE Settings](https://github.com/EasySensors/ButtonSizeNode/blob/master/pics/IDEsettings.jpg?raw=true)


**programming FTDI adapter connection**

![enter image description here](https://github.com/EasySensors/ButtonSizeNode/blob/master/pics/FTDIvcc5-3.jpg?raw=true)


Both 3.3V and 5V power options can be used.

How to use it as home automation (IOT) node controller
------------------------------------------------------


ButtonSizeNode.ino is the Arduino example sketch using [MySensors](https://www.mysensors.org/) API. 

Connect the Node to FTDI USB adaptor, Select Pro Mini 8MHz board in Arduino IDE and upload the ButtonSizeNode.ino sketch.

**Done**


The board is created by  [Koresh](https://www.openhardware.io/user/143/projects/Koresh)

