<div align="right">
	<br>
	<a href="https://www.aliexpress.com/item/1005005118146988.html">
		<img src="pics/ali_buy_btn.png"  alt="Click to buy from Aliexpress">
	</a>
	<br>
</div>

<a href="https://www.tindie.com/products/17981/?ref=offsite_badges&utm_source=sellers_easySensors&utm_medium=badges&utm_campaign=badge_large"><img src="https://d2ss6ovg47m0r5.cloudfront.net/badges/tindie-larges.png" alt="I sell on Tindie" width="200" height="104"></a>

## Full Version ##
![The Button Size Node 3](https://github.com/EasySensors/ButtonSizeNode3/blob/master/pics/BS3_TOP.jpg?raw=true)

## Light Version ##
![The Button Size Node 3](https://github.com/EasySensors/ButtonSizeNode3/blob/master/pics/BS3_LITE_TOP.jpg?raw=true)
![The Button Size Node 3](https://github.com/EasySensors/ButtonSizeNode3/blob/master/pics/BS3_LITE_BOTTOM_RADIO.jpg?raw=true)
![The Button Size Node 3](https://github.com/EasySensors/ButtonSizeNode3/blob/master/pics/BS3_LITE_BOTTOM.jpg?raw=true)


**The Button Size Node 3 is a low cost wireless Arduino IDE compatible (the Atmel ATMega328P 8MHz) microcontroller with LoRa RFM 95 or RFM 69 HW(CW) radio on board and few other nice additions.** 
------------------------------------------------------------------------

Best suitable for Home automation, IOT. Could be used as core board for radio controlling any DIY project. You may think of it as Arduino Pro Mini plus all the items in the picture below::

![](https://github.com/EasySensors/ButtonSizeNode/blob/master/pics/replceA.jpg?raw=true)

## Specification: ##
 - 4 Layers PCB with larger ground plane improving range. 20-40% range increase according to our field tests.
 - Dimensions 45mm x 23mm
 - Wide operating temperature range. Tested -20 +40 Celsius
 - Sleep current consumption 9 - 12 uA
 - Temperature and humidity sensor Si7021 
 - High Accuracy Temperature Sensor ±0.4 °C (max), –10 to 85 °C
 - Precision Relative Humidity Sensor ± 3% RH (max), 0–80% RH
 - Light sensor BH1750,  spectral responsibility is approximately human eye response. 
 - OTA FLASH (W25X40CLNIG)
 - Accelerometer LIS3DH (**Full version only**)
 - Barometer BMP280 (Full **Full version only**)
 - Magnet sensor SM351LT (**Full version only**)
 - Authentication security - Atmel ATSHA204A Crypto Authentication Chip
 - External JDEC EPROM
 - Dualoptiboot bootloader. Implements over the air (OTA) firmware update ability
 - LoRa RFM 95 or RFM 69-HW (high power version) or CW (low power consumption version) 915, 868 or 433 MHz Radio transceivers
 - Battery voltage sensor (via divider)
 - Supply voltage  3.5-10 Volts
 - The Digital and Analog pins are 3.3 volts
 - Powered by two CR2032 batteries in series with high-efficiency power converter (3.5-10V). 
 - FTDI  header for programming
 - LED connected to pin 6
 - Reset button


**Pin out:** 


Arduino Pins|	Description
------------|--------------
A0, A1 |	Available ARDUINO analog GPIO / DIGITAL GPIO
A2 RFM69/95 reset pin
A6 |	Connected to Battery voltage sensor (via divider) 3M/470k 
A4 |	Connected to sensors i2c
A5 |	Connected to sensors i2c
A3 |	Connected to  ATSHA204A
D3, D4, D5, D6,D7, D9 |	Available ARDUINO digital GPIO
D4 | accelerometer interrupt PCINT20 connected 
D5 | magnetic sensor state\interrupt PCINT21 connected 
D6 | LED connected
D8 |	Connected to CS FLASH chip (OTA) M25P40
D2 |	Connected to RFM 69 DIO0 
D9 | Connected to RFM 69 Reset pin 
D10 |	Connected to RFM 69 CS/NSS
D11 |	MOSI
D12 |	MISO
D13 |	SCK
ANT |	RFM69 antenna
Vcc and Bat+ | Unregulated power up to 6.5 Volts is connected before DC-DC converter
Gnd | Ground


**Arduino IDE Settings**

![Arduino IDE Settings](https://github.com/EasySensors/ButtonSizeNode/blob/master/pics/IDEsettings.jpg?raw=true)


**Programming FTDI adapter connection**

![enter image description here](https://github.com/EasySensors/ButtonSizeNode/blob/master/pics/FTDIvcc5-3.jpg?raw=true)


Both 3.3V and 5V power options can be used.

How to use it as home automation (IOT) node controller
------------------------------------------------------


ButtonSizeNode.ino is the Arduino example sketch using [MySensors](https://www.mysensors.org/) API. 

Connect the Node to FTDI USB adaptor, Select Pro Mini 8MHz board in Arduino IDE and upload the ButtonSizeNode.ino sketch. The skecth will create node fith fixed address in Mysensors network. 

**Done**

[Schematics](https://github.com/EasySensors/ButtonSizeNode3/blob/master/ButtonSizedNodeV3.pdf)


The board is created by  [Koresh](https://www.openhardware.io/user/143/projects/Koresh)

