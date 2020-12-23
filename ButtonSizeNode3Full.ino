#include <avr/wdt.h>
#ifdef __AVR__
#include <avr/power.h>
#endif
#include <stdlib.h>

//#include <MemoryFree.h>

//#include <Wire.h>
// #include <ODROID_Si1132.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <BH1750.h>
BH1750 lightMeter;

#include "SparkFun_Si7021_Breakout_Library.h"
//Create Instance of HTU21D or SI7021 temp and humidity sensor and MPL3115A2 barrometric sensor
Weather sensor;

Adafruit_LIS3DH lis = Adafruit_LIS3DH();
Adafruit_BMP280 bmp;

// ------------------------------------------ The node settings start
// Enable debug prints to serial monitor
#define MY_DEBUG
//#define MY_DEBUG_VERBOSE_SIGNING

// Comment it out for Auto Node ID #
#define MY_NODE_ID 0xf0

//  receiving node addrees for magnet sensor states reporting 
#define  relayNodeIDmagSensor 0 

//#define MY_RFM69_NETWORKID 111

// Enable and select radio type attached
#define MY_RADIO_RFM69

// Enable and select radio type attached MY_RADIO_RFM95
//#define MY_RADIO_RFM95


// Comment it out for CW version radio.
// #define MY_IS_RFM69HW

//#define MY_RFM69_FREQUENCY   RFM69_915MHZ
//#define MY_RFM69_FREQUENCY   RFM69_868MHZ
//#define MY_RFM69_FREQUENCY   RFM69_433MHZ
  



//#define   MY_RFM95_FREQUENCY RFM95_868MHZ
//#define   MY_RFM95_FREQUENCY RFM95_915MHZ
#define   MY_RFM95_FREQUENCY RFM95_434MHZ

//#define MY_RFM95_MODEM_CONFIGRUATION  RFM95_BW125CR45SF128
#define MY_RFM95_MODEM_CONFIGRUATION RFM95_BW_500KHZ | RFM95_CODING_RATE_4_5, RFM95_SPREADING_FACTOR_2048CPS | RFM95_RX_PAYLOAD_CRC_ON, RFM95_AGC_AUTO_ON // 
#define MY_RFM95_TX_POWER_DBM (14u)
//#define MY_RFM95_MODEM_CONFIGRUATION RFM95_BW125CR48SF4096
//RFM95_BW125CR45SF128

// air-time approximation for timeout, 1 hop ~15 bytes payload - adjust if needed
// BW125/SF128: 50ms
// BW500/SF128: 15ms
// BW31.25/SF512: 900ms
// BW125/SF4096: 1500ms

#define RFM95_RETRY_TIMEOUT_MS      (2500ul)      //!< Timeout for ACK, adjustments needed if modem configuration changed (air time different)




// Avoid battery drain if Gateway disconnected and the node sends more than MY_TRANSPORT_STATE_RETRIES times message.
#define MY_TRANSPORT_UPLINK_CHECK_DISABLED
#define MY_PARENT_NODE_IS_STATIC
#define MY_PARENT_NODE_ID 0

//Enable OTA feature
//#define MY_OTA_FIRMWARE_FEATURE
//#define MY_OTA_FLASH_JDECID 0//0x2020

// Redefining write codes for JDEC FLASH used in the node
// These two defines should always be after #include declaration
#define SPIFLASH_BLOCKERASE_32K 0xD8
#define SPIFLASH_CHIPERASE 0x60

//Enable Crypto Authentication to secure the node
//#define MY_SIGNING_ATSHA204
//#define MY_SIGNING_REQUEST_SIGNATURES

// ------------------------------------------ The node settings end




#include <MySensors.h>


// Assign numbers for all sensors we will report to gateway\controller (they will be created as child devices)
#define BATT_sensor 1
#define HUM_sensor 2
#define TEMP_sensor 3
#define VIS_sensor 4
#define UVIndex_sensor 5
#define Visible_sensor 6 //Si1132
#define IR_sensor 7
#define P_sensor 8
#define M_sensor 9
#define G_sensor 10


#define BATTERY_SENSE_PIN A6 // select the input pin for the battery sense point
#define LED_PIN 6
#define MAGNET_PIN 5
#define GYRO_PIN   4

// Create MyMessage Instance for sending readins from sensors to gateway\controller (they will be created as child devices)
MyMessage msg_temp(TEMP_sensor, V_TEMP);
MyMessage msg_hum(HUM_sensor, V_HUM);
MyMessage msg_vis(VIS_sensor, V_LEVEL);
MyMessage msgBatt(BATT_sensor, V_VOLTAGE);
MyMessage msg_uvi(UVIndex_sensor, V_UV);
MyMessage msg_visible(Visible_sensor, V_LIGHT_LEVEL);
MyMessage msg_ir(IR_sensor, V_LIGHT_LEVEL);
MyMessage msg_p(P_sensor, V_PRESSURE);
MyMessage msg_m(M_sensor, V_STATUS);
MyMessage msg_g(G_sensor, V_LIGHT);


//Si1132
float Si1132UVIndex = 0;
uint32_t Si1132Visible = 0;
uint32_t Si1132IR = 0;

float oldSi1132UVIndex = 0;
uint32_t oldSi1132Visible = 0;
uint32_t oldSi1132IR = 0;

int16_t oldLux = 0, lux;
int16_t oldhumidty = 0, humidty;
int16_t oldTemp = 0, temp;
uint8_t batteryPcnt, oldBatteryPcnt = 0xFF; 
uint8_t prevoiusGyroPinValue = 0, prevoiusMagnetPinValue = 0;

float oldPressureFl = 0;

volatile bool flagIntGyro = false, flagIntMagnet = false;


//#define G_VALUE 16380
//#define G_VALUE2 268304400 //G_VALUE * G_VALUE

// PCINT21 (PCMSK2 / PCIF2 / PCIE2)
void pinsIntEnable()
{
  PCMSK2 |= bit (PCINT20); // D4 accelerometer interrupt 
  PCMSK2 |= bit (PCINT21); // D5 magnetic sensor state\interrupt
  PCIFR  |= bit (PCIF2);   // clear any outstanding interrupts
  PCICR  |= bit (PCIE2);   // enable pin change interrupts for A0 to A1
}

ISR (PCINT2_vect)
{
  flagIntGyro = true;
  flagIntMagnet = true;
}



void battery_reportOld() {
  //---------------BATTERY REPORTING START
  static int oldBatteryPcnt = 0;
  int batteryPcnt;

  // Get the battery Voltage
  int sensorValue = analogRead(BATTERY_SENSE_PIN);
  /*  Devider values R1 = 3M, R2 = 470K divider across batteries
   *  Vsource = Vout * R2 / (R2+R1)   = 7,383 * Vout;
   *  we use internal refference voltage of 1.1 Volts. Means 1023 Analg Input values  = 1.1Volts
   *  5.5 is dead bateries. 6.3 or more - is 100% something in between is working range.
   */

  float voltage = sensorValue*0.001074*7.38255 ;
  if (voltage > 6.3)  batteryPcnt = 100;
  else if (voltage < 5.5) batteryPcnt = 0;
  else batteryPcnt = (int)((voltage - 5.5) / 0.008) ;

  Serial.print(F("voltage ")); Serial.println(voltage);

  if (oldBatteryPcnt != batteryPcnt ) {
    sendBatteryLevel(batteryPcnt);
    oldBatteryPcnt = batteryPcnt;
  }
  
  if (oldBatteryPcnt != batteryPcnt ) {
    // Power up radio after sleep
    sendBatteryLevel(batteryPcnt);
    send(msgBatt.set(voltage, 2));
    oldBatteryPcnt = batteryPcnt;
  }
  //------------------BATTERY REPORTING END
}


void batteryLevelRead(){
  
  // Get the battery Voltage
  
  int sensorValue = analogRead(BATTERY_SENSE_PIN);
  /*  Devider values R1 = 3M, R2 = 470K divider across batteries
   *  Vsource = Vout * R2 / (R2+R1)   = 7,383 * Vout;
   *  we use internal refference voltage of 1.1 Volts. Means 1023 Analg Input values  = 1.1Volts
   *  5.7 V is close to dead bateries under the working load. 6.3 or more - is 100% something in between is working range.
   */
   
  // voltage report valid in room temperatures

    float voltage = sensorValue*0.007930934;   //3M \ 470k divider formula. analogReference should beanalogReference(INTERNAL);  
 
  
  Serial.print(F("batteryLevelRead() sensorValue ")); Serial.println(sensorValue);
  Serial.print(F("batteryLevelRead()) voltage ")); Serial.println(voltage);
 
  if (voltage > 6.3)  batteryPcnt = 100;
  else if (voltage < 5.7) batteryPcnt = 0;
  else batteryPcnt = (int)((voltage - 5.7) / 0.006) ;
}


void batteryReport(){
  if ( (abs(oldBatteryPcnt - batteryPcnt) > 10 ) || oldBatteryPcnt == 0xFF ) {
    sendBatteryLevel(batteryPcnt);
    // this wait(); is 2.0 and up RFM69 specific. Hope to get rid of it soon
    // TSF:MSG:SEND,238-238-0-0,s=255,c=3,t=0,pt=1,l=1,sg=0,ft=0,st=OK:100
   
    // wait for ACK signal up to RFM95_RETRY_TIMEOUT_MS or 50ms for rfm miliseconds
    #ifdef  MY_RADIO_RFM95
      wait(RFM95_RETRY_TIMEOUT_MS, 3, 0);
    #endif
    #ifdef  MY_RADIO_RFM69
     // waiting up to xxx millis ACK of type 2 message t=2
      wait(500, 3, 0);
    #endif
    oldBatteryPcnt = batteryPcnt;
  }
}



void pressure_report() {
    //----------------PREASURE METER START
    
  char pressure[10]; // 100088.55

  //bmp.wakeup();

  #ifdef MY_DEBUG

    Serial.print(F("BMP280 Temperature = "));
    Serial.print(bmp.readTemperature());
    Serial.println(" *C");
  
    Serial.print(F("Approx altitude = "));
    Serial.print(bmp.readAltitude(1013.25)); // this should be adjusted to your local forcase
    Serial.println(" m");

  #endif


  float pressureFl = bmp.readPressure();
  pressureFl = pressureFl / 100; // covert to hPa 
  dtostrf(pressureFl,0,2,pressure);
  
  if (pressureFl != oldPressureFl && 0 != P_sensor) {
    send(msg_p.set(pressure), true); // Send bmp  sensor readings
    #ifdef  MY_RADIO_RFM95
     // 8052 TSF:MSG:READ,0-0-240,s=8,c=1,t=4,pt=0,l=9,sg=0:100095.33
     // waiting up to xxx millis ACK of type 4 message t=4
      wait(RFM95_RETRY_TIMEOUT_MS, 1, 4);
    #endif
    #ifdef  MY_RADIO_RFM69
     wait(500, 1, 4);
    #endif 
    oldPressureFl = pressureFl;
  }

  //bmp.sleep();
  //----------------PREASURE METER END
}


void lightReport()
{
  char visualLight[10];
 
  lightMeter.begin(BH1750::ONE_TIME_LOW_RES_MODE); // need for correct wake up
  lux = lightMeter.readLightLevel(true);// Get Lux value
  #ifdef MY_DEBUG
    Serial.print("visualLight LUX : ");  Serial.println(lux);
  #endif
  // dtostrf(); converts float into string
  dtostrf(lux,5,0,visualLight);
  
  // Sensor # is 0, no need to report anything
  if (oldLux != lux && 0 != VIS_sensor) { //  If sensor # is 0 the sensor will not report any values to the controller 
    send(msg_vis.set(visualLight), true);  // Send LIGHT BH1750     sensor readings
    #ifdef  MY_RADIO_RFM95
      // TSF:MSG:SEND,209-209-0-0,s=5,c=1,t=37,pt=0,l=5,sg=0,ft=0,st=OK: 
      // waiting up to xxx millis ACK of type 37 message t=37
      wait(RFM95_RETRY_TIMEOUT_MS, 1, 37);
    #endif
    #ifdef  MY_RADIO_RFM69
      wait(500, 1, 37);
    #endif 
    oldLux = lux;
  }
}

void TempHumReport()
{
  char humiditySi7021[10];
  char tempSi7021[10];

   
  // Measure Relative Humidity from the Si7021
  humidty = sensor.getRH();
  dtostrf(humidty,0,2,humiditySi7021);  
  
  if (humidty != oldhumidty && 0 != HUM_sensor) {
    send(msg_hum.set(humiditySi7021), true); // Send humiditySi7021     sensor readings
    #ifdef  MY_RADIO_RFM95
      // 1320 TSF:MSG:READ,0-0-116,s=3,c=1,t=1,pt=0,l=5,sg=0:34.00
      // waiting up to xxx millis ACK of type 1 message t=1
      wait(RFM95_RETRY_TIMEOUT_MS, 1, 1);
    #endif
    #ifdef  MY_RADIO_RFM69
     wait(500, 1, 1);
    #endif    
    oldhumidty = humidty; 
  }

  // Measure Temperature from the Si7021
  // Temperature is measured every time RH is requested.
  // It is faster, therefore, to read it from previous RH
  // measurement with getTemp() instead with readTemp()
  temp = sensor.getTemp();
  dtostrf(temp,0,2,tempSi7021);
  if (temp != oldTemp && 0 != TEMP_sensor) {
    send(msg_temp.set(tempSi7021), true); // Send tempSi7021 temp sensor readings
    #ifdef  MY_RADIO_RFM95
     // TSF:MSG:READ,0-0-116,s=4,c=1,t=0,pt=0,l=5,sg=0:24.00
     // waiting up to xxx millis ACK of type 0 message t=0
      wait(RFM95_RETRY_TIMEOUT_MS, 1, 0);
    #endif
    #ifdef  MY_RADIO_RFM69
      wait(500, 1, 0);
    #endif 
    oldTemp = temp;
 }

}


/*

void light_report() {
  //------------------LIGHT METER BEGIN
  char luxStr[10];

  //lightMeter.begin();
  //lightMeter.begin(BH1750_ONE_TIME_HIGH_RES_MODE); // need for correct wake up
  lightMeter.begin(BH1750::ONE_TIME_LOW_RES_MODE);

  //delay(120); //120ms to wake up BH1750 according the datasheet
  lux = lightMeter.readLightLevel();// Get Lux value
  // dtostrf(); converts float into string
  long luxDelta = (long)(lux - oldLux);
  Serial.print(F("LUX=")); Serial.println(lux);
  dtostrf(lux, 5, 0, luxStr);
  if ( abs(luxDelta) > 50 ) {
    send(msg_vis.set(luxStr), true); // Send LIGHT BH1750 sensor readings
    oldLux = lux;
  }
  wait(5);
  //-----------------LIGHT METER END
}

void temphum_report() {
  //-----------------TEMP&HUM METER BEGIN
  char humiditySi7021[10];
  char tempSi7021[10];

  si7021_env data = SI7021Sensor.getHumidityAndTemperature();

  // Measure Relative Humidity from the Si7021
  humidty = data.humidityBasisPoints;
  dtostrf(humidty, 4, 1, humiditySi7021);
  Serial.print(F("HUM:")); Serial.println(humiditySi7021);
  if (humidty != oldhumidty) {
    send(msg_hum.set(humiditySi7021), true); // Send humiditySi7021 sensor readings
    oldhumidty = humidty;
  }

  wait(5);

  // Measure Temperature from the Si7021
  // Temperature is measured every time RH is requested.
  // It is faster, therefore, to read it from previous RH
  // measurement with getTemp() instead with readTemp()
  temp = data.celsiusHundredths / 100.0;
  dtostrf(temp, 4, 1, tempSi7021);
  Serial.print(F("T:")); Serial.println(tempSi7021);
  if (temp != oldTemp) {
    send(msg_temp.set(tempSi7021), true); // Send tempSi7021 temp sensor readings
    oldTemp = temp;
  }
  wait(5);
  //----------------TEMP&HUM METER END
}
*/

/*
  void lightextra_report()
  {
  char UVI_Si1132[10];
  char VIS_Si1132[10];
  char IR_Si1132[10];
  //Serial.print(F("UV")); Serial.println(si1132.readUV());
  //Serial.print(F("VIS"));Serial.println(si1132.readVisible());
  //Serial.print(F("IR"));Serial.println(Si1132IR);
  Si1132UVIndex = 0;
  Si1132Visible = 0;
  Si1132IR = 0;
  for (int i = 0; i < 10; i++) {
    Si1132Visible += si1132.readVisible();
    //if (Si1132Visible > 4000000) Si1132Visible = 0;
    Si1132IR += si1132.readIR();
    Si1132UVIndex += si1132.readUV();
  }
  Si1132UVIndex /= 10;
  Si1132UVIndex /= 100;
  Si1132Visible /= 10;
  Si1132IR /= 10;
  Serial.print(F("UV=")); Serial.println(Si1132UVIndex);
  Serial.print(F("VIS="));Serial.println(Si1132Visible);
  Serial.print(F("IR="));Serial.println(si1132.readIR());
  long uvIndexDelta = (long)(Si1132UVIndex - oldSi1132UVIndex);
  //Serial.print(F("abs(UVIndex - old UVindex)=")); Serial.print(abs(z)); Serial.print(F("; UVIndex=")); Serial.print(Si1132UVIndex); Serial.print(F("; old UVIndex=")); Serial.println(oldSi1132UVIndex);
  dtostrf(Si1132UVIndex, 4, 2, UVI_Si1132);
  if ( abs(uvIndexDelta) > 0.1 ) {
    send (msg_uvi.set(UVI_Si1132), true);
    oldSi1132UVIndex = Si1132UVIndex;
  }
  wait(5);
  long visDelta = (long)(Si1132Visible - oldSi1132Visible);
  //Serial.print(F("abs(VisibleSi1132 - old)=")); Serial.print(abs(y)); Serial.print(F("; VisibleSi1132=")); Serial.print(Si1132Visible); Serial.print(F("; old VisibleSi1132=")); Serial.println(oldSi1132Visible);
  dtostrf(Si1132Visible, 5, 0, VIS_Si1132);
  if ( abs(visDelta) > 50) {
    send(msg_visible.set(VIS_Si1132), true);
    oldSi1132Visible = Si1132Visible;
  }
  wait(5);
  long irDelta = (long)(Si1132IR - oldSi1132IR);
  //Serial.print(F("abs(IRSi1132 - old)=")); Serial.print(abs(x)); Serial.print(F("; IRSi1132=")); Serial.print(Si1132IR); Serial.print(F("; old IRSi1132 =")); Serial.println(oldSi1132IR);
  dtostrf(Si1132IR, 5, 0, IR_Si1132);
  if (abs(irDelta) > 50) {
    send(msg_ir.set(IR_Si1132), true);
    oldSi1132IR = Si1132IR;
  }
  wait(5);
  //send (msg_ir.set(Si1132IR));
  }
*/



void blinkSensorLed(int  i){
  //return;
  for (;i>0;i--){
    digitalWrite(LED_PIN, HIGH);
    wait(50);
    digitalWrite(LED_PIN, LOW);
    if (i > 1) {wait(50);}
  }
}

void swarm_report()
{
  
  batteryLevelRead();
  lightReport();
  TempHumReport();
  pressure_report();  
  batteryReport();
 
  //lightextra_report();
}


void magnetSensorInterruptHandler(){
  flagIntMagnet = true;
}

void before() {
  
  /* This section  needs to be execuded if JDEC EPROM needs to be acceses in the loop()
  noInterrupts();
  _flash.initialize();
  _flash.sleep();
  interrupts();
  */
  
  analogReference(INTERNAL); // using internal ADC ref of 1.1V
  
  //reading unused pin - init ADC 
  analogRead(A7);

  pinMode(LED_PIN, OUTPUT);

  //No need watch dog enabled in case of battery power.
  //wdt_enable(WDTO_4S);
  wdt_disable();
  /*  RFM reset pin is 9
   *  A manual reset of the RFM69HCW\CW is possible even for applications in which VDD cannot be physically disconnected.
   *  Pin RESET should be pulled high for a hundred microseconds, and then released. The user should then wait for 5 ms
   *  before using the module.
   */
  pinMode(9, OUTPUT);
  //reset RFM module
  digitalWrite(9, 1);
  delay(1);
  digitalWrite(9, 0);
  // set Pin 9 to high impedance
  pinMode(9, INPUT);
  delay(10);  
  lightMeter.begin(BH1750::ONE_TIME_LOW_RES_MODE); // need for correct wake up
  lightMeter.readLightLevel();// Get Lux value
  
  //si1132.begin();


  // bmp280
  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));

  } else {
    //bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
    bmp.setSampling(Adafruit_BMP280::MODE_SLEEP,     /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
  }


  //attachInterrupt(digitalPinToInterrupt(MAGNET_PIN), magnetSensorInterruptHandler, CHANGE);
  
  // Enable PCINT2_vect interrupt - magnet and gyro sensors
   pinsIntEnable();


  if (! lis.begin(0x19) ) {   // change this to 0x19 for alternative i2c address
    Serial.println("Couldnt start");
  } else {
    Serial.println("LIS3DH found!");
    lis.setRange(LIS3DH_RANGE_2_G);
    lis.setDataRate(LIS3DH_DATARATE_10_HZ);
    lis.setClick(1, 20);
  }

  int sensorValue = analogRead(BATTERY_SENSE_PIN);
  float voltage = sensorValue*0.007930934;   //3M \ 470k divider formula. analogReference should beanalogReference(INTERNAL);  

  
  if (voltage > 3.5 && voltage < 5 )  {
   // The battery level is critical. To avoid rebooting and flooding radio with presentation messages
   // turn the device into hwSleep mode.
   // if voltage is less than 3.5. that means FTDI adapter used. no need to worry about.

   #ifdef MY_DEBUG
      Serial.print(F("Before() sensorValue ")); Serial.println(sensorValue);
      Serial.print(F("Before() voltage ")); Serial.println(voltage);
      Serial.println("Voltage is below 5 Volts. Halting boot"); 
   #endif

    blinkSensorLed(4);

    transportDisable();
    hwSleep(0); //BUTTONS_INTERUPT_PIN - 2, FALLING  , 
  }

   blinkSensorLed(1);

}

void setup() {

}

void presentation()
{
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("ButtonSize node", "3.0");

  // Register all sensors we need to gw (they will be created as child devices)
  present(TEMP_sensor, S_TEMP);
  present(HUM_sensor, S_HUM);
  present(VIS_sensor, S_LIGHT_LEVEL);
  present(P_sensor, S_BARO);
  present(M_sensor, S_BINARY);
  
  /*
  present(BATT_sensor, S_MULTIMETER);
  present(UVIndex_sensor, S_UV);
  present(Visible_sensor, S_LIGHT_LEVEL); //Si1132
  present(IR_sensor, S_LIGHT_LEVEL); //Si1132
  present(G_sensor, S_CUSTOM);
  */
}

void loop()
{
  //No need watch dog in case of battery power.
  //wdt_reset();

  uint8_t GyroPinValue = digitalRead(GYRO_PIN);
  uint8_t magnetPinValue = digitalRead(MAGNET_PIN);

  /*This section needs to be execuded if JDEC EPROM needs to be acceses in the loop()  
  noInterrupts();
  _flash.initialize();
  _flash.wakeup();
  interrupts();
  */

  if ( flagIntGyro && (GyroPinValue != prevoiusGyroPinValue))  {
      flagIntGyro = false;
      
        // Something meaningful can be implimented here for gyro sensor.
        // if shake the board this code will print.
        #ifdef MY_DEBUG
            lis.read();
            Serial.print("X:  "); Serial.print(lis.x);
            Serial.print("  \tY:  "); Serial.print(lis.y);
            Serial.print("  \tZ:  "); Serial.println(lis.z);
        #endif
      blinkSensorLed(1);
  } else if (flagIntGyro) {flagIntGyro = false;}

  if ( flagIntMagnet && magnetPinValue != prevoiusMagnetPinValue )  {
    
      flagIntMagnet = false;
      prevoiusMagnetPinValue = magnetPinValue;
      msg_m.setDestination(relayNodeIDmagSensor); 
      uint8_t sendStatus =  send(msg_m.set(magnetPinValue),true);
      // wait for ACK signal up to RFM95_RETRY_TIMEOUT_MS or 50ms for rfm miliseconds
      #ifdef  MY_RADIO_RFM95
        wait(RFM95_RETRY_TIMEOUT_MS, 1, 2);
      #endif
      #ifdef  MY_RADIO_RFM69
       // TSF:MSG:READ,0-0-209,s=1,c=1,t=2,pt=2,l=2,sg=0:1
       // waiting up to xxx millis ACK of type 2 message t=2
      wait(500, 1, 2);
      #endif
      // Blink  respective LED's once if message delivered to controller. 3 times if failed
      if (sendStatus) {
         blinkSensorLed(1);
      } else {
         blinkSensorLed(3); 
      }
  } else if (flagIntMagnet) {flagIntMagnet = false;}

  swarm_report();

  /*This section  needs to be execuded if JDEC EPROM needs to be acceses in the loop()  
  noInterrupts();
  _flash.sleep();
  interrupts();
  */
  
  // Go sleep for some milliseconds

  if ( !flagIntMagnet && !flagIntGyro ) { // make sure no changes in Gyro and magnet sensor happened while we were sending reports. 
      
      //sleep(0); // currently only 0 value can be set. if any other number  set then the Gyro and the Magnet sensor interrupts will be ignired. 

      // hwSleep2 is rework of MySensors native sleep function to be able to sleep for some time and  wakeup on interrups CHANGE 
      // Doesnot support smart sleep, FOTA nor TransportReady. just sleeps miliseconds provided with first parameter. 
      hwSleep2(60000, &flagIntGyro, &flagIntMagnet); 
  }
}





uint32_t hwInternalSleep2(uint32_t ms, volatile bool *flag1, volatile bool *flag2  )
{
  // Sleeping with watchdog only supports multiples of 16ms.
  // Round up to next multiple of 16ms, to assure we sleep at least the
  // requested amount of time. Sleep of 0ms will not sleep at all!
  ms += 15u;


  while (!*flag1 && !*flag2 && ms >= 16) {
    for (uint8_t period = 9u; ; --period) {
      const uint16_t comparatorMS = 1 << (period + 4);
      if ( ms >= comparatorMS) {
        hwPowerDown(period); // 8192ms => 9, 16ms => 0
        ms -= comparatorMS;
        break;
      }
    }
  }

  
  if ( *flag1 or *flag2 ) {
    return ms;
  }
  return 0ul;
}

int8_t hwSleep2(uint32_t ms, volatile bool* flag1, volatile bool* flag2)
{
  transportDisable();
  // Return what woke the mcu.
  // Default: no interrupt triggered, timer wake up
  int8_t ret = MY_WAKE_UP_BY_TIMER;
  sleepRemainingMs = 0ul;
  if (ms > 0u) {
    // sleep for defined time
    sleepRemainingMs = hwInternalSleep2(ms, flag1, flag2);
  } else {
    // sleep until ext interrupt triggered
    hwPowerDown(WDTO_SLEEP_FOREVER);
  }
  transportReInitialise();
  return 0;
}
