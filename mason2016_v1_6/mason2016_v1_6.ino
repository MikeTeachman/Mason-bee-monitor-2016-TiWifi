/*  Mason Bee Capacitive Sensor

v1.0
- started with mason_capsensor_v1_5 (final version used in 2015 unit)
- code optimization to reduce codespace and ram
v1.1
- implement fat16 library for SD card logging. saves 6kB code, 900 bytes ram
v1.2
- refactor cap sensing routines
- 5 second period for SD card logging
- read DAC setting from SD card file
- add TSL2591 light sensor (previously used TSL2561 sensor)
- add BMP280 pressure sensor (previously used BMP180 sensor)
v1.3
- added digital inputs 6, 7
v1.4
- code refactoring
v1.5
- added I2C data push to wifi unit
v1.6
- added maintenance mode.  when input 7 is pulled low data the system is considered in maintenance.  Data is flagged as suspect
- read bee detect threshold from sdCard and push to wifi unit

*/
#define ENV_SENSORS_PRESENT 1  // set to 1 when environmental I2C sensors are present
#define CAP_SENSORS_PRESENT 1  // set to 1 when capacitive I2C sensors are present
#define LOGGING_SHIELD_PRESENT 1  // set to 1 when the logging shield is present

#include <stdio.h>
#include <SPI.h>
#include <Fat16.h>
#include <Wire.h>
#include "RTClib.h"
#include "AD7746.h"
#include "Adafruit_Sensor.h"
#include "Adafruit_MCP9808.h"   // temperature sensor
#include "Adafruit_HTU21DF.h"   // humidity sensor
#include <Adafruit_BMP280.h>    // barometer pressure sensor
#include "Adafruit_TSL2591.h"   // light sensor
#include <EEPROM.h>
#include <avr/wdt.h>

#define NUMBER_OF_AD7746_DEVICES 4
#define NUMBER_OF_NESTING_TUBES (NUMBER_OF_AD7746_DEVICES * 2)
#define LOGGING_PERIOD_IN_SECS 5
#define NUM_INTERVALS_TO_RESET 3
#define DELAY_IN_MS 300

#define DI_WIFI_UNIT_INSTALLED 6  // pin 6: low = wifi unit installed, try to send data packet via I2C
#define DI_MAINTENANCE_MODE 7  // pin 7: low = bee block in maintenance mode, high = normal operation

typedef struct
{
  float tempInDegC;
  float humidityInRh;
  float pressureInHpa;
  float lightInLux;
   
} envSensorReadings_t;

typedef struct 
{
  uint8_t cdcCapChannelSetup;    // Capacitive channel setup register
  uint8_t cdcExcChannelSetup;    // Capacitive channel excitation setup register
  uint8_t muxSelect;             // MUX select  
  
} cdcChannelConfig_t;

typedef struct 
{
  uint8_t startPattern;
  uint32_t currentTime;
  uint8_t endPattern;
} timePacket_t;

typedef struct 
{
  uint8_t resetCountSensorUnit;
  uint8_t maintMode;  // 1 = unit in maintenance mode, 0 = normal operation
  uint32_t currentUtcTime;
  uint32_t timeOfLastUpdate;
  uint32_t beeDetectThreshold;  // used by wifi unit to detect bee activity
} diagnosticsPacket_t;

const cdcChannelConfig_t cdcConfig[] = 
{
  {0x80, 0x0B, 0x00},  // nesting tube #1
  {0xC0, 0x23, 0x00},  // nesting tube #2
  {0x80, 0x0B, 0x01},  // nesting tube #3
  {0xC0, 0x23, 0x01},  // nesting tube #4
  {0x80, 0x0B, 0x02},  // nesting tube #5
  {0xC0, 0x23, 0x02},  // nesting tube #6
  {0x80, 0x0B, 0x03},  // nesting tube #7
  {0xC0, 0x23, 0x03},  // nesting tube #8
};

int32_t capSensorReadings[NUMBER_OF_NESTING_TUBES];
uint8_t cdcDacChannelSetup[NUMBER_OF_NESTING_TUBES];  // Capacitive DAC A setup register
envSensorReadings_t envSensorReadings;

const int chipSelect = 10;  // for the data logging shield, we use digital pin 10 for the SD Chip Select (CS) line
SdCard card;
Fat16 logfile;
Fat16 dacConfigFile;
Fat16 beeDetectConfigFile;
RTC_DS1307 rtc; // define the Real Time Clock object
uint32_t unixtime;
uint8_t wdtCount = NUM_INTERVALS_TO_RESET; // number of intervals before unit will force a reset (total time is intervals x 8 seconds)
const uint16_t EEPROMaddrForResetCount=0;
uint32_t lastPolledTime = 0L; // Last value retrieved from time server
uint32_t beeDetectThreshold = 10000;  // used by wifi unit to detect bee activity

#if ENV_SENSORS_PRESENT == 0
uint16_t simulatedSensorData = 0;
#endif

#if LOGGING_SHIELD_PRESENT == 0 // no real-time clock
uint32_t sketchTime = 0L; // CPU milliseconds since last server query 
uint32_t lastPolledTime = 0L;  
#endif

#if ENV_SENSORS_PRESENT == 1
Adafruit_MCP9808 tempsensor = Adafruit_MCP9808();
Adafruit_HTU21DF htu = Adafruit_HTU21DF();
Adafruit_BMP280 bmp;
Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591); 
#endif

void setup()
{
  // Watchdog timer setup.  
  noInterrupts();
  MCUSR  &= ~_BV(WDRF);
  WDTCSR  =  _BV(WDCE) | _BV(WDE);              // WDT change enable
  WDTCSR  =  _BV(WDIE) | _BV(WDP3) | _BV(WDP0); // Interrupt enable, 8 sec.
  interrupts();  
  
  Wire.begin(); // sets up i2c for operation
  Serial.begin(9600); // set up baud rate for serial
  
  #if LOGGING_SHIELD_PRESENT == 1
  initSdCard();
  readDacConfigFromSdCard();
  readbeeDetectThresholdFromSdCard();
  #endif

  #if ENV_SENSORS_PRESENT == 1
  tempsensor.begin();
  htu.begin();
  bmp.begin();
  tsl.begin();
  // Configure the gain and integration time for the TSL2591 light sensor
  configureSensor();
  #endif
  
  // pins used to control muxing of I2C SDA
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);  

  pinMode(DI_WIFI_UNIT_INSTALLED, INPUT_PULLUP);  
  pinMode(DI_MAINTENANCE_MODE, INPUT_PULLUP);    
  
  #if CAP_SENSORS_PRESENT == 1
  Serial.println(F("Initializing CDCs"));

  for (uint8_t i=0; i<NUMBER_OF_AD7746_DEVICES; i++)
  {
     cdcInit(i);
  }
  #endif 
    
  #if CAP_SENSORS_PRESENT == 0
  Serial.println(F("running in Capacitive sensor simulation mode"));
  #endif 
  
  #if ENV_SENSORS_PRESENT == 0
  Serial.println(F("running in Environmental sensor simulation mode"));
  #endif 

  #if LOGGING_SHIELD_PRESENT == 0
  Serial.println(F("logging to SDCARD disabled"));
  #endif 
 
  Serial.println(F("done setup()"));

  wdtCount = NUM_INTERVALS_TO_RESET;  // restore watchdog count
}

void loop()
{
  #if LOGGING_SHIELD_PRESENT == 1
  DateTime timeNow;
  timeNow = rtc.now();
  
  unixtime = timeNow.unixtime();
  #else
  unixtime = lastPolledTime + (millis() - sketchTime) / 1000;
  #endif
  
  if ((unixtime % LOGGING_PERIOD_IN_SECS) == 0)
  {
    uint32_t startLoop = millis();
    Serial.print(F("unixtime = "));
    Serial.println(unixtime);    
    
    // -------------------------------------------- 
    //            Capacitive Sensors 
    // --------------------------------------------
    
    #if CAP_SENSORS_PRESENT == 1

    for (uint8_t nestingTubeNum = 0; nestingTubeNum < NUMBER_OF_NESTING_TUBES; nestingTubeNum++)
    {
      cdcEnable(nestingTubeNum);   
      delay(DELAY_IN_MS);
      capSensorReadings[nestingTubeNum] = cdcRead(nestingTubeNum);
      cdcDisable(nestingTubeNum); 
    } 

    #else
    delay(2000);   
    #endif 
 
    // -------------------------------------------- 
    //            Environmental Sensors 
    // --------------------------------------------
    
    #if ENV_SENSORS_PRESENT == 1
    // --- HTU21DF Humidity/Temperature Sensor ---
    envSensorReadings.humidityInRh = htu.readHumidity();
     
    // --- MCP9808 Temperature Sensor ---
    envSensorReadings.tempInDegC = tempsensor.readTempC();  

    // --- BMP280 Pressure/Temperature Sensor ---
    envSensorReadings.pressureInHpa = bmp.readPressure()/100.0; // hPa

    // --- TSL2591 Light Sensor ---
    sensors_event_t lightEvent;
    tsl.getEvent(&lightEvent);
    envSensorReadings.lightInLux = lightEvent.light; // lux
    #else
    // substitute fake values when the sensors are not present
    envSensorReadings.tempInDegC = simulatedSensorData++;  
    envSensorReadings.humidityInRh = simulatedSensorData++;
    envSensorReadings.pressureInHpa = simulatedSensorData++; // hPa
    envSensorReadings.lightInLux = simulatedSensorData++; // lux
    
    Serial.println(F("delay for simulated env sensor read"));
    delay(100);  // simulates delay in reading all the envsensors
    #endif

    // -------------------------------------------- 
    //            Maintenance Mode detect 
    // --------------------------------------------

    Serial.print(F("Maintenance Mode = "));
    Serial.println(isMaintMode());

    printSensorData();

    #if LOGGING_SHIELD_PRESENT == 1
    logBeeData(timeNow);
    #endif

    Serial.print(F("------ loop duration[ms] = "));   
    Serial.println(millis()-startLoop);   

    pushDataToExpansionUnit();    
  }
  else
  {
    delay(1000); // handles the loop run that follows a sensor reads -allow RTC advance to the next second 
  }
  
  wdtCount = NUM_INTERVALS_TO_RESET;  // restore watchdog count
}

void cdcInit(uint8_t cdcIndex)
{
  selectCdcByChip(cdcIndex);
  Serial.print(F("\tInit CDC: "));
  Serial.println(cdcIndex);
  Wire.beginTransmission(AD7746_I2C_ADDRESS); // start i2c cycle
  Wire.write(AD7746_RESET_ADDRESS); // reset the AD7746 device
  Wire.endTransmission(); // ends i2c cycle
  
  delay(10);  // wait a bit for AD7746 to reboot
  
  AD7746_writeRegister(AD7746_REGISTER_CONFIGURATION, 0x39);  // 109.6 ms conversion time 
  AD7746_writeRegister(AD7746_REGISTER_CAP_OFFSET, 0x00);  
}

void cdcEnable(uint8_t tubeIndex)
{
  selectCdcByTube(tubeIndex);

  AD7746_writeRegister(AD7746_REGISTER_EXC_SETUP, cdcConfig[tubeIndex].cdcExcChannelSetup);  
  AD7746_writeRegister(AD7746_REGISTER_CAP_SETUP, cdcConfig[tubeIndex].cdcCapChannelSetup);
  AD7746_writeRegister(AD7746_REGISTER_CAP_DAC_A, _BV(7) | cdcDacChannelSetup[tubeIndex]);    
}

void cdcDisable(uint8_t tubeIndex)
{
  selectCdcByTube(tubeIndex);

  AD7746_writeRegister(AD7746_REGISTER_EXC_SETUP, 0x03);  // turn off excitation for both channels
  AD7746_writeRegister(AD7746_REGISTER_CAP_SETUP, 0x00); 
}

uint32_t cdcRead(uint8_t tubeIndex)
{
  selectCdcByTube(tubeIndex);
  return AD7746_readValue();
}

void selectCdcByTube(uint8_t tubeIndex)
{
  digitalWrite(2, cdcConfig[tubeIndex].muxSelect&0x01);
  digitalWrite(3, cdcConfig[tubeIndex].muxSelect&0x02);
  digitalWrite(4, 0);
  digitalWrite(5, 0);  
  delay(10);
}

void selectCdcByChip(uint8_t chipIndex)
{
  uint8_t tubeIndex = chipIndex*2;
  selectCdcByTube(tubeIndex);
}

uint8_t isMaintMode(void)
{
  if (digitalRead(DI_MAINTENANCE_MODE) == 1)
  {
    return 0;
  }
  else
  {
    return 1;
  }
}

#if LOGGING_SHIELD_PRESENT == 1
void initSdCard(void)
{
  // initialize the SD card
  Serial.print(F("Initializing SD card..."));
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(10, OUTPUT);

  // see if the card is present and can be initialized:
  if (!card.begin(chipSelect))
  {
    error("Card failed, or not present");
  }
  Serial.println(F("card initialized."));

  // initialize a FAT16 volume
  if (!Fat16::init(&card)) error("Fat16::init");

  // create a new file
  char name[] = "LOGGER00.CSV";
  for (uint8_t i = 0; i < 100; i++) {
    name[6] = i/10 + '0';
    name[7] = i%10 + '0';
    // O_CREAT - create the file if it does not exist
    // O_EXCL - fail if the file exists
    // O_WRITE - open for write only
    if (logfile.open(name, O_CREAT | O_EXCL | O_WRITE))break;
  }
  if (!logfile.isOpen()) error ("create");

  Serial.print(F("Logging to: "));
  Serial.println(name);

  logfile.println(F("datetime[unixtime],date,time,cap_ch1,cap_ch2,cap_ch3,cap_ch4,cap_ch5,cap_ch6,cap_ch7,cap_ch8,temp[degC],humidity[%RH],pressure[hPa],light[lux],maintMode"));

  // connect to RTC
  Wire.begin();
  if (!rtc.begin())
  {
    logfile.println(F("RTC failed"));
    Serial.println(F("RTC failed"));
  }
}

void readDacConfigFromSdCard(void)
{
  if (dacConfigFile.open("DAC.TXT", O_READ)) 
  {
    Serial.println(F("Opened DAC.TXT"));
  } 
  else
  {
    Serial.println(F("file.open failed"));
  }

  // read the DAC channel settings from file
  char buf[6];  // 4 digits for HEX number, plus CR/LF
  for (uint8_t tubeIndex=0; tubeIndex<NUMBER_OF_NESTING_TUBES; tubeIndex++)
  {
    dacConfigFile.read(buf, sizeof(buf));
    sscanf (buf,"%x",&cdcDacChannelSetup[tubeIndex]);
    Serial.println(cdcDacChannelSetup[tubeIndex], HEX);
  }
  dacConfigFile.close();
}

void readbeeDetectThresholdFromSdCard(void)
{
  if (beeDetectConfigFile.open("BEE.TXT", O_READ)) 
  {
    Serial.println(F("Opened BEE.TXT"));
  } 
  else
  {
    Serial.println(F("file.open failed for BEE.TXT"));
  }

  // threshold from the file
  char buf[12];  // up to 10 digits for DEC number, plus CR/LF
 
  beeDetectConfigFile.read(buf, sizeof(buf));
  sscanf (buf,"%d",&beeDetectThreshold);
  Serial.print("beeDetectThreshold = ");
  Serial.println(beeDetectThreshold);
  
  beeDetectConfigFile.close();
}

void error(char *str)
{
  Serial.print(F("error: "));
  Serial.println(str);

  while (1);
}


void logBeeData(DateTime timeNow)
{
  logfile.print(timeNow.unixtime());
  logfile.print(F(", "));
  logfile.print(timeNow.year(), DEC);
  logfile.print(F("/"));
  logfile.print(timeNow.month(), DEC);
  logfile.print(F("/"));
  logfile.print(timeNow.day(), DEC);
  logfile.print(F(", "));
  logfile.print(timeNow.hour(), DEC);
  logfile.print(F(":"));
  logfile.print(timeNow.minute(), DEC);
  logfile.print(F(":"));
  logfile.print(timeNow.second(), DEC);
  logfile.print(F(", "));

  for (uint8_t nestingTubeNum = 0; nestingTubeNum < NUMBER_OF_NESTING_TUBES; nestingTubeNum++)
  {
    logfile.print(capSensorReadings[nestingTubeNum]);
    logfile.print(F(", ")); 
  } 

  logfile.print(envSensorReadings.tempInDegC);  
  logfile.print(F(", "));  
  logfile.print(envSensorReadings.humidityInRh);  
  logfile.print(F(", "));
  logfile.print(envSensorReadings.pressureInHpa);   
  logfile.print(F(", "));
  logfile.print(envSensorReadings.lightInLux);  

  logfile.print(F(", "));
  logfile.println(isMaintMode());  
    
  logfile.sync();
}
#endif

void printSensorData(void)
{
  Serial.println(F("--------------"));

  for (uint8_t nestingTubeNum = 0; nestingTubeNum < NUMBER_OF_NESTING_TUBES; nestingTubeNum++)
  {
    Serial.print(F("0x"));
    Serial.println(capSensorReadings[nestingTubeNum], HEX);
  } 
  
  Serial.print(F("tempInDegC: "));
  Serial.println(envSensorReadings.tempInDegC);    
  Serial.print(F("humidity %RH: "));
  Serial.println(envSensorReadings.humidityInRh);      
  Serial.print(F("pressure hpa: "));
  Serial.println(envSensorReadings.pressureInHpa);        
  Serial.print(F("light lux: "));
  Serial.println(envSensorReadings.lightInLux);      

  Serial.print(F("maint mode: "));
  Serial.println(isMaintMode());    
}

void pushDataToExpansionUnit(void)
{
  if (digitalRead(DI_WIFI_UNIT_INSTALLED) == 0) // Wifi unit is installed if this input is pulled low
  {
    Serial.println(F("Expansion unit installed"));        

    //
    // transmit Capacitive sensors readings
    //

    Wire.beginTransmission(4); // transmit to device #4
    // Serial.println(sizeof(capSensorReadings));        
    Wire.write((byte *)capSensorReadings, sizeof(capSensorReadings));            
    Wire.endTransmission();    // stop transmitting
    
    //
    // transmit Environmental sensors readings
    //
    
    Wire.beginTransmission(4); // transmit to device #4
    // Serial.println(sizeof(envSensorReadings_t));        
    Wire.write((byte *)&envSensorReadings, sizeof(envSensorReadings_t));            
    Wire.endTransmission();    // stop transmitting   
    
    //
    // transmit Diagnostic data
    //
        
    diagnosticsPacket_t diagnosticsPacket;
    diagnosticsPacket.resetCountSensorUnit = EEPROM.read(EEPROMaddrForResetCount);
    diagnosticsPacket.maintMode = isMaintMode();    
    diagnosticsPacket.currentUtcTime = unixtime;
    diagnosticsPacket.timeOfLastUpdate = lastPolledTime;
    diagnosticsPacket.beeDetectThreshold = beeDetectThreshold;
    
    Wire.beginTransmission(4); // transmit to device #4
    Wire.write((byte *)&diagnosticsPacket, sizeof(diagnosticsPacket_t));            
    Wire.endTransmission();    // stop transmitting   
    
    delay(100);

    //
    // request time update from slave
    //
        
    Serial.println(F("requesting time update"));        

    // request time update
    Wire.requestFrom(4, sizeof(timePacket_t)); // request a time packet from slave 
    int bytes = Wire.available();
    Serial.print(F("Num bytes rx'd from slave unit: "));
    Serial.println(bytes);
    
    timePacket_t timePacket;
    
    if (bytes > 0)
    {
      for(int i = 0; i< bytes; i++)
      {
        byte x = Wire.read();
        Serial.print(F("Slave Sent: "));
        Serial.println(x, HEX);
        ((byte *)(&timePacket))[i] = x;
      }  
      
      // filter to detect a valid time packet response
      // note:  this is a patch to overcome problems where the I2C slave would send back garbage data... should figure the root cause at some point
      if ((timePacket.startPattern == 0xA5) && (timePacket.endPattern == 0x5A) && ((DateTime)(timePacket.currentTime)).year() == 2016)
      {
        Serial.print(F("Valid Time Packet for 2016 -- UTC: "));
        Serial.println(timePacket.currentTime);
        lastPolledTime = timePacket.currentTime;         // Save time
        
        #if LOGGING_SHIELD_PRESENT == 1 // real-time clock is present
        rtc.adjust(timePacket.currentTime);  // update the onboard realtime clock
        #else
        unixtime = lastPolledTime;
        sketchTime     = millis();  // Save sketch time of last valid time query
        #endif
      }
    }
  } 
}

ISR(WDT_vect) 
{ 
  // Watchdog interrupt @ 8 sec. interval
  // Decrement watchdog counter...
  // If it reaches zero force a hw reset
  if(!--wdtCount) 
  { 
    uint8_t resetCount = EEPROM.read(EEPROMaddrForResetCount);
    resetCount++;
    EEPROM.write(EEPROMaddrForResetCount, resetCount);
    wdt_enable(WDTO_15MS); // turn on the WatchDog and allow it to fire
    while(1);
  }
}


