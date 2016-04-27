/*  Mason Bee I2C Slave

v1.0
- started with mason_wifi_v2_3 (final version used in 2015 unit)
v1.1
- added maintenance mode.  when input 7 is pulled low data the system is considered in maintenance.  Data is flagged as suspect
v1.2
- bee detect threshold received from main sensor unit
- don't count bees during maintenance mode

EEPROM map
==========
000: reset count - used for diagnostics

Digital Pins used by CC3000:
===========================
Digital pin 3: IRQ for WiFi
Digital pin 4: Card Select for SD card
Digital pin 5: WiFi enable
Digital pin 10: Chip Select for WiFi
Digital pins 11, 12, 13 for SPI communication

I/O Pin mapping
===============
D6: input:  pushbutton - initiate NTP request
D7: input:  pushbutton - general purpose
D8: output: led1
D9: output: led2

note:  max 40 mA current for Arduino digital output
*/

#include <Wire.h>
#include <Adafruit_CC3000.h>
#include <ccspi.h>
#include <SPI.h>
#include <RTClib.h>  // allows use of DateTime class
// #include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
#include "Bounce2.h"
#include <avr/wdt.h>

#define ADAFRUIT_CC3000_IRQ   3  // These are the interrupt and control pins ... MUST be an interrupt pin!
#define ADAFRUIT_CC3000_VBAT  5  // can be any pin
#define ADAFRUIT_CC3000_CS    10 // can be any pin
#define NUM_INTERVALS_TO_RESET 15
//#define RESET_PIN 2
#define NTP_REQUEST_PIN 6
#define DIAGNOSTICS_PUSHBUTTON_PIN 7
#define LED_RIGHT_PIN 8
#define LED_LEFT_PIN 9
#define WLAN_SSID       "*********"        // cannot be longer than 32 characters!
#define WLAN_PASS       "*********"
#define WLAN_SECURITY   WLAN_SEC_WPA2  // Security can be WLAN_SEC_UNSEC, WLAN_SEC_WEP, WLAN_SEC_WPA or WLAN_SEC_WPA2
#define IDLE_TIMEOUT_MS  3000      // Amount of time to wait (in milliseconds) with no data 
                                   // received before closing the connection.  If you know the server
                                   // you're accessing is quick to respond, you can reduce this value.
#define NUMBER_OF_CAP_SENSORS (8)

typedef struct
{
  float tempInDegC;
  float humidityInRh;
  float pressureInHpa;
  float lightInLux;
   
} envSensorReadings_t;

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

typedef enum
{
  SHORT_TIME_FORMAT,
  LONG_TIME_FORMAT,
  HR24_TIME_FORMAT,
} TIME_FORMAT_t;

void sendSensorDataToSparkFun(uint32_t *capSensorReadings, envSensorReadings_t *envSensorReadings, DateTime logTimestamp);
void sendBeeCountDataToSparkFun(uint16_t *beeCountThisHour, DateTime logTimestamp);

void(* resetFunc) (void) = 0; //declare reset function @ address 0

// Use hardware SPI for the remaining pins
// On an UNO, SCK = 13, MISO = 12, and MOSI = 11
Adafruit_CC3000 cc3000 = Adafruit_CC3000(ADAFRUIT_CC3000_CS, ADAFRUIT_CC3000_IRQ, ADAFRUIT_CC3000_VBAT,
                                         SPI_CLOCK_DIVIDER); // you can change this clock speed
                                         
const unsigned long connectTimeout = 15L * 1000L; // Max time to wait for server connection
const unsigned long responseTimeout = 15L * 1000L; // Max time to wait for data from server
unsigned long lastPolledTime = 0L; // Last value retrieved from time server
unsigned long sketchTime = 0L; // CPU milliseconds since last server query  
unsigned long currentTime = 0;
boolean ntpTimeAvailable = false;
uint32_t timestampSentToSensorUnit;
boolean checkTimestampRxBySensorUnit = false;
const int TZ_OFFSET = (8*3600);  //PST UTC-8
uint16_t I2cReceiveCount = 0;
uint16_t I2cRequestCount = 0;
uint16_t loopCount = 0;
uint8_t wdtCount = NUM_INTERVALS_TO_RESET; // number of intervals before unit will force a reset (total time is intervals x 8 seconds)
const uint16_t EEPROMaddrForResetCount=0;
boolean flashI2cLed = false;
uint32_t capSensorReadings[NUMBER_OF_CAP_SENSORS];
uint32_t oldCapReadings[NUMBER_OF_CAP_SENSORS];
envSensorReadings_t envSensorReadings;
diagnosticsPacket_t diagnosticsPacket;
boolean firstTimeStartup = true;
uint16_t beeCountThisHour[NUMBER_OF_CAP_SENSORS];

// set the LCD address to 0x27 for a 20 chars 4 line display
// Set the pins on the I2C chip used for LCD connections:
//                    addr, en,rw,rs,d4,d5,d6,d7,bl,blpol
// LiquidCrystal_I2C lcd(0x20, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address
// LiquidCrystal_I2C lcd(0x20, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address

Bounce getNtpButton = Bounce(); 
Bounce diagnosticsButton = Bounce(); 

// Sparkfun Data connection information
char sparkfunServer[] = "data.sparkfun.com";    // name address for data.sparkFun (using DNS)
char sensorPublicKey[] = "******************";  
char sensorPrivateKey[] = "****************";

//char simplePublicKey[] = "******************";  
//char simplePrivateKey[] = "******************";



char beeCountPublicKey[] = "*****************";
char beeCountPrivateKey[] = "******************";  

uint32_t ip;
Adafruit_CC3000_Client client;

void getTimeAsString(DateTime dateTime, char *timeBuf_p, TIME_FORMAT_t format);

void setup()
{
  // Watchdog timer setup.  
  noInterrupts();
  MCUSR  &= ~_BV(WDRF);
  WDTCSR  =  _BV(WDCE) | _BV(WDE);              // WDT change enable
  WDTCSR  =  _BV(WDIE) | _BV(WDP3) | _BV(WDP0); // Interrupt enable, 8 sec.
  interrupts();  
  
  Wire.begin(4);                // join i2c bus with address #4
  Wire.onReceive(receiveEvent); // register receive event
  Wire.onRequest(requestEvent); // register request event
  Serial.begin(9600);           // start serial for output

  Serial.println(F("Setup"));

  // lcd.begin(16,2);   
  // lcd.clear();

  // pushbutton to initiate NTP request
  pinMode(NTP_REQUEST_PIN, INPUT_PULLUP);  
  getNtpButton.attach(NTP_REQUEST_PIN);
  getNtpButton.interval(10); // interval in ms
  
  // unused pushbutton
  pinMode(DIAGNOSTICS_PUSHBUTTON_PIN, INPUT_PULLUP);  
  diagnosticsButton.attach(DIAGNOSTICS_PUSHBUTTON_PIN);
  diagnosticsButton.interval(10); // interval in ms
  
  pinMode(LED_LEFT_PIN, OUTPUT);     
  pinMode(LED_RIGHT_PIN, OUTPUT);     
  
  // indicate startup on LEDs
  flashLed(LED_LEFT_PIN, 1, 100);
  flashLed(LED_RIGHT_PIN, 1, 100);
  flashLed(LED_LEFT_PIN, 1, 100);
  flashLed(LED_RIGHT_PIN, 1, 100);
  flashLed(LED_LEFT_PIN, 1, 100);
  flashLed(LED_RIGHT_PIN, 1, 100);  
  
  delay(500);
  
  initBeeCounters();

  #if 0
  lcd.begin(16,2);   
  lcd.clear();
  lcd.noBacklight();
  lcd.setCursor(0,0); //Start at character 0 on line 0
  lcd.print("--- Startup ---");
  delay(1000);
  lcd.clear();
  #endif
  
  digitalWrite(LED_LEFT_PIN, HIGH);   
  cc3000_init();
  digitalWrite(LED_LEFT_PIN, LOW);  
  
  delay(500);
  
  while (!updateDuinoTime());  // loop until successful at getting time update
}

void loop()
{
  //Serial.println(F("Loop: "));
  // display loop counter on LCD
  #if 0
  lcd.begin(16,2);  
  lcd.clear();  
  lcd.setCursor(0,0); 
  lcd.print(loopCount); 

  // display I2C ReceiveEvent and RequestEvent counters on LCD
  lcd.setCursor(0,1); //Start at character 0 on line 1  
  lcd.print("                ");  // clear characters in this line
  lcd.setCursor(0,1); //Start at character 8 on line 1  
  lcd.print("Rx:");
  lcd.print(I2cReceiveCount);
  lcd.setCursor(8,1); //Start at character 10 on line 1   
  lcd.print("Rq:");
  lcd.print(I2cRequestCount);  
  #endif
  
  if (diagnosticsButton.update() == true)  
  {
    if (diagnosticsButton.read() == LOW)
    {  
      // does nothing right now
      
      #if 0
      lcd.begin(16,2);  
      lcd.clear();  
      lcd.setCursor(0,0); 
      lcd.print("sRC ");       
      lcd.print(diagnosticsPacket.resetCountSensorUnit);       
      lcd.print("  wRC ");       
      lcd.print(EEPROM.read(EEPROMaddrForResetCount));    

      lcd.setCursor(0,1); 
      char timeStr[10];
      getTimeAsString(diagnosticsPacket.currentUtcTime, timeStr, HR24_TIME_FORMAT);      
      lcd.print(timeStr);       
      lcd.print("  ");       
      getTimeAsString(diagnosticsPacket.timeOfLastUpdate, timeStr, HR24_TIME_FORMAT);      
      lcd.print(timeStr);             
      delay(2000);   
      #endif   
    }  
  }
  
  if (getNtpButton.update() == true)  
  {
    if (getNtpButton.read() == LOW)
    {  
      updateDuinoTime();
    }  
  }
  
  if (checkTimestampRxBySensorUnit == true)
  {
    if (timestampSentToSensorUnit == diagnosticsPacket.timeOfLastUpdate)
    {
      // sensor unit received timestamp.  flash led to indicate success
      flashLed(LED_RIGHT_PIN, 6, 100);
      checkTimestampRxBySensorUnit = false;  // stop checking for receipt of timestamp
    }
  }
  
  currentTime = lastPolledTime + (millis() - sketchTime) / 1000;

  // time to push sensor data to Sparkfun via wifi?
  if ((currentTime % 300) == 0)
  {
    Serial.println(F("sensor cloud push"));
    Serial.print(F("Current UNIX time: "));
    Serial.println(currentTime);      
    sendSensorDataToSparkFun(capSensorReadings, &envSensorReadings, currentTime);
  }
  
  // time to push bee count data to Sparkfun via wifi?
  if ((currentTime % 3600) == 0)
  {
    Serial.println(F("bee count cloud push"));
    sendBeeCountDataToSparkFun(beeCountThisHour, currentTime);
    initBeeCounters();
  }
  
  if (flashI2cLed == true)
  {
    flashLed(LED_LEFT_PIN, 1, 25);
    flashI2cLed = false; 
  }
  
  loopCount++;
  wdtCount = NUM_INTERVALS_TO_RESET;  // restore watchdog count
  
  delay(50L); 
}

void cc3000_init(void)
{
  // Serial.println(F("Hello, CC3000!\n")); 

  // Serial.print("Free RAM: "); Serial.println(getFreeRam(), DEC);
  
  /* Initialise the module */
  Serial.println(F("CC3000 Init"));
  #if 0
  lcd.setCursor(0,0); 
  lcd.print("     "); 
  lcd.setCursor(0,0); 
  lcd.print("CC"); 
  #endif
  
  if (!cc3000.begin())
  {
    Serial.println(F("Couldn't begin()! Check your wiring?"));
    while(1);
  }
  
  // lcd.print("0"); 
  
  // Optional SSID scan
  // listSSIDResults();
  
  // Serial.print(F("\nAttempting to connect to ")); Serial.println(WLAN_SSID);
  if (!cc3000.connectToAP(WLAN_SSID, WLAN_PASS, WLAN_SECURITY)) {
    Serial.println(F("Failed!"));
    while(1);
  }
  //lcd.print("1"); 

  Serial.println(F("Connected!"));
  
  /* Wait for DHCP to complete */
  Serial.println(F("Request DHCP"));
  while (!cc3000.checkDHCP())
  {
    delay(100); // ToDo: Insert a DHCP timeout!
  }  
    
  // lcd.print("2"); 
  // delay(1000);
  // lcd.setCursor(0,0); 
  // lcd.print("       "); 
  /* Display the IP address DNS, Gateway, etc. */  
  //while (! displayConnectionDetails()) {
  //  delay(1000);
  //}

  // Serial.println(F("-------------------------------------"));
  
  /* You need to make sure to clean up after yourself or the CC3000 can freak out */
  /* the next time your try to connect ... */
  //Serial.println(F("\n\nDisconnecting"));
  //cc3000.disconnect();
  flashLed(LED_RIGHT_PIN, 4, 250);
}

// Minimalist time server query; adapted from Adafruit Gutenbird sketch,
// which in turn has roots in Arduino UdpNTPClient tutorial.
unsigned long getNtpTime(void) {

  uint8_t       buf[48];
  unsigned long ip, startTime, t = 0L;
    
  Serial.println(F("--- Start NTP request"));
  
  #if 0
  lcd.setCursor(0,1); //Start at character 0 on line 2
  lcd.print("                ");  // clear characters in this line
  lcd.setCursor(0,1); //Start at character 0 on line 2
  lcd.print("N");
  #endif
  // Serial.print(F("Locating time server..."));

  // Hostname to IP lookup; use NTP pool (rotates through servers)
  if(cc3000.getHostByName("pool.ntp.org", &ip)) 
  {
    static const char PROGMEM
      timeReqA[] = { 227,  0,  6, 236 },
      timeReqB[] = {  49, 78, 49,  52 };

    Serial.println(F("Attempting connection..."));
    //lcd.print("1");
    startTime = millis();
    do 
    {
      client = cc3000.connectUDP(ip, 123);
    } while((!client.connected()) &&
            ((millis() - startTime) < connectTimeout));

    if(client.connected()) 
    {
      Serial.print(F("connected!\r\nIssuing request..."));

      // Assemble and issue request packet
      memset(buf, 0, sizeof(buf));
      memcpy_P( buf    , timeReqA, sizeof(timeReqA));
      memcpy_P(&buf[12], timeReqB, sizeof(timeReqB));
      client.write(buf, sizeof(buf));

      Serial.print(F("\r\nAwaiting response..."));
      memset(buf, 0, sizeof(buf));
      startTime = millis();
      while((!client.available()) &&
            ((millis() - startTime) < responseTimeout));
      if(client.available()) 
      {
        client.read(buf, sizeof(buf));
        t = (((unsigned long)buf[40] << 24) |
             ((unsigned long)buf[41] << 16) |
             ((unsigned long)buf[42] <<  8) |
              (unsigned long)buf[43]) - 2208988800UL;
        Serial.println(F("OK"));
      }
      client.close();
      //lcd.print("2");
    }
  }
  if(!t) 
  {
    Serial.println(F("error"));
  }
  
  Serial.println(F("--- End NTP request"));

  return t;
}

uint32_t dstOffset (DateTime time)
{
  //Receives unix epoch time and returns seconds of offset for local DST
  //Valid thru 2099 for US only, Calculations from "http://www.webexhibits.org/daylightsaving/i.html"
  //Code idea from jm_wsb @ "http://forum.arduino.cc/index.php/topic,40286.0.html"
  //Get epoch times @ "http://www.epochconverter.com/" for testing
  //DST update wont be reflected until the next time sync
  int beginDSTDay = (14 - (1 + time.year() * 5 / 4) % 7);  
  int beginDSTMonth=3;
  int endDSTDay = (7 - (1 + time.year() * 5 / 4) % 7);
  int endDSTMonth=11;
  if (((time.month() > beginDSTMonth) && (time.month() < endDSTMonth))
    || ((time.month() == beginDSTMonth) && (time.day() > beginDSTDay))
    || ((time.month() == beginDSTMonth) && (time.day() == beginDSTDay) && (time.hour() >= 2))
    || ((time.month() == endDSTMonth) && (time.day() < endDSTDay))
    || ((time.month() == endDSTMonth) && (time.day() == endDSTDay) && (time.hour() < 1)))
    return (3600);  //Add back in one hours worth of seconds - DST in effect
  else
    return (0);  //NonDST
}

// feed in DateTime...get back string with time, like "12:45 pm" or "3:45am"
void getTimeAsString(DateTime dateTime, char *timeBuf_p, TIME_FORMAT_t format)
{
  char *ampm;
  uint8_t hour;
  uint8_t hour24;
  uint8_t minute;
 
  hour = dateTime.hour();
  hour24 = hour;
  minute = dateTime.minute();
  
  if (hour == 12)
  {
    ampm = "PM";
  }
  else if(hour < 13)
  {
    ampm = "AM";
    
    // detect and correct when hour is between midnight and 1am
    if (hour == 0)
    {
      hour = 12;
    }
  }
  else
  {
    ampm = "PM";
    hour = hour - 12;
  }
  
  if (format == LONG_TIME_FORMAT)
  {
    sprintf(timeBuf_p, "%u:%.2u %s", hour, minute, ampm);
  }
  else if (format == SHORT_TIME_FORMAT)
  {
    // don't include the AM/PM in the time string
    sprintf(timeBuf_p, "%u:%.2u", hour, minute);
  } 
  else // 24 hour format
  {
    sprintf(timeBuf_p, "%u:%.2u", hour24, minute);
  } 
}

//
// Push Sensor Data
//
void sendSensorDataToSparkFun(uint32_t *capSensorReadings, envSensorReadings_t *envSensorReadings, DateTime logTimestamp)
{
  wdtCount = NUM_INTERVALS_TO_RESET;  // restore watchdog count

  digitalWrite(LED_RIGHT_PIN, HIGH);   
  
  // Make a TCP connection to remote host
  ip = 0;
  // Try looking up the website's IP address
  Serial.print(sparkfunServer); Serial.print(F(" -> "));
  while (ip == 0) 
  {
    if (! cc3000.getHostByName(sparkfunServer, &ip)) 
    {
      Serial.println(F("Couldn't resolve!"));
    }
    delay(500);
  }
  
  cc3000.printIPdotsRev(ip);
  Serial.println();

  client = cc3000.connectTCP(ip, 80);
  //if ( !client.connect(sparkfunServer, 80) )
  if(!client.connected())
  {
    // Error: 4 - Could not make a TCP connection
    Serial.println(F("Error: 4"));
  }
  else
  {
    Serial.println(F("Connected to Sparkfun Data"));
  }
  
  #if 0
  lcd.setCursor(0,1); //Start at character 0 on line 2
  lcd.print("                ");  // clear characters in this line
  lcd.setCursor(0,1); //Start at character 0 on line 2
  lcd.print("S");
  #endif
  
  // Post the data! Request should look a little something like:
  // GET /input/publicKey?private_key=privateKey&light=1024&switch=0&time=5201 HTTP/1.1\n
  // Host: data.sparkfun.com\n
  // Connection: close\n
  // \n

  client.print("GET /input/");
  client.print(sensorPublicKey);
  client.print("?private_key=");
  client.print(sensorPrivateKey);
  
  client.print("&");
  client.print("c1");
  client.print("=");
  client.print(capSensorReadings[0]);

  client.print("&");
  client.print("c2");
  client.print("=");
  client.print(capSensorReadings[1]);

  client.print("&");
  client.print("c3");
  client.print("=");
  client.print(capSensorReadings[2]);
  
  client.print("&");
  client.print("c4");
  client.print("=");
  client.print(capSensorReadings[3]);
  
  client.print("&");
  client.print("c5");
  client.print("=");
  client.print(capSensorReadings[4]);
  
  client.print("&");
  client.print("c6");
  client.print("=");
  client.print(capSensorReadings[5]);
  
  client.print("&");
  client.print("c7");
  client.print("=");
  client.print(capSensorReadings[6]);
 
  client.print("&");
  client.print("c8");
  client.print("=");
  client.print(capSensorReadings[7]);
  
  client.print("&");
  client.print("degc");
  client.print("=");
  client.print(envSensorReadings->tempInDegC);
   
  client.print("&");
  client.print("rh");
  client.print("=");
  client.print(envSensorReadings->humidityInRh);
    
  client.print("&");
  client.print("hpa");
  client.print("=");
  client.print(envSensorReadings->pressureInHpa);

  client.print("&");
  client.print("lux");
  client.print("=");
  client.print(envSensorReadings->lightInLux);

  uint8_t resetCountSlave = EEPROM.read(EEPROMaddrForResetCount);
  client.print("&");
  client.print("rcwifi");
  client.print("=");
  client.print(resetCountSlave);
 
  uint8_t resetCountSensor = diagnosticsPacket.resetCountSensorUnit;
  client.print("&");
  client.print("rcsensor");
  client.print("=");
  client.print(resetCountSensor);

  client.print("&");
  client.print("mm");
  client.print("=");
  client.print(diagnosticsPacket.maintMode);
  
  client.print("&");
  client.print("unixtime");
  client.print("=");
  client.print(logTimestamp.unixtime());
  
  client.print("&");
  client.print("localdate");
  client.print("=");
  client.print(logTimestamp.day());
  client.print("/");
  client.print(logTimestamp.month());
  client.print("/");
  client.print(logTimestamp.year());

  char timeStr[10];
  getTimeAsString(logTimestamp, timeStr, HR24_TIME_FORMAT);
  client.print("&");
  client.print("localtime");
  client.print("=");
  client.print(timeStr);

  client.println(" HTTP/1.1");
  client.print("Host: ");
  client.println(sparkfunServer);
  client.println("Connection: close");
  client.println();

  while (client.connected())
  {
    if ( client.available() )
    {
      char c = client.read();
      //Serial.print(c);
    }      
  }
  
  digitalWrite(LED_RIGHT_PIN, LOW);   
}  

//
// Push Bee Count Data
//
void sendBeeCountDataToSparkFun(uint16_t *beeCountThisHour, DateTime logTimestamp)
{
  wdtCount = NUM_INTERVALS_TO_RESET;  // restore watchdog count
  
  digitalWrite(LED_RIGHT_PIN, HIGH);   
  
  // Make a TCP connection to remote host
  ip = 0;
  // Try looking up the website's IP address
  Serial.print(sparkfunServer); Serial.print(F(" -> "));
  while (ip == 0) 
  {
    if (! cc3000.getHostByName(sparkfunServer, &ip)) 
    {
      Serial.println(F("Couldn't resolve!"));
    }
    delay(500);
  }
  
  cc3000.printIPdotsRev(ip);
  Serial.println();

  client = cc3000.connectTCP(ip, 80);
  //if ( !client.connect(sparkfunServer, 80) )
  if(!client.connected())
  {
    // Error: 4 - Could not make a TCP connection
    Serial.println(F("Error: 4"));
  }
  #if 0
  lcd.setCursor(0,1); //Start at character 0 on line 2
  lcd.print("                ");  // clear characters in this line
  lcd.setCursor(0,1); //Start at character 0 on line 2
  lcd.print("S");
  #endif
  
  // Post the data! Request should look a little something like:
  // GET /input/publicKey?private_key=privateKey&light=1024&switch=0&time=5201 HTTP/1.1\n
  // Host: data.sparkfun.com\n
  // Connection: close\n
  // \n

  client.print("GET /input/");
  client.print(beeCountPublicKey);
  client.print("?private_key=");
  client.print(beeCountPrivateKey);
  
  client.print("&");
  client.print("cnt1");
  client.print("=");
  client.print(beeCountThisHour[0]);

  client.print("&");
  client.print("cnt2");
  client.print("=");
  client.print(beeCountThisHour[1]);

  client.print("&");
  client.print("cnt3");
  client.print("=");
  client.print(beeCountThisHour[2]);

  client.print("&");
  client.print("cnt4");
  client.print("=");
  client.print(beeCountThisHour[3]);

  client.print("&");
  client.print("cnt5");
  client.print("=");
  client.print(beeCountThisHour[4]);

  client.print("&");
  client.print("cnt6");
  client.print("=");
  client.print(beeCountThisHour[5]);

  client.print("&");
  client.print("cnt7");
  client.print("=");
  client.print(beeCountThisHour[6]);

  client.print("&");
  client.print("cnt8");
  client.print("=");
  client.print(beeCountThisHour[7]);

  client.print("&");
  client.print("localdate");
  client.print("=");
  client.print(logTimestamp.day());
  client.print("/");
  client.print(logTimestamp.month());
  client.print("/");
  client.print(logTimestamp.year());

  char timeStr[10];
  getTimeAsString(logTimestamp, timeStr, HR24_TIME_FORMAT);
  client.print("&");
  client.print("localtime");
  client.print("=");
  client.print(timeStr);

  client.println(" HTTP/1.1");
  client.print("Host: ");
  client.println(sparkfunServer);
  client.println("Connection: close");
  client.println();

  while (client.connected())
  {
    if ( client.available() )
    {
      char c = client.read();
      //Serial.print(c);
    }      
  }

  digitalWrite(LED_RIGHT_PIN, LOW);   
}  

boolean updateDuinoTime(void)
{
  char timeStr[10];
  unsigned long t;
    
  digitalWrite(LED_LEFT_PIN, HIGH);   
  
  t = getNtpTime(); // Query time server
 
  if(t) 
  {                       // Success?
    t = t - TZ_OFFSET + dstOffset(t);  // Adjust for Timezone and DST
    lastPolledTime = t;         // Save time
    sketchTime     = millis();  // Save sketch time of last valid time query
    ntpTimeAvailable = true;  // raise flag to indicate new NTP time update is available
    getTimeAsString(t, timeStr, LONG_TIME_FORMAT);
    Serial.print(F("Time: "));
    Serial.println(timeStr);
    //lcd.setCursor(10,1); 
    //lcd.print(timeStr);
    
    // two flashes indicate success
    flashLed(LED_RIGHT_PIN, 2, 250);
    digitalWrite(LED_LEFT_PIN, LOW); 
    return true;
  }
  else
  {
    // one long flash indicates failure
    flashLed(LED_RIGHT_PIN, 1, 2000);
    digitalWrite(LED_LEFT_PIN, LOW); 
    return false;
  }
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int numBytesReceived)
{
  uint16_t i=0;
  I2cReceiveCount++; 
  
  flashI2cLed = true;  // flash outside the receive event

  switch (numBytesReceived)
  {
    case sizeof(capSensorReadings):
      while(Wire.available())
      {
        ((byte *)(&capSensorReadings))[i++] = Wire.read();
      }

      if (firstTimeStartup == false)
      {
        // look for bees entering tube, indicated by an increase in capacitance
        for(i=0; i<NUMBER_OF_CAP_SENSORS; i++)
        {
          if (capSensorReadings[i]  > (oldCapReadings[i] + diagnosticsPacket.beeDetectThreshold))
          {
            #if 0
            Serial.print(F("newCap["));
            Serial.print(i);
            Serial.print(F("]:"));
            Serial.println(capSensorReadings[i]);
            Serial.print(F("oldCap["));
            Serial.print(i);
            Serial.print(F("]:"));
            Serial.println(oldCapReadings[i]);
            #endif

            // only count bee activity in normal operation mode
            if (diagnosticsPacket.maintMode == 0)
            {
              beeCountThisHour[i]++;
            }
          }
          
          #if 0
          Serial.print(F("bee count["));
          Serial.print(i);
          Serial.print(F("]:"));
          Serial.println(beeCountThisHour[i]);       
          #endif
        }
      }
      else
      {
        // need to wait until there are old cap readings before detecting bees entering or exiting
        #if 0
        Serial.println(F("First time start"));       
        #endif
        
        firstTimeStartup = false;
      }
        
      memcpy(&oldCapReadings, &capSensorReadings, sizeof(capSensorReadings));
      break;
     
    case sizeof(envSensorReadings_t):
      while(Wire.available())
      {
        ((byte *)(&envSensorReadings))[i++] = Wire.read();
      }
      break;
      
    case sizeof(diagnosticsPacket_t):
      while(Wire.available())
      {
        ((byte *)(&diagnosticsPacket))[i++] = Wire.read();
      }
      break;        
  }
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void requestEvent(void)
{
  timePacket_t timePacket;
  
  I2cRequestCount++; 
  
  if (ntpTimeAvailable == true)
  {
    // send back the time to the sensor unit
    timePacket.startPattern = 0xA5;
    timePacket.currentTime = currentTime;
    timePacket.endPattern = 0x5A;
    
    timestampSentToSensorUnit = currentTime;
    
    Wire.write((byte *)&timePacket, sizeof(timePacket));  
    ntpTimeAvailable = false;  // knock down flag used to indicate new NTP time available
    checkTimestampRxBySensorUnit = true;  // initiate check to see if sensor unit received the timestamp
  }
  else
  {
    // send back a known pattern that will not instigate a time correction
    timePacket.startPattern = 0xBB;
    timePacket.currentTime = 0x11223344;
    timePacket.endPattern = 0xCC;
    Wire.write((byte *)&timePacket, sizeof(timePacket_t));  
  }
}

void flashLed(uint8_t ledIdent, uint8_t numberFlashes, uint8_t delayBetweenFlashesInMs)
{
  uint8_t i;
  
  for (i=0; i<numberFlashes; i++)
  {
    digitalWrite(ledIdent, HIGH);   
    delay(delayBetweenFlashesInMs);
    digitalWrite(ledIdent, LOW);   
    delay(delayBetweenFlashesInMs);
  }
}

void initBeeCounters(void)
{
  uint8_t i;
  for(i=0; i<NUMBER_OF_CAP_SENSORS; i++)
  {
    beeCountThisHour[i] = 0;
  }
}

ISR(WDT_vect) 
{ // Watchdog interrupt @ 8 sec. interval
  if(!--wdtCount) 
  { // Decrement sleep interval counter...
    // If it reaches zero, force a processor reset
    uint8_t resetCount = EEPROM.read(EEPROMaddrForResetCount);
    resetCount++;
    EEPROM.write(EEPROMaddrForResetCount, resetCount);
    wdt_enable(WDTO_15MS); // turn on the WatchDog and allow it to fire
    while(1);
  }
}



