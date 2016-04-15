//*****************************************************************************
//
// drifterb.ino
//
// This is the code for the main processor of the drifter device.  This code
// sleeps for an hour in low power mode and then wakes up and takes a GPS 
// reading getting the date, time, location, speed, vector, and altitude.
// The altitude may to sampled for a number of time to detect wave data.  The 
// color of the sky may also be recorded.  This data is collected in the drifter
// data structure.  At certain hours during the day the drifter data structure 
// is transmitted back to the user using the Iridium system.  The system then 
// goes back to sleep for an hour.  
//
//*****************************************************************************

//#define SERIAL_DEBUG  // Turn on serial port debugging. Requires lots of memory!

#ifdef SERIAL_DEBUG
  #define SERIAL_DEBUG_GPS
  #define SERIAL_DEBUG_ROCKBLOCK
  #define SERIAL_DEBUG_TCS
#endif

//#define ALWAYS_TRANSMIT // uncomment this line to transmit every hour.
//#define NEVER_TRANSMIT  // uncomment this line to never transmit.

#define TCS34725_ATTACHED  // If no TCS34725 is attached comment out this define.

#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include <avr/interrupt.h>

#include <SoftwareSerial.h>
#include <IridiumSBD.h>
#include <TinyGPS++.h> // NMEA parsing: http://arduiniana.org

#include "drifter.h"

#ifdef SERIAL_DEBUG
  #include <PString.h> // String buffer formatting: http://arduiniana.org
#endif

#ifdef TCS34725_ATTACHED
  #include <Wire.h>
  #include <Adafruit_TCS34725.h>
#endif

//*****************************************************************************
//
// The TRANSMIT_HOUR_1 through TRANSMIT_HOUR_4 defines below are in UTC 24 hour
// time.  They can be any value between 0 and 23.  Setting a define to 04 will 
// transmit around 9:30 PM and 16 will transmit around 9:30 AM Pacific Daylight Time.  
// These values can be set to transmit once, twice, three times, or four times per day.  
// Setting the transmit times to the same value for any two, three, or four values
// will cause those values to match at the same time and only one transmit to occur
// for all matching values. 
//
// For example:
//
// #define TRANSMIT_HOUR_1 00
// #define TRANSMIT_HOUR_2 06
// #define TRANSMIT_HOUR_3 12
// #define TRANSMIT_HOUR_4 18
// 
// Will transmit four time a day and:
// 
// #define TRANSMIT_HOUR_1 00
// #define TRANSMIT_HOUR_2 00
// #define TRANSMIT_HOUR_3 00
// #define TRANSMIT_HOUR_4 00
// 
// Will transmit once a day. 
// 
// All transmits occur near the 30 minute mark.
// 
//*****************************************************************************
 
#define TRANSMIT_HOUR_1 04
#define TRANSMIT_HOUR_2 16
#define TRANSMIT_HOUR_3 16
#define TRANSMIT_HOUR_4 16

#define ROCKBLOCK_RX_PIN 11 // Pin marked RX on RockBlock
#define ROCKBLOCK_TX_PIN 13 // Pin marked TX on RockBlock
#define ROCKBLOCK_SLEEP_PIN 18
#define ROCKBLOCK_BAUD 19200
#define ROCKBLOCK_POWER_PIN 15

#define GPS_RX_PIN 12 //Pin marked TX on GPS board
#define GPS_TX_PIN 10 //Pin marked RX on GPS board
#define GPS_POWER_PIN 14
#define GPS_BAUD 9600
#define MAX_INVALID_ALTITUDE_RETRY_COUNT 3

#ifdef TCS34725_ATTACHED
  #define TCS_SDA_PIN 17
  #define TCS_SCI_PIN 16
  #define TCS_POWER_PIN 19
  #define TCS_BAUD 9600
#endif

#define CONSOLE_BAUD 115200

#ifdef SERIAL_DEBUG
  #define OUTBUFFER_SIZE  340
#endif

SoftwareSerial ssIridium(ROCKBLOCK_RX_PIN, ROCKBLOCK_TX_PIN);
SoftwareSerial ssGPS(GPS_RX_PIN, GPS_TX_PIN);
IridiumSBD isbd(ssIridium, ROCKBLOCK_SLEEP_PIN);
TinyGPSPlus tinygps;

#ifdef TCS34725_ATTACHED
  Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_1X);
#endif

void gpsReadDelay(unsigned long ms);
int getGPSFix(void);
int transmitGPSFix(int fixfnd);

#ifdef TCS34725_ATTACHED
  int getTSCColorValues(void);
#endif

drifterData dData;

int noFixFoundCount;

#ifdef SERIAL_DEBUG
  int hourCount;
  char outBuffer[OUTBUFFER_SIZE]; // Always try to keep message short

  #define MAX_SPIN 4

  char spin[MAX_SPIN]= {'|', '/', '-', '\\'};
  int spinIndex;
#endif

unsigned long loopStartTime;

enum period_t
{
	SLEEP_15MS,
	SLEEP_30MS,	
	SLEEP_60MS,
	SLEEP_120MS,
	SLEEP_250MS,
	SLEEP_500MS,
	SLEEP_1S,
	SLEEP_2S,
	SLEEP_4S,
	SLEEP_8S,
	SLEEP_FOREVER
};

#define sleep_bod_disable() 										\
do { 																\
  unsigned char tempreg; 													\
  __asm__ __volatile__("in %[tempreg], %[mcucr]" "\n\t" 			\
                       "ori %[tempreg], %[bods_bodse]" "\n\t" 		\
                       "out %[mcucr], %[tempreg]" "\n\t" 			\
                       "andi %[tempreg], %[not_bodse]" "\n\t" 		\
                       "out %[mcucr], %[tempreg]" 					\
                       : [tempreg] "=&d" (tempreg) 					\
                       : [mcucr] "I" _SFR_IO_ADDR(MCUCR), 			\
                         [bods_bodse] "i" (_BV(BODS) | _BV(BODSE)), \
                         [not_bodse] "i" (~_BV(BODSE))); 			\
} while (0)

void	powerDown(void)
{
  ADCSRA &= ~(1 << ADEN);
  wdt_enable(SLEEP_8S);
  WDTCSR |= (1 << WDIE);	
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); 
  cli();		    
  sleep_enable();	    
  sleep_bod_disable();      
  sei();		    
  sleep_cpu();		    
  sleep_disable();	    
  sei();		    
  ADCSRA |= (1 << ADEN);
}

ISR (WDT_vect)
{
	// WDIE & WDIF is cleared in hardware upon entering this ISR
	wdt_disable();
}

void setup() {

  int i;

  pinMode(GPS_POWER_PIN, OUTPUT);
  digitalWrite(GPS_POWER_PIN, LOW);

  pinMode(ROCKBLOCK_POWER_PIN, OUTPUT);
  digitalWrite(ROCKBLOCK_POWER_PIN, LOW);

#ifdef TCS34725_ATTACHED
  pinMode(TCS_POWER_PIN, OUTPUT);
  digitalWrite(TCS_POWER_PIN, LOW);
#endif

  dData.ddYear = 0;
  dData.ddMonth = 0;
  dData.ddDay = 0;
  dData.ddHour = 0;
  dData.ddMinute = 0;
  dData.ddSecond = 0;
  dData.ddLatitude = 0;
  dData.ddLongitude = 0;
  dData.ddSpeed = 0;
  dData.ddCourse = 0;
  dData.ddRawRed = 0;
  dData.ddRawGreen = 0;
  dData.ddRawBlue = 0;
  dData.ddRawClear = 0;
  noFixFoundCount = 0; 

  for ( i = 0; i < WAVE_COUNT; ++i ) {
    dData.ddAltitude[i] = 0;
  }

#ifdef SERIAL_DEBUG
  hourCount = 0;
  // Start the serial ports
  Serial.begin(CONSOLE_BAUD);
#endif
}

void loop() {

  int i;
  int fixFound;
  int sleepSecs;
  int sleepMins;

  fixFound = getGPSFix();

  if (fixFound) {
    noFixFoundCount = 0;
  } else {
    ++noFixFoundCount;
  }

#ifdef SERIAL_DEBUG
  sprintf(outBuffer, "Data received at: %d/%02d/%02d %02d:%02d:%02d\r\n",
          dData.ddYear, dData.ddMonth, dData.ddDay, dData.ddHour, dData.ddMinute, dData.ddSecond);
  Serial.print(outBuffer);
  Serial.flush();
#endif

#ifdef TCS34725_ATTACHED
  getTSCColorValues();
#endif

#ifdef ALWAYS_TRANSMIT
  transmitGPSFix(fixFound);
#else
  #ifndef NEVER_TRANSMIT
  if ((tinygps.time.hour() == TRANSMIT_HOUR_1) || 
      (tinygps.time.hour() == TRANSMIT_HOUR_2) ||
      (tinygps.time.hour() == TRANSMIT_HOUR_3) || 
      (tinygps.time.hour() == TRANSMIT_HOUR_4)) {
    transmitGPSFix(fixFound);
  }
  #endif
#endif

  // Sleep
#ifdef SERIAL_DEBUG
  sprintf(outBuffer, "sleep an hour %d\r\n", hourCount);
  Serial.print(outBuffer);
  Serial.flush();
  Serial.end();
  ++hourCount;
#endif
  sleepSecs = 0;

  if (fixFound) {
    sleepMins = 90 - dData.ddMinute;

    if (sleepMins >= 75) {
      sleepMins -= 60;
    }
  } else {
    sleepMins = 60;
  }

#ifdef SERIAL_DEBUG
  spinIndex = 0;
#endif

  do {
    powerDown();

    sleepSecs += 8;

#ifdef SERIAL_DEBUG
    Serial.begin(CONSOLE_BAUD);
    Serial.write(spin[spinIndex]);
    Serial.write('\r');
    Serial.flush();
    Serial.end();
    if (++spinIndex >= MAX_SPIN) {
      spinIndex = 0;
    }
#endif

    if (sleepSecs > 59) {
      --sleepMins;
#ifdef SERIAL_DEBUG
//      sprintf(outBuffer, "%d mins\r\n", sleepMins);
      Serial.begin(CONSOLE_BAUD);
      Serial.print(sleepMins);
      Serial.print(" mins\r\n");
      Serial.flush();
      Serial.end();
#endif
      sleepSecs -= 60;
    }
  } while (sleepMins > 0);
#ifdef SERIAL_DEBUG
  Serial.begin(CONSOLE_BAUD);
  Serial.print("wake up\r\n");
  Serial.flush();
#endif
}

void gpsReadDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (ssGPS.available())
      tinygps.encode(ssGPS.read());
  } while (millis() - start < ms);
}

int getGPSFix(void) {

  int i;
  int fixfnd = false;
  unsigned long now;
  char *ptr;
  int notValidCount;

  loopStartTime = millis();

  digitalWrite(GPS_POWER_PIN, HIGH);
  ssGPS.begin(GPS_BAUD);

  // Step 1: Reset TinyGPS++ and begin listening to the GPS
#ifdef SERIAL_DEBUG_GPS
  Serial.println("Beginning GPS");
  Serial.flush();
#endif
  tinygps = TinyGPSPlus();
  ssGPS.listen();

  // Step 2: Look for GPS signal for up to 7 minutes
  for (now = millis(); !fixfnd && ((millis() - now) < (5UL * 60UL * 1000UL));) {
    if (ssGPS.available()) {
      tinygps.encode(ssGPS.read());
      fixfnd = tinygps.location.isValid() && tinygps.date.isValid() &&
          tinygps.time.isValid() && tinygps.altitude.isValid();
    }
  }

#ifdef SERIAL_DEBUG_GPS
  if (fixfnd) {
    Serial.println("fix found");
    Serial.flush();
  } else {
    Serial.println("fix not found");
    Serial.flush();
  }
#endif

  if (fixfnd) {
    dData.ddYear = tinygps.date.year();
    dData.ddMonth = tinygps.date.month();
    dData.ddDay = tinygps.date.day();
    dData.ddHour = tinygps.time.hour();
    dData.ddMinute = tinygps.time.minute();
    dData.ddSecond = tinygps.time.second();
    dData.ddLatitude = tinygps.location.lat();
    dData.ddLongitude = tinygps.location.lng();
    dData.ddSpeed = tinygps.speed.knots();
    dData.ddCourse = tinygps.course.value() / 100;
    notValidCount = 0;

    for (i = 0; i < WAVE_COUNT; ) {
      if (tinygps.altitude.isValid()) {
        dData.ddAltitude[i] = tinygps.altitude.meters();
        ++i;
        notValidCount = 0;
      } else {
        ++notValidCount;
        if (notValidCount >= MAX_INVALID_ALTITUDE_RETRY_COUNT) {
          dData.ddAltitude[i] = 0;
          notValidCount = 0;
          ++i;
  #ifdef SERIAL_DEBUG_GPS
          Serial.print("notValidCount = 3\r\n");
          Serial.flush();
  #endif
        }
      }
        
  #ifdef SERIAL_DEBUG_GPS
      sprintf(outBuffer, "i = %d %d\r\n", i, notValidCount);
      Serial.print(outBuffer);
      Serial.flush();
  #endif
      gpsReadDelay(1000);
    }

#ifdef SERIAL_DEBUG_GPS
    *outBuffer = 0;
    PString str(outBuffer, OUTBUFFER_SIZE);
    str.print("fix found! ");
    str.print(dData.ddLatitude, 6);
    str.print(",");
    str.print(dData.ddLongitude, 6);
    str.print(",");
    str.print(dData.ddSpeed, 1);
    str.print(",");
    str.print(dData.ddCourse);
    str.print("\r\n");
    Serial.flush();
    Serial.print(outBuffer);

    str.begin();

    for (i = 0; i < WAVE_COUNT - 1; ++i) {
      str.print(dData.ddAltitude[i], 2);
      str.print(", ");
    }

    str.print(dData.ddAltitude[i], 2);
    str.print("\r\n");
#endif
  } 
#ifdef SERIAL_DEBUG_GPS
  else {
    strcpy(outBuffer, "No fix found.\r\n");
  }

  Serial.flush();
  Serial.print(outBuffer);
  Serial.flush();
#endif

  ssGPS.end();
  digitalWrite(GPS_POWER_PIN, LOW);
  return (fixfnd);
}

#ifdef TCS34725_ATTACHED
int getTSCColorValues(void) {
  boolean TCSFound;

  digitalWrite(TCS_POWER_PIN, HIGH);
  TCSFound = tcs.begin();

  if(TCSFound){

    delay(5);
    tcs.getRawData(&dData.ddRawRed, &dData.ddRawGreen, &dData.ddRawBlue, &dData.ddRawClear);

#ifdef SERIAL_DEBUG_TCS
     PString str(outBuffer, OUTBUFFER_SIZE);
     str.print("TCS Found sensor ");
     str.print(dData.ddRawRed);
     str.print(",");
     str.print(dData.ddRawGreen);
     str.print(",");
     str.print(dData.ddRawBlue);
     str.print(",");
     str.print(dData.ddRawClear);
     str.print("\r\n");
     Serial.print(outBuffer);
     Serial.flush();
#endif

  } else {

#ifdef SERIAL_DEBUG_TCS
    Serial.println("No TCS34725 found ... check your connections");
    Serial.flush();
#endif

    dData.ddRawRed = dData.ddRawGreen = dData.ddRawBlue = dData.ddRawClear = 0xFFFF;
  }

//  tcs.disable();
  tcs.clearInterrupt();
  digitalWrite(TCS_POWER_PIN, LOW);
  return(TCSFound);
}
#endif

int transmitGPSFix(int fixfnd) {

  int i;
  char *ptr;
  int msgLen;

  // Setup the RockBLOCK
#ifdef SERIAL_DEBUG_ROCKBLOCK
  isbd.attachConsole(Serial);
  isbd.attachDiags(Serial);
#endif
  isbd.setPowerProfile(1);

  digitalWrite(ROCKBLOCK_POWER_PIN, HIGH);
  ssIridium.begin(ROCKBLOCK_BAUD);
  // Step 3: Start talking to the RockBLOCK and power it up
#ifdef SERIAL_DEBUG_ROCKBLOCK
  Serial.flush();
  Serial.println("RockBLOCK begin");
  Serial.flush();
#endif
  ssIridium.listen();

  if (isbd.begin() == ISBD_SUCCESS) {
#ifdef SERIAL_DEBUG_ROCKBLOCK
    Serial.flush();
    Serial.println("Transmitting.");
    Serial.flush();
#endif
    isbd.sendSBDBinary((const uint8_t *)&dData, sizeof(dData));
  }

  isbd.sleep();
  ssIridium.end();
  digitalWrite(ROCKBLOCK_POWER_PIN, LOW);
  return (0);
}

