
//#define SERIAL_DEBUG

#ifdef SERIAL_DEBUG
  #define SERIAL_DEBUG_GPS
  #define SERIAL_DEBUG_ROCKBLOCK
  #define SERIAL_DEBUG_TCS
#endif

//#define ALWAYS_TRANSMIT
//#define NEVER_TRANSMIT

#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include <avr/interrupt.h>

#include <SoftwareSerial.h>
#include <IridiumSBD.h>
#include <TinyGPS++.h> // NMEA parsing: http://arduiniana.org

#ifdef SERIAL_DEBUG
  #include <PString.h> // String buffer formatting: http://arduiniana.org
#endif

#include <Wire.h>
#include <Adafruit_TCS34725.h>

#include "drifter.h"

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

#define TCS_SDA_PIN 17
#define TCS_SCI_PIN 16
#define TCS_POWER_PIN 19
#define TCS_BAUD 9600

#define CONSOLE_BAUD 115200

#ifdef SERIAL_DEBUG
  #define OUTBUFFER_SIZE  340
#endif

SoftwareSerial ssIridium(ROCKBLOCK_RX_PIN, ROCKBLOCK_TX_PIN);
SoftwareSerial ssGPS(GPS_RX_PIN, GPS_TX_PIN);
IridiumSBD isbd(ssIridium, ROCKBLOCK_SLEEP_PIN);
TinyGPSPlus tinygps;
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_1X);

void gpsReadDelay(unsigned long ms);
int getGPSFix(void);
int transmitGPSFix(int fixfnd);
int getTSCColorValues(void);

drifterData dData;

int noFixFoundCount;

#ifdef SERIAL_DEBUG
  char outBuffer[340]; // Always try to keep message short
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

  pinMode(TCS_POWER_PIN, OUTPUT);
  digitalWrite(TCS_POWER_PIN, LOW);

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

  getTSCColorValues();

#ifdef ALWAYS_TRANSMIT
  transmitGPSFix(fixFound);
#else
  #ifndef NEVER_TRANSMIT
  if ((tinygps.time.hour() == 0) || (tinygps.time.hour() == 12)) {
    transmitGPSFix(fixFound);
  }
  #endif
#endif

  // Sleep
#ifdef SERIAL_DEBUG
  Serial.print("sleep an hour\r\n");
  Serial.flush();
  Serial.end();
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

  do {
    powerDown();

    sleepSecs += 8;

    if (sleepSecs > 59) {
      --sleepMins;
#ifdef SERIAL_DEBUG
      sprintf(outBuffer, "%d mins\r\n", sleepMins);
      Serial.begin(CONSOLE_BAUD);
      Serial.print(outBuffer);
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

