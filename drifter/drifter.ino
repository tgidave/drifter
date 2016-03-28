//#define SERIAL_DEBUG

#ifdef SERIAL_DEBUG
  #define SERIAL_DEBUG_GPS
  #define SERIAL_DEBUG_ROCKBLOCK
#endif

//#define ALWAYS_TRANSMIT
//#define NEVER_TRANSMIT

//#define ALTITUDE_ARRAY

#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include <avr/interrupt.h>

#include <SoftwareSerial.h>
//#include <LowPower.h>
#include <IridiumSBD.h>
#include <TinyGPS++.h> // NMEA parsing: http://arduiniana.org
#include <PString.h> // String buffer formatting: http://arduiniana.org
//#include <Wire.h>
//#include <Adafruit_TCS34725.h>

#define BEACON_INTERVAL 21600 // Time between transmissions
#define ROCKBLOCK_RX_PIN 11 // Pin marked RX on RockBlock
#define ROCKBLOCK_TX_PIN 13 // Pin marked TX on RockBlock
#define ROCKBLOCK_SLEEP_PIN 18
#define ROCKBLOCK_BAUD 19200
#define ROCKBLOCK_POWER_PIN 15
#define GPS_RX_PIN 12 //Pin marked TX on GPS board
#define GPS_TX_PIN 10 //Pin marked RX on GPS board
#define GPS_POWER_PIN 14
#define GPS_BAUD 9600
#define CONSOLE_BAUD 115200

#ifdef ALTITUDE_ARRAY
  #define WAVE_COUNT  30
#endif

#define OUTBUFFER_SIZE  370

SoftwareSerial ssIridium(ROCKBLOCK_RX_PIN, ROCKBLOCK_TX_PIN);
SoftwareSerial ssGPS(GPS_RX_PIN, GPS_TX_PIN);
IridiumSBD isbd(ssIridium, ROCKBLOCK_SLEEP_PIN);
TinyGPSPlus tinygps;

int getGPSFix(void);
int transmitGPSFix(int fixfnd);

int dateYear;
int dateMonth;
int dateDay;

int timeHour;
int timeMinute;
int timeSecond;

double latitude;
double longitude;

#ifndef ALTITUDE_ARRAY
unsigned long altitude;
#endif

unsigned long speed;
double course;

#ifdef ALTITUDE_ARRAY
double waveData[WAVE_COUNT];
#endif

int noFixFoundCount;

char outBuffer[OUTBUFFER_SIZE]; // Always try to keep message short

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
  pinMode(GPS_POWER_PIN, OUTPUT);
  pinMode(ROCKBLOCK_POWER_PIN, OUTPUT);
  digitalWrite(GPS_POWER_PIN, LOW);
  digitalWrite(ROCKBLOCK_POWER_PIN, LOW);

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

  dateYear = 0;
  dateMonth = 0;
  dateDay = 0;
  timeHour = 0;
  timeMinute = 0;
  timeSecond = 0;
  latitude = 0;
  longitude = 0;

#ifndef ALTITUDE_ARRAY
  altitude = 0;
#endif

  speed = 0;
  course = 0;
  noFixFoundCount = 0; 
  *outBuffer = 0;

#ifdef ALTITUDE_ARRAY
  for ( i = 0; i < WAVE_COUNT; ++i ) {
    waveData[i] = 0;
  }
#endif

  fixFound = getGPSFix();

  if (fixFound) {
    noFixFoundCount = 0;
  } else {
    ++noFixFoundCount;
  }

  #ifdef SERIAL_DEBUG
  sprintf(outBuffer, "%d/%02d/%02d %02d:%02d:%02d\r\n",
          dateYear, dateMonth, dateDay, timeHour, timeMinute, timeSecond);
  Serial.print(outBuffer);
  Serial.flush();
#endif

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
    sleepMins = 90 - timeMinute;

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

int getGPSFix(void) {

  int i;
  int fixfnd = false;
  unsigned long now;
  char *ptr;
  int notAvailableCount;

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
    dateYear = tinygps.date.year();
    dateMonth = tinygps.date.month();
    dateDay = tinygps.date.day();
    timeHour = tinygps.time.hour();
    timeMinute = tinygps.time.minute();
    timeSecond = tinygps.time.second();
    latitude = tinygps.location.lat();
    longitude = tinygps.location.lng();

#ifndef ALTITUDE_ARRAY
    altitude = tinygps.altitude.meters();
#endif

    speed = tinygps.speed.knots();
    course = tinygps.course.value() / 100;
    notAvailableCount = 0;

#ifdef ALTITUDE_ARRAY
    for (i = 0; i < WAVE_COUNT; ) {
      if (ssGPS.available()) {
        tinygps.encode(ssGPS.read());
          waveData[i] = tinygps.altitude.meters();
          ++i;
          notAvailableCount = 0;
      } else {
        ++notAvailableCount;
        if (notAvailableCount >= 3) {
          waveData[i] = 0;
          notAvailableCount = 0;
          ++i;
  #ifdef SERIAL_DEBUG_GPS
          Serial.print("notAvailableCount = 3\r\n");
          Serial.flush();
  #endif
        }
      }
        
  #ifdef SERIAL_DEBUG_GPS
      sprintf(outBuffer, "i = %d %d\r\n", i, notAvailableCount);
      Serial.print(outBuffer);
      Serial.flush();
  #endif
      delay(1000);
    }
#endif

#ifdef SERIAL_DEBUG_GPS
    *outBuffer = 0;
    PString str(outBuffer, OUTBUFFER_SIZE);
    str.print("fix found! ");
    str.print(latitude, 6);
    str.print(",");
    str.print(longitude, 6);
    str.print(",");
    str.print(speed, 1);
    str.print(",");

  #ifndef ALTITUDE_ARRAY
    str.print(altitude);
    str.print(",");
  #endif
    str.print(course);
    str.print("\r\n");
    Serial.flush();
    Serial.print(outBuffer);

  #ifdef ALTITUDE_ARRAY
    str.begin();

    for (i = 0; i < WAVE_COUNT - 1; ++i) {
      str.print(waveData[i] * 100, 2);
      str.print(", ");
    }

    str.print(waveData[i] * 100, 2);
    str.print("\r\n");
  #endif
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

int transmitGPSFix(int fixfnd) {

  int i;
  char *ptr;

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
    int len = strlen(outBuffer);
    PString str(outBuffer, sizeof(outBuffer) - len);
    str.print("OV test,");
    if (fixfnd) {
      str.print(latitude, 6);
      str.print(",");
      str.print(longitude, 6);
      str.print(",");

#ifndef ALTITUDE_ARRAY
      str.print(altitude);
      str.print(",");
#endif

      str.print(speed, 1);
      str.print(",");
      str.print(course);    

#ifdef ALTITUDE_ARRAY
      str.print(",");
      for (i = 0; i < WAVE_COUNT - 1; ++i) {
      str.print(waveData[i] * 100, 2);
        str.print(", ");
      }

      str.print(waveData[i] * 100, 2);
#endif

    } else {
      str.print("fix not found");
    }

#ifdef SERIAL_DEBUG_ROCKBLOCK
    Serial.flush();
    Serial.print("Transmitting: ");
    Serial.println(outBuffer);
    Serial.flush();
#endif
    isbd.sendSBDText(outBuffer);
  }

  isbd.sleep();
  ssIridium.end();
  digitalWrite(ROCKBLOCK_POWER_PIN, LOW);
  return (0);
}

