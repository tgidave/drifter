#define SERIAL_DEBUG
#define ALWAYS_TRANSMIT
//#define NEVER_TRANSMIT

#include <SoftwareSerial.h>
#include <LowPower.h>
#include <IridiumSBD.h>
#include <TinyGPS++.h> // NMEA parsing: http://arduiniana.org
#include <PString.h> // String buffer formatting: http://arduiniana.org
//#include <Wire.h>
//#include <Adafruit_TCS34725.h>

#define BEACON_INTERVAL 21600 // Time between transmissions
#define ROCKBLOCK_RX_PIN 3 // Pin marked RX on RockBlock
#define ROCKBLOCK_TX_PIN 5 // Pin marked TX on RockBlock
#define ROCKBLOCK_SLEEP_PIN 6
#define ROCKBLOCK_BAUD 19200
#define ROCKBLOCK_POWER_PIN 8
#define GPS_RX_PIN 4 //Pin marked TX on GPS board
#define GPS_TX_PIN 2 //Pin marked RX on GPS board
#define GPS_POWER_PIN 9
#define GPS_BAUD 9600
#define CONSOLE_BAUD 115200

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
unsigned long altitude;
unsigned long speed;
double course;

int noFixFoundCount;

char outBuffer[360]; // Always try to keep message short

unsigned long loopStartTime;

void setup() {
  pinMode(GPS_POWER_PIN, OUTPUT);
  pinMode(ROCKBLOCK_POWER_PIN, OUTPUT);
  digitalWrite(GPS_POWER_PIN, LOW);
  digitalWrite(ROCKBLOCK_POWER_PIN, LOW);

  dateYear = 0;
  dateMonth = 0;
  dateDay = 0;
  timeHour = 0;
  timeMinute = 0;
  timeSecond = 0;
  latitude = 0;
  longitude = 0;
  altitude = 0;
  speed = 0;
  course = 0;
  noFixFoundCount = 0; 

#ifdef SERIAL_DEBUG
  // Start the serial ports
  Serial.begin(CONSOLE_BAUD);
#endif
}

void loop() {

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
  Serial.print("Going to sleep mode for about an hour...\r\n");
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
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);

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

  int fixfnd = false;
  unsigned long now;
  char *ptr;

  loopStartTime = millis();

  digitalWrite(GPS_POWER_PIN, HIGH);
  ssGPS.begin(GPS_BAUD);

  // Step 1: Reset TinyGPS++ and begin listening to the GPS
#ifdef SERIAL_DEBUG
  Serial.println("Beginning to listen for GPS traffic...");
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

  if (fixfnd) {
    dateYear = tinygps.date.year();
    dateMonth = tinygps.date.month();
    dateDay = tinygps.date.day();
    timeHour = tinygps.time.hour();
    timeMinute = tinygps.time.minute();
    timeSecond = tinygps.time.second();
    latitude = tinygps.location.lat();
    longitude = tinygps.location.lng();
    altitude = tinygps.altitude.meters();
    speed = tinygps.speed.knots();
    course = tinygps.course.value() / 100;
#ifdef SERIAL_DEBUG
    int len = strlen(outBuffer);
    PString str(outBuffer, sizeof(outBuffer) - len);
    str.print("GPS fix was found! ");
    str.print(latitude, 6);
    str.print(",");
    str.print(longitude, 6);
    str.print(",");
    str.print(altitude);
    str.print(",");
    str.print(speed, 1);
    str.print(",");
    str.print(course);
    str.print("\r\n");
#endif                                  
  } 
#ifdef SERIAL_DEBUG
  else {
    strcpy(outBuffer, "No GPS fix was found.\r\n");
  }
#endif

#ifdef SERIAL_DEBUG
  Serial.flush();
  Serial.print(outBuffer);
  Serial.flush();
#endif
  ssGPS.end();
  digitalWrite(GPS_POWER_PIN, LOW);
  return (fixfnd);
}

int transmitGPSFix(int fixfnd) {

  char *ptr;

  // Setup the RockBLOCK
#ifdef SERIAL_DEBUG
  isbd.attachConsole(Serial);
  isbd.attachDiags(Serial);
#endif
  isbd.setPowerProfile(1);

  digitalWrite(ROCKBLOCK_POWER_PIN, HIGH);
  ssIridium.begin(ROCKBLOCK_BAUD);
  // Step 3: Start talking to the RockBLOCK and power it up
#ifdef SERIAL_DEBUG
  Serial.flush();
  Serial.println("Beginning to talk to the RockBLOCK...");
  Serial.flush();
#endif
  ssIridium.listen();

  if (isbd.begin() == ISBD_SUCCESS) {
    int len = strlen(outBuffer);
    PString str(outBuffer, sizeof(outBuffer) - len);
    str.print("Ocean Voyager land test,");
    str.print(latitude, 6);
    str.print(",");
    str.print(longitude, 6);
    str.print(",");
    str.print(altitude);
    str.print(",");
    str.print(speed, 1);
    str.print(",");
    str.print(course);
#ifdef SERIAL_DEBUG
    Serial.flush();
    Serial.print("Transmitting message: ");
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

