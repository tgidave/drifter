//*****************************************************************************
//
// drifterpower.ino
//
// This program will power down and power-on reset the drifter every 
// "DAYS_TO_SLEEP" days.  Since this process will be using the atmega326P's
// 8Mhz internal clock generator and because the powerDown code uses the watch
// dog timer set to 8 seconds, which is not very accurate, the actual time this 
// processor powers down the main drifter hardware will probably drift by 
// quite a bit.  Initially, in laboratory conditions (here at the house...)
// the time is drifting about 10% less that actual time.  Probably not a big
// deal if the power down time is set to every 3 days.
// 
// Do not set the "DAYS_TO_SLEEP" define to zero!  That will cause the
// seconds to wait to go negative and the next reset will take a long, long
// time!
// 
//*****************************************************************************

//#define SERIAL_DEBUG  // Turn on serial port debugging. Requires lots of memory!

#include <LowPower.h>

#define CONSOLE_BAUD 9600

#define DRIFTER_POWER_PIN 2

#define DAYS_TO_SLEEP 3UL

#define TICKS_TO_WAIT (((24UL*60UL*60UL)/8UL)*DAYS_TO_SLEEP)
//#define SECONDS_PER_DAY 60 // for testing only!
#define POWER_DOWN_WAIT_SECONDS 15UL

void setup() {
  pinMode(DRIFTER_POWER_PIN, OUTPUT);
  digitalWrite(DRIFTER_POWER_PIN, LOW);
#ifdef SERIAL_DEBUG
  // Start the serial ports
//  Serial.begin(CONSOLE_BAUD);
#endif
}

void loop() {

  unsigned long waitTime;

  // The processor can only stay in the power down state for about 8 seconds
  // so the number of seconds to the next power cycle is divided by 8.
  
//  waitTime = ((DAYS_TO_SLEEP * SECONDS_PER_DAY) - POWER_DOWN_WAIT_SECONDS ) / 8UL;
  waitTime = TICKS_TO_WAIT; //((DAYS_TO_SLEEP * SECONDS_PER_DAY) - POWER_DOWN_WAIT_SECONDS ) / 8UL;
#ifdef SERIAL_DEBUG
  Serial.print(waitTime);
  Serial.print("\r\n");
#endif
  
  while (waitTime > 0) {
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
    --waitTime; // Count down to the next power cycle.
#ifdef SERIAL_DEBUG
    Serial.begin(CONSOLE_BAUD);
    Serial.println(waitTime, HEX);
    Serial.flush();
    Serial.end();
#endif
  }

#ifdef SERIAL_DEBUG
    Serial.print("Powering down the drifter\r\n");
#endif
  digitalWrite(DRIFTER_POWER_PIN, HIGH);  // Drop power to system.
  delay(POWER_DOWN_WAIT_SECONDS * 1000);  // Delay() counts milliseconds so wait
                                          // POWER_DOWN_WAIT_SECONDS times 1000.
  digitalWrite(DRIFTER_POWER_PIN, LOW);   // Start the system back up.  
}

