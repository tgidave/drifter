#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include "../drifterb/drifter.h"

#define false 0
#define true !false

drifterData dData;

int json =  false;

int firstJasonFormat = true;

void processSwitch(char *arg);
void processFile(char *fileName);
void formatPrint(void);
void formatJason(void);

int inFile;

int main(int argc, char **argv) {

  int i;
  
  for (i = 1; i < argc; ++i) {
    if (*argv[i] == '-') {
      processSwitch(argv[i]);
    } else {
      processFile(argv[i]);
    }
  }

  if (json) {
    printf("]}\r\n");
  }
}

void processSwitch(char *arg) {
  switch (arg[1]) {
    case 'j':
      json = true;
      break;
    default:
      printf("Invalid argument ");
      printf("%s", arg);
      printf("\r\n");
  }
}
                      
void processFile(char *fileName) {

  int i;
  
  if ((inFile = open(fileName, O_RDONLY)) == -1) {
    printf("Can not open file ");
    printf("%s", fileName);
    printf("\r\n");
    printf("%s", strerror(errno));
    printf("\r\n");
    return;
  }

  if(read(inFile, &dData, sizeof(dData), stdin) == 0) {
    printf("\r\nData read error!\r\n");
    return;
  }

  if (json) {
    formatJason();
  } else {
    formatPrint();
  }
}

void formatPrint(void) {

  int i;

  printf("\r\n%d/%02d/%02d %02d:%02d:%02d\r\n",
         dData.ddYear, 
         dData.ddMonth, 
         dData.ddDay, 
         dData.ddHour, 
         dData.ddMinute, 
         dData.ddSecond);
  printf("Latitude = %f\r\n", dData.ddLatitude);
  printf("Longitude = %f\r\n", dData.ddLongitude);
  printf("Speed = %d\r\n", dData.ddSpeed);
  printf("Course = %f\r\n", dData.ddCourse);
  printf("Raw Red = %d\r\n", dData.ddRawRed);
  printf("Raw Green = %d\r\n", dData.ddRawGreen);
  printf("Raw Blue = %d\r\n", dData.ddRawBlue);
  printf("Raw Clear = %d\r\n", dData.ddRawClear);
  printf("Altitudes = %f\r\n", dData.ddAltitude[0]);
  for (i = 1; i < WAVE_COUNT; i++) {
    printf("            %f\r\n", dData.ddAltitude[i]);
  }
}

void formatJason(void) {

  int i;

  if (firstJasonFormat == true) {
    printf("{\"points\": [{");
    firstJasonFormat = false;
  } else {
    printf(",{");
  }

  printf("\"year\": %d, ", dData.ddYear);
  printf("\"month\": %d, ", dData.ddMonth);
  printf("\"day\": %d, ", dData.ddDay);
  printf("\"hour\": %d, ", dData.ddHour);
  printf("\"minute\": %d, ", dData.ddMinute);
  printf("\"second\": %d, ", dData.ddSecond);
  printf("\"latitude\": %f, ", dData.ddLatitude);
  printf("\"longitude\": %f, ", dData.ddLongitude);
  printf("\"speed\": %d, ", dData.ddSpeed);
  printf("\"course\": %f, ", dData.ddCourse);
  printf("\"rawRed\": %d, ", dData.ddRawRed);
  printf("\"rawGreen\": %d, ", dData.ddRawGreen);
  printf("\"rawBlue\": %d, ", dData.ddRawBlue);
  printf("\"rawClear\": %d, ", dData.ddRawClear);
  printf("\"Altitude\": [");
  
  for (i = 0; i < WAVE_COUNT; ++i) {
    printf("%f", dData.ddAltitude[i]);
    if (i != WAVE_COUNT - 1) {
      printf(",");
    }
  }
  printf("]}");
}
