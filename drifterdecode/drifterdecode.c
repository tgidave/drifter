#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "../drifterb/drifter.h"

#define BUFFSIZE 512

//char buff[BUFFSIZE] = "e00g0400090015002b002400682f2142ffe9d1c2000000000000000033838a4433838a4433838a4433838a4433838a4433838a4433838a4433838a4433838a4433838a4433838a4433838a4433838a4433838a4433838a4433838a4433838a4433838a4433838a4433838a4433838a4433838a4433838a4433838a4433838a4433838a4433838a4433838a4433838a4433838a44";
char buff[BUFFSIZE];

drifterData dData;

uint16_t convertToShort(char *ptr);
uint32_t convertToInt(char *ptr);
float convertToFloat(char *ptr);
uint16_t convertCharToHex(char *ptr);

int main(int argc, char **argv) {
  char *ptr = buff;
  int i;
  char buffHold;
  
  if(fgets(buff, BUFFSIZE, stdin) == NULL) {
    printf("\r\nData read error!\r\n");
    return(1);
  }
  
  dData.ddYear = convertToShort(ptr);
  ptr += 4;
  dData.ddMonth = convertToShort(ptr);
  ptr += 4;
  dData.ddDay = convertToShort(ptr);
  ptr += 4;
  dData.ddHour = convertToShort(ptr);
  ptr += 4;
  dData.ddMinute = convertToShort(ptr);
  ptr += 4;
  dData.ddSecond = convertToShort(ptr);
  ptr += 4;
  dData.ddLatitude = convertToFloat(ptr);
  ptr += 8;
  dData.ddLongitude = convertToFloat(ptr);
  ptr += 8;
  dData.ddSpeed = convertToInt(ptr);
  ptr += 8;
  dData.ddCourse = convertToFloat(ptr);
  ptr += 8;

  for (i = 0; i < WAVE_COUNT; i++) {
    dData.ddAltitude[i] = convertToFloat(ptr);
    ptr += 8;
  }
  
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
  printf("Altitudes = %f\r\n", dData.ddAltitude[0]);
  for (i = 1; i < WAVE_COUNT; i++) {
    printf("            %f\r\n", dData.ddAltitude[i]);
  }
}

uint16_t convertToShort(char *ptr) {
  uint16_t hold;
  uint16_t hold2;
  int i;

  hold = 0;

  for (i = 0; i < 4; i++) {
    hold <<= 4;
    hold2 = convertCharToHex(ptr);
    hold |= hold2;
    ptr++;
  }

  hold2 = hold << 8;
  hold >>= 8;
  hold |= hold2;

  return(hold);
}

uint32_t convertToInt(char *ptr) {
  uint16_t half1;
  uint16_t half2;

  half1 = convertToShort(ptr);
  ptr += 4; 
  half2 = convertToShort(ptr);
  return((half2 << 16) | half1);
}

float convertToFloat(char *ptr) {

  union floatData {
    float fhold;
    uint32_t xhold;
  } fdata;
  
  fdata.xhold = convertToInt(ptr);
  return(fdata.fhold);
}

uint16_t convertCharToHex(char *ptr) {
  if ((*ptr >= '0') && (*ptr <= '9') ) {
    return(*ptr - '0');
  } else if ((*ptr >= 'a') && (*ptr <= 'f')) {
    return((*ptr - 'a') + 10);
  } else if ((*ptr >= 'A') && (*ptr <= 'F')) {
    return((*ptr - 'A') + 10);
  }

  printf("Invalid hex char found at position %ld\r\n", ((ptr - (char *)&buff) + 1));
  exit(1);
}

