#ifndef DRIFTER_H
  #define DRIFTER_H

#define WAVE_COUNT  30

typedef struct drifterData{
  uint16_t ddYear;
  uint16_t ddMonth;
  uint16_t ddDay;
  uint16_t ddHour;
  uint16_t ddMinute;
  uint16_t ddSecond;
  float ddLatitude;
  float ddLongitude;
  uint32_t ddSpeed;
  float ddCourse;
  uint16_t  ddRawRed;
  uint16_t  ddRawGreen;
  uint16_t  ddRawBlue;
  uint16_t  ddRawClear;
  float ddAltitude[WAVE_COUNT];
} drifterData;

#endif
