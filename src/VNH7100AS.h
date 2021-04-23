
/*
 * VNH7100AS Arduino motor driver library
 * 
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public 
 * License as published by the Free Software Foundation; either  
 * version 2.1 of the License, or (at your option) any later version.
 *   
 * Created 2 June 2019 Bart Mellink
 * Modified 23 April 2021 Pascal Goldbrunner
 */
#ifndef VNH7100AS_h
#define VNH7100AS_h

#include <inttypes.h>


class VNH7100AS {

public:
  void begin(int8_t pwmPin=-1, int8_t inaPin=-1, int8_t inbPin=-1, int8_t sel0Pin=-1, int8_t csPin=-1);
  uint8_t setSpeed(int speed); 
  uint8_t brake(int brakePower);
  uint8_t isFault(); 
  int motorCurrent();
  int speed = 0;

private:
  uint8_t _pwmPin = -1;
  uint8_t _inaPin = -1;
  uint8_t _inbPin = -1;
  uint8_t _sel0Pin = -1;
  uint8_t _csPin = -1;
  uint8_t forward = true; // last speed command is forward
};

#endif
