/*
 * VNH7100AS motor driver library
 * 
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public 
 * License as published by the Free Software Foundation; either  
 * version 2.1 of the License, or (at your option) any later version.
 *   
 * Created 2 June 2019 Bart Mellink
 * Modified 23 April 2021 Pascal Goldbrunner
 */

#include <Arduino.h>
#include <VNH7100AS.h>

#ifdef ARDUINO_ARCH_ESP32
#include <analogWrite.h>
#endif

void VNH7100AS::begin(int8_t pwmPin, int8_t inaPin, int8_t inbPin, int8_t sel0Pin, int8_t csPin)
{
  this->_pwmPin = pwmPin;
  this->_inaPin = inaPin;
  this->_inbPin = inbPin;
  this->_sel0Pin = sel0Pin;
  this->_csPin = csPin;
  if (pwmPin > 0)
    pinMode(pwmPin, OUTPUT);
  if (inaPin > 0)
    pinMode(inaPin, OUTPUT);
  if (inbPin > 0)
    pinMode(inbPin, OUTPUT);
  if (sel0Pin > 0 && csPin > 0)
  {
    pinMode(sel0Pin, OUTPUT);
    pinMode(csPin, INPUT); // analog input
  }
  else
  {
    this->_sel0Pin = -1;
    this->_csPin = -1;
  }
  this->setSpeed(0);
}

uint8_t VNH7100AS::setSpeed(int speed)
{
  // Ensure we do not reverse the ina and inb setting in case speed==0 to guarantee the motor
  // will free run to a stop (if you reverse ina and inb setting the controller will issue a full brake)
  if (speed > 0 || (speed == 0 && this->forward))
  {
    digitalWrite(this->_inaPin, HIGH);
    digitalWrite(this->_inbPin, LOW);
    this->forward = true;
  }
  else if (speed < 0)
  {
    digitalWrite(this->_inaPin, LOW);
    digitalWrite(this->_inbPin, HIGH);
    speed = -speed;
    this->forward = false;
  }
  if (speed > 400)
    speed = 400;
  this->speed = (this->forward ? speed : -speed);
  delayMicroseconds(20); // to wake from standby wait 20us after setting INA/INB for PWM. Just do it always
  if (this->digital)
  {
    digitalWrite(this->_pwmPin, (speed == 0) ? LOW : HIGH);
  }
  else
  {
    analogWrite(this->_pwmPin, speed * 51 / 80); // map 400 to 255 and generate pwm
  }
  return !this->isFault();
}

uint8_t VNH7100AS::brake(int brakePower)
{
  if (brakePower < 0)
    brakePower = 0;
  if (brakePower > 400)
    brakePower = 400;
  digitalWrite(this->_inaPin, LOW);
  digitalWrite(this->_inbPin, LOW);
  delayMicroseconds(20); // to wake from standby wait 20us after setting INA/INB for PWM. Just do it always
  if (this->digital)
  {
    digitalWrite(this->_pwmPin, (speed == 0) ? LOW : HIGH);
  }
  else
  {
    analogWrite(this->_pwmPin, brakePower * 51 / 80); // map 400 to 255
  }
  this->speed = 0;
  return !this->isFault();
}

uint8_t VNH7100AS::isFault()
{
  if (this->_sel0Pin <= 0)
    return false;
  uint8_t isfault = analogRead(this->_csPin) >= 1000;
  if (isfault)
  {
    brake(0);
  }
  return isfault;
}

int VNH7100AS::motorCurrent()
{
  if (this->forward) // INA = HIGH, INB = LOW
    digitalWrite(this->_sel0Pin, HIGH);
  else // INA = LOW, INB = HIGH
    digitalWrite(this->_sel0Pin, LOW);

  if (this->_csPin < 0)
    return 0;
  return analogRead(this->_csPin);
}

void setDigital(uint8_t isDigital)
{
  this->digital = isDigital;
}
