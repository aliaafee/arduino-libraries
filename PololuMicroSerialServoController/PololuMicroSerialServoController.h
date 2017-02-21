/*
  PololuMicroSerialServoController.h - Library for for interfacing 
  with Pololu Micro Serial Servo Controller via software serial 
  through the specified pins.

  Ali Aafee (7 Apri 2011)
  Released into the public domain
*/
#ifndef _PololuMicroSerialServoController_H
#define _PololuMicroSerialServoController_H

//#include "Arduino.h"
#include <SoftwareSerial.h>

class PololuMicroSerialServoController
{
  public:
    PololuMicroSerialServoController(int rxpin, int txpin);
    void begin();
    int setParameters(byte servo,byte on,byte rangeVal);
    int turnOn(byte servo);
    int turnOff(byte servo);
    int setSpeed(byte servo, byte speedVal);
    int setPosition7bit(byte servo, byte position);
    int setPosition8bit(byte servo, byte position);
    int setPositionAbsolute(byte servo, int position);
    int setAngle(byte servo, float angle);
    int setNeutral(byte servo, int position);
    char getOutput();
    int _rxpin;
    int _txpin;
  private:
    SoftwareSerial * softSerial;
    int sendCommand(byte command, byte servo, byte data1, byte data2);
    int sendCommand(byte command, byte servo, byte data1);
};

#endif
