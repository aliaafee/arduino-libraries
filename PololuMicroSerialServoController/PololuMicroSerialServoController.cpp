/*
  PololuMicroSerialServoController.cpp - Library for for interfacing 
  with Pololu Micro Serial Servo Controller via software serial 
  through the specified pins.

  Ali Aafee (7 Apri 2011)
  Released into the public domain
*/
#ifndef _PololuMicroSerialServoController_CPP
#define _PololuMicroSerialServoController_CPP

#include "Arduino.h"
#include "PololuMicroSerialServoController.h"

PololuMicroSerialServoController::PololuMicroSerialServoController(int rxpin, int txpin) 
{
  _rxpin = rxpin;
  _txpin = txpin;
}

void PololuMicroSerialServoController::begin()
{ 
  //We need to wait a while before we start communication with servo controller
  //Maybe this is a bug?
  delay(100);

  digitalWrite(_txpin, HIGH);
  
  pinMode(_rxpin, INPUT);
  pinMode(_txpin, OUTPUT);
  
  softSerial = new SoftwareSerial(_rxpin,_txpin);
  
  softSerial->begin(9600);
}

int PololuMicroSerialServoController::sendCommand(byte command, byte servo, byte data1, byte data2)
{
  /*available commands
    0x00 Set Parameters
    0x01 Set Speed
    0x02 Set Position 7-bit (1 data byte)
    0x03 Set Position 8-bit (2 data bytes)
    0x04 Set Position, Absolute (2 data bytes)
    0x05 Set Neutral (2 data bytes)
  */
  // Start Byte
  softSerial->write(0x80);
  // Device ID
  softSerial->write(0x01);
  // Command
  softSerial->write(command);
  // Servo number
  softSerial->write(servo);
  // First data byte
  softSerial->write(data1);
  // Second data byte
  softSerial->write(data2);
  
  return (0); 
}

int PololuMicroSerialServoController::sendCommand(byte command, byte servo, byte data1)
{
  // Start Byte
  softSerial->write(0x80);
  // Device ID
  softSerial->write(0x01);
  // Command
  softSerial->write(command);
  // Servo number
  softSerial->write(servo);
  // First data byte
  softSerial->write(data1);
  
  return (0);
}

int PololuMicroSerialServoController::setParameters(byte servo,byte on,byte rangeVal)
{
  byte temp;
  byte parameters;
/*
  temp = on << 6;			   //set first two bits of parameters (on = 1, off = 0)
  temp = temp + (rangeVal & 0x1f);   //put first five bits of rangeVal into temp
  parameters = temp & 0x7f;	    //take only bottom 7 bits
*/
  sendCommand(0x00,servo,parameters);
}

int PololuMicroSerialServoController::turnOn(byte servo)
{
  //turn on servo with default parameters
  setParameters(servo,1,15);
}

int PololuMicroSerialServoController::turnOff(byte servo)
{
  //turn on servo with default parameters
  setParameters(servo,0,15);
}

int PololuMicroSerialServoController::setSpeed(byte servo, byte speedVal)
{
  /*
    Speed 0 = no speed control
    Speed 1 = Slowest
    Speed 127 = fastest
  */
  //if (speed <0 || speed >127)
  //  return -1;
  
  speedVal = speedVal & 0x7f;
  
  sendCommand(0x01,servo,speedVal);
  
  return (0);
}

int PololuMicroSerialServoController::setPosition7bit(byte servo, byte position)
{
  byte pos = position & 0x7f;
  
  sendCommand(0x02,servo,pos);
  
  return (0);
}

int PololuMicroSerialServoController::setPosition8bit(byte servo, byte position)
{
  byte temp;
  byte pos_hi,pos_low;
  
  temp = position & 0x80;
  pos_hi = temp >> 7;
  pos_low = position & 0x7f;
  
  sendCommand(0x03,servo,pos_hi,pos_low);
  
  return (0);
}

int PololuMicroSerialServoController::setPositionAbsolute(byte servo, int position)
{
   unsigned int temp;
   byte pos_hi,pos_low;

   temp = position & 0x1f80;
   pos_hi = temp >> 7;
   pos_low = position & 0x7f;
   
   sendCommand(0x04,servo,pos_hi,pos_low);
}

int PololuMicroSerialServoController::setAngle(byte servo, float angle)
{
  byte data1 = 0;
  byte data2 = 0;
  int position = 0;
  
  // Check to make sure the servo is right
  if (servo <0 || servo >8)
    return -1;
 
  // Check to make sure position is within bounds
  if (angle < 0 || angle > 180)
    return -1;
  
  //Convert the angle to position(0 is 1100 and 180 is 4760 for my servo)
  position = map(angle,0,180, 1050, 4750);
  //Serial.print(position);
  
  // Calculate data bytes from position
  data2 = position & B01111111;
  data1 = position >> 7;
 
  // Command: 0x04 is set absolute position 
  sendCommand(0x04,servo,data1,data2);
 
  // Everything seems ok, return 0
  return (0);
}

int PololuMicroSerialServoController::setNeutral(byte servo, int position)
{
  byte data1 = 0;
  byte data2 = 0;

  // Calculate data bytes from position
  data2 = position & B01111111;
  data1 = position >> 7;

  sendCommand(0x05,servo,data1,data2);
}

char PololuMicroSerialServoController::getOutput()
{
  return softSerial->read();
}

#endif
