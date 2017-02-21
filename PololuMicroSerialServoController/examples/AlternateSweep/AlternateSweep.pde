#include <SoftwareSerial.h>

#include <PololuMicroSerialServoController.h>

#define RXPIN 6
#define TXPIN 8

PololuMicroSerialServoController servo(RXPIN,TXPIN);

void setup()
{
  //Initialize and set servospeed
  servo.begin();
  servo.setSpeed(1,30);
  servo.setSpeed(2,15);
}

void loop()
{
  servo.setAngle(1,0);
  servo.setAngle(2,180);
  
  delay (2000);
  
  servo.setAngle(1,180);
  servo.setAngle(2,0);
  
  delay (2000);
}
