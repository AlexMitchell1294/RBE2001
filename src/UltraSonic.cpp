
#include <Arduino.h>
#include <RBE1001Lib.h>
#include <ESP32AnalogRead.h>
#include <ESP32Servo.h>
#include <ESP32Encoder.h>
#include <ESP32WifiManager.h>
#include <WebServer.h>
#include <ESP32PWM.h>

/**
 * Sets up rangefiner/ultrasonic sensor
 */
class UltraSonic
{
public:
  UltraSonic();
  Rangefinder rangefinder;

  float getDistance();
};

UltraSonic::UltraSonic()
{
}

/**
 * gets the distance in cm of an object
 */
float UltraSonic::getDistance()
{
  float distance;
  while ((distance = rangefinder.getDistanceCM()) <= 0.0)
  {
    delay(2000);
  }
  return distance;
}
