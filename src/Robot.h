
#include <Arduino.h>
#include <RBE1001Lib.h>
#include <ESP32AnalogRead.h>
#include <ESP32Servo.h>
#include <ESP32Encoder.h>
#include <ESP32WifiManager.h>
#include <WebServer.h>
#include <ESP32PWM.h>

#include <Chassis.h>
#include <LineSensors.h>
#include <Arm.h>
#include <UltraSonic.h>

class Robot
{

public:
  Robot(/* args */);
  Chassis chassis;
  LineSensors linesensors;
  UltraSonic ultrasonic;
  Arm arm;

  void turnWithLine(int direction);
  void FindObject();
  void driveToObject(float desiredDistance);
  void pickupBag();
  void dropBag(float height);
  void followCurveInLine(int direction);
  void lineTracker();
  void centerOnCrossSection(int direction);
  void followLineToCrossSection(int autoTurnDirection, int centeredTurnDirection);
  void findLine();

  bool eitherLineStop();
  bool bothLineStop();
  
};
