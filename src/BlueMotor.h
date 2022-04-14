#pragma once

#include <Arduino.h>
#include <RBE1001Lib.h>
#include <ESP32AnalogRead.h>
#include <ESP32Servo.h>
#include <ESP32Encoder.h>
#include <ESP32WifiManager.h>
#include <WebServer.h>
#include <ESP32PWM.h>
#include <Timer.h>

const int degreeArm45 = 6550;
const int degreeArm25 = 9000;

class BlueMotor {
 public:
  void setEffort(int effort);
  long getPosition();
  void reset();
  void setup();
  void moveTo(int pos);
  float setEffortWihtoutDB(int);
  void setEffort(int effort, bool clockwise);
  //    static portMUX_TYPE mux;

 private:
  const int PWM = 5;
  const int AIN2 = 22;
  const int AIN1 = 23;
  const int ENCA = 18;
  const int ENCB = 19;

  const float kpA = 0.4;
  const float kiA = 0.00000;
  const float kdA = 0.0000;
};