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

const int degreeArm45 = 6630;
const int degreeArm25 = 8750;

class FourBar {
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
  const int AIN2 = 23;
  const int AIN1 = 27;
  const int ENCA = 19;
  const int ENCB = 18;

  const float kpA = 0.4;
  const float kiA = 0.00002;
  const float kdA = 0.0001;
};