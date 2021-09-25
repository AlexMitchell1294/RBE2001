#pragma once

#include <Arduino.h>
#include <RBE1001Lib.h>
#include <ESP32AnalogRead.h>
#include <ESP32Servo.h>
#include <ESP32Encoder.h>
#include <ESP32WifiManager.h>
#include <WebServer.h>
#include <ESP32PWM.h>

class FourBar {
 public:
  void setEffort(int effort);
  long getPosition();
  void reset();
  void setup();
  //    static portMUX_TYPE mux;

 private:
  void setEffort(int effort, bool clockwise);
  const int PWM = 5;
  const int AIN2 = 23;
  const int AIN1 = 27;
  const int ENCA = 19;
  const int ENCB = 18;
};