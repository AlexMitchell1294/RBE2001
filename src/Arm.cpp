#include <Arduino.h>
#include <RBE1001Lib.h>
#include <ESP32AnalogRead.h>
#include <ESP32Servo.h>
#include <ESP32Encoder.h>
#include <ESP32WifiManager.h>
#include <WebServer.h>
#include <ESP32PWM.h>
#include "Arm.h"

Arm::Arm(/* args */)
{
}

/*
* turns arm to specified angle
*/
void Arm::armTurn(float angle)
{
  armServo.write(angle);
}

void Arm::armSetup(){
  armServo.attach(SERVO_PIN);
}