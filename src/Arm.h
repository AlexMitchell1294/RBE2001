#pragma once

#include <Arduino.h>
#include <RBE1001Lib.h>
#include <ESP32AnalogRead.h>
#include <ESP32Servo.h>
#include <ESP32Encoder.h>
#include <ESP32WifiManager.h>
#include <WebServer.h>
#include <ESP32PWM.h>


class Arm
{

public:
    Arm(/* args */);
    Servo armServo;

    void armTurn(float angle);
};