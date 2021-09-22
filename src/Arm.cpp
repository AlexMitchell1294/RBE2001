#include <Arduino.h>
#include <RBE1001Lib.h>
#include <ESP32AnalogRead.h>
#include <ESP32Servo.h>
#include <ESP32Encoder.h>
#include <ESP32WifiManager.h>
#include <WebServer.h>
#include <ESP32PWM.h>

/**
 * Sets up the servo and arm turn methods. arm is not used much in this project.
 */ 
class Arm
{
    
public:
    Arm(/* args */);
    Servo armServo;
    void armTurn(float angle);
};

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