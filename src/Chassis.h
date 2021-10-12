#include <Arduino.h>
#include <RBE1001Lib.h>
#include <ESP32AnalogRead.h>
#include <ESP32Servo.h>
#include <ESP32Encoder.h>
#include <ESP32WifiManager.h>
#include <WebServer.h>
#include <ESP32PWM.h>

class Chassis
{

public:
    Chassis();
    Motor left, right;
    const float diameterWheel = 7.2;
    const float degreesPerCMForWheel = 360 / (3.14 * diameterWheel);

    void forward(float cm);
    void setDriveEffort(float, float);
    void driveFor(int setDistance);
};