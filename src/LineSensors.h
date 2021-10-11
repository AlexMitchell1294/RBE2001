#include <Arduino.h>
#include <RBE1001Lib.h>
#include <ESP32AnalogRead.h>
#include <ESP32Servo.h>
#include <ESP32Encoder.h>
#include <ESP32WifiManager.h>
#include <WebServer.h>
#include <ESP32PWM.h>

class LineSensors
{

public:
    ESP32AnalogRead leftLine;
    ESP32AnalogRead rightLine;
    ESP32AnalogRead leftTrigger;
    ESP32AnalogRead rightTrigger;

    LineSensors(/* args */);
    bool driveSensorsOnLine();
    float getError();
    float readLeft();
    float readRight();
    bool atCrossSection();
};