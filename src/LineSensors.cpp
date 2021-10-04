#include <Arduino.h>
#include <RBE1001Lib.h>
#include <ESP32AnalogRead.h>
#include <ESP32Servo.h>
#include <ESP32Encoder.h>
#include <ESP32WifiManager.h>
#include <WebServer.h>
#include <ESP32PWM.h>

/**
 * Initializes all four line sensors as well as driveSensorsOnLine
 */
class LineSensors
{
private:
    //leftLine and rightLine are used for traveling across the line proportionally
    // left trigger and right trigger are used for cross sections and turns.
    ESP32AnalogRead leftLine;
    ESP32AnalogRead rightLine;
    ESP32AnalogRead leftTrigger;
    ESP32AnalogRead rightTrigger;

public:
    LineSensors(/* args */);
    bool driveSensorsOnLine();
    float getError();
};

LineSensors::LineSensors(/* args */)
{
}

bool LineSensors::driveSensorsOnLine()
{
  return leftLine.readVoltage() >= 1.5 && rightLine.readVoltage() >= 1.5;
}

float LineSensors::getError(){
  return leftLine.readVoltage() - rightLine.readVoltage();
}
