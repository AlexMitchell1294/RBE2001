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
    float readLeft();
    float readRight();
    bool atCrossSection();
};

LineSensors::LineSensors(/* args */)
{
}


bool LineSensors::driveSensorsOnLine()
{
  return (leftLine.readVoltage() >= 2.5 || rightLine.readVoltage() >= 2.5) && (getError() <= 0.05 || getError() >= -0.05);
}

float LineSensors::getError(){
  return leftLine.readVoltage() - rightLine.readVoltage();
}

float LineSensors::readLeft(){
  return leftLine.readVoltage();
}

float LineSensors::readRight(){
  return leftLine.readVoltage();
}

bool LineSensors::atCrossSection(){
  return (readLeft() > 2 && readRight() > 2);
}


