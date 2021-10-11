// /*
// Alex Mitchell, Final Project RBE 1001, 12/6/2020
//  */

#include <Arduino.h>
#include <RBE1001Lib.h>
#include <ESP32AnalogRead.h>
#include <ESP32Servo.h>
#include <ESP32Encoder.h>
#include <ESP32WifiManager.h>
#include <WebServer.h>
#include <ESP32PWM.h>

#include "FourBar.h"
#include "Robot.h"
#include "IRdecoder.h"
#include "RemoteConstants.h"
#include <PID_v1.h>
#include "States.h"

const int button = 13;
long Count = 0;
//long timeToPrint = 0;
long now = 0;
long newPosition = 0;
long oldPosition = 0;
long sampleTime = 100;
int speedInRPM = 0;
int CPR = 270;

int armP = 90;
Timer timeToPrint(10);

Robot robot;
FourBar blueMotor;
IRDecoder decoder(15);
uint16_t code;
RobotStates currentState = PAUSED;
RobotStates lastState = FOLLOWLINE;

double setpoint, inputValue, outputValue;
//PID pid(&inputValue, &outputValue, &setpoint, KpD, KiD, KdD, REVERSE);


//state varibles
int centerTracker;
int pickUPOrDropOff = 1;
int armEncoderSet = 0;
int setDistance = 8;


void setup() {
  Serial.begin(9600);
  
  decoder.init();
  blueMotor.setup();
  robot.arm.armSetup();
  robot.ultrasonic.rangefinder.attach(SIDE_ULTRASONIC_TRIG, SIDE_ULTRASONIC_ECHO);
  digitalWrite(button, INPUT);
  robot.linesensors.leftLine.attach(39);
  robot.linesensors.rightLine.attach(36);
}
//(nwePosition- old position) * 1000) * 60 /(100*CPR), cpr = counts per revolution = 270
int a = -255;
int b=0;
void loop()
{
  code = decoder.getKeyCode();
  if (code==1){
    lastState = currentState;
    currentState = PAUSED;
  }
  else if(code==2) currentState = lastState;

  switch(currentState){
    case PAUSED:
        break;
    case FOLLOWLINE:
        robot.lineTracker();
        if(robot.linesensors.atCrossSection()){
          currentState = TURN;
          centerTracker = -1;
        }
        else if(robot.ultrasonic.getDistance() < 15){
          if(pickUPOrDropOff == 1) currentState = PICKUPPLATE;
          else currentState = DROPOFFPLATE;
        }
        break;
    case TURN:
        robot.centerOnCrossSection(centerTracker);

        if (robot.linesensors.driveSensorsOnLine())
          {
            robot.chassis.left.setEffort(0);
            robot.chassis.right.setEffort(0);
            currentState = FOLLOWLINE;
          }

        break;
    case PICKUPPLATE:
        blueMotor.moveTo(armEncoderSet);
        if((armEncoderSet == blueMotor.getPosition() +5 || armEncoderSet == blueMotor.getPosition() -5) 
        && robot.ultrasonic.getDistance() == setDistance){
          armEncoderSet += 100;
          currentState = GRIP45;
        }
        break;
    case GRIP45:
        blueMotor.moveTo(armEncoderSet);
        if ((armEncoderSet == blueMotor.getPosition() +5 || armEncoderSet == blueMotor.getPosition() -5)){
          robot.arm.armTurn(45);
          armEncoderSet += 1000;
          currentState = AFTERPICKUPRESET;
        }
        break;
    case AFTERPICKUPRESET:
        break;
  }
}