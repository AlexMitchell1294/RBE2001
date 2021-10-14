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
Timer timeToPrintA(100);
Timer timeToTurn(3000);

Robot robot;
FourBar blueMotor;
IRDecoder decoder(15);
uint16_t code;
RobotStates currentState = PAUSED;
RobotStates lastState = FOLLOWLINE;
RobotStates lastStateRun = currentState;
HelloDarknessMyOldFriend  stateC = PAUSEDD;
HelloDarknessMyOldFriend lastStateC = PAUSEDD;

double setpoint, inputValue, outputValue;
//PID pid(&inputValue, &outputValue, &setpoint, KpD, KiD, KdD, REVERSE);


//state varibles
int centerTracker;
int pickUPOrDropOff = 1;
int armEncoderSet = 0;
int setDistance = 15;
bool startOfCenter = true;
int driveDistanceArmSet = 15;
int driveDistanceArmPickup=10.75;
int moveOn = 0;
int dir = -1;
bool first = true;
int substate = 0;


void setup() {
  Serial.begin(9600);
  
  decoder.init();
  blueMotor.setup();
  robot.arm.armSetup();
  robot.ultrasonic.rangefinder.attach(SIDE_ULTRASONIC_TRIG, SIDE_ULTRASONIC_ECHO);
  digitalWrite(button, INPUT);
  robot.linesensors.leftLine.attach(39);
  robot.linesensors.rightLine.attach(36);
  robot.arm.armTurn(130);
}
//(nwePosition- old position) * 1000) * 60 /(100*CPR), cpr = counts per revolution = 270
int a = -255;
int b=0;
void loop()
{
  code = decoder.getKeyCode();
  if (code==remote1){
    lastState = currentState;
    currentState = PAUSED;
    lastStateC = stateC;
    stateC = PAUSEDD;
  }
  else if(code==remote2){ 
    currentState = lastState;
    stateC = lastStateC;
  }


  else if(code==remote5){
    blueMotor.setEffort(-255);
    stateC = EDITD;
  }
  else if(code==remote8){
    blueMotor.setEffort(0);
    stateC = EDITD;
  }
  else if(code==remote9){
    blueMotor.setEffort(255);
    stateC = EDITD;
  }
    else if(code==remoteLeft){
      currentState = PICKUPPLATE;
      stateC = PICKUP45D;
      armEncoderSet = degreeArm45;
  }
      else if(code==remoteRight){
      currentState = PICKUPPLATE;
      stateC = PICKUP25D;
      armEncoderSet = degreeArm25;
  }
      else if(code==remoteUp){
      currentState = PICKUPPLATE;
      stateC = PICKUPG;
      armEncoderSet = 0;
  }
      else if(code==remoteVolMinus){
      currentState = PICKUPPLATE;
      stateC = DROPOFF45;
      armEncoderSet = degreeArm45+2700;
  }
      else if(code==remotePlayPause){
      currentState = PICKUPPLATE;
      stateC = DROPOFF45;
      armEncoderSet = 800;
  }
      else if(code==remoteSetup){
      currentState = PICKUPPLATE;
      stateC = LINEFOLLOWD;
      armEncoderSet = 800;
  }
      else if(code==remoteEnterSave){
      currentState = PICKUPPLATE;
      stateC = CROSSOVER;
  }

  switch(stateC){

    case PAUSEDD:
      blueMotor.setEffort(0);
      robot.chassis.setDriveEffort(0,0);
      break;
    case PICKUP45D:
      switch(currentState){
        case PICKUPPLATE:
          if(timeToPrint.isExpired()) {
            if(timeToPrintA.isExpired()) printf("%d\n", blueMotor.getPosition());
            blueMotor.moveTo(armEncoderSet);
            int error = armEncoderSet -blueMotor.getPosition();
            if((error <= 25 && error >= -25)){
              blueMotor.setEffort(0);
              robot.arm.armTurn(180);
              armEncoderSet += 800;
              currentState = DRIVETOOBJECT;
            }
          }
          break;
      case DRIVETOOBJECT:
          if(timeToPrint.isExpired()) {
            robot.driveToObject(driveDistanceArmPickup);
          }
          if(robot.ultrasonic.getDistance() < 12.5) {
            robot.chassis.setDriveEffort(0,0);
            currentState = GRIP45;
            timeToTurn.reset();
          }  
          break;
      case GRIP45:
          blueMotor.moveTo(armEncoderSet);
          if(timeToTurn.isExpired()) first = false; 
          if(!first || timeToTurn.isExpired()){
            if ((armEncoderSet <= blueMotor.getPosition() +5 && armEncoderSet >= blueMotor.getPosition() -5)){
              robot.arm.armTurn(130);
              armEncoderSet += 2000;
              currentState = BACKUP;
              timeToTurn.reset();
              first = true;
            }
            else{
            first = false;
            }
          }
          break;
      case BACKUP:
          blueMotor.moveTo(armEncoderSet);
          if ((armEncoderSet <= blueMotor.getPosition() +5 && armEncoderSet >= blueMotor.getPosition() -5)){
            robot.chassis.forward(-15);
            stateC = PAUSEDD;
          }
          break;
      }
      break;

    case PICKUP25D:
      switch(currentState){
        case PICKUPPLATE:
          if(timeToPrint.isExpired()) {
            if(timeToPrintA.isExpired()) printf("%d\n", blueMotor.getPosition());
            blueMotor.moveTo(armEncoderSet);
            int error = armEncoderSet -blueMotor.getPosition();
            if((error <= 25 && error >= -25)){
              blueMotor.setEffort(0);
              robot.arm.armTurn(180);
              armEncoderSet += 1000;
              currentState = DRIVETOOBJECT;
            }
          }
          break;
      case DRIVETOOBJECT:
          if(timeToPrint.isExpired()) {
            robot.driveToObject(driveDistanceArmPickup-.5);
          }
          if(robot.ultrasonic.getDistance() < 12.5) {
            robot.chassis.setDriveEffort(0,0);
            currentState = GRIP45;
            timeToTurn.reset();
          }  
          break;
      case GRIP45:
          blueMotor.moveTo(armEncoderSet);
          if(timeToTurn.isExpired()) first = false; 
          if(!first || timeToTurn.isExpired()){
            if ((armEncoderSet <= blueMotor.getPosition() +5 && armEncoderSet >= blueMotor.getPosition() -5)){
              robot.arm.armTurn(130);
              armEncoderSet += 2000;
              currentState = BACKUP;
              timeToTurn.reset();
              first = true;
            }
            else{
            first = false;
            }
          }
          break;
      case BACKUP:
          blueMotor.moveTo(armEncoderSet);
          if ((armEncoderSet <= blueMotor.getPosition() +5 && armEncoderSet >= blueMotor.getPosition() -5)){
            robot.chassis.forward(-15);
            stateC = PAUSEDD;
            blueMotor.setEffort(0);
          }
          break;
      }
      break;

    case PICKUPG:
      switch(currentState){
        case PICKUPPLATE:
          if(timeToPrint.isExpired()) {
            if(timeToPrintA.isExpired()) printf("%d\n", blueMotor.getPosition());
            blueMotor.moveTo(0);
            int error = armEncoderSet -blueMotor.getPosition();
            if((error <= 25 && error >= -25)){
              blueMotor.setEffort(0);
              robot.arm.armTurn(180);
              armEncoderSet += 200;
              currentState = DRIVETOOBJECT;
            }
          }
          break;
      case DRIVETOOBJECT:
          if(timeToPrint.isExpired()) {
            robot.driveToObject(2.75);
          }
          if(robot.ultrasonic.getDistance() < 5) {
            robot.chassis.setDriveEffort(0,0);
            currentState = GRIP45;
            timeToTurn.reset();
          }  
          break;
      case GRIP45:
          blueMotor.moveTo(armEncoderSet);
          if(timeToTurn.isExpired()) first = false; 
          if(!first || timeToTurn.isExpired()){
            if ((armEncoderSet <= blueMotor.getPosition() +5 && armEncoderSet >= blueMotor.getPosition() -5)){
              robot.arm.armTurn(130);
              armEncoderSet += 800;
              currentState = BACKUP;
              timeToTurn.reset();
              first = true;
            }
            else{
            first = false;
            }
          }
          break;
      case BACKUP:
          blueMotor.moveTo(armEncoderSet);
          if ((armEncoderSet <= blueMotor.getPosition() +5 && armEncoderSet >= blueMotor.getPosition() -5)){
            robot.chassis.forward(-15);
            stateC = PAUSEDD;
            blueMotor.setEffort(0);
          }
          break;
      }
      break;

      case DROPOFF45:
        switch(currentState){
          case PICKUPPLATE:
            if(timeToPrint.isExpired()) {
              if(timeToPrintA.isExpired()) printf("%d\n", blueMotor.getPosition());
              blueMotor.moveTo(armEncoderSet);
              int error = armEncoderSet -blueMotor.getPosition();
              if((error <= 25 && error >= -25)){
                blueMotor.setEffort(0);
                robot.arm.armTurn(130);
                currentState = DRIVETOOBJECT;
              }
            }
            break;
        case DRIVETOOBJECT:
            if(timeToPrint.isExpired()) {
              robot.driveToObject(driveDistanceArmPickup);
            }
            if(robot.ultrasonic.getDistance() < 13) {
              robot.chassis.setDriveEffort(0,0);
              currentState = GRIP45;
              timeToTurn.reset();
            }  
            break;
        case GRIP45:
            blueMotor.moveTo(armEncoderSet);
            if(timeToTurn.isExpired()) first = false; 
            if(!first || timeToTurn.isExpired()){
              if ((armEncoderSet <= blueMotor.getPosition() +5 && armEncoderSet >= blueMotor.getPosition() -5)){
                robot.arm.armTurn(180);
                armEncoderSet -= 500;
                currentState = BACKUP;
                timeToTurn.reset();
                first = true;
              }
              else{
              first = false;
              }
            }
            break;
        case BACKUP:
            blueMotor.moveTo(armEncoderSet);
            if ((armEncoderSet <= blueMotor.getPosition() +5 && armEncoderSet >= blueMotor.getPosition() -5)){
              robot.chassis.forward(-15);
              stateC = PAUSEDD;
            }
            break;
        }
      break;

      case DROPOFFG:
        switch(currentState){
          case PICKUPPLATE:
            if(timeToPrint.isExpired()) {
              if(timeToPrintA.isExpired()) printf("%d\n", blueMotor.getPosition());
              blueMotor.moveTo(armEncoderSet);
              int error = armEncoderSet -blueMotor.getPosition();
              if((error <= 25 && error >= -25)){
                blueMotor.setEffort(0);
                robot.arm.armTurn(130);
                currentState = DRIVETOOBJECT;
              }
            }
            break;
        case DRIVETOOBJECT:
            if(timeToPrint.isExpired()) {
              robot.driveToObject(driveDistanceArmPickup);
            }
            if(robot.ultrasonic.getDistance() < 13) {
              robot.chassis.setDriveEffort(0,0);
              currentState = GRIP45;
              timeToTurn.reset();
            }  
            break;
        case GRIP45:
            blueMotor.moveTo(armEncoderSet);
            if(timeToTurn.isExpired()) first = false; 
            if(!first || timeToTurn.isExpired()){
              if ((armEncoderSet <= blueMotor.getPosition() +5 && armEncoderSet >= blueMotor.getPosition() -5)){
                robot.arm.armTurn(180);
                currentState = BACKUP;
                timeToTurn.reset();
                first = true;
              }
              else{
              first = false;
              }
            }
            break;
        case BACKUP:
            blueMotor.moveTo(armEncoderSet);
            if ((armEncoderSet <= blueMotor.getPosition() +5 && armEncoderSet >= blueMotor.getPosition() -5)){
              robot.chassis.forward(-15);
              stateC = PAUSEDD;
            }
            break;
        }
      break;

    case LINEFOLLOWD:
      robot.lineTracker();
      if(robot.linesensors.atCrossSection()){
        robot.centerOnCrossSection(-1);
      }

    case CROSSOVER:
      robot.chassis.setDriveEffort(0, -.3);
      if(robot.linesensors.driveSensorsOnLine()  && first== false){
        first = true;
        robot.chassis.setDriveEffort(0.3,0.3);
      }
      else if(robot.linesensors.driveSensorsOnLine()  && first== false){
        robot.chassis.setDriveEffort(0.0,0.0);
        robot.centerOnCrossSection(-1);
      }
    
    case EDITD:
        break;
      default:
        break;
    
  }
  

  // switch(currentState){
  //   case PAUSED:
  //       blueMotor.setEffort(0);
  //       robot.chassis.setDriveEffort(0,0);
  //       break;
  //   case EDIT:
  //       break;
  //   case FOLLOWLINE:
  //       robot.lineTracker();
  //       if(robot.linesensors.atCrossSection()){

  //         currentState = TURN;
  //         centerTracker = -1;
  //       }
  //       if(robot.ultrasonic.getDistance() <= 20){
  //         robot.chassis.setDriveEffort(0,0);
  //         if(pickUPOrDropOff == 1) {
  //           currentState = PICKUPPLATE;
  //           armEncoderSet = degreeArm45;
  //         }
  //         else {
  //           currentState = DROPOFFPLATE;
  //           armEncoderSet = degreeArm45;
  //         }
  //       }
  //       break;
  //   case TURN:
  //       if (first){
  //         robot.chassis.forward(8);
  //         first=false;
  //       }
  //       robot.turnWithLine(centerTracker);
  //       if (robot.linesensors.driveSensorsOnLine())
  //         {
  //          robot.chassis.setDriveEffort(-.6, .6);
  //          for(int i=0; i<=1000; i++);
  //         currentState = FOLLOWLINE;
  //         }

  //       break;
  //   case PICKUPPLATE:
  //       if(timeToPrint.isExpired()) {
  //         if(timeToPrintA.isExpired()) printf("%d\n", blueMotor.getPosition());
  //         blueMotor.moveTo(armEncoderSet);
  //         int error = armEncoderSet -blueMotor.getPosition();
  //         if((error <= 25 && error >= -25)){
  //           blueMotor.setEffort(0);
  //           robot.arm.armTurn(180);
  //           armEncoderSet += 800;
  //           currentState = DRIVETOOBJECT;
  //         }
  //       }
  //       break;
  //   case DRIVETOOBJECT:
  //       if(timeToPrint.isExpired()) {
  //         robot.driveToObject(driveDistanceArmPickup);
  //       }
  //       if(robot.ultrasonic.getDistance() < 11.5) {
  //         robot.chassis.setDriveEffort(0,0);
  //         currentState = GRIP45;
  //         timeToTurn.reset();
  //       }  
  //       break;
  //   case GRIP45:
  //       blueMotor.moveTo(armEncoderSet);
  //       if(timeToTurn.isExpired()) first = false; 
  //       if(!first || timeToTurn.isExpired()){
  //         if ((armEncoderSet <= blueMotor.getPosition() +5 && armEncoderSet >= blueMotor.getPosition() -5)){
  //           robot.arm.armTurn(130);
  //           armEncoderSet += 2000;
  //           currentState = AFTERPICKUPRESET;
  //           timeToTurn.reset();
  //           first = true;
  //         }
  //         else{
  //         first = false;
  //         }
  //       }
  //       break;
  //   case AFTERPICKUPRESET:
  //   if(timeToPrint.isExpired()){
  //     Serial.println(armEncoderSet);
  //   }
  //       blueMotor.moveTo(armEncoderSet);
  //       if(timeToTurn.isExpired()) first = false; 
  //       if(!first || timeToTurn.isExpired()){
  //         Serial.println(armEncoderSet);
  //         blueMotor.setEffort(0);
  //        currentState = TURN;
  //        first = false;
  //       }
  //       break;

  //   default:
  //     break;
  //   }
  // lastStateRun = currentState;
}