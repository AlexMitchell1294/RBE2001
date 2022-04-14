#include <Arduino.h>
#include "RBE1001Lib.h"
#include "BlueMotor.h"
#include "CServo.h"
#include "IRdecoder.h"
#include "RemoteConstants.h"
#include "UltraSonic.h"
#include "Chassis.h"
#include "LineSensors.h"

BlueMotor BM;
Servo ContServo;
IRDecoder decoder(15);
uint16_t Code;
UltraSonic US;
Chassis Bot;
LineSensors LS;
Timer timerWrist(6900);

float kp = .01;
float kd = .008; 
float lasterror = 0;
float base = .15;

void setup() {
  Serial.begin(9600);
  BM.setup();
  ContServo.attach(33);
  decoder.init();
  US.rangefinder.attach(16,17);
  Bot.left.attach();
  Bot.right.attach();
  LS.rightLine.attach(39);
  LS.leftLine.attach(36);
}

void drivetoobject(int Dist){
  while (US.getDistance() > Dist)
  {
    int CurrentD = US.getDistance();
    Serial.println(US.getDistance());
    Bot.setDriveEffort(CurrentD * .01, CurrentD * .011);
  }
  Bot.setDriveEffort(0, 0);
}

void FollowTheDamnLine(){
  float error = LS.getError();
  float motorspeed = kp * error + kd * (error-lasterror);
  lasterror = error;
  float leftVal = base - motorspeed;
  float rightVal = base + motorspeed;
  // Serial.println(leftVal);
  // Serial.println(rightVal);
  Bot.setDriveEffort(leftVal, rightVal);
}

void turnWithLine(int direction)
{
  Bot.left.setSpeed(120 * direction);
  Bot.right.setSpeed(-120 * direction);
  delay(500);
  if (direction == 0)
  return;
  while (true){
    if (LS.driveSensorsOnLine())
    {
      delay(190);
      Bot.setDriveEffort(0,0);
      break;
    }
  }
}

void centerOnCrossSection(int direction)
{
  Bot.forward(8);
  //+1 is turn right
  //-1 is turn left
  //0 only centers does not turn
  turnWithLine(direction);
}

void place25(){
  uint16_t code =0;

  BM.moveTo(9200);//move arm to 25degree
  BM.setEffort(0);
  Bot.forward(16.5);
  Bot.setDriveEffort(0,0);
  BM.moveTo(7300);
  BM.setEffort(0);
  ContServo.write(180);//go down
  timerWrist.reset();
  while(!timerWrist.isExpired()){
  }
  ContServo.write(98);

  code = 0;
  while(code!=remotePlayPause){
    code = decoder.getKeyCode();
  }

  BM.moveTo(7500);
  BM.setEffort(0);
  while(code!=remotePlayPause){
    code = decoder.getKeyCode();
  }
  Bot.forward(-5);
  turnWithLine(-1);
}

void place45(){
  uint16_t code;
  BM.moveTo(9500);//move arm to 45degree
  BM.setEffort(0);
  while(US.getDistance() > 10){
    FollowTheDamnLine();
  }
  drivetoobject(7);//drive to 45
  BM.moveTo(5450);
  BM.setEffort(0);
  ContServo.write(180);//go down
  timerWrist.reset();
  while(!timerWrist.isExpired()){
  }
  ContServo.write(98);
  while(code!=remotePlayPause){
    code = decoder.getKeyCode();
  }
  BM.moveTo(4600); // get past screws 45
  BM.setEffort(0);
  turnWithLine(-1);
}

void depresso(){
  uint16_t code;
  while(!LS.driveSensorsOnLine()){
    FollowTheDamnLine();
  }
  centerOnCrossSection(-1);
  BM.moveTo(4600);//move arm to 45degree
  BM.setEffort(0);
    while(US.getDistance() > 10){
    FollowTheDamnLine();
  }
  drivetoobject(7);//drive to 45
  BM.moveTo(5450);
  BM.setEffort(0);
  ContServo.write(0);//go down
  timerWrist.reset();
  while(!timerWrist.isExpired()){
  }
  ContServo.write(98);
  while(code!=remotePlayPause){
    code = decoder.getKeyCode();
  }
  BM.moveTo(9500); // get past screws 45
  BM.setEffort(0);
  turnWithLine(-1);
  while(!LS.driveSensorsOnLine()){
    FollowTheDamnLine();
  }
  centerOnCrossSection(-1);
  while(US.getDistance() > 3){ // Move to platform
    FollowTheDamnLine();
  }
  Bot.forward(2.5);
  Bot.setDriveEffort(0,0);
  BM.moveTo(0); // arm to platform
  BM.setEffort(0);

  ContServo.write(180);//go down
  timerWrist.reset();
  while(!timerWrist.isExpired()){

  }
  ContServo.write(98);

code = 0;
  while(code!=remotePlayPause){
    code = decoder.getKeyCode();
  }

  Bot.forward(-5);
  Bot.left.startMoveFor(-180, 180);
  Bot.right.moveFor(180, 180);

  while(!LS.driveSensorsOnLine()){
    Bot.setDriveEffort(0.1, 0.100001);
  }

  centerOnCrossSection(-1);

  while(!LS.driveSensorsOnLine()){ // Move to platform
    FollowTheDamnLine();
  }
  centerOnCrossSection(-1);

  while(US.getDistance() > 13){ // Move to platform
    FollowTheDamnLine();
  }
  Bot.setDriveEffort(0,0);

  BM.moveTo(6200);//move arm to 25degree
  BM.setEffort(0);
    while(US.getDistance() > 14){
    FollowTheDamnLine();
  }
  drivetoobject(9);//drive to 45
  BM.moveTo(7200);
  BM.setEffort(0);
  ContServo.write(0);//go down
  timerWrist.reset();
  while(!timerWrist.isExpired()){
  }
  ContServo.write(98);

  code = 0;
  while(code!=remotePlayPause){
    code = decoder.getKeyCode();
  }

  BM.moveTo(9200);
  BM.setEffort(0);
  while(code!=remotePlayPause){
    code = decoder.getKeyCode();
  }

  turnWithLine(-1);



}

void loop() {
 Code = decoder.getKeyCode();
 if (Code == remote1)
 {
    Serial.println("Arm Down");
    BM.setEffort(255, true);
 }
else if (Code == remote2)
 {
    Serial.println("Arm Up");
    BM.setEffort(255, false);
 }
else if (Code == remote3)
 {
    Serial.println("Arm Stop");
    BM.setEffort(0, false);
 }
else if (Code == remote4)
 {
       ContServo.write(180);
 }
else if (Code == remote5)
{
    ContServo.write(0);
}
else if (Code == remote6)
{
    ContServo.write(98);
}
else if (Code == remote7)
{
    Serial.println("FOLLOW LINE");
  while(!LS.driveSensorsOnLine()){
    FollowTheDamnLine();
  }
  Serial.println("EXIT FOLLOW LINE");
  centerOnCrossSection(-1);
}
else if(Code == remote8)
{
    Serial.print(BM.getPosition());
}
else if(Code == remote9)
{
   Bot.setDriveEffort(0,0);
}
else if(Code == remotePlayPause)
{
   depresso();
}
else if(Code == remoteSetup)
{
   place45();
}
else if(Code == remoteStopMode)
{
   place25();
}
}




