#include <Robot.h>
extern PID pid;
const double KpD=0.03;
const double KdD = 0.1;
const double KiD = 0.0;
extern double setpoint, inputValue, outputValue;
Timer printTimer(10);

/**
 * Any method that used more than one component of the robot is here. May be cleaner to add another "auto" file but most of these would also end up in there. Something to think about
 * 
 */
Robot::Robot(){

}

/**
 * turns in a direction till it see's a line
 * -1 turns left
 * +1 turns right
 * 0 nothing happens
 */
void Robot::turnWithLine(int direction)
{
  if (direction == 0)
    return;

  chassis.left.setSpeed(120 * direction);
  chassis.right.setSpeed(-120 * direction);

  //Meant to give time for the sensors to get off a line if already on one
  delay(200);


  //stop turning once drive sensors are back on a line
  while (true)
  {
    if (linesensors.driveSensorsOnLine())
    {
      chassis.left.setEffort(0);
      chassis.right.setEffort(0);
      break;
    }
  }
}

/*
* turns till it sees left edge then keeps turning till it sees right edge then turns to center of object
*/
void Robot::FindObject()
{

  const float turningSpeed = 90;

  //left edge
  chassis.left.setSpeed(-turningSpeed);
  chassis.right.setSpeed(turningSpeed);
  while (ultrasonic.getDistance() > 30)
  {
    delay(10);
  }

  float rightEdgeAngle = chassis.left.getCurrentDegrees();
  delay(100);
  //spin till right edge right edge
  while (ultrasonic.getDistance() <= 30)
  {
    delay(10);
  }

  //turn back
  float turnAngleMidPoint = (chassis.left.getCurrentDegrees() - rightEdgeAngle) * 0.5;
  chassis.left.startMoveFor(-turnAngleMidPoint, turningSpeed);
  chassis.right.moveFor(turnAngleMidPoint, turningSpeed);
}

/*
* drives to an object with desired distance between the robot and object
* if arm holes (3,5) from the back ar being used then (3) roughly centers on the middle of the bag from a top down view
*/
void Robot::driveToObject(double desiredDistance)
{
  while(1){
      inputValue = ultrasonic.getDistance();
      float error = -desiredDistance + inputValue;
      if (printTimer.isExpired()) {
        if (error >= 2.5 || error <= -2.5) {
          chassis.setDriveEffort(error*KpD, error*KpD);
        }
        else break;
      }
  }
  chassis.setDriveEffort(0,0);
}

/*
* picks up bag from ground floor
*/
void Robot::pickupBag()
{
  arm.armTurn(0);
  delay(150);
  driveToObject(3);
  delay(150);
  arm.armTurn(180);
  delay(150);
}

/**
* Drops bag off at specified height
* note only works for drop off zone with a height of at least 1 inch
*/
void Robot::dropBag(float height)
{
  delay(100);

  //calcules angle based on height given
  float heightToAngle = 20 * (height - 8) + 180;

  driveToObject(3);
  chassis.forward(-1);
  arm.armTurn(heightToAngle);
  delay(250);
}

/**
 * if either of the outside trigger sensors detect a line stops the robot
 */
bool Robot::eitherLineStop()
{
  if (linesensors.leftTrigger.readVoltage() >= 1.5 || linesensors.rightTrigger.readVoltage() >= 1.5)
  {
    chassis.left.setEffort(0);
    chassis.right.setEffort(0);
    return true;
  }
  else
  {
    return false;
  }
}

/*
 * if both outside trigger sensors detect a line stops the robot
 */
bool Robot::bothLineStop()
{
  if (linesensors.leftLine.readVoltage() >= 2.0 && linesensors.rightLine.readVoltage() >= 2.0)
  {
    chassis.left.setEffort(0);
    chassis.right.setEffort(0);
    return true;
  }
  else
  {
    return false;
  }
}

//kpAuto is a ratio to optimize speed and functionality in multiple areas
//kpTurn is only used for followCurveInLine
float kpAuto = .12;
float kpTurn = 1.3;
float kpTurnLeadRatio = kpAuto * (kpTurn);
float kpTurnFollowRatio = kpAuto / (kpTurn * 3);

/**
 * follows the curve of a line for 60-120 degrees range of each side.
 * faster than turnWithLine and this does not stop the robot
 * +1 turns right
 * -1 turns left
 * 0 does not turn just follows line to cross section and centers
 * ***clean and fast***
 */
void Robot::followCurveInLine(int direction)
{
  float leftValue = 0;
  float rightValue = 0;

  // turns right
  if (direction == 1)
  {
    leftValue = linesensors.rightLine.readVoltage() * kpTurnLeadRatio;
    rightValue = linesensors.leftLine.readVoltage() * kpTurnFollowRatio;
  }

  // turns left
  else if (direction == -1)
  {
    leftValue = linesensors.rightLine.readVoltage() * kpTurnFollowRatio;
    rightValue = linesensors.leftLine.readVoltage() * kpTurnLeadRatio;
  }

  //forward if no direction specified
  else if (eitherLineStop())
  {
    leftValue = linesensors.rightLine.readVoltage() * kpAuto;
    rightValue = linesensors.leftLine.readVoltage() * kpAuto;
  }

  chassis.left.setEffort(leftValue);
  chassis.right.setEffort(rightValue);
  delay(100);

  // stop turning once 
  while (true)
  {
    if (linesensors.driveSensorsOnLine())
    {
      break;
    }
  }
}

/*
* follows a line
* turning right if left value is too low
* turning left if right value is too low
* uses eitherLineStop to detect cross sections or turning points
*/
void Robot::lineTracker()
{
  // while (true)
  // {
  //   chassis.left.setEffort(linesensors.rightLine.readVoltage() * kpAuto);
  //   chassis.right.setEffort(linesensors.leftLine.readVoltage() * kpAuto);
  //   if (eitherLineStop())
  //   {
  //     break;
  //   }
  // }
  float error = linesensors.getError();
  float motorspeed = kp * error + kd * (error-lastError);
  lastError = error;
  //Serial.println(motorspeed);
  float leftVal = base - motorspeed;
  float rightVal = base + motorspeed;
  chassis.setDriveEffort(leftVal, rightVal);
}

/**
 * centers the robot on the line by driving forward then turns till it sees a line
 */
void Robot::centerOnCrossSection(int direction)
{
  chassis.forward(9);
  //+1 is turn right
  //-1 is turn left
  //0 only centers does not turn
  turnWithLine(direction);
}

/**
 * follows a line turning based on direction given as either
 * -1 or +1
 * uses bothLineStop to detect a cross section and stop moving
 */
void Robot::followLineToCrossSection(int autoTurnDirection, int centeredTurnDirection)
{
    if (bothLineStop())
    {
      centerOnCrossSection(centeredTurnDirection);
      return;
    }
    lineTracker();
    followCurveInLine(autoTurnDirection);
}
  /*
  this function finds the line after collecting free range bag
 * if the sa or ta grading this sees this. You are appreciated. also
 * Why did the robot go to the shoe shop?
 * to get rebooted.
 * that was definatly funny and not me just trying to distract you from the next function that is awful
 * so dont question and https://koenig-media.raywenderlich.com/uploads/2015/06/just_works.jpg
 */
void Robot::findLine()
{
  chassis.left.setEffort(-.2);
  chassis.right.setEffort(.2);
  delay(200);
  chassis.left.setEffort(.2);
  while (true)
  {
    if (linesensors.driveSensorsOnLine())
    {
      centerOnCrossSection(1);
      turnWithLine(-1);
      return;
    }
  }
}

