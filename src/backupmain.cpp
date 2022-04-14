// /*
// Alex Mitchell, Final Project RBE 1001, 12/6/2020
// code before being converted to OOD
//  */

// #include <Arduino.h>
// #include <RBE1001Lib.h>
// #include <ESP32AnalogRead.h>
// #include <ESP32Servo.h>
// #include <ESP32Encoder.h>
// #include <ESP32WifiManager.h>
// #include <WebServer.h>
// #include <ESP32PWM.h>

// Motor left;
// Motor right;
// Rangefinder rangefinder;
// ESP32AnalogRead leftLine;
// ESP32AnalogRead rightLine;
// ESP32AnalogRead leftTrigger;
// ESP32AnalogRead rightTrigger;
// Servo armServo;

// void setup()
// {
//   Serial.begin(9600);
//   // ESP32PWM::allocateTimer(1);
//   // Motor::allocateTimer(0);
//   // left.attach(MOTOR_LEFT_PWM, MOTOR_LEFT_DIR, MOTOR_LEFT_ENCA,
//   //             MOTOR_LEFT_ENCB);
//   // right.attach(MOTOR_RIGHT_PWM, MOTOR_RIGHT_DIR,
//   //              MOTOR_RIGHT_ENCA, MOTOR_RIGHT_ENCB);
//   rangefinder.attach(SIDE_ULTRASONIC_TRIG, SIDE_ULTRASONIC_ECHO);

//   //leftLine is pin 7
//   //rightLine is pin 5
//   //leftTrigger is pin 1
//   //rightTrigger is pin 11
//   // leftLine.attach(39);
//   // rightLine.attach(36);
//   // leftTrigger.attach(35);
//   // rightTrigger.attach(34);
//   // armServo.attach(SERVO_PIN);
// }

// //const used for forward and turn
// const float diameterWheel = 7.2;
// const float degreesPerCMForWheel = 360 / (3.14 * diameterWheel);
// const float robotDistanceBetweenWheels = 14.2;
// const float robotTurnDegrees = 360 / (3.14 * robotDistanceBetweenWheels);

// /**
//  * drives forward a given amount of cm
//  */
// void forward(float cm)
// {
//   float degree = cm * degreesPerCMForWheel;
//   left.startMoveFor(degree, 360);
//   right.moveFor(degree, 360);
// }

// /**
//  * helper function
//  * detects if drive sensors are on a ling
//  */
// bool driveSensorsOnLine()
// {
//   return leftLine.readVoltage() >= 1.5 && rightLine.readVoltage() >= 1.5;
// }

// /**
//  * turns in a direction till it see's a line
//  * -1 turns left
//  * +1 turns right
//  * 0 nothing happens
//  * ***needs to be implementated will work better than current configureation***
//  * *** Untested ***
//  */
// void turnWithline(int direction)
// {
//   if (direction == 0)
//     return;

//   left.setSpeed(120 * direction);
//   right.setSpeed(-120 * direction);

//   //***delay untested needs tuning***
//   //Meant to give time for the sensors to get off a line if already on one
//   delay(200);

//   while (true)
//   {
//     if (driveSensorsOnLine())
//     {
//       left.setEffort(0);
//       right.setEffort(0);
//       break;
//     }
//   }
// }

// /**
//  * gets the distance in cm of an object
//  */
// float getDistance()
// {
//   float distance;
//   while ((distance = rangefinder.getDistanceCM()) <= 0.0)
//   {
//     delay(2000);
//   }
//   return distance;
// }

// const float turningSpeed = 90;

// /*
// * turns till it sees left edge then keeps turning till it sees right edge then turns to center
// */
// void FindObject()
// {
//   left.setSpeed(-turningSpeed);
//   right.setSpeed(turningSpeed);
//   while (getDistance() > 30)
//   {
//     delay(10);
//   }

//   float rightEdgeAngle = left.getCurrentDegrees();
//   delay(100);
//   while (getDistance() <= 30)
//   {
//     delay(10);
//   }

//   float turnAngleMidPoint = (left.getCurrentDegrees() - rightEdgeAngle) * 0.5;
//   left.startMoveFor(-turnAngleMidPoint, turningSpeed);
//   right.moveFor(turnAngleMidPoint, turningSpeed);
// }

// /*
// * drives to an object with desired distance between the robot and object
// * if arm holes (3,5) from the back ar being used then (4) roughly centers on the middle of the bag from a top down view
// * ***change to proportional control***
// */
// void driveToObject(float desiredDistance)
// {
//   while (getDistance() > desiredDistance)
//   {
//     left.setSpeed(-160);
//     right.setSpeed(-160);
//     delay(100);
//   }

//   left.setSpeed(0);
//   right.setSpeed(0);
// }

// /*
// * turns arm to specified angle
// */
// void armTurn(float angle)
// {
//   armServo.write(angle);
// }

// /*
// * picks up bag from ground floor
// * **delay untested**
// */
// void pickupBag()
// {
//   armTurn(0);
//   delay(150);
//   driveToObject(3);
//   delay(150);
//   armTurn(180);
//   delay(150);
// }

// /**
// * Drops bag off at specified height
// * *** untested ***
// */
// void dropBag(float height)
// {
//   delay(100);
//   //calcules angle based on height given
//   float heightToAngle = 20 * (height - 8) + 180;
//   driveToObject(3);
//   forward(-1);
//   armTurn(heightToAngle);
//   delay(250);
// }

// /**
//  * if either of the outside trigger sensors detect a line stops the robot
//  */
// bool eitherLineStop()
// {
//   if (leftTrigger.readVoltage() >= 1.5 || rightTrigger.readVoltage() >= 1.5)
//   {
//     left.setEffort(0);
//     right.setEffort(0);
//     return true;
//   }
//   else
//   {
//     return false;
//   }
// }

// //kpAuto is a ratio to optimize speed and functionality in multiple areas
// //kpTurn is only used for leftTurnLine and rightTurnLine
// // ***** tune once leftTurnLine and rightTurnLine are merged
// float kpAuto = .12;
// float kpTurn = 1.3 ;
// float kpTurnLeadRatio = kpAuto * (kpTurn);
// float kpTurnFollowRatio = kpAuto / (kpTurn * 3);

// /**
//  * Turns using sensor to a line
//  * +1 turns right
//  * -1 turns left
//  * 0 does not turn just follows line to cross section
//  * ***New version untested***
//  * ***clean***
//  */
// void turnLine(int direction)
// {
//   float leftValue = 0;
//   float rightValue = 0;

//   if (direction == 1)
//   {
//     leftValue = rightTrigger.readVoltage() * kpTurnLeadRatio;
//     rightValue = leftTrigger.readVoltage() * kpTurnFollowRatio;
//   }

//   else if (direction == -1){
//     leftValue = rightTrigger.readVoltage() * kpTurnFollowRatio;
//     rightValue = leftTrigger.readVoltage() * kpTurnLeadRatio;
//   }

//   else if (eitherLineStop()){
//     leftValue = rightTrigger.readVoltage() * kpAuto;
//     rightValue = leftTrigger.readVoltage() * kpAuto;
//   }

//   left.setEffort(leftValue);
//   right.setEffort(rightValue);
//   delay(100);

//   while (true)
//   {
//     if (driveSensorsOnLine())
//     {
//       break;
//     }
//   }
// }

// /*
//  * if both outside trigger sensors detect a line stops the robot
//  */
// bool bothLineStop()
// {
//   if (leftTrigger.readVoltage() >= 1.5 && rightTrigger.readVoltage() >= 1.5)
//   {
//     left.setEffort(0);
//     right.setEffort(0);
//     return true;
//   }
//   else
//   {
//     return false;
//   }
// }

// /*
// * follows a line
// * turning right if left value is too low
// * turning left if right value is too low
// * uses eitherLineStop to detect cross sections or turning points
// */
// void lineTracker()
// {
//   while (true)
//   {
//     left.setEffort(rightLine.readVoltage() * kpAuto);
//     right.setEffort(leftLine.readVoltage() * kpAuto);
//     if (eitherLineStop()){
//       break;
//     }
//   }
// }

// /**
//  * centers the robot on the line by driving forward then turns till it sees a line
//  */
// void centerOnCrossSection(int direction)
// {
//   forward(9);
//   //+1 is turn right
//   //-1 is turn left
//   //0 only centers does not turn
//   //****untested with turnWithLine****
//   turnWithline(direction);
// }

// /**
//  * follows a line turning based on direction given as either
//  * -1 or +1
//  * uses bothLineStop to detect a cross section and stop moving
//  */
// void followLineToCrossSection(int autoTurnDirection, int centeredTurnDirection)
// {
//   while (true)
//   {
//     lineTracker();
//     turnLine(autoTurnDirection);
//     if (bothLineStop())
//     {
//       centerOnCrossSection(centeredTurnDirection);
//       return;
//     }
//   }
// }

// void findLine(){
//   left.setEffort(-.2);
//   right.setEffort(.2);
//   delay(200);
//   left.setEffort(.2 );
//   while (true)
//   {
//     if (driveSensorsOnLine()){
//       centerOnCrossSection(1);
//       turnWithline(-1);
//       return;
//     }
//   }

// }

// /**
//  * 1.5inch = 3.81cm
//  * 3inch = 7.62
//  * 4 to pick up bag **needs testing**
//  * left direction is always -1
//  * right direction is always +1
//  * No direction is always 0
//  */

// int DirectionRight = 1;
// int DirectionLeft = -1;
// int DirectionNone = 0;
// float lowerPlatform = 3.81;
// float higherPlatform = 7.62;

// void loop()
// {

//   //pickup bag
//   pickupBag();

//   //drive to cross section then centers and turns left
//   followLineToCrossSection(DirectionRight, DirectionRight);

//   //drives to platform and drops bag off
//   dropBag(higherPlatform);
//   armTurn(60);

//   //drives to cross section then centers then drives back to start to find second bag
//   followLineToCrossSection(DirectionRight, DirectionRight);
//   armTurn(180);
//   followLineToCrossSection(DirectionLeft, DirectionNone);

//   //finds and picks up second bag
//   turnWithline(DirectionRight);
//   pickupBag();

//   // drives to cross section and turns right then finds drop off area and drops bag off
//   followLineToCrossSection(DirectionRight, DirectionNone);
//   followLineToCrossSection(DirectionRight, DirectionNone);
//   forward(-3);
//   turnWithline(DirectionRight);
//   dropBag(lowerPlatform);

//   //drives to cross section and centers turning right
//   followLineToCrossSection(DirectionNone, DirectionLeft);
//   armTurn(180);

//   // //find a good place to look for free range bag
//   followLineToCrossSection(DirectionNone, DirectionNone);

//   // //collect free range bag
//   FindObject();
//   pickupBag();

//   // //find line
//   findLine();

//   // //drop off bag
//   armTurn(0);

//   // // //implement win button

//   // //infinite loop so function only runs once
//   while (true)
//   {
//     turnLine(-1);
//     delay(2000);
//   }
// }
