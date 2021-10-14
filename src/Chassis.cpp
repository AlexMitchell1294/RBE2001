#include <Arduino.h>
#include <RBE1001Lib.h>
#include <ESP32AnalogRead.h>
#include <ESP32Servo.h>
#include <ESP32Encoder.h>
#include <ESP32WifiManager.h>
#include <WebServer.h>
#include <ESP32PWM.h>
#include <PID_v1.h>
#include <ESP32PWM.h>
#include <Timer.h>
#include "Chassis.h"

Timer driveTimer(10);
float drivekp = 0.5;
/**
 * No arguments
 * initlizes motor and forward function as well as attaches left and right to esp.
 * not sure why these can be attached here and the others could not
 */

// Chassis::Chassis()
// {
//     // left.attach(MOTOR_LEFT_PWM, MOTOR_LEFT_DIR, MOTOR_LEFT_ENCA,
//     //             MOTOR_LEFT_ENCB);
//     // right.attach(MOTOR_RIGHT_PWM, MOTOR_RIGHT_DIR,
//     //              MOTOR_RIGHT_ENCA, MOTOR_RIGHT_ENCB);
// }


/**
 * move forward the given amount of cm
 */
void Chassis::forward(float cm)
{
    float degree = cm * degreesPerCMForWheel;
    left.startMoveFor(degree, 200);
    right.moveFor(degree, 200);
}


void Chassis::setDriveEffort(float leftVal, float rightVal){
    left.setEffort(leftVal);
    right.setEffort(rightVal);
}

void Chassis::driveFor(int setDistance){
    int inputValue = left.nowEncoder;
    float error = -setDistance + inputValue;
    if (driveTimer.isExpired()) {
        if (error >= 2.5 || error <= -2.5) {
          setDriveEffort(error*drivekp, error*drivekp);
        }
    }
}

