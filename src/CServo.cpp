#include "CServo.h"
#include <RBE1001Lib.h>


void CServo::setup() {
    pinMode(PWM, OUTPUT);
}

void CServo::seteffort(int effort){
  int value = constrain(effort, 0, 255);
  analogWrite(PWM, value);
}

void CServo::Close()
{
    seteffort(180);
    delay(3000);
    seteffort(0);
}

void CServo::Open()
{
    seteffort(0);
    delay(3000);
    seteffort(0);
}