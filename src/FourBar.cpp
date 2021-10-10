#include "FourBar.h"

#include <RBE1001Lib.h>
int encoderTable[4][4] = {
  {0,-1,1,0,},
  {1,0,0,-1,},
  {-1,0,0,1,},
  {0,1,-1,0},
};
int oldVal = 0;
int newVal = 0;
volatile int out = 0;

long count = 0;  // encoder counter
// Mutex for the count critical variable
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

/**
 * Interrupt service routine (ISR) for one of the encoder
 * channels. It simply counts how many interrupts occured
 */
void IRAM_ATTR isr() {
  portENTER_CRITICAL_ISR(&mux);
  oldVal = newVal;
  newVal = (digitalRead(18) << 1) | digitalRead(19);

  out = encoderTable[oldVal][newVal];
  count -= out;
  portEXIT_CRITICAL_ISR(&mux);
}

/**
 * Set up all the variables for the blue motor
 * This is not the same as the ESP32 setup() function, although
 * that would be a good place to call this setup function
 */
void FourBar::setup() {
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  pinMode(PWM, OUTPUT);
  attachInterrupt(ENCA, isr, CHANGE);
  attachInterrupt(ENCB, isr, CHANGE);
}

/**
 * Get the current encoder count
 * Returns the encoder count value
 */
long FourBar::getPosition() {
  long value;
  portENTER_CRITICAL(&mux);
  value = count;
  portEXIT_CRITICAL(&mux);
  return value;
}

/**
 * Reset the encoder count to zero
 */
void FourBar::reset() {
  portENTER_CRITICAL(&mux);
  count = 0;
  portEXIT_CRITICAL(&mux);
}

/**
 * Set the motor effort
 * effort value ranges from -255 to +255
 * Negative values turn one way, positive values turn the other way
 */
void FourBar::setEffort(int effort) {
  if (effort < 0) {
    setEffort(-effort, true);
  } else {
    setEffort(effort, false);
  }
}

/**
 * Set the motor effort
 * effort values range from 0-255
 * clockwise is true for one direction, false for the other
 */
void FourBar::setEffort(int effort, bool clockwise) {
  if (clockwise) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  }
  int value = constrain(effort, 0, 255);
  analogWrite(PWM, value);
}

void FourBar::moveTo(int pos){
    int readVal = getPosition();
    float error = pos-readVal;
    float errorTotal = 0;
    float lastError = 0; 
    while(1){
      printf("%d: \t %d:\t %f:\n", readVal, pos, error);
      readVal = getPosition();
      error = pos-readVal;
      errorTotal += error;
      if (error >= 50 || error <= -50) {
        setEffort(error*kpA+errorTotal*kiA + (error - lastError) * kdA);
        lastError = error;
      }
      else break;
    } 
}