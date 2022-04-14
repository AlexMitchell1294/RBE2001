#pragma once

class CServo {

 public:
  void setup();
  void seteffort(int effort);
  void Close();
  void Open();
  const int PWM = 33;
};