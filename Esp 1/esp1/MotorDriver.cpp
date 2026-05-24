#include "MotorDriver.h"

#include <cstdlib>

int appliedPwm[WHEEL_COUNT] = {0, 0, 0, 0};

namespace {

void writePwmDuty(uint8_t pin, uint32_t duty) {
  analogWrite(pin, duty);
}

}  // namespace

int clampSignedPwm(int pwmValue) {
  if (pwmValue > PWM_MAX) return PWM_MAX;
  if (pwmValue < -PWM_MAX) return -PWM_MAX;
  return pwmValue;
}

void configureMotorPinsSafe() {
  for (int i = 0; i < WHEEL_COUNT; i++) {
    pinMode(wheels[i].pwmPin, OUTPUT);
    pinMode(wheels[i].in1Pin, OUTPUT);
    pinMode(wheels[i].in2Pin, OUTPUT);
    digitalWrite(wheels[i].pwmPin, LOW);
    digitalWrite(wheels[i].in1Pin, LOW);
    digitalWrite(wheels[i].in2Pin, LOW);
    writePwmDuty(wheels[i].pwmPin, 0);
  }
}

void stopWheelHardware(WheelIndex wheel) {
  digitalWrite(wheels[wheel].in1Pin, LOW);
  digitalWrite(wheels[wheel].in2Pin, LOW);
  writePwmDuty(wheels[wheel].pwmPin, 0);
}

void stopAllMotorHardware() {
  for (int i = 0; i < WHEEL_COUNT; i++) {
    stopWheelHardware((WheelIndex)i);
  }
}

void applyMotorCommandToWheel(WheelIndex wheel, int commandedPwm) {
  int safeCommand = clampSignedPwm(commandedPwm);
  int hardwareCommand = invertMotor[wheel] ? -safeCommand : safeCommand;

  if (hardwareCommand > 0) {
    digitalWrite(wheels[wheel].in1Pin, HIGH);
    digitalWrite(wheels[wheel].in2Pin, LOW);
    writePwmDuty(wheels[wheel].pwmPin, hardwareCommand);
  } else if (hardwareCommand < 0) {
    digitalWrite(wheels[wheel].in1Pin, LOW);
    digitalWrite(wheels[wheel].in2Pin, HIGH);
    writePwmDuty(wheels[wheel].pwmPin, abs(hardwareCommand));
  } else {
    stopWheelHardware(wheel);
  }

  appliedPwm[wheel] = safeCommand;
}

void applyAllWheelMotorCommands(const int commandedPwm[WHEEL_COUNT]) {
  for (int i = 0; i < WHEEL_COUNT; i++) {
    applyMotorCommandToWheel((WheelIndex)i, commandedPwm[i]);
  }
}