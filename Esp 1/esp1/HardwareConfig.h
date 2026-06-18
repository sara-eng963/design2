#pragma once

#include <Arduino.h>

constexpr float PPR = 360.0f;
constexpr float WHEEL_DIAMETER_M = 0.095f;
constexpr float WHEEL_CIRCUMFERENCE_M = WHEEL_DIAMETER_M * 3.14159265358979323846f;

constexpr unsigned long CONTROL_PERIOD_MS = 50;
constexpr float CONTROL_PERIOD_SECONDS = CONTROL_PERIOD_MS / 1000.0f;
constexpr int PWM_MAX = 255;

constexpr float PI_F = 3.14159265358979323846f;

enum WheelIndex {
  WHEEL_R1 = 0,
  WHEEL_R2 = 1,
  WHEEL_F1 = 2,
  WHEEL_F2 = 3,
  WHEEL_COUNT = 4
};

extern const WheelIndex LEFT_WHEELS[2];
extern const WheelIndex RIGHT_WHEELS[2];

constexpr float TRIM_F1 = 1.00f;
constexpr float TRIM_R1 = 1.00f;
constexpr float TRIM_F2 = 1.00f;
constexpr float TRIM_R2 = 1.00f;

struct WheelConfig {
  const char *name;
  uint8_t pwmPin;
  uint8_t in1Pin;
  uint8_t in2Pin;
  uint8_t encAPin;
  uint8_t encBPin;
};

extern const WheelConfig wheels[WHEEL_COUNT];
extern const bool invertEncoder[WHEEL_COUNT];
extern bool invertMotor[WHEEL_COUNT];
