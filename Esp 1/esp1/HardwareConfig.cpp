#include "HardwareConfig.h"

const WheelIndex LEFT_WHEELS[2] = {WHEEL_F1, WHEEL_R1};
const WheelIndex RIGHT_WHEELS[2] = {WHEEL_F2, WHEEL_R2};

const WheelConfig wheels[WHEEL_COUNT] = {
  {"r1", 32, 33, 25, 35, 34},
  {"r2", 14, 26, 27, 39, 36},
  {"f1", 13, 2, 4, 16, 17},
  {"f2", 5, 23, 15, 18, 19}
};

const bool invertEncoder[WHEEL_COUNT] = {
  true,
  false,
  true,
  false
};

bool invertMotor[WHEEL_COUNT] = {
  true,
  true,
  true,
  true
};
