#include "EncoderManager.h"

volatile long encoderCounts[WHEEL_COUNT] = {0, 0, 0, 0};
long lastEncoderCounts[WHEEL_COUNT] = {0, 0, 0, 0};
long lastDeltaCounts[WHEEL_COUNT] = {0, 0, 0, 0};
float measuredRpm[WHEEL_COUNT] = {0, 0, 0, 0};

namespace {

void IRAM_ATTR updateEncoderCount(WheelIndex wheel) {
  int step = (digitalRead(wheels[wheel].encBPin) == HIGH) ? 1 : -1;
  if (invertEncoder[wheel]) {
    step = -step;
  }
  encoderCounts[wheel] += step;
}

void IRAM_ATTR handleEncoderR1() { updateEncoderCount(WHEEL_R1); }
void IRAM_ATTR handleEncoderR2() { updateEncoderCount(WHEEL_R2); }
void IRAM_ATTR handleEncoderF1() { updateEncoderCount(WHEEL_F1); }
void IRAM_ATTR handleEncoderF2() { updateEncoderCount(WHEEL_F2); }

typedef void (*EncoderIsr)();
const EncoderIsr encoderHandlers[WHEEL_COUNT] = {
  handleEncoderR1,
  handleEncoderR2,
  handleEncoderF1,
  handleEncoderF2
};

}  // namespace

void configureEncoderPins() {
  for (int i = 0; i < WHEEL_COUNT; i++) {
    pinMode(wheels[i].encAPin, INPUT);
    pinMode(wheels[i].encBPin, INPUT);
    attachInterrupt(digitalPinToInterrupt(wheels[i].encAPin), encoderHandlers[i], RISING);
  }
}

void computeWheelRpmFromDelta(long deltaCount, float dtSec, float &rpmOut) {
  if (dtSec > 0.0f) {
    rpmOut = (deltaCount / PPR) * (60.0f / dtSec);
  } else {
    rpmOut = 0.0f;
  }
}