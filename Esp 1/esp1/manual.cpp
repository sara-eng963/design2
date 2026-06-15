#include "manual.h"
#include "HardwareConfig.h"      // provides WHEEL_COUNT, WheelIndex, wheels[]
#include "PositionController.h"
#include "WheelVelocityController.h" // for targetRpm[] (extern), runVelocityLoopForWheel, etc.

// ------------------------------------------------------------
// External references from the main control code
// ------------------------------------------------------------
extern float targetRpm[WHEEL_COUNT];       // array of target RPM per wheel
extern MotionMode motionMode;              // current motion mode (enum)
extern bool positionModeActive;            // true when motion is allowed
extern void setIdleStateAndStopMotors();   // stops robot and resets state
extern void finalizeMotionStop();          // zeroes target RPM, resets PID

// ------------------------------------------------------------
// Manual mode global state
// ------------------------------------------------------------
float manualVx = 0.0f;
float manualVy = 0.0f;
float manualWz = 0.0f;
unsigned long lastManualCmdTime = 0;

static float maxRpm = MANUAL_MAX_RPM;      // can be changed at runtime

// ------------------------------------------------------------
// Initialisation
// ------------------------------------------------------------
void initManualMode() {
    manualVx = 0.0f;
    manualVy = 0.0f;
    manualWz = 0.0f;
    lastManualCmdTime = millis();
}

// ------------------------------------------------------------
// Called when a new MANUAL command is received
// ------------------------------------------------------------
void updateManualCommand(float vx, float vy, float wz) {
    // Clip inputs to valid range
    manualVx = constrain(vx, -1.0f, 1.0f);
    manualVy = constrain(vy, -1.0f, 1.0f);
    manualWz = constrain(wz, -1.0f, 1.0f);
    lastManualCmdTime = millis();
}

// ------------------------------------------------------------
// Mecanum/omni mixer – writes to targetRpm[] array
// ------------------------------------------------------------
void runManualMixer() {
    if (!positionModeActive) {
        // Safety: if the system is not ready (e.g. IMU fault), stop wheels
        for (int i = 0; i < WHEEL_COUNT; i++) {
            targetRpm[i] = 0.0f;
        }
        return;
    }

    // Scale inputs to RPM (maxRpm = 60 by default)
    float vx_rpm = manualVx * maxRpm;
    float vy_rpm = manualVy * maxRpm;
    float wz_rpm = manualWz * maxRpm;

    // Standard mecanum wheel mixer.
    // F1 = front left, F2 = front right, R1 = rear left, R2 = rear right
    // Positive vx = forward, positive vy = strafe right, positive wz = rotate right (CW)
    targetRpm[WHEEL_F1] = vx_rpm - vy_rpm - wz_rpm;
    targetRpm[WHEEL_F2] = vx_rpm + vy_rpm + wz_rpm;
    targetRpm[WHEEL_R1] = vx_rpm + vy_rpm - wz_rpm;
    targetRpm[WHEEL_R2] = vx_rpm - vy_rpm + wz_rpm;

    // Clip each wheel RPM to [-maxRpm, maxRpm]
    for (int i = 0; i < WHEEL_COUNT; i++) {
        targetRpm[i] = constrain(targetRpm[i], -maxRpm, maxRpm);
    }
}

// ------------------------------------------------------------
// Watchdog: returns true if watchdog timed out and robot should stop
// ------------------------------------------------------------
bool checkManualWatchdog() {
    if (motionMode != MODE_MANUAL_VELOCITY) {
        return false;               // watchdog only applies when in manual mode
    }
    unsigned long now = millis();
    if (now - lastManualCmdTime >= MANUAL_WATCHDOG_MS) {
        // No command received for too long – stop the robot
        finalizeMotionStop();
        setIdleStateAndStopMotors();   // also sets motionMode = MODE_IDLE
        return true;
    }
    return false;
}

// ------------------------------------------------------------
// Helper: check if we are currently in manual velocity mode
// ------------------------------------------------------------
bool isManualModeActive() {
    return (motionMode == MODE_MANUAL_VELOCITY);
}

// ------------------------------------------------------------
// Change the maximum RPM (e.g. for speed limiting)
// ------------------------------------------------------------
void setManualMaxRpm(float rpm) {
    if (rpm >= 0.0f) {
        maxRpm = rpm;
    }
}
