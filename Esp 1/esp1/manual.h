#ifndef MANUAL_MODE_H
#define MANUAL_MODE_H

#include <Arduino.h>

// ------------------------------------------------------------
// Manual velocity mode configuration
// ------------------------------------------------------------
constexpr float MANUAL_MAX_RPM = 120.0f;          // maximum wheel RPM (user requested constant 60 RPM)
constexpr unsigned long MANUAL_WATCHDOG_MS = 250; // stop after 250 ms without command

// ------------------------------------------------------------
// Global variables (extern) – defined in ManualMode.cpp
// ------------------------------------------------------------
extern float manualVx;          // normalized forward/back (-1..1)
extern float manualVy;          // normalized strafe left/right (-1..1)
extern float manualWz;          // normalized rotation (-1..1)
extern unsigned long lastManualCmdTime;

// ------------------------------------------------------------
// Function prototypes
// ------------------------------------------------------------
void initManualMode();
void updateManualCommand(float vx, float vy, float wz);
void runManualMixer();                     // writes to global targetRpm[]
bool checkManualWatchdog();                // returns true if watchdog triggered
bool isManualModeActive();                 // checks current motion mode (requires external mode variable)
void setManualMaxRpm(float rpm);

#endif // MANUAL_MODE_H