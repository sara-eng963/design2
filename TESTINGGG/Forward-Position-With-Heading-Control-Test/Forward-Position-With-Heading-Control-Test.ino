#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <cmath>
#include <cstddef>
#include <cstdint>

// ============================================================
// Forward-Position-With-Heading-Control-Test for ESP32
// ------------------------------------------------------------
// Goal of this file:
// - Move straight by a signed distance command (example: +0.50, -0.50)
// - Keep heading (yaw) close to the heading captured at move start
// - Keep architecture simple and beginner-friendly
//
// Cascade structure (important):
// 1) Position loop outputs BASE forward RPM (not PWM)
// 2) Heading loop outputs TURN correction RPM (not PWM)
// 3) Wheel velocity loops output PWM
//
// Why heading control is needed:
// - Even for straight commands, small friction differences can make robot rotate.
// - IMU yaw tells us that unwanted rotation.
// - We counter it by making left/right wheel RPM slightly different.
//
// What this file intentionally does NOT include:
// - No x/y waypoint control
// - No sideways motion
// - No full mecanum kinematics
// - No trajectory planning
// - No Kalman filter
// - No heading integral/derivative
// - No feedforward
// - No deadband compensation
// - No advanced state machine
// ============================================================

namespace app {

constexpr std::uint8_t MPU6050_I2C_ADDRESS = 0x68U;

// Default I2C pins for most 32-pin ESP32 DevKit boards.
constexpr std::int32_t IMU_SDA_PIN = 21;
constexpr std::int32_t IMU_SCL_PIN = 22;
constexpr std::uint32_t I2C_FREQUENCY_HZ = 400000U;

constexpr std::uint32_t IMU_GYRO_BIAS_SAMPLES = 1500U;
constexpr std::uint32_t SERIAL_BAUD_RATE = 115200U;
constexpr std::uint32_t IMU_READ_PERIOD_MS = 20U;
constexpr std::uint32_t DISPLAY_INTERVAL_MS = 200U;

constexpr float kPi = 3.14159265358979323846f;
constexpr float kGravityMps2 = 9.80665f;

inline float radToDeg(float radians) {
    return radians * (180.0f / kPi);
}

inline float wrapAngleRad(float angle_rad) {
    while (angle_rad > kPi) {
        angle_rad -= 2.0f * kPi;
    }
    while (angle_rad < -kPi) {
        angle_rad += 2.0f * kPi;
    }
    return angle_rad;
}

struct IMUState {
    float accel_x = 0.0f;
    float accel_y = 0.0f;
    float accel_z = 0.0f;

    float gyro_x = 0.0f;
    float gyro_y = 0.0f;
    float gyro_z = 0.0f;

    float orientation_x = 0.0f;  // roll
    float orientation_y = 0.0f;  // pitch
    float orientation_z = 0.0f;  // yaw
};

enum class MotionState : std::uint8_t {
    MOVING = 0U,
    SETTLING = 1U,
    STILL = 2U,
};

class ImuDriver {
public:
    bool begin();
    bool read(IMUState& imu_state);
    void zeroYaw();
    bool isHealthy() const;
    bool hasDetectedIdentity() const;
    std::uint8_t detectedWhoAmI() const;
    const char* detectedChipName() const;
    float yawIntegratedRad() const;
    float yawZeroOffsetRad() const;
    float displayedYawRad() const;
    float rawGyroZRadPerSec() const;
    float correctedGyroZRadPerSec() const;
    float gyroBiasZRadPerSec() const;
    const char* motionStateName() const;

private:
    bool writeRegister(std::uint8_t reg, std::uint8_t value);
    bool readRegisters(std::uint8_t start_reg, std::uint8_t* buffer, std::size_t length);
    void calibrateGyroBias();
    void setMotionState(MotionState state, std::uint32_t now_us);

    bool initialized_ = false;
    bool healthy_ = false;
    bool has_detected_identity_ = false;
    std::uint8_t detected_who_am_i_ = 0U;
    float roll_rad_ = 0.0f;
    float pitch_rad_ = 0.0f;
    float yaw_integrated_rad_ = 0.0f;
    float yaw_zero_offset_rad_ = 0.0f;
    float gyro_bias_x_rad_s_ = 0.0f;
    float gyro_bias_y_rad_s_ = 0.0f;
    float gyro_bias_z_rad_s_ = 0.0f;
    float raw_gyro_z_rad_s_ = 0.0f;
    float corrected_gyro_z_rad_s_ = 0.0f;
    std::uint32_t last_update_us_ = 0U;
    std::uint32_t motion_state_since_us_ = 0U;
    MotionState motion_state_ = MotionState::STILL;
};

namespace {

constexpr std::uint8_t kWhoAmIReg = 0x75U;
constexpr std::uint8_t kPwrMgmt1Reg = 0x6BU;
constexpr std::uint8_t kConfigReg = 0x1AU;
constexpr std::uint8_t kGyroConfigReg = 0x1BU;
constexpr std::uint8_t kAccelConfigReg = 0x1CU;
constexpr std::uint8_t kAccelXoutHReg = 0x3BU;

constexpr std::uint8_t kMpu6050WhoAmI = 0x68U;
constexpr std::uint8_t kMpu6500WhoAmI = 0x70U;
constexpr std::uint8_t kMpu9250WhoAmI = 0x71U;
constexpr std::uint8_t kMpu9255WhoAmI = 0x73U;

constexpr float kAccelLsbPerG = 8192.0f;      // +/-4 g
constexpr float kGyroLsbPerDegPerSec = 65.5f; // +/-500 dps
constexpr float kDegToRad = kPi / 180.0f;
constexpr float kComplementaryAlpha = 0.98f;
constexpr float kYawDeadbandRadPerSec = 0.02f;
constexpr float kStillEnterAccelToleranceMps2 = 0.30f;
constexpr float kStillExitAccelToleranceMps2 = 0.45f;
constexpr float kStillEnterGyroThresholdRadPerSec = 0.06f;
constexpr float kStillExitGyroThresholdRadPerSec = 0.16f;
constexpr float kBiasAdaptationRateXYStill = 0.004f;
constexpr float kBiasAdaptationRateZStill = 0.10f;
constexpr std::uint32_t kSettlingDurationUs = 300000U;

ImuDriver g_imu_driver;

std::int16_t combineBigEndian(std::uint8_t msb, std::uint8_t lsb) {
    return static_cast<std::int16_t>((static_cast<std::uint16_t>(msb) << 8U) | lsb);
}

float maxAbs3(float x, float y, float z) {
    const float abs_x = std::fabs(x);
    const float abs_y = std::fabs(y);
    const float abs_z = std::fabs(z);
    const float max_xy = (abs_x > abs_y) ? abs_x : abs_y;
    return (max_xy > abs_z) ? max_xy : abs_z;
}

const char* chipNameForWhoAmI(std::uint8_t who_am_i) {
    switch (who_am_i) {
        case kMpu6050WhoAmI:
            return "MPU6050";
        case kMpu6500WhoAmI:
            return "MPU6500";
        case kMpu9250WhoAmI:
            return "MPU9250";
        case kMpu9255WhoAmI:
            return "MPU9255";
        default:
            return "Unknown IMU";
    }
}

bool isSupportedWhoAmI(std::uint8_t who_am_i) {
    switch (who_am_i) {
        case kMpu6050WhoAmI:
        case kMpu6500WhoAmI:
        case kMpu9250WhoAmI:
        case kMpu9255WhoAmI:
            return true;
        default:
            return false;
    }
}

const char* motionStateToString(MotionState state) {
    switch (state) {
        case MotionState::MOVING:
            return "MOVING";
        case MotionState::SETTLING:
            return "SETTLING";
        case MotionState::STILL:
            return "STILL";
        default:
            return "UNKNOWN";
    }
}

}  // namespace

bool ImuDriver::begin() {
    Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN, I2C_FREQUENCY_HZ);
    delay(50);

    std::uint8_t who_am_i = 0U;
    has_detected_identity_ = readRegisters(kWhoAmIReg, &who_am_i, 1U);
    detected_who_am_i_ = has_detected_identity_ ? who_am_i : 0U;
    healthy_ = has_detected_identity_ && isSupportedWhoAmI(who_am_i);
    if (!healthy_) {
        initialized_ = false;
        return false;
    }

    healthy_ = writeRegister(kPwrMgmt1Reg, 0x00U)
        && writeRegister(kConfigReg, 0x03U)
        && writeRegister(kGyroConfigReg, 0x08U)
        && writeRegister(kAccelConfigReg, 0x08U);

    if (!healthy_) {
        initialized_ = false;
        return false;
    }

    roll_rad_ = 0.0f;
    pitch_rad_ = 0.0f;
    yaw_integrated_rad_ = 0.0f;
    yaw_zero_offset_rad_ = 0.0f;
    gyro_bias_x_rad_s_ = 0.0f;
    gyro_bias_y_rad_s_ = 0.0f;
    gyro_bias_z_rad_s_ = 0.0f;
    raw_gyro_z_rad_s_ = 0.0f;
    corrected_gyro_z_rad_s_ = 0.0f;
    last_update_us_ = 0U;
    motion_state_since_us_ = 0U;
    motion_state_ = MotionState::STILL;
    initialized_ = true;

    calibrateGyroBias();
    last_update_us_ = micros();
    motion_state_since_us_ = last_update_us_;
    return healthy_;
}

bool ImuDriver::read(IMUState& imu_state) {
    if (!initialized_) {
        return false;
    }

    std::uint8_t buffer[14] = {0};
    healthy_ = readRegisters(kAccelXoutHReg, buffer, sizeof(buffer));
    if (!healthy_) {
        return false;
    }

    const std::int16_t raw_ax = combineBigEndian(buffer[0], buffer[1]);
    const std::int16_t raw_ay = combineBigEndian(buffer[2], buffer[3]);
    const std::int16_t raw_az = combineBigEndian(buffer[4], buffer[5]);
    const std::int16_t raw_gx = combineBigEndian(buffer[8], buffer[9]);
    const std::int16_t raw_gy = combineBigEndian(buffer[10], buffer[11]);
    const std::int16_t raw_gz = combineBigEndian(buffer[12], buffer[13]);

    const float accel_scale = kGravityMps2 / kAccelLsbPerG;
    const float gyro_scale = kDegToRad / kGyroLsbPerDegPerSec;

    imu_state.accel_x = static_cast<float>(raw_ax) * accel_scale;
    imu_state.accel_y = static_cast<float>(raw_ay) * accel_scale;
    imu_state.accel_z = static_cast<float>(raw_az) * accel_scale;

    const float raw_gyro_x = static_cast<float>(raw_gx) * gyro_scale;
    const float raw_gyro_y = static_cast<float>(raw_gy) * gyro_scale;
    const float raw_gyro_z = static_cast<float>(raw_gz) * gyro_scale;
    raw_gyro_z_rad_s_ = raw_gyro_z;

    imu_state.gyro_x = raw_gyro_x - gyro_bias_x_rad_s_;
    imu_state.gyro_y = raw_gyro_y - gyro_bias_y_rad_s_;
    imu_state.gyro_z = raw_gyro_z - gyro_bias_z_rad_s_;
    corrected_gyro_z_rad_s_ = imu_state.gyro_z;

    const std::uint32_t now_us = micros();
    const float dt_seconds = (last_update_us_ == 0U)
        ? 0.0f
        : static_cast<float>(now_us - last_update_us_) * 1.0e-6f;
    last_update_us_ = now_us;

    const float accel_roll = std::atan2(imu_state.accel_y, imu_state.accel_z);
    const float accel_pitch = std::atan2(
        -imu_state.accel_x,
        std::sqrt((imu_state.accel_y * imu_state.accel_y) + (imu_state.accel_z * imu_state.accel_z)));

    const float accel_magnitude = std::sqrt(
        (imu_state.accel_x * imu_state.accel_x)
        + (imu_state.accel_y * imu_state.accel_y)
        + (imu_state.accel_z * imu_state.accel_z));
    const float accel_error = std::fabs(accel_magnitude - kGravityMps2);
    const float max_abs_gyro = maxAbs3(imu_state.gyro_x, imu_state.gyro_y, imu_state.gyro_z);

    const bool qualifies_for_still = (accel_error < kStillEnterAccelToleranceMps2)
        && (max_abs_gyro < kStillEnterGyroThresholdRadPerSec);
    const bool clearly_moving = (accel_error > kStillExitAccelToleranceMps2)
        || (max_abs_gyro > kStillExitGyroThresholdRadPerSec);

    switch (motion_state_) {
        case MotionState::MOVING:
            if (qualifies_for_still) {
                setMotionState(MotionState::SETTLING, now_us);
            }
            break;
        case MotionState::SETTLING:
            if (clearly_moving) {
                setMotionState(MotionState::MOVING, now_us);
            } else if (qualifies_for_still
                && (now_us - motion_state_since_us_) >= kSettlingDurationUs) {
                setMotionState(MotionState::STILL, now_us);
            }
            break;
        case MotionState::STILL:
            if (clearly_moving) {
                setMotionState(MotionState::MOVING, now_us);
            } else if (!qualifies_for_still) {
                setMotionState(MotionState::SETTLING, now_us);
            }
            break;
    }

    if (motion_state_ == MotionState::STILL) {
        gyro_bias_x_rad_s_ += kBiasAdaptationRateXYStill * (raw_gyro_x - gyro_bias_x_rad_s_);
        gyro_bias_y_rad_s_ += kBiasAdaptationRateXYStill * (raw_gyro_y - gyro_bias_y_rad_s_);
        gyro_bias_z_rad_s_ += kBiasAdaptationRateZStill * (raw_gyro_z - gyro_bias_z_rad_s_);

        imu_state.gyro_x = raw_gyro_x - gyro_bias_x_rad_s_;
        imu_state.gyro_y = raw_gyro_y - gyro_bias_y_rad_s_;
        imu_state.gyro_z = raw_gyro_z - gyro_bias_z_rad_s_;
        corrected_gyro_z_rad_s_ = imu_state.gyro_z;
    }

    float yaw_rate_for_integration = imu_state.gyro_z;
    if (std::fabs(yaw_rate_for_integration) < kYawDeadbandRadPerSec) {
        yaw_rate_for_integration = 0.0f;
    }
    if (motion_state_ == MotionState::STILL) {
        yaw_rate_for_integration = 0.0f;
    }

    if (dt_seconds > 0.0f && dt_seconds < 0.25f) {
        roll_rad_ = wrapAngleRad(
            (kComplementaryAlpha * (roll_rad_ + imu_state.gyro_x * dt_seconds))
            + ((1.0f - kComplementaryAlpha) * accel_roll));

        pitch_rad_ = wrapAngleRad(
            (kComplementaryAlpha * (pitch_rad_ + imu_state.gyro_y * dt_seconds))
            + ((1.0f - kComplementaryAlpha) * accel_pitch));

        if (motion_state_ != MotionState::STILL) {
            yaw_integrated_rad_ = wrapAngleRad(yaw_integrated_rad_ + yaw_rate_for_integration * dt_seconds);
        }
    } else {
        roll_rad_ = accel_roll;
        pitch_rad_ = accel_pitch;
    }

    imu_state.orientation_x = roll_rad_;
    imu_state.orientation_y = pitch_rad_;
    imu_state.orientation_z = displayedYawRad();
    return true;
}

void ImuDriver::zeroYaw() {
    yaw_zero_offset_rad_ = yaw_integrated_rad_;
}

bool ImuDriver::isHealthy() const {
    return healthy_;
}

bool ImuDriver::hasDetectedIdentity() const {
    return has_detected_identity_;
}

std::uint8_t ImuDriver::detectedWhoAmI() const {
    return detected_who_am_i_;
}

const char* ImuDriver::detectedChipName() const {
    if (!has_detected_identity_) {
        return "No IMU response";
    }
    return chipNameForWhoAmI(detected_who_am_i_);
}

float ImuDriver::yawIntegratedRad() const {
    return yaw_integrated_rad_;
}

float ImuDriver::yawZeroOffsetRad() const {
    return yaw_zero_offset_rad_;
}

float ImuDriver::displayedYawRad() const {
    return wrapAngleRad(yaw_integrated_rad_ - yaw_zero_offset_rad_);
}

float ImuDriver::rawGyroZRadPerSec() const {
    return raw_gyro_z_rad_s_;
}

float ImuDriver::correctedGyroZRadPerSec() const {
    return corrected_gyro_z_rad_s_;
}

float ImuDriver::gyroBiasZRadPerSec() const {
    return gyro_bias_z_rad_s_;
}

const char* ImuDriver::motionStateName() const {
    return motionStateToString(motion_state_);
}

bool ImuDriver::writeRegister(std::uint8_t reg, std::uint8_t value) {
    Wire.beginTransmission(MPU6050_I2C_ADDRESS);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

bool ImuDriver::readRegisters(std::uint8_t start_reg, std::uint8_t* buffer, std::size_t length) {
    Wire.beginTransmission(MPU6050_I2C_ADDRESS);
    Wire.write(start_reg);
    if (Wire.endTransmission(false) != 0) {
        return false;
    }

    const std::size_t received = Wire.requestFrom(
        static_cast<int>(MPU6050_I2C_ADDRESS),
        static_cast<int>(length),
        static_cast<int>(true));
    if (received != length) {
        return false;
    }

    for (std::size_t index = 0; index < length; ++index) {
        buffer[index] = static_cast<std::uint8_t>(Wire.read());
    }
    return true;
}

void ImuDriver::calibrateGyroBias() {
    float bias_x = 0.0f;
    float bias_y = 0.0f;
    float bias_z = 0.0f;
    std::uint32_t samples_collected = 0U;
    const float gyro_scale = kDegToRad / kGyroLsbPerDegPerSec;

    for (std::uint32_t sample = 0; sample < IMU_GYRO_BIAS_SAMPLES; ++sample) {
        std::uint8_t buffer[14] = {0};
        if (!readRegisters(kAccelXoutHReg, buffer, sizeof(buffer))) {
            continue;
        }

        const std::int16_t raw_gx = combineBigEndian(buffer[8], buffer[9]);
        const std::int16_t raw_gy = combineBigEndian(buffer[10], buffer[11]);
        const std::int16_t raw_gz = combineBigEndian(buffer[12], buffer[13]);

        bias_x += static_cast<float>(raw_gx) * gyro_scale;
        bias_y += static_cast<float>(raw_gy) * gyro_scale;
        bias_z += static_cast<float>(raw_gz) * gyro_scale;
        ++samples_collected;
        delay(2);
    }

    if (samples_collected == 0U) {
        return;
    }

    const float inv_samples = 1.0f / static_cast<float>(samples_collected);
    gyro_bias_x_rad_s_ = bias_x * inv_samples;
    gyro_bias_y_rad_s_ = bias_y * inv_samples;
    gyro_bias_z_rad_s_ = bias_z * inv_samples;
}

void ImuDriver::setMotionState(MotionState state, std::uint32_t now_us) {
    if (motion_state_ != state) {
        motion_state_ = state;
        motion_state_since_us_ = now_us;
    }
}

ImuDriver& imuDriver() {
    return g_imu_driver;
}

}  // namespace app

const char *WIFI_SSID = "Pluto";
const char *WIFI_PASSWORD = "12345678";
WebServer server(80);

// -----------------------------
// Hardware constants
// -----------------------------
const float PPR = 374.0f;
const float WHEEL_DIAMETER_M = 0.097f;
const float WHEEL_CIRCUMFERENCE_M = WHEEL_DIAMETER_M * 3.14159265358979323846f;

const unsigned long CONTROL_PERIOD_MS = 50;
const float CONTROL_PERIOD_SECONDS = CONTROL_PERIOD_MS / 1000.0f;
const int PWM_MAX = 255;

const float PI_F = 3.14159265358979323846f;

// -----------------------------
// Position loop tuning
// -----------------------------
float Kp_pos = 0.8f;
float Ki_pos = 0.0f;
const float MAX_VX = 0.25f;
const float POSITION_TOLERANCE = 0.03f;
const float MIN_MOVE_RPM = 70.0f;

// -----------------------------
// Heading loop tuning (PI)
// -----------------------------
// Kp_heading_rpm converts heading error (rad) into RPM correction.
// Example: 0.1 rad error * 30 => 3 RPM correction.
// Beginner tuning:
// - hkp: how strongly yaw error changes left/right RPM split.
// - hki: removes steady heading bias while moving.
// - hmax: hard cap on that RPM split for safe, predictable behavior.
// Test presets:
// Gentle: Kp_heading_rpm = 30, Ki_heading_rpm_per_rad_s = 0, MAX_TURN_CORRECTION_RPM = 15
// Medium: Kp_heading_rpm = 60, Ki_heading_rpm_per_rad_s = 2, MAX_TURN_CORRECTION_RPM = 30
// Strong: Kp_heading_rpm = 90, Ki_heading_rpm_per_rad_s = 4, MAX_TURN_CORRECTION_RPM = 40
float Kp_heading_rpm = 60.0f;
float Ki_heading_rpm_per_rad_s = 2.0f;
float MAX_TURN_CORRECTION_RPM = 30.0f;
bool headingControlEnabled = true;
// Robot testing showed heading correction direction is reversed, so inversion is enabled by default.
bool invertHeadingCorrection = true;
const float HEADING_INTEGRAL_LIMIT_RAD_S = 2.0f;

// Extra safety: cap final per-wheel RPM target.
const float MAX_WHEEL_TARGET_RPM = 220.0f;

// -----------------------------
// Wheel/index definitions
// -----------------------------
// Mapping used in this file:
// - F1 = front left
// - R1 = rear left
// - F2 = front right
// - R2 = rear right
//
// If your hardware mapping changes later, update these constants only.
enum WheelIndex {
  WHEEL_R1 = 0,
  WHEEL_R2 = 1,
  WHEEL_F1 = 2,
  WHEEL_F2 = 3,
  WHEEL_COUNT = 4
};

const WheelIndex LEFT_WHEELS[2] = {WHEEL_F1, WHEEL_R1};
const WheelIndex RIGHT_WHEELS[2] = {WHEEL_F2, WHEEL_R2};

struct WheelConfig {
  const char *name;
  uint8_t pwmPin;
  uint8_t in1Pin;
  uint8_t in2Pin;
  uint8_t encAPin;
  uint8_t encBPin;
};

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

// -----------------------------
// Shared state (RTOS + ISR)
// -----------------------------
volatile long encoderCounts[WHEEL_COUNT] = {0, 0, 0, 0};
long lastEncoderCounts[WHEEL_COUNT] = {0, 0, 0, 0};
long lastDeltaCounts[WHEEL_COUNT] = {0, 0, 0, 0};

float measuredRpm[WHEEL_COUNT] = {0, 0, 0, 0};
float targetRpm[WHEEL_COUNT] = {0, 0, 0, 0};
float velocityError[WHEEL_COUNT] = {0, 0, 0, 0};
float velocityIntegral[WHEEL_COUNT] = {0, 0, 0, 0};
float velocityDerivative[WHEEL_COUNT] = {0, 0, 0, 0};
float previousVelocityError[WHEEL_COUNT] = {0, 0, 0, 0};
float outputPwmUnclamped[WHEEL_COUNT] = {0, 0, 0, 0};
int finalPwm[WHEEL_COUNT] = {0, 0, 0, 0};
int appliedPwm[WHEEL_COUNT] = {0, 0, 0, 0};

float kpVel[WHEEL_COUNT] = {5.0f, 5.5f, 4.5f, 5.5f};
float kiVel[WHEEL_COUNT] = {1.15f, 1.15f, 1.15f, 1.30f};
float kdVel[WHEEL_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};
const float VELOCITY_INTEGRAL_LIMIT = 300.0f;

// Position / heading state
bool positionModeActive = false;
enum MotionMode {
  MODE_IDLE = 0,
  MODE_MOVE_FORWARD = 1
};
MotionMode motionMode = MODE_IDLE;

float targetDistanceM = 0.0f;
float currentDistanceM = 0.0f;
float distanceErrorM = 0.0f;
float positionIntegral = 0.0f;

float baseForwardRpm = 0.0f;
float rawBaseForwardRpm = 0.0f;
bool minimumMoveRpmActive = false;
float turnCorrectionRPM = 0.0f;
float leftRpmComposed = 0.0f;
float rightRpmComposed = 0.0f;

float targetHeadingRad = 0.0f;
float currentHeadingRad = 0.0f;
float headingErrorRad = 0.0f;
float headingIntegralRadS = 0.0f;

bool imuHealthy = false;
uint32_t imuLastOkReadMs = 0;
float imuRawGyroZDegPerSec = 0.0f;
float imuCorrectedGyroZDegPerSec = 0.0f;
float imuBiasZDegPerSec = 0.0f;
float imuIntegratedYawDeg = 0.0f;
float imuZeroOffsetYawDeg = 0.0f;
const char *imuMotionState = "UNKNOWN";

float currentDtSeconds = CONTROL_PERIOD_SECONDS;
unsigned long lastControlTime = 0;

SemaphoreHandle_t stateMutex = nullptr;
TaskHandle_t controlTaskHandle = nullptr;
TaskHandle_t webTaskHandle = nullptr;
TaskHandle_t imuTaskHandle = nullptr;

String serialLineBuffer;

// -----------------------------
// Utility helpers
// -----------------------------
bool lockState(TickType_t timeoutTicks = pdMS_TO_TICKS(20)) {
  return (stateMutex != nullptr) && (xSemaphoreTake(stateMutex, timeoutTicks) == pdTRUE);
}

void unlockState() {
  if (stateMutex != nullptr) {
    xSemaphoreGive(stateMutex);
  }
}

float wrapAngleRad(float angle) {
  // Keep angles inside [-PI, +PI] so the controller always takes
  // the shortest turn direction. Example: +179 deg and -179 deg are
  // close in reality, and wrapping prevents a false 358 deg error.
  while (angle > PI_F) angle -= 2.0f * PI_F;
  while (angle < -PI_F) angle += 2.0f * PI_F;
  return angle;
}

float radToDeg(float rad) {
  return rad * (180.0f / PI_F);
}

float clampWheelTargetRpm(float rpm) {
  if (rpm > MAX_WHEEL_TARGET_RPM) return MAX_WHEEL_TARGET_RPM;
  if (rpm < -MAX_WHEEL_TARGET_RPM) return -MAX_WHEEL_TARGET_RPM;
  return rpm;
}

void writePwmDuty(uint8_t pin, uint32_t duty) {
  analogWrite(pin, duty);
}

int clampSignedPwm(int pwmValue) {
  if (pwmValue > PWM_MAX) return PWM_MAX;
  if (pwmValue < -PWM_MAX) return -PWM_MAX;
  return pwmValue;
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

void applyAllWheelMotorCommands() {
  for (int i = 0; i < WHEEL_COUNT; i++) {
    applyMotorCommandToWheel((WheelIndex)i, finalPwm[i]);
  }
}

void setAllTargetRpm(float rpmValue) {
  for (int i = 0; i < WHEEL_COUNT; i++) {
    targetRpm[i] = rpmValue;
  }
}

void resetVelocityControllerState() {
  for (int i = 0; i < WHEEL_COUNT; i++) {
    velocityError[i] = 0.0f;
    velocityIntegral[i] = 0.0f;
    velocityDerivative[i] = 0.0f;
    previousVelocityError[i] = 0.0f;
    outputPwmUnclamped[i] = 0.0f;
    finalPwm[i] = 0;
    appliedPwm[i] = 0;
    targetRpm[i] = 0.0f;
  }
}

void resetHeadingControllerState() {
  baseForwardRpm = 0.0f;
  rawBaseForwardRpm = 0.0f;
  minimumMoveRpmActive = false;
  turnCorrectionRPM = 0.0f;
  leftRpmComposed = 0.0f;
  rightRpmComposed = 0.0f;
  targetHeadingRad = 0.0f;
  currentHeadingRad = 0.0f;
  headingErrorRad = 0.0f;
  headingIntegralRadS = 0.0f;
}

void resetPositionControllerState() {
  targetDistanceM = 0.0f;
  currentDistanceM = 0.0f;
  distanceErrorM = 0.0f;
  positionIntegral = 0.0f;
  positionModeActive = false;
  motionMode = MODE_IDLE;
}

void setIdleStateAndStopMotors() {
  setAllTargetRpm(0.0f);
  resetVelocityControllerState();
  resetPositionControllerState();
  resetHeadingControllerState();
  stopAllMotorHardware();
}

// -----------------------------
// Encoder ISR logic
// -----------------------------
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

// -----------------------------
// Move start / stop
// -----------------------------
void initializeNewMove(float distanceM, bool useManualHeading, float manualHeadingDeg) {
  // Safety first.
  stopAllMotorHardware();

  // Reset control states.
  resetVelocityControllerState();
  resetPositionControllerState();
  resetHeadingControllerState();

  currentDistanceM = 0.0f;
  targetDistanceM = distanceM;
  positionModeActive = true;
  motionMode = MODE_MOVE_FORWARD;

  // Read current yaw once at move start so IMU health is checked before enabling motion.
  currentHeadingRad = app::imuDriver().displayedYawRad();
  if (!isfinite(currentHeadingRad)) {
    imuHealthy = false;
    // First version safety policy: stop safely if heading control would be used.
    setIdleStateAndStopMotors();
    return;
  }

  imuHealthy = true;

  // Heading target mode:
  // - captured: hold the yaw measured at move start
  // - manual: hold the yaw entered by the user (degrees)
  targetHeadingRad = useManualHeading
    ? wrapAngleRad(manualHeadingDeg * (PI_F / 180.0f))
    : currentHeadingRad;
  headingErrorRad = 0.0f;
  headingIntegralRadS = 0.0f;
  turnCorrectionRPM = 0.0f;

  // New distance run starts with fresh encoder baseline.
  noInterrupts();
  for (int i = 0; i < WHEEL_COUNT; i++) {
    lastEncoderCounts[i] = encoderCounts[i];
  }
  interrupts();

  for (int i = 0; i < WHEEL_COUNT; i++) {
    lastDeltaCounts[i] = 0;
  }
}

void emergencyStopAndReset() {
  setIdleStateAndStopMotors();
}

// -----------------------------
// Control computations
// -----------------------------
void computeWheelRpmFromDelta(long deltaCount, float dtSec, float &rpmOut) {
  if (dtSec > 0.0f) {
    rpmOut = (deltaCount / PPR) * (60.0f / dtSec);
  } else {
    rpmOut = 0.0f;
  }
}

float deltaCountsToDistanceMeters(long deltaCount) {
  return (deltaCount / PPR) * WHEEL_CIRCUMFERENCE_M;
}

// Position loop output is BASE FORWARD RPM only (not PWM).
// The velocity loops later convert RPM targets into PWM commands.
const char *modeToString(MotionMode mode) {
  switch (mode) {
    case MODE_MOVE_FORWARD: return "MOVE_FORWARD";
    default: return "IDLE";
  }
}

void runPositionLoop(float dtSec) {
  if (!positionModeActive || motionMode != MODE_MOVE_FORWARD) {
    baseForwardRpm = 0.0f;
    rawBaseForwardRpm = 0.0f;
    minimumMoveRpmActive = false;
    return;
  }

  distanceErrorM = targetDistanceM - currentDistanceM;

  // Stop when target is reached or crossed.
  bool reachedByTolerance = fabs(distanceErrorM) <= POSITION_TOLERANCE;
  bool crossedTarget =
    (targetDistanceM >= 0.0f && currentDistanceM >= targetDistanceM) ||
    (targetDistanceM < 0.0f && currentDistanceM <= targetDistanceM);

  if (reachedByTolerance || crossedTarget) {
    // Stop immediately at target distance. No rotate-after-move behavior.
    baseForwardRpm = 0.0f;
    rawBaseForwardRpm = 0.0f;
    minimumMoveRpmActive = false;
    turnCorrectionRPM = 0.0f;
    setAllTargetRpm(0.0f);
    for (int i = 0; i < WHEEL_COUNT; i++) {
      finalPwm[i] = 0;
      appliedPwm[i] = 0;
    }
    positionIntegral = 0.0f;
    headingErrorRad = 0.0f;
    headingIntegralRadS = 0.0f;
    resetVelocityControllerState();
    positionModeActive = false;
    motionMode = MODE_IDLE;
    stopAllMotorHardware();
    return;
  }

  positionIntegral += distanceErrorM * dtSec;

  // Position PI controller: distance error -> forward linear speed command.
  // Then convert linear speed to wheel RPM for the next stage.
  float vxCommand = (Kp_pos * distanceErrorM) + (Ki_pos * positionIntegral);
  vxCommand = constrain(vxCommand, -MAX_VX, MAX_VX);

  rawBaseForwardRpm = (vxCommand / WHEEL_CIRCUMFERENCE_M) * 60.0f;

  // Motion-layer deadzone handling:
  // - stop cleanly when close enough
  // - otherwise move with at least MIN_MOVE_RPM magnitude
  minimumMoveRpmActive = false;
  if (fabs(distanceErrorM) <= POSITION_TOLERANCE) {
    baseForwardRpm = 0.0f;
  } else if (fabs(rawBaseForwardRpm) > 0.001f && fabs(rawBaseForwardRpm) < MIN_MOVE_RPM) {
    baseForwardRpm = (rawBaseForwardRpm > 0.0f) ? MIN_MOVE_RPM : -MIN_MOVE_RPM;
    minimumMoveRpmActive = true;
  } else {
    baseForwardRpm = rawBaseForwardRpm;
  }
}

// Heading loop output is TURN correction RPM only (not PWM).
// Positive/negative correction is mixed with baseForwardRpm so left/right
// wheels receive slightly different RPM targets to cancel yaw drift.
void runHeadingLoopAndComposeWheelTargets() {
  // Default if heading correction is not active.
  turnCorrectionRPM = 0.0f;
  leftRpmComposed = 0.0f;
  rightRpmComposed = 0.0f;

  if (!positionModeActive) {
    headingIntegralRadS = 0.0f;
    setAllTargetRpm(0.0f);
    return;
  }

  if (motionMode == MODE_MOVE_FORWARD) {
    if (headingControlEnabled) {
      headingErrorRad = wrapAngleRad(targetHeadingRad - currentHeadingRad);

      headingIntegralRadS += headingErrorRad * currentDtSeconds;
      if (headingIntegralRadS > HEADING_INTEGRAL_LIMIT_RAD_S) headingIntegralRadS = HEADING_INTEGRAL_LIMIT_RAD_S;
      if (headingIntegralRadS < -HEADING_INTEGRAL_LIMIT_RAD_S) headingIntegralRadS = -HEADING_INTEGRAL_LIMIT_RAD_S;

      turnCorrectionRPM =
        (Kp_heading_rpm * headingErrorRad) +
        (Ki_heading_rpm_per_rad_s * headingIntegralRadS);
      turnCorrectionRPM = constrain(
        turnCorrectionRPM,
        -MAX_TURN_CORRECTION_RPM,
        MAX_TURN_CORRECTION_RPM
      );

      if (invertHeadingCorrection) {
        turnCorrectionRPM = -turnCorrectionRPM;
      }
    } else {
      headingErrorRad = 0.0f;
      headingIntegralRadS = 0.0f;
      turnCorrectionRPM = 0.0f;
    }

    const float leftRpm = clampWheelTargetRpm(baseForwardRpm - turnCorrectionRPM);
    const float rightRpm = clampWheelTargetRpm(baseForwardRpm + turnCorrectionRPM);
    leftRpmComposed = leftRpm;
    rightRpmComposed = rightRpm;

    targetRpm[WHEEL_F1] = leftRpm;
    targetRpm[WHEEL_R1] = leftRpm;
    targetRpm[WHEEL_F2] = rightRpm;
    targetRpm[WHEEL_R2] = rightRpm;
    return;
  }

  // IDLE fallback.
  setAllTargetRpm(0.0f);
  turnCorrectionRPM = 0.0f;
}

void runVelocityLoopForWheel(WheelIndex wheel, float dtSec) {
  int i = (int)wheel;
  velocityError[i] = targetRpm[i] - measuredRpm[i];

  if (dtSec > 0.0f) {
    velocityDerivative[i] = (velocityError[i] - previousVelocityError[i]) / dtSec;
  } else {
    velocityDerivative[i] = 0.0f;
  }

  float candidateIntegral = velocityIntegral[i] + (velocityError[i] * dtSec);
  if (candidateIntegral > VELOCITY_INTEGRAL_LIMIT) candidateIntegral = VELOCITY_INTEGRAL_LIMIT;
  if (candidateIntegral < -VELOCITY_INTEGRAL_LIMIT) candidateIntegral = -VELOCITY_INTEGRAL_LIMIT;

  float candidateOutput =
    (kpVel[i] * velocityError[i]) +
    (kiVel[i] * candidateIntegral) +
    (kdVel[i] * velocityDerivative[i]);

  int candidateFinal = clampSignedPwm((int)candidateOutput);

  bool blockIntegralGrowth =
    (candidateFinal >= PWM_MAX && velocityError[i] > 0.0f) ||
    (candidateFinal <= -PWM_MAX && velocityError[i] < 0.0f);

  if (!blockIntegralGrowth) {
    velocityIntegral[i] = candidateIntegral;
  }

  outputPwmUnclamped[i] =
    (kpVel[i] * velocityError[i]) +
    (kiVel[i] * velocityIntegral[i]) +
    (kdVel[i] * velocityDerivative[i]);

  finalPwm[i] = clampSignedPwm((int)outputPwmUnclamped[i]);
  previousVelocityError[i] = velocityError[i];
}

// -----------------------------
// Command parser (Serial + Web)
// -----------------------------
// Supported commands:
// - move 0.50
// - move 0.50 15
// - stop
// - heading on
// - heading off
// - hkp 30
// - hki 2
// - hmax 15
// - hinvert
bool executeCommandLine(const String &commandLine, String &response) {
  String cmd = commandLine;
  cmd.trim();

  if (cmd.length() == 0) {
    response = "Empty command";
    return false;
  }

  int splitPos = cmd.indexOf(' ');
  String key = (splitPos >= 0) ? cmd.substring(0, splitPos) : cmd;
  key.toLowerCase();
  String arg = (splitPos >= 0) ? cmd.substring(splitPos + 1) : "";
  arg.trim();

  if (!lockState(pdMS_TO_TICKS(50))) {
    response = "State lock busy";
    return false;
  }

  if (key == "move") {
    bool useManualHeading = false;
    float manualHeadingDeg = 0.0f;

    int sep = arg.indexOf(' ');
    String distText = (sep >= 0) ? arg.substring(0, sep) : arg;
    distText.trim();
    float d = distText.toFloat();

    if (sep >= 0) {
      String headingText = arg.substring(sep + 1);
      headingText.trim();
      if (headingText.length() > 0) {
        useManualHeading = true;
        manualHeadingDeg = headingText.toFloat();
      }
    }

    if (fabs(d) < 0.0001f) {
      emergencyStopAndReset();
      response = "Move value near zero, system stopped";
    } else {
      initializeNewMove(d, useManualHeading, manualHeadingDeg);
      response = useManualHeading
        ? String("Move started with target heading ") + String(manualHeadingDeg, 2) + " deg"
        : "Move started with captured heading";
    }
    unlockState();
    return true;
  }

  if (key == "stop") {
    emergencyStopAndReset();
    unlockState();
    response = "Stopped";
    return true;
  }

  if (key == "heading") {
    String opt = arg;
    opt.toLowerCase();
    if (opt == "on") {
      headingControlEnabled = true;
      response = "Heading control ON";
    } else if (opt == "off") {
      headingControlEnabled = false;
      turnCorrectionRPM = 0.0f;
      response = "Heading control OFF";
    } else {
      unlockState();
      response = "Use: heading on | heading off";
      return false;
    }
    unlockState();
    return true;
  }

  if (key == "hkp") {
    // Beginner tuning tip:
    // - Increase hkp if robot still drifts heading.
    // - Decrease hkp if robot oscillates left/right.
    Kp_heading_rpm = arg.toFloat();
    if (Kp_heading_rpm < 0.0f) Kp_heading_rpm = 0.0f;
    response = "Heading Kp updated";
    unlockState();
    return true;
  }

  if (key == "hki") {
    Ki_heading_rpm_per_rad_s = arg.toFloat();
    if (Ki_heading_rpm_per_rad_s < 0.0f) Ki_heading_rpm_per_rad_s = 0.0f;
    response = "Heading Ki updated";
    unlockState();
    return true;
  }

  if (key == "hmax") {
    // hmax limits correction strength for safety and smoothness.
    // If turns are too weak, increase hmax. If too aggressive, reduce it.
    MAX_TURN_CORRECTION_RPM = arg.toFloat();
    if (MAX_TURN_CORRECTION_RPM < 0.0f) MAX_TURN_CORRECTION_RPM = 0.0f;
    if (MAX_TURN_CORRECTION_RPM > 50.0f) MAX_TURN_CORRECTION_RPM = 50.0f;
    response = "Heading max correction updated";
    unlockState();
    return true;
  }

  if (key == "hinvert") {
    // Use hinvert if heading correction direction is reversed.
    // Example symptom: robot drifts right and controller makes it drift more right.
    invertHeadingCorrection = !invertHeadingCorrection;
    response = String("Heading invert is now ") + (invertHeadingCorrection ? "ON" : "OFF");
    unlockState();
    return true;
  }

  unlockState();
  response = "Unknown command";
  return false;
}

void pollSerialCommands() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialLineBuffer.length() > 0) {
        String resp;
        bool ok = executeCommandLine(serialLineBuffer, resp);
        Serial.print(ok ? "[OK] " : "[ERR] ");
        Serial.println(resp);
        serialLineBuffer = "";
      }
    } else {
      serialLineBuffer += c;
      if (serialLineBuffer.length() > 120) {
        serialLineBuffer = "";
      }
    }
  }
}

void imuUpdateTask(void *parameter) {
  const TickType_t imuPeriodTicks = pdMS_TO_TICKS(app::IMU_READ_PERIOD_MS);
  TickType_t lastWakeTime = xTaskGetTickCount();

  while (true) {
    vTaskDelayUntil(&lastWakeTime, imuPeriodTicks);

    if (!lockState(pdMS_TO_TICKS(5))) {
      continue;
    }

    app::IMUState imuState;
    const bool ok = app::imuDriver().read(imuState);
    imuHealthy = ok;
    if (ok) {
      imuLastOkReadMs = millis();
      imuRawGyroZDegPerSec = app::radToDeg(app::imuDriver().rawGyroZRadPerSec());
      imuCorrectedGyroZDegPerSec = app::radToDeg(app::imuDriver().correctedGyroZRadPerSec());
      imuBiasZDegPerSec = app::radToDeg(app::imuDriver().gyroBiasZRadPerSec());
      imuIntegratedYawDeg = app::radToDeg(app::imuDriver().yawIntegratedRad());
      imuZeroOffsetYawDeg = app::radToDeg(app::imuDriver().yawZeroOffsetRad());
      imuMotionState = app::imuDriver().motionStateName();
    }

    unlockState();
  }
}

// -----------------------------
// RTOS control task
// -----------------------------
void controlLoopTask(void *parameter) {
  TickType_t lastWakeTime = xTaskGetTickCount();

  while (true) {
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(CONTROL_PERIOD_MS));

    unsigned long now = millis();
    float dtSec = (now - lastControlTime) / 1000.0f;
    lastControlTime = now;
    if (dtSec <= 0.0f) dtSec = CONTROL_PERIOD_SECONDS;

    if (!lockState()) {
      continue;
    }

    currentDtSeconds = dtSec;

    // Update IMU exactly once per control cycle.
    // This is the only place that advances yaw integration in runtime.
    // Web/status/getters only read already-updated state.
    if (!imuHealthy) {
      setIdleStateAndStopMotors();
      unlockState();
      continue;
    }

    currentHeadingRad = app::imuDriver().displayedYawRad();
    if (!isfinite(currentHeadingRad)) {
      imuHealthy = false;
      setIdleStateAndStopMotors();
      unlockState();
      continue;
    }

    // Required timing order:
    // 1) Update/read IMU yaw once
    // 2) Copy encoder counts safely
    // 3) Compute measured wheel RPM
    // 4) Compute current forward distance
    // 5) Position loop => baseForwardRpm
    // 6) Heading loop => turnCorrectionRPM and per-wheel targets
    // 7) Velocity loops => PWM
    // 8) Apply PWM

    long copiedCounts[WHEEL_COUNT];
    noInterrupts();
    for (int i = 0; i < WHEEL_COUNT; i++) {
      copiedCounts[i] = encoderCounts[i];
    }
    interrupts();

    float avgDeltaDistanceM = 0.0f;
    for (int i = 0; i < WHEEL_COUNT; i++) {
      long delta = copiedCounts[i] - lastEncoderCounts[i];
      lastEncoderCounts[i] = copiedCounts[i];
      lastDeltaCounts[i] = delta;

      computeWheelRpmFromDelta(delta, dtSec, measuredRpm[i]);
      avgDeltaDistanceM += deltaCountsToDistanceMeters(delta);
    }

    avgDeltaDistanceM /= (float)WHEEL_COUNT;
    currentDistanceM += avgDeltaDistanceM;

    runPositionLoop(dtSec);

    if (!positionModeActive) {
      // Hold hard stop when idle.
      resetVelocityControllerState();
      stopAllMotorHardware();
      unlockState();
      continue;
    }

    runHeadingLoopAndComposeWheelTargets();

    if (!positionModeActive) {
      // Could become inactive if IMU safety stop happened.
      resetVelocityControllerState();
      stopAllMotorHardware();
      unlockState();
      continue;
    }

    for (int i = 0; i < WHEEL_COUNT; i++) {
      runVelocityLoopForWheel((WheelIndex)i, dtSec);
    }

    applyAllWheelMotorCommands();
    unlockState();
  }
}

// -----------------------------
// Web status / commands
// -----------------------------
String jsonEscape(const char *text) {
  return String(text);
}

String buildStatusJson() {
  if (!lockState(pdMS_TO_TICKS(50))) {
    return "{\"ok\":false}";
  }

  long encoderSnapshot[WHEEL_COUNT];
  noInterrupts();
  for (int i = 0; i < WHEEL_COUNT; i++) {
    encoderSnapshot[i] = encoderCounts[i];
  }
  interrupts();

  const int imuUpdateAgeMs = (imuLastOkReadMs == 0U) ? -1 : static_cast<int>(millis() - imuLastOkReadMs);

  String json = "{";
  json += "\"ok\":true,";
  json += "\"positionModeActive\":" + String(positionModeActive ? "true" : "false") + ",";
  json += "\"motionMode\":\"" + String(modeToString(motionMode)) + "\",";
  json += "\"headingControlEnabled\":" + String(headingControlEnabled ? "true" : "false") + ",";
  json += "\"imuHealthy\":" + String(imuHealthy ? "true" : "false") + ",";
  json += "\"imuUpdateAgeMs\":" + String(imuUpdateAgeMs) + ",";
  json += "\"imuMotionState\":\"" + String(imuMotionState) + "\",";
  json += "\"imuRawGyroZDegPerSec\":" + String(imuRawGyroZDegPerSec, 3) + ",";
  json += "\"imuCorrectedGyroZDegPerSec\":" + String(imuCorrectedGyroZDegPerSec, 3) + ",";
  json += "\"imuBiasZDegPerSec\":" + String(imuBiasZDegPerSec, 3) + ",";
  json += "\"imuIntegratedYawDeg\":" + String(imuIntegratedYawDeg, 3) + ",";
  json += "\"imuZeroOffsetYawDeg\":" + String(imuZeroOffsetYawDeg, 3) + ",";

  json += "\"targetDistanceM\":" + String(targetDistanceM, 4) + ",";
  json += "\"currentDistanceM\":" + String(currentDistanceM, 4) + ",";
  json += "\"distanceErrorM\":" + String(distanceErrorM, 4) + ",";

  json += "\"baseRPM\":" + String(baseForwardRpm, 3) + ",";
  json += "\"rawBaseRPM\":" + String(rawBaseForwardRpm, 3) + ",";
  json += "\"minimumMoveRpmActive\":" + String(minimumMoveRpmActive ? "true" : "false") + ",";
  json += "\"leftRpmComposed\":" + String(leftRpmComposed, 3) + ",";
  json += "\"rightRpmComposed\":" + String(rightRpmComposed, 3) + ",";
  json += "\"leftRpm\":" + String(leftRpmComposed, 3) + ",";
  json += "\"rightRpm\":" + String(rightRpmComposed, 3) + ",";
  json += "\"currentHeadingDeg\":" + String(radToDeg(currentHeadingRad), 3) + ",";
  json += "\"targetHeadingDeg\":" + String(radToDeg(targetHeadingRad), 3) + ",";
  json += "\"headingErrorDeg\":" + String(radToDeg(headingErrorRad), 3) + ",";

  json += "\"Kp_heading_rpm\":" + String(Kp_heading_rpm, 3) + ",";
  json += "\"Ki_heading_rpm_per_rad_s\":" + String(Ki_heading_rpm_per_rad_s, 3) + ",";
  json += "\"headingIntegralRadS\":" + String(headingIntegralRadS, 4) + ",";
  json += "\"MAX_TURN_CORRECTION_RPM\":" + String(MAX_TURN_CORRECTION_RPM, 3) + ",";
  json += "\"turnCorrectionRPM\":" + String(turnCorrectionRPM, 3) + ",";
  json += "\"invertHeadingCorrection\":" + String(invertHeadingCorrection ? "true" : "false") + ",";

  json += "\"targetRPM_F1\":" + String(targetRpm[WHEEL_F1], 3) + ",";
  json += "\"targetRPM_F2\":" + String(targetRpm[WHEEL_F2], 3) + ",";
  json += "\"targetRPM_R1\":" + String(targetRpm[WHEEL_R1], 3) + ",";
  json += "\"targetRPM_R2\":" + String(targetRpm[WHEEL_R2], 3) + ",";

  json += "\"measuredRPM_F1\":" + String(measuredRpm[WHEEL_F1], 3) + ",";
  json += "\"measuredRPM_F2\":" + String(measuredRpm[WHEEL_F2], 3) + ",";
  json += "\"measuredRPM_R1\":" + String(measuredRpm[WHEEL_R1], 3) + ",";
  json += "\"measuredRPM_R2\":" + String(measuredRpm[WHEEL_R2], 3) + ",";

  json += "\"finalPWM_F1\":" + String(finalPwm[WHEEL_F1]) + ",";
  json += "\"finalPWM_F2\":" + String(finalPwm[WHEEL_F2]) + ",";
  json += "\"finalPWM_R1\":" + String(finalPwm[WHEEL_R1]) + ",";
  json += "\"finalPWM_R2\":" + String(finalPwm[WHEEL_R2]) + ",";

  json += "\"dtSeconds\":" + String(currentDtSeconds, 4) + ",";

  json += "\"wheels\":[";
  for (int i = 0; i < WHEEL_COUNT; i++) {
    if (i > 0) json += ",";
    json += "{";
    json += "\"name\":\"" + jsonEscape(wheels[i].name) + "\",";
    json += "\"encoderCount\":" + String(encoderSnapshot[i]) + ",";
    json += "\"deltaCount\":" + String(lastDeltaCounts[i]) + ",";
    json += "\"measuredRpm\":" + String(measuredRpm[i], 3) + ",";
    json += "\"targetRpm\":" + String(targetRpm[i], 3) + ",";
    json += "\"errorRpm\":" + String(velocityError[i], 3) + ",";
    json += "\"integralTerm\":" + String(velocityIntegral[i], 4) + ",";
    json += "\"derivativeTerm\":" + String(velocityDerivative[i], 4) + ",";
    json += "\"outputPwm\":" + String(outputPwmUnclamped[i], 3) + ",";
    json += "\"finalPwm\":" + String(finalPwm[i]) + ",";
    json += "\"appliedPwm\":" + String(appliedPwm[i]);
    json += "}";
  }
  json += "]";
  json += "}";

  unlockState();
  return json;
}

void sendOkResponse(const String &text = "OK") {
  server.send(200, "text/plain", text);
}

void handleStatus() {
  server.send(200, "application/json", buildStatusJson());
}

void handleMove() {
  if (!server.hasArg("distance")) {
    server.send(400, "text/plain", "Missing distance");
    return;
  }

  String cmd = String("move ") + server.arg("distance");
  if (server.hasArg("targetHeadingDeg")) {
    cmd += " ";
    cmd += server.arg("targetHeadingDeg");
  }

  String resp;
  executeCommandLine(cmd, resp);
  sendOkResponse(resp);
}

void handleStop() {
  String resp;
  executeCommandLine("stop", resp);
  sendOkResponse(resp);
}

void handleHeadingEnable() {
  if (!server.hasArg("enabled")) {
    server.send(400, "text/plain", "Missing enabled");
    return;
  }
  String enabled = server.arg("enabled");
  enabled.toLowerCase();
  String resp;
  executeCommandLine(enabled == "1" || enabled == "true" ? "heading on" : "heading off", resp);
  sendOkResponse(resp);
}

void handleSetHkp() {
  if (!server.hasArg("value")) {
    server.send(400, "text/plain", "Missing value");
    return;
  }
  String resp;
  executeCommandLine(String("hkp ") + server.arg("value"), resp);
  sendOkResponse(resp);
}

void handleSetHki() {
  if (!server.hasArg("value")) {
    server.send(400, "text/plain", "Missing value");
    return;
  }
  String resp;
  executeCommandLine(String("hki ") + server.arg("value"), resp);
  sendOkResponse(resp);
}

void handleSetHmax() {
  if (!server.hasArg("value")) {
    server.send(400, "text/plain", "Missing value");
    return;
  }
  String resp;
  executeCommandLine(String("hmax ") + server.arg("value"), resp);
  sendOkResponse(resp);
}

void handleToggleHinvert() {
  String resp;
  executeCommandLine("hinvert", resp);
  sendOkResponse(resp);
}

void handleCommand() {
  if (!server.hasArg("line")) {
    server.send(400, "text/plain", "Missing line");
    return;
  }
  String resp;
  bool ok = executeCommandLine(server.arg("line"), resp);
  server.send(ok ? 200 : 400, "text/plain", resp);
}

const char WEB_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Forward Position + Heading Test</title>
  <style>
    body { font-family: "Trebuchet MS", "Segoe UI", sans-serif; margin: 0; background: #f4f8fc; color: #173043; }
    .wrap { max-width: 980px; margin: 0 auto; padding: 18px; }
    .panel { background: white; border: 1px solid #d7e3ec; border-radius: 14px; padding: 16px; margin-bottom: 14px; }
    .row { display: grid; grid-template-columns: 1fr 1fr 1fr; gap: 10px; }
    .field { display: flex; flex-direction: column; gap: 6px; }
    .field label { font-size: 0.85rem; color: #4e6678; font-weight: 700; }
    .checkline { display: flex; align-items: center; gap: 8px; font-size: 0.92rem; color: #27465e; }
    .checkline input[type="checkbox"] { width: 18px; height: 18px; }
    input, button { padding: 10px; border-radius: 10px; border: 1px solid #c6d7e4; font-size: 1rem; }
    button { cursor: pointer; font-weight: 700; }
    .primary { background: #e66b2f; color: white; border: 0; }
    .danger { background: #d65555; color: white; border: 0; }
    .secondary { background: #2f7dbd; color: white; border: 0; }
    .status-grid { display: grid; grid-template-columns: repeat(3, 1fr); gap: 10px; }
    .card { border: 1px solid #d7e3ec; border-radius: 12px; padding: 10px; background: #fbfdff; }
    .lbl { color: #5b6f7f; font-size: 0.84rem; text-transform: uppercase; }
    .val { font-size: 1.1rem; font-weight: 700; }
    .hint { color: #5b6f7f; font-size: 0.9rem; margin-top: 8px; }
    .logger-toolbar { display: flex; gap: 10px; flex-wrap: wrap; margin-top: 8px; }
    .logger-meta { color: #5b6f7f; font-size: 0.9rem; margin-top: 6px; }
    #logOutput { width: 100%; min-height: 210px; margin-top: 10px; border-radius: 10px; border: 1px solid #c6d7e4; padding: 10px; font-family: Consolas, monospace; font-size: 0.85rem; box-sizing: border-box; }
    @media (max-width: 760px) { .row, .status-grid { grid-template-columns: 1fr; } }
  </style>
</head>
<body>
  <div class="wrap">
    <div class="panel">
      <h2>Forward Position + Heading Hold</h2>
      <div class="row">
        <div class="field">
          <label for="distanceInput">Distance Command (m)</label>
          <input id="distanceInput" type="number" step="0.01" value="0.50" placeholder="+ forward, - backward">
        </div>
        <div class="field">
          <label for="targetHeadingInput">Target Heading (deg)</label>
          <input id="targetHeadingInput" type="number" step="1" value="0" placeholder="Used when manual heading is enabled">
        </div>
        <button class="primary" onclick="moveCmd()">move</button>
      </div>
      <div class="row" style="margin-top:10px;">
        <div class="checkline">
          <input id="useManualHeadingInput" type="checkbox">
          <label for="useManualHeadingInput">Use manual target heading from GUI (otherwise capture heading at move start)</label>
        </div>
      </div>
      <div class="row" style="margin-top:10px;">
        <button class="danger" onclick="stopCmd()">stop</button>
      </div>
      <div class="row" style="margin-top:10px;">
        <button class="secondary" onclick="headingCmd(true)">heading on</button>
        <button class="secondary" onclick="headingCmd(false)">heading off</button>
        <button class="secondary" onclick="hinvertCmd()">hinvert</button>
      </div>
      <div class="row" style="margin-top:10px;">
        <div class="field">
          <label for="hkpInput">Heading Kp (RPM per rad)</label>
          <input id="hkpInput" type="number" step="0.1" value="60" placeholder="Example: 30 to 90">
        </div>
        <div class="field">
          <label for="hkiInput">Heading Ki (RPM per rad*s)</label>
          <input id="hkiInput" type="number" step="0.1" value="2" placeholder="Example: 0 to 5">
        </div>
        <div class="field">
          <label for="hmaxInput">Heading Max Correction (RPM)</label>
          <input id="hmaxInput" type="number" step="0.1" value="30" placeholder="Example: 15 to 40 (capped at 50)">
        </div>
        <button class="secondary" onclick="applyHeadingGains()">apply hkp/hki/hmax</button>
      </div>
      <p id="msg">Idle</p>
      <div class="hint">Commands also available via Serial: move 0.50, stop, heading on/off, hkp 30, hmax 15, hinvert</div>
    </div>

    <div class="panel">
      <div class="status-grid">
        <div class="card"><div class="lbl">Position Active</div><div class="val" id="activeVal">false</div></div>
        <div class="card"><div class="lbl">Motion Mode</div><div class="val" id="modeVal">IDLE</div></div>
        <div class="card"><div class="lbl">Heading Enabled</div><div class="val" id="headingEnabledVal">false</div></div>
        <div class="card"><div class="lbl">IMU Healthy</div><div class="val" id="imuHealthyVal">false</div></div>

        <div class="card"><div class="lbl">Target Distance (m)</div><div class="val" id="targetDistVal">0.0000</div></div>
        <div class="card"><div class="lbl">Current Distance (m)</div><div class="val" id="currentDistVal">0.0000</div></div>
        <div class="card"><div class="lbl">Distance Error (m)</div><div class="val" id="errorVal">0.0000</div></div>

        <div class="card"><div class="lbl">Base RPM</div><div class="val" id="baseRpmVal">0.000</div></div>
        <div class="card"><div class="lbl">Turn Correction RPM</div><div class="val" id="turnVal">0.000</div></div>
        <div class="card"><div class="lbl">dt (s)</div><div class="val" id="dtVal">0.0000</div></div>

        <div class="card"><div class="lbl">Target Heading (deg)</div><div class="val" id="targetHeadingVal">0.000</div></div>
        <div class="card"><div class="lbl">Heading Control</div><div class="val" id="headingModeVal">HOLD_START_YAW</div></div>
        <div class="card"><div class="lbl">Current Heading (deg)</div><div class="val" id="currentHeadingVal">0.000</div></div>
        <div class="card"><div class="lbl">Heading Error (deg)</div><div class="val" id="headingErrVal">0.000</div></div>

        <div class="card"><div class="lbl">Kp Heading</div><div class="val" id="hkpVal">60.0</div></div>
        <div class="card"><div class="lbl">Ki Heading</div><div class="val" id="hkiVal">2.0</div></div>
        <div class="card"><div class="lbl">Hmax</div><div class="val" id="hmaxVal">30.0</div></div>
        <div class="card"><div class="lbl">Invert Heading</div><div class="val" id="hinvertVal">false</div></div>
      </div>
    </div>

    <div class="panel">
      <h3 style="margin-top:0;">Heading Response Graph</h3>
      <div class="hint">Blue: target heading, orange: current heading, red: heading error</div>
      <canvas id="headingChart" width="940" height="260" style="width:100%; max-width:100%; border:1px solid #d7e3ec; border-radius:10px; background:#ffffff;"></canvas>
    </div>

    <div class="panel">
      <h3 style="margin-top:0;">Run Logger</h3>
      <div class="hint">Logs heading and motion telemetry while a move is active. Use Download CSV to analyze in Excel/MATLAB/Python.</div>
      <div id="loggerMeta" class="logger-meta">Logger idle.</div>
      <div class="logger-toolbar">
        <button class="secondary" onclick="copyLogText()">copy log</button>
        <button class="secondary" onclick="downloadLogCsv()">download csv</button>
        <button onclick="clearLogText()">clear log box</button>
      </div>
      <textarea id="logOutput" readonly spellcheck="false"></textarea>
    </div>
  </div>

  <script>
    const MAX_GRAPH_POINTS = 150;
    const headingSeries = [];
    const targetHeadingSeries = [];
    const errorHeadingSeries = [];
    let loggingActive = false;
    let logStartTimeMs = 0;
    let logLineCount = 0;
    let lastActiveState = false;

    async function moveCmd() {
      const d = Number(document.getElementById('distanceInput').value || 0);
      const useManual = document.getElementById('useManualHeadingInput').checked;
      const headingDeg = Number(document.getElementById('targetHeadingInput').value || 0);

      const qs = new URLSearchParams();
      qs.set('distance', String(d));
      if (useManual) {
        qs.set('targetHeadingDeg', String(headingDeg));
      }

      await fetch(`/move?${qs.toString()}`, { cache: 'no-store' });
      document.getElementById('msg').textContent = useManual
        ? `move ${d.toFixed(2)} with target heading ${headingDeg.toFixed(1)} deg`
        : `move ${d.toFixed(2)} with captured heading`;
      refreshStatus();
    }

    async function stopCmd() {
      await fetch('/stop', { cache: 'no-store' });
      document.getElementById('msg').textContent = 'stop';
      refreshStatus();
    }

    async function headingCmd(enabled) {
      await fetch(`/heading?enabled=${enabled ? 1 : 0}`, { cache: 'no-store' });
      document.getElementById('msg').textContent = enabled ? 'heading on' : 'heading off';
      refreshStatus();
    }

    async function hinvertCmd() {
      await fetch('/hinvert', { cache: 'no-store' });
      document.getElementById('msg').textContent = 'hinvert';
      refreshStatus();
    }

    async function applyHeadingGains() {
      const hkp = Number(document.getElementById('hkpInput').value || 0);
      const hki = Number(document.getElementById('hkiInput').value || 0);
      const hmax = Number(document.getElementById('hmaxInput').value || 0);
      await fetch(`/hkp?value=${encodeURIComponent(hkp)}`, { cache: 'no-store' });
      await fetch(`/hki?value=${encodeURIComponent(hki)}`, { cache: 'no-store' });
      await fetch(`/hmax?value=${encodeURIComponent(hmax)}`, { cache: 'no-store' });
      document.getElementById('msg').textContent = `hkp ${hkp}, hki ${hki}, hmax ${hmax}`;
      refreshStatus();
    }

    function setText(id, value) {
      document.getElementById(id).textContent = value;
    }

    function getLogOutput() {
      return document.getElementById('logOutput');
    }

    function setLoggerMeta(message) {
      document.getElementById('loggerMeta').textContent = message;
    }

    function appendLogLine(line) {
      const box = getLogOutput();
      box.value += line + '\n';
      box.scrollTop = box.scrollHeight;
    }

    function csvValue(value) {
      const text = String(value);
      if (text.includes(',') || text.includes('"')) {
        return '"' + text.replaceAll('"', '""') + '"';
      }
      return text;
    }

    function formatCsvSample(data, elapsedSeconds) {
      const columns = [
        elapsedSeconds.toFixed(3),
        Number(data.dtSeconds).toFixed(4),
        data.positionModeActive ? 1 : 0,
        data.headingControlEnabled ? 1 : 0,
        data.imuHealthy ? 1 : 0,
        Number(data.imuUpdateAgeMs),
        String(data.imuMotionState),
        Number(data.imuRawGyroZDegPerSec).toFixed(3),
        Number(data.imuCorrectedGyroZDegPerSec).toFixed(3),
        Number(data.imuBiasZDegPerSec).toFixed(3),
        Number(data.imuIntegratedYawDeg).toFixed(3),
        Number(data.imuZeroOffsetYawDeg).toFixed(3),
        Number(data.targetDistanceM).toFixed(4),
        Number(data.currentDistanceM).toFixed(4),
        Number(data.distanceErrorM).toFixed(4),
        Number(data.baseRPM).toFixed(3),
        Number(data.rawBaseRPM).toFixed(3),
        data.minimumMoveRpmActive ? 1 : 0,
        Number(data.turnCorrectionRPM).toFixed(3),
        Number(data.leftRpmComposed).toFixed(3),
        Number(data.rightRpmComposed).toFixed(3),
        Number(data.targetHeadingDeg).toFixed(3),
        Number(data.currentHeadingDeg).toFixed(3),
        Number(data.headingErrorDeg).toFixed(3),
        Number(data.Kp_heading_rpm).toFixed(3),
        Number(data.Ki_heading_rpm_per_rad_s).toFixed(3),
        Number(data.headingIntegralRadS).toFixed(4),
        Number(data.MAX_TURN_CORRECTION_RPM).toFixed(3),
        data.invertHeadingCorrection ? 1 : 0,
        Number(data.targetRPM_F1).toFixed(3),
        Number(data.targetRPM_F2).toFixed(3),
        Number(data.targetRPM_R1).toFixed(3),
        Number(data.targetRPM_R2).toFixed(3),
        Number(data.measuredRPM_F1).toFixed(3),
        Number(data.measuredRPM_F2).toFixed(3),
        Number(data.measuredRPM_R1).toFixed(3),
        Number(data.measuredRPM_R2).toFixed(3),
        data.finalPWM_F1,
        data.finalPWM_F2,
        data.finalPWM_R1,
        data.finalPWM_R2
      ];
      return columns.map(csvValue).join(',');
    }

    function startLogging(data, reasonLabel) {
      loggingActive = true;
      logStartTimeMs = Date.now();
      logLineCount = 0;
      appendLogLine('=== Heading Run Started ===');
      appendLogLine(`reason=${reasonLabel} | targetDistanceM=${Number(data.targetDistanceM).toFixed(4)} | hkp=${Number(data.Kp_heading_rpm).toFixed(3)} | hmax=${Number(data.MAX_TURN_CORRECTION_RPM).toFixed(3)} | headingEnabled=${String(data.headingControlEnabled)} | invert=${String(data.invertHeadingCorrection)}`);
      appendLogLine('t_s,dt_s,position_active,heading_enabled,imu_healthy,imu_update_age_ms,imu_motion_state,imu_raw_gyro_z_deg_s,imu_corrected_gyro_z_deg_s,imu_bias_z_deg_s,imu_integrated_yaw_deg,imu_zero_offset_yaw_deg,target_distance_m,current_distance_m,distance_error_m,base_rpm,raw_base_rpm,min_move_rpm_active,turn_rpm,left_rpm,right_rpm,target_heading_deg,current_heading_deg,heading_error_deg,hkp,hki,heading_i_rad_s,hmax,invert,target_rpm_f1,target_rpm_f2,target_rpm_r1,target_rpm_r2,measured_rpm_f1,measured_rpm_f2,measured_rpm_r1,measured_rpm_r2,final_pwm_f1,final_pwm_f2,final_pwm_r1,final_pwm_r2');
      setLoggerMeta('Logging active...');
    }

    function stopLogging(reasonLabel) {
      if (!loggingActive) {
        setLoggerMeta('Logger idle.');
        return;
      }
      const elapsed = (Date.now() - logStartTimeMs) / 1000;
      appendLogLine(`=== Heading Run Stopped | reason=${reasonLabel} | duration=${elapsed.toFixed(2)}s | samples=${logLineCount} ===`);
      appendLogLine('');
      loggingActive = false;
      setLoggerMeta('Logger idle.');
    }

    async function copyLogText() {
      const box = getLogOutput();
      try {
        await navigator.clipboard.writeText(box.value);
        document.getElementById('msg').textContent = 'log copied';
      } catch (e) {
        box.focus();
        box.select();
        document.getElementById('msg').textContent = 'copy failed, selected log for manual copy';
      }
    }

    function downloadLogCsv() {
      const text = getLogOutput().value;
      if (!text.trim()) {
        document.getElementById('msg').textContent = 'log is empty';
        return;
      }
      const blob = new Blob([text], { type: 'text/csv;charset=utf-8' });
      const url = URL.createObjectURL(blob);
      const a = document.createElement('a');
      const stamp = new Date().toISOString().replace(/[:.]/g, '-');
      a.href = url;
      a.download = `heading-log-${stamp}.csv`;
      document.body.appendChild(a);
      a.click();
      document.body.removeChild(a);
      URL.revokeObjectURL(url);
      document.getElementById('msg').textContent = 'csv downloaded';
    }

    function clearLogText() {
      getLogOutput().value = '';
      setLoggerMeta(loggingActive ? 'Logging active...' : 'Logger idle.');
      document.getElementById('msg').textContent = 'log cleared';
    }

    function pushPoint(list, value) {
      list.push(Number(value) || 0);
      if (list.length > MAX_GRAPH_POINTS) list.shift();
    }

    function drawLine(ctx, values, minY, maxY, color, width) {
      if (!values.length) return;
      const w = ctx.canvas.width;
      const h = ctx.canvas.height;
      const denom = Math.max(1, values.length - 1);
      const range = Math.max(0.001, maxY - minY);
      ctx.beginPath();
      for (let i = 0; i < values.length; i++) {
        const x = (i / denom) * w;
        const y = h - ((values[i] - minY) / range) * h;
        if (i === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      }
      ctx.strokeStyle = color;
      ctx.lineWidth = width;
      ctx.stroke();
    }

    function drawHeadingGraph() {
      const canvas = document.getElementById('headingChart');
      if (!canvas) return;
      const ctx = canvas.getContext('2d');
      if (!ctx) return;

      const allValues = targetHeadingSeries.concat(headingSeries, errorHeadingSeries);
      if (!allValues.length) {
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        return;
      }

      let minY = Math.min(...allValues);
      let maxY = Math.max(...allValues);
      if (Math.abs(maxY - minY) < 1.0) {
        minY -= 0.5;
        maxY += 0.5;
      }
      const pad = Math.max(1.0, (maxY - minY) * 0.10);
      minY -= pad;
      maxY += pad;

      ctx.clearRect(0, 0, canvas.width, canvas.height);

      // Midline for quick visual reference.
      const zeroY = canvas.height - ((0 - minY) / Math.max(0.001, maxY - minY)) * canvas.height;
      if (zeroY >= 0 && zeroY <= canvas.height) {
        ctx.beginPath();
        ctx.moveTo(0, zeroY);
        ctx.lineTo(canvas.width, zeroY);
        ctx.strokeStyle = '#e8edf2';
        ctx.lineWidth = 1;
        ctx.stroke();
      }

      drawLine(ctx, targetHeadingSeries, minY, maxY, '#2f7dbd', 2.0);
      drawLine(ctx, headingSeries, minY, maxY, '#e66b2f', 2.0);
      drawLine(ctx, errorHeadingSeries, minY, maxY, '#d65555', 1.6);

      ctx.fillStyle = '#5b6f7f';
      ctx.font = '12px Trebuchet MS';
      ctx.fillText(`max ${maxY.toFixed(1)} deg`, 8, 14);
      ctx.fillText(`min ${minY.toFixed(1)} deg`, 8, canvas.height - 8);
    }

    async function refreshStatus() {
      try {
        const response = await fetch('/status', { cache: 'no-store' });
        const data = await response.json();
        if (!data.ok) return;

        setText('activeVal', String(data.positionModeActive));
        setText('modeVal', String(data.motionMode));
        setText('headingEnabledVal', String(data.headingControlEnabled));
        setText('imuHealthyVal', String(data.imuHealthy));

        setText('targetDistVal', Number(data.targetDistanceM).toFixed(4));
        setText('currentDistVal', Number(data.currentDistanceM).toFixed(4));
        setText('errorVal', Number(data.distanceErrorM).toFixed(4));

        setText('baseRpmVal', Number(data.baseRPM).toFixed(3));
        setText('turnVal', Number(data.turnCorrectionRPM).toFixed(3));
        setText('dtVal', Number(data.dtSeconds).toFixed(4));

        setText('targetHeadingVal', Number(data.targetHeadingDeg).toFixed(3));
        setText('headingModeVal', 'HOLD_START_YAW');
        setText('currentHeadingVal', Number(data.currentHeadingDeg).toFixed(3));
        setText('headingErrVal', Number(data.headingErrorDeg).toFixed(3));

        setText('hkpVal', Number(data.Kp_heading_rpm).toFixed(3));
        setText('hkiVal', Number(data.Ki_heading_rpm_per_rad_s).toFixed(3));
        setText('hmaxVal', Number(data.MAX_TURN_CORRECTION_RPM).toFixed(3));
        setText('hinvertVal', String(data.invertHeadingCorrection));

        pushPoint(targetHeadingSeries, data.targetHeadingDeg);
        pushPoint(headingSeries, data.currentHeadingDeg);
        pushPoint(errorHeadingSeries, data.headingErrorDeg);
        drawHeadingGraph();

        const isActive = Boolean(data.positionModeActive);
        if (isActive && !lastActiveState) {
          startLogging(data, 'move started');
        } else if (!isActive && lastActiveState) {
          stopLogging('move ended');
        }

        if (loggingActive) {
          const elapsedSeconds = (Date.now() - logStartTimeMs) / 1000;
          appendLogLine(formatCsvSample(data, elapsedSeconds));
          logLineCount++;
          setLoggerMeta(`Logging active | ${logLineCount} samples`);
        }

        lastActiveState = isActive;
      } catch (e) {
        document.getElementById('msg').textContent = 'Connection failed';
      }
    }

    refreshStatus();
    setInterval(refreshStatus, 200);
  </script>
</body>
</html>
)rawliteral";

void handleRoot() {
  server.send_P(200, "text/html", WEB_PAGE);
}

void configureWebServer() {
  server.on("/", handleRoot);
  server.on("/status", handleStatus);
  server.on("/move", handleMove);
  server.on("/stop", handleStop);
  server.on("/heading", handleHeadingEnable);
  server.on("/hkp", handleSetHkp);
  server.on("/hki", handleSetHki);
  server.on("/hmax", handleSetHmax);
  server.on("/hinvert", handleToggleHinvert);
  server.on("/cmd", handleCommand);
  server.begin();
}

void webServerTask(void *parameter) {
  while (true) {
    server.handleClient();
    pollSerialCommands();
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

// -----------------------------
// Setup/loop
// -----------------------------
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

void configureEncoderPins() {
  for (int i = 0; i < WHEEL_COUNT; i++) {
    pinMode(wheels[i].encAPin, INPUT);
    pinMode(wheels[i].encBPin, INPUT);
    attachInterrupt(digitalPinToInterrupt(wheels[i].encAPin), encoderHandlers[i], RISING);
  }
}

/*
Testing plan:
1) Test: heading off + move 0.30
2) Test: heading on  + move 0.30
3) If correction is wrong direction, run: hinvert
4) Tune hkp: 20, 30, 40
5) Tune hmax: 10, 15, 20
6) Then test move 0.50

Tuning hints:
- If robot still curves, increase hkp slowly.
- If robot wiggles, decrease hkp.
- If correction is too aggressive, reduce hmax.
- Keep robot still during IMU startup calibration.
- Gyro-integrated yaw drifts over long runs, but is acceptable for short moves.
*/

void setup() {
  Serial.begin(115200);

  configureMotorPinsSafe();
  stopAllMotorHardware();
  configureEncoderPins();

  // IMU startup.
  // If begin fails, heading control safety behavior will stop motion when needed.
  imuHealthy = app::imuDriver().begin();

  WiFi.mode(WIFI_AP);
  WiFi.softAP(WIFI_SSID, WIFI_PASSWORD);
  configureWebServer();

  stateMutex = xSemaphoreCreateMutex();

  if (lockState(pdMS_TO_TICKS(50))) {
    setIdleStateAndStopMotors();

    noInterrupts();
    for (int i = 0; i < WHEEL_COUNT; i++) {
      lastEncoderCounts[i] = encoderCounts[i];
      lastDeltaCounts[i] = 0;
    }
    interrupts();

    unlockState();
  }

  lastControlTime = millis();

  xTaskCreatePinnedToCore(
    imuUpdateTask,
    "IMUUpdate",
    4096,
    nullptr,
    4,
    &imuTaskHandle,
    1
  );

  xTaskCreatePinnedToCore(
    controlLoopTask,
    "ControlLoop",
    6144,
    nullptr,
    3,
    &controlTaskHandle,
    1
  );

  xTaskCreatePinnedToCore(
    webServerTask,
    "WebServer",
    6144,
    nullptr,
    1,
    &webTaskHandle,
    0
  );

  Serial.println();
  Serial.println("Forward-Position-With-Heading-Control-Test ready.");
  Serial.print("SSID: ");
  Serial.println(WIFI_SSID);
  Serial.print("Password: ");
  Serial.println(WIFI_PASSWORD);
  Serial.print("IP: ");
  Serial.println(WiFi.softAPIP());
  Serial.println("Open http://192.168.4.1/");
  Serial.println("Commands: move 0.50 | stop | heading on/off | hkp 30 | hki 2 | hmax 15 | hinvert");
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
