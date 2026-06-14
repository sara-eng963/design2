#pragma once

#include <Arduino.h>
#include <cstddef>
#include <cstdint>

namespace app {

constexpr std::uint8_t MPU6050_I2C_ADDRESS = 0x68U;

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

    float orientation_x = 0.0f;
    float orientation_y = 0.0f;
    float orientation_z = 0.0f;
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
    float lastDtMs() const { return last_dt_s_ * 1000.0f; }

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
    float last_dt_s_ = 0.0f;
    std::uint32_t last_update_us_ = 0U;
    std::uint32_t motion_state_since_us_ = 0U;
    MotionState motion_state_ = MotionState::STILL;
};

ImuDriver& imuDriver();

}  // namespace app