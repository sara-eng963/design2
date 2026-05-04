#include <Arduino.h>
#include <Wire.h>
#include <cmath>
#include <cstddef>
#include <cstdint>

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

app::IMUState g_imu_state;
std::uint32_t g_last_display_ms = 0U;

void printImuDebug() {
    const app::ImuDriver& imu = app::imuDriver();
    Serial.printf(
        "Debug | State: %s | Yaw: %.2f deg | RawGz: %.2f deg/s | CorrGz: %.2f deg/s | BiasZ: %.2f deg/s | IntYaw: %.2f deg | Offset: %.2f deg\n",
        imu.motionStateName(),
        app::radToDeg(imu.displayedYawRad()),
        app::radToDeg(imu.rawGyroZRadPerSec()),
        app::radToDeg(imu.correctedGyroZRadPerSec()),
        app::radToDeg(imu.gyroBiasZRadPerSec()),
        app::radToDeg(imu.yawIntegratedRad()),
        app::radToDeg(imu.yawZeroOffsetRad()));
}

void handleSerialCommands() {
    while (Serial.available() > 0) {
        const char command = static_cast<char>(Serial.read());
        switch (command) {
            case 'z':
            case 'Z':
            case 'r':
            case 'R':
                app::imuDriver().zeroYaw();
                Serial.println("Yaw re-zeroed. Current heading is now 0 deg.");
                break;
            case 'p':
            case 'P':
                printImuDebug();
                break;
            case '\r':
            case '\n':
            case ' ':
            case '\t':
                break;
            default:
                Serial.printf("Unknown command '%c'. Use z/r to zero yaw, p for debug.\n", command);
                break;
        }
    }
}

void setup() {
    Serial.begin(app::SERIAL_BAUD_RATE);
    delay(1000);

    Serial.println();
    Serial.println("ESP32 IMU monitor starting...");
    Serial.printf("Using I2C pins SDA=%ld SCL=%ld\n", app::IMU_SDA_PIN, app::IMU_SCL_PIN);

    if (!app::imuDriver().begin()) {
        Serial.println("IMU initialization failed.");
        Serial.printf("Detected chip: %s (WHO_AM_I=0x%02X)\n",
                      app::imuDriver().detectedChipName(),
                      app::imuDriver().detectedWhoAmI());
        Serial.println("Check wiring, sensor address (0x68), and power.");
        while (true) {
            delay(1000);
        }
    }

    Serial.printf("IMU connected: %s (WHO_AM_I=0x%02X)\n",
                  app::imuDriver().detectedChipName(),
                  app::imuDriver().detectedWhoAmI());
    Serial.println("Keep the robot still for 3 to 5 seconds after boot for better gyro calibration.");
    Serial.println("Commands: z/r = zero yaw, p = print debug.");
    Serial.println("X/Y shown below are acceleration values, not position.");
    Serial.println("Readable output is limited to about 5 lines per second.");
}

void loop() {
    const bool read_ok = app::imuDriver().read(g_imu_state);
    handleSerialCommands();

    if (read_ok) {
        g_imu_state.orientation_z = app::imuDriver().displayedYawRad();
        const std::uint32_t now_ms = millis();
        if ((g_last_display_ms == 0U) || ((now_ms - g_last_display_ms) >= app::DISPLAY_INTERVAL_MS)) {
            g_last_display_ms = now_ms;
            const float yaw_deg = app::radToDeg(g_imu_state.orientation_z);
            Serial.printf("X accel: %.3f m/s^2 | Y accel: %.3f m/s^2 | Yaw: %.2f deg\n",
                          g_imu_state.accel_x,
                          g_imu_state.accel_y,
                          yaw_deg);
        }
    } else {
        Serial.println("IMU read failed.");
    }

    delay(app::IMU_READ_PERIOD_MS);
}
