#ifndef WALL_E_MOTION_MODEL_H_
#define WALL_E_MOTION_MODEL_H_

#include <cstdint>
#include <optional>
#include <string_view>

namespace wall_e {

constexpr uint8_t kServoCount{8};
constexpr uint32_t kMinimumDriveDurationMs{200};
constexpr uint32_t kMaximumDriveDurationMs{5000};

enum class ServoId : uint8_t {
    kLeftEye,
    kRightEye,
    kLeftArm,
    kRightArm,
    kLowerNeck,
    kUpperNeck,
    kHeadYaw,
    kAuxiliary,
};

enum class DriveAction : uint8_t {
    kStop,
    kForward,
    kBackward,
    kTurnLeft,
    kTurnRight,
};

enum class PoseAction : uint8_t {
    kNeutral,
    kLookUp,
    kLookDown,
    kLookLeft,
    kLookRight,
    kLeftArmUp,
    kLeftArmDown,
    kRightArmUp,
    kRightArmDown,
    kBothArmsUp,
    kBothArmsDown,
    kEyesLeft,
    kEyesRight,
    kEyesCenter,
};

struct ServoCalibration {
    float min_angle;
    float max_angle;
    float home_angle;
    uint16_t min_pulse_us;
    uint16_t max_pulse_us;
    bool reversed;
    bool enabled;
};

struct MotorOutput {
    int8_t left_percent;
    int8_t right_percent;

    bool operator==(const MotorOutput& other) const noexcept {
        return left_percent == other.left_percent && right_percent == other.right_percent;
    }
};

uint16_t AngleToPwmTicks(const ServoCalibration& calibration, float angle,
                         uint16_t frequency_hz = 50) noexcept;
float StepToward(float current, float target, float max_step) noexcept;
MotorOutput MixDrive(DriveAction action, uint8_t speed_percent) noexcept;
uint32_t ClampDriveDurationMs(uint32_t duration_ms) noexcept;
std::optional<DriveAction> ParseDriveAction(std::string_view action) noexcept;
std::optional<PoseAction> ParsePoseAction(std::string_view action) noexcept;
std::string_view DriveActionName(DriveAction action) noexcept;
std::string_view PoseActionName(PoseAction action) noexcept;

}  // namespace wall_e

#endif  // WALL_E_MOTION_MODEL_H_
