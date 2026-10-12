#include "wall_e_motion_model.h"

#include <algorithm>
#include <cmath>

namespace wall_e {
namespace {

constexpr uint32_t kMicrosecondsPerSecond{1000000};
constexpr uint32_t kPca9685TicksPerCycle{4096};
constexpr uint8_t kMaximumSpeedPercent{100};

}  // namespace

uint16_t AngleToPwmTicks(const ServoCalibration& calibration, float angle,
                         uint16_t frequency_hz) noexcept {
    if (frequency_hz == 0 || calibration.max_angle <= calibration.min_angle ||
        calibration.max_pulse_us <= calibration.min_pulse_us) {
        return 0;
    }

    const float clamped_angle = std::clamp(angle, calibration.min_angle, calibration.max_angle);
    float ratio = (clamped_angle - calibration.min_angle) /
                  (calibration.max_angle - calibration.min_angle);
    if (calibration.reversed) {
        ratio = 1.0F - ratio;
    }

    const float pulse_width_us =
        static_cast<float>(calibration.min_pulse_us) +
        ratio * static_cast<float>(calibration.max_pulse_us - calibration.min_pulse_us);
    const float ticks = pulse_width_us * static_cast<float>(frequency_hz) *
                        static_cast<float>(kPca9685TicksPerCycle) /
                        static_cast<float>(kMicrosecondsPerSecond);
    return static_cast<uint16_t>(std::clamp(std::lround(ticks), 0L, 4095L));
}

float StepToward(float current, float target, float max_step) noexcept {
    if (max_step <= 0.0F) {
        return current;
    }
    if (target > current + max_step) {
        return current + max_step;
    }
    if (target < current - max_step) {
        return current - max_step;
    }
    return target;
}

MotorOutput MixDrive(DriveAction action, uint8_t speed_percent) noexcept {
    const auto speed = static_cast<int8_t>(std::min(speed_percent, kMaximumSpeedPercent));
    switch (action) {
        case DriveAction::kForward:
            return {speed, speed};
        case DriveAction::kBackward:
            return {static_cast<int8_t>(-speed), static_cast<int8_t>(-speed)};
        case DriveAction::kTurnLeft:
            return {static_cast<int8_t>(-speed), speed};
        case DriveAction::kTurnRight:
            return {speed, static_cast<int8_t>(-speed)};
        case DriveAction::kStop:
            return {0, 0};
    }
    return {0, 0};
}

uint32_t ClampDriveDurationMs(uint32_t duration_ms) noexcept {
    return std::clamp(duration_ms, kMinimumDriveDurationMs, kMaximumDriveDurationMs);
}

std::optional<DriveAction> ParseDriveAction(std::string_view action) noexcept {
    if (action == "forward") {
        return DriveAction::kForward;
    }
    if (action == "backward") {
        return DriveAction::kBackward;
    }
    if (action == "turn_left") {
        return DriveAction::kTurnLeft;
    }
    if (action == "turn_right") {
        return DriveAction::kTurnRight;
    }
    if (action == "stop") {
        return DriveAction::kStop;
    }
    return std::nullopt;
}

std::optional<PoseAction> ParsePoseAction(std::string_view action) noexcept {
    if (action == "neutral") {
        return PoseAction::kNeutral;
    }
    if (action == "look_up") {
        return PoseAction::kLookUp;
    }
    if (action == "look_down") {
        return PoseAction::kLookDown;
    }
    if (action == "look_left") {
        return PoseAction::kLookLeft;
    }
    if (action == "look_right") {
        return PoseAction::kLookRight;
    }
    if (action == "left_arm_up") {
        return PoseAction::kLeftArmUp;
    }
    if (action == "left_arm_down") {
        return PoseAction::kLeftArmDown;
    }
    if (action == "right_arm_up") {
        return PoseAction::kRightArmUp;
    }
    if (action == "right_arm_down") {
        return PoseAction::kRightArmDown;
    }
    if (action == "both_arms_up") {
        return PoseAction::kBothArmsUp;
    }
    if (action == "both_arms_down") {
        return PoseAction::kBothArmsDown;
    }
    if (action == "eyes_left") {
        return PoseAction::kEyesLeft;
    }
    if (action == "eyes_right") {
        return PoseAction::kEyesRight;
    }
    if (action == "eyes_center") {
        return PoseAction::kEyesCenter;
    }
    return std::nullopt;
}

std::string_view DriveActionName(DriveAction action) noexcept {
    switch (action) {
        case DriveAction::kForward:
            return "forward";
        case DriveAction::kBackward:
            return "backward";
        case DriveAction::kTurnLeft:
            return "turn_left";
        case DriveAction::kTurnRight:
            return "turn_right";
        case DriveAction::kStop:
            return "stop";
    }
    return "stop";
}

std::string_view PoseActionName(PoseAction action) noexcept {
    switch (action) {
        case PoseAction::kNeutral:
            return "neutral";
        case PoseAction::kLookUp:
            return "look_up";
        case PoseAction::kLookDown:
            return "look_down";
        case PoseAction::kLookLeft:
            return "look_left";
        case PoseAction::kLookRight:
            return "look_right";
        case PoseAction::kLeftArmUp:
            return "left_arm_up";
        case PoseAction::kLeftArmDown:
            return "left_arm_down";
        case PoseAction::kRightArmUp:
            return "right_arm_up";
        case PoseAction::kRightArmDown:
            return "right_arm_down";
        case PoseAction::kBothArmsUp:
            return "both_arms_up";
        case PoseAction::kBothArmsDown:
            return "both_arms_down";
        case PoseAction::kEyesLeft:
            return "eyes_left";
        case PoseAction::kEyesRight:
            return "eyes_right";
        case PoseAction::kEyesCenter:
            return "eyes_center";
    }
    return "neutral";
}

}  // namespace wall_e
