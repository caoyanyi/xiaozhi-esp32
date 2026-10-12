#include "wall_e_controller.h"

#include <algorithm>
#include <array>

#include <esp_log.h>
#include <esp_timer.h>

#include "config.h"

namespace wall_e {
namespace {

constexpr char kTag[]{"WallEController"};
constexpr uint16_t kServoFrequencyHz{50};
constexpr uint32_t kControlPeriodMs{20};
constexpr float kControlPeriodSeconds{static_cast<float>(kControlPeriodMs) / 1000.0F};
constexpr UBaseType_t kCommandQueueLength{12};
constexpr uint32_t kTaskStackSize{4096};
constexpr UBaseType_t kTaskPriority{4};

constexpr size_t ServoIndex(ServoId servo) { return static_cast<size_t>(servo); }

struct ServoDefinition {
    uint8_t channel;
    ServoCalibration calibration;
    float speed_degrees_per_second;
};

constexpr std::array<ServoDefinition, kServoCount> kServos{{
    {0, {70.0F, 110.0F, 90.0F, 500, 2500, false, true}, 100.0F},
    {1, {70.0F, 110.0F, 90.0F, 500, 2500, true, true}, 100.0F},
    {2, {30.0F, 150.0F, 40.0F, 500, 2500, false, true}, 90.0F},
    {3, {30.0F, 150.0F, 40.0F, 500, 2500, true, true}, 90.0F},
    {4, {70.0F, 110.0F, 90.0F, 500, 2500, false, true}, 55.0F},
    {5, {65.0F, 115.0F, 90.0F, 500, 2500, false, true}, 55.0F},
    {6, {45.0F, 135.0F, 90.0F, 500, 2500, false, true}, 70.0F},
    {7, {60.0F, 120.0F, 90.0F, 500, 2500, false, false}, 60.0F},
}};

int64_t NowMs() { return esp_timer_get_time() / 1000; }

}  // namespace

WallEController::WallEController(i2c_master_bus_handle_t i2c_bus) : i2c_bus_(i2c_bus) {
    for (size_t index = 0; index < kServos.size(); ++index) {
        current_angles_[index] = kServos[index].calibration.home_angle;
        target_angles_[index] = kServos[index].calibration.home_angle;
    }
}

WallEController::~WallEController() {
    Stop();
    if (task_ != nullptr) {
        vTaskDelete(task_);
    }
    if (command_queue_ != nullptr) {
        vQueueDelete(command_queue_);
    }
}

bool WallEController::Initialize() {
    if (initialized_.load()) {
        return true;
    }

    motors_ = std::make_unique<L298pMotor>(
        MotorPins{MOTOR_LEFT_ENABLE_GPIO, MOTOR_LEFT_INPUT_A_GPIO, MOTOR_LEFT_INPUT_B_GPIO,
                  LEDC_CHANNEL_0},
        MotorPins{MOTOR_RIGHT_ENABLE_GPIO, MOTOR_RIGHT_INPUT_A_GPIO, MOTOR_RIGHT_INPUT_B_GPIO,
                  LEDC_CHANNEL_1});
    const esp_err_t motor_error = motors_->Initialize();
    motor_available_.store(motor_error == ESP_OK);
    if (motor_error != ESP_OK) {
        ESP_LOGE(kTag, "L298P initialization failed: %s", esp_err_to_name(motor_error));
    }

    pca9685_ = std::make_unique<Pca9685>(i2c_bus_, PCA9685_I2C_ADDRESS);
    esp_err_t servo_error = pca9685_->Initialize(kServoFrequencyHz);
    if (servo_error == ESP_OK) {
        for (const ServoDefinition& servo : kServos) {
            if (!servo.calibration.enabled) {
                continue;
            }
            const uint16_t ticks = AngleToPwmTicks(servo.calibration,
                                                   servo.calibration.home_angle,
                                                   kServoFrequencyHz);
            servo_error = pca9685_->SetPulse(servo.channel, ticks);
            if (servo_error != ESP_OK) {
                break;
            }
        }
    }
    servo_available_.store(servo_error == ESP_OK);
    if (servo_error != ESP_OK) {
        ESP_LOGW(kTag, "PCA9685 unavailable; servo commands are disabled: %s",
                 esp_err_to_name(servo_error));
    }

    command_queue_ = xQueueCreate(kCommandQueueLength, sizeof(Command));
    if (command_queue_ == nullptr) {
        ESP_LOGE(kTag, "Failed to create motion command queue");
        return false;
    }
    if (xTaskCreate(TaskEntry, "wall_e_motion", kTaskStackSize, this, kTaskPriority, &task_) !=
        pdPASS) {
        ESP_LOGE(kTag, "Failed to create motion task");
        vQueueDelete(command_queue_);
        command_queue_ = nullptr;
        return false;
    }

    initialized_.store(true);
    return true;
}

bool WallEController::Drive(DriveAction action, uint8_t speed_percent, uint32_t duration_ms) {
    if (!initialized_.load() || !motor_available_.load()) {
        return false;
    }
    if (action == DriveAction::kStop) {
        Stop();
        return true;
    }

    const Command command{
        .type = CommandType::kDrive,
        .drive_action = action,
        .pose_action = PoseAction::kNeutral,
        .speed_percent = static_cast<uint8_t>(std::min<uint16_t>(speed_percent, 100)),
        .duration_ms = ClampDriveDurationMs(duration_ms),
        .stop_generation = stop_generation_.load(),
    };
    return Enqueue(command);
}

void WallEController::Stop() {
    stop_generation_.fetch_add(1);
    stop_requested_.store(true);
}

bool WallEController::SetPose(PoseAction action) {
    if (!initialized_.load() || !servo_available_.load()) {
        return false;
    }
    const Command command{
        .type = CommandType::kPose,
        .drive_action = DriveAction::kStop,
        .pose_action = action,
        .speed_percent = 0,
        .duration_ms = 0,
        .stop_generation = stop_generation_.load(),
    };
    return Enqueue(command);
}

std::string WallEController::GetStatus() const {
    return std::string{"{\"initialized\":"} + (initialized_.load() ? "true" : "false") +
           ",\"motor_available\":" + (motor_available_.load() ? "true" : "false") +
           ",\"servo_available\":" + (servo_available_.load() ? "true" : "false") +
           ",\"drive\":\"" + std::string{DriveActionName(current_drive_.load())} +
           "\",\"pose\":\"" + std::string{PoseActionName(current_pose_.load())} + "\"}";
}

void WallEController::TaskEntry(void* context) {
    static_cast<WallEController*>(context)->TaskLoop();
}

void WallEController::TaskLoop() {
    TickType_t last_wake = xTaskGetTickCount();
    while (true) {
        const int64_t now_ms = NowMs();
        if (stop_requested_.exchange(false)) {
            if (motor_available_.load()) {
                motors_->Stop();
            }
            current_drive_.store(DriveAction::kStop);
            drive_deadline_ms_ = 0;
        }

        Command command{};
        while (xQueueReceive(command_queue_, &command, 0) == pdTRUE) {
            HandleCommand(command, now_ms);
        }

        if (drive_deadline_ms_ > 0 && now_ms >= drive_deadline_ms_) {
            if (motor_available_.load()) {
                motors_->Stop();
            }
            current_drive_.store(DriveAction::kStop);
            drive_deadline_ms_ = 0;
        }
        UpdateServos();
        xTaskDelayUntil(&last_wake, pdMS_TO_TICKS(kControlPeriodMs));
    }
}

void WallEController::HandleCommand(const Command& command, int64_t now_ms) {
    if (command.type == CommandType::kDrive) {
        if (command.stop_generation != stop_generation_.load()) {
            return;
        }
        const MotorOutput output = MixDrive(command.drive_action, command.speed_percent);
        const esp_err_t error = motors_->SetOutput(output);
        if (error != ESP_OK) {
            ESP_LOGE(kTag, "Motor command failed: %s", esp_err_to_name(error));
            motors_->Stop();
            motor_available_.store(false);
            current_drive_.store(DriveAction::kStop);
            return;
        }
        current_drive_.store(command.drive_action);
        drive_deadline_ms_ = now_ms + command.duration_ms;
        return;
    }

    ApplyPose(command.pose_action);
    current_pose_.store(command.pose_action);
}

void WallEController::ApplyPose(PoseAction action) {
    switch (action) {
        case PoseAction::kNeutral:
            for (size_t index = 0; index < kServos.size(); ++index) {
                target_angles_[index] = kServos[index].calibration.home_angle;
            }
            break;
        case PoseAction::kLookUp:
            SetTarget(ServoId::kLowerNeck, 100.0F);
            SetTarget(ServoId::kUpperNeck, 110.0F);
            break;
        case PoseAction::kLookDown:
            SetTarget(ServoId::kLowerNeck, 80.0F);
            SetTarget(ServoId::kUpperNeck, 70.0F);
            break;
        case PoseAction::kLookLeft:
            SetTarget(ServoId::kHeadYaw, 120.0F);
            break;
        case PoseAction::kLookRight:
            SetTarget(ServoId::kHeadYaw, 60.0F);
            break;
        case PoseAction::kLeftArmUp:
            SetTarget(ServoId::kLeftArm, 130.0F);
            break;
        case PoseAction::kLeftArmDown:
            SetTarget(ServoId::kLeftArm, 40.0F);
            break;
        case PoseAction::kRightArmUp:
            SetTarget(ServoId::kRightArm, 130.0F);
            break;
        case PoseAction::kRightArmDown:
            SetTarget(ServoId::kRightArm, 40.0F);
            break;
        case PoseAction::kBothArmsUp:
            SetTarget(ServoId::kLeftArm, 130.0F);
            SetTarget(ServoId::kRightArm, 130.0F);
            break;
        case PoseAction::kBothArmsDown:
            SetTarget(ServoId::kLeftArm, 40.0F);
            SetTarget(ServoId::kRightArm, 40.0F);
            break;
        case PoseAction::kEyesLeft:
            SetTarget(ServoId::kLeftEye, 75.0F);
            SetTarget(ServoId::kRightEye, 75.0F);
            break;
        case PoseAction::kEyesRight:
            SetTarget(ServoId::kLeftEye, 105.0F);
            SetTarget(ServoId::kRightEye, 105.0F);
            break;
        case PoseAction::kEyesCenter:
            SetTarget(ServoId::kLeftEye, 90.0F);
            SetTarget(ServoId::kRightEye, 90.0F);
            break;
    }
}

void WallEController::UpdateServos() {
    if (!servo_available_.load()) {
        return;
    }
    for (size_t index = 0; index < kServos.size(); ++index) {
        const ServoDefinition& servo = kServos[index];
        if (!servo.calibration.enabled) {
            continue;
        }
        const float next_angle =
            StepToward(current_angles_[index], target_angles_[index],
                       servo.speed_degrees_per_second * kControlPeriodSeconds);
        if (next_angle == current_angles_[index]) {
            continue;
        }
        const uint16_t ticks =
            AngleToPwmTicks(servo.calibration, next_angle, kServoFrequencyHz);
        const esp_err_t error = pca9685_->SetPulse(servo.channel, ticks);
        if (error != ESP_OK) {
            ESP_LOGE(kTag, "Servo update failed: %s", esp_err_to_name(error));
            servo_available_.store(false);
            pca9685_->DisableAll();
            return;
        }
        current_angles_[index] = next_angle;
    }
}

void WallEController::SetTarget(ServoId servo, float angle) {
    const size_t index = ServoIndex(servo);
    const ServoCalibration& calibration = kServos[index].calibration;
    target_angles_[index] = std::clamp(angle, calibration.min_angle, calibration.max_angle);
}

bool WallEController::Enqueue(const Command& command) {
    return command_queue_ != nullptr && xQueueSend(command_queue_, &command, 0) == pdTRUE;
}

}  // namespace wall_e
