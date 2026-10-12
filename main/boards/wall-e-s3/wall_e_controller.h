#ifndef WALL_E_CONTROLLER_H_
#define WALL_E_CONTROLLER_H_

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <string>

#include <driver/i2c_master.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include "l298p_motor.h"
#include "pca9685.h"
#include "wall_e_motion_model.h"

namespace wall_e {

class WallEController {
public:
    explicit WallEController(i2c_master_bus_handle_t i2c_bus);
    ~WallEController();

    WallEController(const WallEController&) = delete;
    WallEController& operator=(const WallEController&) = delete;
    WallEController(WallEController&&) = delete;
    WallEController& operator=(WallEController&&) = delete;

    bool Initialize();
    bool Drive(DriveAction action, uint8_t speed_percent, uint32_t duration_ms);
    void Stop();
    bool SetPose(PoseAction action);
    std::string GetStatus() const;

private:
    enum class CommandType : uint8_t { kDrive, kPose };

    struct Command {
        CommandType type;
        DriveAction drive_action;
        PoseAction pose_action;
        uint8_t speed_percent;
        uint32_t duration_ms;
        uint32_t stop_generation;
    };

    static void TaskEntry(void* context);
    void TaskLoop();
    void HandleCommand(const Command& command, int64_t now_ms);
    void ApplyPose(PoseAction action);
    void UpdateServos();
    void SetTarget(ServoId servo, float angle);
    bool Enqueue(const Command& command);

    i2c_master_bus_handle_t i2c_bus_;
    std::unique_ptr<Pca9685> pca9685_;
    std::unique_ptr<L298pMotor> motors_;
    QueueHandle_t command_queue_{nullptr};
    TaskHandle_t task_{nullptr};
    std::array<float, kServoCount> current_angles_{};
    std::array<float, kServoCount> target_angles_{};
    std::atomic<bool> initialized_{false};
    std::atomic<bool> servo_available_{false};
    std::atomic<bool> motor_available_{false};
    std::atomic<bool> stop_requested_{false};
    std::atomic<uint32_t> stop_generation_{0};
    std::atomic<DriveAction> current_drive_{DriveAction::kStop};
    std::atomic<PoseAction> current_pose_{PoseAction::kNeutral};
    int64_t drive_deadline_ms_{0};
};

}  // namespace wall_e

#endif  // WALL_E_CONTROLLER_H_
