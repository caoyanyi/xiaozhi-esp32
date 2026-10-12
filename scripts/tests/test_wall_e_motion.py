"""Host-side tests for the Wall-E motion model and board integration."""

import pathlib
import shutil
import subprocess
import tempfile
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[2]
BOARD = ROOT / "main/boards/wall-e-s3"


class WallEMotionModelTests(unittest.TestCase):
    @unittest.skipUnless(shutil.which("c++"), "requires a host C++ compiler")
    def test_motion_model_safety_and_command_mapping(self):
        source = r'''
#include <cassert>
#include <cmath>
#include <cstdint>

#include "wall_e_motion_model.h"

using namespace wall_e;

int main() {
    const ServoCalibration normal{
        .min_angle = 0.0F,
        .max_angle = 180.0F,
        .home_angle = 90.0F,
        .min_pulse_us = 500,
        .max_pulse_us = 2500,
        .reversed = false,
        .enabled = true,
    };
    const ServoCalibration reversed{
        .min_angle = 0.0F,
        .max_angle = 180.0F,
        .home_angle = 90.0F,
        .min_pulse_us = 500,
        .max_pulse_us = 2500,
        .reversed = true,
        .enabled = true,
    };

    assert(AngleToPwmTicks(normal, -20.0F) == 102);
    assert(AngleToPwmTicks(normal, 90.0F) == 307);
    assert(AngleToPwmTicks(normal, 220.0F) == 512);
    assert(AngleToPwmTicks(reversed, 0.0F) == 512);
    assert(AngleToPwmTicks(reversed, 180.0F) == 102);

    assert(std::fabs(StepToward(90.0F, 100.0F, 3.0F) - 93.0F) < 0.001F);
    assert(std::fabs(StepToward(90.0F, 80.0F, 3.0F) - 87.0F) < 0.001F);
    assert(std::fabs(StepToward(90.0F, 92.0F, 3.0F) - 92.0F) < 0.001F);
    assert(std::fabs(StepToward(90.0F, 100.0F, 0.0F) - 90.0F) < 0.001F);

    assert((MixDrive(DriveAction::kStop, 80) == MotorOutput{0, 0}));
    assert((MixDrive(DriveAction::kForward, 60) == MotorOutput{60, 60}));
    assert((MixDrive(DriveAction::kBackward, 60) == MotorOutput{-60, -60}));
    assert((MixDrive(DriveAction::kTurnLeft, 60) == MotorOutput{-60, 60}));
    assert((MixDrive(DriveAction::kTurnRight, 60) == MotorOutput{60, -60}));
    assert((MixDrive(DriveAction::kForward, 255) == MotorOutput{100, 100}));

    assert(ClampDriveDurationMs(10) == 200);
    assert(ClampDriveDurationMs(1200) == 1200);
    assert(ClampDriveDurationMs(9000) == 5000);

    assert(ParseDriveAction("forward") == DriveAction::kForward);
    assert(ParseDriveAction("backward") == DriveAction::kBackward);
    assert(ParseDriveAction("turn_left") == DriveAction::kTurnLeft);
    assert(ParseDriveAction("turn_right") == DriveAction::kTurnRight);
    assert(ParseDriveAction("stop") == DriveAction::kStop);
    assert(!ParseDriveAction("run_forever").has_value());

    assert(ParsePoseAction("neutral") == PoseAction::kNeutral);
    assert(ParsePoseAction("look_up") == PoseAction::kLookUp);
    assert(ParsePoseAction("look_down") == PoseAction::kLookDown);
    assert(ParsePoseAction("look_left") == PoseAction::kLookLeft);
    assert(ParsePoseAction("look_right") == PoseAction::kLookRight);
    assert(ParsePoseAction("left_arm_up") == PoseAction::kLeftArmUp);
    assert(ParsePoseAction("left_arm_down") == PoseAction::kLeftArmDown);
    assert(ParsePoseAction("right_arm_up") == PoseAction::kRightArmUp);
    assert(ParsePoseAction("right_arm_down") == PoseAction::kRightArmDown);
    assert(ParsePoseAction("both_arms_up") == PoseAction::kBothArmsUp);
    assert(ParsePoseAction("both_arms_down") == PoseAction::kBothArmsDown);
    assert(ParsePoseAction("eyes_left") == PoseAction::kEyesLeft);
    assert(ParsePoseAction("eyes_right") == PoseAction::kEyesRight);
    assert(ParsePoseAction("eyes_center") == PoseAction::kEyesCenter);
    assert(!ParsePoseAction("unsafe_pose").has_value());
}
'''
        with tempfile.TemporaryDirectory() as temp_dir:
            binary = pathlib.Path(temp_dir) / "wall-e-motion-test"
            subprocess.run(
                [
                    "c++",
                    "-std=c++17",
                    "-Wall",
                    "-Wextra",
                    "-Werror",
                    "-fsanitize=undefined",
                    "-I",
                    str(BOARD),
                    "-x",
                    "c++",
                    "-",
                    str(BOARD / "wall_e_motion_model.cc"),
                    "-o",
                    str(binary),
                ],
                input=source,
                text=True,
                check=True,
            )
            subprocess.run([str(binary)], check=True)

    def test_board_declares_safe_voice_tools_and_shared_i2c(self):
        board_source = (BOARD / "wall_e_s3.cc").read_text(encoding="utf-8")
        config = (BOARD / "config.h").read_text(encoding="utf-8")

        for tool_name in (
            "self.walle.drive",
            "self.walle.stop",
            "self.walle.pose",
            "self.walle.get_status",
        ):
            self.assertIn(tool_name, board_source)

        self.assertIn("DISPLAY_SDA_PIN", config)
        self.assertIn("DISPLAY_SCL_PIN", config)
        self.assertIn("PCA9685_I2C_ADDRESS", config)
        self.assertIn("MOTOR_LEFT_ENABLE_GPIO", config)
        self.assertIn("MOTOR_RIGHT_ENABLE_GPIO", config)
        self.assertIn("ClampDriveDurationMs", board_source)


if __name__ == "__main__":
    unittest.main()
