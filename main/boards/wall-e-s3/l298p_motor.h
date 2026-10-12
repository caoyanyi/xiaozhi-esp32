#ifndef WALL_E_L298P_MOTOR_H_
#define WALL_E_L298P_MOTOR_H_

#include <cstdint>

#include <driver/gpio.h>
#include <driver/ledc.h>
#include <esp_err.h>

#include "wall_e_motion_model.h"

namespace wall_e {

struct MotorPins {
    gpio_num_t enable;
    gpio_num_t input_a;
    gpio_num_t input_b;
    ledc_channel_t pwm_channel;
};

class L298pMotor {
public:
    L298pMotor(MotorPins left, MotorPins right);
    ~L298pMotor();

    L298pMotor(const L298pMotor&) = delete;
    L298pMotor& operator=(const L298pMotor&) = delete;
    L298pMotor(L298pMotor&&) = delete;
    L298pMotor& operator=(L298pMotor&&) = delete;

    esp_err_t Initialize();
    esp_err_t SetOutput(const MotorOutput& output);
    esp_err_t Stop();

private:
    esp_err_t SetMotor(const MotorPins& pins, int8_t speed_percent);

    MotorPins left_;
    MotorPins right_;
    bool initialized_{false};
};

}  // namespace wall_e

#endif  // WALL_E_L298P_MOTOR_H_
