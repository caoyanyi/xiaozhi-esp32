#include "l298p_motor.h"

#include <algorithm>
#include <cstdlib>

namespace wall_e {
namespace {

constexpr ledc_mode_t kSpeedMode{LEDC_LOW_SPEED_MODE};
constexpr ledc_timer_t kTimer{LEDC_TIMER_0};
constexpr ledc_timer_bit_t kDutyResolution{LEDC_TIMER_10_BIT};
constexpr uint32_t kPwmFrequencyHz{20000};
constexpr uint32_t kMaximumDuty{(1U << 10U) - 1U};

esp_err_t FirstError(esp_err_t first, esp_err_t next) { return first == ESP_OK ? next : first; }

}  // namespace

L298pMotor::L298pMotor(MotorPins left, MotorPins right) : left_(left), right_(right) {}

L298pMotor::~L298pMotor() {
    if (initialized_) {
        Stop();
        ledc_stop(kSpeedMode, left_.pwm_channel, 0);
        ledc_stop(kSpeedMode, right_.pwm_channel, 0);
    }
}

esp_err_t L298pMotor::Initialize() {
    const uint64_t direction_pin_mask = (1ULL << left_.input_a) | (1ULL << left_.input_b) |
                                        (1ULL << right_.input_a) | (1ULL << right_.input_b);
    const gpio_config_t output_config = {
        .pin_bit_mask = direction_pin_mask,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t error = gpio_config(&output_config);
    if (error != ESP_OK) {
        return error;
    }

    const ledc_timer_config_t timer_config = {
        .speed_mode = kSpeedMode,
        .duty_resolution = kDutyResolution,
        .timer_num = kTimer,
        .freq_hz = kPwmFrequencyHz,
        .clk_cfg = LEDC_AUTO_CLK,
        .deconfigure = false,
    };
    if ((error = ledc_timer_config(&timer_config)) != ESP_OK) {
        return error;
    }

    for (const MotorPins& motor : {left_, right_}) {
        const ledc_channel_config_t channel_config = {
            .gpio_num = motor.enable,
            .speed_mode = kSpeedMode,
            .channel = motor.pwm_channel,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = kTimer,
            .duty = 0,
            .hpoint = 0,
            .sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD,
            .flags = {.output_invert = 0},
        };
        if ((error = ledc_channel_config(&channel_config)) != ESP_OK) {
            return error;
        }
    }

    initialized_ = true;
    return Stop();
}

esp_err_t L298pMotor::SetOutput(const MotorOutput& output) {
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t error = SetMotor(left_, output.left_percent);
    return FirstError(error, SetMotor(right_, output.right_percent));
}

esp_err_t L298pMotor::Stop() {
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }
    return SetOutput({0, 0});
}

esp_err_t L298pMotor::SetMotor(const MotorPins& pins, int8_t speed_percent) {
    const int speed = std::clamp(static_cast<int>(speed_percent), -100, 100);
    esp_err_t error = ledc_set_duty(kSpeedMode, pins.pwm_channel, 0);
    error = FirstError(error, ledc_update_duty(kSpeedMode, pins.pwm_channel));
    if (error != ESP_OK) {
        return error;
    }

    const int level_a = speed > 0 ? 1 : 0;
    const int level_b = speed < 0 ? 1 : 0;
    error = FirstError(gpio_set_level(pins.input_a, level_a),
                       gpio_set_level(pins.input_b, level_b));

    const uint32_t duty = static_cast<uint32_t>(std::abs(speed)) * kMaximumDuty / 100U;
    error = FirstError(error, ledc_set_duty(kSpeedMode, pins.pwm_channel, duty));
    return FirstError(error, ledc_update_duty(kSpeedMode, pins.pwm_channel));
}

}  // namespace wall_e
