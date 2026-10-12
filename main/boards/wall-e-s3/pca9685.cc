#include "pca9685.h"

#include <algorithm>
#include <array>
#include <cmath>

#include <esp_log.h>
#include <esp_rom_sys.h>

namespace wall_e {
namespace {

constexpr char kTag[]{"Pca9685"};
constexpr uint8_t kMode1Register{0x00};
constexpr uint8_t kMode2Register{0x01};
constexpr uint8_t kFirstLedRegister{0x06};
constexpr uint8_t kPrescaleRegister{0xFE};
constexpr uint8_t kAllLedOnLowRegister{0xFA};
constexpr uint8_t kMode1Restart{0x80};
constexpr uint8_t kMode1AutoIncrement{0x20};
constexpr uint8_t kMode1Sleep{0x10};
constexpr uint8_t kMode2TotemPole{0x04};
constexpr uint8_t kFullOffBit{0x10};
constexpr uint32_t kOscillatorFrequencyHz{25000000};
constexpr uint32_t kPwmResolution{4096};
constexpr uint32_t kI2cTimeoutMs{100};

}  // namespace

Pca9685::Pca9685(i2c_master_bus_handle_t bus, uint8_t address) : bus_(bus), address_(address) {}

Pca9685::~Pca9685() {
    if (device_ != nullptr) {
        i2c_master_bus_rm_device(device_);
    }
}

esp_err_t Pca9685::Initialize(uint16_t frequency_hz) {
    if (bus_ == nullptr || frequency_hz == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = address_,
        .scl_speed_hz = 400 * 1000,
        .scl_wait_us = 0,
        .flags = {.disable_ack_check = 0},
    };
    esp_err_t error = i2c_master_bus_add_device(bus_, &device_config, &device_);
    if (error != ESP_OK) {
        return error;
    }

    uint8_t previous_mode{0};
    if ((error = ReadRegister(kMode1Register, previous_mode)) != ESP_OK) {
        return error;
    }
    if ((error = WriteRegister(kMode1Register,
                               static_cast<uint8_t>((previous_mode & 0x7F) | kMode1Sleep))) !=
        ESP_OK) {
        return error;
    }

    const float raw_prescale = static_cast<float>(kOscillatorFrequencyHz) /
                                   (static_cast<float>(kPwmResolution) * frequency_hz) -
                               1.0F;
    const auto prescale = static_cast<uint8_t>(std::clamp(std::lround(raw_prescale), 3L, 255L));
    if ((error = WriteRegister(kPrescaleRegister, prescale)) != ESP_OK ||
        (error = WriteRegister(kMode1Register, previous_mode)) != ESP_OK) {
        return error;
    }

    esp_rom_delay_us(500);
    if ((error = WriteRegister(kMode1Register,
                               static_cast<uint8_t>(previous_mode | kMode1Restart |
                                                    kMode1AutoIncrement))) != ESP_OK ||
        (error = WriteRegister(kMode2Register, kMode2TotemPole)) != ESP_OK) {
        return error;
    }
    return EnableAll();
}

esp_err_t Pca9685::SetPulse(uint8_t channel, uint16_t off_tick) {
    if (device_ == nullptr || channel >= 16 || off_tick >= kPwmResolution) {
        return ESP_ERR_INVALID_ARG;
    }

    const uint8_t register_address =
        static_cast<uint8_t>(kFirstLedRegister + static_cast<uint8_t>(channel * 4));
    const std::array<uint8_t, 5> payload{
        register_address,
        0,
        0,
        static_cast<uint8_t>(off_tick & 0xFF),
        static_cast<uint8_t>((off_tick >> 8) & 0x0F),
    };
    return i2c_master_transmit(device_, payload.data(), payload.size(), kI2cTimeoutMs);
}

esp_err_t Pca9685::DisableAll() {
    if (device_ == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    const std::array<uint8_t, 5> payload{kAllLedOnLowRegister, 0, 0, 0, kFullOffBit};
    return i2c_master_transmit(device_, payload.data(), payload.size(), kI2cTimeoutMs);
}

esp_err_t Pca9685::EnableAll() {
    if (device_ == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    const std::array<uint8_t, 5> payload{kAllLedOnLowRegister, 0, 0, 0, 0};
    return i2c_master_transmit(device_, payload.data(), payload.size(), kI2cTimeoutMs);
}

esp_err_t Pca9685::ReadRegister(uint8_t reg, uint8_t& value) {
    if (device_ == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    return i2c_master_transmit_receive(device_, &reg, 1, &value, 1, kI2cTimeoutMs);
}

esp_err_t Pca9685::WriteRegister(uint8_t reg, uint8_t value) {
    if (device_ == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    const std::array<uint8_t, 2> payload{reg, value};
    const esp_err_t error =
        i2c_master_transmit(device_, payload.data(), payload.size(), kI2cTimeoutMs);
    if (error != ESP_OK) {
        ESP_LOGW(kTag, "I2C write failed at register 0x%02x: %s", reg,
                 esp_err_to_name(error));
    }
    return error;
}

}  // namespace wall_e
