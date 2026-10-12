#ifndef WALL_E_PCA9685_H_
#define WALL_E_PCA9685_H_

#include <cstdint>

#include <driver/i2c_master.h>
#include <esp_err.h>

namespace wall_e {

class Pca9685 {
public:
    Pca9685(i2c_master_bus_handle_t bus, uint8_t address);
    ~Pca9685();

    Pca9685(const Pca9685&) = delete;
    Pca9685& operator=(const Pca9685&) = delete;
    Pca9685(Pca9685&&) = delete;
    Pca9685& operator=(Pca9685&&) = delete;

    esp_err_t Initialize(uint16_t frequency_hz = 50);
    esp_err_t SetPulse(uint8_t channel, uint16_t off_tick);
    esp_err_t EnableAll();
    esp_err_t DisableAll();

private:
    esp_err_t ReadRegister(uint8_t reg, uint8_t& value);
    esp_err_t WriteRegister(uint8_t reg, uint8_t value);

    i2c_master_bus_handle_t bus_;
    i2c_master_dev_handle_t device_{nullptr};
    uint8_t address_;
};

}  // namespace wall_e

#endif  // WALL_E_PCA9685_H_
