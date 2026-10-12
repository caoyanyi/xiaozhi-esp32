#include "application.h"
#include "button.h"
#include "codecs/no_audio_codec.h"
#include "config.h"
#include "display/oled_display.h"
#include "led/single_led.h"
#include "mcp_server.h"
#include "wall_e_controller.h"
#include "wifi_board.h"

#include <algorithm>
#include <memory>
#include <string>

#include <driver/i2c_master.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_sh1106.h>
#include <esp_log.h>

namespace {

constexpr char kTag[]{"WallES3Board"};

}  // namespace

class WallES3Board : public WifiBoard {
public:
    WallES3Board()
        : boot_button_(BOOT_BUTTON_GPIO),
          touch_button_(TOUCH_BUTTON_GPIO),
          volume_up_button_(VOLUME_UP_BUTTON_GPIO),
          volume_down_button_(VOLUME_DOWN_BUTTON_GPIO) {
        InitializeI2c();
        InitializeDisplay();
        InitializeButtons();
        InitializeRobot();
        InitializeTools();
    }

    Led* GetLed() override {
        static SingleLed led(BUILTIN_LED_GPIO);
        return &led;
    }

    AudioCodec* GetAudioCodec() override {
        static NoAudioCodecSimplex audio_codec(
            AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE, AUDIO_I2S_SPK_GPIO_BCLK,
            AUDIO_I2S_SPK_GPIO_LRCK, AUDIO_I2S_SPK_GPIO_DOUT, AUDIO_I2S_MIC_GPIO_SCK,
            AUDIO_I2S_MIC_GPIO_WS, AUDIO_I2S_MIC_GPIO_DIN);
        return &audio_codec;
    }

    Display* GetDisplay() override { return display_.get(); }

private:
    void InitializeI2c() {
        const i2c_master_bus_config_t bus_config = {
            .i2c_port = I2C_NUM_0,
            .sda_io_num = DISPLAY_SDA_PIN,
            .scl_io_num = DISPLAY_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {.enable_internal_pullup = 1},
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &i2c_bus_));
    }

    void InitializeDisplay() {
        esp_lcd_panel_io_i2c_config_t io_config = {
            .dev_addr = 0x3C,
            .scl_speed_hz = 400 * 1000,
            .control_phase_bytes = 1,
            .dc_bit_offset = 6,
            .lcd_cmd_bits = 8,
            .lcd_param_bits = 8,
            .on_color_trans_done = nullptr,
            .user_ctx = nullptr,
            .flags =
                {
                    .dc_low_on_data = 0,
                    .disable_control_phase = 0,
                },
        };
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(i2c_bus_, &io_config, &panel_io_));

        esp_lcd_panel_dev_config_t panel_config{};
        panel_config.reset_gpio_num = GPIO_NUM_NC;
        panel_config.bits_per_pixel = 1;
        ESP_ERROR_CHECK(esp_lcd_new_panel_sh1106(panel_io_, &panel_config, &panel_));
        ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_));
        ESP_ERROR_CHECK(esp_lcd_panel_init(panel_));
        ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_, false));
        ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_, true));

        display_ = std::make_unique<OledDisplay>(panel_io_, panel_, DISPLAY_WIDTH, DISPLAY_HEIGHT,
                                                 DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& application = Application::GetInstance();
            if (application.GetDeviceState() == kDeviceStateStarting) {
                EnterWifiConfigMode();
                return;
            }
            application.ToggleChatState();
        });
        touch_button_.OnPressDown([]() { Application::GetInstance().StartListening(); });
        touch_button_.OnPressUp([]() { Application::GetInstance().StopListening(); });
        volume_up_button_.OnClick([this]() { ChangeVolume(10); });
        volume_down_button_.OnClick([this]() { ChangeVolume(-10); });
    }

    void ChangeVolume(int difference) {
        AudioCodec* codec = GetAudioCodec();
        const int volume = std::clamp(codec->output_volume() + difference, 0, 100);
        codec->SetOutputVolume(volume);
        GetDisplay()->ShowNotification(std::string{"音量 "} + std::to_string(volume));
    }

    void InitializeRobot() {
        robot_ = std::make_unique<wall_e::WallEController>(i2c_bus_);
        if (!robot_->Initialize()) {
            ESP_LOGE(kTag, "Wall-E controller failed to start");
        }
    }

    void InitializeTools() {
        auto& mcp_server = McpServer::GetInstance();
        mcp_server.AddTool(
            "self.walle.drive",
            "控制瓦力底盘。action只能是forward、backward、turn_left、turn_right或stop；"
            "行走会在duration_ms后自动停止。",
            PropertyList({Property("action", kPropertyTypeString),
                          Property("speed", kPropertyTypeInteger, 60, 20, 100),
                          Property("duration_ms", kPropertyTypeInteger, 1200, 200, 5000)}),
            [this](const PropertyList& properties) -> ToolResult {
                const std::string& action_text = properties["action"].value<std::string>();
                const auto action = wall_e::ParseDriveAction(action_text);
                if (!action.has_value()) {
                    return std::unexpected("不支持的底盘动作: " + action_text);
                }
                const auto speed =
                    static_cast<uint8_t>(properties["speed"].value<int>());
                const auto duration_ms = wall_e::ClampDriveDurationMs(
                    static_cast<uint32_t>(properties["duration_ms"].value<int>()));
                if (!robot_->Drive(*action, speed, duration_ms)) {
                    return std::unexpected("底盘控制器未就绪或命令队列已满");
                }
                return true;
            });

        mcp_server.AddTool("self.walle.stop", "立即停止瓦力底盘，优先级最高。", PropertyList(),
                           [this](const PropertyList&) -> ReturnValue {
                               robot_->Stop();
                               return true;
                           });

        mcp_server.AddTool(
            "self.walle.pose",
            "控制瓦力头部、眼睛和胳膊。action可用neutral、look_up、look_down、look_left、"
            "look_right、left_arm_up、left_arm_down、right_arm_up、right_arm_down、"
            "both_arms_up、both_arms_down、eyes_left、eyes_right、eyes_center。",
            PropertyList({Property("action", kPropertyTypeString)}),
            [this](const PropertyList& properties) -> ToolResult {
                const std::string& action_text = properties["action"].value<std::string>();
                const auto action = wall_e::ParsePoseAction(action_text);
                if (!action.has_value()) {
                    return std::unexpected("不支持的舵机动作: " + action_text);
                }
                if (!robot_->SetPose(*action)) {
                    return std::unexpected("舵机控制器未就绪或命令队列已满");
                }
                return true;
            });

        mcp_server.AddTool("self.walle.get_status", "获取瓦力底盘和舵机控制状态。",
                           PropertyList(), [this](const PropertyList&) -> ReturnValue {
                               return robot_->GetStatus();
                           });
    }

    i2c_master_bus_handle_t i2c_bus_{nullptr};
    esp_lcd_panel_io_handle_t panel_io_{nullptr};
    esp_lcd_panel_handle_t panel_{nullptr};
    std::unique_ptr<Display> display_;
    std::unique_ptr<wall_e::WallEController> robot_;
    Button boot_button_;
    Button touch_button_;
    Button volume_up_button_;
    Button volume_down_button_;
};

DECLARE_BOARD(WallES3Board);
