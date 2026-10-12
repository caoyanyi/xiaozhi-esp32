# Wall-E ESP32-S3

中文采购、供电和接线步骤请先阅读：[瓦力 ESP32-S3 接线与电源采购指南](../../../docs/wall-e-s3-wiring-and-power-zh-CN.md)。

Custom XiaoZhi board support for an ESP32-S3 N16R8 Wall-E robot using one 128x64 SH1106 OLED,
a PCA9685 servo controller, and an L298P dual motor driver. Existing microphone, speaker, OLED,
buttons, and voice-assistant behavior are inherited from the breadboard ESP32-S3 setup.

## Signal wiring

| Device | Signal | ESP32-S3 GPIO |
|---|---|---:|
| OLED + PCA9685 | SDA | 41 |
| OLED + PCA9685 | SCL | 42 |
| L298P left | ENA / IN1 / IN2 | 8 / 9 / 10 |
| L298P right | ENB / IN3 / IN4 | 11 / 12 / 13 |

The OLED normally uses address `0x3c`; PCA9685 uses `0x40`.

## Servo channels

| PCA9685 channel | Function |
|---:|---|
| 0 | left eye |
| 1 | right eye |
| 2 | left arm |
| 3 | right arm |
| 4 | lower neck |
| 5 | upper neck |
| 6 | head yaw |
| 7 | reserved and disabled |

The initial limits in `wall_e_controller.cc` are deliberately conservative. Center each servo
before attaching its horn, then calibrate direction and mechanical limits one channel at a time.

## Power

Do not power motors or servos from USB, the ESP32 board, or a breadboard rail. Use a dedicated
5V supply sized for the installed servos (15A recommended for the listed MG90S/MG92B set), feed
the PCA9685 servo `V+` rail separately, and join all grounds at the power distribution point.

## Build

```sh
python3 scripts/build.py wall-e-s3 --name wall-e-s3
```

The merged image is written to `build/merged-binary.bin`.
