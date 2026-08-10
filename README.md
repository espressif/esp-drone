
## ESP-Drone

* [中文](./README_cn.md)

[![Build project](https://github.com/espressif/esp-drone/actions/workflows/build_project.yml/badge.svg)](https://github.com/espressif/esp-drone/actions/workflows/build_project.yml)

### Introduction

**ESP-Drone** is an open-source drone solution for Espressif SoCs. The flight hardware can be controlled by a mobile app or gamepad over **Wi-Fi**. ESP-Drone has **simple hardware** and an extensible architecture, making it suitable for **STEAM education** and prototyping. The flight-control core is derived from the **Crazyflie** open-source project and is licensed under GPL-3.0.

> Supported ESP-IDF branches are [release/v5.5](https://docs.espressif.com/projects/esp-idf/en/release-v5.5/esp32s2/get-started/index.html) and `release/v6.1` only. ESP32-S31 requires the 6.1 preview target. ESP32-P4 currently uses a no-radio diagnostic profile, and P4/S31 flight hardware still requires board-specific pin maps and validation.

![ESP-Drone](./docs/_static/espdrone_s2_v1_2_2.png)

For more information, please check the sections below:
* **Getting Started**: [Getting Started](https://docs.espressif.com/projects/espressif-esp-drone/zh_CN/latest/gettingstarted.html)
* **Hardware Schematic**：[Hardware](https://docs.espressif.com/projects/espressif-esp-drone/zh_CN/latest/_static/ESP32_S2_Drone_V1_2/SCH_Mainboard_ESP32_S2_Drone_V1_2.pdf)
* **iOS APP Source code**: [ESP-Drone-iOS](https://github.com/EspressifApps/ESP-Drone-iOS)
* **Android APP Source code**: [ESP-Drone-Android](https://github.com/EspressifApps/ESP-Drone-Android)

### Features

1. Stabilize Mode
2. Height-hold Mode
3. Position-hold Mode
4. APP Control
5. cfclient support; see https://github.com/leeebo/crazyflie-clients-python
6. ESP-BOX-3 joystick control through ESP-NOW

Note: to implement Height-hold/Position-hold mode, extension boards are needed. For more information, see Hardware Reference. 

### Third Party Copyrighted Code

Additional third party copyrighted code is included under the following licenses.

| Component | License | Origin |Commit ID |
| :---:  | :---: | :---: |:---: |
| core/crazyflie | GPL3.0  |[Crazyflie](https://github.com/bitcraze/crazyflie-firmware/tree/2021.01) |tag_2021_01 b448553|
| lib/dsp_lib |  | [esp32-lin](https://github.com/whyengineer/esp32-lin/tree/master/components/dsp_lib) |6fa39f4c|

### Support Policy

From December 2022, we will offer limited support on this project, but Pull Request is still welcomed!

### THANKS

1. Thanks to Bitcraze for the great [Crazyflie project](https://www.bitcraze.io/).
2. Thanks to Espressif for the powerful [ESP-IDF framework](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s2/get-started/index.html).
3. Thanks to WhyEngineer for the useful [ESP-DSP lib](https://github.com/whyengineer/esp32-lin/tree/master/components/dsp_lib).
