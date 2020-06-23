
## ESP-Drone

* [中文版](./README_cn.md)

### Introduction

**ESP-Drone** is an open source **drone solution** based on espressif **ESP32-S2 / ESP32** Wi-Fi chip, which can be controlled through **Wi-Fi** network using mobile APP or gamepad. ESP-Drone supports multiple flight modes, `stabilize`, `height-hold`, `position-hold` and more. With **simple hardware**, **clear and extendible code architecture**, ESP-Drone can be used in **STEAM education** and other fields. The main code ported from **Crazyflie** open source project with **GPL3.0** protocol.

![ESP-Drone](./docs/_static/espdrone_s2_v1_2_2.png)

**Getting Started**: [Getting Started](https://docs.espressif.com/projects/espressif-esp-drone/zh_CN/latest/gettingstarted.html)
**Hardware Schematic**：[Hardware](https://docs.espressif.com/projects/espressif-esp-drone/zh_CN/latest/_static/ESP32_S2_Drone_V1_2/SCH_Mainboard_ESP32_S2_Drone_V1_2.pdf)
**iOS APP Sourcecode**: [ESP-Drone-iOS](https://github.com/EspressifApps/ESP-Drone-iOS)
**Android APP Sourcecode**: [ESP-Drone-Android](https://github.com/EspressifApps/ESP-Drone-Android)

### Features

1. Stabilize Mode
2. Height-hold Mode
3. Position-hold Mode
4. App Control
5. CFclient Supported

> 2/3:expansion module requried

### Third Party Copyrighted Code

Additional third party copyrighted code is included under the following licenses.

| Component | License | Origin |commit id |
| :---:  | :---: | :---: |:---: |
| core/crazyflie | GPL-3.0 |[Crazyflie](https://github.com/bitcraze/crazyflie-firmware) |a2a26abd53a5f328374877bfbcb7b25ed38d8111|
| lib/dsp_lib |  | [esp32-lin](https://github.com/whyengineer/esp32-lin/tree/master/components/dsp_lib) |6fa39f4cd5f7782b3a2a052767f0fb06be2378ff|

### THANKS

1. Thanks to the Bitcraze for the great [Crazyflie project](https://www.bitcraze.io/%20)
2. Thanks to Espressif for the powerful [ESP-IDF environment](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s2/get-started/index.html)
3. Thanks to WhyEngineer for the useful [ESP-DSP lib](https://github.com/whyengineer/esp32-lin/tree/master/components/dsp_lib)

