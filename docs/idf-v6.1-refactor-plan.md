# ESP-IDF 6.1 / ESP32-P4 / ESP32-S31 重构计划

日期：2026-08-10

依据：ESP-IDF 官方 `release/v6.1` 分支 `6a9c44fe7e725af45cb99293ae38afd7d481f1e3`（本地描述 `v6.1-beta1-685-g6a9c44fe7e7`）以及乐鑫官方文档和组件注册表。

## 1. 目标与边界

本轮重构目标：

1. ESP32-S2 在 ESP-IDF `release/v6.1` 上可构建、可烧录、默认安全且能完成现有硬件自检。
2. ESP32-S31 形成原生 Wi-Fi/UDP 构建，复用飞控核心和 transport 接口；增强 ESP-NOW 在组件支持到位后独立启用。
3. ESP32-P4 形成可构建的无无线安全配置，并预留官方 Wi-Fi Expansion 后端；只有配置外置无线芯片后才启用 UDP/ESP-NOW。
4. 板型、SoC 能力和 transport 分离，不再通过 `CONFIG_IDF_TARGET_*` 隐式选择引脚。
5. 删除 IDF 6.1 已移除的 ADC API，并停止新增对 EOL legacy I2C API 的依赖。

硬件边界：仓库只有 ESP32-S2 Drone/ESPlane 的原理图和引脚定义，没有 P4/S31 飞控板原理图。因此本轮可验证 S2 的真实硬件行为；P4/S31 的验收先做到目标编译和安全 no-actuator 默认配置。具体 P4/S31 电机、ADC 和传感器引脚必须在对应 PCB 确定后单独验收，不能把 S2 引脚直接视为兼容。

## 2. 官方支持结论

### 2.1 版本状态

- `release/v6.1` 当前包含 `esp32p4`、`esp32s2` 和 `esp32s31` target。
- 分支的 target 列表把 ESP32-P4、ESP32-S2 列为 supported，把 ESP32-S31 列为 preview。
- ESP32-S31 官方 FAQ 给出的首个预发布版本是 `v6.1-beta1`，预期正式支持版本是 `v6.1.1`，并建议正式支持前使用最新 master。结论：可以完成 preview 构建支持，但生产发布门槛设为 `v6.1.1+` 和官方功能状态复核。

### 2.2 芯片能力矩阵

以下能力由 `release/v6.1` 的 `soc_caps.h` 和官方 Wi-Fi Expansion 文档确认：

| 能力 | ESP32-S2 | ESP32-P4 | ESP32-S31 | 工程决策 |
|---|---:|---:|---:|---|
| CPU | Xtensa，单核 | RISC-V，双核 | RISC-V，双核 | 核心代码不得依赖 Xtensa 头文件或指令 |
| ADC | 支持 | 支持 | 支持 | 使用 ADC oneshot；通道来自 board profile |
| I2C | 支持 | 支持 | 支持 | 使用新 master bus/device API |
| LEDC | 8 通道 | 8 通道 | 8 通道 | 可作为四路 brushed motor PWM 后端 |
| 片上 Wi-Fi | 支持 | 不支持 | 支持（Wi-Fi 6） | native Wi-Fi 只在 `SOC_WIFI_SUPPORTED` 时编译 |
| P4 Wi-Fi | 不适用 | 需外置 Wi-Fi 芯片 | 不适用 | 使用官方 `esp_wifi_remote`/Wi-Fi Expansion；默认关闭 |
| ESP-NOW | 原生 | 取决于外置 Wi-Fi 后端 | 芯片原生能力/IDF preview | 增强组件必须可选，不能作为 Wi-Fi 或系统启动硬依赖 |

### 2.3 ESP-IDF 6.x 迁移约束

- v6.0 已移除 legacy ADC 和 legacy ADC calibration API；当前 `adc1_get_raw()`、`esp_adc_cal_*` 必须替换。
- legacy I2C (`driver/i2c.h`) 在 v6.0 已进入 EOL，计划 v7.0 删除；本轮直接迁移到 `driver/i2c_master.h`。
- legacy `driver` 组件不再传递 LEDC、SPI 等驱动依赖；每个工程组件必须显式声明 `esp_driver_ledc`、`esp_driver_spi`、`esp_driver_i2c` 等依赖。
- 外设公共头不再隐式包含 FreeRTOS 头；源码必须显式 include 所用的 FreeRTOS 类型。
- 当前增强 ESP-NOW 组件最新版为 `espressif/esp-now 2.5.3`，注册表声明支持 ESP-IDF `>=4.4` 和所有 target；其变更记录明确包含 IDF 6.0 编译修复、绑定表删除修复和接收长度验证修复。工程应从 `2.5.2` 升级并减少私有复制代码。

## 3. 目标架构

```text
main
  -> app_boot (NVS + board + safety state)
  -> flight_core
       -> flight_transport API
            -> wifi_native (S2/S31)
            -> wifi_remote (P4，可选)
            -> transport_none (构建/安全模式)
       -> flight_storage API -> NVS
       -> actuator API -> LEDC motor backend
       -> sensor bus API -> IDF I2C master backend
  -> board profile
       -> esp32s2_drone_v1_2
       -> portable_safe (P4/S31 初始编译配置，输出默认禁用)
```

依赖规则：

1. `flight_core` 只能依赖接口和标准 C/FreeRTOS，不依赖 board 或具体 ESP-IDF 外设头。
2. board profile 可以依赖 SoC capability，但 transport 不得反向依赖飞控模块。
3. P4 没有配置 Wi-Fi Expansion 时，native Wi-Fi 组件不进入构建；系统可启动诊断但保持 disarmed。
4. 初始化函数返回 `esp_err_t` 或明确的状态对象；`isInit` 只表示完成，不替代错误传播。

## 4. 执行阶段与提交边界

### R1：安全和输入边界

改动：

- 默认 disarmed；增加编译期显式 opt-in，禁止隐式上电 armed。
- 修复 UDP 长度、发送缓冲、密码/SSID、CRTP port 边界。
- 修复 ESP-NOW 绑定表容量和删除逻辑，升级组件约束至 2.5.3。
- 修复 legacy app 越界和 I2C bit 操作未初始化值。

验收：边界测试通过；`git diff --check` 通过；不改变合法 CRTP 包格式。

提交建议：`fix: harden control input and startup safety`

### R2：IDF 6.1 构建基线

改动：

- 顶层 `EXTRA_COMPONENT_DIRS` 改为 CMake list。
- 补全 LEDC、SPI、I2C、NVS、netif/event 等直接组件依赖。
- 删除对 `FREERTOS_ORIG_INCLUDE_PATH` 和兼容 include 路径的依赖，统一 `freertos/...` include。
- 更新通用 sdkconfig 名称，新增 6.1 CI 构建脚本/矩阵。

验收：ESP32-S2 在固定 `release/v6.1` revision 上完整链接；构建日志不再出现工程自身的缺依赖错误。

提交建议：`build: establish ESP-IDF 6.1 baseline`

### R3：板型与 transport 解耦

改动：

- Kconfig 分离 `BOARD_*`、`TRANSPORT_*` 和 SoC capability。
- 把 CRTP 对 `wifilink` 的固定引用改成统一 transport 注册接口。
- native Wi-Fi 组件以 `SOC_WIFI_SUPPORTED` 为构建条件；实现 `transport_none`。
- P4 profile 默认 `transport_none`；增加 remote Wi-Fi 选项和依赖说明，但不在没有协处理器硬件时声称运行验证。

验收：S2 native Wi-Fi 与 P4 no-radio 均可链接；P4 构建不直接调用片上 Wi-Fi 初始化。

提交建议：`refactor: separate board and transport capabilities`

### R4：外设和存储迁移

改动：

- ADC 改为 `adc_oneshot_*` + target 可用的 calibration scheme。
- I2C 改为 bus handle + device handle；现有设备 API 暂由兼容适配层承接，避免一次重写所有传感器。
- EEPROM 假实现替换为 NVS blob；保留 config block 版本和 checksum 迁移逻辑。
- 电机 LEDC 后端检查所有返回值，失败时归零并保持 disarmed。

验收：S2 ADC/I2C/NVS 组件测试通过；初始化失败不会被报告为成功；四路 PWM 启动保持 0 duty。

提交建议：按 `refactor: migrate ...` 拆成 I2C、ADC/storage 两个可独立回滚提交。

### R5：P4/S31 构建支持

改动：

- 增加 `sdkconfig.defaults.esp32p4`、`sdkconfig.defaults.esp32s31` 和 portable safe board defaults。
- 用 `SOC_*_SUPPORTED` 能力宏编译 ADC/I2C/LEDC/Wi-Fi 后端。
- 将数学库接口改为架构无关命名；先复用纯 C 实现，后续再按芯片选择优化实现。
- S31 以 preview target 构建，记录 IDF revision；P4 以 no-radio 构建，remote Wi-Fi 作为可选扩展。

实施校正（2026-08-10）：三目标实编译发现 `espressif/esp-now 2.5.3` 在 S31 上访问 `wifi_pkt_rx_ctrl_t.secondary_channel`，而当前 S31 preview 头文件没有该成员。工程不修改 managed component，改为把增强 ESP-NOW 控制通道从原生 Wi-Fi/UDP 能力中拆出；S31 暂时关闭前者，S2 保持启用。恢复条件是上游组件明确兼容 S31 后重新完成三目标构建和通信回归。

验收：`idf.py --preview set-target esp32s31`、`idf.py set-target esp32p4` 和 S2 三目标完整构建。S31 生产状态仍为“等待 v6.1.1 和硬件验证”。

提交建议：`build: add ESP32-P4 and ESP32-S31 targets`

### R6：ESP32-S2 实机验证

步骤：

1. 读取 `/dev/ttyUSB0` 芯片身份，确认目标确为 ESP32-S2。
2. 构建并烧录；保留烧录命令、IDF revision 和 image hash。
3. 观察启动、自检、heap、传感器探测和异常日志。
4. 验证默认 disarmed、无控制包时电机 duty 为 0。
5. 用合法/空/超长 UDP 包验证进程不崩溃且非法包被丢弃。
6. 若控制端可用，再验证 CRTP/ESP-NOW；没有第二个 ESP-NOW 设备时明确记为未覆盖。

验收：连续运行至少 2 分钟无重启/看门狗；启动和边界验证证据落盘。

提交建议：`docs: record ESP32-S2 hardware validation`

## 5. 构建矩阵与完成定义

| Target | Board profile | Transport | 编译 | 实机 | 完成条件 |
|---|---|---|---:|---:|---|
| esp32s2 | ESP32-S2 Drone v1.2 | native Wi-Fi | 必须 | 必须 | build + flash + boot + safe idle + UDP 边界 |
| esp32p4 | portable safe | none | 必须 | 无硬件 | 完整链接且不引用 native Wi-Fi；默认 disarmed |
| esp32p4 | 待定制板 | remote Wi-Fi | 配置骨架 | 无协处理器 | 文档化依赖和接口，不计入本轮运行验收 |
| esp32s31 | portable safe | native Wi-Fi/UDP | 必须（preview） | 无硬件 | preview 完整链接；增强 ESP-NOW 等待上游兼容；标注 v6.1.1/硬件验证门槛 |

本轮“支持 P4/S31”的定义是：公共飞控代码和 IDF 适配可为两目标完整编译，缺失硬件信息时默认输出安全关闭；不是宣称 S2 PCB 可直接替换芯片，也不是未经硬件测试的适航声明。

## 6. 风险与回滚

- 飞控实时性：I2C/ADC API 迁移后测量采样周期和 stabilizer deadline；每个驱动迁移独立提交，可单独回滚。
- 通信兼容：保留 CRTP wire format；transport 只搬运完整帧。UDP/ESP-NOW 改动需同时保留合法包回归。
- S31 preview：固定构建 revision，不把 preview 构建等同于正式支持。
- P4 无原生无线：没有 companion chip 时不做虚假 fallback，transport 显式为 none。
- 板级未知：portable safe profile 不提供可驱动电机的默认引脚；新增真实板型必须提交原理图来源、pin map 和台架验收。

## 7. 官方资料

- [ESP-IDF v6.1-beta1 release](https://github.com/espressif/esp-idf/releases/tag/v6.1-beta1)
- [ESP-IDF v6.0 peripheral migration guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/migration-guides/release-6.x/6.0/peripherals.html)
- [ESP-IDF v6.1 peripheral migration guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32p4/migration-guides/release-6.x/6.1/peripherals.html)
- [ESP32-P4 Wi-Fi Expansion](https://docs.espressif.com/projects/esp-idf/en/latest/esp32p4/api-guides/wifi-expansion.html)
- [ESP32-S31 ESP-IDF compatibility FAQ](https://esp32-s31.espressif.com/en/faq#faq-heading-idf)
- [ESP32-S31 support status](https://developer.espressif.com/hardware/esp32s31/)
- [espressif/esp-now 2.5.3](https://components.espressif.com/components/espressif/esp-now/versions/2.5.3)
