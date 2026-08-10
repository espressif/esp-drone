# ESP-IDF 6.1 重构执行记录

本文按提交记录实施结果、验证证据和暂未覆盖项。总体计划见 `idf-v6.1-refactor-plan.md`。

## R1：启动安全与输入边界

状态：完成

实施内容：

- 飞控改为默认 disarmed，必须经过显式运行时命令才允许电机输出。
- UDP 接收拒绝空包、仅校验和包和超长包；发送为校验和预留 1 字节并做双重长度检查。
- Wi-Fi SSID/密码在初始化前验证，使用有界格式化；队列分配失败会回滚。
- legacy app 包长度检查不再越界，生成的 RPYT CRTP payload 长度修正为 14 字节。
- CRTP callback port 拒绝负值和 `port >= 16`。
- ESP-NOW 控制帧在解引用前验证长度，绑定表做容量检查，删除时正确搬移尾条目。
- ESP-NOW 持久化数据不再包含函数指针，只保存绑定条目；使用新 key `bindlist_v2`，旧绑定需重新配对。
- I2C bit 读写校验参数和底层读取结果，不再使用未初始化值。
- ESP-NOW 组件最低版本提升到 2.5.3，以包含官方 IDF 6.0、长度和解绑修复。

验证：

- `git diff --check` 通过。
- 已人工核对所有 P0 越界入口均在数组访问前拒绝非法长度/索引。
- 完整 IDF 6.1 编译安排在 R2；当前基线仍会先被已知的 LEDC 组件依赖缺失阻断。

兼容性说明：

- 合法 UDP CRTP 帧格式未变化。
- ESP-NOW `bindlist_v2` 不读取旧版将回调指针一并落盘的 blob，升级后控制器需要重新绑定。

## R2：ESP-IDF 6.1 / ESP32-S2 构建基线

状态：完成

基线：

- ESP-IDF 分支：`release/v6.1`
- ESP-IDF 提交：`6a9c44fe7e725af45cb99293ae38afd7d481f1e3`
- ESP-IDF 版本：`v6.1-beta1-685-g6a9c44fe7e7`（`6.1.0`）
- 目标：`esp32s2`
- ESP-NOW：`2.5.3`

实施内容：

- 将 `EXTRA_COMPONENT_DIRS` 改为 CMake 列表并显式列出叶子组件，消除空格拼接和路径歧义。
- 为 LEDC、GPIO 和 SPI 用户补齐 IDF 6.x 要求的直接组件依赖。
- ADC 从已移除的 `driver/adc.h` / `esp_adc_cal.h` 迁移到 oneshot 与 calibration API；初始化失败不再把驱动标记为可用。
- 移除 `pm_esplane.h` 对旧 ADC 头文件的无效依赖。
- Wi-Fi interface 常量迁移为 `WIFI_IF_AP`。
- 四元数解压先写入本地四元素数组，再逐成员赋值，避免越过单个成员对象边界。
- 更新 IDF 6.1 的 sdkconfig 名称，显式启用 ESP-NOW 所需的 Protocomm Security v1。
- S2 的 IRAM/DRAM 共用 SRAM；关闭可选的 Wi-Fi IRAM/RX IRAM 加速以给飞控状态和任务栈保留内部 RAM。协议功能不变，代价是 Wi-Fi 热路径吞吐性能下降。

验证命令：

```sh
. /home/libo/esp/release_v6_1/esp-idf/export.sh
idf.py -B /tmp/esp-drone-v61-s2-r2 \
  -D SDKCONFIG=/tmp/esp-drone-v61-s2-r2/sdkconfig \
  -D IDF_TARGET=esp32s2 build
idf.py -B /tmp/esp-drone-v61-s2-r2 size
```

验证结果：

- 完整编译、链接和镜像生成通过。
- `ESPDrone.bin`：`0xe8b10` 字节，1 MiB app 分区剩余 `0x174f0` 字节（9%）。
- DIRAM：使用 162,741 / 172,032 字节（94.6%），剩余 9,291 字节。
- `git diff --check` 通过。

后续项：

- 旧 I2C driver 在 IDF 6.1 仍可构建但已 EOL，安排在 R4 迁移到 `driver/i2c_master.h`。
- IDF 当前分支自身仍会输出 NimBLE/FatFS Kconfig 提示和 Wi-Fi 私有 include 校验提示；均不来自本项目，未阻断构建。
- 本阶段仅建立可复现的软件构建基线；串口刷写和运行时验证安排在硬件验证阶段。

## R3：板型、SoC 能力与 transport 解耦

状态：完成

实施内容：

- 新增统一 `transport` 接口，CRTP 和系统启动不再直接依赖 `wifilink`；native Wi-Fi、remote Wi-Fi 骨架和阻塞式 `none` link 由 Kconfig 选择。
- ESP32-S2/S3 默认使用 native Wi-Fi；ESP32-P4 默认使用 `TARGET_PORTABLE_SAFE + TRANSPORT_NONE`，不再伪装成 S2 板型。
- P4/no-radio 构建从项目 Wi-Fi 源文件和 ESP-NOW managed component 中隔离；`transport_none` 的接收端带 100 ms 阻塞，避免空闲 CRTP task 忙轮询。
- portable safe 启动路径不会初始化 LED、ADC、I2C、传感器、LEDC 电机、蜂鸣器、估算器和 stabilizer，也不会运行板级 app；只启动核心诊断、CRTP/comm 和 worker，保证未知 P4/S31 板卡不会因占位引脚产生输出。
- P4 的 IDF 6.1 bootloader 大于旧布局在 `0x8000` 前的可用空间，目标默认值将 partition table 移到 `0x10000`，app 相应从 `0x20000` 开始。
- 为 IDF 公共 `MIN`/`MAX` 宏增加保护，解除 RISC-V/P4 的宏重定义 `-Werror`。
- remote Wi-Fi 目前只保留显式集成入口；仍需具体 `esp-extconn` transport、无线 companion 固件和板级连接配置，不计入本阶段运行验收。

验证命令：

```sh
. /home/libo/esp/release_v6_1/esp-idf/export.sh
idf.py -B /tmp/esp-drone-v61-s2-r2 \
  -D IDF_TARGET=esp32s2 \
  -D SDKCONFIG=/tmp/esp-drone-v61-s2-r2/sdkconfig build
idf.py -B /tmp/esp-drone-v61-p4-r3b \
  -D IDF_TARGET=esp32p4 \
  -D SDKCONFIG=/tmp/esp-drone-v61-p4-r3b/sdkconfig build
```

验证结果：

- ESP32-S2/native Wi-Fi 完整编译、链接和镜像生成通过；`ESPDrone.bin` 为 `0xe8bc0` 字节，app 分区剩余 9%。
- ESP32-P4/no-radio 完整 RISC-V 编译、链接和镜像生成通过；`ESPDrone.bin` 为 `0x3b210` 字节，app 分区剩余 77%。
- P4 的 `build.ninja` 不含 `wifi_esp32.c`、`wifilink.c` 或 `espnow_ctrl.c`；ELF 符号表不含 `wifiInit`、`wifilink*` 或 `espnow_ctrl*`。
- P4 sdkconfig 确认为 `IDF_TARGET=esp32p4`、`TARGET_PORTABLE_SAFE=y`、`TRANSPORT_NONE=y`。
- `git diff --check` 通过。

## R4a：I2C master bus/device API 迁移

状态：完成

实施内容：

- 删除项目对 EOL `driver/i2c.h`、command link、`i2c_param_config` 和 `i2c_driver_install` 的使用，改为 IDF 6.1 `driver/i2c_master.h`。
- 每条物理总线持有 `i2c_master_bus_handle_t`，按 7-bit 地址惰性缓存 `i2c_master_dev_handle_t`；同一总线仍由 mutex 串行化，保留传感器层的同步语义。
- 8/16-bit 寄存器读取使用 repeated-start `i2c_master_transmit_receive`，写入使用 multi-buffer transmit，避免动态拼包。
- 初始化只有在 mutex 和 master bus 都创建成功后才标记完成；失败会释放 mutex 并向上返回 `false`。
- 补上此前仅声明、未实现的无寄存器地址 `i2cdevWrite`；统一拒绝空指针、零长度和非法 7-bit 地址。
- bit-field 读写统一校验 `bitStart/length`，避免负移位或超出 8-bit 寄存器。
- I2C 组件改为直接依赖 `esp_driver_i2c`。旧传感器仍通过兼容头取得 `M2T`，后续清理 STM32 compatibility layer 时统一替换。

验证结果：

- 全仓库不再包含 `#include "driver/i2c.h"`。
- ESP32-S2/native Wi-Fi 完整编译、链接和镜像生成通过；`ESPDrone.bin` 为 `0xe9050` 字节，app 分区剩余 9%。
- ESP32-P4/no-radio 完整编译、链接和镜像生成通过；`ESPDrone.bin` 为 `0x3b0b0` 字节，app 分区剩余 77%。
- `git diff --check` 通过。

暂未覆盖：

- I2C 设备在线读写与采样周期需要在 S2 硬件阶段验证。
- EEPROM/NVS 和 motor LEDC 错误传播属于 R4b。

## R4b：NVS 配置存储与电机输出失败闭环

状态：完成

实施内容：

- 将原来所有读写都返回成功、但实际不保存数据的 EEPROM 桩替换为 NVS blob；保留 8 KiB 地址空间兼容 API，未写区域按真实擦除 EEPROM 返回 `0xff`。
- NVS 读写增加地址/长度溢出检查、mutex、分配失败处理、`nvs_commit` 和错误返回；partial write 会保留 blob 中其他地址的数据。
- config block 不再为了 flash 存储初始化无关的 I2C1/deck bus；NVS 打开失败会使 config block 初始化失败，而不是伪报成功。
- motor timer、四个 LEDC channel、初始 0 duty、运行时 duty 更新均检查返回值。任一 channel 初始化失败会停止已配置 channel；运行时失败会尝试把对应输出拉回 0 并把驱动标记为 unhealthy。
- `motorsInit` 和 `motorsSetRatio` 返回真实结果，power distribution self-test 会传播初始化失败；非法 motor id 不再依赖断言后继续访问数组。
- 新增 `MOTORS_STARTUP_TEST`，默认关闭。固件自检默认只验证四路 PWM 保持 0 duty，不再在每次上电时自动转动电机；需要实物旋转测试时必须显式启用并先拆桨。
- 电池补偿路径拒绝无效供电电压，避免除零后把异常值送入 PWM。

验证结果：

- S2 与 P4 sdkconfig 均确认为 `CONFIG_MOTORS_STARTUP_TEST is not set`。
- ESP32-S2/native Wi-Fi 完整编译、链接和镜像生成通过；`ESPDrone.bin` 为 `0xe9510` 字节，app 分区剩余 9%。
- ESP32-P4/no-radio 完整编译、链接和镜像生成通过；`ESPDrone.bin` 为 `0x3b160` 字节，app 分区剩余 77%。
- `git diff --check` 通过。

硬件阶段待验证：

- NVS config block 首次写入及复位后读取。
- S2 四路 PWM 上电保持低 duty；本轮不会主动执行旋转测试。

## R5：ESP32-P4 / ESP32-S31 构建与架构中立数学接口

状态：完成（S31 为 preview，增强 ESP-NOW 有明确上游阻塞）

实施内容：

- 新增 `sdkconfig.defaults.esp32s31`：使用 `TARGET_PORTABLE_SAFE`，保留 S31 原生 Wi-Fi/UDP，partition table 移到 `0x10000` 以容纳 IDF 6.1 bootloader。
- 将飞控侧矩阵、三角、平方根和均值调用收口到架构中立的 `dsp_*` facade；现有 `xtensa_*` 纯 C 实现只作为内部兼容 backend。Kalman 和 flow deck 活跃代码不再暴露 CPU 架构命名，并同时通过 Xtensa S2 与 RISC-V P4/S31 编译。
- 原生 Wi-Fi、UDP 和增强 ESP-NOW 控制通道变为独立能力。S2 继续启用 `espressif/esp-now 2.5.3`；S31 保留原生 Wi-Fi/UDP，但不把增强组件作为启动依赖。
- 组件依赖图按 SoC 固定能力解析，Kconfig 负责选择 transport 源码；P4 no-radio 仍不引入项目 Wi-Fi/ESP-NOW 实现。
- `transport_none` 实现只在 no-radio 配置编译，消除 native Wi-Fi 构建中的无效对象。

S31 preview 兼容性证据：

- `espressif/esp-now 2.5.3` 在 `src/espnow/src/espnow.c:286` 访问 `wifi_pkt_rx_ctrl_t.secondary_channel`。
- 当前 `release/v6.1` S31 preview 的 `esp_wifi_rxctrl_t` 没有该字段，因此增强组件会编译失败。工程没有修改或固化 managed component 补丁，而是通过 `CONFIG_ESPNOW_CONTROL` 和 manifest target rule 隔离该功能。
- 这不表示 S31 芯片没有 ESP-NOW 能力；结论仅限当前固定 IDF revision 与增强组件版本的组合。恢复前需要上游组件兼容和通信实测。

验证命令：

```sh
. /home/libo/esp/release_v6_1/esp-idf/export.sh
idf.py -B /tmp/esp-drone-v61-s2-r2 -D IDF_TARGET=esp32s2 \
  -D SDKCONFIG=/tmp/esp-drone-v61-s2-r2/sdkconfig reconfigure build
idf.py -B /tmp/esp-drone-v61-p4-r3b -D IDF_TARGET=esp32p4 \
  -D SDKCONFIG=/tmp/esp-drone-v61-p4-r3b/sdkconfig reconfigure build
idf.py --preview -B /tmp/esp-drone-v61-s31-r5 -D IDF_TARGET=esp32s31 \
  -D SDKCONFIG=/tmp/esp-drone-v61-s31-r5/sdkconfig reconfigure build
```

验证结果：

- ESP32-S2/native Wi-Fi + enhanced ESP-NOW：`ESPDrone.bin` 为 `0xe9510` 字节，app 分区剩余 9%。
- ESP32-P4/portable safe + no-radio：`ESPDrone.bin` 为 `0x3b160` 字节，app 分区剩余 77%。
- ESP32-S31/portable safe + native Wi-Fi/UDP：`ESPDrone.bin` 为 `0xddfb0` 字节，app 分区剩余 13%。
- S31 sdkconfig 确认为 `IDF_TARGET=esp32s31`、`TARGET_PORTABLE_SAFE=y`、`TRANSPORT_NATIVE_WIFI=y`，且 `ESPNOW_CONTROL` 未启用。
- 三目标均使用 IDF `6a9c44fe7e725af45cb99293ae38afd7d481f1e3`，完整编译、链接和镜像生成通过。

生产门槛：

- S31 仍需 ESP-IDF `v6.1.1+` 状态复核、增强 ESP-NOW 上游兼容和真实板卡 pin map/硬件验证。
- P4 remote Wi-Fi 仍需选定 companion、`esp-extconn` transport 和真实板卡连接，不计入本阶段验收。

## R6a：S2 硬件首轮启动与关键传感器失效闭环

状态：完成（硬件回归记录见 R6b）

首轮硬件证据：

- `/dev/ttyUSB0` 可识别为 ESP32-S2 rev v0.0，MAC 为 `7c:df:a1:00:40:08`，外部 flash 为 4 MiB。
- IDF 6.1 固件完成刷写和镜像 hash 校验；原生 Wi-Fi、UDP 与增强 ESP-NOW 正常启动，AP 为 `ESP-DRONE_7CDFA1021008`。
- I2C0 在 SDA 11、SCL 10 上初始化，但 MPU6050 事务超时。原实现随后执行 `assert(0)`，设备约每 2.5 秒复位一次。
- 当前连接平台未能确认 MPU6050 在线，因此该结果不能用于传感器采样或飞行闭环验收；但它暴露了任何关键 IMU 缺失时都会发生的确定性故障路径。

修复内容：

- I2C 初始化日志包含总线、SDA/SCL 和时钟，便于区分 pin map、连线和器件故障。
- 传感器总线或 MPU6050 初始化失败不再触发断言，也不会创建中断与采样任务；失败会传播到系统自检。
- 稳定器任务在关键传感器自检失败后显式停止电机并停留在低频诊断循环，避免复位风暴，也防止远程强制启动绕过传感器条件。

构建验证：

- ESP32-S2/native Wi-Fi + enhanced ESP-NOW 完整编译、链接和镜像生成通过；`ESPDrone.bin` 为 `0xe95b0` 字节，app 分区剩余 9%。
- `git diff --check` 通过。

R6b 待验证：

- 刷入修复后的提交，连续观察复位稳定性和电机安全状态。
- 对 UDP 空包、短包、超长包及合法校验帧执行边界输入测试。
- 复位后复核 NVS config block 初始化路径。
