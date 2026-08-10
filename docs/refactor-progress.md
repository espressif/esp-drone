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
