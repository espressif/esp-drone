# ESP-Drone 架构与缺陷审查

日期：2026-08-10

审查基线：`master@cbe4e80`

范围：应用入口、飞控核心、平台层、外设驱动、通信、构建配置和 CI

## 1. 结论

当前工程可以概括为“Crazyflie 单体飞控核心 + ESP32 专用适配代码”。算法、任务、通信、板级配置和具体 ESP-IDF 驱动互相直接依赖，尚未形成可替换的 BSP/HAL 边界。`crazyflie` 组件同时依赖 `platform`、`wifi`、`motors`、传感器和算法库，而 `platform`、`wifi`、`motors` 又反向依赖 `crazyflie`。顶层通过 `__PROJECT_GROUP_LINK_COMPONENTS` 允许循环链接，隐藏了组件边界问题。

审查确认了 4 个 P0、6 个 P1 问题。迁移到 ESP-IDF 6.x 的首次构建也已在编译阶段失败，首个错误是 `motors` 未声明 `esp_driver_ledc` 依赖。建议先修安全和内存边界，再解开通信/板级/芯片能力，最后迁移遗留驱动。

## 2. 当前架构

启动链路：

```text
app_main
  -> NVS 初始化
  -> platformInit（选择 ED12/EP20）
  -> systemLaunch
       -> wifiInit
       -> systemInit（CRTP、配置、ADC、LED、电源、蜂鸣器）
       -> commInit（固定选择 wifilink）
       -> commander / estimator / stabilizer / sensors / motors
```

主要模块：

| 层次 | 目录 | 当前职责 | 主要问题 |
|---|---|---|---|
| 入口 | `main` | NVS、平台初始化、启动系统任务 | 直接假设 Wi-Fi 需要 NVS；失败时忙等 |
| 飞控核心 | `components/core/crazyflie` | CRTP、状态估计、控制器、稳定器、日志、参数、任务编排 | 约 70 个源文件组成单一组件；直接依赖具体驱动 |
| 平台 | `components/platform` | 设备类型和传感器/电机映射 | 设备字符串硬编码为 `EP20`；没有真正的板级初始化 |
| 通信 | `drivers/general/wifi`、`wifilink.c` | SoftAP UDP、ESP-NOW、CRTP 适配 | 传输、网络和协议转换混在一个实现中 |
| 外设 | `drivers/general`、`i2c_bus`、`i2c_devices`、`spi_devices` | 电机、ADC、I2C/SPI 传感器 | 使用多套 ESP-IDF 遗留 API；芯片能力和引脚硬编码 |
| 数学 | `components/lib/dsp_lib` | 由 CMSIS-DSP 派生的浮点实现 | 以 `xtensa_*` 命名并直接暴露给核心，妨碍 RISC-V 目标识别和替换 |
| 构建 | 根目录及各组件 `CMakeLists.txt` | 手工枚举组件和源文件 | 循环依赖、依赖声明缺失、使用空格拼接 CMake 列表 |

## 3. 已确认缺陷

### P0：安全或内存破坏

| ID | 位置 | 问题与证据 | 影响 | 修复方向 |
|---|---|---|---|---|
| SAF-01 | `modules/src/system.c:76-85` | 未定义 `START_DISARMED` 时 `ARM_INIT` 为 `true` | 飞控默认处于 armed 状态，启动异常或控制链路错误时存在电机误启动风险 | 默认 disarmed；仅通过显式配置和运行时解锁进入 armed |
| MEM-01 | `wifi_esp32.c:95-101, 184-187` | `wifiSendData()` 未校验 `size`；发送任务还会在 `data[size]` 追加校验和 | 当输入为 32 字节时追加校验和越界；更大输入在 `memcpy` 时直接越界 | 明确载荷上限为数组长度减一，入口和发送任务双重校验 |
| MEM-02 | `wifi_esp32.c:143-158`、`wifilink.c:143-147` | UDP 长度为 0 时读取 `rx_buffer[-1]`；仅校验和的数据报会产生 `size=0`，随后 `size-1` 下溢 | 栈越界读、断言或后续大长度复制 | 拒绝 `len < 2`，在 CRTP 适配层再次校验帧长度 |
| MEM-03 | `modules/src/crtp.c:211-217` | `crtpRegisterPortCB()` 只拒绝 `port > 16`，允许 `port == 16` 和负值 | 写越界 `callbacks[16]` 或数组前部 | 使用 `port < 0 || port >= CRTP_NBR_OF_PORTS` |

### P1：确定的功能错误或高可靠性风险

| ID | 位置 | 问题与证据 | 影响 | 修复方向 |
|---|---|---|---|---|
| NET-01 | `wifi_esp32.c:122-129` | UDP `bind()` 失败后仍设置 `isUDPInit=true` 并返回成功 | 后续任务在不可用 socket 上永久重试，`wifiTest()` 仍可能通过 | 失败时关闭 socket并返回错误；初始化状态只在全部成功后置位 |
| NET-02 | `wifi_esp32.c:34-35, 280-292`、Kconfig | 密码允许 64 字符，但缓冲和 IDF AP password 字段均为 64 字节，代码复制 `strlen+1` | 64 字符配置导致末尾越界；SSID 长度也只靠文案约束 | Kconfig 限制密码最大 63；使用 `snprintf`/有界复制并检查截断 |
| NOW-01 | `espnow_ctrl.c:117-123` | 绑定表新增前不检查 `g_bindlist.size < 32` | 第 33 个绑定写出静态数组 | 参数校验、容量检查、返回 `ESP_ERR_NO_MEM` |
| NOW-02 | `espnow_ctrl.c:128-139` | 删除后把被删条目写到旧尾部，没有把尾条目搬到被删位置 | 非尾部条目实际未删除，尾部有效条目反而丢失 | 将 `data[last]` 复制到 `data[i]`，清零旧尾部 |
| STO-01 | `eeprom.c:101-130` | 读写实现被注释，但函数固定返回成功；读缓冲不写入数据，写入不落盘 | 配置块每次启动退回默认值，却报告存储自检通过 | 用 NVS blob/专用分区实现，并对范围、校验和和错误码做真实验证 |
| I2C-01 | `i2c_drv.c:80-102` | `i2c_param_config`/`i2c_driver_install` 失败后仍创建互斥量并把端口标记为已初始化 | 之后不再重试，并可能在无驱动或空互斥量上操作 | 初始化返回错误；仅在驱动和 mutex 全部成功后置位 |

### P2：维护性、诊断性和迁移阻塞项

| ID | 位置 | 问题 | 建议 |
|---|---|---|---|
| API-01 | `i2cdev_esp32.c:56-65, 205-211` | 单 bit 读在底层读取失败时仍使用未初始化局部变量；单 bit 写忽略读取结果 | 失败立即返回，参数范围校验后再移位 |
| APP-01 | `wifilink.c:75-85` | legacy 包要求 `size == 11`，却读取 `data[11]` | 修正协议长度并加单元测试；默认关闭遗留协议 |
| RES-01 | 多个初始化函数 | 未检查 queue/task/semaphore 创建结果 | 统一返回 `esp_err_t`，失败时回滚已创建资源 |
| ARCH-01 | `crazyflie/CMakeLists.txt`、`platform/CMakeLists.txt` 等 | 组件循环依赖并用 group linking 绕过 | 建立 `flight_core`、`board`、`transport` 单向依赖 |
| ARCH-02 | `system.c`、`comm.c` | 系统任务固定调用 Wi-Fi，CRTP 固定选择 `wifilink` | 用 Kconfig/能力宏选择 transport；P4 允许 external Wi-Fi 或 no-radio |
| BLD-01 | 根 `CMakeLists.txt` | `EXTRA_COMPONENT_DIRS` 以字符串拼接，IDF 5+ 明确告警 | 改为原生 CMake list，并只列叶子组件目录 |
| BLD-02 | 多个组件 CMake | 依赖通过其他组件偶然传递；IDF 6.x 首次失败于缺少 `esp_driver_ledc` | 按 include/API 所属组件补齐 `REQUIRES`/`PRIV_REQUIRES` |
| BLD-03 | `sdkconfig.defaults*` | 使用已重命名的芯片专用 CPU/Wi-Fi Kconfig 项 | 改用通用配置项并按 target 分文件 |
| TGT-01 | `config.h:49-61`、`Kconfig.projbuild` | 板型与 SoC 耦合；新 target 会落入错误 choice 或没有引脚默认值 | 板型定义 pin map；SoC 只表达能力，二者在配置期校验兼容性 |
| TGT-02 | `adc_esp32.c` | ADC 通道仅定义 ESP32/S2/S3；使用 legacy ADC API | 迁移到 oneshot + calibration scheme，并从板级配置注入通道 |
| TGT-03 | `wifi_esp32.c` | 假定 SoC 有片上 Wi-Fi | 以 `SOC_WIFI_SUPPORTED`/构建能力选择 native、hosted 或 disabled 后端 |
| CI-01 | `.github/workflows/build_project.yml` | 审计时旧 CI 只在 push 上构建 IDF 4.4/5.0，没有测试和新目标；R7 已替换为 5.5/6.1 矩阵 | 增加 5.5/6.1 的 ESP32/S2/S3/P4 与 6.1 S31 构建矩阵、格式/静态检查和最小主机测试 |

## 4. 构建审查记录

使用本机 ESP-IDF `v6.2-dev-22-g58fc6996082` 对未修改基线执行 ESP32-S2 构建（单独的 `/tmp` 构建目录）：

```text
warning: EXTRA_COMPONENT_DIRS contains leading spaces / string concatenation
warning: sdkconfig defaults contain renamed symbols
fatal error: driver/ledc.h: No such file or directory
hint: add esp_driver_ledc to motors component requirements
```

这说明工程并非仅有 API 弃用告警，而是当前无法通过 ESP-IDF 6.x 的组件依赖检查。该次构建没有烧写设备。

## 5. 建议的目标架构

```text
app
  -> flight_core (controller / estimator / commander / CRTP services)
       -> flight_port (time / sync / storage / sensor samples / actuator / transport interfaces)
            -> board (pin map + enabled devices + safety policy)
                 -> IDF drivers
            -> transport_native_wifi | transport_hosted_wifi | transport_none
```

约束：

1. `flight_core` 不包含 `esp_wifi.h`、`driver/*.h` 或 target 专用宏。
2. 板型决定引脚和器件，SoC 能力决定可用后端；板型与 SoC 不再等价。
3. 电机输出在系统完成自检且显式解锁前必须保持为零。
4. 所有跨任务输入必须先验证长度、来源和状态，再写入控制队列。
5. 存储、transport 和外设初始化使用可传播的错误码，不以 `isInit` 代替健康状态。

## 6. 执行优先级

1. 修复 SAF-01、MEM-01～03、NOW-01～02，并补充可在主机或组件级执行的边界测试。
2. 修正 CMake 组件声明和 target 配置，使 ESP32-S2 在 IDF 6.1 首先恢复可构建。
3. 抽离 board/transport/storage 边界，替换 EEPROM 假实现和遗留 ADC/I2C API。
4. 建立 S2、P4、S31 构建矩阵；P4 默认使用 no-radio 或 hosted transport。
5. 在连接的 ESP32-S2 上做烧录、自检、默认 disarmed、UDP/ESP-NOW 和电机零输出验证。
