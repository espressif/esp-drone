# ESP32-S2 / ESP-IDF 6.1 硬件验证记录

日期：2026-08-10

设备：`/dev/ttyUSB0`

固件提交：`acc3d61`
ESP-IDF：`release/v6.1`，`6a9c44fe7e725af45cb99293ae38afd7d481f1e3`（`v6.1-beta1-685-g6a9c44fe7e7`）

## 1. 设备与镜像

esptool 识别结果：

- ESP32-S2 revision v0.0，240 MHz，40 MHz crystal。
- MAC：`7c:df:a1:00:40:08`。
- 外部 flash 实际容量 4 MiB；当前通用 S2 镜像头配置为 2 MiB，分区只使用前 2 MiB。本次不为单块验证板扩大产品默认分区。
- 无芯片内置 PSRAM/flash。

构建产物：

| 文件 | 地址 | SHA-256 |
|---|---:|---|
| `bootloader.bin` | `0x1000` | `34f86cfd6d33dae0a96615f8e956c5ca2c0298ca7f558d2e2cd3c4dfc45ab0d3` |
| `partition-table.bin` | `0x8000` | `7f00b6c042a89b15b0cac534f82ed988caf29278ff5700b0c511eb1b5bb7c820` |
| `ESPDrone.bin` | `0x10000` | `cba7ff76f4a94e363446d2321a8be58056405454a64d18e6649d56d0c3a54e12` |

应用镜像大小为 `0xe95b0` 字节，1 MiB app 分区剩余 `0x16a50` 字节（9%）。三段均由 esptool 完成写后 hash 校验，未擦除 NVS 分区。

## 2. 启动与通信

通过项：

- 固件报告版本 `acc3d61` 和预期 IDF revision。
- SoftAP 启动为 `ESP-DRONE_7CDFA1021008`，native Wi-Fi transport 初始化成功。
- UDP socket 成功绑定端口 2390，RX/TX/WIFILINK task 均运行。
- `espressif/esp-now 2.5.3` 完成初始化。
- CRTP transport、system、worker、comm、commander 自检路径均通过。
- `configblockTest = 1`；在保留 NVS 的再次硬复位后仍通过，覆盖 NVS-backed config block 的复位读取路径。

未覆盖项：

- 验证主机只有 `enp1s0` 有线接口，没有 802.11 接口，无法加入设备 SoftAP。因此本轮没有把 UDP 空包、短包、超长包和合法帧从外部空口注入；只确认服务端创建及任务存活，不能把它记录为端到端 UDP 数据面通过。
- 没有连接示波器或逻辑分析仪；电机验证限于默认 `CONFIG_MOTORS_STARTUP_TEST` 关闭、初始化代码写入 0 duty、系统未通过飞行自检且未执行 `systemStart()`。未执行带桨或主动旋转测试。

## 3. 关键传感器失效验证

I2C0 初始化为 SDA 11、SCL 10、400 kHz，但 MPU6050 事务超时。当前连接平台不能确认包含可用 MPU6050，因此飞行传感器、姿态估计和电机闭环不具备硬件验收条件。

首轮固件在该条件下执行断言并约每 2.5 秒复位。提交 `acc3d61` 后的行为为：

- 报告 `Critical IMU unavailable; stabilization remains disabled.`；
- 传感器中断和采样任务不启动；
- `stabilizerTest = 0`，系统不发布飞行启动信号；
- 即使之后由远程参数强制发布启动信号，稳定器任务也会调用 `powerStop()` 并停在诊断循环。

修复固件连续观察 130 秒：启动 1 次，MPU 失败 1 次，panic 0 次，assert 0 次，额外复位 0 次。随后再次硬复位，NVS config block 自检仍通过。

## 4. 结论

本轮 S2 硬件验证通过以下验收：IDF 6.1 镜像可刷写和校验、Wi-Fi/ESP-NOW/UDP 服务可启动、NVS config block 可跨复位读取、关键 IMU 缺失时不会形成复位风暴且飞行输出保持受抑制。

完整飞行硬件验收仍阻塞于 MPU6050 不在线；UDP 数据面端到端验收仍需要一块带 Wi-Fi 的测试主机。两项均作为硬件环境限制保留，不能用本次结果替代。

## 5. IDF 支持矩阵变更回归（2026-08-11）

验证提交：`5d664f8`（包含仅支持 IDF 5.5/6.1 的构建约束和文档修订）

配置与镜像：

- ESP-IDF：`release/v6.1`，`6a9c44fe7e725af45cb99293ae38afd7d481f1e3`。
- `CONFIG_LWIP_IPV6` 未启用；现有 IPv4 SoftAP/UDP transport 保持不变。
- `CONFIG_MOTORS_STARTUP_TEST` 未启用。
- `ESPDrone.bin` 大小：`0xe39a0` 字节，1 MiB app 分区剩余 `0x1c660` 字节（11%）。
- `ESPDrone.bin` SHA-256：`86df9a3b74c4237b754dbc42d07db0738912ba98cb8fda647350e758504368b7`。
- bootloader 与 app 写入均通过 esptool hash 校验；partition table 内容未变化并跳过写入，NVS 未擦除。

串口回归结果：

- 固件报告 app version `5d664f8` 和预期 IDF revision。
- SoftAP `ESP-DRONE_7CDFA1021008`、DHCP、ESP-NOW 2.5.3、UDP/2390 和 native Wi-Fi CRTP transport 均成功启动。
- `transportTest`、`configblockTest`、`commTest` 和 `commanderTest` 均为 1；空闲时 free heap 为 64,828 字节。
- MPU6050 仍在 SDA 11/SCL 10/400 kHz 上超时；固件报告关键 IMU 不可用，`stabilizerTest = 0`，飞行输出保持受抑制。
- 约 45 秒串口观察内只有一次预期启动，无 panic、assert 或额外复位。

该回归确认关闭未使用的 IPv6 表没有破坏现有 IPv4 控制服务，也没有改变已验证的传感器失效安全行为。受硬件环境限制，端到端 UDP 注入、传感器采样、PWM 波形和实际飞行仍未覆盖。
