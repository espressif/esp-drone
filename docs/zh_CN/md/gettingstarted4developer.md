
## 开发指引

### ESP-IDF 开发环境搭建

请参照ESP-IDF 入门指南： [https://docs.espressif.com/projects/esp-idf/en/latest/esp32s2/get-started/index.html](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s2/get-started/index.html)，按照步骤设置 ESP-IDF。

注意事项：
* 请完成链接页面的所有步骤。
* 请按照上面链接中的步骤构建一个或多个示例应用程序。

**ESP32(S2) 链接脚本修改**

打开 ESP32(S2) 的链接脚本模板`${IDF_PATH}/components/esp32/ld/esp32.project.ld.in` 和 ` ${IDF_PATH}/components/esp32s2/ld/esp32s2.project.ld.in`, 将以下代码添加到 `.flash.rodata` 段的末尾.

```
   /* Parameters and log system datas */
    _param_start = .;
    KEEP(*(.param))
    KEEP(*(.param.*))
    _param_stop = .;
    . = ALIGN(4);
    _log_start = .;
    KEEP(*(.log))
    KEEP(*(.log.*))
    _log_stop = .;
    . = ALIGN(4);
```
以上代码可以实现，将具有 `.param.*` 或 `.log.*` 段属性的变量，放置在连续的存储区域，从而加快变量遍历速度。


### 获取项目源代码

测试版本代码，目前放在 gitlab 仓库，使用 git 工具获取

```
git clone -b feature/develop https://gitlab.espressif.cn:6688/esp-college/esp-plane.git
```

**项目软件主要由飞控内核、硬件驱动和依赖库组成**

* 飞控内核来自 crazyflie 开源工程，主要包括硬件抽象层和飞控程序
* 硬件驱动按照硬件接口进行了文件结构划分，包括 I2C 设备和 SPI 设备等
* 依赖库包括 ESP-IDF 提供的默认组件，以及来自第三方的 DSP 等

文件结构如下所示：

```
.
├── components                        | 项目组件目录
│   ├── config                              | 系统 task 配置
│   │   └── include
│   ├── core                                 | 系统内核目录
│   │   └── crazyflie                  | crazyflie 内核
│   │       ├── hal                         | 硬件抽象代码 
│   │       └── modules             |  飞行控制代码 
│   ├── drivers                            | 硬件驱动目录
│   │   ├── deck                         | 硬件扩展接口驱动
│   │   ├── general                    | 一般设备目录
│   │   │   ├── adc                     | ADC 驱动，用于电压监测
│   │   │   ├── buzzer              | 蜂鸣器驱动，用于状态反馈
│   │   │   ├── led                     | LED驱动，用于状态反馈
│   │   │   ├── motors             | 电机驱动，用于推力输出
│   │   │   └── wifi                    | Wi-Fi 驱动，用于通信
│   │   ├── i2c_bus                   | I2C 驱动
│   │   ├── i2c_devices           | I2C 设备目录
│   │   │   ├── eeprom           | eeprom驱动，用于参数存储
│   │   │   ├── hmc5883l         | hmc5883l 磁罗盘传感器
│   │   │   ├── mpu6050          | mpu6050 陀螺仪加速度计传感器
│   │   │   ├── ms5611             | ms5611 气压传感器
│   │   │   ├── vl53l0                 | Ivl53l0 2m 激光传感器
│   │   │   └── vl53l1                 |  Ivl53l1 4m 激光传感器
│   │   └── spi_devices           | SPI 设备目录
│   │       └── pmw3901           | pmw3901 光流传感器
│   ├── lib                                      | 外部库目录
│   │   └── dsp_lib                    | dsp 库
│   ├── platform                         | 用于支持多平台
│   └── utils                                  | 工具函数目录
├── CMakeLists.txt                    | 工具函数
├── LICENSE                                | 开源协议
├── main                                       | 入口函数
├── README.md                        | 项目说明
└── sdkconfig.defaults            | 默认参数
```

### 硬件结构

硬件原理图可查阅：[hardware](./hardware.md)