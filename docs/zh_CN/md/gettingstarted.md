
## 项目预览

ESP-Drone 是基于乐鑫 ESP32/ESP32-S2 开发的小型无人机解决方案，可使用手机 APP 或游戏手柄通过 Wi-Fi 网络进行连接和控制，目前已支持自稳定飞行、定高飞行、定点飞行等多种模式。该方案硬件结构简单，代码架构清晰，支持功能扩展，可用于 STEAM 教育等领域。项目部分代码来自 Crazyflie 开源工程，继承 GPL3.0 开源协议。

TODO:添加代表性图片

### 功能介绍

* 支持自稳定模式-stabilize：自动控制机身水平，保持平稳飞行
* 支持定高模式-heighthold：自动控制油门输出，保持固定高度
* 支持定点模式-positionhold：自动控制机身角度，保持固定空间位置
* 支持 PC 上位机调试：使用 cfclient 上位机进行静态/动态调试
* 支持 APP 控制：使用手机 APP 通过 Wi-Fi 轻松控制
* 支持游戏手柄控制（gamepad）：通过 cfclient 使用游戏手柄轻松控制

### 硬件组成结构

**ESP-Drone 2.0 使用模块化的设计思路，由主控板和扩展板组成：**

* **主控制板**：搭载支持基础飞行的必要元器件，提供硬件扩展接口
* **扩展板**：基于硬件扩展接口，用于支持高级飞行功能或其他 DIY 功能

|序号| 模块名 | 主要元器件 | 功能 | 接口 |安装位置 |
|--|--|--|--|--|--|
|1| **主控制板-ESP32-S2** |  ESP32-S2-WROVER + MPU6050| 基础飞行 |提供 I2C SPI GPIO 扩展接口  ||
|2|扩展板- **定点模块** |  PMW3901 + VL53L1X | 室内定点飞行 | SPI + I2C | 底部，面向地面 |
|3| 扩展板-**气压定高模块** |  MS5611 气压 | 气压定高 | I2C 或 MPU6050从机|顶部或底部 |
|4| 扩展板-指南针模块 |  HMC5883 罗盘 | 无头模式等高级模式 | I2C 或 MPU6050从机|顶部或底部 |

硬件原理图可查阅：[hardware](./hardware.md)


### ESP-IDF 简介

ESP-IDF 是乐鑫为 ESP32/ESP32-S2 提供的物联网开发框架。

* ESP-IDF 包含一系列库及头文件，提供了基于 ESP32/ESP32-S2 构建软件项目所需的核心组件。
* ESP-IDF 还提供了开发和量产过程中最常用的工具及功能，例如：构建、烧录、调试和测量等。

### Crazyflie 简介

Crazyflie 是来自 bitcraze 开源工程的四旋翼飞行器。

* Crazyflie 支持多种传感器组合，可以轻松实现定高模式、定点模式等高级飞行模式。
* Crazyflie 基于 FreeRTOS 编写，将复杂的无人机系统，分解成多个具有不同优先级的软件任务。
* Crazyflie 设计了功能完备的 cfclient 上位机和 CRTP 通信协议，便于实现调试、测量和控制。

![A swarm of drones exploring the environment, avoiding obstacles and each other. \(Guus Schoonewille, TU Delft\)](https://img-blog.csdnimg.cn/20191030202634944.jpg?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzIwNTE1NDYx,size_16,color_FFFFFF,t_70)

## 使用指引

### 手机 APP 控制

1. 安装ESPDroe APP
2. 个性化设置
3. 控制飞行

### PC cfclient 控制

**1.安装crtp协议支持包**

1.1 下载源代码 

```text
git clone -b dev_esplane  https://github.com/qljz1993/crazyflie-lib-python.git
```

1.2 进入源码目录，安装依赖 

```text
pip3 install -r requirements.txt
```

1.3 安装crtp包 

```text
pip3 install -e .
```

**2.安装cfclient**

2.1 下载源代码

```text
git clone -b dev_esplane https://github.com/qljz1993/crazyflie-clients-python.git
```

2.2  进入源码目录，安装依赖 

```text
sudo apt-get install python3 python3-pip python3-pyqt5 python3-pyqt5.qtsvg
```

2.3  安装cfclient客户端

```text
pip3 install -e .
```

2.4 启动客户端


```text
python3 ./bin/cfclient
```

**3.配置遥控器**

![gamepad_settings](../../_static/gamepad_settings.png)

3.1 配置4个控制轴 `Roll 、Pitch、Yaw、Thrust`

3.2 配置一个按键为`Assisted control` ，用于飞行模式切换

## 注意事项

* 注意检查电机自检时转向是否正确，见下图
* 注意观察上位机水平面是否置平
* 轻轻推小油门试一下
* 起飞吧

![crazyfile](../../_static/motors_direction.png)

![esplane_1_0](../../_static/esplane_1_0.jpg)

## 开发指引

**请查阅**: [ESP-Drone Develop Guide](./docs/zh_CN/md/gettingstarted4developer.md)