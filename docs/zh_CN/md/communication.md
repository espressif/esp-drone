
# 通信协议

## 通信层级结构

||||
|:--:|:--:|:--:|
|终端|手机/PC|ESP-Drone|
|应用层|APP | 飞控固件|
|协议层|CRTP |CRTP |
|传输层|UDP|UDP|
|物理层|Wi-Fi STA|Wi-Fi AP |

## Wi-Fi 通信

### ESP32(S2) Wi-Fi 性能

 **ESP32 Wi-Fi 性能**

| 项目 | 参数 |
|--|--|
| Mode | Station, AP, Coexistence |
| Protocol | IEEE-802.11B, IEEE-802.11G, IEEE802.11N ，802.11 LR(乐鑫) 支持软件切换 |
| Safety |WPA/WPA2/WPA2-Enterprise and WPS |
| Keyfeature |AMPDU, HT40, QoS |
| Distance |1 km with  Espressif-specific protocol|
| Speed |20 MBit/sec TCP throughput, 30 MBit/sec UDP |

其它参数：[https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/wifi.html#esp32-wi-fi-feature-list](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/wifi.html#esp32-wi-fi-feature-list)

 **ESP32-S2 Wi-Fi 性能**

TODO: ESP32-S2 Wi-Fi 参数需要验证和补充

| 项目 | 参数 |
|--|--|
| Mode | Station, AP, Coexistence |
| Protocol | IEEE-802.11B, IEEE-802.11G, IEEE802.11N 支持软件切换 |
| Safety |WPA/WPA2/WPA2-Enterprise and WPS |
| Keyfeature |AMPDU, HT40, QoS |
| Distance |1 km with  Espressif-specific protocol|
| Speed |20 MBit/sec TCP throughput, 30 MBit/sec UDP |

其他参数：[https://docs.espressif.com/projects/esp-idf/en/latest/esp32s2/api-guides/wifi.html#esp32-s2-wi-fi-feature-list](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s2/api-guides/wifi.html#esp32-s2-wi-fi-feature-list)

### Wi-Fi 编程框架

**基于 ESP-IDF 的 Wi-Fi 编程框架:**

![Wi-Fi Programming Model](https://img-blog.csdnimg.cn/20200423173923300.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzIwNTE1NDYx,size_16,color_FFFFFF,t_70#pic_center)

**一般使用过程:**

1.  应用层调用[ Wi-Fi driver APIs ](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s2/api-reference/network/esp_wifi.html)进行 Wi-Fi 初始化。
2. Wi-Fi 可被看成一个独立工作的黑箱，当事件发生时向默认事件循环 [ default event loop ](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s2/api-reference/system/esp_event.html#esp-event-default-loops)
 发布 `event` ，应用程序可根据需求编写 `handle` 程序，进行注册。
3. 网络接口组件 [ esp_netif ](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s2/api-reference/network/esp_netif.html)提供了一系列 `handle` 程序，与 WiFi 驱动 `event` 默认关联，例如 ESP32 作为 AP，当有用户接入时，esp_netif 将自动启动 DHCP 服务。
 
具体的使用过程，可查阅代码 `\components\drivers\general\wifi\wifi_esp32.c`

 > 注意：Wi-Fi 初始化之前应该使用 `WIFI_INIT_CONFIG_DEFAULT` 获取初始化配置结构体，对该结构体进行个性化配置，然后进行初始化工作。防止结构体成员未初始化导致的问题，特别是 IDF 更新添加了新的结构体成员的时候。

**AP 模式工作状态图:**

![Sample Wi-Fi Event Scenarios in AP Mode](https://img-blog.csdnimg.cn/2020042622523887.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzIwNTE1NDYx,size_16,color_FFFFFF,t_70#pic_center)

### 提高 Wi-Fi 通信距离

依次进入：`Component config>>PHY>>Max WiFi TX power (dBm)`
将 `Max WiFi TX power` 改为 `20`， 该项配置将提高 PHY 增益，提高 Wi-Fi 通信距离。

## UDP 通信

### UDP 端口号

| App |Direction | ESP-Drone |
|--|--|--|--|
| 192.168.43.42::2399 | TX / RX | 192.168.43.42::2390 | 

### UDP 包结构

```text
/* Frame format:
 * +=============+-----+-----+
 * | CRTP                      |   CKSUM   |
 * +=============+-----+-----+
 */
 UDP传输的数据包为： CRTP +校验信息。
CRTP：按照 CRTP 包结构定义，包含 Header+Data，具体参考 CRTP 协议部分
CKSUM：为校验信息，大小为 1 byte ，将 CRTP 包按照 byte 累加即可。
```

**CKSUM 计算方法：**

```python
#python为例：计算 raw 的 cksum，并将其添加到包尾
 raw = (pk.header,) + pk.datat
 cksum = 0
 for i in raw:
		cksum += i
 cksum %= 256
 raw = raw + (cksum,)
```

## CRTP 协议

ESP-Drone 项目继承 Crazyflie 项目使用的 CRTP \(Crazy RealTime Protocol\) 协议，用于飞行指令发送、飞行数据回传、参数设置等。

> CRTP is designed to be state-less, so there's no handshaking procedure that is needed. Any command can be sent at any time, but for some logging/param/mem commands the TOC \(table of contents\) needs to be downloaded in order for the host to be able to send the correct information. The implementation of the Pyton API will download the param/log/mem TOC at connect in order to be able to use all the functionality.

### CRTP 包结构

> Each CRTP packet is 32 bytes, of which 1 byte is taken by the header. This gives a total payload of 31 bytes per packet. The header holds the port \(8 bits\), channel \(2 bits\) and reserved \(2 bits\).

```text
  7   6   5   4   3   2   1   0
+---+---+---+---+---+---+---+---+
|     Port      |  Res. | Chan. | 
+---+---+---+---+---+---+---+---+
|            DATA 0             |
+---+---+---+---+---+---+---+---+
:   :   :   :   :   :   :   :   :
+---+---+---+---+---+---+---+---+
|            DATA 30            |
+---+---+---+---+---+---+---+---+
```

| Field | Byte | Bit | Description |
| :---: | :---: | :---: | :---: |
| Header | 0 | 0-1 | The destination channel |
|  | 0 | 2-3 | Reserved for transport layer |
|  | 0 | 4-7 | The destination port |
| Data | 1-31 | The data in the packet |  |

### 端口分配

| Port | Target | Used for |
| :---: | :---: | :---: |
| 0 | Console | Read console text that is printed to the console on the Crazyflie using consoleprintf |
| 2 | Parameters | Get/set parameters from the Crazyflie. Parameters are defined using a macro in the Crazyflie source-code |
| 3 | Commander | Sending control set-points for the roll/pitch/yaw/thrust regulators |
| 4 | Memory access | Accessing non-volatile memories like 1-wire and I2C \(only supported for Crazyflie 2.0\) |
| 5 | Log | Set up log blocks with variables that will be sent back to the Crazyflie at a specified period. Log variables are defined using a macro in the Crazyflie source-code |
| 6 | Localization | Packets related to localization |
| 7 | Generic Setpoint | Allows to send setpoint and control modes |
| 13 | Platform | Used for misc platform control, like debugging and power off |
| 14 | Client-side debugging | Debugging the UI and exists only in the Crazyflie Python API and not in the Crazyflie itself. |
| 15 | Link layer | Used to control and query the communication link |

> `In the firmware most modules (that are connected to ports) are implemented as tasks. 
>  Each task is blocking on a message delivery queue where incoming CRTP packets are delivered. 
>  At start up each of these tasks (and other modules) register a callback with the communication layer for their pre-defined ports.`

* [各个端口使用详情可参考：crtp](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/functional-areas/crtp/)

### CRTP 协议支持包

* **cflib 是 CRTP 协议的 python 支持包，提供了通信协议的应用层接口，可以用于构建上位机。** 
* 对于固件中每一个使用 CRTP 协议的组件，在 cflib 中都有一个脚本与其对应。

> cflib is an API written in Python that is used to communicate with the Crazyflie and Crazyflie 2.0 quadcopters. It is intended to be used by client software to communicate with and control a Crazyflie quadcopter. For instance the cfclient Crazyflie PC client uses the cflib.

* 源工程仓库地址：[github: crazyflie-lib-python](https://github.com/bitcraze/crazyflie-lib-python)

* 适配 ESP-Drone 的 cflib 工程仓库地址：[github: qljz1993/crazyflie-lib-python](https://github.com/qljz1993/crazyflie-lib-python.git)。需要切换到 `esplane` 分支。

## 基于 CRTP 协议的应用开发

### 各个平台工程模板

1. [crazyflie2-ios-client](https://github.com/bitcraze/crazyflie2-ios-client)

2. [crazyflie2-windows-uap-client](https://github.com/bitcraze/crazyflie2-windows-uap-client)

3. [crazyflie-android-client](https://github.com/bitcraze/crazyflie-android-client)

	[安卓版本使用指南](https://wiki.bitcraze.io/doc:crazyflie:client:cfandroid:index)

	[安卓版本开发指南](https://wiki.bitcraze.io/doc:crazyflie:dev:env:android)

### CFclient

CFclient 是 `crazeflie` 源工程的上位机，完全实现了 `CRTP` 协议中定义的功能，可以加快飞机的调试过程，ESP-Drone 项目对该上位机进行裁剪和调整，满足功能设计需求。

![cfclient控制台界面](../../_static/cfclient.png)

CFclient 具体使用说明可查阅：[CFclient](gettingstarted.html#pc-cfclient)