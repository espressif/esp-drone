
# Introduction

## ESP-Drone

**Drones powered by ESP32-S2&ESP\_IDF&Crazyflie**

### 一、简介

ESP-Drone 是基于乐鑫 `ESP32-S2 / ESP32` 开发的小型无人机解决方案，可使用手机 APP 或游戏手柄通过 Wi-Fi 网络进行连接和控制，目前已支持自稳定飞行、定高飞行、定点飞行等多种模式。该方案硬件结构简单，代码架构清晰完善,方便功能扩展，可用于STEAM教育等领域。控制系统代码来自 Crazyflie 开源工程，使用GPL3.0开源协议。
项目wiki：[https://qljz1993.gitbook.io/esplane/](https://qljz1993.gitbook.io/esplane/)

![ESP-Drone](https://img-blog.csdnimg.cn/20191030202043361.jpg?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzIwNTE1NDYx,size_16,color_FFFFFF,t_70)

![A swarm of drones exploring the environment, avoiding obstacles and each other. \(Guus Schoonewille, TU Delft\)](https://img-blog.csdnimg.cn/20191030202634944.jpg?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzIwNTE1NDYx,size_16,color_FFFFFF,t_70)

### 二、已实现功能

1. 自稳定模式
2. 定高模式（目前只支持手柄，APP不支持）
3. 定点模式（目前只支持手柄，APP不支持）
4. 对接cfclient上位机 
5. espilot APP控制


### 三、配置表

#### 传感器

| Sensor  | Interface | Comment |
|--|--|--|
| MPU6050 | I2C0 | must |
| VL53L1X | I2C0 | altitude hold  |
| ~~HMC5883L~~  | AUX_I2C | MPU6050 slave |
| ~~MS5611~~  | AUX_I2C | MPU6050 slave |
|PMW3901|	HSPI | | 

#### 指示灯

| State | LED | Action |
|--|--|--|
|SENSORS READY|BLUE|SOLID|
|SYSTEM READY|BLUE|SOLID|
|UDP_RX|GREEN|BLINK|


#### 引脚配置

| 引脚 | 功能 | 备注 |
| :---: | :---: | :---: |
| GPIO11 | I2C0_SDA | MPU6050 专用|
| GPIO10 | I2C0_SCL | MPU6050 专用|
| GPIO37 | SPI_MISO | MISO |
| GPIO35 | SPI_MOSI |MOSI |
| GPIO36 | SPI_CLK|SCLK|
| GPIO34 | SPI_CS0|CS0* |
| GPIO40 | I2C1_SDA|VL53L1X|
| GPIO41 | I2C1_SCL |VL53L1X|
| GPIO12 | interrupt | MPU6050 interrupt |
| GPIO39 |  BUZ_1|BUZZ+ |
| GPIO38 |BUZ_2| BUZZ- | 
| GPIO8 | LED\_RED | LED\_1 |
| GPIO9 | LED\_GREEN | LED\_2 |
| GPIO7 | LED\_BLUE | LED\_3 |
| GPIO5 | MOT\_1 | |
| GPIO6 | MOT\_2 | |
| GPIO3 | MOT\_3 | |
| GPIO4 | MOT\_4 | |
| GPIO2 | ADC\_7\_BAT | VBAT/2 |
| GPIO1 |EXT_IO1  |  |

#### 扩展接口

| 左引脚 | IO |功能 | 右引脚 | IO |功能|
| :---: | :---: | :---: | :---: | :---: | :---:|
|SPI_CS0  | GPIO34 |功能 | VDD_33 | IO |功能|
|SPI_MOSI |GPIO35 | |I2C0_SDA |GPIO11| |
| SPI_CLK| GPIO36| | I2C0_SCL|GPIO10 | |
| SPI_MISO|GPIO37 | | GND| | |
| GND| | | AUX_SCL| | |
| I2C1_SDA| GPIO40| | AUX_SDA| | |
| I2C1_SCL|GPIO41 | | BUZ_2|GPIO38 | |
| EXT_IO1| GPIO1| | BUZ_1|GPIO39 | |

#### 摄像头接口

| 引脚 | 功能 | 备注 |
| :---: | :---: | :---: |
|GPIO13  |  CAM_VSYNC|  |
|GPIO14  |  CAM_HREF|  |
|GPIO15  |  CAM_Y9|  |
|GPIO16  |  CAM_XCLK|  |
|GPIO17  |CAM_Y8  |  |
|GPIO18  |CAM_RESET  |  
|GPIO19  |CAM_Y7  |  |
|GPIO20  |  CAM_PCLK|  |
|GPIO21  |  CAM_Y6|  ||
|GPIO33  |CAM_Y2  |  ||
|GPIO45 |  CAM_Y4| 
|GPIO46  |CAM_Y3  | 

### 四、 ESP-IDF 组件修改

打开 ESP32(S2) 的链接脚本模板`${IDF_PATH}/components/esp32/ld/esp32.project.ld.in` 或者` ${IDF_PATH}/components/esp32s2/ld/esp32s2.project.ld.in`, 将以下代码添加到 `.flash.rodata` 段的末尾.

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

### 五、第三方代码

Additional third party copyrighted code is included under the following licenses.

| Component | License | Origin |commit id |
| :---:  | :---: | :---: |:---: |
| core/crazyflie | GPL-3.0 |[Crazyflie](https://github.com/bitcraze/crazyflie-firmware) |a2a26abd53a5f328374877bfbcb7b25ed38d8111|
| lib/dsp_lib |  | [esp32-lin](https://github.com/whyengineer/esp32-lin/tree/master/components/dsp_lib) |6fa39f4cd5f7782b3a2a052767f0fb06be2378ff|


### 六、感谢/THANKS

1. 感谢Bitcraze开源组织提供很棒的[Crazyflie](https://www.bitcraze.io/%20)无人机项目代码
2. 感谢Espressif提供ESP32和[ESP-IDF操作系统](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s2/get-started/index.html)
3. 感谢WhyEngineer提供的stm32 dsp移植库[esp-dsp](https://github.com/whyengineer/esp32-lin/tree/master/components/dsp_lib)

