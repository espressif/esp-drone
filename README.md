# Introduction

## ESPlane2.0

**Drones powered by ESP32&ESP-IDF&Crazyflie**

### Introduction

**ESPlane2-S2** is an open source **drone solution** based on espressif **ESP32-S2** Wi-Fi chip, which can be controlled through **Wi-Fi** network using mobile APP or gamepad. ESPlane2-S2 supports multiple fly modes, `stabilize`, `height-hold`, `position-hold` and more. ESPlane2-S2 solution has **simple hardware structure**,**clear and extendible code architecture**, can be used in **STEAM education** and other fields. The main code ported from **Crazyflie** open source project, using the **GPL3.0** open source protocol.

**For User**: [01-ESPlane2.0 Operate Guide](esplane2.0-kai-fa-bi-ji/00esplane-shang-wei-ji-an-zhuang-zhi-yin.md)

**For Developer**: [01-ESPlane2.0 Develop Guide](esplane2.0-kai-fa-bi-ji/00esplane-kai-fa-zhi-yin.md)

![ESPlane](https://img-blog.csdnimg.cn/20191030202043361.jpg?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzIwNTE1NDYx,size_16,color_FFFFFF,t_70)

![A swarm of drones exploring the environment, avoiding obstacles and each other. \(Guus Schoonewille, TU Delft\)](https://img-blog.csdnimg.cn/20191030202634944.jpg?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzIwNTE1NDYx,size_16,color_FFFFFF,t_70)

### Implemented Features

1. Stabilize mode
2. Height-hold mode (through cfcilent)
3. position-hold mode (through cfcilent)
4. cfclient supported
5. ESPilot supported

### Configuration

#### Sensor

| Sensor  | Interface | Comment |
|--|--|--|
| MPU6050 | I2C0 | must |
| VL53L1X | I2C0 | altitude hold  |
| ~~HMC5883L~~  | AUX_I2C | MPU6050 slave |
| ~~MS5611~~  | AUX_I2C | MPU6050 slave |
|PMW3901|	HSPI | | 

#### LED


```
#define LINK_LED         LED_BLUE
//#define CHG_LED          LED_RED
#define LOWBAT_LED       LED_RED
//#define LINK_DOWN_LED  LED_BLUE
#define SYS_LED          LED_GREEN 
#define ERR_LED1         LED_RED
#define ERR_LED2         LED_RED
```

| State | LED | Action |
|--|--|--|
|SENSORS READY|BLUE|SOLID|
|SYSTEM READY|BLUE|SOLID|
|UDP_RX|GREEN|BLINK|


#### PIN

| Pin | Function | Remarks |
| :---: | :---: | :---: |
| GPIO11 | I2C0_SDA | MPU6050 dedicated|
| GPIO10 | I2C0_SCL | MPU6050 dedicated|
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
| GPIO3 | MOT\_1 |  |
| GPIO4 | MOT\_2 |  |
| GPIO5 | MOT\_3 |  |
| GPIO6 | MOT\_4 |  |
| GPIO2 | ADC\_7\_BAT | VBAT/2 |
| GPIO1 |EXT_IO1  |  |

#### Extend PIN

| Left | IO |Function | Right | IO |Function|
| :---: | :---: | :---: | :---: | :---: | :---:|
|SPI_CS0  | GPIO34 | | VDD_33 | IO ||
|SPI_MOSI |GPIO35 | |I2C0_SDA |GPIO11| |
| SPI_CLK| GPIO36| | I2C0_SCL|GPIO10 | |
| SPI_MISO|GPIO37 | | GND| | |
| GND| | | AUX_SCL| | |
| I2C1_SDA| GPIO40| | AUX_SDA| | |
| I2C1_SCL|GPIO41 | | BUZ_2|GPIO38 | |
| EXT_IO1| GPIO1| | BUZ_1|GPIO39 | |

#### Camera

| IO | Function | Others |
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

####  ESP-IDF Version

|ESPlane|CommitID| ESP-IDF|CommitID|
| :---: | :---: | :---: | :---: |
|master||release/v3.3 update20200306|6f9a7264ce20c6132fbd8309112630d0eb490fe4|
|Esplane-S2||master update20200404|d85d3d969ff4b42e2616fd40973d637ff337fae6|


### THANKS

1. Thanks to the Bitcraze for the great [Crazyflie project](https://www.bitcraze.io/%20)
2. Thanks to Espressif for the powerful [ESP-IDF environment](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s2/get-started/index.html)
3. Thanks to WhyEngineer for the useful [ESP-DSP lib](https://github.com/whyengineer/esp32-lin/tree/master/components/dsp_lib)

