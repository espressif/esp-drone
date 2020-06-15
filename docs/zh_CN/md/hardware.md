### 配置表

主板原理图 ： [SCH_Mainboard_ESP32_S2_Drone_V1_2](../../../hardware/ESP32_S2_Drone_V1_2/SCH_Mainboard_ESP32_S2_Drone_V1_2.pdf)
主板 PCB ： [PCB_Mainboard_ESP32_S2_Drone_V1_2](../../../hardware/ESP32_S2_Drone_V1_2/PCB_Mainboard_ESP32_S2_Drone_V1_2.pdf)

**传感器**

| Sensor  | Interface | Comment |
|--|--|--|
| MPU6050 | I2C0 | must |
| VL53L1X | I2C0 | altitude hold  |
| HMC5883L  | AUX_I2C | MPU6050 slave |
| MS5611  | AUX_I2C | MPU6050 slave |
|PMW3901|	HSPI | | 

**指示灯**

| State | LED | Action |
|--|--|--|
|SENSORS READY|BLUE|SOLID|
|SYSTEM READY|BLUE|SOLID|
|UDP_RX|GREEN|BLINK|


**主板引脚分配**

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

**扩展接口**

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

**摄像头接口**

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