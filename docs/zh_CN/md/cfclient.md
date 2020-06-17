
## CFclient 简介

CFclient 是 `crazeflie` 源工程的上位机，完全实现了 `CRTP` 协议中定义的功能，可以加快飞机的调试过程，ESP-Drone 项目对该上位机进行裁剪和调整，满足功能设计需求。

![Architecture](https://img-blog.csdnimg.cn/20191022115149326.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzIwNTE1NDYx,size_16,color_FFFFFF,t_70)

![cfclient控制台界面](../../_static/cfclient.png)

项目中有很多相关的文件，例如配置文件和缓存文件，其中JSON文件用来存储配置信息。关于配置信息中内容的解读：[User configuration file](https://www.bitcraze.io/docs/crazyflie-clients-python/master/dev_info_client/)

**基本飞行设置 Basic Flight Control**

1. 飞行模式 Flight mode selector \(Normal and Advanced\)
   * 基本模式 _Normal: 初学者使用
   * 高级模式 _Advanced: 设置解锁最大角度，设置最大油门。
2. 自动模式 Assisted mode selection. 
   * 定海拔模式 _Altitude hold_: 保持飞行海拔，需要气压计支持。
   * 定点模式 _Position hold_: 保持当前位置，需要光流和TOF支持。 
   * 定高模式 _Height hold_: 保持相对高度, 触发时保持高于地面 40cm ，需要TOF支持。
   * 悬停模式 _Hover : 触发时保持高于地面 40cm，并悬停在起飞点，需要光流和TOF支持。
3. 角度修正 trim
	* Roll Trim : 翻滚角修正，用于弥补传感器水平安装误差
	* Pitch Trim : 俯仰角修正，用于弥补传感器水平安装误差

> 在自动模式下，油门摇杆变为高度控制摇杆

**高级飞行设置 Advanced Flight Control**

1.  最大倾角 _Max angle: 设置最大允许的俯仰和翻滚角度 roll/pitch
2. 最大自选速度 _Max yaw rate: 设置允许的偏航速度  yaw 
3. 最大油门 _Max thrust: 设置最大油门
4. 最小油门 _Min thrust: 设置最小油门
5. 回转极限 _Slew limit:  回转极限。防止油门骤降，油门低于该值时，将被平滑的接管。 Set the percentage where the thrust is slew controlled \(the thrust value lowering will be limited\). This makes the Crazyflie a bit easier to fly for beginners
6. 回转率 _Slew rate: 油门到回转极限时，最大的油门值。When the thrust is below the slew limit, this is the maximum rate of lowering the thrust

**遥控器设置 Configure input device**

按照提示绑定遥控器摇杆与各个控制通道：

![按照提示绑定遥控器摇杆与各个控制通道](https://img-blog.csdnimg.cn/2019123116533679.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzIwNTE1NDYx,size_16,color_FFFFFF,t_70)

**飞行数据 Fligt Data**

驾驶仪可以看到当前飞机姿态，右下方显示对应的详细数据。
1. 目标角度 Target
2. 测量角度 Actual
3. 当前油门值 Thrust
4. 电机实际输出 M1\M2\M3\M4

## 在线参数修改 parameter

**注意事项**

1. 修改的参数实时生效，避免了频繁烧录固件。
2. 可在代码中通过宏定义配置那些参数可被上位机实时修改。
3. 注意参数在线修改仅用于调试，掉电不保存。

**在线调整 PID 参数**

![在这里插入图片描述](https://img-blog.csdnimg.cn/2019123116253174.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzIwNTE1NDYx,size_16,color_FFFFFF,t_70)

## 飞行数据监控 logging

**配置要监控的参数**

![logging](https://img-blog.csdnimg.cn/20191231162435849.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzIwNTE1NDYx,size_16,color_FFFFFF,t_70)

![在这里插入图片描述](https://img-blog.csdnimg.cn/201912311624470.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzIwNTE1NDYx,size_16,color_FFFFFF,t_70)

**实时波形绘制**

陀螺仪加速度计实时数据监测：

![陀螺仪加速度计实时数据](https://img-blog.csdnimg.cn/20191231160734754.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzIwNTE1NDYx,size_16,color_FFFFFF,t_70)
