
## ESP-Drone_S2_V1.2 硬件组装过程

详细的硬件介绍可查阅：[hardware](./hardware.md)

![assembling](../../_static/assembling.png)

TODO: 添加组装视频链接

## 手机 APP 使用指引

1. 安装ESPDroe APP

    **Android APP 下载：**

    [https://github.com/qljz1993/DailyMD_public/raw/master/esplanes2_test/ESP-Drone-android-debug.apk](https://github.com/qljz1993/DailyMD_public/raw/master/esplanes2_test/ESP-Drone-android-debug.apk)

    **IOS APP 扫码下载：**

    ![ios_app](../../_static/ios_app_download.png)

2. WiFi 连接

    SSID：ESPDRONE_XXXX (XXXX根据 MAC 设置） PASSWORD：12345678

3. 个性化设置

    ```
    默认配置：

    Flight control settings 
        1. Mode: Mode2
        2. Deadzone: 0.2
        3. Roll trim: 0.0
        4. Pitch trim: 0.0
        5. Advanced flight control : true
        6. Advanced flight control preferences 
            1. max roll/pitch angle: 15
            2. max yaw angle: 90
            3. max thrust: 90
            4. min thrust: 25
            5. X-Mode: true
    Controller settings 
        1. use full travel for thrust: false
        2. virtual joystick size: 100
    App settings
        1. Screen rotation lock: true
        2. full screen mode:true
        3. show console: true   
    ```

4. 控制飞行
    1. 打开通信连接，小飞机绿灯将闪烁
    2. 轻推油门，起飞

    ![espdrone_app_android](../../_static/espdrone_app_android.png)

## PC cfclient 使用指引

![cfclient](../../_static/cfclient.png)

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

## 螺旋桨方向

* 注意按照下图所示位置，安装 A B 螺旋桨
* 注意检查飞机上电自检时，螺旋桨转向是否正确？

![esplane_1_0](../../_static/espdrone_s2_v1_2_diretion2.png)

## 其他注意事项

* 将飞机头部朝前放置，尾部天线朝向自己
* 注意将飞机置于水平面上，机身稳定时上电
* 注意观察上位机水平面是否置平？
* 注意观察通信建立以后，小飞机尾部绿灯是否快速闪速
* 注意观察小飞机头部红灯是否熄灭，亮起代表电量不足
* 轻推左手小油门，检查飞机是否能快速响应
* 轻推右手方向，检查方向控制是否正确
* 起飞吧！


## 开发指引

**请查阅**: [ESP-Drone Develop Guide](./2_developer_guide.md)