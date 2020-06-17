
## 已经支持的姿态计算算法

* 互补滤波
* 卡尔曼滤波

ESP-Drone 姿态计算代码来自 `crazyflie`，ESP-Drone 固件已经对互补滤波和卡尔曼滤波进行了实际测试，可以有效的计算飞行姿态，包括各个自由度的角度、角速度、和空间位置，为控制系统提供了可靠的状态输入。需要注意的是，在定点模式下，必须切换到卡尔曼滤波算法，才能保证工作正常。

crazyflie 状态估计：https://www.bitcraze.io/2020/01/state-estimation-to-be-or-not-to-be/

## 互补滤波

![Extended-Kalman-Filter](../../_static/Schematic-overview-of-inputs-and-outputs-of-the-Complementary-filter.png)

互补滤波中文说明可参考：https://zhuanlan.zhihu.com/p/34323865

## 卡尔曼滤波

![Extended-Kalman-Filter](../../_static/Schematic-overview-of-inputs-and-outputs-of-the-Extended-Kalman-Filter.png)

卡尔曼滤波中文说明可参考：https://zhuanlan.zhihu.com/p/39912633