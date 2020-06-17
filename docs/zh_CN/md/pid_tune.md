
## PID参数整定流程

**crazyflie `Rate PID`调整过程**

1. 先调整`Rate `模式，将`rollType `,`pitchType` 和 `yawType`都调整为`RATE`
2. 将 `ATTITUDE`模式对应的 `roll`, `pitch` 和 `yaw`的`KP`,`KI`和`KD`调整为`0.0`，仅保留`Rate `相关的参数
3. 将`RATE`模式对应的 `roll`, `pitch` 和 `yaw` 的`KI`和`KD`调整为`0.0`，先调整比例控制`KP`
4. 烧写代码，使用cfclient的param功能开始在线进行`KP`的调整
5. 注意，使用cfclient修改后的参数，掉电是不保存的。
6. 注意安全，因为在PID调整期间会出现超调的情况
7. 先固定住飞行器，让其只能进行`pitch`轴的翻转。逐渐增加`pitch`对应的`KP`,直到飞机出现前后的震荡（超调） 
8. 当出现严重的震荡时，可以稍微降低`KP`（ Once you reach the point of instability, tune it down 5-10 points），然后即可确定`KP`参数
9. 同样的方法调整 `roll` 
10. 最后同样的方法调整`yaw` 
11. 下面调整 `KI`，该参数用于消除稳态误差，因为如果不引入该参数，只有比例调整的话，飞机受到重力等干扰会在0位置上线摆动。设置 `KI`的初始值为`KP`的50%。
12.  当`KI`增大到一定程度，也会导致飞机不稳定的晃动，但是`KI`造成的晃动频率会相比`KP`带来的震动，频率更小。然后以造成这个状态的 `KI`为基础确定 `KI`的值（This is your critical KI, and so tune down 5-10 points.）
13. 同样的方法调整 `roll` 和 `yaw`
14.  yaw axis, except KI is usually around 80%+ of KP.

****
以上完成了`Rate `模式参数的调整
****

**下面开始整定 `Attitude PID`**
 
14. 确保`Rate PID`调整已经完成。
15. 将`rollType `,`pitchType` 和 `yawType`都调整为`ANGLE`，意味着飞机已经进入attitude mode。
16. 改变 `roll`和`pitch`的`KI`和`KD`为`0.0`，将`Yaw` 的 `KP``KI``KD`都设置为`0.0` 。
17. 烧写代码，使用cfclient的param功能开始在线进行`KP`的调整。
18. 将`roll`和`pitch`的`KP `设置为`3.5`，寻找任何不稳定性，例如振荡。持续增加KP，直到达到极限。
19. 如果您发现`KP`导致不稳定，如果此时已经高于`4`，需要将`RATE`模式的 `KP`和`KI`稍微降低5-10点。这使您在调整姿势模式时更加“自由”
20. 要调整KI，请再次缓慢增加KI。不稳定性的状态是产生低频振荡。