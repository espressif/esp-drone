## 五、I2C相关知识

* 数据传输：SCL为高电平时，SDA线若保持稳定，那么SDA上是在传输数据bit；若SDA发生跳变，则用来表示一个会话的开始或结束。
* 数据改变：SCL为低电平时，SDA线才能改变传输的bit。
* 处理繁忙：如果从机处于繁忙状态，并且现在不方便接受下一个字节，可以保持SCL为低电平，从而强制主机进入等待状态，等准备号后释放时钟线。
* 处理应答：应答时钟由主机控制，应答信号-在第9个时钟周期，SCL为高，SDA为低。拒绝应答-在第9个时钟周期，SDA一直为高。

![I2C&#x4FE1;&#x53F7;](https://img-blog.csdnimg.cn/20191031211402937.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzIwNTE1NDYx,size_16,color_FFFFFF,t_70)

![&#x5355;&#x5B57;&#x8282;&#x5199;](https://img-blog.csdnimg.cn/2019103121150031.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzIwNTE1NDYx,size_16,color_FFFFFF,t_70)

![&#x53CC;&#x5B57;&#x8282;&#x5199;](https://img-blog.csdnimg.cn/20191031211510492.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzIwNTE1NDYx,size_16,color_FFFFFF,t_70)

![&#x5355;&#x5B57;&#x8282;&#x8BFB;](https://img-blog.csdnimg.cn/20191031211519173.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzIwNTE1NDYx,size_16,color_FFFFFF,t_70)

![&#x53CC;&#x5B57;&#x8282;&#x8BFB;](https://img-blog.csdnimg.cn/20191031211528953.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzIwNTE1NDYx,size_16,color_FFFFFF,t_70)