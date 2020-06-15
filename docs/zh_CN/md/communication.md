##  ESP32(-S2) Wi-Fi 性能

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

 **ESP32-S2 Wi-Fi 性能（需要验证和补充）**

| 项目 | 参数 |
|--|--|
| Mode | Station, AP, Coexistence |
| Protocol | IEEE-802.11B, IEEE-802.11G, IEEE802.11N 支持软件切换 |
| Safety |WPA/WPA2/WPA2-Enterprise and WPS |
| Keyfeature |AMPDU, HT40, QoS |
| Distance |1 km with  Espressif-specific protocol|
| Speed |20 MBit/sec TCP throughput, 30 MBit/sec UDP |

其他参数：[https://docs.espressif.com/projects/esp-idf/en/latest/esp32s2/api-guides/wifi.html#esp32-s2-wi-fi-feature-list](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s2/api-guides/wifi.html#esp32-s2-wi-fi-feature-list)

## Wi-Fi 编程框架

![Wi-Fi Programming Model](https://img-blog.csdnimg.cn/20200423173923300.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzIwNTE1NDYx,size_16,color_FFFFFF,t_70#pic_center)

1.  应用层调用[ Wi-Fi driver APIs ](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s2/api-reference/network/esp_wifi.html)进行 Wi-Fi 初始化。
2. Wi-Fi 可被看成一个独立工作的黑箱，当事件发生时向默认事件循环[ default event loop ](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s2/api-reference/system/esp_event.html#esp-event-default-loops)
发布`event`，应用程序可根据需求编写`handle`程序，进行注册。
3. 网络接口组件 [ esp_netif ](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s2/api-reference/network/esp_netif.html)提供了一系列`handle`程序，与WiFi驱动`event`默认关联，例如 ESP32 作为 AP，当有用户接入时，esp_netif 将自动启动 DHCP 服务。
 
 > 注意：Wi-Fi 初始化之前应该使用`WIFI_INIT_CONFIG_DEFAULT`获取初始化配置结构体，对该结构体进行个性化配置，然后进行初始化工作。防止结构体成员未初始化导致的问题，特别是 IDF 更新添加了新的结构体成员的时候。

### 2.1 一般使用过程

**Sample Wi-Fi Event Scenarios in Station Mode**

![Sample Wi-Fi Event Scenarios in Station Mode](https://img-blog.csdnimg.cn/20200426225120463.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzIwNTE1NDYx,size_16,color_FFFFFF,t_70#pic_center)

**Sample Wi-Fi Event Scenarios in AP Mode**

![Sample Wi-Fi Event Scenarios in AP Mode](https://img-blog.csdnimg.cn/2020042622523887.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzIwNTE1NDYx,size_16,color_FFFFFF,t_70#pic_center)

