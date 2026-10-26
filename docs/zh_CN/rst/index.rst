ESP-Drone
===========
:link_to_translation:`en:[English]`

ESP-Drone 是面向乐鑫 SoC 的开源无人机解决方案。通过实物验证的飞行硬件可使用手机 app 或游戏手柄经 Wi-Fi 控制，并支持自稳定、定高和定点等飞行模式。该方案硬件结构简单，固件便于扩展，可用于 STEAM 教育和原型开发。飞行控制核心来自 Crazyflie 开源工程，使用 GPL-3.0 许可证。

本文档包含 ESP-Drone 基本的开发和使用说明。

ESP-Drone 支持政策
--------------------

自 2022 年 12 月起，我们仅提供有限的支持，但是 Pull Request 仍然是被欢迎的！

支持的版本和目标
----------------

本项目仅支持 ESP-IDF ``release/v5.5`` 和 ``release/v6.1``。

.. list-table::
   :header-rows: 1

   * - ESP-IDF
     - 目标
     - 状态
   * - ``release/v5.5``
     - ESP32、ESP32-S2、ESP32-S3、ESP32-P4
     - CI 构建支持；P4 使用无无线安全诊断配置
   * - ``release/v6.1``
     - ESP32、ESP32-S2、ESP32-S3、ESP32-P4、ESP32-S31
     - CI 构建支持；S31 为 preview target，P4/S31 飞行硬件尚未实物验证

快速入门
----------

.. list-table::
   :widths: 1 1 1

   * - |快速入门|
     - |开发指引|
     - |硬件参考|
   * - :doc:`快速入门 <gettingstarted>`
     - :doc:`开发指引 <developerguide>`
     - :doc:`硬件参考 <hardware>`

.. |快速入门| image:: ../../_static/get-started.png

.. |开发指引| image:: ../../_static/api-reference.png

.. |硬件参考| image:: ../../_static/hw-reference.png

.. toctree::
   :hidden:

   快速入门 <gettingstarted>
   环境搭建 <getespidf>
   开发指引 <developerguide>
   硬件参考 <hardware>
   驱动程序 <drivers>
   飞控系统 <system>
   通信协议 <communication>
   声明 <notice>
