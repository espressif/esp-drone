
ESP-Drone
===========
:link_to_translation:`zh_CN:[中文]`

**ESP-Drone** is an open source **drone solution** based on Espressif **ESP32/ESP32-S2/ESP32-S3** Wi-Fi chip, which can be controlled over a **Wi-Fi** network using a mobile APP or gamepad. ESP-Drone supports multiple flight modes, including `Stabilize mode`, `Height-hold mode`, and `Position-hold mode`. With **simple hardware**, **clear and extensible code architecture**, ESP-Drone can be used in **STEAM education** and other fields. The main code is ported from **Crazyflie** open source project with **GPL3.0** protocol.

This document describes the development guide for ESP-Drone project.

ESP-Drone Support Policy
-------------------------

From December 2022, we will offer limited support on this project, but Pull Request is still welcomed!

Supported versions and chips
-----------------------------

+-----------+-----------------------------+---------------------+-----------------+
| ESP-Drone |      Dependent ESP-IDF      |       Target        |  Support State  |
+===========+=============================+=====================+=================+
| master    | release/v4.4 , release/v5.0 | ESP32-S2 / ESP32-S3 | Limited Support |
+-----------+-----------------------------+---------------------+-----------------+

Get Started
------------

==================  ==================  ==================
|Get Started|_      |Develop Guide|_    |H/W Reference|_
------------------  ------------------  ------------------
`Get Started`_      `Develop Guide`_    `H/W Reference`_
==================  ==================  ==================

.. |Get Started| image:: ../../_static/get-started.png
.. _Get Started: gettingstarted.html

.. |Develop Guide| image:: ../../_static/api-reference.png
.. _Develop Guide: gettingstarted.html

.. |H/W Reference| image:: ../../_static/hw-reference.png
.. _H/W Reference: gettingstarted.html

.. toctree::
   :hidden:

   Get Started <gettingstarted>
   Set up Development Environment <getespidf>
   Development Guide <developerguide>
   Hardware Reference <hardware>
   Drivers <drivers>
   Flight Control System <system>
   Communications <communication>
   Notice <notice>
