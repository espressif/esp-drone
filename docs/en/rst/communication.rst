Communication Protocols
========================
:link_to_translation:`zh_CN:[中文]`

Communication Hierarchy
--------------------------

==================  =================== =======================
Terminal            Mobile/PC           ESP-Drone
Application Layer   APP                 Flight Control Firmware
Protocol Layer      CRTP                CRTP
Transport Layer     UDP                 UDP
Physical Layer      Wi-Fi STA (Station) Wi-Fi AP (Access Point)
==================  =================== =======================

Wi-Fi Communication
--------------------

Wi-Fi Performance
~~~~~~~~~~~~~~~~~~

**ESP32 Wi-Fi Performance**

=====================   ================================================================================================
Item                    Parameter
=====================   ================================================================================================
Mode                    STA mode, AP mode, STA+AP mode
Protocol                IEEE 802.11b/g/n, and 802.11 LR (Espressif). Support switching over software
Security                WPA, WPA2, WPA2-Enterprise, WPS
Main Feature            AMPDU, HT40, QoS
Supported Distance      1 km under the Exclusive Agreement of Espressif
Transfer Rate           20 Mbit/s TCP throughput, 30 Mbit/s UDP
=====================   ================================================================================================

For other parameters, see `ESP32 Wi-Fi Feature List <https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/wifi.html#esp32-wi-fi-feature-list>`__\.

**ESP32-S2 Wi-Fi Performance**

=====================   ============================================================
Item                    Parameter
=====================   ============================================================
Mode                    STA mode, AP mode, STA+AP mode
Protocol                IEEE 802.11b/g/n. Support switch over software
Security                WPA, WPA2, WPA2-Enterprise, WPS
Main Feature            AMPDU, HT40, QoS
Supported Distance      1 km under the Exclusive Agreement of Espressif
Transfer Rate           20 Mbit/s TCP throughput, 30 Mbit/s UDP
=====================   ============================================================

For other parameters, see `ESP32-S2 Wi-Fi Feature List <https://docs.espressif.com/projects/esp-idf/en/latest/esp32s2/api-guides/wifi.html#esp32-s2-wi-fi-feature-list>`__\.

Wi-Fi Programming Framework
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Wi-Fi Programming Framework Based on ESP-IDF** 

.. figure:: https://img-blog.csdnimg.cn/20200423173923300.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzIwNTE1NDYx,size_16,color_FFFFFF,t_70#pic_center
    :align: center
    :alt: Wi-Fi Programming Example
    :figclass: align-center

    Wi-Fi Programming Example

**General Programming Process** 

1. At application layer, call `Wi-Fi Driver API <https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_wifi.html>`__ to initialize Wi-Fi.
2. Wi-Fi driver is transparent to developers. When an event occurs, Wi-Fi driver sends an ``event`` to `default event loop <https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/esp_event.html#esp-event-default-loops>`__. Applications can write and register ``handle`` program on demand.
3. Network interface component `esp_netif <https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_netif.html>`__ provides the ``handle`` program associated with the Wi-Fi driver ``event`` by default. For example, when ESP32 works as an AP, and a user connects to this AP, esp_netif will automatically start the DHCP service. 

For the detailed usage, please refer to the code ``\components\drivers\general\wifi\wifi_esp32.c``\.

.. note::

    Before Wi-Fi initialization, please use ``WIFI_INIT_CONFIG_DEFAULT`` to obtain the initialization configuration struct, and customize this struct first, then start the initialization. Be aware of problems caused by unitialized members of the struct, and pay special attention to this issue when new structure members are added to the ESP-IDF during update.

**AP Mode Workflow**

.. figure:: https://img-blog.csdnimg.cn/2020042622523887.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzIwNTE1NDYx,size_16,color_FFFFFF,t_70#pic_center
    :align: center
    :alt: Sample Wi-Fi Event Scenarios in AP Mode
    :figclass: align-center

    Sample Wi-Fi Event Scenarios in AP Mode

Increase Wi-Fi Communication Distance
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Navigate to ``Component config>>PHY>>Max WiFi TX power (dBm)``, and update ``Max WiFi TX power`` to ``20``. This configuration increases the PHY gain and Wi-Fi communication distance.

UDP Communication
------------------

UDP Port
~~~~~~~~~~

=====================   =================== =======================
App                     Direction           ESP-Drone
=====================   =================== =======================
192.168.43.42::2399     TX/RX               192.168.43.42::2390
=====================   =================== =======================

UDP Packet Structure
~~~~~~~~~~~~~~~~~~~~

.. code:: text

   /* Frame format:
    * +=============+-----+-----+
    * | CRTP                      |   CKSUM   |
    * +=============+-----+-----+
    */

- The packet transmitted by the UDP: CRTP + verification information. 
- CRTP: As defined by the CRTP packet structure, it contains Header and Data, as detailed in the CRTP protocol section. 
- CKSUM: the verification information. Its size is 1 byte, and this CKSUM is incremented by CRTP packet byte.

**CKSUM Calculation Method**

.. code:: python

   #take python as an example: Calculate the raw cksum and add it to the end of the packet
    raw = (pk.header,) + pk.datat
    cksum = 0
    for i in raw:
           cksum += i
    cksum %= 256
    raw = raw + (cksum,)

CRTP Protocol
------------------

The ESP-Drone project continues the CRTP protocol used by the Crazyflie project for flight instruction sending, flight data passback, parameter settings, etc.

CRTP implements a stateless design that does not require a handshake step. Any command can be sent at any time, but for some log/param/mem commands, the TOC (directory) needs to be downloaded to assist the host in sending the information correctly. The implemented Python API (cflib) can download param/log/mem TOC to ensure that all functions are available.

CRTP Packet Structure
~~~~~~~~~~~~~~~~~~~~~~

The 32-byte CRTP packet contains one byte of Header and 31 bytes of Payload. Header records the information about the ports (4 bits), channels (2 bits), and reserved bits (2 bits).

.. code:: text

     7   6   5   4   3   2   1   0
   +---+---+---+---+---+---+---+---+
   |     Port      |  Res. | Chan. | 
   +---+---+---+---+---+---+---+---+
   |            DATA 0             |
   +---+---+---+---+---+---+---+---+
   :   :   :   :   :   :   :   :   :
   +---+---+---+---+---+---+---+---+
   |            DATA 30            |
   +---+---+---+---+---+---+---+---+

========    ========    ==============  =============================
Field       Byte        Bit             Description
========    ========    ==============  =============================
Header      0           0 ~ 1           Target data channel
\           0           2 ~ 3           Reserved for transport layer
\           0           4 ~ 7           Target data port
Data        1 ~ 31      0 ~ 7           The data in this packet
========    ========    ==============  =============================

Port Allocation
~~~~~~~~~~~~~~~~

======  =====================   ===================================================================================
Port    Target                  Purpose
======  =====================   ===================================================================================
0       Console                 Read console text that is printed to the console on the Crazyflie using consoleprintf.
2       Parameters              Get/set parameters from the Crazyflie. Parameters are defined using a macro in the Crazyflie source-code
3       Commander               Send control set-points for the roll/pitch/yaw/thrust regulators
4       Memory access           Access non-volatile memories like 1-wire and I2C (only supported for Crazyflie 2.0)
5       Data logging            Set up log blocks with variables that will be sent back to the Crazyflie at a specified period. Log variables are defined using a macro in the Crazyflie source-code
6       Localization            Packets related to localization
7       Generic Setpoint        Allows to send setpoint and control modes
13      Platform                Used for misc platform control, like debugging and power off
14      Client-side debugging   Debugging the UI and exists only in the Crazyflie Python API and not in the Crazyflie itself.
15      Link layer              Used to control and query the communication link
======  =====================   ===================================================================================

Most of the modules in the firmware that are connected to the port are implemented as tasks. If an incoming CRTP packet is delivered in the messaging queue, the task is blocked in the queue. At startup, each task and other modules need to be registered for a predefined port at the communication link layer.

Details of the use of each port can be found at \ `CRTP - Communicate with Crazyflie <https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/functional-areas/crtp/>`__\.

Supported Package by CRTP Protocol
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

cflib is a Python package supported by CRTP protocol,  and provides an application-layer interface for communication protocols that can be used to build an upper PC, to communicate with Crazyflie and Crazyflie 2.0 quadcopters.
Each component in the firmware that uses the CRTP protocol has a script corresponding to it in cflib. 

-  Source repository: `crazyflie-lib-python <https://github.com/bitcraze/crazyflie-lib-python>`__.
-  cflib repository specially for ESP-Drone: `qljz1993/crazyflie-lib-python <https://github.com/qljz1993/crazyflie-lib-python.git>`__. Please checkout to ``esplane`` branch.

Application Development Based on CRTP Protocol
------------------------------------------------

Examples for Various Platform
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

1. `crazyflie2-ios-client <https://github.com/bitcraze/crazyflie2-ios-client>`__

2. `crazyflie2-windows-uap-client <https://github.com/bitcraze/crazyflie2-windows-uap-client>`__

3. `crazyflie-android-client <https://github.com/bitcraze/crazyflie-android-client>`__

4. `User Guide on Android <https://wiki.bitcraze.io/doc:crazyflie:client:cfandroid:index>`__

5. `Development Guide on Android <https://wiki.bitcraze.io/doc:crazyflie:dev:env:android>`__

cfclient
~~~~~~~~

cfclient is the upper PC for ``Crazeflie`` project, which has fully implemented the functions defined in ``CRTP`` Protocol, and speeds up the debug process for the drone. The ESP-Drone project tailors and adjusts the upper PC to meet functional design needs.

.. figure:: ../../_static/cfclient.png
    :align: center
    :alt: Cfclient Control Interface
    :figclass: align-center

    Cfclient Control Interface

For detailed information about cfclient, please refer to `cfclient <gettingstarted.html#pc-cfclient>`__.
