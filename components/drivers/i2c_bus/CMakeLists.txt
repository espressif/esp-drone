idf_component_register(SRCS "i2c_drv.c" "i2cdev_esp32.c"
                       INCLUDE_DIRS "include"
                       REQUIRES crazyflie platform esp_driver_i2c
                       PRIV_REQUIRES config)
