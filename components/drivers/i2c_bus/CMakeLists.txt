idf_component_register(SRCS "i2c_drv.c" "i2cdev_esp32.c"
                       INCLUDE_DIRS "include"
                       REQUIRES utils platform 
                       PRIV_REQUIRES config)