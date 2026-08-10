idf_component_register(SRCS "eeprom.c"
                       INCLUDE_DIRS "." "include"
                       REQUIRES i2c_bus
                       PRIV_REQUIRES nvs_flash)
