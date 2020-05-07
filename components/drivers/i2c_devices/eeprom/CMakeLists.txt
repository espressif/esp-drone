idf_component_register(SRCS "eeprom.c"
                       INCLUDE_DIRS "." "include"
                       REQUIRES utils spi_flash)