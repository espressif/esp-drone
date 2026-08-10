idf_component_register(SRCS "deck_digital.c" "deck_spi.c"
                       INCLUDE_DIRS "." "include"
                       PRIV_REQUIRES crazyflie config esp_driver_gpio esp_driver_spi)
