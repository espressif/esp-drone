idf_component_register(SRCS "deck_digital.c" "deck_spi.c"
                       INCLUDE_DIRS "." "include"
                       REQUIRES crazyflie utils config)