idf_component_register(SRCS "adc_esp32.c"
                       INCLUDE_DIRS "." "include"
                       REQUIRES crazyflie config esp_adc_cal)