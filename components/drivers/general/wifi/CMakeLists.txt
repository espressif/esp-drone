set(wifi_sources)
idf_build_get_property(wifi_idf_target IDF_TARGET)
if(NOT wifi_idf_target STREQUAL "esp32p4" OR CONFIG_TRANSPORT_REMOTE_WIFI)
  list(APPEND wifi_sources "wifi_esp32.c")
endif()

idf_component_register(SRCS ${wifi_sources}
                      INCLUDE_DIRS "." "include"
                      REQUIRES crazyflie platform config esp_wifi esp_event esp_netif lwip nvs_flash)
