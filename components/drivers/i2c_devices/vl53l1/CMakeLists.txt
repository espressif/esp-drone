idf_component_register(SRCS "vl53l1x.c" 
                    "zranger2.c"
                    "core/src/vl53l1_api_calibration.c"
                    "core/src/vl53l1_api_core.c"
                    "core/src/vl53l1_api_debug.c"
                    "core/src/vl53l1_api_preset_modes.c"
                    "core/src/vl53l1_api_strings.c"
                    "core/src/vl53l1_api.c"
                    "core/src/vl53l1_core_support.c"
                    "core/src/vl53l1_core.c"
                    "core/src/vl53l1_error_strings.c"
                    "core/src/vl53l1_register_funcs.c"
                    "core/src/vl53l1_silicon_core.c"
                    "core/src/vl53l1_wait.c"
                       INCLUDE_DIRS "." "include" "core/inc"
                     REQUIRES i2c_bus crazyflie platform config)

target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-overflow")