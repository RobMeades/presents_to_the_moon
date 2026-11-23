idf_build_get_property(project_dir PROJECT_DIR)
idf_component_register(SRCS "presents_to_the_moon.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES esp_event esp_driver_uart esp_driver_gpio bt fgr_util fgr_ws2812 fgr_debug fgr_nvs fgr_task fgr_rram fgr_network fgr_time tmc5160 ht16k33a
                       REQUIRES esp_psram)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)
