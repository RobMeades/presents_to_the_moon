# HT16K33A 14-digit display driver driver
idf_component_register(SRCS "ht16k33a.c"
                       INCLUDE_DIRS "api"
                       PRIV_REQUIRES fgr_util esp_driver_gpio esp_driver_i2c esp_timer)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)
