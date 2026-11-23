# HT16K33A 14-digit display driver driver
idf_component_register(SRCS "display.c"
                       INCLUDE_DIRS "api"
                       PRIV_REQUIRES fgr_util fgr_task esp_timer
                       REQUIRES ht16k33a)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)
