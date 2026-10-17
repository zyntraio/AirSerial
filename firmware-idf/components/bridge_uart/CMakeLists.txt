idf_component_register(SRCS "bridge_uart.c"
                       INCLUDE_DIRS "include"
                       REQUIRES bridge_core esp_driver_uart)
