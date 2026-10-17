# main = 纯装配层：只做积木装配
idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
                       REQUIRES board bridge_wifi bridge_web bridge_core ui_cga bridge_usb_host bridge_uart
                       PRIV_REQUIRES esp_hw_support)
