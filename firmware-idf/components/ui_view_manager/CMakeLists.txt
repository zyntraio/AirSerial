idf_component_register(
    SRCS "ui_view_manager.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_lcd esp_driver_gpio driver bridge_uart bridge_usb_host bridge_core esp_psram heap
)
