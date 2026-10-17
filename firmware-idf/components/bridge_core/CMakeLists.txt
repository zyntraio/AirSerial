idf_component_register(SRCS "bridge_serial.c" "bridge_tcp.c" "bridge_mdns.c" "bridge_cfg.c" "bridge_rfc2217.c" "bridge_crash.c" "bridge_vt100.c"
                       INCLUDE_DIRS "include"
                       REQUIRES lwip freertos mdns esp_netif nvs_flash esp_timer)
