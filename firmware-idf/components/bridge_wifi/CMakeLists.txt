idf_component_register(SRCS "bridge_wifi.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_event esp_netif esp_wifi nvs_flash
                       PRIV_REQUIRES esp_timer)
