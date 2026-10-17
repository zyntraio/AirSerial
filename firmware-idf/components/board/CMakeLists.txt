# 根据 sdkconfig.defaults 里 CONFIG_BOARD_TYPE 选板级源文件
# board_common.c 通用实现（PSRAM 能力检测 + AI 判断 + 便利查询）
idf_component_register(
    SRCS
        board_common.c
        $<$<BOOL:${CONFIG_BOARD_TYPE_STICKS3}>:board_sticks3.c>
        $<$<BOOL:${CONFIG_BOARD_TYPE_N16R8}>:board_n16r8.c>
        $<$<BOOL:${CONFIG_BOARD_TYPE_CARD_PUTER_ADV}>:board_cardputer_adv.c>
    INCLUDE_DIRS "include"
    REQUIRES esp_lcd esp_driver_uart esp_driver_spi esp_driver_i2c driver ui_cga ui_view_manager efuse esp_psram
)

message(STATUS "[board] CONFIG_BOARD_TYPE_STICKS3=${CONFIG_BOARD_TYPE_STICKS3}")
message(STATUS "[board] CONFIG_BOARD_TYPE_N16R8=${CONFIG_BOARD_TYPE_N16R8}")
message(STATUS "[board] CONFIG_BOARD_TYPE_CARD_PUTER_ADV=${CONFIG_BOARD_TYPE_CARD_PUTER_ADV}")
if(CONFIG_BOARD_TYPE_STICKS3)
    message(STATUS "[board] → M5Stack StickS3 (SHARED_PHY, 8MB flash)")
elseif(CONFIG_BOARD_TYPE_N16R8)
    message(STATUS "[board] → N16R8 公模板子 (INDEPENDENT, 16MB flash)")
elseif(CONFIG_BOARD_TYPE_CARD_PUTER_ADV)
    message(STATUS "[board] → M5Stack Cardputer-Adv (SHARED_PHY, 8MB flash, 有屏有键盘)")
else()
    message(FATAL_ERROR "CONFIG_BOARD_TYPE not set! Use sdkconfig.defaults.sticks3 / .n16r8")
endif()
