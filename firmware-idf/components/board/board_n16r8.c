/*
 * board_n16r8 — N16R8 公模板子 板级胶水
 *
 * 硬件:
 *   - ESP32-S3 + 16MB boya Flash + 8MB Octal PSRAM
 *   - Type-C #1 → CH343 → GPIO43(TX)/GPIO44(RX) (独立 USB 转串口 console)
 *   - Type-C #2 → 直通 OTG (GPIO19/20 + VBUS)
 *   - 双口独立 → **INDEPENDENT** topo → USB Host 模式 console 活！
 *
 * Tier 3: 无屏无键盘 → 状态页 (ui_cga 日志版)
 */
#include "board.h"
#include "ui_cga.h"

#include "esp_log.h"
#include "esp_efuse.h"
#include "esp_efuse_table.h"
#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <string.h>

#define TAG "board_n16r8"

/* — 设备描述 — */
static board_info_t s_info = {
    .name       = "N16R8 公模板子",
    .target     = "esp32s3",
    .has_lcd    = false,
    .has_kbd    = false,
    .has_psram  = true,
    .cols       = 30,   /* ui_cga 默认 */
    .rows       = 8,
    .tier       = BOARD_TIER_3_STATUS,
};

const board_info_t *board_get_info(void) { return &s_info; }

/* — DTR/RTS 直控 —
 * 公模未定义连目标板 RST/BOOT 的 GPIO：返回 NOT_SUPPORTED。
 * USB Host 路线下 esptool 复位由 USB-UART 模块自身的 DTR/RTS 经 CDC-ACM 完成；
 * UART 兜底路线若用户自行接线，可在此板型上补引脚定义。 */
esp_err_t board_serial_ctrl_init(void) { return ESP_ERR_NOT_SUPPORTED; }
esp_err_t board_serial_ctrl_set(bool dtr_active, bool rts_active)
{
    (void)dtr_active; (void)rts_active;
    return ESP_ERR_NOT_SUPPORTED;
}
bool board_serial_ctrl_is_supported(void) { return false; }

/* board_has_psram / board_has_display / board_has_keyboard / board_get_tier
 * 现在统一在 board_common.c 里实现 */

/* — USB 拓扑状态 — */
static board_usb_topo_t s_detected_topo = USB_TOPO_UNKNOWN;

board_usb_topo_t board_get_usb_topo(void) { return s_detected_topo; }

bool board_usb_host_init_safe(void)
{
    /* INDEPENDENT → 有 CH343 独立串口通路 → Host 模式 console 活 ✅ */
    return s_detected_topo == USB_TOPO_INDEPENDENT;
}

/* — USB 拓扑检测 —
 * N16R8: efuse 硬声明 INDEPENDENT，efuse OTG disabled 是唯一覆写条件 */
void board_detect_usb_topo(void)
{
    ESP_LOGI(TAG, "=== USB topology detect ===");

    bool otg_disabled = esp_efuse_read_field_bit(ESP_EFUSE_DIS_USB_OTG);
    ESP_LOGI(TAG, "  efuse OTG disabled: %d", otg_disabled);
    ESP_LOGI(TAG, "  board declared: INDEPENDENT (CH343 GPIO43/44 + 直通 OTG)");

    if (otg_disabled) {
        s_detected_topo = USB_TOPO_NO_HOST;
        ESP_LOGW(TAG, "  ⚠️ efuse OTG disabled overrides → NO_HOST");
    } else {
        s_detected_topo = USB_TOPO_INDEPENDENT;
    }

    ESP_LOGI(TAG, "  ⇒ USB topo: %s",
             s_detected_topo == USB_TOPO_INDEPENDENT ? "INDEPENDENT ✅" :
             s_detected_topo == USB_TOPO_NO_HOST      ? "NO_HOST (efuse)"  : "UNKNOWN");
    ESP_LOGI(TAG, "=== USB topology detect done ===");
}

esp_err_t board_init(void)
{
    ESP_LOGI(TAG, "board_init (N16R8 公模板子)");
    ESP_LOGI(TAG, "  console UART0: TX=GPIO43 RX=GPIO44 (CH343, Type-C #1)");
    ESP_LOGI(TAG, "  OTG: GPIO19/20 直通 Type-C #2");
    ESP_LOGI(TAG, "  flash: 16MB boya, PSRAM: 8MB Octal");

    board_detect_usb_topo();

    ui_cga_boot_splash(30);

    /* PSRAM 能力检测 */
    board_fill_psram_cap(&s_info);

    ESP_LOGI(TAG, "board_init done");
    return ESP_OK;
}
