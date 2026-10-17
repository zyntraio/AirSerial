/*
 * board_cardputer_adv — M5Stack Cardputer-Adv 板级胶水
 *
 * 核心: Stamp-S3A (ESP32-S3FN8, 8MB Flash, 无外部 PSRAM)
 * USB:  单 Type-C, USB-SERIAL-JTAG (GPIO19/20) + OTG 共享 PHY
 *       → SHARED_PHY: Host 模式 console 会断
 * 屏幕: ST7789V2 1.14", 240x135 (SPI)
 * 键盘: TCA8418 I2C (G8=SDA, G9=SCL, G11=INT) — 56 键
 * 音频: ES8311 (G8=SDA, G9=SCL + 音频专用引脚)
 * IMU:  BMI270 (I2C 共用 G8/G9)
 * SD:   microSD (G12=CS, G14=MOSI, G40=CLK, G39=MISO)
 * IR:   G44 (⚠️ 不是 console RX!)
 * RGB:  GPIO38 = power switch (软件控制前要拉高)
 *
 * USB 拓扑: SHARED_PHY (跟 StickS3 一样)
 * Console: USB-SERIAL-JTAG GPIO19/20
 */
#include "board.h"
#include "ui_cga.h"

#include "esp_log.h"
#include "esp_efuse.h"
#include "esp_efuse_table.h"
#include "esp_system.h"
#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <string.h>

#define TAG "board_cardputer"

/* — 屏幕参数 —
 * ST7789V2 1.14": 240x135, 但通常会用 240x240 驱动，中心 240x135 可见 */
#define BOARD_LCD_W      240
#define BOARD_LCD_H      135
#define BOARD_LCD_COLS   30   /* 240px / 8px 字符宽 */
#define BOARD_LCD_ROWS   8    /* 135px / 16px 行高 */

/* — 设备描述（ui_cga 分级 + Portal 显示）—
 * Tier 2: 有屏有键盘 → 完整交互（v0.5.0 目标）*/
static board_info_t s_info = {
    .name       = "M5Stack Cardputer-Adv",
    .target     = "esp32s3",
    .has_lcd    = true,    /* 有 ST7789V2 屏幕 */
    .has_kbd    = true,    /* 有 TCA8418 键盘 */
    .has_psram  = false,   /* ESP32-S3FN8 无外部 PSRAM */
    .cols       = BOARD_LCD_COLS,
    .rows       = BOARD_LCD_ROWS,
    .tier       = BOARD_TIER_1_TERMINAL,   /* 有屏有键盘 → 全屏终端 */
};

/* — 屏幕引脚 (SPI) — */
#define LCD_CS_GPIO     GPIO_NUM_37
#define LCD_SCK_GPIO    GPIO_NUM_36
#define LCD_MOSI_GPIO   GPIO_NUM_35    /* DAT */
#define LCD_RS_GPIO     GPIO_NUM_34
#define LCD_RST_GPIO    GPIO_NUM_33
#define LCD_BL_GPIO     GPIO_NUM_38    /* 同时也是 RGB LED power */

/* — I2C (键盘/IMU/音频共用) — */
#define I2C_SDA_GPIO    GPIO_NUM_8
#define I2C_SCL_GPIO    GPIO_NUM_9

/* — microSD (SPI) — */
#define SD_CS_GPIO      GPIO_NUM_12
#define SD_MOSI_GPIO    GPIO_NUM_14
#define SD_CLK_GPIO     GPIO_NUM_40
#define SD_MISO_GPIO    GPIO_NUM_39

/* — IR — */
#define IR_TX_GPIO      GPIO_NUM_44    /* ⚠️ 不是 console! Cardputer 没有 UART0 */

/* — RGB LED —
 * Stamp-S3A 相比 Stamp-S3 多了独立电子开关，
 * 软件控制 RGB LED 前必须把 GPIO38 拉高 (power enable) */
#define RGB_PWR_EN_GPIO GPIO_NUM_38

const board_info_t *board_get_info(void)
{
    return &s_info;
}

/* board_has_psram / board_has_display / board_has_keyboard / board_get_tier
 * 现在统一在 board_common.c 里实现 */

/* — USB 拓扑 —
 * Stamp-S3A 核心模块: 单 Type-C 共享 USB-SERIAL-JTAG + OTG
 * → SHARED_PHY: Host 模式下 console 会断 */
static board_usb_topo_t s_detected_topo = USB_TOPO_UNKNOWN;

board_usb_topo_t board_get_usb_topo(void) { return s_detected_topo; }
bool board_usb_host_init_safe(void)
{
    /* SHARED_PHY → Host init 后 USB-SERIAL-JTAG 会被 OTG 抢占 */
    return s_detected_topo == USB_TOPO_INDEPENDENT;
}

/* — USB 拓扑检测 — Cardputer-Adv 硬声明 SHARED_PHY + efuse sanity check — */
void board_detect_usb_topo(void)
{
    ESP_LOGI(TAG, "=== USB topology detect ===");

    /* 板级硬声明 —— M5Stack Cardputer-Adv (Stamp-S3A)
     *   - 单 Type-C, USB-SERIAL-JTAG (GPIO19/20) 跟 OTG 共用 PHY
     *   - 没有独立的 UART0 (GPIO43/44 在 Stamp-S3A 上没接)
     *   → SHARED_PHY: Host 模式下 console 会断 */
    const board_usb_topo_t board_declared = USB_TOPO_SHARED_PHY;

    bool otg_disabled = esp_efuse_read_field_bit(ESP_EFUSE_DIS_USB_OTG);

    ESP_LOGI(TAG, "  efuse OTG disabled: %d", otg_disabled);
    ESP_LOGI(TAG, "  board declared: SHARED_PHY (Stamp-S3A 单口共享)");

    if (otg_disabled) {
        s_detected_topo = USB_TOPO_NO_HOST;
        ESP_LOGW(TAG, "  ⚠️ efuse OTG disabled overrides → NO_HOST");
    } else {
        s_detected_topo = board_declared;
    }

    ESP_LOGI(TAG, "  ⇒ USB topo: %s (Host default OFF)",
             s_detected_topo == USB_TOPO_SHARED_PHY ? "SHARED_PHY" :
             s_detected_topo == USB_TOPO_NO_HOST ? "NO_HOST" : "OTHER");
    ESP_LOGI(TAG, "=== USB topology detect done ===");
}

/* — RGB LED power enable —
 * Stamp-S3A 特有的电子开关，控制前必须先拉高 GPIO38 */
static void board_rgb_pwr_enable(void)
{
    gpio_config_t cfg = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << RGB_PWR_EN_GPIO),
    };
    gpio_config(&cfg);
    gpio_set_level(RGB_PWR_EN_GPIO, 1);
    ESP_LOGI(TAG, "RGB LED power enabled (GPIO38=HIGH)");
}

esp_err_t board_init(void)
{
    ESP_LOGI(TAG, "board_init (Cardputer-Adv, Stamp-S3A)");
    ESP_LOGI(TAG, "  core:   ESP32-S3FN8, 8MB flash, no PSRAM");
    ESP_LOGI(TAG, "  console: USB-SERIAL-JTAG GPIO19/20");
    ESP_LOGI(TAG, "  display: ST7789V2 1.14\" 240x135");
    ESP_LOGI(TAG, "  keyboard: TCA8418 (I2C G8/G9)");
    ESP_LOGI(TAG, "  sd:     microSD (SPI G12/G14/G40/G39)");

    /* RGB LED power — Stamp-S3A 必须先开 */
    board_rgb_pwr_enable();

    /* USB 拓扑检测（efuse + 硬声明）*/
    board_detect_usb_topo();

    /* boot_splash — v0.5.0: LCD panel 版替代 monitor 日志版 */
    ui_cga_boot_splash(BOARD_LCD_COLS);

    /* PSRAM 能力检测（Cardputer-Adv 无 PSRAM → psram_size=0, ai=false）*/
    board_fill_psram_cap(&s_info);

    ESP_LOGI(TAG, "board_init done");
    return ESP_OK;
}
