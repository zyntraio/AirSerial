/*
 * board_cardputer_adv — M5Stack Cardputer ADV 板级胶水
 *
 * Stamp-S3A (ESP32-S3FN8)：8MB flash，无 PSRAM
 * 单 Type-C：USB-SERIAL-JTAG + OTG 共享 PHY → SHARED_PHY topo
 *   USB Host 需自供电 OTG 转接（VBUS 只进不出，用户实测可枚举）
 * 屏：ST7789V2 1.14" 240x135 SPI
 * 键盘：TCA8418 @ I2C 0x34（SDA=G8/SCL=G9，INT=G11）——v0.4.0 后续接入
 * 主路线：USB Host（自供电 OTG）；兜底：Grove G1/G2 UART + EXT 排针 DTR/RTS
 *
 * Tier 1: 有屏有键盘 → 全屏终端
 */
#include "board.h"
#include "ui_cga.h"
#include "ui_view_manager.h"

#include "esp_log.h"
#include "esp_efuse.h"
#include "esp_efuse_table.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_st7789.h"
#include "esp_lcd_panel_dev.h"
#include "esp_lcd_io_spi.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <string.h>

#define TAG "board_cardputer_adv"

/* — 屏幕能力（ui_cga 用）—
 * ST7789V2 240x135 原生横屏 */
#define BOARD_LCD_COLS   30    /* 240px / ~8px 每字符 */
#define BOARD_LCD_ROWS   8     /* 135px / 16px 每行 */
#define BOARD_LCD_HRES   240
#define BOARD_LCD_VRES   135

/* — LCD GPIO（官方 PinMap：ST7789V2）— */
#define LCD_MOSI_GPIO   35
#define LCD_SCK_GPIO    36
#define LCD_RS_GPIO     34    /* 官方丝印 RS = DC */
#define LCD_CS_GPIO     37
#define LCD_RST_GPIO    33
#define LCD_BL_GPIO     38
#define LCD_SPI_HOST    SPI2_HOST

/* — DTR/RTS（EXT 14P 排针 pin3/pin5，丝印 INT/BUSY）—
 * 开漏 + idle 释放，同 sticks3/powerhub 模式（防探测期误夹目标板）。
 * esptool classic_bootloader_reset 约定：RTS→EN, DTR→IO0 */
#define BOARD_DTR_GPIO   4    /* 客户端 DTR → 目标板 IO0/BOOT */
#define BOARD_RTS_GPIO   6    /* 客户端 RTS → 目标板 EN/RST */

/* — 键盘（TCA8418 @ I2C 0x34，SDA=G8/SCL=G9，INT=G11）——v0.4.0 后续接入 — */
#define KBD_I2C_SDA_GPIO  8
#define KBD_I2C_SCL_GPIO  9
#define KBD_INT_GPIO      11

static esp_lcd_panel_handle_t s_panel;

/* — 设备描述（ui_cga 分级用）— */
static board_info_t s_info = {
    .name       = "M5Stack Cardputer ADV",
    .target     = "esp32s3",
    .has_lcd    = true,
    .has_kbd    = false,  /* TCA8418 接入后改 true */
    .has_psram  = false,  /* S3FN8 无 PSRAM */
    .cols       = BOARD_LCD_COLS,
    .rows       = BOARD_LCD_ROWS,
    .tier       = BOARD_TIER_1_TERMINAL,
};

const board_info_t *board_get_info(void)
{
    return &s_info;
}

/* — USB 拓扑：硬声明 SHARED_PHY（与 sticks3 同构，无 PMIC/mux） — */
static board_usb_topo_t s_detected_topo = USB_TOPO_UNKNOWN;

board_usb_topo_t board_get_usb_topo(void) { return s_detected_topo; }
bool board_usb_host_init_safe(void)
{
    /* SHARED_PHY：Host init 后 console 会断——开发期注意安全 */
    return s_detected_topo == USB_TOPO_INDEPENDENT;
}

void board_detect_usb_topo(void)
{
    ESP_LOGI(TAG, "=== USB topology detect ===");
    const board_usb_topo_t board_declared = USB_TOPO_SHARED_PHY;
    bool otg_disabled = esp_efuse_read_field_bit(ESP_EFUSE_DIS_USB_OTG);
    ESP_LOGI(TAG, "  efuse OTG disabled: %d", otg_disabled);
    if (otg_disabled) {
        s_detected_topo = USB_TOPO_NO_HOST;
        ESP_LOGW(TAG, "  efuse OTG disabled overrides -> NO_HOST");
    } else {
        s_detected_topo = board_declared;
    }
    ESP_LOGI(TAG, "  => USB topo: %s",
             s_detected_topo == USB_TOPO_SHARED_PHY ? "SHARED_PHY" :
             s_detected_topo == USB_TOPO_NO_HOST   ? "NO_HOST"    : "UNKNOWN");
}

/* — 对外暴露 LCD panel handle — */
void *board_get_lcd_panel(void) { return s_panel; }
int   board_get_lcd_hres(void) { return BOARD_LCD_HRES; }
int   board_get_lcd_vres(void) { return BOARD_LCD_VRES; }

/* — ui_cga draw_row 回调 — */
static esp_err_t board_lcd_draw_row(int row, const uint8_t *pixels, int width_bytes)
{
    if (!s_panel) return ESP_ERR_INVALID_STATE;
    int y_start = row * 16;
    int y_end   = y_start + 15;
    esp_lcd_panel_draw_bitmap(s_panel, 0, y_start, BOARD_LCD_HRES, y_end + 1, pixels);
    return ESP_OK;
}

/* — LCD init（ADV 无 PMIC，BL 直接 GPIO）— */
static esp_err_t board_lcd_init(void)
{
    ESP_LOGI(TAG, "=== LCD init (ST7789V2 240x135) ===");

    /* BL 先关，init 完再开 */
    gpio_config_t bl_cfg = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << LCD_BL_GPIO),
    };
    ESP_ERROR_CHECK(gpio_config(&bl_cfg));
    gpio_set_level(LCD_BL_GPIO, 0);

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = LCD_MOSI_GPIO,
        .miso_io_num = -1,
        .sclk_io_num = LCD_SCK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = BOARD_LCD_HRES * 16 * 2,   /* 行刷：一行 16px 高 */
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO));

    esp_lcd_panel_io_spi_config_t io_cfg = {
        .dc_gpio_num = LCD_RS_GPIO,
        .cs_gpio_num = LCD_CS_GPIO,
        .pclk_hz = 40 * 1000 * 1000,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };
    esp_lcd_panel_io_handle_t io_handle = NULL;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_SPI_HOST, &io_cfg, &io_handle));

    esp_lcd_panel_dev_config_t dev_cfg = {
        .reset_gpio_num = LCD_RST_GPIO,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .data_endian = LCD_RGB_DATA_ENDIAN_LITTLE,
        .bits_per_pixel = 16,
        .flags.reset_active_high = 0,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &dev_cfg, &s_panel));

    ESP_ERROR_CHECK(esp_lcd_panel_reset(s_panel));
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_ERROR_CHECK(esp_lcd_panel_init(s_panel));

    /* 与原版 Cardputer 同代面板：invert + 横屏修正。
     * 240x135 原生横屏，不 swap_xy；offset 实机再校 */
    esp_lcd_panel_invert_color(s_panel, true);
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(s_panel, true, false));
    ESP_ERROR_CHECK(esp_lcd_panel_set_gap(s_panel, 40, 53));

    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(s_panel, true));
    vTaskDelay(pdMS_TO_TICKS(100));

    gpio_set_level(LCD_BL_GPIO, 1);
    ESP_LOGI(TAG, "BL GPIO%d = 1 (ON)", LCD_BL_GPIO);

    ui_cga_set_panel_callbacks(board_lcd_draw_row, BOARD_LCD_HRES);
    ESP_LOGI(TAG, "=== LCD init done ===");
    return ESP_OK;
}

/* ================================================================
 *  DTR/RTS — UART 兜底路线的 esptool 自动复位（EXT 排针 G4/G6）
 * ================================================================ */
static bool s_serial_ctrl_inited;

esp_err_t board_serial_ctrl_init(void)
{
    if (s_serial_ctrl_inited) return ESP_OK;

    gpio_config_t cfg = {
        .mode = GPIO_MODE_OUTPUT_OD,
        .pin_bit_mask = (1ULL << BOARD_DTR_GPIO) | (1ULL << BOARD_RTS_GPIO),
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&cfg));

    /* idle：释放（高阻），电平由目标板板上拉定义 */
    gpio_set_level(BOARD_DTR_GPIO, 1);
    gpio_set_level(BOARD_RTS_GPIO, 1);

    s_serial_ctrl_inited = true;
    ESP_LOGI(TAG, "DTR/RTS init: DTR=G%d RTS=G%d (EXT 14P pin3/pin5, idle=release/OD)",
             BOARD_DTR_GPIO, BOARD_RTS_GPIO);
    return ESP_OK;
}

esp_err_t board_serial_ctrl_set(bool dtr_active, bool rts_active)
{
    if (!s_serial_ctrl_inited) return ESP_ERR_INVALID_STATE;
    gpio_set_level(BOARD_DTR_GPIO, dtr_active ? 0 : 1);
    gpio_set_level(BOARD_RTS_GPIO, rts_active ? 0 : 1);
    return ESP_OK;
}

bool board_serial_ctrl_is_supported(void) { return true; }

esp_err_t board_init(void)
{
    ESP_LOGI(TAG, "board_init (M5Stack Cardputer ADV)");
    ESP_LOGI(TAG, "  console: USB-SERIAL-JTAG (Type-C)");
    ESP_LOGI(TAG, "  flash: 8MB, PSRAM: none (S3FN8)");

    /* LCD init — 失败不 crash，log warning 继续跑 */
    esp_err_t lcd_err = board_lcd_init();
    if (lcd_err != ESP_OK) {
        ESP_LOGW(TAG, "LCD init FAILED: %s — continuing without display",
                 esp_err_to_name(lcd_err));
        s_info.has_lcd = false;
    }

    /* bridge_uart: Grove G1(TX)/G2(RX)，引脚分叉在 bridge_uart.h 按板宏 */

    /* DTR/RTS（EXT 排针 G4/G6） */
    board_serial_ctrl_init();

    board_detect_usb_topo();

    if (s_panel) {
        ESP_LOGI(TAG, "starting ui_view_manager...");
        ui_view_manager_start(s_panel, BOARD_LCD_HRES, BOARD_LCD_VRES,
                              BOARD_LCD_COLS, BOARD_LCD_ROWS);
    }

    board_fill_psram_cap(&s_info);

    ESP_LOGI(TAG, "board_init done");
    return ESP_OK;
}
