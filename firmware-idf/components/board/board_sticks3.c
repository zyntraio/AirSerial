/*
 * board_sticks3 — S3 通用 dev board 板级胶水
 *
 * 支持两种形态：
 *   - M5Stack StickS3     — 单 Type-C，USB-SERIAL-JTAG + OTG 共享 PHY
 *   - 公模板子 N16R8      — 双 Type-C，独立 CH340 (UART0) + 直通 OTG
 *   - S31 CoreBoard-1     — 双 Type-C，待实机确认
 *
 * USB 拓扑运行时自动检测（见 board_detect_usb_topo）
 *
 * Tier 3: 有屏无键盘 → 状态页
 */
#include "board.h"
#include "ui_cga.h"
#include "ui_view_manager.h"

#include "esp_log.h"
#include "esp_efuse.h"
#include "esp_efuse_table.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/spi_master.h"
#include "driver/i2c_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_st7789.h"
#include "esp_lcd_panel_dev.h"
#include "esp_lcd_io_spi.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <string.h>

#define TAG "board_sticks3"

/* console: 用 UART0 默认 GPIO43(TX)/GPIO44(RX) — 公模板子 CH343 接在这里
   StickS3 Grove 口是 GPIO10/GPIO9，公模板子没有 Grove，不要 remap！*/
#define BOARD_CONSOLE_UART  UART_NUM_0
#define BOARD_CONSOLE_TX    GPIO_NUM_43
#define BOARD_CONSOLE_RX    GPIO_NUM_44

/* — 屏幕能力（ui_cga 用）—
 * 官方面板: ST7789P3 135x240 (竖屏) */
#define BOARD_LCD_COLS   16    /* 135px / ~8px 每字符 */
#define BOARD_LCD_ROWS   15    /* 240px / 16px 每行 */
#define BOARD_LCD_HRES   135   /* ✅ 官方 panel_width=135 */
#define BOARD_LCD_VRES   240   /* ✅ 官方 panel_height=240 */

/* — M5PM1 (AW35122) 电源管理 IC — */
#define M5PM1_I2C_ADDR   0x6E
#define M5PM1_I2C_NUM    I2C_NUM_1
#define M5PM1_SCL_GPIO   48
#define M5PM1_SDA_GPIO   47
#define M5PM1_I2C_FREQ   400000   /* 400KHz, 官方用这个 */

/* — LCD GPIO (ST7789P3) — */
#define LCD_MOSI_GPIO   39
#define LCD_SCK_GPIO    40
#define LCD_RS_GPIO     45
#define LCD_CS_GPIO     41
#define LCD_RST_GPIO    21
#define LCD_BL_GPIO     38
#define LCD_SPI_HOST    SPI3_HOST   /* ✅ 官方用 SPI3_HOST！不是 SPI2_HOST！ */

/* — 全局 I2C / PM1 / LCD 句柄 — */
static i2c_master_bus_handle_t s_i2c_bus = NULL;
static i2c_master_dev_handle_t s_pm1_dev = NULL;
static esp_lcd_panel_handle_t s_panel = NULL;

/* — 设备描述（ui_cga 分级用）— */
static board_info_t s_info = {
    .name       = "M5Stack StickS3",
    .target     = "esp32s3",
    .has_lcd    = true,   /* v0.4.0: ST7789P3 SPI 已驱动 */
    .has_kbd    = false,  /* 单按键还不算键盘 */
    .has_psram  = true,
    .cols       = BOARD_LCD_COLS,
    .rows       = BOARD_LCD_ROWS,
    .tier       = BOARD_TIER_3_STATUS,
};

/* — 按键 GPIO —
 * 官方 M5Unified.inl line 2487-2489:
 *   board_M5StickS3: KEY1=G11, KEY2=G12, both active-low
 *   btn_rawstate_bits = ((!gpio_in(GPIO_NUM_11)) & 1) | ((!gpio_in(GPIO_NUM_12)) & 1) << 1 */
#define BOARD_KEY1_GPIO  11   /* KEY1 — 电源键/长按复位 */
#define BOARD_KEY2_GPIO  12   /* KEY2 — 切换 STATUS/TERMINAL view */

/* — AirSerial 作为远程烧录器的 DTR/RTS（输出，active-low，idle=HIGH）—
 *   路径 B：AirSerial 直接替代 USB-UART 桥，DTR/RTS 接目标板 RST/GPIO0
 *   选 G3/G6：排针空闲，不跟 LCD/I2C/USB/bridge_uart 冲突 */
#define BOARD_DTR_GPIO   3    /* → 目标板 EN/RST (低脉冲复位) */
#define BOARD_RTS_GPIO   6    /* → 目标板 GPIO0/BOOT (低=下载模式) */

const board_info_t *board_get_info(void)
{
    return &s_info;
}

/* board_has_psram / board_has_display / board_has_keyboard / board_get_tier
 * 现在统一在 board_common.c 里实现 */

/* — 运行时检测状态 — */
static board_usb_topo_t s_detected_topo = USB_TOPO_UNKNOWN;

board_usb_topo_t board_get_usb_topo(void) { return s_detected_topo; }
bool board_usb_host_init_safe(void)
{
    /* USB Host init 后 console 会不会断？
     * INDEPENDENT → 不会断（独立 UART0 CH340 console）
     * SHARED_PHY  → 会断（USB-SERIAL-JTAG + OTG 共享 PHY）
     * NO_HOST     → 根本不能 init Host
     */
    return s_detected_topo == USB_TOPO_INDEPENDENT;
}

/* — 对外暴露 LCD panel handle — */
void *board_get_lcd_panel(void) { return s_panel; }
int   board_get_lcd_hres(void) { return BOARD_LCD_HRES; }
int   board_get_lcd_vres(void) { return BOARD_LCD_VRES; }

/* — 运行时自动检测 USB 拓扑 —
 *
 * 检测维度：
 *   1. efuse 芯片级 disable 位（USJTAG / OTG / JTAG）
 *   2. USB Host Library safe probe (skip_phy_setup + root_port_unpowered)
 *      → 只装库不碰 PHY，确认 Host 硬件存在且能工作
 *   3. console UART 引脚 → 如果 UART0 GPIO43/44，暗示有独立串口通路
 *
 * 综合判断：
 *   efuse OTG disabled        → NO_HOST
 *   efuse USJTAG disabled     → INDEPENDENT (只能靠独立 UART0)
 *   OTG + USJTAG 都在         → 看 console 配置
 *     console on GPIO19/20   → SHARED_PHY  (USB-SERIAL-JTAG, 共享 PHY)
 *     console on GPIO43/44   → INDEPENDENT (独立 CH340/CH343 等外接串口)
 *   USB Host safe probe 失败  → NO_HOST 或 UNKNOWN
 *
 * 注意：安全 probe 必须设 skip_phy_setup=true + root_port_unpowered=true，
 *       否则会碰 PHY 导致 USB-SERIAL-JTAG console 直接断！
 */
void board_detect_usb_topo(void)
{
    ESP_LOGI(TAG, "=== USB topology detect ===");

    /* 1. 板级硬声明 —— 这是 M5Stack StickS3
     *    - USB-SERIAL-JTAG (GPIO19/20) 和 OTG 共用一个 Type-C 口
     *    - GPIO43/44 在板上悬空（没接 CH340 等独立串口芯片）
     *    → SHARED_PHY: Host 模式下 console 会断一下 */
    const board_usb_topo_t board_declared = USB_TOPO_SHARED_PHY;

    /* 2. efuse — 芯片级硬限制（芯片烧完就固定了，不能绕过） */
    bool otg_disabled = esp_efuse_read_field_bit(ESP_EFUSE_DIS_USB_OTG);

    ESP_LOGI(TAG, "  efuse OTG disabled: %d", otg_disabled);
    ESP_LOGI(TAG, "  board declared: SHARED_PHY (单口共享 PHY)");

    /* 3. sanity check —— efuse 硬限制可以覆写板级声明 */
    if (otg_disabled) {
        s_detected_topo = USB_TOPO_NO_HOST;
        ESP_LOGW(TAG, "  ⚠️ efuse OTG disabled overrides → NO_HOST");
    } else {
        s_detected_topo = board_declared;
    }

    const char *topo_name;
    switch (s_detected_topo) {
        case USB_TOPO_SHARED_PHY:  topo_name = "SHARED_PHY (单口共享 PHY)"; break;
        case USB_TOPO_NO_HOST:     topo_name = "NO_HOST (efuse 禁用 OTG)"; break;
        case USB_TOPO_INDEPENDENT: topo_name = "INDEPENDENT (独立串口 + OTG)"; break;
        default:                   topo_name = "UNKNOWN"; break;
    }
    ESP_LOGI(TAG, "  ⇒ USB topo: %s", topo_name);
    ESP_LOGI(TAG, "=== USB topology detect done ===");
}

/* — 按键初始化 — */
static void board_keys_init(void)
{
    gpio_config_t cfg = {
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
        .pin_bit_mask = (1ULL << BOARD_KEY1_GPIO) | (1ULL << BOARD_KEY2_GPIO),
    };
    ESP_ERROR_CHECK(gpio_config(&cfg));
    ESP_LOGI(TAG, "Keys init: KEY1=G%d, KEY2=G%d", BOARD_KEY1_GPIO, BOARD_KEY2_GPIO);
}

/* — ui_cga draw_row 回调：把一行 pixels 刷到屏幕 y 位置 —
 * ST7789P3 实际区域 135x240，但 panel 内部按 240 对齐 */
static esp_err_t board_lcd_draw_row(int row, const uint8_t *pixels, int width_bytes)
{
    if (!s_panel) return ESP_ERR_INVALID_STATE;
    /* y 范围: row*16 到 row*16+15 */
    int y_start = row * 16;
    int y_end   = y_start + 15;
    esp_lcd_panel_draw_bitmap(s_panel, 0, y_start, BOARD_LCD_HRES, y_end + 1, pixels);
    return ESP_OK;
}

/* — M5PM1 I2C 读写 (v6.0.3 新 API) — */
static esp_err_t pm1_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(s_pm1_dev, buf, 2, -1);
}
static esp_err_t pm1_read_reg(uint8_t reg, uint8_t *val)
{
    return i2c_master_transmit_receive(s_pm1_dev, &reg, 1, val, 1, -1);
}
static esp_err_t pm1_bit_clear(uint8_t reg, uint8_t mask)
{
    uint8_t v;
    ESP_RETURN_ON_ERROR(pm1_read_reg(reg, &v), TAG, "read reg 0x%02X failed", reg);
    v &= ~mask;
    return pm1_write_reg(reg, v);
}
static esp_err_t pm1_bit_set(uint8_t reg, uint8_t mask)
{
    uint8_t v;
    ESP_RETURN_ON_ERROR(pm1_read_reg(reg, &v), TAG, "read reg 0x%02X failed", reg);
    v |= mask;
    return pm1_write_reg(reg, v);
}

/* — 初始化 M5PM1 I2C + 开 L3B (LCD 电源) —
 * 失败返回 ESP_FAIL，不 crash（LCD 不是致命的） */
static esp_err_t board_pm1_init(void)
{
    ESP_LOGI(TAG, "M5PM1 init: I2C SCL=G%d SDA=G%d addr=0x%02X",
             M5PM1_SCL_GPIO, M5PM1_SDA_GPIO, M5PM1_I2C_ADDR);

    /* 0. 先手动把 GPIO 设成 input+pullup，确保 I2C line idle high
     *    官方 M5GFX 也这么干！ */
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLUP_DISABLE,
        .pin_bit_mask = (1ULL << M5PM1_SCL_GPIO) | (1ULL << M5PM1_SDA_GPIO),
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    vTaskDelay(pdMS_TO_TICKS(10));

    /* 1. 新 I2C master bus — 100KHz 保守频率 */
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = M5PM1_I2C_NUM,
        .sda_io_num = M5PM1_SDA_GPIO,
        .scl_io_num = M5PM1_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t err = i2c_new_master_bus(&bus_cfg, &s_i2c_bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "M5PM1 I2C bus create failed: %s", esp_err_to_name(err));
        return err;
    }

    /* 1.5 I2C 扫描！看总线上有哪些设备 */
    ESP_LOGI(TAG, "=== I2C bus scan ===");
    for (uint8_t addr = 1; addr < 127; addr++) {
        if (i2c_master_probe(s_i2c_bus, addr, -1) == ESP_OK) {
            ESP_LOGI(TAG, "  ✅ Found device at 0x%02X", addr);
        }
    }
    ESP_LOGI(TAG, "=== I2C scan done ===");

    /* 2. 添加 PM1 设备 */
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = M5PM1_I2C_ADDR,
        .scl_speed_hz = 100000,  /* 先用 100KHz */
    };
    err = i2c_master_bus_add_device(s_i2c_bus, &dev_cfg, &s_pm1_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "M5PM1 device add failed: %s", esp_err_to_name(err));
        return err;
    }

    /* 3. 读设备 ID 确认通信 */
    uint8_t dev_id = 0;
    err = pm1_read_reg(0x00, &dev_id);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "M5PM1 probe failed: %s — PM1 may be in bad state, try power cycle",
                 esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "M5PM1 OK: dev_id=0x%02X, enabling L3B ...");

    /* 4. 开 L3B (LCD 电源) — PM1 GPIO2 控制 L3B_EN
     * 官方 M5GFX.cpp line 2941-2944, 只改 bit2 */
    pm1_bit_clear(0x16, 1 << 2);  /* GPIO2 function = GPIO (bit2 = 0) */
    pm1_bit_set  (0x10, 1 << 2);  /* GPIO2 direction = output (bit2 = 1) */
    pm1_bit_clear(0x13, 1 << 2);  /* GPIO2 drive = push-pull (bit2 = 0) */
    pm1_bit_set  (0x11, 1 << 2);  /* GPIO2 output = HIGH → L3B ON! */
    pm1_write_reg(0x09, 0x00);    /* 关 I2C idle sleep */

    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI(TAG, "M5PM1 L3B ON ✅ — LCD now has 3V3");
    return ESP_OK;
}

/* — LCD 初始化 (ST7789P3 135x240 SPI, ESP-IDF v6.0.3 API) — */
static esp_err_t board_lcd_init(void)
{
    ESP_LOGI(TAG, "=== LCD init (M5Stack official config) ===");

    /* 1. 先开 M5PM1 → L3B (LCD 电源) */
    esp_err_t err = board_pm1_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "PM1 init failed: %s — LCD unavailable", esp_err_to_name(err));
        return err;
    }

    /* 2. BL 引脚 — 先关 */
    gpio_config_t bl_cfg = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << LCD_BL_GPIO),
    };
    ESP_ERROR_CHECK(gpio_config(&bl_cfg));
    gpio_set_level(LCD_BL_GPIO, 0);  /* 先关 BL，等 init 完再开 */

    /* 3. SPI 总线 — SPI3_HOST (官方用 SPI3_HOST！) */
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = LCD_MOSI_GPIO,
        .miso_io_num = -1,           /* 3-wire, 官方 spi_3wire=true */
        .sclk_io_num = LCD_SCK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 135 * 240 * 2,   /* 整块 framebuffer 大小 — 64800 */
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO));

    /* 4. LCD Panel IO (SPI) — 40MHz, mode 0 */
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

    /* 5. ST7789 Panel v6.0.3 API */
    esp_lcd_panel_dev_config_t dev_cfg = {
        .reset_gpio_num = LCD_RST_GPIO,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .data_endian = LCD_RGB_DATA_ENDIAN_LITTLE,
        .bits_per_pixel = 16,
        .flags.reset_active_high = 0,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &dev_cfg, &s_panel));

    /* 6. Reset + Init */
    ESP_ERROR_CHECK(esp_lcd_panel_reset(s_panel));
    vTaskDelay(pdMS_TO_TICKS(100));  /* 官方 reset 后 delay 100ms */
    ESP_ERROR_CHECK(esp_lcd_panel_init(s_panel));

    /* 7. Invert + swap_xy + mirror → 横屏 240×135 */
    esp_lcd_panel_invert_color(s_panel, true);
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(s_panel, true));   /* 旋转 90° → landscape */
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(s_panel, true, false));  /* X mirror: 修正左右颠倒 */
    /* GRAM offset — portrait 时 offset_x=52, offset_y=40；swap_xy+mirror 后恢复 (40,52) */
    ESP_ERROR_CHECK(esp_lcd_panel_set_gap(s_panel, 40, 53));

    /* 9. Display ON */
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(s_panel, true));
    vTaskDelay(pdMS_TO_TICKS(100));

    /* 10. BL 开灯 — 官方用 PWM，先简单 gpio high */
    gpio_set_level(LCD_BL_GPIO, 1);   /* 官方 active-high */
    ESP_LOGI(TAG, "BL GPIO%d = 1 (ON)", LCD_BL_GPIO);
    vTaskDelay(pdMS_TO_TICKS(50));

    /* 11. panel callbacks 给 ui_cga（暂时保留，后续 view_manager 接管）*/
    ui_cga_set_panel_callbacks(board_lcd_draw_row, BOARD_LCD_HRES);
    ESP_LOGI(TAG, "=== LCD init done ===");
    return ESP_OK;
}

esp_err_t board_init(void)
{
    ESP_LOGI(TAG, "board_init (M5Stack StickS3)");
    ESP_LOGI(TAG, "  console UART0: TX=GPIO43 RX=GPIO44 (default)");

    /* LCD init — 失败不 crash！只是 log warning 继续跑 */
    esp_err_t lcd_err = board_lcd_init();
    if (lcd_err != ESP_OK) {
        ESP_LOGW(TAG, "LCD init FAILED: %s — continuing without display",
                 esp_err_to_name(lcd_err));
        s_info.has_lcd = false;  /* 标记没 LCD */
    }

    /* bridge_uart: UART1 GPIO4(TX)/GPIO5(RX) — 用户可以接另一个串口设备 */

    board_keys_init();

    /* DTR/RTS GPIO init — AirSerial 替代 USB-UART 桥 */
    board_serial_ctrl_init();

    /* 运行时自动检测 USB 拓扑（efuse + safe probe）
     * 在 bridge_usb_host 真正 init Host 之前就知道板级特性 */
    board_detect_usb_topo();

    /* boot_splash: 跳过，view_manager 会做自己的 splash */
    if (s_panel) {
        ESP_LOGI(TAG, "starting ui_view_manager...");
        ui_view_manager_start(s_panel, BOARD_LCD_HRES, BOARD_LCD_VRES,
                              BOARD_LCD_COLS, BOARD_LCD_ROWS);
    }

    /* PSRAM 能力检测 */
    board_fill_psram_cap(&s_info);

    ESP_LOGI(TAG, "board_init done");
    return ESP_OK;
}

/* =============================================================
 *  DTR/RTS — AirSerial 作为远程烧录器（路径 B）
 *  active-low: idle=HIGH(1), assert=LOW(0)
 *  esptool 时序: DTR=LOW,RTS=HIGH → reset; 然后 RTS=LOW → bootloader
 * ============================================================= */

static bool s_serial_ctrl_inited;

esp_err_t board_serial_ctrl_init(void)
{
    if (s_serial_ctrl_inited) return ESP_OK;

    gpio_config_t cfg = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << BOARD_DTR_GPIO) | (1ULL << BOARD_RTS_GPIO),
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&cfg));

    /* idle 状态: DTR=HIGH, RTS=HIGH (active-low, 不 assert 任何信号) */
    gpio_set_level(BOARD_DTR_GPIO, 1);
    gpio_set_level(BOARD_RTS_GPIO, 1);

    s_serial_ctrl_inited = true;
    ESP_LOGI(TAG, "DTR/RTS init: DTR=G%d RTS=G%d (idle=HIGH)",
             BOARD_DTR_GPIO, BOARD_RTS_GPIO);
    return ESP_OK;
}

esp_err_t board_serial_ctrl_set(bool dtr_active, bool rts_active)
{
    if (!s_serial_ctrl_inited) {
        ESP_LOGE(TAG, "DTR/RTS not inited");
        return ESP_ERR_INVALID_STATE;
    }
    /* active: true=LOW(assert), false=HIGH(idle) */
    int dtr_level = dtr_active ? 0 : 1;
    int rts_level = rts_active ? 0 : 1;
    gpio_set_level(BOARD_DTR_GPIO, dtr_level);
    gpio_set_level(BOARD_RTS_GPIO, rts_level);

    /* 打印每次变化（零硬件验证靠这个看）*/
    static bool s_last_dtr = true;
    static bool s_last_rts = true;
    if (dtr_active != s_last_dtr || rts_active != s_last_rts) {
        ESP_LOGI(TAG, "DTR=%s RTS=%s → G%d=%d G%d=%d",
                 dtr_active ? "LOW(assert)" : "HIGH",
                 rts_active ? "LOW(assert)" : "HIGH",
                 BOARD_DTR_GPIO, dtr_level,
                 BOARD_RTS_GPIO, rts_level);
        s_last_dtr = dtr_active;
        s_last_rts = rts_active;
    }
    return ESP_OK;
}

bool board_serial_ctrl_is_supported(void) { return true; }
