/*
 * ui_view_manager — StickS3 240x135 横屏 UI
 *
 * 双字体: AsciiFont8x16 (大字体, STATUS) + Adafruit 5x7 (小字体, TERMINAL)
 * 单一 UI task 独占 LCD，整块 framebuffer → 单次 draw_bitmap
 */

#include "ui_view_manager.h"
#include "esp_lcd_panel_ops.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_psram.h"
#include "esp_heap_caps.h"
#include "bridge_uart.h"
#include "bridge_usb_host.h"
#include "bridge_tcp.h"
#include "bridge_rfc2217.h"
#include "app_version.h"
#include "esp_mac.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"

#include <string.h>
#include <stdio.h>

/* — 8x16 字体（M5GFX Ascii8x16.h, PROGMEM）— */
#define PROGMEM
#include "Ascii8x16.h"

/* — 5x7 字体（M5GFX glcdfont.h, 256 字符 0x00-0xFF, 列优先, PROGMEM）— */
#include "glcdfont.h"

#define TAG "ui_view"

/* — 屏幕参数（横屏 240×135）— */
static int  s_hres = 240;
static int  s_vres = 135;
static void *s_panel = NULL;

/* — 全局状态 — */
typedef struct {
    ui_wifi_mode_t wifi_mode;   /* AP / STA_CONNECTED / STA_DISCONNECTED */
    char  wifi_ip[16];          /* STA 连上时的 IP, AP 时显示 192.168.4.1 */
    bool  host_on;              /* USB Host 开关 */
    int   cdc_count;            /* USB Host 枚举到的 CDC 设备数 */
    int   tcp_conns;            /* TCP 客户端连接数 */
    int   rfc2217_conns;        /* RFC2217 客户端连接数 */
    uint32_t uart_baud;         /* 当前 UART 波特率 */
    int   psram_free_kb;        /* PSRAM 剩余 KB（无 PSRAM = -1）*/
} s_status_t;

static s_status_t s_st = { .wifi_mode = UI_WIFI_STA_DISCONNECTED,
                           .wifi_ip = "-", .host_on = false, .cdc_count = 0,
                           .tcp_conns = 0, .rfc2217_conns = 0 };

/* — TERMINAL view 行缓冲（小字体 5x7: 40 chars/line × 16 lines）— */
#define TERM_MAX_ROWS 64
#define TERM_MAX_COLS 40
static char s_term_buf[TERM_MAX_ROWS][TERM_MAX_COLS + 1];
static int  s_term_head = 0;
static int  s_term_used = 0;
static int  s_term_col  = 0;

/* — lock-free 串口 ring buffer — */
#define SERIAL_RING_SIZE 2048
static uint8_t  s_serial_ring[SERIAL_RING_SIZE];
static volatile int s_ring_wr = 0;
static volatile int s_ring_rd = 0;

static inline int ring_writable(void) {
    int w = s_ring_wr, r = s_ring_rd;
    if (w >= r) return SERIAL_RING_SIZE - (w - r) - 1;
    return r - w - 1;
}
static inline int ring_readable(void) {
    int w = s_ring_wr, r = s_ring_rd;
    if (w >= r) return w - r;
    return SERIAL_RING_SIZE - r + w;
}
static void ring_write(const uint8_t *data, int len) {
    if (ring_writable() < len) {
        int drop = len - ring_writable();
        s_ring_rd = (s_ring_rd + drop) % SERIAL_RING_SIZE;
    }
    for (int i = 0; i < len; i++) {
        s_serial_ring[s_ring_wr] = data[i];
        s_ring_wr = (s_ring_wr + 1) % SERIAL_RING_SIZE;
    }
}
static int ring_read(uint8_t *dst, int max) {
    int n = ring_readable();
    if (n > max) n = max;
    for (int i = 0; i < n; i++) {
        dst[i] = s_serial_ring[s_ring_rd];
        s_ring_rd = (s_ring_rd + 1) % SERIAL_RING_SIZE;
    }
    return n;
}

/* — FreeRTOS — */
typedef enum { EVT_KEY, EVT_REFRESH } ui_evt_t;
static QueueHandle_t s_evt_queue;
static TimerHandle_t s_status_timer;
static TaskHandle_t  s_ui_task;
static volatile ui_view_t s_current_view = VIEW_BOOT;

typedef struct { ui_evt_t type; int key_gpio; } ui_event_msg_t;

static void IRAM_ATTR key_isr_handler(void *arg) {
    int gpio_num = (int)(intptr_t)arg;
    ui_event_msg_t msg = { .type = EVT_KEY, .key_gpio = gpio_num };
    BaseType_t high = pdFALSE;
    xQueueSendFromISR(s_evt_queue, &msg, &high);
    if (high) portYIELD_FROM_ISR();
}

/* — RGB565 颜色 — */
#define CGA_BG          0x0000
#define CGA_FG_NORMAL   0x03E0
#define CGA_FG_OK       0x07E0   /* 纯绿, M5Launcher 标准 */
#define CGA_FG_ERROR    0xF800
#define CGA_FG_WARN     0xFFE0
#define CGA_FG_HUD      0x07FF
#define CGA_FG_DIM      0x7BEF
#define CGA_FG_TITLE    0xFFFF

/* — Framebuffer — */
static uint16_t s_fb[240 * 135];

/* — ========= 绘制原语 ========= — */
static inline void set_px(int x, int y, uint16_t color) {
    if ((unsigned)x < (unsigned)s_hres && (unsigned)y < (unsigned)s_vres)
        s_fb[y * s_hres + x] = color;
}

static void clear_fb(uint16_t color) {
    for (int i = 0; i < s_hres * s_vres; i++) s_fb[i] = color;
}

static void draw_hline(int y, uint16_t color) {
    if ((unsigned)y >= (unsigned)s_vres) return;
    for (int x = 0; x < s_hres; x++) s_fb[y * s_hres + x] = color;
}

static void flush_screen(void) {
    if (!s_panel) return;
    esp_lcd_panel_draw_bitmap((esp_lcd_panel_handle_t)s_panel,
                              0, 0, s_hres, s_vres, (uint8_t *)s_fb);
}

/* — ========= 8x16 大字体绘制（STATUS view 用）—
 * FontLib8x16: 256 字符, 16 字节/字符
 * 字符索引 = char code (0x00-0xFF) — 0x20=space 在 index 32
 * 行优先: glyph[row], bit7=左, bit0=右
 * 8px 宽 × 16px 高 — 240/8 = 30 列, 135/16 = 8 行
 * 为了更好看，每行加 2px gap — 实际用 8px char + 2px gap = 10px/char
 * 240/10 = 24 列，正好整数！ */
#define F8X16_W  8
#define F8X16_H  16
#define F8X16_GAP 2   /* 字符间距 */
#define F8X16_PITCH (F8X16_W + F8X16_GAP)  /* 10px per char */
#define F8X16_COLS 24 /* 240 / 10 */
#define F8X16_ROWS 8  /* 135 / 16 = 8.4 → 8行，底部留 7px */

static void draw_char_8x16(int col, int row, char c, uint16_t color) {
    if (col < 0 || col >= F8X16_COLS) return;
    if (row < 0 || row >= F8X16_ROWS) return;

    /* FontLib8x16 直接用 char code 做 index */
    const uint8_t *glyph = &FontLib8x16[(uint8_t)c * F8X16_H];

    int x_start = col * F8X16_PITCH;
    int y_start = row * F8X16_H;

    for (int r = 0; r < F8X16_H; r++) {
        uint8_t row_data = glyph[r];
        for (int bit = 0; bit < F8X16_W; bit++) {
            /* bit7 = leftmost pixel */
            if (row_data & (0x80 >> bit)) {
                set_px(x_start + bit, y_start + r, color);
            }
        }
    }
}

static void draw_string_8x16(int col, int row, const char *str, uint16_t color) {
    while (*str && col < F8X16_COLS) {
        draw_char_8x16(col, row, *str++, color);
        col++;
    }
}

/* — ========= 5x7 小字体绘制（TERMINAL view 用）—
 * M5GFX glcdfont.h: 256 字符 (0x00-0xFF), 5 字节/字符，列优先
 * 索引: (uint8_t)c * 5, bit0=top (Adafruit 标准)
 * 5px 宽 + 1px gap = 6px, 8px 行高
 * 240/6 = 40 列, 135/8 = 16 行 */
#define F5X7_W  5
#define F5X7_H  8
#define F5X7_PITCH 6   /* 5+1 */
#define F5X7_COLS 40   /* 240/6 */
#define F5X7_ROWS 16   /* 135/8 = 16.8 → 16 */

static void draw_char_5x7(int col, int row, char c, uint16_t color) {
    if (col < 0 || col >= F5X7_COLS) return;
    /* row 边界放宽，set_px 会 clip 超出屏幕的像素 */
    if (row < 0) return;

    /* M5GFX glcdfont.h: 256 字符从 0x00 开始, 数组名 font[] */
    const uint8_t *glyph = &font[(uint8_t)c * 5];
    int x_start = col * F5X7_PITCH;
    int y_start = row * F5X7_H;

    for (int col_idx = 0; col_idx < F5X7_W; col_idx++) {
        uint8_t col_data = glyph[col_idx];
        for (int row_idx = 0; row_idx < F5X7_H; row_idx++) {
            /* Adafruit glcdfont: 列优先, bit0=top */
            if (col_data & (0x01 << row_idx)) {
                set_px(x_start + col_idx, y_start + row_idx, color);
            }
        }
    }
}

static void draw_string_5x7(int col, int row, const char *str, uint16_t color) {
    while (*str && col < F5X7_COLS) {
        draw_char_5x7(col, row, *str++, color);
        col++;
    }
}

/* — ========= STATUS view ========= —
 * 布局 (240×135 横屏):
 *   row 0-3: 8x16 大字 (WiFi/Host/TCP 状态)
 *   底部 1 行: 5x7 小字提示 ([KEY2]=Terminal)
 *   中间自动留白 */
static void render_status_view(void) {
    clear_fb(CGA_BG);

    /* 标题 + 状态行: 8x16 大字。标题 = AirSerial-<MAC 后四位>，多机同网一眼区分 */
    uint8_t s_mac[6];
    esp_read_mac(s_mac, ESP_MAC_WIFI_STA);
    char title[24];
    snprintf(title, sizeof(title), "AirSerial-%02X%02X", s_mac[4], s_mac[5]);
    int title_col = (F8X16_COLS - (int)strlen(title)) / 2;
    if (title_col < 0) title_col = 0;
    draw_string_8x16(title_col, 0, title, CGA_FG_TITLE);

    char line[32];
    uint16_t wifi_color;
    const char *wifi_label;
    switch (s_st.wifi_mode) {
    case UI_WIFI_AP:
        wifi_color = CGA_FG_WARN;     /* 黄色 */
        wifi_label = "AP: 192.168.4.1";
        break;
    case UI_WIFI_STA_CONNECTED:
        wifi_color = CGA_FG_OK;       /* 绿色 */
        snprintf(line, sizeof(line), "IP: %s", s_st.wifi_ip);
        wifi_label = line;
        break;
    default: /* UI_WIFI_STA_DISCONNECTED */
        wifi_color = CGA_FG_ERROR;    /* 红色 */
        wifi_label = "IP: ---.---.---.---";
        break;
    }
    draw_string_8x16(0, 1, wifi_label, wifi_color);

    snprintf(line, sizeof(line), "Host: %s CDC:%d",
             s_st.host_on ? "ON " : "OFF", s_st.cdc_count);
    draw_string_8x16(0, 2, line, s_st.host_on ? CGA_FG_OK : CGA_FG_ERROR);

    snprintf(line, sizeof(line), "TCP:%d RFC2217:%d",
             s_st.tcp_conns, s_st.rfc2217_conns);
    draw_string_8x16(0, 3, line, CGA_FG_WARN);

    /* 波特率 + PSRAM free 合并为大字 row 4 */
    if (s_st.psram_free_kb >= 0) {
        snprintf(line, sizeof(line), "%lubd PSRAM:%dK",
                 (unsigned long)s_st.uart_baud, s_st.psram_free_kb);
    } else {
        snprintf(line, sizeof(line), "%lubd PSRAM:none",
                 (unsigned long)s_st.uart_baud);
    }
    draw_string_8x16(0, 4, line, CGA_FG_HUD);

    /* 底部提示: 5x7 小字, 版本号在前, 落底 (row 16 → y=128~135, 刚好贴边) */
    const char *hint = AIRSERIAL_VERSION "   [KEY2]=Terminal";
    int hint_len = (int)strlen(hint);
    int hint_col = (F5X7_COLS - hint_len) / 2;
    if (hint_col < 0) hint_col = 0;
    int hint_row = F5X7_ROWS;   /* row 16, y=128, 落底 */
    draw_string_5x7(hint_col, hint_row, hint, CGA_FG_DIM);

    flush_screen();
}

/* — ========= TERMINAL view（小字体密排）— */
static void render_terminal_view(void) {
    clear_fb(CGA_BG);

    /* 标题条 */
    const char *title = "=== Terminal [KEY2]=Status ===";
    /* 用 5x7 字体写标题，240px 宽正好 40 字符 */
    for (int c = 0; title[c] && c < F5X7_COLS; c++)
        draw_char_5x7(c, 0, title[c], CGA_FG_HUD);

    /* 分隔 */
    for (int y = F5X7_H; y < F5X7_H + 1; y++) draw_hline(y, CGA_FG_HUD);

    if (s_term_used == 0) {
        const char *wait = "(waiting for serial data...)";
        draw_string_5x7(0, 2, wait, CGA_FG_DIM);
    } else {
        /* 显示最后 F5X7_ROWS - 2 = 14 行历史 */
        int display_rows = F5X7_ROWS - 2;
        int start = s_term_head - display_rows + 1;
        if (start < 0) start += TERM_MAX_ROWS;
        for (int i = 0; i < display_rows && i < s_term_used; i++) {
            int buf_idx = (start + i) % TERM_MAX_ROWS;
            draw_string_5x7(0, 2 + i, s_term_buf[buf_idx], CGA_FG_OK);
        }
    }

    flush_screen();
}

/* — ========= 主 UI task ========= — */
static void ui_task(void *arg) {
    ui_view_manager_switch(VIEW_BOOT);

    /* === Boot splash：ROADMAP 完整版 AirSerial ASCII art (5x7) === */
    ESP_LOGI(TAG, "Boot splash landscape 240x135...");
    clear_fb(CGA_BG);

    /* ROADMAP §7.3 定稿版: 去掉虚线分隔, 名字一体 */
    static const char *s_logo_lines[] = {
        "   _   _       ",
        "  /_\\ (_)_ _  ",
        " / _ \\| | '_| ",
        "/_/ \\_\\_|_|  ",
        " ___          _      _",
        "/ __| ___ _ _(_)__ _| |",
        "\\__ \\/ -_) '_| / _` | |",
        "|___/\\___|_| |_\\__,_|_|",
    };
    #define LOGO_LINES 8
    #define LOGO_COLS  30

    /* 5x7: pitch=6px, 30×6=180px, 240 居中 → col 5 */
    int logo_col = (F5X7_COLS - LOGO_COLS) / 2;
    /* 9 行 × 8px = 72px, 135 居中 → row 3 (y=24) */
    int logo_row = 3;
    for (int i = 0; i < LOGO_LINES; i++) {
        draw_string_5x7(logo_col, logo_row + i, s_logo_lines[i], CGA_FG_OK);
    }

    flush_screen();
    vTaskDelay(pdMS_TO_TICKS(2000));

    /* 切到 STATUS view + 启动 timer */
    ui_view_manager_switch(VIEW_STATUS);
    xTimerStart(s_status_timer, 0);

    /* === 主循环 === */
    ui_event_msg_t msg;
    while (true) {
        if (xQueueReceive(s_evt_queue, &msg, pdMS_TO_TICKS(100))) {
            if (msg.type == EVT_KEY) {
                vTaskDelay(pdMS_TO_TICKS(20));
                if (gpio_get_level(msg.key_gpio) == 0) {
                    ESP_LOGI(TAG, "KEY GPIO%d pressed", msg.key_gpio);
                    if (msg.key_gpio == 12) {
                        ui_view_t next = (s_current_view == VIEW_STATUS) ? VIEW_TERMINAL : VIEW_STATUS;
                        ui_view_manager_switch(next);
                    }
                }
            } else if (msg.type == EVT_REFRESH && s_current_view == VIEW_STATUS) {
                render_status_view();
            }
        }

        /* TERMINAL: 消费 ring buffer → 写入 line buffer → 重绘 */
        if (s_current_view == VIEW_TERMINAL && ring_readable() > 0) {
            uint8_t tmp[128];
            int n = ring_read(tmp, sizeof(tmp));
            bool need_redraw = false;
            for (int i = 0; i < n; i++) {
                uint8_t ch = tmp[i];
                if (ch == '\n' || ch == '\r') {
                    s_term_head = (s_term_head + 1) % TERM_MAX_ROWS;
                    if (s_term_used < TERM_MAX_ROWS) s_term_used++;
                    s_term_col = 0;
                    memset(s_term_buf[s_term_head], 0, TERM_MAX_COLS + 1);
                    need_redraw = true;
                } else if (ch >= 0x20 && s_term_col < TERM_MAX_COLS) {
                    s_term_buf[s_term_head][s_term_col++] = ch;
                    need_redraw = true;
                }
            }
            if (need_redraw) render_terminal_view();
        }
    }
}

static void status_timer_cb(TimerHandle_t timer) {
    /* 轮询拉取波特率 + PSRAM free + 桥接连接状态（1.5s 一次，不频繁）*/
    s_st.uart_baud = bridge_uart_get_baud();
    if (esp_psram_is_initialized()) {
        s_st.psram_free_kb = (int)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024);
    } else {
        s_st.psram_free_kb = -1;
    }
    /* CDC/TCP/RFC2217 连接状态直接轮询权威来源（setter 推送曾接线遗漏，恒显 0）*/
    s_st.cdc_count      = bridge_usb_host_is_open() ? 1 : 0;
    s_st.tcp_conns      = bridge_tcp_has_client() ? 1 : 0;
    s_st.rfc2217_conns  = bridge_rfc2217_has_client() ? 1 : 0;

    ui_event_msg_t msg = { .type = EVT_REFRESH };
    xQueueSend(s_evt_queue, &msg, 0);
}

/* ============ 公共 API ============ */

esp_err_t ui_view_manager_start(void *panel, int hres, int vres, int cols, int rows) {
    s_panel = panel;
    s_hres = 240;   /* 强制横屏 */
    s_vres = 135;
    ESP_LOGI(TAG, "Landscape %dx%d — 8x16: %dx%d chars, 5x7: %dx%d chars",
             s_hres, s_vres, F8X16_COLS, F8X16_ROWS, F5X7_COLS, F5X7_ROWS);

    memset(s_term_buf, 0, sizeof(s_term_buf));

    s_evt_queue = xQueueCreate(8, sizeof(ui_event_msg_t));
    if (!s_evt_queue) return ESP_ERR_NO_MEM;

    gpio_config_t key_cfg = {
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_ANYEDGE,  /* 必须开中断！board_keys_init 会 DISABLE */
        .pin_bit_mask = (1ULL << GPIO_NUM_11) | (1ULL << GPIO_NUM_12),
    };
    ESP_ERROR_CHECK(gpio_config(&key_cfg));
    gpio_install_isr_service(0);
    gpio_isr_handler_add(GPIO_NUM_11, key_isr_handler, (void *)(intptr_t)11);
    gpio_isr_handler_add(GPIO_NUM_12, key_isr_handler, (void *)(intptr_t)12);
    ESP_LOGI(TAG, "GPIO11/12 ISR installed (ANYEDGE)");

    s_status_timer = xTimerCreate("ui_status", pdMS_TO_TICKS(1500),
                                  pdTRUE, NULL, status_timer_cb);
    xTaskCreatePinnedToCore(ui_task, "ui_task", 4096, NULL, 4, &s_ui_task, 1);
    return ESP_OK;
}

void ui_view_manager_switch(ui_view_t view) {
    if (s_current_view == view) return;
    ESP_LOGI(TAG, "View: %d → %d", s_current_view, view);
    s_current_view = view;
    if (view == VIEW_STATUS) render_status_view();
    else if (view == VIEW_TERMINAL) render_terminal_view();
}

ui_view_t ui_view_manager_current(void) { return s_current_view; }

void ui_view_manager_push_serial(const uint8_t *data, int len) {
    if (!data || len > 0) ring_write(data, len);
}

void ui_view_manager_set_wifi(ui_wifi_mode_t mode, const char *ip) {
    s_st.wifi_mode = mode;
    if (ip && ip[0]) snprintf(s_st.wifi_ip, sizeof(s_st.wifi_ip), "%s", ip);
    else             snprintf(s_st.wifi_ip, sizeof(s_st.wifi_ip), "-");
}

void ui_view_manager_set_host_state(bool on, int cdc) {
    s_st.host_on = on; s_st.cdc_count = cdc;
}

void ui_view_manager_set_conn_count(int tcp, int rfc) {
    s_st.tcp_conns = tcp; s_st.rfc2217_conns = rfc;
}
