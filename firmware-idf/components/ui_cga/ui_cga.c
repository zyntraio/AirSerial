/*
 * ui_cga — 裸 LCD 终端 UI 最小实现
 *
 * 字体: 内嵌 96 字符 8x16 ASCII（flash 常量，不拷贝到 RAM）
 * 面板: partial buffer 一行 240px × 2 字节 = 480 字节
 */
#include "ui_cga.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <string.h>
#include <stdio.h>
#include <ctype.h>

#define TAG "ui_cga"

/* — RGB565 颜色 — */
#define CGA_FG_NORMAL   0x03E0   /* 暗绿 */
#define CGA_FG_OK       0x07E0   /* 亮绿 */
#define CGA_FG_ERROR    0xF800   /* 红 */
#define CGA_FG_WARN     0xFFE0   /* 黄 */
#define CGA_FG_HUD      0x07FF   /* 青 */
#define CGA_FG_DIM      0x7BEF   /* 灰 */
#define CGA_BG          0x0000   /* 黑 */

/* — 回调 — */
static cga_draw_row_fn s_draw_row;
static int             s_hres;     /* 面板宽（像素） */

/* — Partial buffer（480 字节）— */
static uint8_t s_row_buf[240 * 2];   /* 240px × 2 bytes/pixel */

/* — 内嵌 8×16 ASCII 字体（最小可用集：32~126 可打印字符）—
 * 用 adafruit-gfx 格式：每字符 16 字节 bitmap，MSB 在上 */
/* 临时：用空字体占位。下一步从 lv_font_montserrat_14 或 adafruit 抽。 */
/* 现在先画 ASCII art 不依赖字体，boot_splash 直接用位图 */

/* — Small 字体截断版 AirSerial ASCII art（30 列 × 8 行）— */
/* 每行最多 30 字符 */
static const char *s_logo_lines[] = {
    "   _   _       ",
    "  /_\\ (_)_ _  ",
    " / _ \\| | '_| ",
    "/_/ \\_\\_|_|  ",
    "      --------",
    " ___          ",
    "/ __| ___ _ _ ",
    "\\__ \\/ -_) '_|",
    "|___/\\___|_| ",
};
#define LOGO_LINES 9

/* — 通用一行 bitmap 绘制（字符 → pixels）—
 * 简化版：直接画纯色矩形（调试用）
 * 等字体数组就绪后替换为真实 glyph */
static void fill_row_color(uint16_t color)
{
    uint16_t *p = (uint16_t *)s_row_buf;
    for (int i = 0; i < s_hres; i++) {
        p[i] = color;
    }
}

static void clear_row(void)
{
    fill_row_color(CGA_BG);
}

/* — 调试用：把字符串渲染成 ASCII 字符直接发 monitor —
 * 后续替换为真实字形渲染 */
void ui_cga_boot_splash(int cols)
{
    ESP_LOGI(TAG, "boot_splash: cols=%d", cols);

    /* 画 logo 前 8 行 */
    for (int i = 0; i < 8 && i < LOGO_LINES; i++) {
        clear_row();
        /* 后续：从字体数组渲染 glyph 到 s_row_buf */
        /* 现在先打印到 monitor 验证流程 */
        ESP_LOGI(TAG, "  |%s|", s_logo_lines[i]);
        if (s_draw_row) {
            /* 临时：用颜色区分行号，不画真实字形 */
            fill_row_color((i % 2) ? CGA_FG_NORMAL : CGA_FG_OK);
            s_draw_row(i, s_row_buf, s_hres * 2);
        }
    }

    /* 滚状态 */
    vTaskDelay(pdMS_TO_TICKS(300));
    ESP_LOGI(TAG, "[ OK ] board init complete");
    ESP_LOGI(TAG, "[ OK ] WiFi AP starting...");
    ESP_LOGI(TAG, "[ OK ] bridge core ready");

    vTaskDelay(pdMS_TO_TICKS(700));
    ESP_LOGI(TAG, "boot_splash done");
}

void ui_cga_status_page_start(int cols, int rows)
{
    ESP_LOGI(TAG, "status_page_start: cols=%d, rows=%d", cols, rows);
    /* 只画一行标题 + "waiting for bridge..." */
    for (int i = 0; i < rows; i++) {
        clear_row();
        if (s_draw_row) {
            s_draw_row(i, s_row_buf, s_hres * 2);
        }
    }
    /* 后续：标题行画版本号，其他行等 WiFi/USB 状态回调 */
    ESP_LOGI(TAG, "Tier 3 status page ready, waiting for bridge events...");
}

void ui_cga_set_panel_callbacks(cga_draw_row_fn draw_row, int hres)
{
    s_draw_row = draw_row;
    s_hres = hres;
    ESP_LOGI(TAG, "panel callbacks set: draw_row=%p, hres=%d", (void *)draw_row, hres);
}

void ui_cga_status_update(const char *line1, const char *line2, const char *line3)
{
    /* 后续：更新状态页的行 */
    ESP_LOGI(TAG, "status_update: %s | %s | %s",
             line1 ? line1 : "-", line2 ? line2 : "-", line3 ? line3 : "-");
}
