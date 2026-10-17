/*
 * ui_cga — 裸 esp_lcd 终端风格 UI（Color Graphics Adapter）
 *
 * 不依赖 LVGL，直接操作 panel bitmap buffer。
 * 字体: 8x16 ASCII（flash 常量数组）
 * 颜色: 暗绿 0x03E0 主文本 / 亮绿 0x07E0 OK / 黑 0x0000 背景
 */
#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* — panel 回调: 画一行到屏幕 y 位置 — */
typedef esp_err_t (*cga_draw_row_fn)(int row, const uint8_t *pixels, int width_bytes);

/* — 初始化 — */
void ui_cga_set_panel_callbacks(cga_draw_row_fn draw_row, int hres);

/* — boot_splash — */
void ui_cga_boot_splash(int cols);

/* — Tier 3 状态页 — */
void ui_cga_status_page_start(int cols, int rows);

/* — 状态页热更新 — */
void ui_cga_status_update(const char *line1, const char *line2, const char *line3);

#ifdef __cplusplus
}
#endif
