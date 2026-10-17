/*
 * ui_view_manager — StickS3 LCD 界面管理
 *
 * 单一 UI task 拥有 LCD panel handle，从 event queue 读事件：
 *   - 按键事件（KEY2 切换 view）
 *   - 串口数据（TERMINAL view 显示）
 *   - 定时刷新（STATUS view 1.5s）
 *
 * Views:
 *   VIEW_BOOT       — splash（现在只画几行纯色）
 *   VIEW_STATUS     — WiFi IP / USB Host / 连接数 / 版本号
 *   VIEW_TERMINAL   — 串口数据显示（行缓冲，自动滚动）
 */
#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    VIEW_BOOT     = 0,
    VIEW_STATUS   = 1,
    VIEW_TERMINAL = 2,
    VIEW_COUNT    = 3,
} ui_view_t;

/* — 初始化并启动 UI task —
 * 参数: panel   已初始化的 ST7789 panel handle
 *       hres    可见区域宽（像素，StickS3=135）
 *       vres    可见区域高（像素，StickS3=240）
 *       cols    8x16 字体下的字符列数（StickS3=16）
 *       rows    8x16 字体下的字符行数（StickS3=15）
 *       rows_per_char  每行像素数（=16, 8x16 字体）*/
esp_err_t ui_view_manager_start(void *panel, int hres, int vres, int cols, int rows);

/* — 切换 view — 线程安全，任何 task 可调用 */
void ui_view_manager_switch(ui_view_t view);

/* — 推串口数据到 TERMINAL view —
 * bridge_usb_host 的 cdc_data_cb 或 bridge_serial_on_rx 里调
 * 内部 ring buffer，丢最老数据不丢新数据 */
void ui_view_manager_push_serial(const uint8_t *data, int len);

/* — STATUS view 内容更新 —
 * WiFi STA IP / USB Host 状态 / 连接数 */
typedef enum {
    UI_WIFI_AP,              /* AP 模式 (192.168.4.1) */
    UI_WIFI_STA_CONNECTED,   /* STA 已连 */
    UI_WIFI_STA_DISCONNECTED,/* STA 未连 + AP 未启 */
} ui_wifi_mode_t;

void ui_view_manager_set_wifi(ui_wifi_mode_t mode, const char *ip);
void ui_view_manager_set_host_state(bool on, int cdc);
void ui_view_manager_set_conn_count(int tcp_count, int rfc2217_count);

/* — 获取当前 view（调试用）— */
ui_view_t ui_view_manager_current(void);

#ifdef __cplusplus
}
#endif
