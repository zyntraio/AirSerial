/*
 * board — 板级公共头（一板一文件，board_xxx.c 实现）
 *
 * 分级:
 *   BOARD_TIER_3_STATUS  — 有屏无键盘（StickS3, StickC, CoreC）→ 状态页
 *   BOARD_TIER_2_HUD     — 大屏+有 PSRAM（CoreS3）→ HUD + 终端
 *   BOARD_TIER_1_TERMINAL — 有屏有键盘（Cardputer ADV）→ 全屏终端
 *   BOARD_TIER_0_HEADLESS — 无屏无键盘（PowerHub）→ 跳过 UI
 */
#pragma once

#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    BOARD_TIER_0_HEADLESS   = 0,
    BOARD_TIER_3_STATUS     = 3,   /* StickS3, StickC */
    BOARD_TIER_2_HUD        = 2,   /* CoreS3 */
    BOARD_TIER_1_TERMINAL   = 1,   /* Cardputer ADV */
} board_tier_t;

typedef struct {
    const char   *name;
    const char   *target;     /* "esp32s3", "esp32p4", ... */
    bool          has_lcd;
    bool          has_kbd;
    bool          has_psram;
    int           cols;       /* 8x16 字体下每行列数 */
    int           rows;       /* 8x16 字体下屏幕行数 */
    board_tier_t  tier;
    /* PSRAM 硬件能力（board_init 运行时填充）—
     * 用来给 AI 功能、大 framebuffer 等做降级判断 */
    size_t        psram_size;    /* 0 = 无 PSRAM */
    bool          psram_octal;   /* true = Octal, false = Quad 或无 */
} board_info_t;

/* — USB 拓扑 —
 * ESP32-S3 芯片有两路 USB：USB-OTG (GPIO19/20) + USB-SERIAL-JTAG (GPIO19/20) 共享 PHY
 * 但板级连接差异大：
 *
 *   USB_TOPO_INDEPENDENT   — 独立 UART0 (CH340/CH343 等外接串口芯片硬连 GPIO43/44)
 *                            + 独立直通 OTG (Type-C → GPIO19/20)
 *                            代表: 公模板子 N16R8
 *                            → Host 模式下 CH340 console 活！
 *
 *   USB_TOPO_SHARED_PHY    — 单 Type-C 口，USB-SERIAL-JTAG + OTG 共享 GPIO19/20
 *                            GPIO43/44 板上没连（或只焊了 CH340 但没拉出来）
 *                            代表: M5StickS3
 *                            → Host 模式下 USB-SERIAL-JTAG console 死！
 *
 *   USB_TOPO_NO_HOST       — 只有 USB-SERIAL-JTAG 没有独立 OTG
 *                            代表: 某些最小开发板
 *                            → 完全不能跑 USB Host
 */
typedef enum {
    USB_TOPO_UNKNOWN = 0,
    USB_TOPO_INDEPENDENT,
    USB_TOPO_SHARED_PHY,
    USB_TOPO_NO_HOST,
} board_usb_topo_t;

/* — 板级硬件信息（board_xxx.c 里填）— */
const board_info_t *board_get_info(void);
board_usb_topo_t    board_get_usb_topo(void);   /* 板级声明的拓扑 */
bool                board_usb_host_init_safe(void); /* USB Host init 后 console 会不会断？INDEPENDENT=true */

/* — main 调这个一次性 init — */
esp_err_t board_init(void);

/* — 板级 PSRAM 能力检测 — board_init 末尾自动调，填充 board_info_t.psram_size/psram_octal */
void board_fill_psram_cap(board_info_t *info);

/* — LCD panel handle（ui_view_manager 等组件用）—
 * 返回 esp_lcd_panel_handle_t，可能是 NULL（LCD init 失败时） */
void *board_get_lcd_panel(void);
int   board_get_lcd_hres(void);
int   board_get_lcd_vres(void);

/* — 运行时 USB 拓扑自动检测 —
 * 组合 efuse 芯片能力 + 启动 probe（安全 USB Host init/uninstall 不碰 PHY）
 * 如果和板级声明一致 → 用板级声明；否则 → 用检测结果 + 打 W 警告
 */
void board_detect_usb_topo(void);

/* — 给其他组件用的便利查询 — */
bool board_has_psram(void);
bool board_has_display(void);
bool board_has_keyboard(void);
board_tier_t board_get_tier(void);

/* — AirSerial 替代 USB-UART 桥的 DTR/RTS 带外控制 —
 *   远程烧录路径 B：AirSerial GPIO 直接接目标板 RST/BOOT
 *   active-low（跟标准 UART MODEM 信号一致）
 *   不支持的板 stub 返回 ESP_ERR_NOT_SUPPORTED */
esp_err_t board_serial_ctrl_init(void);
esp_err_t board_serial_ctrl_set(bool dtr_active, bool rts_active);
bool      board_serial_ctrl_is_supported(void);

/* — 硬件能力判断 — */
/* AI 功能（小智 AI 语音识别/LLM 推理）门槛：
 *   ✅ Octal PSRAM ≥ 8MB — 够放大模型 + 帧缓冲
 *   ❌ Quad PSRAM (任何大小) — 带宽不够，会卡得要死
 *   ❌ 无 PSRAM (FN8) — 内部 RAM 不够
 *   ❌ Octal PSRAM < 8MB — 太小 */
bool board_can_ai(void);
bool board_can_ai_detail(char **reason);   /* 可选，返回为什么不行 */

#ifdef __cplusplus
}
#endif
