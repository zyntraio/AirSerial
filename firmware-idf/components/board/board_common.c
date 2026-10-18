/*
 * board_common.c — 板级通用实现（不依赖具体板子）
 *
 * board_can_ai() 门槛: Octal PSRAM ≥ 8MB
 *   ✅ CoreS3:   16MB Flash + 8MB Octal PSRAM → 过
 *   ✅ AtomS3R:  8MB Flash + 8MB Octal PSRAM → 过
 *   ❌ StickS3:  8MB Flash + 8MB Quad PSRAM  → Quad 带宽不够
 *   ❌ Cardputer-Adv: Stamp-S3A 无 PSRAM    → 内部 RAM 不够
 *   ❌ N16R8:    16MB Flash + 8MB Octal PSRAM → 过（等 PSRAM 开）
 */
#include "board.h"
#include "esp_psram.h"
#include "esp_log.h"
#include "sdkconfig.h"

#include <string.h>

#define TAG "board_common"

/* — PSRAM 硬件能力检测 —
 * 编译时靠 sdkconfig 判断 Quad/Octal 模式，
 * 运行时 esp_psram_get_size() 拿实际大小（如果 PSRAM 关着就返回 0）*/
void board_fill_psram_cap(board_info_t *info)
{
    if (!info) return;

    /* 运行时大小 — PSRAM 关着 (CONFIG_SPIRAM=n) → 返回 0 */
#if CONFIG_SPIRAM
    info->psram_size = esp_psram_get_size();
#else
    info->psram_size = 0;
#endif

    /* 编译时模式 — 哪个在 sdkconfig.defaults 里开了 */
#if defined(CONFIG_SPIRAM_MODE_OCT)
    info->psram_octal = true;
#elif defined(CONFIG_SPIRAM_MODE_QUAD)
    info->psram_octal = false;
#else
    info->psram_octal = false;   /* PSRAM 关着也不影响，psram_size=0 挡在前面 */
#endif

    info->has_psram = (info->psram_size > 0);

    ESP_LOGI(TAG, "PSRAM cap: size=%zu KB, mode=%s",
             info->psram_size / 1024,
             info->psram_octal ? "Octal" :
             (info->psram_size > 0 ? "Quad" : "none"));
}

/* — 便利查询 — 三个 board 文件的实现都是一样的，放这省得重复 — */
bool board_has_psram(void)     { return board_get_info()->has_psram; }
bool board_has_display(void)   { return board_get_info()->has_lcd; }
bool board_has_keyboard(void)  { return board_get_info()->has_kbd; }
board_tier_t board_get_tier(void) { return board_get_info()->tier; }

/* — USB Host 板级准备 默认空实现（弱符号）—
 * PowerHub 在 board_powerhub.c 覆写：开 PC_USB + 切 USB Switch 到正面口 */
__attribute__((weak)) esp_err_t board_usb_host_prepare(void)
{
    return ESP_OK;
}

/* — A/C 口自动回退 默认不支持（弱符号）— */
__attribute__((weak)) bool board_usb_host_switch_alt(void)
{
    return false;
}

/* — 正面 USB 口电压/电流监测 默认不支持（弱符号）— */
__attribute__((weak)) bool board_powerhub_vmusb_get(int16_t *ma, uint16_t *mv)
{
    (void)ma; (void)mv;
    return false;
}

/* — 倒灌保护触发状态 默认否（弱符号）— */
__attribute__((weak)) bool board_powerhub_backfeed_tripped(void)
{
    return false;
}

/* — WS2812 状态灯驱动原语 默认空（弱符号）— */
__attribute__((weak)) void board_led_raw(uint8_t idx, uint8_t r, uint8_t g,
                                         uint8_t b, uint8_t bri)
{
    (void)idx; (void)r; (void)g; (void)b; (void)bri;
}

/* — 电源状态查询 默认不支持（弱符号）— */
__attribute__((weak)) bool board_powerhub_pwr_get(uint16_t *bat_mv, int *pct,
                                                  uint8_t *chg, uint8_t *ext)
{
    (void)bat_mv; (void)pct; (void)chg; (void)ext;
    return false;
}

/* — USB 路由模式 默认 0（弱符号）— */
__attribute__((weak)) uint8_t board_powerhub_usb_mode_get(void)
{
    return 0;
}

/* — AI 能力判断 — */
bool board_can_ai(void)
{
    const board_info_t *bi = board_get_info();
    /* Octal + 至少 8MB */
    return bi->psram_octal && bi->psram_size >= (8 * 1024 * 1024);
}

bool board_can_ai_detail(char **reason)
{
    const board_info_t *bi = board_get_info();
    if (bi->psram_size == 0) {
        if (reason) *reason = (char *)"无 PSRAM (FN8/Stamp-S3A 等)";
        return false;
    }
    if (!bi->psram_octal) {
        if (reason) *reason = (char *)"Quad PSRAM 带宽不够（卡死）";
        return false;
    }
    if (bi->psram_size < (8 * 1024 * 1024)) {
        if (reason) *reason = (char *)"Octal PSRAM < 8MB 太小";
        return false;
    }
    if (reason) *reason = (char *)"OK";
    return true;
}
