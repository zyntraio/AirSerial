/*
 * bridge_crash.c —— 崩溃遗言机制实现，见头文件注释。
 */
#include "bridge_crash.h"

#include "esp_attr.h"
#include "esp_system.h"                    /* esp_reset_reason_t + esp_reset_reason() */
#include "esp_private/system_internal.h"  /* esp_reset_reason_set_hint() strong override hook */
#include "esp_timer.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include <string.h>
#include <stdlib.h>

#define TAG "crash"
#define MAGIC 0x43524153  /* "CRAS" */
#define NVS_NS "airsn_crash"

/* RTC_NOINIT_ATTR：放在 RTC 慢速内存，启动代码不会清零。
 * 正常 reset（esp_restart()）、panic、WDT、brownout 后保留；
 * power-on（硬掉电）和 USB 复位不保留——但那两类也没有 panic 遗言。 */
static struct {
    uint32_t magic;          /* MAGIC=有遗言，0=正常启动 */
    esp_reset_reason_t reason;  /* panic.c 传入的 hint，如 ESP_RST_PANIC */
    int64_t  uptime_us;      /* panic 当时的系统 uptime */
    char     abort_reason[64]; /* bridge_crash_abort() 写入的自定义原因 */
} s_crash_info RTC_NOINIT_ATTR;

/* ---- panic-side hook（只有异常重启才调） ----
 *
 * Strong override of the weak esp_reset_reason_set_hint() in IDF panic.c:483.
 * panic handler 打完全部日志、即将 panic_restart() 之前调用，
 * 此时内存安全、RTC 内存尚活，禁止碰 flash/网络/复杂操作。
 * 写完全部字段后 abort_reason 字段保留（可能已被 bridge_crash_abort() 预写）。
 */
void esp_reset_reason_set_hint(esp_reset_reason_t hint)
{
    s_crash_info.magic = MAGIC;
    s_crash_info.reason = hint;
    s_crash_info.uptime_us = esp_timer_get_time();
    /* abort_reason 保留预写入值，此时不能清、不能碰 flash */
}

/* ---- boot-side restore（正常启动路径） ---- */
void bridge_crash_try_load_past(void)
{
    if (s_crash_info.magic != MAGIC) {
        return;   /* 正常启动，上次没有 panic */
    }

    char reason_str[64] = {0};
    switch (s_crash_info.reason) {
    case ESP_RST_PANIC:        strcpy(reason_str, "PANIC"); break;
    case ESP_RST_INT_WDT:      strcpy(reason_str, "INT_WDT"); break;
    case ESP_RST_TASK_WDT:     strcpy(reason_str, "TASK_WDT"); break;
    case ESP_RST_BROWNOUT:     strcpy(reason_str, "BROWNOUT"); break;
    case ESP_RST_SW:           strcpy(reason_str, "SW_RESTART"); break;
    case ESP_RST_DEEPSLEEP:    strcpy(reason_str, "DEEPSLEEP"); break;
    case ESP_RST_POWERON:      strcpy(reason_str, "POWERON"); break;
    case ESP_RST_USB:          strcpy(reason_str, "USB"); break;
    default:                   snprintf(reason_str, sizeof(reason_str), "%d", (int)s_crash_info.reason); break;
    }

    double uptime_s = s_crash_info.uptime_us / 1000000.0;
    ESP_LOGW(TAG, "========================================");
    ESP_LOGW(TAG, " LAST BOOT: PANIC (reason=%s, uptime=%.3fs)", reason_str, uptime_s);
    if (s_crash_info.abort_reason[0]) {
        ESP_LOGW(TAG, " abort detail: %s", s_crash_info.abort_reason);
    }
    ESP_LOGW(TAG, "========================================");

    /* 落 NVS 供远程读取（v0.5.4 HTTP 状态 API）——
     * 只做 set+commit，失败静默（NVS 已 init 但 namespace 可能未建） */
    nvs_handle_t h;
    esp_err_t ne = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (ne == ESP_OK) {
        nvs_set_str(h, "reason", reason_str);
        nvs_set_i64(h, "uptime_us", s_crash_info.uptime_us);
        if (s_crash_info.abort_reason[0]) {
            nvs_set_str(h, "abort", s_crash_info.abort_reason);
        }
        nvs_commit(h);
        nvs_close(h);
    }

    /* 清掉 magic，下次正常启动不会误报；RTC 内存里可能残留其他字段，
     * 但没有 magic 就不会被当成有效遗言 */
    s_crash_info.magic = 0;
}

/* ---- 主动 abort 路径 ---- */
void bridge_crash_abort(const char *reason)
{
    if (reason) {
        strncpy(s_crash_info.abort_reason, reason,
                sizeof(s_crash_info.abort_reason) - 1);
        s_crash_info.abort_reason[sizeof(s_crash_info.abort_reason) - 1] = '\0';
    }
    abort();
}
