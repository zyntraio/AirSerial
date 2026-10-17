/*
 * bridge_wifi —— WiFi 抽象积木（板无关接口）。
 *
 * 后端可切换：Tab5 用 ESP32-C6（esp_hosted over SDIO），
 * 其他板子可能用 C5 或板载 WiFi——main 只调这组接口，
 * 后端差异由本组件内部屏蔽。
 *
 * v0.1.0：骨架占位。
 * v0.1.2：esp_hosted + C6 STA + 扫描验证（TCP echo 待接）。
 */
#pragma once

#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 初始化 WiFi 子系统（后端探测 + 驱动加载）。
 * 成功返回 ESP_OK；后端不存在/未实现返回 ESP_ERR_NOT_SUPPORTED。
 */
esp_err_t bridge_wifi_init(void);

/* 连接 AP；凭证为 NULL 时走 NVS/默认值 */
esp_err_t bridge_wifi_connect(const char *ssid, const char *password);
/* 连接超时上限（ms）；0 用默认 30s。Boot auto-connect 建议 8000 */
esp_err_t bridge_wifi_connect_ex(const char *ssid, const char *password, uint32_t timeout_ms);

/* 扫描周围 AP 并打印（验证 C6 链路的最小手段，无需凭证） */
esp_err_t bridge_wifi_scan(void);

/* AP 扫描结果：精简字段，UI 列表页直接消费 */
typedef struct {
    char     ssid[33];      /* 原始 utf-8，可直接喂给 LVGL label；隐藏 SSID 则为空串 */
    uint8_t  bssid[6];      /* BSSID（MAC），所有扫到的 AP 都有 */
    int8_t   rssi;
    uint8_t  channel;
    bool     open;          /* 无密码 */
    bool     hidden;        /* 隐藏 SSID（ssid[0]=='\0'） */
} bridge_wifi_ap_t;

/*
 * 异步扫描：立即返回 ESP_OK，扫描在独立任务里进行。
 * 结果通过 bridge_wifi_get_scan_results() 读取（扫描完成前返回空）。
 * UI 层用 LVGL timer 轮询即可，不需要回调跨线程。
 */
esp_err_t bridge_wifi_scan_async(void);

/*
 * 读取最近一次异步扫描结果。
 * @param out    调用方提供的数组（建议 >= MAX 条目）
 * @param out_cap  out 数组容量
 * @return        实际写入的条目数；0 表示扫描还没完成或没结果
 */
int bridge_wifi_get_scan_results(bridge_wifi_ap_t *out, int out_cap);

/* 异步扫描是否仍在进行（UI 用它区分"扫描中"与"扫到 0 个"） */
bool bridge_wifi_scan_in_progress(void);

/* 是否已拿到 IP */
bool bridge_wifi_is_connected(void);

/* 获取本机 IP 字符串（如 "192.168.1.100"），未连接返回 NULL */
const char *bridge_wifi_get_ip_str(void);

/* ---- 异步连接（UI 配网流程专用，避免冻结 LVGL 线程） ---- */
typedef enum {
    BRIDGE_WIFI_CONN_IDLE = 0,     /* 未发起 */
    BRIDGE_WIFI_CONN_CONNECTING,   /* 正在连（connect_async 已调，FreeRTOS 任务阻塞等待） */
    BRIDGE_WIFI_CONN_OK,           /* 成功拿到 IP */
    BRIDGE_WIFI_CONN_FAIL,        /* 失败（超时/认证失败等），读 last_reason */
} bridge_wifi_conn_state_t;

/*
 * 异步连接：立即返回 ESP_OK，真正的阻塞 connect 在独立 FreeRTOS 任务里跑。
 * UI 线程轮询 bridge_wifi_get_conn_state() 即可拿到结果。
 * 同一时刻只能有一个 connect 进行中。
 * timeout_ms=0 用默认 30s；boot auto-connect 建议 8000。
 */
esp_err_t bridge_wifi_connect_async(const char *ssid, const char *password);
esp_err_t bridge_wifi_connect_async_ex(const char *ssid, const char *password, uint32_t timeout_ms);

/* 查询异步连接状态 */
bridge_wifi_conn_state_t bridge_wifi_get_conn_state(void);

/* 失败时拿到具体 reason（wifi_event_sta_disconnected_t.reason 或 -1 表示超时） */
int bridge_wifi_get_last_reason(void);

/* 重置连接状态（开始下一次 connect_async 前可先调） */
void bridge_wifi_reset_conn_state(void);

/* ---- v0.1.10+：多槽 NVS 凭证（最多 8 槽 LRU） ---- */
#define BRIDGE_WIFI_MAX_CREDS 8
typedef struct {
    char     ssid[33];
    char     pw[65];
    bool     open;
    uint32_t last_used_ms;   /* 最近一次成功连接的 uptime_ms，0=空槽 */
    uint8_t  bssid[6];      /* 连接成功时拿到的 BSSID；全 0 = 没记录 */
    bool     bssid_set;
} bridge_wifi_cred_t;

/* 存凭证：8 槽 LRU。open AP pw 存 ""；bssid 可传 NULL */
esp_err_t bridge_wifi_save_cred(const char *ssid, const char *password, bool open);
/* 带 BSSID 的版本（连接成功时用） */
esp_err_t bridge_wifi_save_cred_ex(const char *ssid, const char *password, bool open,
                                    const uint8_t *bssid);
/* 读全部已存凭证，按 last_used 降序排列；返回条目数（≤8） */
int bridge_wifi_load_all_creds(bridge_wifi_cred_t *out, int out_cap);
/* 擦除某个 SSID（忘记密码/想换） */
esp_err_t bridge_wifi_forget_cred(const char *ssid);
/* 全清（重置用） */
esp_err_t bridge_wifi_clear_all_creds(void);
/* 查某 SSID 是否已存（给 UI 渲染 ★ 用） */
bool bridge_wifi_cred_exists(const char *ssid);
/* 直接改某 SSID 的 last_used_ms（不碰密码/open）；找不到返回 ESP_ERR_NOT_FOUND */
esp_err_t bridge_wifi_touch_cred(const char *ssid, uint32_t last_used_ms);

/* ---- v0.1.11：boot scan-first connect ----
 * 阻塞 scan → 与存的凭证做交集 → 按 last_used 降序 → connect_async 第一个匹配上的。
 * 比直冲 creds[0] 更聪明（换环境不浪费 8s 冲不在场的 SSID）。
 * 返回：ESP_OK=已发起 connect；ESP_ERR_NOT_FOUND=没扫到匹配的；其他=nvs/scan 失败 */
esp_err_t bridge_wifi_boot_connect_scan_first(void);

/* v0.1.6：链路事件计数，供掉线/重连长测取证 */
typedef struct {
    uint32_t got_ip;        /* 拿到 IP 次数（首次 + 每次自动重连） */
    uint32_t disconnected;  /* 断开次数 */
    int      last_reason;   /* 最近一次 wifi_event_sta_disconnected_t.reason */
} bridge_wifi_stats_t;
void bridge_wifi_get_stats(bridge_wifi_stats_t *out);

/* ---- v0.3.1：配网 SoftAP（APSTA 模式，STA 连接/重连不受影响） ----
 * 开热点的同时 STA 可以继续在后台 fallback 重连，某刻连上了由调用方收掉热点。
 * esp_hosted V1.4.1 slave 对 APSTA 的透传支持需实机验证；失败时退化纯 AP。 */

/* 开 SoftAP。password NULL/空 = 开放网络（v0.5.1 才引入 AP 密码）。
 * SSID 最长 32 字节。重复调用=刷新配置（幂等）。 */
esp_err_t bridge_wifi_ap_start(const char *ssid, const char *password);

/* 关 SoftAP（回到纯 STA；STA 侧连接状态不受影响） */
esp_err_t bridge_wifi_ap_stop(void);

/* SoftAP 是否开着 */
bool bridge_wifi_ap_is_active(void);

/* 当前 SoftAP SSID（未开启返回 NULL） */
const char *bridge_wifi_ap_get_ssid(void);

/* SoftAP IP 字符串（默认 192.168.4.1；未初始化返回 NULL） */
const char *bridge_wifi_ap_get_ip(void);

/* v0.2.2：低时延模式开关（RFC2217/esptool 需要）。
 * en=true  → WIFI_PS_NONE：关闭 modem sleep，消除下行数据被 AP 缓存到
 *            beacon/DTIM 边界才投递造成的 ~200ms 分批（sync 应答 8 帧被
 *            拆成 0.2s 间隔，超出 esptool 0.1s 读超时）。
 * en=false → 恢复 WIFI_PS_MIN_MODEM（默认省电）。
 * 仅在 STA 已连接时生效；未连接时调用仅记录期望状态。 */
esp_err_t bridge_wifi_set_low_latency(bool en);

#ifdef __cplusplus
}
#endif
