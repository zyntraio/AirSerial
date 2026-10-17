/*
 * bridge_wifi —— WiFi 抽象积木实现（v0.1.2，Tab5 C6 后端）。
 *
 * 链路：P4 --SDIO 4bit--> ESP32-C6（esp_hosted slave，出厂固件 V1.4.1）
 * 软件：esp_wifi_remote（标准 esp_wifi API 透明转发）+ esp_hosted（SDIO 传输）。
 * 对上层暴露的就是标准 STA 行为：init / scan / connect / 拿 IP。
 *
 * C6 上电时序（IO 扩展器 WLAN_PWR_EN）在 board_tab5/board_wifi.c，
 * 由 main 在调用 bridge_wifi_init 之前装配——本组件不碰板级电源。
 */
#include "bridge_wifi.h"

#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "nvs_flash.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include <string.h>
#include <inttypes.h>

#define TAG "bridge_wifi"

#define WIFI_CONNECTED_BIT  BIT0
#define WIFI_FAIL_BIT       BIT1
#define WIFI_DISCONN_ACK_BIT BIT2  /* 主动切换时，断链事件已被抑制吃掉 */

#define DEFAULT_CONNECT_TIMEOUT_MS  30000
#define DISCONN_ACK_TIMEOUT_MS 2000
#define MAX_SCAN_AP         24   /* UI 列表页上限 */
#define RECONNECT_DELAY_MS  2000

#define MAX_CREDS_SLOTS     8
#define NVS_NS_CREDS         "airsn_wifi"
#define NVS_SLOT_KEY_FMT    "slot_%d"

static EventGroupHandle_t s_wifi_eg = NULL;
static esp_netif_t       *s_sta_netif = NULL;
static bool               s_connected = false;
static char               s_ip_str[16] = {0};
static volatile uint32_t  s_got_ip_cnt;
static volatile uint32_t  s_disconn_cnt;
static volatile int       s_last_reason = -1;

/* ---- v0.3.1：配网 SoftAP（APSTA）状态 ---- */
static esp_netif_t *s_ap_netif = NULL;
static volatile bool s_ap_active = false;
static char           s_ap_ssid[33] = {0};
static char           s_ap_ip_str[16] = {0};

/* ---- 异步扫描结果缓存（供 UI 层轮询读取） ---- */
static bridge_wifi_ap_t   s_scan_buf[MAX_SCAN_AP];
static volatile int       s_scan_count;     /* >=0 时扫描已完成；读后置 0 清 flag */
static volatile bool      s_scan_running;

/* ---- 异步连接状态（供 UI 层轮询读取） ---- */
static volatile bridge_wifi_conn_state_t s_conn_state = BRIDGE_WIFI_CONN_IDLE;
static volatile bool      s_conn_running;
static char               s_conn_ssid[33];
static char               s_conn_pw[65];

/* 显式重连：esp_wifi_remote 后端不会对所有断开原因自动重连，
 * 必须在 STA_DISCONNECTED 后自己 esp_wifi_connect()。
 * 标志位保证全局只有一个重连任务，断连风暴下不堆叠。 */
static volatile bool s_reconnecting;

/* 用户主动连接/切换进行中：此窗口内的断链不触发自动重连，
 * 其中"主动断开旧链路"的那一次事件还要被完全吃掉（不算失败）。 */
static volatile bool s_manual_mode;
static volatile bool s_switch_suppress;

/* ---- Smart fallback: 路由器挂了就扫一遍，从存过的凭证里选 top1 ----
 * 最多 fallback FALLBACK_MAX 次，每次 FALLBACK_DELAY_MS 后重试。 */
#define FALLBACK_MAX        3
#define FALLBACK_DELAY_MS   1000

/* Scan mutex：保护所有 scan 路径串行化。
 * esp_wifi_scan_start() 内部有驱动层堆分配，并发调用会写坏堆导致 tlsf_free double free。
 * 三个入口：scan_task（用户 Rescan）、fallback_task（断链 fallback）、boot_connect_scan_first。 */
static SemaphoreHandle_t s_scan_mutex;

/* 阻塞扫描 + 去重（复用 scan_task 逻辑） */
static wifi_ap_record_t s_scan_records[MAX_SCAN_AP];   /* 避免栈爆 */
static int do_blocking_scan(bridge_wifi_ap_t *out, int cap)
{
    if (xSemaphoreTake(s_scan_mutex, pdMS_TO_TICKS(60000)) != pdTRUE) {
        ESP_LOGE(TAG, "scan mutex timeout (another scan stuck?)");
        return 0;
    }
    wifi_scan_config_t scan_cfg = { .show_hidden = true };
    esp_err_t ret = esp_wifi_scan_start(&scan_cfg, true);
    ESP_LOGI(TAG, "esp_wifi_scan_start ret=%s", esp_err_to_name(ret));
    if (ret != ESP_OK) { xSemaphoreGive(s_scan_mutex); return 0; }

    uint16_t ap_num = MAX_SCAN_AP;
    esp_err_t ret2 = esp_wifi_scan_get_ap_records(&ap_num, s_scan_records);
    ESP_LOGI(TAG, "esp_wifi_scan_get_ap_records ret=%s, ap_num=%u", esp_err_to_name(ret2), ap_num);
    if (ret2 != ESP_OK) {
        xSemaphoreGive(s_scan_mutex);
        return 0;
    }

    int written = 0;
    for (int i = 0; i < ap_num && written < cap; i++) {
        bool hidden = (s_scan_records[i].ssid[0] == '\0');
        bool dup = false;
        for (int j = 0; j < written; j++) {
            /* 去重：正常 SSID 用 SSID 匹配；隐藏 SSID 用 BSSID 匹配 */
            if (hidden || out[j].hidden) {
                if (memcmp(out[j].bssid, s_scan_records[i].bssid, 6) == 0) {
                    dup = true;
                    if (s_scan_records[i].rssi > out[j].rssi) {
                        out[j].rssi = s_scan_records[i].rssi;
                        out[j].channel = s_scan_records[i].primary;
                    }
                    break;
                }
            } else if (strcmp(out[j].ssid, (char *)s_scan_records[i].ssid) == 0) {
                dup = true;
                if (s_scan_records[i].rssi > out[j].rssi) {
                    out[j].rssi = s_scan_records[i].rssi;
                    out[j].channel = s_scan_records[i].primary;
                }
                break;
            }
        }
        if (dup) continue;

        out[written].hidden = hidden;
        memcpy(out[written].bssid, s_scan_records[i].bssid, 6);
        memcpy(out[written].ssid, s_scan_records[i].ssid, 32);
        out[written].ssid[32] = '\0';
        out[written].rssi = s_scan_records[i].rssi;
        out[written].channel = s_scan_records[i].primary;
        out[written].open = (s_scan_records[i].authmode == WIFI_AUTH_OPEN);
        written++;
    }
    xSemaphoreGive(s_scan_mutex);
    return written;
}

static void fallback_task(void *arg)
{
    int attempt = (int)(intptr_t)arg;
    vTaskDelay(pdMS_TO_TICKS(FALLBACK_DELAY_MS));

    if (s_manual_mode) {
        ESP_LOGI(TAG, "fallback skipped (manual connect in progress)");
        s_reconnecting = false;
        vTaskDelete(NULL);
    }

    ESP_LOGI(TAG, "fallback scan attempt %d/%d", attempt + 1, FALLBACK_MAX);

    /* 用 static s_scan_buf 避免栈爆 */
    int n = do_blocking_scan(s_scan_buf, MAX_SCAN_AP);
    if (n <= 0) {
        ESP_LOGW(TAG, "fallback: no APs found in scan");
        goto try_next;
    }
    ESP_LOGI(TAG, "fallback: scan found %d unique APs", n);

    bridge_wifi_cred_t creds[BRIDGE_WIFI_MAX_CREDS];
    int nc = bridge_wifi_load_all_creds(creds, BRIDGE_WIFI_MAX_CREDS);
    if (nc <= 0) {
        ESP_LOGI(TAG, "fallback: no saved creds");
        goto try_next;
    }

    const bridge_wifi_cred_t *match = NULL;
    for (int ci = 0; ci < nc; ci++) {
        for (int ai = 0; ai < n; ai++) {
            if (strcmp(creds[ci].ssid, s_scan_buf[ai].ssid) == 0) {
                match = &creds[ci];
                break;
            }
        }
        if (match) break;
    }

    if (match == NULL) {
        ESP_LOGW(TAG, "fallback: no saved cred matches scanned APs");
        goto try_next;
    }

    ESP_LOGI(TAG, "fallback: try '%s'", match->ssid);
    bridge_wifi_connect_async_ex(match->ssid,
                                 match->pw[0] ? match->pw : NULL,
                                 8000);
    /* connect_async 会设 s_manual_mode=false，等 GOT_IP/DISCONNECT 事件 */
    s_reconnecting = false;
    vTaskDelete(NULL);

try_next:
    if (attempt + 1 < FALLBACK_MAX) {
        xTaskCreate(fallback_task, "wifi_fb", 8192,
                    (void *)(intptr_t)(attempt + 1), 5, NULL);
    } else {
        ESP_LOGW(TAG, "fallback: all %d attempts exhausted", FALLBACK_MAX);
        s_reconnecting = false;
    }
    vTaskDelete(NULL);
}

esp_err_t bridge_wifi_boot_connect_scan_first(void)
{
    ESP_LOGI(TAG, "boot scan-first: starting blocking scan...");

    /* 栈省 ~1KB：用 static creds_buf（boot 期只调一次，无并发问题） */
    static bridge_wifi_cred_t s_boot_creds[BRIDGE_WIFI_MAX_CREDS];
    int nc = bridge_wifi_load_all_creds(s_boot_creds, BRIDGE_WIFI_MAX_CREDS);
    if (nc <= 0) {
        ESP_LOGI(TAG, "boot scan-first: no saved creds");
        return ESP_ERR_NOT_FOUND;
    }

    /* 用 static s_scan_buf 避免栈爆 */
    int n = do_blocking_scan(s_scan_buf, MAX_SCAN_AP);
    if (n > 0) {
        ESP_LOGI(TAG, "boot scan-first: scan found %d unique APs", n);

        for (int ci = 0; ci < nc; ci++) {
            for (int ai = 0; ai < n; ai++) {
                bool match = false;
                if (s_scan_buf[ai].hidden) {
                    if (s_boot_creds[ci].bssid_set
                        && memcmp(s_boot_creds[ci].bssid, s_scan_buf[ai].bssid, 6) == 0) {
                        match = true;
                    }
                } else {
                    if (strcmp(s_boot_creds[ci].ssid, s_scan_buf[ai].ssid) == 0) {
                        match = true;
                        /* 坏凭证检测：NVS 存 open=true 但 AP 实际加密 → parser bug 存错 → 清掉 */
                        if (s_boot_creds[ci].open && !s_scan_buf[ai].open) {
                            ESP_LOGW(TAG, "boot scan-first: cred open but AP '%s' encrypted — clearing stale",
                                     s_boot_creds[ci].ssid);
                            bridge_wifi_forget_cred(s_boot_creds[ci].ssid);
                            match = false;  // 跳过这条错的
                        }
                    }
                }
                if (match) {
                    ESP_LOGI(TAG, "boot scan-first: match '%s' %s",
                             s_boot_creds[ci].ssid,
                             s_scan_buf[ai].hidden ? "[hidden]" : "");
                    return bridge_wifi_connect_async_ex(s_boot_creds[ci].ssid,
                                                        s_boot_creds[ci].pw[0] ? s_boot_creds[ci].pw : NULL,
                                                        8000);
                }
            }
        }
        ESP_LOGI(TAG, "boot scan-first: scan done but no intersection with saved creds");
    } else {
        ESP_LOGW(TAG, "boot scan-first: no APs found in scan");
    }

    /* 兜底：scan 没找到交集 → 直接冲 creds[0] */
    ESP_LOGI(TAG, "boot scan-first: falling back to direct connect creds[0]='%s'", s_boot_creds[0].ssid);
    return bridge_wifi_connect_async_ex(s_boot_creds[0].ssid,
                                        s_boot_creds[0].pw[0] ? s_boot_creds[0].pw : NULL,
                                        8000);
}

static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "STA started");
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        int reason = ((wifi_event_sta_disconnected_t *)data)->reason;
        s_connected = false;
        s_ip_str[0] = '\0';
        s_disconn_cnt++;
        s_last_reason = reason;

        /* 主动切换：esp_wifi_disconnect() 引发的断链，不算失败，回 ACK 即可 */
        if (s_switch_suppress) {
            s_switch_suppress = false;
            xEventGroupSetBits(s_wifi_eg, WIFI_DISCONN_ACK_BIT);
            ESP_LOGI(TAG, "disconnect during switch suppressed (reason=%d)", reason);
            return;
        }

        ESP_LOGW(TAG, "disconnected #%lu, reason=%d, fallback in %dms",
                 (unsigned long)s_disconn_cnt, reason, FALLBACK_DELAY_MS);
        xEventGroupSetBits(s_wifi_eg, WIFI_FAIL_BIT);
        /* 用户主动连接窗口内（如密码错误导致的断链）由 connect() 主控，不自动重连 */
        if (!s_manual_mode && !s_reconnecting) {
            s_reconnecting = true;
            if (xTaskCreate(fallback_task, "wifi_fb", 8192,
                            (void *)(intptr_t)0, 5, NULL) != pdPASS) {
                s_reconnecting = false;
                ESP_LOGE(TAG, "fallback task create failed");
            }
        }
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_AP_START) {
        /* v0.3.1：SoftAP 起来了，拿实际 IP（默认 192.168.4.1）供 portal/DNS 使用 */
        esp_netif_ip_info_t info = {0};
        if (s_ap_netif && esp_netif_get_ip_info(s_ap_netif, &info) == ESP_OK) {
            snprintf(s_ap_ip_str, sizeof(s_ap_ip_str), IPSTR, IP2STR(&info.ip));
        }
        ESP_LOGI(TAG, "[AP] started ssid='%s' ip=%s", s_ap_ssid, s_ap_ip_str);
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_AP_STACONNECTED) {
        ESP_LOGI(TAG, "[AP] station joined (%d/%d)",
                 ((wifi_event_ap_staconnected_t *)data)->aid,
                 4);
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_AP_STADISCONNECTED) {
        ESP_LOGI(TAG, "[AP] station left");
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)data;
        snprintf(s_ip_str, sizeof(s_ip_str), IPSTR, IP2STR(&event->ip_info.ip));
        s_connected = true;
        s_got_ip_cnt++;
        s_reconnecting = false;
        /* 任何 got IP（手动 connect / boot auto / fallback 链）都标 OK，
         * UI 层（splash timer / wifi_scan poll）靠这个判断状态 */
        s_conn_state = BRIDGE_WIFI_CONN_OK;
        ESP_LOGI(TAG, "got IP #%lu: %s", (unsigned long)s_got_ip_cnt, s_ip_str);
        xEventGroupSetBits(s_wifi_eg, WIFI_CONNECTED_BIT);
    }
}

esp_err_t bridge_wifi_init(void)
{
    /* NVS：WiFi 校准数据需要 */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) {
        return ret;
    }

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    s_wifi_eg = xEventGroupCreate();
    s_sta_netif = esp_netif_create_default_wifi_sta();
    s_ap_netif = esp_netif_create_default_wifi_ap();   /* v0.3.1：配网 SoftAP netif（默认 192.168.4.1/24 + DHCP） */

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    s_scan_mutex = xSemaphoreCreateMutex();
    assert(s_scan_mutex != NULL);

    ESP_LOGI(TAG, "WiFi init OK (backend: s3_builtin)");
    return ESP_OK;
}

/* 扫描并打印周围 AP：验证 SDIO 链路 + C6 射频存活的最小手段（无需凭证） */
esp_err_t bridge_wifi_scan(void)
{
    wifi_scan_config_t scan_cfg = { .show_hidden = true };
    esp_err_t ret = esp_wifi_scan_start(&scan_cfg, true);  /* 阻塞至完成 */
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "scan start failed: %s", esp_err_to_name(ret));
        return ret;
    }

    uint16_t ap_num = MAX_SCAN_AP;
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&ap_num, s_scan_records));

    ESP_LOGI(TAG, "scan done, %u AP found (showing up to %d):", ap_num, MAX_SCAN_AP);
    for (int i = 0; i < ap_num; i++) {
        ESP_LOGI(TAG, "  %-32s rssi=%d ch=%d",
                 (char *)s_scan_records[i].ssid, s_scan_records[i].rssi, s_scan_records[i].primary);
    }
    return ESP_OK;
}

/* ---- 异步扫描：独立 FreeRTOS 任务，完成后写 s_scan_buf + s_scan_count ---- */
static void scan_task(void *arg)
{
    (void)arg;
    /* do_blocking_scan 内部自己拿 mutex，完成后释放 */
    int n = do_blocking_scan(s_scan_buf, MAX_SCAN_AP);
    ESP_LOGI(TAG, "async scan done: %d APs", n);
    s_scan_count = n;
    s_scan_running = false;
    vTaskDelete(NULL);
}

esp_err_t bridge_wifi_scan_async(void)
{
    if (s_scan_running) {
        ESP_LOGW(TAG, "scan already in progress");
        return ESP_ERR_INVALID_STATE;
    }
    if (s_sta_netif == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    s_scan_running = true;
    s_scan_count = -1;   /* -1 = 扫描中 */
    if (xTaskCreate(scan_task, "wifi_scan", 4096, NULL, 4, NULL) != pdPASS) {
        s_scan_running = false;
        s_scan_count = 0;
        ESP_LOGE(TAG, "scan task create failed");
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

int bridge_wifi_get_scan_results(bridge_wifi_ap_t *out, int out_cap)
{
    if (out == NULL || out_cap <= 0) return 0;
    int n = s_scan_count;
    if (n <= 0) {
        return 0;    /* 扫描中(-1) 或 刚读完(0) */
    }
    if (n > out_cap) n = out_cap;
    memcpy(out, s_scan_buf, sizeof(bridge_wifi_ap_t) * n);
    s_scan_count = 0;   /* 读一次清一次，下次新扫描再填 */
    return n;
}

bool bridge_wifi_scan_in_progress(void)
{
    return s_scan_running;
}

/* ---- 异步连接：FreeRTOS 任务里跑阻塞 connect_ex ---- */
static void connect_task(void *arg)
{
    uint32_t timeout_ms = (uint32_t)(uintptr_t)arg;
    bridge_wifi_conn_state_t outcome = BRIDGE_WIFI_CONN_FAIL;
    esp_err_t ret = bridge_wifi_connect_ex(s_conn_ssid,
                                           s_conn_pw[0] ? s_conn_pw : NULL,
                                           timeout_ms);
    if (ret == ESP_OK) {
        outcome = BRIDGE_WIFI_CONN_OK;
    } else {
        ESP_LOGW(TAG, "async connect to '%s' failed: %s (reason=%d)",
                 s_conn_ssid, esp_err_to_name(ret), s_last_reason);
    }
    s_conn_state = outcome;
    s_conn_running = false;
    vTaskDelete(NULL);
}

esp_err_t bridge_wifi_connect_async(const char *ssid, const char *password)
{
    return bridge_wifi_connect_async_ex(ssid, password, DEFAULT_CONNECT_TIMEOUT_MS);
}

esp_err_t bridge_wifi_connect_async_ex(const char *ssid, const char *password, uint32_t timeout_ms)
{
    if (ssid == NULL || ssid[0] == '\0') {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_sta_netif == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_conn_running) {
        ESP_LOGW(TAG, "connect already in progress");
        return ESP_ERR_INVALID_STATE;
    }
    /* 留存凭证（任务异步读取） */
    strlcpy(s_conn_ssid, ssid, sizeof(s_conn_ssid));
    if (password) {
        strlcpy(s_conn_pw, password, sizeof(s_conn_pw));
    } else {
        s_conn_pw[0] = '\0';
    }
    s_conn_state = BRIDGE_WIFI_CONN_CONNECTING;
    s_conn_running = true;
    if (xTaskCreate(connect_task, "wifi_conn", 4096,
                    (void *)(uintptr_t)timeout_ms, 4, NULL) != pdPASS) {
        s_conn_running = false;
        s_conn_state = BRIDGE_WIFI_CONN_IDLE;
        ESP_LOGE(TAG, "connect task create failed");
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "async connect to '%s' started (timeout=%lums)",
             ssid, (unsigned long)timeout_ms);
    return ESP_OK;
}

bridge_wifi_conn_state_t bridge_wifi_get_conn_state(void)
{
    return s_conn_state;
}

int bridge_wifi_get_last_reason(void)
{
    return s_last_reason;
}

void bridge_wifi_reset_conn_state(void)
{
    s_conn_state = BRIDGE_WIFI_CONN_IDLE;
}

esp_err_t bridge_wifi_connect(const char *ssid, const char *password)
{
    return bridge_wifi_connect_ex(ssid, password, DEFAULT_CONNECT_TIMEOUT_MS);
}

esp_err_t bridge_wifi_connect_ex(const char *ssid, const char *password, uint32_t timeout_ms)
{
    if (ssid == NULL || ssid[0] == '\0') {
        return ESP_ERR_INVALID_ARG;
    }
    if (timeout_ms == 0) timeout_ms = DEFAULT_CONNECT_TIMEOUT_MS;

    /* 全程 manual 模式：压住后台自动重连，断链事件由本函数主控 */
    s_manual_mode = true;
    s_last_reason = -1;   /* reset：timeout 就留 -1，收到真实 disconnect 事件会更新 */

    /* 切换 AP：必须先断开旧链路/取消进行中的连接尝试，
     * 否则 esp_wifi_connect() 在已连接时返回 ESP_ERR_WIFI_CONN。
     * 这次主动断链事件由 handler 抑制并回 DISCONN_ACK。 */
    s_switch_suppress = true;
    xEventGroupClearBits(s_wifi_eg, WIFI_DISCONN_ACK_BIT);
    esp_err_t dret = esp_wifi_disconnect();
    if (dret == ESP_OK) {
        xEventGroupWaitBits(s_wifi_eg, WIFI_DISCONN_ACK_BIT,
                            pdTRUE, pdTRUE,
                            pdMS_TO_TICKS(DISCONN_ACK_TIMEOUT_MS));
    } else {
        /* ESP_ERR_WIFI_NOT_CONN 等：本来就没连，没有事件会来，直接继续 */
        s_switch_suppress = false;
        ESP_LOGI(TAG, "pre-connect disconnect: %s (nothing to tear down)",
                 esp_err_to_name(dret));
    }

    wifi_config_t wifi_config = {0};
    strlcpy((char *)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid));
    if (password != NULL) {
        strlcpy((char *)wifi_config.sta.password, password, sizeof(wifi_config.sta.password));
    }
    /* 让 ESP32 自动选择 WPA2 或 WPA3-SAE（兼容 mixed 路由器）*/
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    xEventGroupClearBits(s_wifi_eg, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT);
    esp_err_t cfg_ret = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    if (cfg_ret != ESP_OK) {
        s_manual_mode = false;
        ESP_LOGE(TAG, "set_config failed: %s", esp_err_to_name(cfg_ret));
        return cfg_ret;
    }
    esp_err_t con_ret = esp_wifi_connect();
    if (con_ret != ESP_OK) {
        s_manual_mode = false;
        ESP_LOGE(TAG, "esp_wifi_connect call failed: %s", esp_err_to_name(con_ret));
        return con_ret;
    }

    ESP_LOGI(TAG, "connecting to %s ...", ssid);
    EventBits_t bits = xEventGroupWaitBits(s_wifi_eg,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE, pdFALSE,
                                           pdMS_TO_TICKS(timeout_ms));
    bool ok = (bits & WIFI_CONNECTED_BIT) != 0;
    s_manual_mode = false;

    if (ok) {
        /* v0.1.10+：成功即存 8 槽 LRU，带 open 标记 + last_used_ms + BSSID */
        bool is_open = (password == NULL || password[0] == '\0');
        /* 拿一下当前连接的 BSSID（隐藏 SSID 场景下这是唯一可靠的标识） */
        wifi_ap_record_t sta_info = {0};
        const uint8_t *bssid_ptr = NULL;
        if (esp_wifi_sta_get_ap_info(&sta_info) == ESP_OK) {
            bssid_ptr = sta_info.bssid;
        }
        (void)bridge_wifi_save_cred_ex(ssid, is_open ? "" : password, is_open, bssid_ptr);
        return ESP_OK;
    }

    /* 失败：交还自动 fallback 机制（smart fallback 会扫描存的凭证挑 top1） */
    if (!s_reconnecting) {
        s_reconnecting = true;
        if (xTaskCreate(fallback_task, "wifi_fb", 8192,
                        (void *)(intptr_t)0, 5, NULL) != pdPASS) {
            s_reconnecting = false;
        }
    }
    ESP_LOGE(TAG, "connect to %s failed/timeout", ssid);
    return ESP_FAIL;
}

bool bridge_wifi_is_connected(void)
{
    return s_connected;
}

const char *bridge_wifi_get_ip_str(void)
{
    return s_connected ? s_ip_str : NULL;
}

/* ---- v0.1.10+：8 槽 LRU NVS 凭证 ---- */
/* 每个槽存一个 bridge_wifi_cred_t 结构体，key 是 "slot_0" ~ "slot_7" */

static bool slot_load(nvs_handle_t h, int idx, bridge_wifi_cred_t *out)
{
    char key[16];
    snprintf(key, sizeof(key), NVS_SLOT_KEY_FMT, idx);
    memset(out, 0, sizeof(*out));
    size_t len = sizeof(*out);
    esp_err_t ret = nvs_get_blob(h, key, out, &len);
    if (ret != ESP_OK) return false;
    /* 兼容旧版本：len < sizeof(*out) 时新字段（bssid/bssid_set）已由上面 memset 置零 */
    if (len > sizeof(*out)) len = sizeof(*out);
    return out->ssid[0] != '\0';
}

static esp_err_t slot_save(nvs_handle_t h, int idx, const bridge_wifi_cred_t *in)
{
    char key[16];
    snprintf(key, sizeof(key), NVS_SLOT_KEY_FMT, idx);
    return nvs_set_blob(h, key, in, sizeof(*in));
}

static void slot_invalidate(nvs_handle_t h, int idx)
{
    char key[16];
    snprintf(key, sizeof(key), NVS_SLOT_KEY_FMT, idx);
    nvs_erase_key(h, key);  /* 找不到返回 ERR_NOT_FOUND，无害 */
}

esp_err_t bridge_wifi_save_cred(const char *ssid, const char *password, bool open)
{
    return bridge_wifi_save_cred_ex(ssid, password, open, NULL);
}

esp_err_t bridge_wifi_save_cred_ex(const char *ssid, const char *password, bool open,
                                    const uint8_t *bssid)
{
    if (ssid == NULL || ssid[0] == '\0') return ESP_ERR_INVALID_ARG;

    ESP_LOGI(TAG, "save_cred ENTER: ssid='%s' open=%d bssid=%s",
             ssid, open, bssid ? "set" : "null");

    nvs_handle_t h;
    esp_err_t ret = nvs_open(NVS_NS_CREDS, NVS_READWRITE, &h);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open creds save: %s", esp_err_to_name(ret));
        return ret;
    }

    bridge_wifi_cred_t slots[MAX_CREDS_SLOTS];
    for (int i = 0; i < MAX_CREDS_SLOTS; i++) {
        memset(&slots[i], 0, sizeof(slots[i]));
        if (slot_load(h, i, &slots[i])
            && slots[i].ssid[0] != '\0') {
            /* valid */
        } else {
            slots[i].last_used_ms = 0;  /* 显式标记空 */
        }
    }

    /* 1) 同 SSID 已存 → 更新（不覆盖已有的 bssid，除非新的非 NULL） */
    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
    int found_idx = -1;
    for (int i = 0; i < MAX_CREDS_SLOTS; i++) {
        if (slots[i].ssid[0] != '\0'
            && strcmp(slots[i].ssid, ssid) == 0) {
            found_idx = i; break;
        }
    }
    if (found_idx >= 0) {
        strlcpy(slots[found_idx].pw, password ? password : "", sizeof(slots[found_idx].pw));
        slots[found_idx].open = open;
        slots[found_idx].last_used_ms = now_ms;
        if (bssid != NULL) {
            memcpy(slots[found_idx].bssid, bssid, 6);
            slots[found_idx].bssid_set = true;
        }
        slot_save(h, found_idx, &slots[found_idx]);
        ret = nvs_commit(h);
        nvs_close(h);
        ESP_LOGI(TAG, "updated cred ssid='%s' open=%d bssid=%s (idx=%d)",
                 ssid, open, slots[found_idx].bssid_set ? "set" : "keep", found_idx);
        return ret;
    }

    /* 2) 找空槽 */
    int empty_idx = -1;
    for (int i = 0; i < MAX_CREDS_SLOTS; i++) {
        if (slots[i].last_used_ms == 0 || slots[i].ssid[0] == '\0') {
            empty_idx = i; break;
        }
    }
    int target_idx = (empty_idx >= 0) ? empty_idx : -1;

    /* 3) 没空槽 → LRU：覆盖 last_used_ms 最小的 */
    if (target_idx < 0) {
        uint32_t min_ms = UINT32_MAX;
        for (int i = 0; i < MAX_CREDS_SLOTS; i++) {
            if (slots[i].last_used_ms < min_ms) {
                min_ms = slots[i].last_used_ms;
                target_idx = i;
            }
        }
        ESP_LOGI(TAG, "LRU evict slot %d (last_used_ms=%lu)",
                 target_idx, (unsigned long)min_ms);
    }

    bridge_wifi_cred_t c = {0};
    strlcpy(c.ssid, ssid, sizeof(c.ssid));
    strlcpy(c.pw, password ? password : "", sizeof(c.pw));
    c.open = open;
    c.last_used_ms = now_ms;
    if (bssid != NULL) {
        memcpy(c.bssid, bssid, 6);
        c.bssid_set = true;
    }
    slot_save(h, target_idx, &c);
    ret = nvs_commit(h);
    nvs_close(h);
    ESP_LOGI(TAG, "saved cred ssid='%s' open=%d (idx=%d)", ssid, open, target_idx);
    return ret;
}

int bridge_wifi_load_all_creds(bridge_wifi_cred_t *out, int out_cap)
{
    if (out == NULL || out_cap <= 0) return 0;

    nvs_handle_t h;
    if (nvs_open(NVS_NS_CREDS, NVS_READONLY, &h) != ESP_OK) {
        return 0;
    }
    bridge_wifi_cred_t slots[MAX_CREDS_SLOTS];
    int n = 0;
    for (int i = 0; i < MAX_CREDS_SLOTS && n < out_cap; i++) {
        if (slot_load(h, i, &slots[i]) && slots[i].ssid[0] != '\0') {
            out[n++] = slots[i];
        }
    }
    nvs_close(h);

    /* 按 last_used_ms 降序（简单冒泡，≤8 条） */
    for (int i = 0; i < n - 1; i++) {
        for (int j = i + 1; j < n; j++) {
            if (out[j].last_used_ms > out[i].last_used_ms) {
                bridge_wifi_cred_t t = out[i];
                out[i] = out[j];
                out[j] = t;
            }
        }
    }
    ESP_LOGI(TAG, "loaded %d saved creds", n);
    return n;
}

esp_err_t bridge_wifi_forget_cred(const char *ssid)
{
    if (ssid == NULL || ssid[0] == '\0') return ESP_ERR_INVALID_ARG;
    nvs_handle_t h;
    if (nvs_open(NVS_NS_CREDS, NVS_READWRITE, &h) != ESP_OK) return ESP_ERR_NOT_FOUND;
    esp_err_t ret = ESP_ERR_NOT_FOUND;
    bridge_wifi_cred_t c;
    for (int i = 0; i < MAX_CREDS_SLOTS; i++) {
        if (slot_load(h, i, &c) && c.ssid[0] != '\0'
            && strcmp(c.ssid, ssid) == 0) {
            slot_invalidate(h, i);
            ret = nvs_commit(h);
            ESP_LOGI(TAG, "forgot cred ssid='%s'", ssid);
            break;
        }
    }
    nvs_close(h);
    return ret;
}

esp_err_t bridge_wifi_clear_all_creds(void)
{
    nvs_handle_t h;
    esp_err_t ret = nvs_open(NVS_NS_CREDS, NVS_READWRITE, &h);
    if (ret != ESP_OK) return ret;
    /* nvs_erase_namespace 在 v6 里没了——逐个 slot_0~7 删 */
    for (int i = 0; i < MAX_CREDS_SLOTS; i++) {
        char key[16];
        snprintf(key, sizeof(key), NVS_SLOT_KEY_FMT, i);
        nvs_erase_key(h, key);
    }
    ret = nvs_commit(h);
    nvs_close(h);
    ESP_LOGI(TAG, "all creds cleared");
    return ret;
}

bool bridge_wifi_cred_exists(const char *ssid)
{
    if (ssid == NULL || ssid[0] == '\0') return false;
    bridge_wifi_cred_t out[MAX_CREDS_SLOTS];
    int n = bridge_wifi_load_all_creds(out, MAX_CREDS_SLOTS);
    for (int i = 0; i < n; i++) {
        if (strcmp(out[i].ssid, ssid) == 0) return true;
    }
    return false;
}

esp_err_t bridge_wifi_touch_cred(const char *ssid, uint32_t last_used_ms)
{
    if (ssid == NULL || ssid[0] == '\0') return ESP_ERR_INVALID_ARG;

    nvs_handle_t h;
    esp_err_t ret = nvs_open(NVS_NS_CREDS, NVS_READWRITE, &h);
    if (ret != ESP_OK) return ret;

    bridge_wifi_cred_t slots[MAX_CREDS_SLOTS];
    for (int i = 0; i < MAX_CREDS_SLOTS; i++) {
        memset(&slots[i], 0, sizeof(slots[i]));
        if (slot_load(h, i, &slots[i])
            && slots[i].ssid[0] != '\0') {
            /* valid */
        } else {
            slots[i].last_used_ms = 0;
        }
    }

    int found = -1;
    for (int i = 0; i < MAX_CREDS_SLOTS; i++) {
        if (slots[i].ssid[0] != '\0'
            && strcmp(slots[i].ssid, ssid) == 0) {
            found = i; break;
        }
    }

    if (found < 0) {
        nvs_close(h);
        return ESP_ERR_NOT_FOUND;
    }

    slots[found].last_used_ms = last_used_ms;
    slot_save(h, found, &slots[found]);
    ret = nvs_commit(h);
    nvs_close(h);
    ESP_LOGI(TAG, "touched cred '%s' last_used_ms=%lu (idx=%d)",
             ssid, (unsigned long)last_used_ms, found);
    return ret;
}

void bridge_wifi_get_stats(bridge_wifi_stats_t *out)
{
    if (out == NULL) {
        return;
    }
    out->got_ip = s_got_ip_cnt;
    out->disconnected = s_disconn_cnt;
    out->last_reason = s_last_reason;
}

/* v0.2.2：低时延模式（见头文件注释）。esp_wifi_remote 经 RPC 落到 C6 执行 */
esp_err_t bridge_wifi_set_low_latency(bool en)
{
    if (!s_connected) {
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t ret = esp_wifi_set_ps(en ? WIFI_PS_NONE : WIFI_PS_MIN_MODEM);
    ESP_LOGI(TAG, "low_latency=%d (ps %s): %s", (int)en,
             en ? "NONE" : "MIN_MODEM", esp_err_to_name(ret));
    return ret;
}

/* ---- v0.3.1：配网 SoftAP（APSTA） ---- */

esp_err_t bridge_wifi_ap_start(const char *ssid, const char *password)
{
    if (s_ap_netif == NULL || ssid == NULL || ssid[0] == '\0') {
        return ESP_ERR_INVALID_ARG;
    }
    if (strlen(ssid) > 32) {
        ESP_LOGE(TAG, "[AP] ssid too long (%d > 32)", (int)strlen(ssid));
        return ESP_ERR_INVALID_ARG;
    }

    wifi_config_t ap_cfg = { 0 };
    strlcpy((char *)ap_cfg.ap.ssid, ssid, sizeof(ap_cfg.ap.ssid));
    ap_cfg.ap.ssid_len = strlen(ssid);
    ap_cfg.ap.max_connection = 4;
    if (password && password[0]) {
        if (strlen(password) < 8) {
            ESP_LOGE(TAG, "[AP] password must be >=8 chars (WPA2)");
            return ESP_ERR_INVALID_ARG;
        }
        strlcpy((char *)ap_cfg.ap.password, password, sizeof(ap_cfg.ap.password));
        ap_cfg.ap.authmode = WIFI_AUTH_WPA2_PSK;
    } else {
        ap_cfg.ap.authmode = WIFI_AUTH_OPEN;   /* v0.5.1 才引入 AP 密码 */
    }
    ap_cfg.ap.channel = 6;   /* 不设则默认与 STA 同信道，STA 未连时为 1；显式 6 更通用 */

    /* STA 已连接/重连中切 APSTA 不断 STA——这是选 APSTA 而非纯 AP 的原因 */
    esp_err_t ret = esp_wifi_set_mode(WIFI_MODE_APSTA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[AP] set_mode(APSTA) failed: %s (esp_hosted APSTA support?)",
                 esp_err_to_name(ret));
        return ret;
    }
    ret = esp_wifi_set_config(WIFI_IF_AP, &ap_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[AP] set_config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    strlcpy(s_ap_ssid, ssid, sizeof(s_ap_ssid));
    s_ap_active = true;   /* AP_START 事件里补拿 IP */
    ESP_LOGI(TAG, "[AP] softap '%s' starting (%s)", ssid,
             (password && password[0]) ? "wpa2" : "open");
    return ESP_OK;
}

esp_err_t bridge_wifi_ap_stop(void)
{
    if (!s_ap_active) {
        return ESP_OK;   /* 幂等 */
    }
    esp_err_t ret = esp_wifi_set_mode(WIFI_MODE_STA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[AP] set_mode(STA) failed: %s", esp_err_to_name(ret));
        return ret;
    }
    s_ap_active = false;
    s_ap_ssid[0] = '\0';
    ESP_LOGI(TAG, "[AP] softap stopped (back to pure STA)");
    return ESP_OK;
}

bool bridge_wifi_ap_is_active(void)
{
    return s_ap_active;
}

const char *bridge_wifi_ap_get_ssid(void)
{
    return s_ap_active ? s_ap_ssid : NULL;
}

const char *bridge_wifi_ap_get_ip(void)
{
    return s_ap_ip_str[0] ? s_ap_ip_str : NULL;
}
