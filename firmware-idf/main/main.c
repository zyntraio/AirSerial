/*
 * AirSerial — M5Stack StickS3 (ESP32-S3) 启动链
 *
 * main = 纯调度器：只做积木装配，不含板级细节。
 * Tier 3: 有屏无键盘 → 状态页
 *
 * 启动顺序（无依赖冲突，能早 init 就早）：
 *   1. NVS init + 崩溃遗言检查 + USB Host 开关读 flag
 *   2. board_init() — LCD + 状态页（boot_splash 抢最早屏幕输出）
 *   2.5 SHARED_PHY 且 Device 模式：PHY 强制归位 Serial-JTAG（非 POR 复位）
 *   3. bridge_core 子系统（serial/tcp/rfc2217/mdns）
 *   4. bridge_wifi init（S3 内置）
 *   5. USB Host init（默认 OFF，开发调试安全；需要时网页打开）
 *   6. portal_mgr_task（WiFi 管理 + bridge 服务启动）
 *
 * USB Host 安全默认：
 *   启动时 s_usb_host_enabled = false → 跳 USB Host init → PHY 保持 Device 模式
 *   USB-Serial-JTAG monitor 不会断，开发时随便重启都能看日志
 *   用户 POST /api/usb/host_on 写 NVS → reboot → Host 启用（monitor 会断）
 *   随时 POST /api/usb/host_off → 清 flag → reboot → 回到安全 Device 模式
 */
#include "sdkconfig.h"

#include "board.h"
#include "bridge_wifi.h"
#include "bridge_tcp.h"
#include "bridge_rfc2217.h"
#include "bridge_mdns.h"
#include "bridge_serial.h"
#include "bridge_cfg.h"
#include "bridge_crash.h"
#include "bridge_web.h"
#include "bridge_usb_host.h"
#include "bridge_uart.h"
#include "app_version.h"
#include "ui_view_manager.h"

#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_http_server.h"
#include "esp_system.h"
#include "esp_netif.h"
#include "esp_heap_caps.h"
#include "esp_psram.h"
#include "esp_private/usb_phy.h"
#include "esp32s3/rom/rtc.h"        /* esp_rom_get_reset_reason：避开 esp_reset_reason() 链接冲突 */
#include "usb/usb_host.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

#include <stdio.h>
#include <string.h>

#define TAG "airserial"

/* — USB Host 开关（开发调试用，默认 OFF）— */
static bool s_usb_host_enabled;   /* false = Device mode (default safe), true = Host */

#define NVS_NS_DIAG      "airsn_diag"
#define NVS_KEY_USB_ON   "usb_host_on"   /* 1 = Host ON, 0/absent = Host OFF (default) */

/* — dummy transport（USB Host 未实现时用，send 直接 return OK）— */
static esp_err_t dummy_send_fn(const uint8_t *d, size_t l, void *a)
    { (void)d; (void)l; (void)a; return ESP_OK; }
static const bridge_serial_transport_t s_dummy_transport = {
    .send = dummy_send_fn, .arg = NULL
};

/* — SoftAP SSID（启动时补 MAC 后 4 位）— */
static char s_ap_ssid[33] = "AirSerial";

/* — HTTP /api/usb/host_on — 写 NVS → reboot 启用 Host — */
static esp_err_t usb_host_on_handler(httpd_req_t *req)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS_DIAG, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_u8(h, NVS_KEY_USB_ON, 1);
        nvs_commit(h);
        nvs_close(h);
    }
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"ok\":true,\"msg\":\"usb host ON, rebooting — monitor will disconnect\"}");
    ESP_LOGW(TAG, "USB Host ENABLED via API — rebooting in 2s (monitor will disconnect)");
    vTaskDelay(pdMS_TO_TICKS(2000));
    esp_restart();
    return ESP_OK;
}

/* — HTTP /api/usb/host_off — 清 NVS flag → reboot 回到安全 Device 模式 — */
static esp_err_t usb_host_off_handler(httpd_req_t *req)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS_DIAG, NVS_READWRITE, &h) == ESP_OK) {
        nvs_erase_key(h, NVS_KEY_USB_ON);
        nvs_commit(h);
        nvs_close(h);
    }
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"ok\":true,\"msg\":\"usb host OFF, rebooting — monitor will return\"}");
    ESP_LOGI(TAG, "USB Host DISABLED via API — rebooting in 2s (monitor returns)");
    vTaskDelay(pdMS_TO_TICKS(2000));
    esp_restart();
    return ESP_OK;
}

/* — HTTP /api/usb/host_status — 查当前状态（不重启）— */
static esp_err_t usb_host_status_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    board_usb_topo_t topo = board_get_usb_topo();
    bool supported = (topo != USB_TOPO_NO_HOST);
    bool safe_init = board_usb_host_init_safe();
    char buf[384];
    snprintf(buf, sizeof(buf),
             "{"
             "\"host_supported\":%s,"
             "\"host_init_safe\":%s,"
             "\"topo\":\"%s\","
             "\"host_enabled\":%s,"
             "\"mode\":\"%s\","
             "\"uart_ready\":%s,"
             "\"cdc_open\":%s,"
             "\"tcp_port\":3333,"
             "\"rfc2217_port\":3334"
             "}",
             supported ? "true" : "false",
             safe_init ? "true" : "false",
             topo == USB_TOPO_INDEPENDENT ? "INDEPENDENT" :
             topo == USB_TOPO_SHARED_PHY  ? "SHARED_PHY" :
             topo == USB_TOPO_NO_HOST     ? "NO_HOST"    : "UNKNOWN",
             s_usb_host_enabled ? "true" : "false",
             s_usb_host_enabled ? "host" : "device",
             bridge_uart_is_ready() ? "true" : "false",
             bridge_usb_host_is_open() ? "true" : "false");
    httpd_resp_sendstr(req, buf);
    return ESP_OK;
}

/* — HTTP /api/usb/debug — 最简 debug，返回 retry 状态 — */
static int s_cdc_retry_count;
static esp_err_t s_cdc_retry_last_err;
static TaskHandle_t s_cdc_retry_task_hdl;   /* 掉线事件即时唤醒快速重开 */

static esp_err_t usb_debug_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    if (!s_usb_host_enabled) {
        httpd_resp_sendstr(req, "{\"error\":\"USB Host not enabled\"}");
        return ESP_OK;
    }
    char buf[256];
    int ver = bridge_usb_host_ch340_version();
    char ver_str[12];
    if (ver >= 0 && ver <= 0xFF) {
        snprintf(ver_str, sizeof(ver_str), "\"%02x\"", ver);
    } else {
        snprintf(ver_str, sizeof(ver_str), "null");
    }
    snprintf(buf, sizeof(buf),
             "{\"host_enabled\":true,\"cdc_open\":%s,\"retry_count\":%d,\"last_err\":%d,\"uart_ready\":%s"
             ",\"wch\":%s,\"ch340_ver\":%s,\"ch340_clone\":%s}",
             bridge_usb_host_is_open() ? "true" : "false",
             s_cdc_retry_count, (int)s_cdc_retry_last_err,
             bridge_uart_is_ready() ? "true" : "false",
             bridge_usb_host_is_wch() ? "true" : "false",
             ver_str,
             bridge_usb_host_ch340_is_clone() ? "true" : "false");
    httpd_resp_sendstr(req, buf);
    return ESP_OK;
}

static httpd_uri_t s_host_on_uri = {
    .uri     = "/api/usb/host_on",
    .method  = HTTP_POST,
    .handler = usb_host_on_handler,
};

static httpd_uri_t s_host_off_uri = {
    .uri     = "/api/usb/host_off",
    .method  = HTTP_POST,
    .handler = usb_host_off_handler,
};

static httpd_uri_t s_host_status_uri = {
    .uri     = "/api/usb/host_status",
    .method  = HTTP_GET,
    .handler = usb_host_status_handler,
};

static httpd_uri_t s_debug_uri = {
    .uri     = "/api/usb/debug",
    .method  = HTTP_GET,
    .handler = usb_debug_handler,
};

/* —— Bridge 客户端统一协调器 ——
 *
 * bridge_tcp 和 bridge_rfc2217 各自独立，各有 client_cb。
 * 这里统一协调：维护总客户端数，STA 模式下：
 *   count > 0 → 停 portal（httpd + WebSocket forward task）省 RAM
 *   count → 0 → 延迟 10s 恢复 portal（避免 esptool 快速重连抖动）
 * AP 配网模式下 portal 永远不关（用户需要配网页面）
 *
 * ★ 关键守卫：portal 自动暂停 **只在无 PSRAM 的板子上启用**。
 *   有 PSRAM 的板子 portal 永不因 bridge client 而停——用户烧录时还能
 *   开 Web 终端看 log，体验更好。这就是内存富裕带来的体验差距。
 *
 * RAM 节省：httpd 栈 8KB + ws_fwd task 4KB + WS subscriber 2KB
 *           + lwIP 若干 socket 开销 ≈ 15-20KB，裸 N4 S3 很关键
 */
static int s_bridge_client_count;
static TimerHandle_t s_portal_resume_timer;
static bool s_portal_autopause;  /* true = 无 PSRAM → 需要省 RAM */

static void portal_resume_timer_cb(TimerHandle_t t)
{
    (void)t;
    /* 安全检查：恢复前再确认一下确实没有客户端 + portal 没被 portal_mgr 开起来 */
    if (s_bridge_client_count > 0) {
        ESP_LOGI(TAG, "portal resume: client appeared, abort");
        return;
    }
    if (bridge_web_portal_is_running()) {
        ESP_LOGI(TAG, "portal resume: already running, skip");
        return;
    }
    if (bridge_wifi_ap_is_active()) {
        ESP_LOGI(TAG, "portal resume: AP mode, let portal_mgr handle it");
        return;
    }
    ESP_LOGI(TAG, "portal resume: WiFi STA, no clients → restarting portal");
    if (bridge_web_portal_start(s_ap_ssid, bridge_wifi_get_ip_str()) == ESP_OK) {
        bridge_web_add_uri_handler(&s_host_on_uri);
        bridge_web_add_uri_handler(&s_host_off_uri);
        bridge_web_add_uri_handler(&s_host_status_uri);
        bridge_web_add_uri_handler(&s_debug_uri);
        ESP_LOGI(TAG, "portal resumed ✅");
    } else {
        ESP_LOGE(TAG, "portal resume: start failed, will retry next count→0");
    }
}

/* tcp + rfc2217 共用的 client callback（arg 传模块名用于日志）*/
static void bridge_client_cb(bool connected, void *arg)
{
    const char *mod = (const char *)arg;

    if (connected) {
        s_bridge_client_count++;
        ESP_LOGI(TAG, "bridge client [%s] connected — total count=%d", mod, s_bridge_client_count);

        /* 取消待执行的 resume 定时器（避免重复 start）*/
        if (s_portal_resume_timer) xTimerStop(s_portal_resume_timer, 0);

        /* ★ 只有无 PSRAM 的板子才真的停 portal（有 PSRAM 的板子 portal 保持运行，
         * 用户还能开 Web 终端看 log，体验更好）*/
        if (s_portal_autopause
            && !bridge_wifi_ap_is_active()
            && bridge_web_portal_is_running()) {
            ESP_LOGW(TAG, "portal_stop: no-PSRAM board + STA + client → free ~20KB");
            bridge_web_portal_stop();
        }
    } else {
        if (s_bridge_client_count > 0) s_bridge_client_count--;
        ESP_LOGI(TAG, "bridge client [%s] disconnected — total count=%d", mod, s_bridge_client_count);

        /* count → 0 + 无 PSRAM + STA：10s 后恢复 portal（抗 esptool 重连抖动）*/
        if (s_portal_autopause
            && s_bridge_client_count == 0
            && !bridge_wifi_ap_is_active()) {
            if (bridge_web_portal_is_running()) {
                ESP_LOGI(TAG, "portal resume: already running, no-op");
            } else {
                ESP_LOGI(TAG, "portal resume scheduled in 10s (no-PSRAM STA idle)");
                if (s_portal_resume_timer) {
                    xTimerReset(s_portal_resume_timer, pdMS_TO_TICKS(10000));
                }
            }
        }
    }

    /* 通知 WiFi 省电切换：有客户端低时延优先（不管有没有 PSRAM）*/
    bridge_wifi_set_low_latency(s_bridge_client_count > 0);
}

/* — RFC2217 带外控制回调（SET_BAUDRATE/DTR/RTS）— */
static void rfc2217_ctrl_cb(uint32_t baud, bool dtr, bool rts, void *arg)
{
    (void)arg;
    /* 按当前 active transport 路由：USB Host CDC 在口上就转发给 USB 设备
     *（esptool 自动复位电路 RTS→EN / DTR→GPIO0 靠这个），否则走板载 UART 路径 */
    if (bridge_usb_host_is_open()) {
        if (baud > 0) bridge_usb_host_set_baud(baud);
        esp_err_t cr = bridge_usb_host_set_control(dtr, rts);  /* 含双低：esptool 复位时序末尾要释放 */
        ESP_LOGI(TAG, "RFC2217 → USB CDC: baud=%lu DTR=%d RTS=%d ret=%s",
                 (unsigned long)baud, (int)dtr, (int)rts, esp_err_to_name(cr));
        return;
    }
    /* SET_BAUDRATE: 同步到 bridge_uart（95% 场景够用，当前 active transport 就是 UART）*/
    if (baud > 0 && baud != bridge_uart_get_params()->baud_rate) {
        ESP_LOGI(TAG, "RFC2217 SET_BAUDRATE → bridge_uart: %lu", (unsigned long)baud);
        const bridge_uart_params_t *cur = bridge_uart_get_params();
        bridge_uart_params_t p = {
            .baud_rate = baud,
            .data_bits = cur->data_bits,
            .stop_bits = cur->stop_bits,
            .parity    = cur->parity,
            .gpio_tx   = cur->gpio_tx,
            .gpio_rx   = cur->gpio_rx,
        };
        bridge_uart_set_params(&p);
    }
    /* RFC2217 SET_CONTROL: DTR/RTS → board GPIO（路径 B：直接替代 USB-UART 桥）*/
    if (board_serial_ctrl_is_supported()) {
        board_serial_ctrl_set(dtr, rts);
    } else {
        ESP_LOGI(TAG, "RFC2217 DTR=%d RTS=%d (board not wired)", dtr, rts);
    }
    /* USB 目标可能正处于 EN 复位后的重枚举窗口：纯控制事件（非 SET_BAUDRATE
     * 复用回调）也同步记忆到 USB 侧，重开后按主机最新态重放。*/
    if (baud == 0) bridge_usb_host_set_control(dtr, rts);
}

/* — USB CDC 拔出：立即切回 UART 兜底 transport（retry 重开成功后再切回 Host）— */
static void cdc_disconnect_cb(void *arg)
{
    (void)arg;
    bridge_serial_set_transport(bridge_uart_transport());
    ESP_LOGW(TAG, "USB CDC gone — transport fell back to UART1");
    /* USB-JTAG 目标 EN 复位会触发同一事件：必须立刻唤醒重开，
     * ROM 下载模式只在总线上停留几百 ms（旧节拍最坏要等 8s）。*/
    if (s_cdc_retry_task_hdl) xTaskNotifyGive(s_cdc_retry_task_hdl);
}

/* — cdc_retry_task: 后台重扫 CDC-ACM（Host 模式下插/拔 USB 设备自动重连）— */
static void cdc_retry_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "cdc_retry_task started — fast reopen on disconnect, 5s idle scan...");
    vTaskDelay(pdMS_TO_TICKS(8000));  /* 等 USB Host PHY 稳定 + 设备枚举 */
    while (1) {
        if (bridge_usb_host_is_open()) {
            /* 在线：5s 空闲节拍，掉线通知可立即打断 */
            xTaskNotifyWait(0, 0xFFFFFFFF, NULL, pdMS_TO_TICKS(5000));
            continue;
        }
        ESP_LOGI(TAG, "cdc_retry #%d: trying bridge_usb_host_open(2000)...", ++s_cdc_retry_count);
        esp_err_t r = bridge_usb_host_open(2000);
        s_cdc_retry_last_err = r;
        if (r == ESP_OK) {
            bridge_serial_set_transport(bridge_usb_host_transport());
            ESP_LOGI(TAG, "=== CDC-ACM OPENED via retry! bridge READY ===");
            continue;
        }
        ESP_LOGI(TAG, "cdc_retry: still no device (%s)", esp_err_to_name(r));
        /* 离线：短间隔重试；掉线通知把 500ms 等待压成 0，复位重枚举几乎下一轮就抓到 */
        xTaskNotifyWait(0, 0xFFFFFFFF, NULL, pdMS_TO_TICKS(500));
    }
}

/* — portal_mgr_task: WiFi 连启 bridge；断连开 AP+portal — */
static void portal_mgr_task(void *arg)
{
    uint16_t tcp_port = *(uint16_t *)arg;
    static char s_ap_ip[] = "192.168.4.1";

    bool services_started = false;
    int  s_fail_loops = 0;  /* 连续连接失败计数, ×2s = 实际等待时间 */
    #define BOOT_GRACE_LOOPS 4  /* 4×2s = 8s ≈ bridge_wifi_connect 超时 */

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(2000));
        bool connected = bridge_wifi_is_connected();

        if (connected) {
            s_fail_loops = 0;
            if (!services_started) {
                esp_err_t t1 = bridge_tcp_start(tcp_port);
                esp_err_t t2 = bridge_rfc2217_start(tcp_port + 1);
                esp_err_t t3 = bridge_mdns_start(tcp_port, tcp_port + 1);
                services_started = true;
                ESP_LOGI(TAG, "bridge: tcp=%s rfc2217=%s mdns=%s",
                         t1 == ESP_OK ? "on" : esp_err_to_name(t1),
                         t2 == ESP_OK ? "on" : esp_err_to_name(t2),
                         t3 == ESP_OK ? "on" : esp_err_to_name(t3));
            }
            /* UI update: WiFi IP (绿色) */
            ui_view_manager_set_wifi(UI_WIFI_STA_CONNECTED, bridge_wifi_get_ip_str());
            if (bridge_wifi_ap_is_active()) {
                bridge_wifi_ap_stop();
                ESP_LOGI(TAG, "WiFi recovered, SoftAP stopped — portal stays on http://airserial.local/");
            }
        } else {
            s_fail_loops++;

            /* 耐心窗口: 前 8 秒不开 AP, 等 STA async connect 自己超时 */
            if (s_fail_loops <= BOOT_GRACE_LOOPS && !bridge_wifi_ap_is_active()) {
                ESP_LOGI(TAG, "WiFi not connected yet (wait %d/%d) — keep red",
                         s_fail_loops, BOOT_GRACE_LOOPS);
                ui_view_manager_set_wifi(UI_WIFI_STA_DISCONNECTED, NULL);
                continue;
            }

            /* 超时了 — 开 AP + portal */
            if (!bridge_wifi_ap_is_active()) {
                ESP_LOGW(TAG, "WiFi offline — opening SoftAP %s", s_ap_ssid);
                bridge_wifi_ap_start(s_ap_ssid, NULL);
            }
            /* UI update: AP 已启动才显示黄色, 否则显示红色等待 */
            if (bridge_wifi_ap_is_active()) {
                ui_view_manager_set_wifi(UI_WIFI_AP, s_ap_ip);
            } else {
                ui_view_manager_set_wifi(UI_WIFI_STA_DISCONNECTED, NULL);
            }
            if (!bridge_web_portal_is_running()) {
                bridge_web_portal_start(s_ap_ssid, s_ap_ip);
                /* portal 起了就注册 USB Host 三个开关 endpoint */
                bridge_web_add_uri_handler(&s_host_on_uri);
                bridge_web_add_uri_handler(&s_host_off_uri);
                bridge_web_add_uri_handler(&s_host_status_uri);
                bridge_web_add_uri_handler(&s_debug_uri);
            }
        }
    }
}

/* — USB Host flag 加载 —
 *
 * 三态逻辑：
 *   NVS key 存在, val=1    → 用户手动开了 Host (尊重)
 *   NVS key 存在, val=0    → 用户手动关了 Host (尊重)
 *   NVS key 不存在         → 第一次开机！按 USB 拓扑自动决定默认值 + 写入 NVS
 *
 * 拓扑自动默认：
 *   INDEPENDENT → 默认 ON（有独立串口芯片，Host 模式 console 活）
 *   SHARED_PHY  → 默认 OFF（单口共享 PHY，Host 模式 console 断）
 *   NO_HOST     → 默认 OFF（根本不能跑 Host）
 *   UNKNOWN     → 默认 OFF（保守安全）
 */
static void load_usb_host_flag(void)
{
    nvs_handle_t h;
    esp_err_t nvs_ret = nvs_open(NVS_NS_DIAG, NVS_READONLY, &h);

    if (nvs_ret == ESP_OK) {
        uint8_t val = 0;
        esp_err_t get_ret = nvs_get_u8(h, NVS_KEY_USB_ON, &val);
        nvs_close(h);

        if (get_ret == ESP_OK) {
            /* key 存在 → 用户显式设置过，尊重 */
            s_usb_host_enabled = (val == 1);
            ESP_LOGI(TAG, "USB Host = %s (NVS user setting)", s_usb_host_enabled ? "ON" : "OFF");
            return;
        }
        /* ESP_ERR_NVS_NOT_FOUND → 第一次开机，继续走自动检测 */
    }

    /* —— 自动检测默认值 —— */
    board_usb_topo_t topo = board_get_usb_topo();
    s_usb_host_enabled = false;  /* 保守默认 */

    switch (topo) {
        case USB_TOPO_INDEPENDENT:
            s_usb_host_enabled = true;
            ESP_LOGI(TAG, "Auto-detect: INDEPENDENT topo → default Host ON ✅");
            break;
        case USB_TOPO_SHARED_PHY:
            ESP_LOGI(TAG, "Auto-detect: SHARED_PHY topo → default Host OFF (console disconnect risk)");
            break;
        case USB_TOPO_NO_HOST:
            ESP_LOGW(TAG, "Auto-detect: NO_HOST topo → Host unsupported, staying OFF");
            break;
        default:
            ESP_LOGW(TAG, "Auto-detect: UNKNOWN topo → conservative OFF");
            break;
    }

    /* 把自动检测结果写入 NVS，变成 sticky，用户 Portal 开关会覆盖 */
    if (nvs_open(NVS_NS_DIAG, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_u8(h, NVS_KEY_USB_ON, s_usb_host_enabled ? 1 : 0);
        nvs_commit(h);
        nvs_close(h);
        ESP_LOGI(TAG, "Auto-set default written to NVS: usb_host_on=%d", s_usb_host_enabled ? 1 : 0);
    }
}

/* — SHARED_PHY 板 PHY 归位（host_off 软重启后 Device 不枚举修复）—
 *
 * StickS3 等 SHARED_PHY 拓扑上 USB-OTG 与 USB-Serial-JTAG 共用同一个内置
 * FSLS PHY，归属由 RTC 域寄存器 sw_usb_phy_sel 决定（0=Serial-JTAG,
 * 1=OTG Wrap）。数字复位（esp_restart / 看门狗 / USB_UART_CHIP_RESET /
 * panic）不清 RTC 域：跑过 Host 后该位残留为 1，重启后 Serial-JTAG 拿不到
 * PHY → PC 不枚举，唯真断电(POR)可解。
 *
 * Device 模式启动时显式把 PHY 申请给 Serial-JTAG：内部会复位 USB WRAP
 *（清掉 Host 残留的 D+/D- 15K 下拉）+ 拨回 mux + 使能 pad 重新枚举。
 * POR 默认态本就归 Serial-JTAG，跳过以免无谓的 USB 闪断。handle 持有
 * 到关机，不释放（释放会关掉 wrap 时钟）。*/
static usb_phy_handle_t s_device_phy;

static void reclaim_usb_phy_for_device(void)
{
    if (board_get_usb_topo() != USB_TOPO_SHARED_PHY || s_usb_host_enabled) {
        return;
    }
    if (esp_rom_get_reset_reason(PRO_CPU_NUM) == RESET_REASON_CHIP_POWER_ON) {
        return;  /* 上电复位：mux 默认归 Serial-JTAG，无需干预 */
    }

    usb_phy_config_t cfg = {
        .controller  = USB_PHY_CTRL_SERIAL_JTAG,
        .target      = USB_PHY_TARGET_INT,
        /* 必须 DEFAULT：SERIAL_JTAG 控制器不允许 OTG set_mode，传 DEVICE 会 abort */
        .otg_mode    = USB_PHY_MODE_DEFAULT,
        .otg_speed   = USB_PHY_SPEED_UNDEFINED,
        .ext_io_conf = NULL,
        .otg_io_conf = NULL,
    };
    esp_err_t r = usb_new_phy(&cfg, &s_device_phy);
    if (r == ESP_OK) {
        ESP_LOGW(TAG, "USB PHY reclaimed for Serial-JTAG (non-POR reset, was held by OTG Host)");
    } else {
        ESP_LOGW(TAG, "USB PHY reclaim failed: %s — power cycle may be needed", esp_err_to_name(r));
    }
}

/* — 生成 SoftAP SSID（AirSerial-XXXX, XXXX = STA MAC 后 4 位）— */
static void make_ap_ssid(void)
{
    uint8_t mac[6] = {0};
    if (esp_wifi_get_mac(WIFI_IF_STA, mac) == ESP_OK) {
        snprintf(s_ap_ssid, sizeof(s_ap_ssid), "AirSerial-%02X%02X", mac[4], mac[5]);
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "AirSerial %s booting on %s (%s)",
             AIRSERIAL_VERSION,
             board_get_info()->name,
             board_get_info()->target);

    /* —— 1. NVS init + crash 恢复 —— */
    {
        esp_err_t ne = nvs_flash_init();
        if (ne == ESP_ERR_NVS_NO_FREE_PAGES || ne == ESP_ERR_NVS_NEW_VERSION_FOUND) {
            nvs_flash_erase();
            nvs_flash_init();
        }
        bridge_crash_try_load_past();
    }

    /* —— 2. board_init — boot_splash + 状态页 + USB 拓扑检测 —— */
    ESP_ERROR_CHECK(board_init());
    /* board_init 内部调了 board_detect_usb_topo()，现在 board_get_usb_topo() 有值了 */

    /* —— 3. USB Host flag —— */
    load_usb_host_flag();
#ifdef AIRSERIAL_FORCE_USB_HOST
    ESP_LOGW(TAG, "=== BUILD FLAG: Forcing USB Host ON ===");
    s_usb_host_enabled = true;
#endif

    /* —— 3.5 SHARED_PHY Device 模式：把 PHY 从 Host 残留态领回 Serial-JTAG —— */
    /* 必须在任何后续 USB 输出依赖之前；Host 模式时跳过（host_init 自己会配 PHY）*/
    reclaim_usb_phy_for_device();

    /* —— 3. bridge_core 子系统 —— */
    /* bridge_serial_init 必须有 transport，先传 dummy send，UART/USB Host 会热替换 */
    ESP_ERROR_CHECK(bridge_serial_init(&s_dummy_transport));

    /* —— 3b. UART transport（GPIO4/5, UART1, 115200 8N1）—— */
    /* 默认启用：简单可靠，不需要 USB Host 模式，CP2104/Downloader Kit 一插就通 */
    {
        esp_err_t u = bridge_uart_init();
        if (u == ESP_OK) {
            ESP_LOGI(TAG, "UART transport READY (GPIO4 TX / GPIO5 RX, 115200 8N1)");
        } else {
            ESP_LOGW(TAG, "UART init failed: %s — stay with dummy transport", esp_err_to_name(u));
        }
    }
    /* tcp/rfc2217/mdns 无 init，portal_mgr 任务里按连接状态 start */

    /* RFC2217 + TCP 共用统一的 client 协调器（portal pause/resume + WiFi 省电）
     *
     * Runtime 检测 PSRAM：esp_psram_is_initialized() → 有 PSRAM
     *   有 PSRAM → s_portal_autopause=false → portal 永不因 bridge client 而停
     *   无 PSRAM → s_portal_autopause=true  → 有 client 时停 portal 省 ~20KB
     * 用 esp_psram_is_initialized() 而非 heap_caps_get_free_size(MALLOC_CAP_SPIRAM)
     *   因为 SPIRAM_USE_MEMMAP=y 但 USE_MALLOC=n 时 heap_caps 看不到 PSRAM，
     *   而 esp_psram_is_initialized() 是 bootloader init 后就靠谱的硬件检测。*/
    s_portal_autopause = !esp_psram_is_initialized();
    ESP_LOGI(TAG, "portal auto-pause = %s (%s PSRAM, size=%d KB)",
             s_portal_autopause ? "ON" : "off",
             s_portal_autopause ? "no" : "has",
             esp_psram_get_size() / 1024);

    s_portal_resume_timer = xTimerCreate("portal_resume", pdMS_TO_TICKS(10000),
                                          pdFALSE, NULL, portal_resume_timer_cb);
    bridge_rfc2217_set_client_callback(bridge_client_cb, (void *)"rfc2217");
    bridge_tcp_set_client_callback(bridge_client_cb, (void *)"tcp");

    /* RFC2217 带外控制回调：SET_BAUDRATE → bridge_uart，DTR/RTS 占位 */
    bridge_rfc2217_set_ctrl_callback(rfc2217_ctrl_cb, NULL);

    /* —— 4. WiFi —— */
    esp_err_t wifi_ret = bridge_wifi_init();
    if (wifi_ret != ESP_OK) {
        ESP_LOGE(TAG, "WiFi init failed: %s", esp_err_to_name(wifi_ret));
    } else {
        /* DHCP hostname + SoftAP SSID：AirSerial-XXXX（XXXX = STA MAC 后 4 位）*/
        uint8_t mac[6] = {0};
        char hostname[32] = {0};
        esp_wifi_get_mac(WIFI_IF_STA, mac);
        snprintf(hostname, sizeof(hostname), "AirSerial-%02X%02X", mac[4], mac[5]);
        make_ap_ssid();  /* 复用同一 MAC 段 */

        esp_netif_t *sta_netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
        if (sta_netif) {
            esp_err_t hret = esp_netif_set_hostname(sta_netif, hostname);
            ESP_LOGI(TAG, "DHCP hostname: '%s' -> %s", hostname, esp_err_to_name(hret));
        }

        bridge_wifi_cred_t creds[BRIDGE_WIFI_MAX_CREDS];
        int nc = bridge_wifi_load_all_creds(creds, BRIDGE_WIFI_MAX_CREDS);

        if (nc > 0) {
            ESP_LOGI(TAG, "boot: %d saved creds, scan-first connect", nc);
            bridge_wifi_boot_connect_scan_first();
        } else {
            ESP_LOGI(TAG, "no saved creds → open SoftAP");
            ESP_ERROR_CHECK(bridge_wifi_ap_start(s_ap_ssid, NULL));
        }

        /* Portal + USB Host handlers — 无论 AP 还是 STA 模式都必须启动！
         * 之前的 bug：STA 模式有保存凭证时完全跳过 portal → HTTP server 挂了 */
        ESP_LOGI(TAG, "starting web portal + USB Host handlers");
        bridge_web_portal_start(s_ap_ssid, bridge_wifi_ap_is_active() ? "192.168.4.1" : bridge_wifi_get_ip_str());
        bridge_web_add_uri_handler(&s_host_on_uri);
        bridge_web_add_uri_handler(&s_host_off_uri);
        bridge_web_add_uri_handler(&s_host_status_uri);
        bridge_web_add_uri_handler(&s_debug_uri);
    }

    /* —— 5. USB Host —— */
    if (!s_usb_host_enabled) {
        ESP_LOGI(TAG, "USB Host init SKIPPED (default safe)");
        ESP_LOGI(TAG, "  → Device mode, USB-Serial-JTAG stays available");
    } else {
        ESP_LOGW(TAG, "=== USB Host ENABLED — switching PHY to OTG Host ===");
        if (board_usb_host_init_safe()) {
            ESP_LOGI(TAG, "  → INDEPENDENT topo: console survives PHY switch ✅");
        } else {
            ESP_LOGW(TAG, "  → SHARED_PHY topo: CONSOLE WILL DISCONNECT after usb_host_install() ⚠️");
            ESP_LOGW(TAG, "  → Last message before console dies...");
            vTaskDelay(pdMS_TO_TICKS(500));  /* 等日志刷出来 */
        }
        ESP_LOGW(TAG, "  → POST /api/usb/host_off + reboot to recover Device mode");

        if (bridge_usb_host_init() == ESP_OK) {
            bridge_usb_host_set_disconnect_callback(cdc_disconnect_cb, NULL);
            esp_err_t op = bridge_usb_host_open(5000);
            if (op == ESP_OK) {
                bridge_serial_set_transport(bridge_usb_host_transport());
                ESP_LOGI(TAG, "=== CDC-ACM device OPENED, bridge READY (Host transport) ===");
            } else {
                ESP_LOGW(TAG, "No CDC-ACM device at boot (%s) — UART stays active",
                         esp_err_to_name(op));
            }
            /* 无论 open 成功与否，都启动 retry task（兼容设备后插 / open 假阳性）*/
            BaseType_t ok = xTaskCreate(cdc_retry_task, "cdc_retry", 3072, NULL, 3, &s_cdc_retry_task_hdl);
            if (ok != pdPASS) ESP_LOGE(TAG, "cdc_retry task create failed");
        } else {
            ESP_LOGE(TAG, "USB Host init FAILED — stay with UART transport");
        }
    }

    /* —— UI update: Host 状态 —— */
    ui_view_manager_set_host_state(s_usb_host_enabled, bridge_usb_host_is_open() ? 1 : 0);

    /* —— 6. portal_mgr 任务 —— */
    {
        static uint16_t s_tcp_port = 3333;
        BaseType_t ok = xTaskCreate(portal_mgr_task, "portal_mgr", 4096,
                                    &s_tcp_port, 4, NULL);
        if (ok != pdPASS) {
            ESP_LOGE(TAG, "portal_mgr task create failed");
            if (bridge_wifi_is_connected()) {
                (void)bridge_tcp_start(s_tcp_port);
                (void)bridge_rfc2217_start(s_tcp_port + 1);
                (void)bridge_mdns_start(s_tcp_port, s_tcp_port + 1);
            }
        }
    }

    ESP_LOGI(TAG, "========== AirSerial boot complete ==========");
    ESP_LOGI(TAG, "  board:    %s (%s)", board_get_info()->name, board_get_info()->target);
    ESP_LOGI(TAG, "  wifi:     %s", bridge_wifi_is_connected() ? "connected" : "AP mode");
    ESP_LOGI(TAG, "  usb_host: %s", s_usb_host_enabled ? "ENABLED (risky)" : "OFF (safe default)");
    ESP_LOGI(TAG, "  monitor:  USB-Serial-JTAG available");
    ESP_LOGI(TAG, "==============================================");
}
