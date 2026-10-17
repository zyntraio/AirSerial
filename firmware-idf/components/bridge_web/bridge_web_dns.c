/*
 * bridge_web_dns —— 最小 captive DNS（v0.3.1）。
 *
 * UDP 53 上把所有 A 查询应答为热点 IP：手机/电脑连上热点后，
 * 无论访问什么域名都会被指到设备配网页。
 *
 * 只做一件事，不求完整 DNS 语义：
 *  - A 查询（type=1）→ 单条 A 记录应答
 *  - 其他类型（AAAA 等）→ 无 answer 应答（rcode=0）
 *  - 响应包（qr=1）/格式异常包 → 忽略
 *
 * 仅 portal 运行期间存活；STA 正常时不开（不干扰正常域名解析）。
 */
#include "bridge_web_dns.h"

#include "esp_log.h"
#include "lwip/sockets.h"
#include "lwip/ip4_addr.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <string.h>
#include <stdint.h>

#define TAG "bridge_web_dns"

#define DNS_PORT        53
#define DNS_BUF_LEN     512
#define DNS_TASK_STACK  3584
#define DNS_TTL         60      /* 应答 TTL（秒），短一点便于关闭 portal 后收敛 */

static volatile bool s_dns_running;
static volatile bool s_dns_quit;
static int           s_dns_sock = -1;
static uint32_t      s_dns_ip_u32;    /* 网络序热点 IP */

/* task 句柄仅用于 stop 时置 quit 位（lwip socket 无线程安全 close，
 * 由 task 自己 close 再退出） */
static TaskHandle_t s_dns_task_hdl;

static void dns_task(void *arg)
{
    (void)arg;
    uint8_t rx[DNS_BUF_LEN];
    uint8_t tx[DNS_BUF_LEN];

    struct sockaddr_in addr = { 0 };
    addr.sin_family = AF_INET;
    addr.sin_port = htons(DNS_PORT);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);

    s_dns_sock = lwip_socket(AF_INET, SOCK_DGRAM, 0);
    if (s_dns_sock < 0) {
        ESP_LOGE(TAG, "socket failed");
        s_dns_running = false;
        vTaskDelete(NULL);
        return;
    }

    /* 500ms 收包超时：让循环有机会检查 quit 标志 */
    struct timeval tv = { .tv_sec = 0, .tv_usec = 500 * 1000 };
    setsockopt(s_dns_sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    if (lwip_bind(s_dns_sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "bind :53 failed (already in use?)");
        lwip_close(s_dns_sock);
        s_dns_sock = -1;
        s_dns_running = false;
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "captive DNS on :53 -> %u.%u.%u.%u",
             (s_dns_ip_u32 >> 24) & 0xFF, (s_dns_ip_u32 >> 16) & 0xFF,
             (s_dns_ip_u32 >> 8) & 0xFF, s_dns_ip_u32 & 0xFF);

    while (!s_dns_quit) {
        struct sockaddr_in from;
        socklen_t fromlen = sizeof(from);
        int n = lwip_recvfrom(s_dns_sock, rx, sizeof(rx), 0,
                              (struct sockaddr *)&from, &fromlen);
        if (n < 12) {
            continue;   /* 超时/短包：0x0000 无法构造，丢 */
        }

        /* DNS 头：id(2) flags(2) qd(2) an(2) ns(2) ar(2) */
        uint16_t flags = ((uint16_t)rx[2] << 8) | rx[3];
        if (flags & 0x8000) continue;                 /* 响应包，忽略 */
        uint16_t qdcount = ((uint16_t)rx[4] << 8) | rx[5];
        if (qdcount < 1) continue;

        /* 跳过第一个 question 的 qname（标签序列，遇 0 结束） */
        int pos = 12;
        while (pos < n && rx[pos] != 0) {
            pos += rx[pos] + 1;
        }
        pos += 1;                                     /* 结尾 0 */
        if (pos + 4 > n) continue;
        uint16_t qtype = ((uint16_t)rx[pos] << 8) | rx[pos + 1];
        int qend = pos + 4;                           /* question 段结束（含 qtype/qclass） */

        /* 组应答：头 + 原 question + (A 时) 单条 answer */
        int tl = 0;
        memcpy(tx, rx, qend);                         /* id/flags/qd + question 原样回 */
        tx[2] = 0x81; tx[3] = 0x80;                   /* QR=1 RD=1 RA=1 rcode=0 */
        int an = (qtype == 1) ? 1 : 0;                /* type A=1 */
        tx[6] = 0; tx[7] = an;
        tx[8] = 0; tx[9] = 0; tx[10] = 0; tx[11] = 0;
        tl = qend;

        if (an) {
            tx[tl++] = 0xC0; tx[tl++] = 0x0C;         /* name 指针 → 偏移 12 */
            tx[tl++] = 0x00; tx[tl++] = 0x01;         /* type A */
            tx[tl++] = 0x00; tx[tl++] = 0x01;         /* class IN */
            tx[tl++] = 0x00; tx[tl++] = 0x00;
            tx[tl++] = 0x00; tx[tl++] = DNS_TTL;
            tx[tl++] = 0x00; tx[tl++] = 0x04;         /* rdlength */
            tx[tl++] = (s_dns_ip_u32 >> 24) & 0xFF;
            tx[tl++] = (s_dns_ip_u32 >> 16) & 0xFF;
            tx[tl++] = (s_dns_ip_u32 >> 8) & 0xFF;
            tx[tl++] = s_dns_ip_u32 & 0xFF;
        }

        lwip_sendto(s_dns_sock, tx, tl, 0,
                    (struct sockaddr *)&from, fromlen);
    }

    lwip_close(s_dns_sock);
    s_dns_sock = -1;
    s_dns_running = false;
    ESP_LOGI(TAG, "captive DNS off");
    vTaskDelete(NULL);
}

esp_err_t bridge_web_dns_start(const char *ap_ip)
{
    if (s_dns_running) {
        return ESP_ERR_INVALID_STATE;
    }
    /* "192.168.4.1" → host 序 u32。
     * ip4addr_aton 返回网络序(大端)→ 转 host 序(s_dns_ip_u32 用于日志拆字节,
     * 而 lwIP 在 ESP32-P4(RISC-V 小端)下读的是 host 序,所以 RDATA 填 host 序)。
     * 之前直接当网络序用,RDATA 输出 0x0104A8C0 → 日志看到 1.4.168.192,错误! */
    ip_addr_t dummy;
    if (!ip4addr_aton(ap_ip, (ip4_addr_t *)&dummy)) {
        return ESP_ERR_INVALID_ARG;
    }
    s_dns_ip_u32 = lwip_htonl(((ip4_addr_t *)&dummy)->addr);  /* 网络序 → host 序 */

    s_dns_quit = false;
    if (xTaskCreate(dns_task, "web_dns", DNS_TASK_STACK, NULL, 4, &s_dns_task_hdl) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    s_dns_running = true;
    return ESP_OK;
}

esp_err_t bridge_web_dns_stop(void)
{
    if (!s_dns_running) {
        return ESP_OK;
    }
    s_dns_quit = true;
    /* 等 task 自己 close socket 并退出（最多 ~1s，收包超时 500ms） */
    for (int i = 0; i < 20 && s_dns_running; i++) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    if (s_dns_running) {
        ESP_LOGW(TAG, "dns task not exited in 1s, leaking");
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}
