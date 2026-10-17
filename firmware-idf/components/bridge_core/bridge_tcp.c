/* bridge_tcp 实现，见头文件注释。 */
#include "bridge_tcp.h"
#include "bridge_serial.h"

#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"

#define TAG "bridge_tcp"

#define BRIDGE_TASK_STACK  4096
#define PUMP_CHUNK         512

/* s_lock 保护 s_client_fd，且 fanout 发送 / 会话 close 期间持锁，
 * 保证 fd 生命周期内不会有另一个线程在同一 fd 上收发或关闭。 */
static SemaphoreHandle_t s_lock;
static int s_client_fd = -1;
static volatile size_t s_drop_noclient;
static volatile uint32_t s_accepts;
static volatile uint32_t s_kicks;
static volatile bool s_started;

/* 单会话退场门（与 bridge_rfc2217 同构）：blackout 中旧任务靠 send 超时/
 * keepalive 才能死，accept 踢旧后须等其退场再处理下一个连接，避免 4KB 栈
 * 任务在内部 RAM 上堆积。 */
static TaskHandle_t s_acc_task;
static TaskHandle_t s_sess_task;

/* v0.1.7: 客户端状态回调，供装配层协调 portal pause/resume */
typedef void (*tcp_client_cb_t)(bool connected, void *arg);
static tcp_client_cb_t s_client_cb = NULL;
static void *s_client_cb_arg = NULL;

void bridge_tcp_set_client_callback(tcp_client_cb_t cb, void *arg)
{
    s_client_cb = cb;
    s_client_cb_arg = arg;
}

bool bridge_tcp_has_client(void)
{
    return s_client_fd >= 0;
}

void bridge_tcp_get_stats(bridge_tcp_stats_t *out)
{
    if (out == NULL) {
        return;
    }
    out->accepts = s_accepts;
    out->kicks = s_kicks;
    out->dropped_no_client = s_drop_noclient;
}

/* subscriber: fanout 从独立 ring 读，不抢占主 RX 环 */
static bridge_serial_subscriber_t *s_tcp_sub;

/* 全局唯一的 串口->TCP 泵（从 subscriber ring 读）。
 * 无客户端时取出的字节无处可去，直接丢弃（等价"终端未接线"），
 * 保证 subscriber 环不被塞满。 */
static void ser_to_tcp_fanout(void *arg)
{
    (void)arg;
    uint8_t buf[PUMP_CHUNK];
    while (1) {
        if (s_tcp_sub == NULL) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        size_t n = bridge_serial_sub_read(s_tcp_sub, buf, sizeof(buf), 5000);
        if (n == 0) {
            continue;
        }

        /* 锁里只拷 fd，send 放锁外——send 可能阻塞，绝不能持锁 */
        int fd;
        xSemaphoreTake(s_lock, portMAX_DELAY);
        fd = s_client_fd;
        xSemaphoreGive(s_lock);

        if (fd < 0) {
            s_drop_noclient += n;
            continue;
        }

        size_t off = 0;
        bool fail = false;
        while (off < n) {
            /* 阻塞 send — 锁外，不影响其他路径。背压正确传回 PC */
            int w = send(fd, buf + off, n - off, 0);
            if (w < 0) {
                fail = true;
                break;
            }
            off += w;
        }
        if (fail) {
            /* 客户端已死：持锁标记 fd=-1，让后续 accept/tcp2ser 知道 */
            xSemaphoreTake(s_lock, portMAX_DELAY);
            if (s_client_fd == fd) {
                s_client_fd = -1;
            }
            xSemaphoreGive(s_lock);
            shutdown(fd, SHUT_RDWR);
            s_drop_noclient += n - off;
        }
    }
}

/* PC -> 目标板，每连接一个；参数为 fd 副本 */
static void tcp_to_ser_task(void *arg)
{
    int fd = (int)(intptr_t)arg;
    uint8_t buf[PUMP_CHUNK];
    while (1) {
        /* 阻塞 socket：直接 recv，lwIP 自动处理 flow control */
        int n = recv(fd, buf, sizeof(buf), 0);
        if (n <= 0) break;
        size_t off = 0;
        /* 目标侧忙（如 esptool 擦写 flash）时入环会长时间满。
         * 绝不丢字节：长等待让背压经 TCP 窗口传回 PC，esptool 自行减速。
         * 只有连续 60s 推不进（目标彻底假死）才断会话。 */
        int stall_rounds = 0;
        while (off < (size_t)n) {
            size_t w = bridge_serial_write(buf + off, n - off, 10000);
            if (w == 0) {
                if (++stall_rounds >= 6) {
                    ESP_LOGE(TAG, "serial TX ring stalled 60s, abort session");
                    goto recv_done;
                }
                size_t st_rx, st_tx, st_bp, st_drop;
                bridge_serial_stats(&st_rx, &st_tx, &st_bp, &st_drop);
                ESP_LOGW(TAG, "serial TX ring full, backpressure %d/6 "
                         "(ser rx=%u tx=%u rxbp=%u txdrop=%u)",
                         stall_rounds, (unsigned)st_rx, (unsigned)st_tx,
                         (unsigned)st_bp, (unsigned)st_drop);
                continue;
            }
            stall_rounds = 0;
            off += w;
        }
    }
recv_done: ;
    /* 退场：若自己仍是当前会话则注销并 shutdown；持锁 close 与 fanout 串行 */
    xSemaphoreTake(s_lock, portMAX_DELAY);
    bool active = (s_client_fd == fd);
    if (active) {
        s_client_fd = -1;
    }
    if (s_sess_task == xTaskGetCurrentTaskHandle()) {
        s_sess_task = NULL;
    }
    shutdown(fd, SHUT_RDWR);
    close(fd);
    xSemaphoreGive(s_lock);

    /* 通知 accept 单会话槽已释放（通知位锁存，值带退场任务句柄） */
    if (s_acc_task != NULL) {
        xTaskNotify(s_acc_task,
                    (uint32_t)(intptr_t)xTaskGetCurrentTaskHandle(),
                    eSetValueWithOverwrite);
    }

    /* 我是最后一个活跃会话（没有被新连接踢掉），通知装配层：客户端走了 */
    if (active && s_client_cb) {
        s_client_cb(false, s_client_cb_arg);
    }
    size_t st_rx, st_tx, st_bp, st_drop;
    bridge_serial_stats(&st_rx, &st_tx, &st_bp, &st_drop);
    ESP_LOGI(TAG, "tcp2ser fd=%d exited (active=%d): ser rx=%u tx=%u rxbp=%u txdrop=%u",
             fd, (int)active, (unsigned)st_rx, (unsigned)st_tx,
             (unsigned)st_bp, (unsigned)st_drop);
    vTaskDelete(NULL);
}

static void accept_task(void *arg)
{
    uint16_t port = (uint16_t)(intptr_t)arg;

    int ls = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (ls < 0) {
        ESP_LOGE(TAG, "socket() errno %d", errno);
        vTaskDelete(NULL);
    }
    int one = 1;
    setsockopt(ls, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_port = htons(port),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    if (bind(ls, (struct sockaddr *)&addr, sizeof(addr)) != 0
        || listen(ls, 2) != 0) {
        ESP_LOGE(TAG, "bind/listen(%u) errno %d", port, errno);
        close(ls);
        vTaskDelete(NULL);
    }
    ESP_LOGI(TAG, "raw serial bridge listening on 0.0.0.0:%u", port);

    while (1) {
        struct sockaddr_in src;
        socklen_t slen = sizeof(src);
        int fd = accept(ls, (struct sockaddr *)&src, &slen);
        if (fd < 0) {
            ESP_LOGW(TAG, "accept errno %d, retry", errno);
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
        /* keepalive：WiFi 静默切换/客户端失踪时让死连接最终被 lwIP 回收，
         * 避免 tcp2ser 永远挂在 recv 上占住唯一会话槽 */
        setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &one, sizeof(one));
        /* 30s 发不出才按死连接拆除（blackout 楔形时泵任务不能永久冻死；
         * 不能更短——拥塞门下合法慢客户端的 send 可阻塞 10s+） */
        struct timeval snd_to;
        snd_to.tv_sec = 30;
        snd_to.tv_usec = 0;
        setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &snd_to, sizeof(snd_to));
        /* keepalive 10s 起探 / 2s 间隔 / 3 次失败 */
        {
            int ka_idle = 10, ka_intvl = 2, ka_cnt = 3;
            setsockopt(fd, IPPROTO_TCP, TCP_KEEPIDLE, &ka_idle, sizeof(ka_idle));
            setsockopt(fd, IPPROTO_TCP, TCP_KEEPINTVL, &ka_intvl, sizeof(ka_intvl));
            setsockopt(fd, IPPROTO_TCP, TCP_KEEPCNT, &ka_cnt, sizeof(ka_cnt));
        }

        /* 设 SO_RCVBUF 让 lwIP TCP 窗口有界且及时更新 */
        {
            int rcvbuf = 16384;
            setsockopt(fd, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));
        }

        /* 踢旧连接：登记新 fd，shutdown 旧 fd；旧 tcp2ser 的 recv 立即返回，
         * 其退场在锁内 close（那时 fanout 已不可能再用旧 fd） */
        s_accepts++;
        xSemaphoreTake(s_lock, portMAX_DELAY);
        int old_fd = s_client_fd;
        TaskHandle_t old_task = s_sess_task;
        s_client_fd = fd;
        if (old_fd >= 0) {
            shutdown(old_fd, SHUT_RDWR);
            s_kicks++;
        }
        xSemaphoreGive(s_lock);

        if (old_fd >= 0) {
            ESP_LOGI(TAG, "client %s kicks old fd=%d, new fd=%d",
                     inet_ntoa(src.sin_addr), old_fd, fd);
        } else {
            ESP_LOGI(TAG, "client %s:%u connected (fd=%d)",
                     inet_ntoa(src.sin_addr), ntohs(src.sin_port), fd);
        }

        /* 通知装配层：现在有活跃客户端了（portal 可以停省 RAM） */
        if (s_client_cb) s_client_cb(true, s_client_cb_arg);

        char tname[configMAX_TASK_NAME_LEN];
        snprintf(tname, sizeof(tname), "tcp2ser%d", fd % 100);
        TaskHandle_t h = NULL;
        if (xTaskCreate(tcp_to_ser_task, tname, BRIDGE_TASK_STACK,
                        (void *)(intptr_t)fd, 5, &h) != pdPASS) {
            ESP_LOGE(TAG, "tcp2ser create failed, reject fd=%d", fd);
            xSemaphoreTake(s_lock, portMAX_DELAY);
            if (s_client_fd == fd) s_client_fd = -1;
            close(fd);
            xSemaphoreGive(s_lock);
        } else {
            s_sess_task = h;
        }

        /* 单会话门：等旧任务真正退场再回 accept（结构同 bridge_rfc2217） */
        if (old_task != NULL) {
            TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(35000);
            bool gone = false;
            while (!gone) {
                xSemaphoreTake(s_lock, portMAX_DELAY);
                if (s_sess_task != old_task) gone = true;
                xSemaphoreGive(s_lock);
                TickType_t remain = deadline - xTaskGetTickCount();
                if (gone || (int32_t)remain <= 0) break;
                uint32_t exited = 0;
                if (xTaskNotifyWait(0, ULONG_MAX, &exited, remain) == pdTRUE
                    && exited == (uint32_t)(intptr_t)old_task) {
                    gone = true;
                }
            }
            if (!gone) {
                ESP_LOGW(TAG, "old session task %p did not exit in 35s (blackout?)",
                         (void *)old_task);
            }
        }
    }
}

esp_err_t bridge_tcp_start(uint16_t port)
{
    if (s_started) {
        ESP_LOGI(TAG, "already started on port %u", port);
        return ESP_OK;
    }
    if (s_lock == NULL) {
        s_lock = xSemaphoreCreateMutex();
        if (s_lock == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }
    /* v0.3.1: subscribe for multi-client fanout (TCP + RFC2217 + WS all parallel) */
    if (s_tcp_sub == NULL) {
        s_tcp_sub = bridge_serial_subscribe("tcp");
        if (s_tcp_sub == NULL) {
            ESP_LOGE(TAG, "subscribe tcp failed");
            return ESP_ERR_NO_MEM;
        }
    }
    BaseType_t ok1 = xTaskCreate(accept_task, "bridge_tcp",
                                 BRIDGE_TASK_STACK, (void *)(intptr_t)port, 5, &s_acc_task);
    BaseType_t ok2 = xTaskCreate(ser_to_tcp_fanout, "ser2tcp",
                                 BRIDGE_TASK_STACK, NULL, 5, NULL);
    if (ok1 == pdPASS && ok2 == pdPASS) {
        s_started = true;
        return ESP_OK;
    }
    return ESP_ERR_NO_MEM;
}
