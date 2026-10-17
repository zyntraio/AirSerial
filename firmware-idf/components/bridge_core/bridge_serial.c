/*
 * bridge_serial —— 板无关双向串口数据管道（v0.1.4 / v0.1.5）。
 *
 * 纯核心模块：只依赖 FreeRTOS（xStreamBuffer + 任务通知），不含板级头。
 *
 *   目标板 -> [板级 VCP data_cb] -> on_rx() -> RX 环 -> 唯一读者
 *   生产者 -> write() -> TX 环 -> ser_tx 任务 -> 板级 send 回调
 *
 * v0.1.5 关键约束：FreeRTOS 流缓冲同一时刻只允许一个任务阻塞接收。
 * 因此读写双方都不在流缓冲上死等——先非阻塞收，空了才在任务通知上睡，
 * 数据到达由对端 xTaskNotify 唤醒。这样 flush 重置缓冲时绝不会有任务
 * 挂在缓冲上（v0.1.5 踢旧连接场景的 assert 根因）。
 *
 * RX 读者唯一：第一次调用 bridge_serial_read 的任务自动注册为 RX 读者。
 * TX 写者可能瞬时多个（新老连接交替），由 s_tx_lock 串行化入环。
 *
 * 背压：RX 环装不下整批时 on_rx 返回 false，数据留在 VCP 驱动自身
 * 缓冲（4KB）稍后重投，不做覆盖式丢字节。
 */
#include "bridge_serial.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/stream_buffer.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_heap_caps.h"

#define TAG "bridge_serial"

#define TX_TASK_STACK       4096
/* 单次 USB 发送粒度=一个 full-speed MPS（64B）。
 * S3 USB-Serial/JTAG 硬件只有单包 RX 缓冲：多包 URB 在设备忙时中途 NAK，
 * 拆成单包传输彻底规避多包重传/时序问题（v0.1.7 烧录实测路径）。 */
#define TX_MAX_CHUNK        64

/* v0.3.1 P0：主 RX 环已删除（fanout 订阅者机制取代，无调用者）。
 * 仅剩 TX 环，数据区放 PSRAM（内部 RAM 省 32KB），控制块留内部 RAM。 */
static StreamBufferHandle_t s_tx_ring;
static uint8_t *s_tx_storage;
static StaticStreamBuffer_t *s_tx_ctrl;
static SemaphoreHandle_t s_tx_lock;
static bridge_serial_transport_t s_transport;
static TaskHandle_t s_tx_task_hdl;
static volatile bool s_inited;

/* P0 端到端拥塞门：任一订阅者环空间低于高水位 → 关门，tx_task 停止从 TX 环
 * 抽数（=停止往物理 UART 送），TX 环涨满后 write() 阻塞 → TCP 接收窗口关闭 →
 * PC 停发，实现零丢字节背压。所有订阅者排空到低水位后开门（迟滞防抖）。
 * 在途上限：UART TX 驱动 8KB + FIFO 128B + UART RX 驱动已收未取 8KB ≈ 16.5KB；
 * 关门在入环后判定，48KB 环 / 20KB 水位留出 ≥16KB 余量。 */
#define CONGEST_HIGH_WATER   20480  /* 入环后剩余 < 20KB 关门 */
#define CONGEST_LOW_WATER    32768  /* 剩余 ≥ 32KB 开门（48KB 环，迟滞 12KB） */
static volatile bool s_congested;

static volatile size_t s_rx_bytes;       /* on_rx 进入管道的字节（≈UART 实收） */
static volatile size_t s_tx_bytes;
static volatile size_t s_rx_backpressure;  /* 拥塞门触发次数 */
static volatile size_t s_tx_drop;

/* 前向声明：init 里创建 stats task；拥塞门开门检查（定义在 on_rx 之后） */
static void stats_task_impl(void *arg);
static void congestion_maybe_clear_locked(void);

/* —— 订阅者表（v0.3.0）——
 * 固定容量数组，链表 free-list 管理槽位，避免 malloc 碎片。
 * on_rx 广播持 s_sub_lock 零超时入环，订阅者自己的任务在 sub_read 上取。 */
typedef struct bridge_serial_subscriber {
    char name[16];
    StreamBufferHandle_t ring;
    TaskHandle_t reader;
    volatile size_t drop_cnt;  /* 零超时入环失败计数 */
    uint8_t *storage;          /* 非 NULL=PSRAM 静态创建的数据区（NULL=动态分配） */
    void *ctrl;                /* 静态创建时的 StaticStreamBuffer_t 控制块（内部 RAM） */
    struct bridge_serial_subscriber *free_next;
} bridge_serial_subscriber_t;

static bridge_serial_subscriber_t s_sub_pool[BRIDGE_SERIAL_MAX_SUBSCRIBERS];
static bridge_serial_subscriber_t *s_sub_free;
static SemaphoreHandle_t s_sub_lock;

/* ser_tx：核心唯一的物理发送者。空环时睡在通知上，由 write 唤醒 */
static void tx_task(void *arg)
{
    (void)arg;
    uint8_t buf[TX_MAX_CHUNK];
    while (1) {
        /* 拥塞门关闭：订阅者（ser→TCP 泵）排空后会通知开门；
         * 1s 兜底唤醒复查标志，防止通知错位丢唤醒（看门狗式开门） */
        while (s_congested) {
            if (xTaskNotifyWait(0, ULONG_MAX, NULL, pdMS_TO_TICKS(1000)) == pdFALSE) {
                xSemaphoreTake(s_sub_lock, portMAX_DELAY);
                congestion_maybe_clear_locked();
                xSemaphoreGive(s_sub_lock);
            }
        }
        xSemaphoreTake(s_tx_lock, portMAX_DELAY);
        size_t n = xStreamBufferReceive(s_tx_ring, buf, sizeof(buf), 0);
        xSemaphoreGive(s_tx_lock);
        if (n == 0) {
            xTaskNotifyWait(0, ULONG_MAX, NULL, pdMS_TO_TICKS(1000));
            continue;
        }
        esp_err_t ret = s_transport.send(buf, n, s_transport.arg);
        if (ret == ESP_OK) {
            s_tx_bytes += n;
        } else {
            s_tx_drop += n;
            ESP_LOGW(TAG, "transport send failed: %s, drop %u bytes",
                     esp_err_to_name(ret), (unsigned)n);
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
}

esp_err_t bridge_serial_init(const bridge_serial_transport_t *tx_transport)
{
    if (tx_transport == NULL || tx_transport->send == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_inited) {
        return ESP_ERR_INVALID_STATE;
    }

    s_transport = *tx_transport;

    /* TX 环数据区优先 PSRAM 静态创建（32KB，内部 RAM 留给任务栈/lwIP），
     * 控制块留内部 RAM；无 PSRAM 回退动态分配 */
    s_tx_lock = xSemaphoreCreateMutex();
    s_sub_lock = xSemaphoreCreateMutex();
    s_tx_storage = heap_caps_malloc(BRIDGE_SERIAL_RING_SIZE, MALLOC_CAP_SPIRAM);
    s_tx_ctrl = pvPortMalloc(sizeof(StaticStreamBuffer_t));
    if (s_tx_storage != NULL && s_tx_ctrl != NULL) {
        s_tx_ring = xStreamBufferCreateStatic(BRIDGE_SERIAL_RING_SIZE, 1,
                                              s_tx_storage, s_tx_ctrl);
    }
    if (s_tx_ring == NULL) {
        if (s_tx_storage) { heap_caps_free(s_tx_storage); s_tx_storage = NULL; }
        if (s_tx_ctrl) { vPortFree(s_tx_ctrl); s_tx_ctrl = NULL; }
        s_tx_ring = xStreamBufferCreate(BRIDGE_SERIAL_RING_SIZE, 1);
    }
    if (s_tx_ring == NULL || s_tx_lock == NULL || s_sub_lock == NULL) {
        ESP_LOGE(TAG, "ring/lock create failed");
        return ESP_ERR_NO_MEM;
    }

    /* 订阅者 free-list：池数组首尾相连 */
    s_sub_free = &s_sub_pool[0];
    for (int i = 0; i < BRIDGE_SERIAL_MAX_SUBSCRIBERS - 1; i++) {
        s_sub_pool[i].free_next = &s_sub_pool[i + 1];
        s_sub_pool[i].ring = NULL;
    }
    s_sub_pool[BRIDGE_SERIAL_MAX_SUBSCRIBERS - 1].free_next = NULL;
    s_sub_pool[BRIDGE_SERIAL_MAX_SUBSCRIBERS - 1].ring = NULL;

    /* 不钉核（回滚 xTaskCreatePinnedToCore 实验） */
    BaseType_t ok = xTaskCreate(tx_task, "ser_tx", TX_TASK_STACK, NULL, 5, &s_tx_task_hdl);
    if (ok != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    s_inited = true;
    ESP_LOGI(TAG, "serial pipe ready (tx ring %d bytes, PSRAM)", BRIDGE_SERIAL_RING_SIZE);

    /* v0.3.1: 周期性 stats dump（背压监控基线） */
    xTaskCreate(stats_task_impl, "ser_stats", 2048, NULL, 1, NULL);

    return ESP_OK;
}

void bridge_serial_set_transport(const bridge_serial_transport_t *tx_transport)
{
    if (!tx_transport || !tx_transport->send) return;
    s_transport = *tx_transport;
    ESP_LOGI(TAG, "transport swapped (send=%p, arg=%p)", (void*)s_transport.send, s_transport.arg);
}

/* —— 周期性状态监控（v0.3.1 背压基线）—— */
static void stats_task_impl(void *arg)
{
    (void)arg;
    /* 记住上一次的计数器用于算增量 */
    size_t prev_rx = 0, prev_tx = 0;
    vTaskDelay(pdMS_TO_TICKS(15000));  /* 启动后 15s 开始打印 */
    while (1) {
        size_t rx, tx, bp, drop;
        bridge_serial_stats(&rx, &tx, &bp, &drop);
        size_t rx_delta = rx - prev_rx;
        size_t tx_delta = tx - prev_tx;
        prev_rx = rx; prev_tx = tx;

        bridge_serial_sub_stats_t subs[BRIDGE_SERIAL_MAX_SUBSCRIBERS];
        int n_subs = bridge_serial_sub_stats_all(subs, BRIDGE_SERIAL_MAX_SUBSCRIBERS);

        /* TX ring 使用率 + 拥塞门状态 */
        size_t tx_used = xStreamBufferBytesAvailable(s_tx_ring);

        ESP_LOGI(TAG, "STATS: rx=%lu tx=%lu gate=%d rxbp=%lu txdrop=%lu | "
                 "txring=%lu/%u",
                 (unsigned long)rx_delta, (unsigned long)tx_delta,
                 (int)s_congested, (unsigned long)bp, (unsigned long)drop,
                 (unsigned long)tx_used, BRIDGE_SERIAL_RING_SIZE);

        for (int i = 0; i < n_subs; i++) {
            ESP_LOGI(TAG, "  sub '%s': drop=%lu space=%lu/%lu",
                     subs[i].name, (unsigned long)subs[i].drop_cnt,
                     (unsigned long)subs[i].space_left,
                     (unsigned long)BRIDGE_SERIAL_SUB_RING_SIZE);
        }

        /* 堆余量轨迹：定位 blackout 后 xTaskCreate 持续 NO_MEM 是泄漏还是碎片化 */
        ESP_LOGI(TAG, "  heap: int free=%lu largest=%lu | psram free=%lu largest=%lu",
                 (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                 (unsigned long)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
                 (unsigned long)heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
                 (unsigned long)heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM));

        vTaskDelay(pdMS_TO_TICKS(10000));  /* 每 10s 一次 */
    }
}

bool bridge_serial_on_rx(const uint8_t *data, size_t len)
{
    if (!s_inited || data == NULL || len == 0) {
        return true;
    }

    /* UI hook: TERMINAL view 显示串口数据 — CDC 和 UART 都经过这里 */
    {
        extern void ui_view_manager_push_serial(const uint8_t *data, int len);
        ui_view_manager_push_serial(data, (int)len);
    }

    s_rx_bytes += len;

    /* 广播给所有订阅者（零超时入环）。任一订阅者剩余空间低于高水位即关拥塞门：
     * tx_task 停发 → TX 环满 → write() 阻塞 → TCP 窗口关 → PC 停发，
     * 在丢字节之前把背压传回源头（回环场景源头就是本设备自己的 TX）。 */
    if (s_sub_lock != NULL) {
        xSemaphoreTake(s_sub_lock, portMAX_DELAY);
        for (int i = 0; i < BRIDGE_SERIAL_MAX_SUBSCRIBERS; i++) {
            bridge_serial_subscriber_t *sub = &s_sub_pool[i];
            if (sub->ring == NULL) continue; /* 槽空闲 */
            size_t w = xStreamBufferSend(sub->ring, data, len, 0);
            if (w < len) {
                sub->drop_cnt++;
            }
            /* 入环后剩余空间低于水位才关门：本批已占空间计入，余量精确 */
            if (!s_congested &&
                xStreamBufferSpacesAvailable(sub->ring) < CONGEST_HIGH_WATER) {
                s_congested = true;
                s_rx_backpressure++;
            }
            if (sub->reader != NULL) {
                xTaskNotify(sub->reader, 0, eNoAction);
            }
        }
        xSemaphoreGive(s_sub_lock);
    }
    return true;
}

/* 检查开门条件：所有现存订阅者环剩余空间均 ≥ 低水位则开门并唤醒 tx_task。
 * 必须在持 s_sub_lock 时调用（与 on_rx 的关门检查互斥）。 */
static void congestion_maybe_clear_locked(void)
{
    if (!s_congested) return;
    for (int i = 0; i < BRIDGE_SERIAL_MAX_SUBSCRIBERS; i++) {
        bridge_serial_subscriber_t *sub = &s_sub_pool[i];
        if (sub->ring == NULL) continue;
        if (xStreamBufferSpacesAvailable(sub->ring) < CONGEST_LOW_WATER) {
            return; /* 仍有慢消费者，维持关门 */
        }
    }
    s_congested = false;
    if (s_tx_task_hdl != NULL) {
        xTaskNotify(s_tx_task_hdl, 0, eNoAction);
    }
}

size_t bridge_serial_write(const uint8_t *data, size_t len, uint32_t timeout_ms)
{
    if (!s_inited || data == NULL || len == 0) {
        return 0;
    }
    /* 绝不静默丢字节：在总超时预算内循环到全部入环为止。
     * 关键：等待 ring 空间必须在【锁外】进行——tx_task 抽数据前要拿同一把
     * s_tx_lock，若持锁阻塞等待，消费者永远拿不到锁 → 死锁。
     * 因此每次只持锁做一次非阻塞 Send，装不下就放锁 vTaskDelay 让 tx_task 腾空。*/
    size_t off = 0;
    TickType_t start = xTaskGetTickCount();
    TickType_t total = (timeout_ms == (uint32_t)portMAX_DELAY)
                       ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    while (off < len) {
        xSemaphoreTake(s_tx_lock, portMAX_DELAY);
        size_t w = xStreamBufferSend(s_tx_ring, data + off, len - off, 0);
        xSemaphoreGive(s_tx_lock);
        if (w > 0) {
            off += w;
            if (s_tx_task_hdl != NULL) {
                xTaskNotify(s_tx_task_hdl, 0, eNoAction);
            }
        }
        if (off < len) {
            if (total != portMAX_DELAY &&
                (int32_t)(total - (xTaskGetTickCount() - start)) <= 0) {
                break;
            }
            /* 锁外等待 tx_task 抽走数据，绝不在持锁状态下阻塞 */
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
    if (off < len) {
        s_tx_drop += len - off;
        return 0; /* 整批超时未入环，调用者按 stall 处理、不得推进偏移 */
    }
    return len;
}

void bridge_serial_flush(void)
{
    if (!s_inited) {
        return;
    }
    /* 任务都睡在任务通知而非流缓冲上，reset 不会撞到阻塞等待者 */
    xSemaphoreTake(s_tx_lock, portMAX_DELAY);
    xStreamBufferReset(s_tx_ring);
    xSemaphoreGive(s_tx_lock);

    /* PURGE 双向：同时清空所有订阅者环（ser→TCP 方向）并复查拥塞门 */
    xSemaphoreTake(s_sub_lock, portMAX_DELAY);
    for (int i = 0; i < BRIDGE_SERIAL_MAX_SUBSCRIBERS; i++) {
        if (s_sub_pool[i].ring != NULL) {
            xStreamBufferReset(s_sub_pool[i].ring);
        }
    }
    congestion_maybe_clear_locked();
    xSemaphoreGive(s_sub_lock);
}

void bridge_serial_stats(size_t *rx_total, size_t *tx_total,
                         size_t *rx_backpressure, size_t *tx_drop)
{
    if (rx_total) *rx_total = s_rx_bytes;
    if (tx_total) *tx_total = s_tx_bytes;
    if (rx_backpressure) *rx_backpressure = s_rx_backpressure;
    if (tx_drop) *tx_drop = s_tx_drop;
}

int bridge_serial_sub_stats_all(bridge_serial_sub_stats_t *out, int max_out)
{
    if (out == NULL || max_out <= 0) return 0;
    int filled = 0;
    xSemaphoreTake(s_sub_lock, portMAX_DELAY);
    for (int i = 0; i < BRIDGE_SERIAL_MAX_SUBSCRIBERS && filled < max_out; i++) {
        bridge_serial_subscriber_t *sub = &s_sub_pool[i];
        if (sub->ring == NULL) continue;
        out[filled].active = true;
        strncpy(out[filled].name, sub->name, 15);
        out[filled].name[15] = '\0';
        out[filled].drop_cnt = sub->drop_cnt;
        out[filled].space_left = xStreamBufferSpacesAvailable(sub->ring);
        filled++;
    }
    xSemaphoreGive(s_sub_lock);
    return filled;
}

/* —— 订阅者实现（v0.3.0）—— */

bridge_serial_subscriber_t *bridge_serial_subscribe(const char *name)
{
    if (!s_inited || s_sub_lock == NULL) {
        ESP_LOGE(TAG, "subscribe: not inited");
        return NULL;
    }

    xSemaphoreTake(s_sub_lock, portMAX_DELAY);
    bridge_serial_subscriber_t *sub = s_sub_free;
    if (sub == NULL) {
        xSemaphoreGive(s_sub_lock);
        ESP_LOGE(TAG, "subscribe: pool full (%d)", BRIDGE_SERIAL_MAX_SUBSCRIBERS);
        return NULL;
    }
    s_sub_free = sub->free_next; /* 从 free-list 摘出 */
    /* 数据区优先放 PSRAM：WiFi 初始化后内部 RAM 已无 16KB 连续块，
     * 第二个订阅者走内部动态分配必然失败（rfc2217 起不来）。
     * 控制块（小）留内部 RAM；无 PSRAM 的板子回退原动态分配。 */
    sub->storage = heap_caps_malloc(BRIDGE_SERIAL_SUB_RING_SIZE, MALLOC_CAP_SPIRAM);
    sub->ctrl = pvPortMalloc(sizeof(StaticStreamBuffer_t));
    if (sub->storage != NULL && sub->ctrl != NULL) {
        sub->ring = xStreamBufferCreateStatic(BRIDGE_SERIAL_SUB_RING_SIZE, 1,
                                              sub->storage,
                                              (StaticStreamBuffer_t *)sub->ctrl);
    }
    if (sub->ring == NULL) {
        if (sub->storage) { heap_caps_free(sub->storage); sub->storage = NULL; }
        if (sub->ctrl) { vPortFree(sub->ctrl); sub->ctrl = NULL; }
        sub->ring = xStreamBufferCreate(BRIDGE_SERIAL_SUB_RING_SIZE, 1);
    }
    if (sub->ring == NULL) {
        sub->free_next = s_sub_free;
        s_sub_free = sub;
        xSemaphoreGive(s_sub_lock);
        ESP_LOGE(TAG, "subscribe: ring alloc failed");
        return NULL;
    }
    sub->reader = NULL;
    sub->drop_cnt = 0;
    sub->free_next = NULL;
    if (name) {
        snprintf(sub->name, sizeof(sub->name), "%s", name);
    } else {
        snprintf(sub->name, sizeof(sub->name), "sub%p", sub);
    }
    xSemaphoreGive(s_sub_lock);
    ESP_LOGI(TAG, "subscribe '%s' ok (ring %d bytes)", sub->name, BRIDGE_SERIAL_SUB_RING_SIZE);
    return sub;
}

void bridge_serial_unsubscribe(bridge_serial_subscriber_t *sub)
{
    if (sub == NULL || sub->ring == NULL) return;

    xSemaphoreTake(s_sub_lock, portMAX_DELAY);
    vStreamBufferDelete(sub->ring);
    if (sub->storage != NULL) {
        /* PSRAM 静态创建：数据区与控制块需自行释放（动态分配由 Delete 释放） */
        heap_caps_free(sub->storage);
        vPortFree(sub->ctrl);
        sub->storage = NULL;
        sub->ctrl = NULL;
    }
    sub->ring = NULL;
    sub->reader = NULL;
    /* 放回 free-list 头部 */
    sub->free_next = s_sub_free;
    s_sub_free = sub;
    congestion_maybe_clear_locked();  /* 慢订阅者摘除后可能开门 */
    xSemaphoreGive(s_sub_lock);
    ESP_LOGI(TAG, "unsubscribe '%s' (drop_cnt=%u)", sub->name, (unsigned)sub->drop_cnt);
}

void bridge_serial_sub_reset(bridge_serial_subscriber_t *sub)
{
    if (sub == NULL || sub->ring == NULL) return;
    /* 与 on_rx/clear 检查互斥；reset 腾出空间后复查开门 */
    xSemaphoreTake(s_sub_lock, portMAX_DELAY);
    xStreamBufferReset(sub->ring);
    congestion_maybe_clear_locked();
    xSemaphoreGive(s_sub_lock);
}

size_t bridge_serial_sub_read(bridge_serial_subscriber_t *sub,
                              uint8_t *buf, size_t len, uint32_t timeout_ms)
{
    if (sub == NULL || sub->ring == NULL || buf == NULL || len == 0) {
        return 0;
    }

    /* 首个调用者自动注册为该订阅者的 reader，让 on_rx 能通知 */
    if (sub->reader == NULL) {
        sub->reader = xTaskGetCurrentTaskHandle();
    }

    size_t n = xStreamBufferReceive(sub->ring, buf, len, 0);
    if (n != 0) {
        /* 排空后持锁复查开门条件（慢消费者恢复时给 tx_task 开门） */
        xSemaphoreTake(s_sub_lock, portMAX_DELAY);
        congestion_maybe_clear_locked();
        xSemaphoreGive(s_sub_lock);
        return n;
    }

    (void)xTaskNotifyWait(0, ULONG_MAX, NULL, pdMS_TO_TICKS(timeout_ms));
    n = xStreamBufferReceive(sub->ring, buf, len, 0);
    if (n != 0) {
        xSemaphoreTake(s_sub_lock, portMAX_DELAY);
        congestion_maybe_clear_locked();
        xSemaphoreGive(s_sub_lock);
    }
    return n;
}
