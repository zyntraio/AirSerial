/*
 * bridge_uart.c —— UART1 transport for AirSerial bridge
 *
 * 数据流：
 *   bridge_serial_write() → tx_task → bridge_uart_send() → uart_write_bytes → GPIO4 TX
 *   GPIO5 RX → RX task read → bridge_serial_on_rx() → bridge 管道 → TCP/RFC2217
 *
 * 默认 pin: TX=GPIO4, RX=GPIO5（UART1 IOMUX 直连，性能最好）
 * 默认参数: 115200 8N1, no flow control
 *
 * 和 bridge_usb_host 完全平行：提供 send_fn + transport struct
 */
#include "bridge_uart.h"
#include "bridge_serial.h"

#include "driver/uart.h"
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include <string.h>

#define TAG "bridge_uart"

#ifndef BRIDGE_UART_NUM
#define BRIDGE_UART_NUM  UART_NUM_1
#endif

#define BRIDGE_UART_QUEUE_SIZE  20
#define BRIDGE_UART_RX_STACK    4096
#define BRIDGE_UART_TX_BUF      8192
#define BRIDGE_UART_RX_BUF      4096

static bridge_uart_params_t s_params = {
    .baud_rate = BRIDGE_UART_DEFAULT_BAUD,
    .data_bits = 8,
    .stop_bits = 1,
    .parity    = 0,
    .gpio_tx   = BRIDGE_UART_DEFAULT_TX_PIN,
    .gpio_rx   = BRIDGE_UART_DEFAULT_RX_PIN,
};

static volatile bool s_inited;
static volatile bool s_ready;

/* RX 侧确定性诊断：UART 实际收到字节数 / 硬件 FIFO 溢出 / 驱动 ring 满（字节真丢失点） */
static QueueHandle_t s_uart_evt_queue;
static volatile size_t s_dbg_rx_bytes;
static volatile size_t s_dbg_fifo_ovf;
static volatile size_t s_dbg_buf_full;
static volatile size_t s_dbg_frame_err;
static volatile size_t s_dbg_parity_err;
static volatile size_t s_dbg_break;

/* —— bridge_serial transport send —— */
static esp_err_t uart_send_fn(const uint8_t *d, size_t l, void *a)
{
    (void)a;
    if (!s_ready) return ESP_ERR_INVALID_STATE;
    if (d == NULL || l == 0) return ESP_OK;
    int written = uart_write_bytes(BRIDGE_UART_NUM, d, l);
    if (written < 0) {
        ESP_LOGW(TAG, "uart_write_bytes failed");
        return ESP_FAIL;
    }
    if ((size_t)written != l) {
        ESP_LOGW(TAG, "uart_write_bytes partial: %d/%u", written, (unsigned)l);
        return ESP_ERR_TIMEOUT;
    }
    /* 同步等 TX FIFO 排空（64B@921600≈0.7ms，即线速）：让 tx_task 以线速
     * 节拍发送，而不是把 8KB 驱动 TX ring 瞬间塞满。实测在拥塞门开关交替的
     * 大流量停滞场景下，纯异步写会在零错误计数情况下丢失 23-31KB（TX→RX），
     * 改为逐块等 FIFO 排空后 rfc_w==uart_rx 全量平衡。线速等待不增加功耗
     * （任务阻塞等待，不轮询）。 */
    esp_err_t w = uart_wait_tx_done(BRIDGE_UART_NUM, pdMS_TO_TICKS(200));
    if (w != ESP_OK) {
        static volatile size_t s_dbg_tx_wait_to;
        s_dbg_tx_wait_to++;
        if (s_dbg_tx_wait_to <= 5) {
            ESP_LOGW(TAG, "uart_wait_tx_done timeout #%u", (unsigned)s_dbg_tx_wait_to);
        }
    }
    return ESP_OK;
}

static const bridge_serial_transport_t s_uart_transport = {
    .send = uart_send_fn,
    .arg  = NULL,
};

/* —— RX task：事件驱动，UART_DATA 到达即读即灌 bridge_serial_on_rx ——
 * 不可用 uart_read_bytes(sizeof, 100ms) 轮询：它凑不满长度就等满超时，
 * 每个小包被强行批处理 ~100ms，恰好顶死 esptool 0.1s SYNC 窗口（35/35 全败）。
 * ISR 在 FIFO 阈值(64B)或空闲超时(~2字符)时即投递 DATA 事件，延迟 <1ms */
static void uart_rx_task(void *arg)
{
    (void)arg;
    /* static：4KB 接收缓冲不占任务栈（栈仅 4KB，放栈上必溢出）；本任务单实例 */
    static uint8_t rx_buf[BRIDGE_UART_RX_BUF];
    ESP_LOGI(TAG, "RX task started (UART%d, TX=GPIO%d, RX=GPIO%d)",
             BRIDGE_UART_NUM, s_params.gpio_tx, s_params.gpio_rx);

    while (1) {
        uart_event_t ev;
        /* 等事件；100ms 超时兜底防队列溢出丢事件导致停摆 */
        if (s_uart_evt_queue == NULL ||
            xQueueReceive(s_uart_evt_queue, &ev, pdMS_TO_TICKS(100)) != pdTRUE) {
            ev.type = UART_EVENT_MAX;
        }
        /* 统计真正的字节丢失事件 */
        switch (ev.type) {
        case UART_FIFO_OVF:    s_dbg_fifo_ovf++;    break;
        case UART_BUFFER_FULL: s_dbg_buf_full++;    break;
        case UART_FRAME_ERR:   s_dbg_frame_err++;   break;
        case UART_PARITY_ERR:  s_dbg_parity_err++;  break;
        case UART_BREAK:       s_dbg_break++;       break;
        default: break;
        }
        /* 任何事件（DATA/溢出/兜底超时）都把现存字节排空 */
        int len = uart_read_bytes(BRIDGE_UART_NUM, rx_buf, sizeof(rx_buf), 0);
        while (len > 0) {
            s_dbg_rx_bytes += (size_t)len;
            /* 主 RX 环无读者时会返回 false，但 fanout 订阅者已收到本批，忽略返回值；
             * 绝不在此打日志——921600 突发期每批一条同步日志会卡住本任务撑爆驱动 ring */
            (void)bridge_serial_on_rx(rx_buf, (size_t)len);
            len = uart_read_bytes(BRIDGE_UART_NUM, rx_buf, sizeof(rx_buf), 0);
        }
    }
}

void bridge_uart_rx_debug(size_t *bytes, size_t *fifo_ovf, size_t *buf_full,
                          size_t *frame_err, size_t *parity_err, size_t *brk)
{
    if (bytes)     *bytes     = s_dbg_rx_bytes;
    if (fifo_ovf)  *fifo_ovf  = s_dbg_fifo_ovf;
    if (buf_full)  *buf_full  = s_dbg_buf_full;
    if (frame_err) *frame_err = s_dbg_frame_err;
    if (parity_err)*parity_err= s_dbg_parity_err;
    if (brk)       *brk       = s_dbg_break;
}

/* —— UART 初始化 ——
 * run_loopback_test：8KB 硬件回环自检只在初始上电跑一次。动态切参（esptool
 * RFC2217 SET_BAUDRATE 等）绝不重跑——自检里 uart_read_bytes 在 TX/RX 未短接
 * 时必等满 1s 超时，会阻塞调用者（rfc2217 参数回执）远超 pyserial 的 3s
 * network_timeout，esptool 直接 "Remote does not accept parameter change"。 */
static esp_err_t uart_config_and_open(bool run_loopback_test)
{
    uart_parity_t parity_map[] = { UART_PARITY_DISABLE, UART_PARITY_ODD, UART_PARITY_EVEN };
    uart_config_t cfg = {
        .baud_rate  = s_params.baud_rate,
        .data_bits  = (uart_word_length_t)(s_params.data_bits - 5), /* UART_DATA_5_BITS=0..UART_DATA_8_BITS=3 */
        .parity     = parity_map[s_params.parity],                   /* 0→DISABLE, 1→ODD, 2→EVEN */
        .stop_bits  = s_params.stop_bits == 2 ? UART_STOP_BITS_2 : UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
    };

    uart_driver_delete(BRIDGE_UART_NUM);
    /* 传 &queue：驱动内部创建事件队列并回填句柄（rx 任务在 install 后才启动） */
    s_uart_evt_queue = NULL;
    esp_err_t ret = uart_driver_install(BRIDGE_UART_NUM,
                                         BRIDGE_UART_RX_BUF * 2,
                                         BRIDGE_UART_TX_BUF,
                                         BRIDGE_UART_QUEUE_SIZE,
                                         &s_uart_evt_queue, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = uart_param_config(BRIDGE_UART_NUM, &cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config failed: %s", esp_err_to_name(ret));
        uart_driver_delete(BRIDGE_UART_NUM);
        return ret;
    }

    ret = uart_set_pin(BRIDGE_UART_NUM,
                       s_params.gpio_tx,
                       s_params.gpio_rx,
                       UART_PIN_NO_CHANGE,
                       UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_pin failed: %s", esp_err_to_name(ret));
        uart_driver_delete(BRIDGE_UART_NUM);
        return ret;
    }

    /* RX FIFO 满阈值 120→64：921600 下把 ISR 响应余量从 ~87us 提到 ~700us，
     * 配合 CONFIG_UART_ISR_IN_IRAM（ISR 不因 flash/cache 操作被延迟）避免 FIFO 溢出丢字节 */
    uart_set_rx_full_threshold(BRIDGE_UART_NUM, 64);

    /* 清 FIFO */
    uart_flush(BRIDGE_UART_NUM);

    /* 硬件回环深度自检（仅 init 时）：连续发 8KB 看能否完全收回来。
     * 连续大流量通过才说明 UART FIFO / driver buffer / GPIO 都 OK */
    if (run_loopback_test) {
        uint8_t *big = heap_caps_malloc(8192, MALLOC_CAP_DMA);
        if (big) {
            for (int i = 0; i < 8192; i++) big[i] = (uint8_t)(i * 7 + 13);
            int wrote = uart_write_bytes(BRIDGE_UART_NUM, big, 8192);
            /* 等发完：115200bps 发 8KB ≈ 700ms */
            uart_wait_tx_done(BRIDGE_UART_NUM, pdMS_TO_TICKS(2000));
            int got = uart_read_bytes(BRIDGE_UART_NUM, big, 8192, pdMS_TO_TICKS(1000));
            ESP_LOGI(TAG, "HW loopback big-test: wrote=%d got=%d/8192", wrote, got);
            if (got < 8192) {
                /* UART 是兜底路径，TX/RX 未短接时自检失败属预期——降级为日志，禁止杀整机 */
                ESP_LOGE(TAG, "HW LOOPBACK FAIL — only got %d/8192 (TX/RX not shorted? continuing anyway)", got);
            }
            heap_caps_free(big);
        } else {
            ESP_LOGW(TAG, "HW loopback big-test SKIP — DMA alloc failed");
        }
    }
    return ESP_OK;
}

/* —— 公共 API —— */
esp_err_t bridge_uart_init(void)
{
    return bridge_uart_init_custom(NULL);
}

esp_err_t bridge_uart_init_custom(const bridge_uart_params_t *params)
{
    if (s_inited) {
        ESP_LOGW(TAG, "already inited, ignore");
        return ESP_OK;
    }

    /* 可选：拷贝自定义参数 */
    if (params) {
        if (params->baud_rate) s_params.baud_rate = params->baud_rate;
        if (params->data_bits >= 5 && params->data_bits <= 8) s_params.data_bits = params->data_bits;
        if (params->stop_bits >= 1 && params->stop_bits <= 2) s_params.stop_bits = params->stop_bits;
        if (params->parity <= 2) s_params.parity = params->parity;
        if (params->gpio_tx >= 0) s_params.gpio_tx = params->gpio_tx;
        if (params->gpio_rx >= 0) s_params.gpio_rx = params->gpio_rx;
    }

    esp_err_t ret = uart_config_and_open(true);
    if (ret != ESP_OK) return ret;

    s_inited = true;
    s_ready  = true;

    /* 启动 RX task */
    BaseType_t ok = xTaskCreate(uart_rx_task, "uart_rx", BRIDGE_UART_RX_STACK, NULL, 4, NULL);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "RX task create failed");
        uart_driver_delete(BRIDGE_UART_NUM);
        s_inited = false;
        s_ready  = false;
        return ESP_ERR_NO_MEM;
    }

    /* 热替换 bridge_serial transport（dummy → UART）*/
    bridge_serial_set_transport(&s_uart_transport);

    ESP_LOGI(TAG, "bridge_uart ready: UART%d %u baud %d%c%d%s TX=GPIO%d RX=GPIO%d",
             BRIDGE_UART_NUM,
             (unsigned)s_params.baud_rate,
             s_params.data_bits,
             s_params.parity == 0 ? 'N' : s_params.parity == 1 ? 'O' : 'E',
             s_params.stop_bits,
             " ",
             s_params.gpio_tx, s_params.gpio_rx);

    return ESP_OK;
}

esp_err_t bridge_uart_set_params(const bridge_uart_params_t *params)
{
    if (!params) return ESP_ERR_INVALID_ARG;
    if (!s_inited) return ESP_ERR_INVALID_STATE;

    /* 只改变更的字段 */
    bool changed = false;
    if (params->baud_rate && params->baud_rate != s_params.baud_rate) {
        s_params.baud_rate = params->baud_rate; changed = true;
    }
    if (params->data_bits >= 5 && params->data_bits <= 8 && params->data_bits != s_params.data_bits) {
        s_params.data_bits = params->data_bits; changed = true;
    }
    if (params->stop_bits >= 1 && params->stop_bits <= 2 && params->stop_bits != s_params.stop_bits) {
        s_params.stop_bits = params->stop_bits; changed = true;
    }
    if (params->parity <= 2 && params->parity != s_params.parity) {
        s_params.parity = params->parity; changed = true;
    }

    if (!changed) return ESP_OK;

    /* 热改参：不 delete/reinstall 驱动（rx 任务常驻 uart_read_bytes，
     * 拆驱动会把内部锁从读者脚下抽走 → spinlock assert 重启；且重装窗口
     * 恰好落在 esptool SYNC 等待期内吃掉回包）。uart_set_* 可在运行中调用 */
    static const uart_parity_t parity_map[] = { UART_PARITY_DISABLE, UART_PARITY_ODD, UART_PARITY_EVEN };
    esp_err_t ret = uart_set_baudrate(BRIDGE_UART_NUM, s_params.baud_rate);
    if (ret == ESP_OK)
        ret = uart_set_word_length(BRIDGE_UART_NUM, (uart_word_length_t)(s_params.data_bits - 5));
    if (ret == ESP_OK)
        ret = uart_set_parity(BRIDGE_UART_NUM, parity_map[s_params.parity]);
    if (ret == ESP_OK)
        ret = uart_set_stop_bits(BRIDGE_UART_NUM,
                                 s_params.stop_bits == 2 ? UART_STOP_BITS_2 : UART_STOP_BITS_1);
    /* 切参瞬间线上的乱码字节清掉，避免污染后续 SLIP 解码 */
    uart_flush_input(BRIDGE_UART_NUM);
    if (ret == ESP_OK) {
        s_ready = true;
        ESP_LOGI(TAG, "params updated: %u baud %d%c%d",
                 (unsigned)s_params.baud_rate,
                 s_params.data_bits,
                 s_params.parity == 0 ? 'N' : s_params.parity == 1 ? 'O' : 'E',
                 s_params.stop_bits);
    } else {
        ESP_LOGE(TAG, "reconfig failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

bool bridge_uart_is_ready(void) { return s_ready; }

uint32_t bridge_uart_get_baud(void) { return s_params.baud_rate; }

bool bridge_uart_set_baud(uint32_t baud)
{
    bridge_uart_params_t p = s_params;
    p.baud_rate = baud;
    return bridge_uart_set_params(&p) == ESP_OK;
}

void bridge_uart_deinit(void)
{
    if (!s_inited) return;
    s_ready  = false;
    s_inited = false;
    uart_driver_delete(BRIDGE_UART_NUM);
    ESP_LOGI(TAG, "deinited");
}

const bridge_serial_transport_t *bridge_uart_transport(void)
{
    return &s_uart_transport;
}

const bridge_uart_params_t *bridge_uart_get_params(void)
{
    return &s_params;
}
