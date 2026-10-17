/*
 * bridge_serial —— 板无关双向串口数据管道（v0.1.4）。
 *
 * 纯核心模块：只依赖 FreeRTOS（xStreamBuffer），不含任何板级/USB 头文件。
 * 数据流：
 *
 *   目标板 -> [板级 VCP data_cb] -> bridge_serial_on_rx() -> RX 环 -> read()
 *                                                                      |
 *   v0.1.4：临时 console 验收台；v0.1.5：TCP:3333 裸桥（这里换消费者）
 *
 *   write() -> TX 环 -> 核心 ser_tx 任务 -> 板级注册的 send 回调（USB bulk OUT）
 *
 * 生产者（TCP/console）write 时只入环，不被 USB 传输延迟阻塞；
 * 全板唯一的 USB 发送动作由 ser_tx 任务串行化，无需上层加锁。
 *
 * 背压：RX 环装不下整次回调数据时 on_rx 返回 false，数据留在 VCP
 * 驱动自身缓冲（4KB）稍后重投，等效流控，不做覆盖式丢字节。
 */
#pragma once

#include "esp_err.h"
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* TX 环形缓冲容量（PC→串口方向）。数据区放 PSRAM。 */
#define BRIDGE_SERIAL_RING_SIZE   32768

/* 订阅者环容量（v0.3.1 P0 fanout: TCP + RFC2217 + WS 都走这里）：
 * 48KB PSRAM 吸收回环在途数据——拥塞门关闭后在途上限≈UART 双向驱动缓冲
 * 16KB+FIFO，门限 20KB 保证零丢字节（大缓冲换确定性，PSRAM 无功耗成本）。
 * on_rx 零超时写入；入环后剩余 <20KB 关门 / 全部 ≥32KB 开门（迟滞 12KB）。
 * 数据区从 PSRAM 静态分配，无 PSRAM 回退内部 RAM */
#define BRIDGE_SERIAL_SUB_RING_SIZE  49152
#define BRIDGE_SERIAL_MAX_SUBSCRIBERS  6  /* tcp + rfc + ws + 预留 3 */

/* 板级发送回调：把 len 字节送出物理串口（阻塞到传输完成或超时）。
 * 返回非 ESP_OK 时核心丢弃本批并继续（设备拔出等场景由 v0.1.6 恢复逻辑处理）。 */
typedef esp_err_t (*bridge_serial_send_fn_t)(const uint8_t *data, size_t len, void *arg);

typedef struct {
    bridge_serial_send_fn_t send;
    void *arg;
} bridge_serial_transport_t;

/* 安装管道：建两个 8KB 环 + ser_tx 任务。必须在板级 USB 枚举开始前调用 */
esp_err_t bridge_serial_init(const bridge_serial_transport_t *tx_transport);

/* 热替换 transport —— USB Host 启用后把 dummy 换成 CDC */
void bridge_serial_set_transport(const bridge_serial_transport_t *tx_transport);

/* 板级 VCP 数据回调入口（UART/CDC 收到的字节）。
 * 数据经 fanout 零超时广播给所有订阅者；订阅者环接近满时拥塞门关闭，
 * tx_task 停发把背压经 TCP 窗口传回 PC，避免丢字节。 */
bool bridge_serial_on_rx(const uint8_t *data, size_t len);

/* 生产者写数据：入 TX 环即返回，返回实际入环字节数（环满时可能短写） */
size_t bridge_serial_write(const uint8_t *data, size_t len, uint32_t timeout_ms);

/* 清空两个环（设备拔出时调用，防止旧数据发给新设备） */
void bridge_serial_flush(void);

/* —— 订阅者机制（v0.3.0）——
 *
 * 本地观察者（终端、日志过滤器等）订阅原始串口数据流。与主 RX 环的唯一读者
 * 机制完全独立：订阅者有自己的小 StreamBuffer，on_rx 零超时灌数据（满则丢），
 * 订阅者自己的任务从自己小环读，不影响主管道的背压和网络互斥让位。
 *
 * 典型用法：
 *   bridge_serial_subscriber_t *sub = bridge_serial_subscribe("vt100");
 *   while (1) {
 *       uint8_t buf[256];
 *       size_t n = bridge_serial_sub_read(sub, buf, sizeof(buf), 100);
 *       vt100_feed(parser, buf, n);
 *   } */
typedef struct bridge_serial_subscriber bridge_serial_subscriber_t;

/* 注册订阅者：返回句柄，失败返回 NULL。名字用于日志（最多 16 字） */
bridge_serial_subscriber_t *bridge_serial_subscribe(const char *name);

/* 订阅者取数据（任务上下文）。接口同 bridge_serial_read 但从独立小环读 */
size_t bridge_serial_sub_read(bridge_serial_subscriber_t *sub,
                              uint8_t *buf, size_t len, uint32_t timeout_ms);

/* 释放订阅者。调用后 on_rx 不再向其广播 */
void bridge_serial_unsubscribe(bridge_serial_subscriber_t *sub);

/* 清空订阅者 ring（RFC2217 新会话前冲刷旧数据） */
void bridge_serial_sub_reset(bridge_serial_subscriber_t *sub);

/* 字节计数（压测零误差用）：累计入环/实际发出/各类丢弃 */
void bridge_serial_stats(size_t *rx_total, size_t *tx_total,
                         size_t *rx_backpressure, size_t *tx_drop);

/* 订阅者状态：名称/drop计数/剩余空间 */
typedef struct {
    char name[16];
    size_t drop_cnt;
    size_t space_left;  /* 当前 ring 剩余字节 */
    bool active;
} bridge_serial_sub_stats_t;

/* 导出所有订阅者当前状态（调用方传数组 + 容量，返回实际填充数） */
int bridge_serial_sub_stats_all(bridge_serial_sub_stats_t *out, int max_out);

#ifdef __cplusplus
}
#endif
