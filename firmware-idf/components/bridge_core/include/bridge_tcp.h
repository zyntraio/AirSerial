/*
 * bridge_tcp —— TCP 裸串口桥（v0.1.5），v0.1.2 echo 的正式继任者。
 *
 * 纯核心模块：lwIP socket + bridge_serial，零板级依赖。
 *
 *   PC (nc/烧录器) ──TCP:3333──> bridge_tcp ──> TX 环 ──> 板级 VCP ──> 目标板
 *   目标板 ──> VCP ──> RX 环 ──> bridge_tcp ──TCP:3333──> PC
 *
 * 会话规则：
 *  - 单客户端：同一时刻只服务一个 TCP 连接
 *  - 新连接踢旧连接：对旧 fd 发 shutdown，旧会话的收包任务自行收尸
 *    （串口->TCP 是全局唯一 fanout，流缓冲只允许一个读者，不能每连接建）
 *  - TCP_NODELAY：串口交互/烧录握手禁 Nagle，字节即来即发
 *  - 无客户端期间目标板输出直接丢弃并计数（终端未接线语义），
 *    防止 RX 环塞满反压目标板；USB 拔出时仍由上层 flush 清环
 *
 * v0.1.6 将补：连接异常与 WiFi 掉线的恢复长测、mDNS。
 */
#pragma once

#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 启动裸桥监听（内部 accept 任务，不阻塞调用者）。常用端口 3333 */
esp_err_t bridge_tcp_start(uint16_t port);

/* 客户端状态回调：connected=true/false。用于按需切换资源，
 * 例如 main.c 在有 bridge 客户端时停 Web portal 省 RAM，退出后延迟恢复 */
typedef void (*tcp_client_cb_t)(bool connected, void *arg);
void bridge_tcp_set_client_callback(tcp_client_cb_t cb, void *arg);

/* 当前是否有活跃客户端（3333 单客户端模型，供 UI 状态页轮询） */
bool bridge_tcp_has_client(void);

/* v0.1.6：会话统计，供反复重连/拔插长测取证 */
typedef struct {
    uint32_t accepts;           /* 累计 accept 次数 */
    uint32_t kicks;             /* 新连接踢掉旧连接次数 */
    uint32_t dropped_no_client; /* 无客户端期间丢弃的目标板字节数 */
} bridge_tcp_stats_t;
void bridge_tcp_get_stats(bridge_tcp_stats_t *out);

#ifdef __cplusplus
}
#endif
