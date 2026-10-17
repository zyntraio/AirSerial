/*
 * bridge_mdns —— mDNS 广播积木（v0.1.6），纯核心模块（mdns + esp_netif）。
 *
 * 上电连网后：
 *  - 主机名 airserial-XXXX（XXXX=STA MAC 后两字节共 4 个十六进制位）
 *    → PC 端用 airserial-XXXX.local 直接连，多台设备同时在线不打架
 *  - 两个端口（裸 TCP / RFC2217）统一广播同一服务类型 _airserial_tcp._tcp，
 *    TXT 记录 proto 区分 raw-tcp / rfc2217，客户端按类型一次枚举全部服务
 *
 * 与 WiFi 后端无关：C6/C5/以太网板拿到 IP 后都调同一个入口。
 * 必须在 netif 拿到 IP 后调用；IP 变更无需重跑（mdns 跟随 netif 事件）。
 */
#pragma once

#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 启动 mDNS：主机名/实例名按本机 MAC 自动生成
 * raw_port：裸 TCP 串口端口（3333）；rfc_port：RFC2217 端口（3334） */
esp_err_t bridge_mdns_start(uint16_t raw_port, uint16_t rfc_port);

/* 本次启动生成的主机名（不含 .local），如 "airserial-a1b2"；未 start 时返回 NULL */
const char *bridge_mdns_hostname(void);

#ifdef __cplusplus
}
#endif
