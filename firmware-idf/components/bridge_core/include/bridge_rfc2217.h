/*
 * bridge_rfc2217 —— RFC2217 协商骨架（v0.2.0）。
 *
 * 纯核心模块：只依赖 lwIP socket + FreeRTOS + bridge_serial，
 * 不含任何板级/USB 头文件。
 *
 * 职责：
 *  - TCP server 监听 3334（独立于 3333 裸口）
 *  - Telnet IAC 字节流状态机：解析协商命令 + COM-PORT-OPTION 子协商，
 *    剥离 telnet 控制字节后纯数据透传到 bridge_serial
 *  - 串口→网络方向：对 0xFF 做 IAC IAC 转义
 *  - COM-PORT-OPTION 子协商解析（v0.2.0 仅 log，不映射硬件；
 *    控制回调预留，v0.2.1 接入 board_usb）
 *
 * 与 bridge_tcp 共享 bridge_serial 管道：3334 有客户端时 3333 丢弃
 * （通过弱符号 bridge_rfc2217_has_client() 协调）。
 */
#pragma once

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 控制回调：v0.2.0 可注册 NULL 仅 log；v0.2.1 接入 board_usb。
 * 参数：baud=波特率(0=不变), dtr/rts=电平(true=assert) */
typedef void (*rfc2217_ctrl_cb_t)(uint32_t baud, bool dtr, bool rts, void *arg);

/* 客户端状态回调（v0.2.2）：connected=true/false。用于按需切换 WiFi
 * 低时延模式（有客户端关省电，无客户端恢复），由装配层（main）接线 */
typedef void (*rfc2217_client_cb_t)(bool connected, void *arg);

esp_err_t bridge_rfc2217_start(uint16_t port);
void      bridge_rfc2217_stop(void);
void      bridge_rfc2217_set_ctrl_callback(rfc2217_ctrl_cb_t cb, void *arg);
void      bridge_rfc2217_set_client_callback(rfc2217_client_cb_t cb, void *arg);

/* —— AirSerial CLI（v0.3.3 起 RFC2217 与 Web WS 终端共用）——
 * 回复通道由调用方注入：RFC2217 传 socket 发送，WS 终端传 WebSocket 帧发送。
 * line 为以 \0 结尾的一行文本；前缀 "!airsn " 不符时返回 false，
 * 调用方应把原行穿透下发到串口桥（套娃场景二台桥的扩展命令必须能到达）。 */
typedef void (*bridge_cli_reply_t)(const char *line, void *arg);
bool bridge_cli_process(char *line, bridge_cli_reply_t reply, void *arg, const char *channel);

/* 供 bridge_tcp 查询：3334 是否有活跃客户端（弱符号，无 rfc2217 时返回 false） */
bool bridge_rfc2217_has_client(void);

#ifdef __cplusplus
}
#endif
