/*
 * bridge_web —— 网页服务积木（纯核心，零板级/零 UI 依赖）。
 *
 * v0.3.1：captive portal（配网占位页 + DNS 劫持）。
 * v0.3.2：同实例加配网表单 handler（URL 空间 /=配网页）。
 * v0.4.x：同实例加 Web 终端（/ws）与状态 API（/api/ 前缀）。
 *
 * URL 空间规划（自 v0.3.1 起冻结）：
 *   /        —— 配网页（当前为占位页）
 *   /api/    —— 状态/控制 JSON（v0.4.4）
 *   /ws      —— WebSocket 终端（v0.4.1）
 *   其余     —— 探针路径统一 302 回 /（captive 检测）
 *
 * httpd 只绑 0.0.0.0:80，AP/STA 网卡都能访问；
 * DNS 劫持仅在 portal 运行期间开启（STA 正常时不开，不干扰正常上网）。
 */
#pragma once
#include "esp_err.h"
#include "esp_http_server.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* HTTP 固定端口（v0.3.1 起定死，mDNS/文档均引用此值） */
#define BRIDGE_WEB_HTTP_PORT  80

/*
 * 启动 portal：httpd（占位页 + 探针 302）+ captive DNS（UDP 53 全域名劫持）。
 *
 * @param ap_ssid 设备热点名，渲染进占位页（可为 NULL，占位页省略该行）
 * @param ap_ip   热点 IP 字符串（如 "192.168.4.1"），302 跳转与 DNS 应答目标；
 *                NULL 时默认 "192.168.4.1"
 * 重复调用：已运行时返回 ESP_ERR_INVALID_STATE。
 */
esp_err_t bridge_web_portal_start(const char *ap_ssid, const char *ap_ip);

/* 停 portal：httpd + DNS 一并收掉（幂等，未运行返回 ESP_OK） */
esp_err_t bridge_web_portal_stop(void);

/* portal 是否运行中 */
bool bridge_web_portal_is_running(void);

/* v0.4.4+: 外部模块注册自定义 URI handler（portal start 后才能调）
 * 注册失败（httpd 未启动）返回 ESP_ERR_INVALID_STATE */
esp_err_t bridge_web_add_uri_handler(const httpd_uri_t *uri);

#ifdef __cplusplus
}
#endif
