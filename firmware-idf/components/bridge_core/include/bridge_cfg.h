/*
 * bridge_cfg —— bridge.cfg 配置解析（纯核心，零板级依赖）
 *
 * 优先级：SD 卡 /bridge.cfg > NVS > 编译默认值
 * 解析：纯文本 key=value，# 开头行忽略，value 可选引号
 *
 * 消费侧：main 装配时先 mount SD、读 cfg，再把 cfg 值喂给各模块 init/start。
 * 每个模块的"配置注入点"是既有 API 参数（如 bridge_tcp_start(port)、
 * bridge_mdns_start(raw_port, rfc_port)），bridge_cfg 不反向依赖任何模块。
 */
#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- 配置字段结构体 ---- */

#define BRIDGE_CFG_MAX_SSID   33
#define BRIDGE_CFG_MAX_PW     65
#define BRIDGE_CFG_MAX_HOST   33
#define BRIDGE_CFG_MAX_TOKEN  129
#define BRIDGE_CFG_MAX_PATH   128

typedef struct {
    /* WiFi（可选：有则 save_cred 进 NVS 作预置凭证，无则走既有 boot 逻辑） */
    char wifi_ssid[BRIDGE_CFG_MAX_SSID];
    char wifi_password[BRIDGE_CFG_MAX_PW];

    /* USB Host：多口设备（如 CH342 Watcher 双 CDC）的接口索引，默认 0 */
    uint8_t usb_interface;

    /* mDNS 主机名（不含 .local），默认 "airserial" */
    char hostname[BRIDGE_CFG_MAX_HOST];

    /* 串口桥 TCP 端口，默认 3333 */
    uint16_t tcp_port;

    /* tunnel token（v0.1.11 预留：空字符串=禁用；非空时 tunnel 启动需匹配） */
    char tunnel_token[BRIDGE_CFG_MAX_TOKEN];

    /* 日志级别：error/warn/info/debug/verbose，默认 info */
    char log_level[16];

    /* ---- 元信息 ---- */
    bool has_wifi;         /* cfg 里有 wifi_ssid */
    bool has_hostname;     /* cfg 里有 hostname */
    bool has_tcp_port;     /* cfg 里有 tcp_port */
    bool has_tunnel_token; /* cfg 里有 tunnel_token（值可空，key 出现即 true） */
    bool has_log_level;    /* cfg 里有 log_level */
} bridge_cfg_t;

/* ---- API ---- */

/* 用编译默认值填充 cfg（所有 has_* = false）。任何解析前先调这个。 */
void bridge_cfg_set_defaults(bridge_cfg_t *cfg);

/* 从文本 buffer 解析 cfg。
 * 返回：成功=true / 格式错误=log warning 并忽略异常行、其余正常返回 false。
 * 解析器容忍：行首空格、key 后多空格、value 双引号、# 注释行、空行 */
bool bridge_cfg_parse_buffer(const char *buf, size_t buf_len, bridge_cfg_t *cfg);

/* 便捷：直接给"/sdcard/bridge.cfg"路径的完整读取+解析。
 * 文件不存在返回 false（不是错误——只是没 cfg），调用方决定降级。 */
bool bridge_cfg_parse_file(const char *path, bridge_cfg_t *cfg);

#ifdef __cplusplus
}
#endif
