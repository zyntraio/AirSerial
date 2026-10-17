/*
 * bridge_cfg.c —— 纯核心：key=value 文本解析
 *
 * 零板级依赖、零 WiFi/USB 依赖。只做字符串切分和类型转换。
 */
#include "bridge_cfg.h"

#include "esp_log.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

static const char *TAG = "bridge_cfg";

/* ---- 默认值 ---- */

static const char DEFAULT_HOSTNAME[]    = "airserial";
static const uint16_t DEFAULT_TCP_PORT  = 3333;
static const uint8_t  DEFAULT_USB_ITF   = 0;
static const char DEFAULT_LOG_LEVEL[]   = "info";

/* ---- 内部 helpers ---- */

static void ltrim(const char **p)
{
    while (**p && isspace((unsigned char)**p)) (*p)++;
}

static void rtrim(char *s)
{
    size_t len = strlen(s);
    while (len > 0 && isspace((unsigned char)s[len - 1])) {
        s[--len] = '\0';
    }
}

/* 解析一行，找到 key=value，两者都做 trim
 * value 支持双引号包裹（"..."），引号会被去掉 */
static bool split_kv(const char *line, char *key_out, size_t key_cap,
                     char *val_out, size_t val_cap)
{
    const char *p = line;
    ltrim(&p);

    /* 跳过 # 注释和空行 */
    if (*p == '\0' || *p == '#') return false;

    const char *eq = strchr(p, '=');
    if (!eq) return false;

    /* key */
    size_t klen = (size_t)(eq - p);
    if (klen >= key_cap) return false;
    memcpy(key_out, p, klen);
    key_out[klen] = '\0';
    rtrim(key_out);
    if (key_out[0] == '\0') return false;

    /* value */
    const char *v = eq + 1;
    ltrim(&v);

    size_t vlen = strlen(v);
    if (vlen >= val_cap) vlen = val_cap - 1;
    memcpy(val_out, v, vlen);
    val_out[vlen] = '\0';
    rtrim(val_out);

    /* 去掉两端引号 */
    if (val_out[0] == '"' && val_out[vlen - 1] == '"') {
        val_out[0] = '\0';
        memmove(val_out, val_out + 1, vlen - 2);
        val_out[vlen - 2] = '\0';
    }

    return true;
}

static void safe_copy(char *dst, size_t cap, const char *src)
{
    if (!src) { dst[0] = '\0'; return; }
    size_t len = strlen(src);
    if (len >= cap) len = cap - 1;
    memcpy(dst, src, len);
    dst[len] = '\0';
}

static bool parse_uint(const char *s, uint16_t *out)
{
    if (!s || s[0] == '\0') return false;
    char *end = NULL;
    unsigned long v = strtoul(s, &end, 10);
    if (end == s || v > 0xFFFF) return false;
    *out = (uint16_t)v;
    return true;
}

static bool parse_uint8(const char *s, uint8_t *out)
{
    if (!s || s[0] == '\0') return false;
    char *end = NULL;
    unsigned long v = strtoul(s, &end, 10);
    if (end == s || v > 0xFF) return false;
    *out = (uint8_t)v;
    return true;
}

/* ---- 公开 API ---- */

void bridge_cfg_set_defaults(bridge_cfg_t *cfg)
{
    memset(cfg, 0, sizeof(*cfg));
    safe_copy(cfg->hostname, sizeof(cfg->hostname), DEFAULT_HOSTNAME);
    cfg->tcp_port    = DEFAULT_TCP_PORT;
    cfg->usb_interface = DEFAULT_USB_ITF;
    safe_copy(cfg->log_level, sizeof(cfg->log_level), DEFAULT_LOG_LEVEL);
    /* wifi_ssid/tunnel_token 保持空字符串 */
}

bool bridge_cfg_parse_buffer(const char *buf, size_t buf_len, bridge_cfg_t *cfg)
{
    bool any_valid = false;
    size_t pos = 0;

    while (pos < buf_len) {
        /* 找行尾 */
        const char *line_start = buf + pos;
        size_t line_len = 0;
        while (pos + line_len < buf_len
               && buf[pos + line_len] != '\n'
               && buf[pos + line_len] != '\r') {
            line_len++;
        }

        /* 拷贝到临时缓冲 */
        char line[256];
        if (line_len >= sizeof(line)) line_len = sizeof(line) - 1;
        memcpy(line, line_start, line_len);
        line[line_len] = '\0';

        /* 跳过 \r */
        pos += line_len;
        if (pos < buf_len && buf[pos] == '\r') pos++;
        if (pos < buf_len && buf[pos] == '\n') pos++;

        char key[64], val[256];
        if (!split_kv(line, key, sizeof(key), val, sizeof(val))) continue;

        any_valid = true;
        ESP_LOGI(TAG, "  cfg: %s=%s", key, val);

        /* ---- 字段识别 ---- */
        if (strcmp(key, "wifi_ssid") == 0) {
            safe_copy(cfg->wifi_ssid, sizeof(cfg->wifi_ssid), val);
            cfg->has_wifi = true;
        } else if (strcmp(key, "wifi_password") == 0) {
            safe_copy(cfg->wifi_password, sizeof(cfg->wifi_password), val);
        } else if (strcmp(key, "hostname") == 0) {
            safe_copy(cfg->hostname, sizeof(cfg->hostname), val);
            cfg->has_hostname = true;
        } else if (strcmp(key, "tcp_port") == 0) {
            uint16_t v;
            if (parse_uint(val, &v)) {
                cfg->tcp_port = v;
                cfg->has_tcp_port = true;
            } else {
                ESP_LOGW(TAG, "tcp_port invalid: '%s' (keep default %u)", val, cfg->tcp_port);
            }
        } else if (strcmp(key, "usb_interface") == 0) {
            uint8_t v;
            if (parse_uint8(val, &v)) {
                cfg->usb_interface = v;
            } else {
                ESP_LOGW(TAG, "usb_interface invalid: '%s'", val);
            }
        } else if (strcmp(key, "tunnel_token") == 0) {
            safe_copy(cfg->tunnel_token, sizeof(cfg->tunnel_token), val);
            cfg->has_tunnel_token = true;
        } else if (strcmp(key, "log_level") == 0) {
            safe_copy(cfg->log_level, sizeof(cfg->log_level), val);
            cfg->has_log_level = true;
        } else {
            ESP_LOGW(TAG, "unknown cfg key: '%s' (ignored)", key);
        }
    }

    return any_valid;
}

bool bridge_cfg_parse_file(const char *path, bridge_cfg_t *cfg)
{
    FILE *f = fopen(path, "rb");
    if (!f) {
        /* 文件不存在——写一份默认模板给用户手改，然后 parse 它 */
        ESP_LOGI(TAG, "no cfg file at %s, writing default template", path);
        const char *default_template =
            "# AirSerial bridge.cfg —— SD 卡根目录\n"
            "# 优先级：SD 此文件 > NVS 存的凭证/设置 > 编译默认值\n"
            "# 改完重启生效；# 开头行忽略\n"
            "\n"
            "# WiFi（可选：预置进 NVS 作 boot auto-connect 首选）\n"
            "wifi_ssid=\n"
            "wifi_password=\n"
            "\n"
            "# mDNS 主机名（不含 .local）\n"
            "hostname=airserial\n"
            "\n"
            "# 串口桥 TCP 端口\n"
            "tcp_port=3333\n"
            "\n"
            "# 多口设备接口索引（如 CH342 Watcher 双 CDC 口选 0/1）\n"
            "usb_interface=0\n"
            "\n"
            "# v0.1.11 预留：tunnel token（空=禁用）\n"
            "tunnel_token=\n"
            "\n"
            "# 日志级别：error / warn / info / debug / verbose\n"
            "log_level=info\n";
        FILE *out = fopen(path, "w");
        if (out) {
            fputs(default_template, out);
            fclose(out);
            ESP_LOGI(TAG, "default bridge.cfg written to %s", path);
            /* 继续打开 parse——模板里的默认值全生效 */
            f = fopen(path, "rb");
        } else {
            ESP_LOGW(TAG, "failed to write default cfg (SD read-only?)");
            return false;
        }
    }

    /* 读文件到缓冲（bridge.cfg 通常几 KB，16KB 绰绰有余） */
    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    fseek(f, 0, SEEK_SET);

    if (sz <= 0) { fclose(f); return false; }
    if (sz > 16 * 1024) {
        ESP_LOGW(TAG, "cfg file too large (%ld bytes, max 16KB), skipping", sz);
        fclose(f);
        return false;
    }

    char *buf = (char *)malloc((size_t)sz + 1);
    if (!buf) {
        ESP_LOGE(TAG, "malloc %ld for cfg failed", sz);
        fclose(f);
        return false;
    }

    size_t got = fread(buf, 1, (size_t)sz, f);
    buf[got] = '\0';
    fclose(f);

    bool ok = bridge_cfg_parse_buffer(buf, got, cfg);
    free(buf);
    return ok;
}
