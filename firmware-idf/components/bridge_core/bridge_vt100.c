/*
 * bridge_vt100 —— 纯核心 VT100/ANSI 解析器（v0.3.0）。
 *
 * 三态状态机：NORMAL → ESC → CSI。零阻塞、零拷贝，每次 feed 直接回调命令。
 * v0.3.0 核心子集够支撑 vi/nano 这类终端程序。
 */
#include "bridge_vt100.h"
#include <string.h>

#define ESC_CHAR  0x1B
#define CSI_CHAR  '['

void vt100_init(vt100_parser_t *p, vt100_cmd_cb_t cmd_cb, void *ctx)
{
    if (p == NULL || cmd_cb == NULL) return;
    memset(p, 0, sizeof(*p));
    p->cb = cmd_cb;
    p->cb_ctx = ctx;
    p->state = VT_NORMAL;
    p->param_count = 0;
    p->csi_prefix = -1;
    p->text_len = 0;
}

static void send_cmd(vt100_parser_t *p, const vt100_cmd_t *cmd)
{
    if (p->cb) p->cb(cmd, p->cb_ctx);
}

/* 批量 flush text_buf —— 攒够 32 字节或遇到控制符/ESC 时调用 */
static void flush_text(vt100_parser_t *p)
{
    if (p->text_len == 0) return;
    vt100_cmd_t cmd = { .type = VT100_CMD_TEXT };
    memcpy(cmd.data.text.bytes, p->text_buf, p->text_len);
    cmd.data.text.len = p->text_len;
    p->text_len = 0;
    send_cmd(p, &cmd);
}

/* SGR: ESC [ n m —— 颜色/样式设置。参数可多个（分号分隔） */
static void handle_sgr(vt100_parser_t *p)
{
    vt100_cmd_t cmd = { .type = VT100_CMD_COLOR };
    cmd.data.color.fg = VT100_COLOR_DEFAULT;
    cmd.data.color.bg = VT100_COLOR_DEFAULT;
    cmd.data.color.bold = false;
    cmd.data.color.underline = false;

    int *params = p->params;
    int count = p->param_count;
    /* 无参数或首个参数=0 → reset 所有样式 */
    if (count == 0 || (count == 1 && params[0] == 0)) {
        send_cmd(p, &cmd);
        return;
    }
    for (int i = 0; i < count; i++) {
        int n = params[i];
        if (n == 1)          cmd.data.color.bold = true;
        else if (n == 4)     cmd.data.color.underline = true;
        else if (n == 22)    cmd.data.color.bold = false;
        else if (n == 24)    cmd.data.color.underline = false;
        else if (n >= 30 && n <= 37) cmd.data.color.fg = (uint8_t)(n - 29); /* 1=BLACK */
        else if (n == 39)   cmd.data.color.fg = VT100_COLOR_DEFAULT;
        else if (n >= 40 && n <= 47) cmd.data.color.bg = (uint8_t)(n - 39);
        else if (n == 49)   cmd.data.color.bg = VT100_COLOR_DEFAULT;
        /* v0.3.0：90-97 bright fg, 100-107 bright bg 与深色同色处理 */
        else if (n >= 90 && n <= 97)  cmd.data.color.fg = (uint8_t)(n - 89);
        else if (n >= 100 && n <= 107) cmd.data.color.bg = (uint8_t)(n - 99);
        /* 27 dim off, 28 blink off, 29 inverse off —— 忽略 */
    }
    send_cmd(p, &cmd);
}

static void handle_csi_final(vt100_parser_t *p, int final_char)
{
    int *params = p->params;
    int count = p->param_count;

    switch (final_char) {
    case 'm':   handle_sgr(p); break;

    case 'H':   /* CUP: 绝对定位 ESC [ row;col H */
    case 'f': { /* HVP: 同上 */
        vt100_cmd_t cmd = { .type = VT100_CMD_CURSOR_ABS };
        cmd.data.abs.row = (count > 0 && params[0] > 0) ? (uint16_t)params[0] : 1;
        cmd.data.abs.col = (count > 1 && params[1] > 0) ? (uint16_t)params[1] : 1;
        send_cmd(p, &cmd);
        break;
    }

    case 'A':   /* CUU: 上移 ESC [ n A */
    case 'B':   /* CUD: 下移 ESC [ n B */
    case 'C':   /* CUF: 右移 ESC [ n C */
    case 'D':   /* CUB: 左移 ESC [ n D */
    case 'G': { /* CHA: ESC [ n G 绝对列 —— v0.3.0 按相对左移处理 */
        vt100_cmd_t cmd = { .type = VT100_CMD_CURSOR_REL };
        int n = (count > 0 && params[0] > 0) ? params[0] : 1;
        cmd.data.rel.dx = 0; cmd.data.rel.dy = 0;
        switch (final_char) {
        case 'A': cmd.data.rel.dy = (int16_t)(-n); break;
        case 'B': cmd.data.rel.dy = (int16_t)( n); break;
        case 'C': cmd.data.rel.dx = (int16_t)( n); break;
        case 'D': cmd.data.rel.dx = (int16_t)(-n); break;
        case 'G': cmd.data.rel.dx = (int16_t)(-n + 1); break; /* 绝对列→相对 */
        }
        send_cmd(p, &cmd);
        break;
    }

    case 'J': { /* ED: 清屏 ESC [ n J */
        vt100_cmd_t cmd = { .type = VT100_CMD_CLEAR_SCREEN };
        cmd.data.clear_mode = (count > 0 && params[0] >= 0 && params[0] <= 2)
                              ? (uint8_t)params[0] : 0;
        send_cmd(p, &cmd);
        break;
    }

    case 'K': { /* EL: 清行 ESC [ n K */
        vt100_cmd_t cmd = { .type = VT100_CMD_CLEAR_LINE };
        cmd.data.clear_mode = (count > 0 && params[0] >= 0 && params[0] <= 2)
                              ? (uint8_t)params[0] : 0;
        send_cmd(p, &cmd);
        break;
    }

    case 'h': { /* SM/Private Set: ESC [ ? 25 h 显示光标 */
        if (p->csi_prefix == '?' && count == 1 && params[0] == 25) {
            vt100_cmd_t cmd = { .type = VT100_CMD_CURSOR_SHOW };
            send_cmd(p, &cmd);
        }
        break;
    }

    case 'l': { /* RM/Private Reset: ESC [ ? 25 l 隐藏光标 */
        if (p->csi_prefix == '?' && count == 1 && params[0] == 25) {
            vt100_cmd_t cmd = { .type = VT100_CMD_CURSOR_HIDE };
            send_cmd(p, &cmd);
        }
        break;
    }

    default:
        /* 未知 CSI 序列静默丢弃 —— 不打印乱码给 UI */
        break;
    }
}

static void feed_normal(vt100_parser_t *p, uint8_t c)
{
    if (c == ESC_CHAR) {
        flush_text(p);   /* ESC 前先 flush 积压的文本 */
        p->state = VT_ESC;
        return;
    }
    /* 可打印字符：攒到 text_buf，满 32 字节或遇到控制符才 flush */
    if (c >= 0x20 && c <= 0x7E) {
        p->last_was_cr = false;
        if (p->text_len >= sizeof(p->text_buf)) {
            flush_text(p);
        }
        p->text_buf[p->text_len++] = (char)c;
        return;
    }
    /* 控制符：先 flush 积压文本，再发控制命令 */
    flush_text(p);
    switch (c) {
    case 0x0D: { /* CR */
        vt100_cmd_t cmd = { .type = VT100_CMD_CRLF };
        send_cmd(p, &cmd);
        p->last_was_cr = true;
        break;
    }
    case 0x0A: { /* LF */
        if (p->last_was_cr) {
            p->last_was_cr = false;
            break;
        }
        vt100_cmd_t cmd = { .type = VT100_CMD_CRLF };
        send_cmd(p, &cmd);
        break;
    }
    case 0x08: {
        vt100_cmd_t cmd = { .type = VT100_CMD_BACKSPACE };
        send_cmd(p, &cmd);
        break;
    }
    case 0x09: {
        vt100_cmd_t cmd = { .type = VT100_CMD_TAB };
        send_cmd(p, &cmd);
        break;
    }
    /* 0x00-0x07, 0x0B, 0x0C, 0x0E-0x1F —— 其他控制符忽略 */
    default: break;
    }
}

static void feed_esc(vt100_parser_t *p, uint8_t c)
{
    flush_text(p);   /* ESC 前积压文本先 flush */
    if (c == CSI_CHAR) {
        p->state = VT_CSI;
        p->param_count = 0;
        p->csi_prefix = -1;
        /* 预清零 params —— 某些序列（如 ESC [ H）没有参数 */
        p->params[0] = 0;
        return;
    }
    /* 非 CSI 的 ESC 序列（ESC D 索引等）—— v0.3.0 忽略 */
    p->state = VT_NORMAL;
}

static void feed_csi(vt100_parser_t *p, uint8_t c)
{
    /* '?' '>' '=' '<' 这些是 private mode 前缀 —— v0.3.0 只识别 '?' */
    if (p->csi_prefix < 0 && (c == '?' || c == '>' || c == '=' || c == '<')) {
        p->csi_prefix = c;
        return;
    }
    if (c >= '0' && c <= '9') {
        int digit = c - '0';
        if (p->param_count == 0) {
            p->params[0] = digit;
            p->param_count = 1;
        } else if (p->param_count < (int)(sizeof(p->params) / sizeof(p->params[0]))) {
            /* 多位数拼接 */
            p->params[p->param_count - 1] = p->params[p->param_count - 1] * 10 + digit;
        }
        return;
    }
    if (c == ';') {
        if (p->param_count < (int)(sizeof(p->params) / sizeof(p->params[0]))) {
            p->param_count++;
            p->params[p->param_count - 1] = 0;
        }
        return;
    }
    /* CSI 终止符：字母或 '?' 后的字母，范围 0x40-0x7E（CSI final byte） */
    if (c >= 0x40 && c <= 0x7E) {
        handle_csi_final(p, c);
        p->state = VT_NORMAL;
        p->csi_prefix = -1;
        return;
    }
    /* 其他异常字节（如控制符）—— 放弃当前 CSI，回到 NORMAL */
    p->state = VT_NORMAL;
    p->csi_prefix = -1;
}

void vt100_feed(vt100_parser_t *p, const uint8_t *data, size_t len)
{
    if (p == NULL || data == NULL || len == 0) return;
    for (size_t i = 0; i < len; i++) {
        uint8_t c = data[i];
        switch (p->state) {
        case VT_NORMAL: feed_normal(p, c); break;
        case VT_ESC:   feed_esc(p, c); break;
        case VT_CSI:    feed_csi(p, c); break;
        }
    }
    flush_text(p);   /* chunk 末尾残留的文本也要发出去 */
}

void vt100_reset(vt100_parser_t *p)
{
    if (p == NULL) return;
    p->state = VT_NORMAL;
    p->param_count = 0;
    p->csi_prefix = -1;
    p->text_len = 0;
}
