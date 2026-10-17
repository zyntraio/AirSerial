/*
 * bridge_vt100 —— 纯核心 VT100/ANSI 终端转义序列解析器（v0.3.0）。
 *
 * 零 UI / 零板级依赖：只依赖 stdlib.h / string.h / stdint.h。
 * 字节流输入 → 回调输出结构化命令，UI 层消费命令做渲染。
 *
 * 支持的转义序列（v0.3.0 核心子集）：
 *   - 普通可打印字符 (0x20-0x7E) → VT100_CMD_TEXT
 *   - CR (0x0D), LF (0x0A), BS (0x08), TAB (0x09)
 *   - ESC [ n m         —— SGR（前景/背景色 + 粗体/下划线）
 *   - ESC [ H           —— 光标归位
 *   - ESC [ row;col H   —— 绝对定位
 *   - ESC [ n A/B/C/D   —— 相对移动（上/下/右/左 n 格）
 *   - ESC [ 2J / ESC [ J —— 清屏（全部 / 光标后）
 *   - ESC [ K           —— 清行（光标后到行尾）
 *
 * 用法：
 *   vt100_parser_t parser;
 *   vt100_init(&parser, my_cmd_cb, my_ctx);
 *   vt100_feed(&parser, serial_buf, len);
 *   // my_cmd_cb 被调用 N 次，每个命令一条
 */
#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    VT100_CMD_TEXT,         /* 打印文本: str + len (0x20-0x7E, 不含控制符) */
    VT100_CMD_CRLF,         /* 回车换行（CR 或 LF 触发；UI 负责判断当前位置） */
    VT100_CMD_BACKSPACE,    /* 退格 BS (0x08) */
    VT100_CMD_TAB,          /* TAB (0x09) —— UI 按 8 列对齐 */
    VT100_CMD_COLOR,        /* SGR 颜色/样式: fg, bg, bold, underline */
    VT100_CMD_CURSOR_ABS,   /* 光标绝对定位: row, col (1-based) */
    VT100_CMD_CURSOR_REL,   /* 相对移动: dx (正=右/负=左), dy (正=下/负=上) */
    VT100_CMD_CLEAR_SCREEN, /* 清屏: mode (0=光标后, 1=光标前, 2=全部) */
    VT100_CMD_CLEAR_LINE,   /* 清行: mode (0=光标后, 1=光标前, 2=整行) */
    VT100_CMD_CURSOR_HIDE,
    VT100_CMD_CURSOR_SHOW,
    VT100_CMD_RESET,        /* 应用或上层主动 reset 时发 */
} vt100_cmd_type_t;

typedef struct {
    vt100_cmd_type_t type;
    union {
        /* VT100_CMD_TEXT: 内联缓冲（v0.3.0 单字符也拷贝，避免指向 feed 临时缓冲导致
         * 悬挂指针——feed 返回后 sub_task 下一次 sub_read 会覆盖 buf） */
        struct { char bytes[32]; size_t len; } text;
        struct { uint8_t fg;   /* 0=default, 1-9=ANSI bright/dark color 见 VT100_COLOR_* */
                 uint8_t bg;   /* 同上 */
                 bool bold;
                 bool underline;
        } color;
        struct { uint16_t row; uint16_t col; } abs;   /* 1-based */
        struct { int16_t dx;   int16_t dy;  } rel;    /* 正=右/下, 负=左/上 */
        uint8_t clear_mode; /* VT100_CMD_CLEAR_SCREEN / CLEAR_LINE 的 mode 参数 */
    } data;
} vt100_cmd_t;

/* ANSI 颜色常量（color.fg / color.bg 字段用） */
enum {
    VT100_COLOR_DEFAULT = 0,
    VT100_COLOR_BLACK   = 1,
    VT100_COLOR_RED     = 2,
    VT100_COLOR_GREEN   = 3,
    VT100_COLOR_YELLOW  = 4,
    VT100_COLOR_BLUE    = 5,
    VT100_COLOR_MAGENTA = 6,
    VT100_COLOR_CYAN    = 7,
    VT100_COLOR_WHITE   = 8,
    /* 9-16 为 bright 色 —— v0.3.0 UI 先不区分，和 1-8 同色 */
};

/* 上层命令回调：解析器产生一个命令就立刻调用，不做缓冲。
 * ctx 为 vt100_init 时传入的用户数据；cmd 指针在回调返回后无效，
 * 如需保留请自行拷贝。返回 true 表示消费成功；false 解析器仅做日志提示（v0.3.0 可选） */
typedef bool (*vt100_cmd_cb_t)(const vt100_cmd_t *cmd, void *ctx);

/* —— 解析器对象（不透明，由 vt100.c 内部 struct 承载）—— */
typedef struct vt100_parser vt100_parser_t;

/* 初始化解析器。cmd_cb 不能为 NULL */
void vt100_init(vt100_parser_t *p, vt100_cmd_cb_t cmd_cb, void *ctx);

/* 喂入新字节流。内部状态机会持续产出命令并回调。
 * 必须在单次调用内完整处理完所有字节（无阻塞、无拷贝） */
void vt100_feed(vt100_parser_t *p, const uint8_t *data, size_t len);

/* 重置解析器内部状态（如设备重新枚举时）。不会发送 VT100_CMD_RESET */
void vt100_reset(vt100_parser_t *p);

/* —— 内部状态（vt100.c 可见，放头文件是因为 vt100_parser_t 不是 void* 类型）—— */
struct vt100_parser {
    vt100_cmd_cb_t cb;
    void *cb_ctx;
    /* 解析状态机 */
    enum { VT_NORMAL, VT_ESC, VT_CSI } state;
    /* CSI 参数（最多 8 个；ESC [ 1;2;3 H 这种形式） */
    int params[8];
    int param_count;
    /* ESC [ 收集到的中间缓冲（CSI 序列的引导字符如 '?' '>'） */
    int csi_prefix;  /* -1=无; 其他=如 '?' (0x3F) 表示 private mode */
    /* v0.3.0：CR→LF 合并标志，避免 "\r\n" 产生两个 CRLF 命令 */
    bool last_was_cr;
    /* v0.3.0：批量 TEXT 缓冲——连续可打印字符攒一起发，砍命令数 10-20 倍 */
    char text_buf[32];
    uint8_t text_len;
};

#ifdef __cplusplus
}
#endif
