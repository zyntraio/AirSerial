/*
 * bridge_rfc2217 实现，见头文件注释。
 *
 * Telnet 协议要点：
 *   IAC = 0xFF
 *   IAC DO   = 0xFF 0xFD
 *   IAC DONT = 0xFF 0xFE
 *   IAC WILL = 0xFF 0xFB
 *   IAC WONT = 0xFC
 *   IAC SB   = 0xFF 0xFA  (subnegotiation begin)
 *   IAC SE   = 0xFF 0xF0  (subnegotiation end)
 *   数据中的 0xFF 必须转义为 0xFF 0xFF
 *
 * COM-PORT-OPTION: option code = 44 (0x2C)
 *   SET_BAUDRATE  = 1  (cmd + 4 bytes big-endian baud)
 *   SET_DATASIZE  = 2
 *   SET_PARITY    = 3
 *   SET_STOPSIZE  = 4
 *   SET_CONTROL   = 5  (cmd + mask + value; bit0=DTR, bit1=RTS)
 *   PURGE_DATA    = 12
 */
#include "bridge_rfc2217.h"
#include "bridge_serial.h"
#include "app_version.h"

#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <errno.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include "nvs_flash.h"
#include <arpa/inet.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"

#define TAG "rfc2217"

#define RFC2217_TASK_STACK  4096
#define PUMP_CHUNK          512
#define SB_BUF_SIZE         256

/* Telnet 命令码 */
#define TN_IAC   0xFF
#define TN_DONT  0xFE
#define TN_DO    0xFD
#define TN_WONT  0xFC
#define TN_WILL  0xFB
#define TN_SB    0xFA
#define TN_SE    0xF0

/* COM-PORT-OPTION */
#define OPT_COM_PORT  44

/* COM-PORT-OPTION 子命令（客户端→服务端 1..12；服务端回执 = 命令码 + 100） */
#define CPO_SET_BAUDRATE  1
#define CPO_SET_DATASIZE  2
#define CPO_SET_PARITY    3
#define CPO_SET_STOPSIZE  4
#define CPO_SET_CONTROL   5
#define CPO_PURGE_DATA    12
#define CPO_RESP_OFFSET   100

/* SET_CONTROL 单字节码（RFC2217 §4.5，与 pyserial rfc2217.py 一致） */
#define CTRL_REQ_FLOW     0x00
#define CTRL_NO_FLOW      0x01
#define CTRL_SW_FLOW      0x02
#define CTRL_HW_FLOW      0x03
#define CTRL_REQ_BREAK    0x04
#define CTRL_BREAK_ON     0x05
#define CTRL_BREAK_OFF    0x06
#define CTRL_REQ_DTR      0x07
#define CTRL_DTR_ON       0x08
#define CTRL_DTR_OFF      0x09
#define CTRL_REQ_RTS      0x0a
#define CTRL_RTS_ON       0x0b
#define CTRL_RTS_OFF      0x0c

/* —— AirSerial CLI (v0.3.1) ——
 * RFC2217 端口上以 "!airsn " 开头的文本命令行被固件拦截处理，
 * 不下发给目标板。命令以 \r\n 或 \n 结尾。 */
#define CLI_PREFIX "!airsn "
#define CLI_PREFIX_LEN 7  /* strlen("!airsn ") */
#define CLI_VERSION "v1"

/* debug counter: ST_DATA branch 总共 write 了多少字节 */
static volatile size_t s_debug_bytes_written;
static volatile size_t s_debug_recv_bytes;     /* recv 成功 */
static volatile size_t s_debug_select_ok;      /* select>0 次数 */
static volatile size_t s_debug_fionread_max;   /* FIONREAD 最大可读字节 */

/* 每次启动随机生成的 boot magic，stats 输出用于客户端检测设备是否中途重启 */
static uint32_t s_boot_magic;

/* 前向声明：cli_reply 里要用 locked_send */
static void locked_send(int fd, const uint8_t *buf, size_t n);

/* CLI 回复函数（持锁 send，带 \r\n 结尾） */
static void cli_reply(int fd, const char *fmt, ...)
{
    char buf[384];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n > 0 && n < (int)sizeof(buf) - 3) {
        buf[n++] = '\r'; buf[n++] = '\n'; buf[n] = '\0';
        locked_send(fd, (const uint8_t *)buf, (size_t)n);
    }
}

/* CLI 命令处理器，返回 true 表示这行是 CLI 命令已被消费 */
#include <stdarg.h>
static bool cli_handle_line(int fd, char *line)
{
    /* 去掉尾部 \r\n */
    size_t len = strlen(line);
    while (len > 0 && (line[len-1] == '\n' || line[len-1] == '\r')) {
        line[--len] = '\0';
    }

    /* 检查前缀 */
    if (strncmp(line, CLI_PREFIX, CLI_PREFIX_LEN) != 0) {
        return false;
    }

    char *cmd = line + CLI_PREFIX_LEN;

    /* === 命令分发 === */

    if (strncmp(cmd, "status", 6) == 0 && (cmd[6] == '\0' || cmd[6] == ' ')) {
        /* !airsn status — 版本/板名必须来自编译期宏，禁止硬编码（套娃场景下
         * 第一台桥的应答是用户辨认链路身份的唯一依据） */
        extern uint32_t bridge_uart_get_baud(void);
        cli_reply(fd, "AirSerial %s (%s)", CLI_VERSION, AIRSERIAL_VERSION);
#if defined(CONFIG_BOARD_TYPE_STICKS3)
        cli_reply(fd, "  chip:   ESP32-S3 M5Stack StickS3");
#elif defined(CONFIG_BOARD_TYPE_N16R8)
        cli_reply(fd, "  chip:   ESP32-S3 N16R8 devkit");
#elif defined(CONFIG_BOARD_TYPE_CARD_PUTER_ADV)
        cli_reply(fd, "  chip:   ESP32-S3 Cardputer ADV");
#else
        cli_reply(fd, "  chip:   ESP32-S3 (board unset)");
#endif
        cli_reply(fd, "  uart:   %lu baud 8N1", (unsigned long)bridge_uart_get_baud());
        cli_reply(fd, "  client: RFC2217 active");
        return true;
    }

    if (strncmp(cmd, "set baud ", 9) == 0) {
        /* !airsn set baud 9600 */
        uint32_t baud = (uint32_t)strtoul(cmd + 9, NULL, 10);
        if (baud == 0 || baud > 5000000) {
            cli_reply(fd, "ERR: invalid baud rate");
        } else {
            extern bool bridge_uart_set_baud(uint32_t baud);
            if (bridge_uart_set_baud(baud)) {
                cli_reply(fd, "OK baud=%lu", (unsigned long)baud);
            } else {
                cli_reply(fd, "ERR: set baud failed");
            }
        }
        return true;
    }

    if (strncmp(cmd, "usbctrl ", 8) == 0) {
        /* !airsn usbctrl <bmReqType> <bReq> <wValue> [wIndex] [inLen]
         * 全部十六进制。对当前 USB Host 串口设备发原始 vendor 控制传输，
         * 例如 "!airsn usbctrl 40 a4 ffbf 0"（RTS 断言）。
         * inLen>0 时做 IN 读并回显字节（最多 8 字节）。*/
        unsigned int bmtype, breq, wval, widx = 0;
        int inlen = 0;
        int n = sscanf(cmd + 8, "%x %x %x %x %d", &bmtype, &breq, &wval, &widx, &inlen);
        if (n < 3) {
            cli_reply(fd, "ERR: usage: usbctrl <bmtype> <req> <wvalue> [windex] [inlen]");
            return true;
        }
        if (inlen < 0 || inlen > 8) inlen = 0;
        uint8_t inbuf[8] = {0};
        extern esp_err_t bridge_usb_host_raw_ctrl(uint8_t, uint8_t, uint16_t,
                                                  uint16_t, uint8_t *, size_t);
        esp_err_t r = bridge_usb_host_raw_ctrl((uint8_t)bmtype, (uint8_t)breq,
                                               (uint16_t)wval, (uint16_t)widx,
                                               inlen ? inbuf : NULL, (size_t)inlen);
        if (inlen > 0 && r == ESP_OK) {
            char hex[32];
            int p = 0;
            for (int i = 0; i < inlen && p < (int)sizeof(hex) - 3; i++) {
                p += snprintf(hex + p, sizeof(hex) - p, "%02x ", inbuf[i]);
            }
            cli_reply(fd, "OK %s in=[%s]", esp_err_to_name(r), hex);
        } else {
            cli_reply(fd, "%s %s", (r == ESP_OK ? "OK" : "ERR"), esp_err_to_name(r));
        }
        return true;
    }

    if (strncmp(cmd, "usbhost", 7) == 0 && (cmd[7] == '\0' || cmd[7] == ' ')) {
        /* !airsn usbhost [on|off] — 与 Web /api/usb/host_on|off 同语义：
         * 写 NVS flag 后重启生效（SHARED_PHY 切换必须重启）。无参查询 NVS 设定。 */
        extern uint32_t bridge_uart_get_baud(void);
        const char *arg = cmd + 7;
        while (*arg == ' ') arg++;
        if (*arg == '\0') {
            nvs_handle_t h;
            if (nvs_open("airsn_diag", NVS_READONLY, &h) == ESP_OK) {
                uint8_t v = 0;
                esp_err_t r = nvs_get_u8(h, "usb_host_on", &v);
                nvs_close(h);
                cli_reply(fd, "OK usbhost=%s (NVS)", (r == ESP_OK && v == 1) ? "on" : "off");
            } else {
                cli_reply(fd, "ERR: nvs open failed");
            }
            return true;
        }
        bool want_on = (strncmp(arg, "on", 2) == 0);
        if (!want_on && strncmp(arg, "off", 3) != 0) {
            cli_reply(fd, "ERR: usage: usbhost [on|off]");
            return true;
        }
        nvs_handle_t h;
        if (nvs_open("airsn_diag", NVS_READWRITE, &h) == ESP_OK) {
            if (want_on) nvs_set_u8(h, "usb_host_on", 1);
            else         nvs_erase_key(h, "usb_host_on");
            nvs_commit(h);
            nvs_close(h);
        }
        cli_reply(fd, "OK usbhost=%s, rebooting...", want_on ? "on" : "off");
        vTaskDelay(pdMS_TO_TICKS(300));
        esp_restart();
        return true;
    }

    if (strncmp(cmd, "restart", 7) == 0 && (cmd[7] == '\0' || cmd[7] == ' ')) {
        /* !airsn restart */
        cli_reply(fd, "restarting...");
        vTaskDelay(pdMS_TO_TICKS(200));
        esp_restart();
        return true;
    }

    if (strncmp(cmd, "hello", 5) == 0 && (cmd[5] == '\0' || cmd[5] == ' ')) {
        /* !airsn hello — 彩蛋 */
        cli_reply(fd, "Hello, pioneer of the new era.");
        return true;
    }

    if (strncmp(cmd, "stats", 5) == 0 && (cmd[5] == '\0' || cmd[5] == ' ')) {
        /* !airsn stats — 内部诊断 */
        size_t rx, tx, bp, drop;
        bridge_serial_stats(&rx, &tx, &bp, &drop);
        extern void bridge_uart_rx_debug(size_t *, size_t *, size_t *,
                                         size_t *, size_t *, size_t *);
        size_t urx, ufifo, ubuf, ufrm, upar, ubrk;
        bridge_uart_rx_debug(&urx, &ufifo, &ubuf, &ufrm, &upar, &ubrk);
        extern void bridge_usb_host_debug(size_t *, size_t *, size_t *);
        size_t crx, ctx, ctf;
        bridge_usb_host_debug(&crx, &ctx, &ctf);
        long up_ms = (long)(esp_timer_get_time() / 1000);
        cli_reply(fd, "boot=%08lx up=%ldms rx_total=%lu tx_total=%lu rxbp=%lu txdrop=%lu rfc_w=%lu rfc_r=%lu fionread_max=%lu uart_rx=%lu ufifo=%lu ubuf=%lu ufrm=%lu upar=%lu ubrk=%lu cdc_rx=%lu cdc_tx=%lu cdc_txf=%lu",
                  (unsigned long)s_boot_magic, up_ms,
                  (unsigned long)rx, (unsigned long)tx,
                  (unsigned long)bp, (unsigned long)drop,
                  (unsigned long)s_debug_bytes_written,
                  (unsigned long)s_debug_recv_bytes,
                  (unsigned long)s_debug_fionread_max,
                  (unsigned long)urx, (unsigned long)ufifo,
                  (unsigned long)ubuf, (unsigned long)ufrm,
                  (unsigned long)upar, (unsigned long)ubrk,
                  (unsigned long)crx, (unsigned long)ctx, (unsigned long)ctf);
        bridge_serial_sub_stats_t subs[BRIDGE_SERIAL_MAX_SUBSCRIBERS];
        int n = bridge_serial_sub_stats_all(subs, BRIDGE_SERIAL_MAX_SUBSCRIBERS);
        for (int i = 0; i < n; i++) {
            cli_reply(fd, "  sub '%s': drop=%lu space=%lu",
                      subs[i].name, (unsigned long)subs[i].drop_cnt,
                      (unsigned long)subs[i].space_left);
        }
        return true;
    }

    /* 未知命令：不回 ERR 不消费，返回 false 由调用处原样穿透下发。
     * 套娃场景（桥的目标板也跑 AirSerial）下，第二台的扩展命令必须能到达；
     * 烧录二进制流中巧合出现的 "!airsn " 序列也只有穿透才能保证镜像无损。 */
    return false;
}

/* IAC 状态机状态 */
typedef enum {
    ST_DATA,       /* 透传数据 */
    ST_IAC,        /* 刚收到 0xFF，等下一字节 */
    ST_IAC_CMD,    /* IAC + DO/WILL/SB 等，等 option byte */
    ST_SB,         /* 子协商数据中 */
    ST_SB_IAC,     /* 子协商中收到 0xFF，等下一字节 */
} iac_state_t;

static SemaphoreHandle_t s_lock;
static int s_client_fd = -1;
static volatile bool s_started;
static volatile bool s_com_port_negotiated;  /* 客户端是否协商了 COM-PORT-OPTION */

/* 单会话退场门：blackout 时旧会话任务要靠 send 超时(10s)/keepalive(~16s) 才死。
 * accept 踢旧后必须等旧任务真正退场再建新任务，否则重连风暴 × 4KB 栈会在
 * 仅 30KB 余量的内部 RAM 上堆出 xTaskCreate NO_MEM。 */
static TaskHandle_t s_acc_task;
static TaskHandle_t s_sess_task;

/* v0.3.1: subscriber ring for fanout (parallel with TCP and WS) */
static bridge_serial_subscriber_t *s_rfc_sub;

static rfc2217_ctrl_cb_t s_ctrl_cb;
static void *s_ctrl_arg;
static rfc2217_client_cb_t s_client_cb;
static void *s_client_cb_arg;

/* v0.2.2：跟踪当前 DTR/RTS 电平，供 REQ_DTR/REQ_RTS 查询回执 */
static bool s_dtr;
static bool s_rts;

void bridge_rfc2217_set_ctrl_callback(rfc2217_ctrl_cb_t cb, void *arg)
{
    s_ctrl_cb = cb;
    s_ctrl_arg = arg;
}

void bridge_rfc2217_set_client_callback(rfc2217_client_cb_t cb, void *arg)
{
    s_client_cb = cb;
    s_client_cb_arg = arg;
}

bool bridge_rfc2217_has_client(void)
{
    return s_client_fd >= 0;
}

/* 所有 socket 发送必须串行化（v0.2.2 竞态修复）：
 * tcp_to_ser_task 的协商回执（send_iac/send_subneg）与 ser_to_tcp_task 的
 * 串口数据并发 send 同一 socket 时，TCP 流中字节交错会拆散 IAC SB...SE
 * 序列——esptool 复位后 StickS3 启动 banner 突发期回执必损坏，pyserial
 * 因收不齐 SET_DATASIZE/PARITY/STOPSIZE 回执而拒绝打开端口。 */
static void locked_send(int fd, const uint8_t *buf, size_t n)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    size_t off = 0;
    while (off < n) {
        /* 阻塞 send — lwIP socket 有足够大的 TX buffer，通常不会真正阻塞很久。
         * 这是正确的背压行为：如果 PC 端慢，send 阻塞让背压经 TCP 窗口传回。*/
        int w = send(fd, buf + off, n - off, 0);
        if (w < 0) {
            break;
        }
        off += w;
    }
    xSemaphoreGive(s_lock);
}

/* 发送一个 IAC 协商响应 */
static void send_iac(int fd, uint8_t cmd, uint8_t opt)
{
    uint8_t buf[3] = { TN_IAC, cmd, opt };
    locked_send(fd, buf, 3);
}

/* 发送 COM-PORT-OPTION 子协商（payload 中的 0xFF 需转义） */
static void send_subneg(int fd, uint8_t cmd, const uint8_t *payload, size_t plen)
{
    uint8_t buf[2 + 1 + 1 + (SB_BUF_SIZE * 2) + 2];
    size_t n = 0;
    buf[n++] = TN_IAC;
    buf[n++] = TN_SB;
    buf[n++] = OPT_COM_PORT;
    buf[n++] = cmd;
    for (size_t i = 0; i < plen; i++) {
        if (payload[i] == TN_IAC) buf[n++] = TN_IAC;
        buf[n++] = payload[i];
    }
    buf[n++] = TN_IAC;
    buf[n++] = TN_SE;
    locked_send(fd, buf, n);
}

/* 解析 COM-PORT-OPTION 子协商数据。
 * v0.2.2：SET_* 请求处理后回执（命令码+100，原值回显）——pyserial rfc2217
 * 无回执会抛 "Remote does not accept parameter change"。 */
static void parse_com_port_option(int fd, const uint8_t *data, size_t len)
{
    if (len == 0) {
        return;
    }
    uint8_t cmd = data[0];
    switch (cmd) {
    case CPO_SET_BAUDRATE:
        if (len >= 5) {
            uint32_t baud = ((uint32_t)data[1] << 24) |
                            ((uint32_t)data[2] << 16) |
                            ((uint32_t)data[3] << 8)  |
                            ((uint32_t)data[4]);
            ESP_LOGI(TAG, "SET_BAUDRATE: %lu", (unsigned long)baud);
            if (s_ctrl_cb) s_ctrl_cb(baud, false, false, s_ctrl_arg);
            send_subneg(fd, cmd + CPO_RESP_OFFSET, data + 1, 4);
        }
        break;
    case CPO_SET_DATASIZE:
        if (len >= 2) {
            ESP_LOGI(TAG, "SET_DATASIZE: %u", data[1]);
            send_subneg(fd, cmd + CPO_RESP_OFFSET, data + 1, 1);
        }
        break;
    case CPO_SET_PARITY:
        if (len >= 2) {
            ESP_LOGI(TAG, "SET_PARITY: %u", data[1]);
            send_subneg(fd, cmd + CPO_RESP_OFFSET, data + 1, 1);
        }
        break;
    case CPO_SET_STOPSIZE:
        if (len >= 2) {
            ESP_LOGI(TAG, "SET_STOPSIZE: %u", data[1]);
            send_subneg(fd, cmd + CPO_RESP_OFFSET, data + 1, 1);
        }
        break;
    case CPO_SET_CONTROL:
        if (len >= 2) {
            uint8_t code = data[1];
            uint8_t ack = code;   /* 回执值：REQ_* 查询回当前态，其余原值回显 */
            bool changed = false;
            switch (code) {
            case CTRL_DTR_ON:  s_dtr = true;  changed = true; break;
            case CTRL_DTR_OFF: s_dtr = false; changed = true; break;
            case CTRL_RTS_ON:  s_rts = true;  changed = true; break;
            case CTRL_RTS_OFF: s_rts = false; changed = true; break;
            case CTRL_REQ_DTR: ack = s_dtr ? CTRL_DTR_ON : CTRL_DTR_OFF; break;
            case CTRL_REQ_RTS: ack = s_rts ? CTRL_RTS_ON : CTRL_RTS_OFF; break;
            default: break;  /* BREAK/流控/REQ_FLOW：仅回执不映射硬件 */
            }
            ESP_LOGI(TAG, "SET_CONTROL: 0x%02x DTR=%d RTS=%d", code, (int)s_dtr, (int)s_rts);
            if (changed && s_ctrl_cb) s_ctrl_cb(0, s_dtr, s_rts, s_ctrl_arg);
            send_subneg(fd, cmd + CPO_RESP_OFFSET, &ack, 1);
        }
        break;
    case CPO_PURGE_DATA:
        ESP_LOGI(TAG, "PURGE_DATA");
        bridge_serial_flush();
        send_subneg(fd, cmd + CPO_RESP_OFFSET, data + 1, len - 1);
        break;
    default:
        ESP_LOGI(TAG, "COM-PORT-OPTION cmd=%u len=%u (unhandled)", cmd, (unsigned)len);
        break;
    }
}

/* 处理一个完整的协商命令（IAC cmd opt） */
static void handle_negotiation(int fd, uint8_t cmd, uint8_t opt)
{
    if (opt == OPT_COM_PORT) {
        switch (cmd) {
        case TN_DO:
            /* 客户端请求我们启用 COM-PORT-OPTION → 同意 */
            send_iac(fd, TN_WILL, OPT_COM_PORT);
            s_com_port_negotiated = true;
            ESP_LOGI(TAG, "client DO COM-PORT-OPTION -> WILL");
            break;
        case TN_WILL:
            /* 客户端表示它会发 COM-PORT-OPTION → 我们也 DO */
            send_iac(fd, TN_DO, OPT_COM_PORT);
            s_com_port_negotiated = true;
            ESP_LOGI(TAG, "client WILL COM-PORT-OPTION -> DO");
            break;
        case TN_DONT:
            send_iac(fd, TN_WONT, OPT_COM_PORT);
            s_com_port_negotiated = false;
            ESP_LOGI(TAG, "client DONT COM-PORT-OPTION -> WONT");
            break;
        case TN_WONT:
            send_iac(fd, TN_DONT, OPT_COM_PORT);
            s_com_port_negotiated = false;
            ESP_LOGI(TAG, "client WONT COM-PORT-OPTION -> DONT");
            break;
        default:
            break;
        }
    } else {
        /* 其他 option 一律拒绝 */
        switch (cmd) {
        case TN_DO:
            send_iac(fd, TN_WONT, opt);
            break;
        case TN_WILL:
            send_iac(fd, TN_DONT, opt);
            break;
        case TN_DONT:
        case TN_WONT:
            /* 客户端拒绝，无需回应 */
            break;
        default:
            break;
        }
    }
}

/* 串口→网络：对 0xFF 做 IAC IAC 转义后发送
 * v0.3.1: 从 subscriber ring 读，与 TCP/WS 并行 fanout */
static void ser_to_tcp_task(void *arg)
{
    (void)arg;
    /* static：大批量缓冲不占任务栈（单任务实例）；2KB/批快速 drain sub ring，
     * 高速回环下避免小批量 + send 抖动导致 sub ring 满丢帧 */
    static uint8_t buf[2048];
    static uint8_t esc[4096];  /* 最坏情况全是 0xFF，转义后 2x */
    while (1) {
        if (s_rfc_sub == NULL) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        size_t n = bridge_serial_sub_read(s_rfc_sub, buf, sizeof(buf), 5000);
        if (n == 0) {
            continue;
        }

        /* 转义 0xFF（先转义再查 fd） */
        size_t esc_len = 0;
        for (size_t i = 0; i < n; i++) {
            if (buf[i] == TN_IAC) {
                esc[esc_len++] = TN_IAC;
                esc[esc_len++] = TN_IAC;
            } else {
                esc[esc_len++] = buf[i];
            }
        }

        int fd;
        bool fail = false;
        /* 整批 send 必须持 s_lock：与 tcp_to_ser 的 IAC 回执互斥。
         * v0.3.1 曾把 send 移到锁外，导致回执若被部分写入，数据批次会插进
         * IAC SB...SE 中间——pyserial IAC 状态机整会话失步（option 超时重试
         * 十几次全灭），SLIP 数据流也被插坏（read-flash 尾部停滞）。
         * 持锁阻塞是正确的 TCP 背压：数据批 ≤4KB，回执等待有界。*/
        xSemaphoreTake(s_lock, portMAX_DELAY);
        fd = s_client_fd;
        if (fd < 0) {
            xSemaphoreGive(s_lock);
            continue;  /* 读环期间客户端断开，丢弃本批 */
        }
        size_t off = 0;
        while (off < esc_len) {
            int w = send(fd, esc + off, esc_len - off, 0);
            if (w < 0) {
                fail = true;
                break;
            }
            off += w;
        }
        xSemaphoreGive(s_lock);
        if (fail) {
            bool was_active;
            xSemaphoreTake(s_lock, portMAX_DELAY);
            was_active = (s_client_fd == fd);
            if (was_active) {
                s_client_fd = -1;
            }
            xSemaphoreGive(s_lock);
            shutdown(fd, SHUT_RDWR);
            /* 发送失败=连接已死，tcp_to_ser 退出时 active=false 不会再上报 */
            if (was_active && s_client_cb) {
                s_client_cb(false, s_client_cb_arg);
            }
            continue;
        }
    }
}

/* 网络→串口：Telnet IAC 状态机 + CLI 行拦截 + 纯数据写入 bridge_serial */
static void tcp_to_ser_task(void *arg)
{
    int fd = (int)(intptr_t)arg;
    uint8_t buf[PUMP_CHUNK];
    iac_state_t st = ST_DATA;
    uint8_t saved_cmd = 0;       /* IAC cmd byte (DO/WILL/SB...) */
    uint8_t sb_buf[SB_BUF_SIZE];
    size_t sb_len = 0;
    uint8_t opt = 0;             /* 子协商的 option code */

    /* v0.3.1: CLI 行缓冲（跨 recv 边界累积不完整行）
     * 只在"行首"投机匹配 CLI_PREFIX 长度的字节：任何一位不匹配立即放行，
     * 绝不能跨行缓冲——SLIP 二进制流里大量出现 0x21('!')，旧实现会把后续
     * 整段（含帧尾 0xC0）扣到下一个 0x0A 才发，导致 esptool stub 上传停死。*/
    char cli_buf[128];
    int cli_len = 0;
    bool cli_armed = false;       /* 已完整匹配 CLI_PREFIX，本行按命令消费 */
    bool at_line_start = true;    /* 是否位于行首（连接初态视为行首） */
    uint8_t pending[PUMP_CHUNK];  /* 待一次性 write 的普通 ST_DATA 字节 */
    int pending_len = 0;

    while (1) {
        /* 阻塞 socket：直接 recv，lwIP 会自动处理 flow control */
        {
            int avail = 0;
            ioctl(fd, FIONREAD, &avail);
            if ((size_t)avail > s_debug_fionread_max)
                s_debug_fionread_max = avail;
        }
        int n = recv(fd, buf, sizeof(buf), 0);
        if (n <= 0) {
            /* 连接关闭前先 flush 剩余 pending 数据 */
            if (pending_len > 0) {
                bridge_serial_write(pending, pending_len, portMAX_DELAY);
            }
            break;
        }
        s_debug_recv_bytes += n;

        /* === 逐字节过 IAC 状态机 + 累积 write ===
         * 普通 ST_DATA 字节先攒到 pending，chunk 结束或遇到特殊字节时一次性 write。
         * 这样 bridge_serial_write 调用次数从 N 减到几个，避免逐 poll 阻塞。 */
        for (int i = 0; i < n; i++) {
            uint8_t b = (uint8_t)buf[i];
            switch (st) {
            case ST_DATA:
                if (b == TN_IAC) {
                    /* IAC 字节：先 flush pending */
                    if (pending_len > 0) {
                        s_debug_bytes_written += bridge_serial_write(pending, pending_len, portMAX_DELAY);
                        pending_len = 0;
                    }
                    st = ST_IAC;
                    break;
                }
                if (cli_armed) {
                    /* 前缀已命中：攒到行尾再裁决（合法命令消费 / 不认识则穿透），期间字节不下发 */
                    if (b == '\n') {
                        cli_buf[cli_len] = '\0';
                        if (!cli_handle_line(fd, cli_buf)) {
                            /* 前缀命中但命令字不认识：整行补 \n 原样穿透给目标板。
                             * armed 期间没有任何字节下发过，此处回放时序即原始时序。 */
                            cli_buf[cli_len++] = '\n';
                            s_debug_bytes_written += bridge_serial_write(
                                (const uint8_t *)cli_buf, cli_len, portMAX_DELAY);
                        }
                        cli_len = 0;
                        cli_armed = false;
                        at_line_start = true;
                    } else if (cli_len < (int)sizeof(cli_buf) - 1) {
                        cli_buf[cli_len++] = (char)b;   /* \r 同样攒入，穿透时零损 */
                    } else {
                        /* 超 127 字节仍无行尾——合法命令最长 ~40 字符，此必为二进制流
                         * 巧合序列：已攒内容原样下发并退出拦截，本字节走普通 pending，
                         * 全程零字节丢失（烧录镜像安全性刚需）。 */
                        s_debug_bytes_written += bridge_serial_write(
                            (const uint8_t *)cli_buf, cli_len, portMAX_DELAY);
                        cli_len = 0;
                        cli_armed = false;
                        at_line_start = false;
                        if (pending_len < (int)sizeof(pending)) {
                            pending[pending_len++] = b;
                        } else {
                            s_debug_bytes_written += bridge_serial_write(pending, pending_len, portMAX_DELAY);
                            pending[0] = b;
                            pending_len = 1;
                        }
                    }
                    break;
                }
                if (cli_len > 0) {
                    /* 行首 '!' 之后的前缀投机匹配：逐位比对，不匹配立刻放行 */
                    if (cli_len < CLI_PREFIX_LEN &&
                        b == (uint8_t)CLI_PREFIX[cli_len]) {
                        cli_buf[cli_len++] = (char)b;
                        if (cli_len == CLI_PREFIX_LEN) {
                            cli_armed = true;
                        }
                    } else {
                        if (pending_len > 0) {
                            s_debug_bytes_written += bridge_serial_write(pending, pending_len, portMAX_DELAY);
                            pending_len = 0;
                        }
                        cli_buf[cli_len++] = (char)b;
                        s_debug_bytes_written += bridge_serial_write(
                            (const uint8_t *)cli_buf, cli_len, portMAX_DELAY);
                        cli_len = 0;
                        /* 放行字节本身是 \n 时必须置行首，否则下一行的 !airsn 会漏拦截 */
                        at_line_start = (b == '\n');
                    }
                    break;
                }
                if (at_line_start && b == '!') {
                    /* 可能是 CLI 开头：flush pending 保序，开始投机匹配 */
                    if (pending_len > 0) {
                        s_debug_bytes_written += bridge_serial_write(pending, pending_len, portMAX_DELAY);
                        pending_len = 0;
                    }
                    cli_buf[0] = (char)b;
                    cli_len = 1;
                    at_line_start = false;
                    break;
                }
                /* 普通 ST_DATA 字节 → 累积到 pending */
                at_line_start = (b == '\n');
                if (pending_len < (int)sizeof(pending)) {
                    pending[pending_len++] = b;
                } else {
                    /* pending 满了，先 flush 再累积 */
                    s_debug_bytes_written += bridge_serial_write(pending, pending_len, portMAX_DELAY);
                    pending[0] = b;
                    pending_len = 1;
                }
                break;
            case ST_IAC:
                if (b == TN_IAC) {
                    /* 转义的 0xFF → write 一个 */
                    uint8_t raw_iac = TN_IAC;
                    s_debug_bytes_written += bridge_serial_write(&raw_iac, 1, portMAX_DELAY);
                    st = ST_DATA;
                } else if (b == TN_DO || b == TN_DONT || b == TN_WILL || b == TN_WONT) {
                    saved_cmd = b; st = ST_IAC_CMD;
                } else if (b == TN_SB) { sb_len = 0; st = ST_SB; }
                else if (b == TN_SE) { st = ST_DATA; }
                else { st = ST_DATA; }
                break;
            case ST_IAC_CMD:
                handle_negotiation(fd, saved_cmd, b);
                st = ST_DATA;
                break;
            case ST_SB:
                if (b == TN_IAC) { st = ST_SB_IAC; }
                else if (sb_len < sizeof(sb_buf)) {
                    if (sb_len == 0) opt = b;
                    sb_buf[sb_len++] = b;
                }
                break;
            case ST_SB_IAC:
                if (b == TN_IAC) {
                    if (sb_len < sizeof(sb_buf)) sb_buf[sb_len++] = TN_IAC;
                    st = ST_SB;
                } else if (b == TN_SE) {
                    if (opt == OPT_COM_PORT && sb_len > 1)
                        parse_com_port_option(fd, sb_buf + 1, sb_len - 1);
                    st = ST_DATA;
                } else { st = ST_DATA; }
                break;
            }
        }
        /* chunk 处理完，flush 剩余 pending 数据 */
        if (pending_len > 0) {
            /* 用 0ms timeout —— 绝不阻塞 tcp_to_ser_task，
             * TCP 窗口需要及时更新才能让 PC 端继续发数据！*/
            size_t w = bridge_serial_write(pending, pending_len, portMAX_DELAY);
            s_debug_bytes_written += w;
            if (w < (size_t)pending_len) {
                ESP_LOGW(TAG, "TX ring full! dropped %d bytes", pending_len - (int)w);
            }
            pending_len = 0;
        }
    }

    /* 退场 */
    xSemaphoreTake(s_lock, portMAX_DELAY);
    bool active = (s_client_fd == fd);
    if (active) {
        s_client_fd = -1;
        s_com_port_negotiated = false;
    }
    if (s_sess_task == xTaskGetCurrentTaskHandle()) {
        s_sess_task = NULL;
    }
    shutdown(fd, SHUT_RDWR);
    close(fd);
    xSemaphoreGive(s_lock);
    ESP_LOGI(TAG, "rfc2217 fd=%d exited (active=%d)", fd, (int)active);
    /* 通知 accept 单会话槽已释放（通知位锁存，accept 尚未 wait 也不丢；
     * 值带退场任务句柄，accept 据此区分新/旧任务的通知） */
    if (s_acc_task != NULL) {
        xTaskNotify(s_acc_task,
                    (uint32_t)(intptr_t)xTaskGetCurrentTaskHandle(),
                    eSetValueWithOverwrite);
    }
    /* 只有活跃连接的退出才上报断开（被踢的旧连接退出时新客户端已上报连接） */
    if (active && s_client_cb) {
        s_client_cb(false, s_client_cb_arg);
    }
    vTaskDelete(NULL);
}

static void accept_task(void *arg)
{
    uint16_t port = (uint16_t)(intptr_t)arg;

    int ls = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (ls < 0) {
        ESP_LOGE(TAG, "socket() errno %d", errno);
        vTaskDelete(NULL);
    }
    int one = 1;
    setsockopt(ls, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    /* 注意：不要设 SO_RCVBUF —— lwIP 默认 recv_bufsize=INT_MAX，
     * 设小了反而导致 TCP 窗口被限制！*/

    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_port = htons(port),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    if (bind(ls, (struct sockaddr *)&addr, sizeof(addr)) != 0
        || listen(ls, 2) != 0) {
        ESP_LOGE(TAG, "bind/listen(%u) errno %d", port, errno);
        close(ls);
        vTaskDelete(NULL);
    }
    ESP_LOGI(TAG, "RFC2217 server listening on 0.0.0.0:%u", port);

    while (1) {
        struct sockaddr_in src;
        socklen_t slen = sizeof(src);
        int fd = accept(ls, (struct sockaddr *)&src, &slen);
        if (fd < 0) {
            ESP_LOGW(TAG, "accept errno %d, retry", errno);
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
        setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &one, sizeof(one));
        {
            int rcvbuf = 16384;
            setsockopt(fd, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));
            ESP_LOGI(TAG, "[RFC] fd=%d SO_RCVBUF=16384 set", fd);
        }
        /* blackout 时无超时的阻塞 send 永不返回：泵任务冻死、lwIP 发送内存永不释放，
         * 最终所有 socket 假死。30s 发不出才按死连接拆除——不能太短：合法慢客户端
         * （esptool 同步读、目标板 bootloop 刷屏而主机暂不读）在拥塞门下 send 会
         * 合法阻塞 10s+，实测 10s 超时会误杀停滞背压场景（recv 不加超时，空闲会话不能误杀）*/
        struct timeval snd_to;
        snd_to.tv_sec = 30;
        snd_to.tv_usec = 0;
        setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &snd_to, sizeof(snd_to));
        /* keepalive 10s 起探 / 2s 间隔 / 3 次失败：链路 blackout ~16s 后 recv 侧收到死连接通知 */
        {
            int ka_idle = 10, ka_intvl = 2, ka_cnt = 3;
            setsockopt(fd, IPPROTO_TCP, TCP_KEEPIDLE, &ka_idle, sizeof(ka_idle));
            setsockopt(fd, IPPROTO_TCP, TCP_KEEPINTVL, &ka_intvl, sizeof(ka_intvl));
            setsockopt(fd, IPPROTO_TCP, TCP_KEEPCNT, &ka_cnt, sizeof(ka_cnt));
        }

        /* 阻塞模式：lwIP 在 ESP-IDF 上阻塞 socket 工作更可靠。
         * fanout send 用 MSG_DONTWAIT 避免阻塞持锁。*/

        /* 踢旧连接 */
        xSemaphoreTake(s_lock, portMAX_DELAY);
        int old_fd = s_client_fd;
        TaskHandle_t old_task = s_sess_task;
        s_client_fd = fd;
        s_com_port_negotiated = false;
        if (old_fd >= 0) {
            shutdown(old_fd, SHUT_RDWR);
        }
        xSemaphoreGive(s_lock);

        ESP_LOGI(TAG, "RFC2217 client %s:%u connected (fd=%d)%s",
                 inet_ntoa(src.sin_addr), ntohs(src.sin_port), fd,
                 old_fd >= 0 ? " (kicked old)" : "");

        if (s_client_cb) {
            s_client_cb(true, s_client_cb_arg);
        }

        /* 新会话清掉 subscriber ring 里积压的旧数据（v0.3.1 fanout:
         * 无客户端期间 fanout 停读，tick/日志等噪音不冲刷会混进 esptool 的 SLIP 流） */
        bridge_serial_sub_reset(s_rfc_sub);

        char tname[configMAX_TASK_NAME_LEN];
        snprintf(tname, sizeof(tname), "rfc2ser%d", fd % 100);
        TaskHandle_t h = NULL;
        if (xTaskCreate(tcp_to_ser_task, tname, RFC2217_TASK_STACK,
                        (void *)(intptr_t)fd, 5, &h) != pdPASS) {
            ESP_LOGE(TAG, "rfc2ser create failed, reject fd=%d | int free=%lu largest=%lu psram free=%lu",
                     fd,
                     (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                     (unsigned long)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
                     (unsigned long)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
            xSemaphoreTake(s_lock, portMAX_DELAY);
            if (s_client_fd == fd) s_client_fd = -1;
            close(fd);
            xSemaphoreGive(s_lock);
            /* 不落 s_sess_task；若旧任务仍在退场，下面的等待会等到它释放 RAM */
        } else {
            s_sess_task = h;
        }

        /* 单会话门：等旧任务真正退场（blackout 下最坏 ~send 超时 30s），
         * 才回循环 accept 下一个连接；backlog=2 暂存 SYN。
         * 新会话任务已建立可正常服务，不等它。*/
        if (old_task != NULL) {
            TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(35000);
            bool gone = false;
            while (!gone) {
                xSemaphoreTake(s_lock, portMAX_DELAY);
                if (s_sess_task != old_task) gone = true;
                xSemaphoreGive(s_lock);
                TickType_t remain = deadline - xTaskGetTickCount();
                if (gone || (int32_t)remain <= 0) break;
                uint32_t exited = 0;
                if (xTaskNotifyWait(0, ULONG_MAX, &exited, remain) == pdTRUE
                    && exited == (uint32_t)(intptr_t)old_task) {
                    gone = true;
                }
            }
            if (!gone) {
                ESP_LOGW(TAG, "old session task %p did not exit in 35s (blackout?)",
                         (void *)old_task);
            }
        }
    }
}

esp_err_t bridge_rfc2217_start(uint16_t port)
{
    if (s_started) {
        ESP_LOGI(TAG, "already started on port %u", port);
        return ESP_OK;
    }
    /* 启动 magic：本次运行唯一标识，客户端靠它检测中途是否重启 */
    s_boot_magic = esp_random() | 0x80000000u;
    if (s_lock == NULL) {
        s_lock = xSemaphoreCreateMutex();
        if (s_lock == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }
    /* v0.3.1: subscribe for multi-client fanout (TCP + RFC2217 + WS all parallel) */
    if (s_rfc_sub == NULL) {
        s_rfc_sub = bridge_serial_subscribe("rfc");
        if (s_rfc_sub == NULL) {
            ESP_LOGE(TAG, "subscribe rfc failed");
            return ESP_ERR_NO_MEM;
        }
    }
    BaseType_t ok1 = xTaskCreate(accept_task, "rfc2217_acc",
                                 RFC2217_TASK_STACK, (void *)(intptr_t)port, 5, &s_acc_task);
    BaseType_t ok2 = xTaskCreate(ser_to_tcp_task, "rfc2ser_tx",
                                 RFC2217_TASK_STACK, NULL, 5, NULL);
    if (ok1 == pdPASS && ok2 == pdPASS) {
        s_started = true;
        return ESP_OK;
    }
    return ESP_ERR_NO_MEM;
}

void bridge_rfc2217_stop(void)
{
    /* v0.2.0 不实现 stop（服务生命周期随固件） */
}
