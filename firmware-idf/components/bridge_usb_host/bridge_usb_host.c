/*
 * bridge_usb_host.c — USB Host CDC-ACM 桥接实现
 *
 * RX 数据流: CDC-ACM data_cb → bridge_serial_on_rx() → 订阅者广播 (TCP/RFC2217)
 * TX 数据流: bridge_serial_write() → ser_tx task → bridge_usb_host_send() → cdc_acm_host_data_tx_blocking()
 */
#include "bridge_usb_host.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "usb/usb_host.h"
#include "usb/cdc_acm_host.h"
#include "usb/cdc_host_types.h"
#include "esp_private/cdc_host_common.h"
#include "usb/vcp_ch34x.h"

#include <string.h>

#define TAG "bridge_usb_host"

static cdc_acm_dev_hdl_t s_cdc_hdl = NULL;
static volatile bool s_driver_installed;
static volatile bool s_device_open;

/* 分层字节计数：rx=CDC data_cb 实收，tx=send 成功字节，tx_fail=send 失败次数 */
static volatile size_t s_dbg_cdc_rx;
static volatile size_t s_dbg_cdc_tx;
static volatile size_t s_dbg_cdc_tx_fail;

/* 拔出回调：让主程序把 transport 切回 UART 兜底（不切的话拔线期间 send 全返 INVALID_STATE） */
static bridge_usb_host_disconnect_cb_t s_disc_cb;
static void *s_disc_arg;

/* WCH CH340 厂商请求（与 Linux drivers/usb/serial/ch341.c 一致）。
 * 注意：managed component 里 ch34x_vcp 的 0xA4 wValue 用 0x60 基值（其余位写 0），
 * 实测 CH340 对这种写法整体忽略——DTR/RTS 物理引脚不动。Linux 的写法是 ~mcr：
 * 保留位全部为 1（u16 高字节也是 0xFF），仅 bit5=~DTR/bit6=~RTS（0=有效），
 * 即双断言 0xFF9F/双释放 0xFFFF。 */
#define CH340_REQ_TYPE_OUT   0x40   /* VENDOR | DEVICE | OUT */
#define CH340_REQ_READ_REG   0x95   /* 读寄存器——CH340 0x31 克隆执行后固件崩溃，禁用 */
#define CH340_REQ_WRITE_REG  0x9A
#define CH340_REQ_SERIAL_INIT 0xA1
#define CH340_REQ_MODEM_OUT  0xA4
#define CH340_REQ_READ_VERSION 0x5F /* 版本号（IN 2B）——克隆片报 0x31，正品 ≥ 0x40 */
#define CH340_BIT_DTR        0x20
#define CH340_BIT_RTS        0x40

/* 当前打开的设备是否为 WCH CH34x（决定 set_control 走厂商请求还是标准 CDC） */
static volatile bool s_is_wch;

/* 克隆 CH340 检测：open 时读 0x5F 版本，0x31 = 已知克隆版本（实测两块板，
 * 均存在 RX 尾包 ≥64B 楔形卡死硬件缺陷）。-1 = 非 WCH 或未读 */
static int  s_ch340_version = -1;
static bool s_ch340_clone;

/* ---- 控制线合并（coalesce）----
 * esptool 经典自动复位序列按两条 SET_CONTROL 下发（DTR/RTS 各一条），中间态
 * "两线同时断言"在 NodeMCU 式双管交叉电路上等于 EN/IO0 全释放。直连 USB 时
 * 两条请求间隔亚毫秒可被复位 RC 滤掉；经 RFC2217/WiFi 间隔被拉到 30~60ms，
 * 会让 EN 在 IO0 还是高时提前抬起，目标正常启动（实测进不了下载模式）。
 * 因此把"双线同时断言"态压住一小段时间：若后续状态很快到来（序列的真正
 * 落点通常是仅 DTR 断言），直接发最终态，跳过多余的 EN 脉冲。 */
#define MODEM_BOTH_HOLD_MS   150

/* ---- 原生 CDC（含 ESP32-S3/C3/C6 USB-Serial-JTAG）复位整形 ----
 * 经典序列先 RTS_ON（EN 低）100ms 后才 DTR_ON（IO0 低），而 USB-JTAG 在
 * EN 下降沿采样 GPIO0，且 RTS 一下拉芯片立刻复位、USB 掉线上的后续写全丢。
 * 因此收到 RTS_ON 时先压住，等尾随的 DTR_ON（经典序列主机侧 100ms 后发，
 * 实测经 WiFi RFC2217 送达可达 +210ms，守卫取 300ms 覆盖网络抖动）：
 * 窗口内 DTR 出现=boot，按官方 USB-JTAG 脉冲列驱动；RTS 先释放=hard reset，
 * 补发保底 100ms EN 脉冲；超时仍按住=USB 外设 200ms 型 hard reset。*/
#define NATIVE_RTS_GUARD_MS  300
#define NATIVE_IO0_SETUP_MS  100  /* IO0 拉低到 EN 下降沿的建立时间（官方 settle 0.1s，
                                   * C3 实测 50ms 不够：USB-JTAG 检测有 RTC 级阈值）*/
#define NATIVE_EN_HOLD_MS    100  /* EN 低电平保持（IO0 已提前释放），同样取官方 0.1s */
static TaskHandle_t s_modem_task_hdl;
static volatile bool s_target_dtr;
static volatile bool s_target_rts;

/* 当前线路参数（open 默认 115200 8N1，RFC2217 SET_BAUDRATE 经 bridge_usb_host_set_baud 更新） */
static cdc_acm_line_coding_t s_line_coding = {
    .dwDTERate   = 115200,
    .bCharFormat = 0,
    .bParityType = 0,
    .bDataBits   = 8,
};

void bridge_usb_host_set_disconnect_callback(bridge_usb_host_disconnect_cb_t cb, void *arg)
{
    s_disc_cb = cb;
    s_disc_arg = arg;
}

void bridge_usb_host_debug(size_t *cdc_rx, size_t *cdc_tx, size_t *cdc_tx_fail)
{
    if (cdc_rx)      *cdc_rx      = s_dbg_cdc_rx;
    if (cdc_tx)      *cdc_tx      = s_dbg_cdc_tx;
    if (cdc_tx_fail) *cdc_tx_fail = s_dbg_cdc_tx_fail;
}

static void usb_host_events_task(void *arg); /* 前向声明 */
static void modem_coalesce_task(void *arg); /* 前向声明 */
static esp_err_t modem_emit(bool dtr, bool rts); /* 前向声明（open 尾部重放用） */

/* ---- CDC RX 回调 ---- */
static bool cdc_data_cb(const uint8_t *data, size_t data_len, void *user_arg)
{
    (void)user_arg;
    if (data && data_len) {
        s_dbg_cdc_rx += data_len;
        static int s_log_cnt = 0;
        if (s_log_cnt++ < 10) {
            ESP_LOGI(TAG, "cdc_data_cb: %zu bytes -> bridge_serial_on_rx()", data_len);
        }
        bridge_serial_on_rx(data, data_len);
    }
    return true; /* 已处理，可 flush RX buffer */
}

/* ---- CDC 设备事件回调 ---- */
static void cdc_event_cb(const cdc_acm_host_dev_event_data_t *event, void *user_ctx)
{
    (void)user_ctx;
    if (!event) return;
    switch (event->type) {
    case CDC_ACM_HOST_DEVICE_DISCONNECTED:
        ESP_LOGW(TAG, "CDC-ACM device DISCONNECTED");
        s_device_open = false;
        s_ch340_version = -1;
        s_ch340_clone = false;
        if (s_cdc_hdl) {
            cdc_acm_host_close(s_cdc_hdl);
            s_cdc_hdl = NULL;
        }
        /* 立即切回 UART 兜底：USB 拔出到 retry 重开成功有十几秒窗口，
         * 不切的话这期间所有下发字节都 INVALID_STATE */
        if (s_disc_cb) s_disc_cb(s_disc_arg);
        break;
    case CDC_ACM_HOST_ERROR:
        ESP_LOGE(TAG, "CDC-ACM error: %d", event->data.error);
        break;
    case CDC_ACM_HOST_SERIAL_STATE:
        break;
    case CDC_ACM_HOST_NETWORK_CONNECTION:
        break;
    default:
        break;
    }
}

/* ---- bridge_serial transport send ---- */
static esp_err_t host_send_fn(const uint8_t *d, size_t l, void *a)
{
    (void)a;
    if (!s_device_open || !s_cdc_hdl) return ESP_ERR_INVALID_STATE;
    esp_err_t r = cdc_acm_host_data_tx_blocking(s_cdc_hdl, d, l, 200);
    if (r != ESP_OK) {
        s_dbg_cdc_tx_fail++;
        ESP_LOGW(TAG, "TX %d bytes failed: %s", (int)l, esp_err_to_name(r));
    } else {
        s_dbg_cdc_tx += l;
    }
    return r;
}

static const bridge_serial_transport_t s_host_transport = {
    .send = host_send_fn,
    .arg  = NULL,
};

/* ---- Init: install USB Host Library + CDC-ACM driver ---- */
esp_err_t bridge_usb_host_init(void)
{
    if (s_driver_installed) return ESP_OK;

    /* USB Host Library */
    usb_host_config_t host_cfg = {0};
    esp_err_t ret = usb_host_install(&host_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "usb_host_install failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "USB Host Library installed");

    /* CDC-ACM driver — v0.5: 明确设 driver_task_priority=7，
     * 默认 10 太激进，会饿死 HTTP server/task_mgr 等低优先级任务 */
    cdc_acm_host_driver_config_t cdc_drv_cfg = {
        .driver_task_stack_size = 4096,
        .driver_task_priority   = 7,   /* WiFi 默认 23，USB Host events 4，HTTP 6 */
        .xCoreID                = 0,
        .new_dev_cb             = NULL,
    };
    ret = cdc_acm_host_install(&cdc_drv_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "cdc_acm_host_install failed: %s", esp_err_to_name(ret));
        usb_host_uninstall();
        return ret;
    }
    s_driver_installed = true;
    ESP_LOGI(TAG, "CDC-ACM driver installed (prio=7)");

    /* USB Host lib event handler task — 处理 USB 总线事件
     * USB Host Library 内部枚举/HUB 处理需要较大栈，给 8KB 留余量 */
    xTaskCreate(usb_host_events_task, "usb_host_evt", 8192, NULL, 4, NULL);

    /* 控制线合并任务：抑制交叉复位电路上的"双线断言"瞬态（见 coalesce 注释）*/
    xTaskCreate(modem_coalesce_task, "usb_modem", 3072, NULL, 5, &s_modem_task_hdl);

    return ESP_OK;
}

/* USB Host Library 事件处理任务（阻塞等事件，驱动 USB 总线枚举等）*/
static void usb_host_events_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "usb_host_events_task started");
    uint32_t event_flags = 0;
    while (1) {
        /* usb_host_lib_handle_events 会阻塞直到有事件 */
        esp_err_t ret = usb_host_lib_handle_events(pdMS_TO_TICKS(100), &event_flags);
        if (ret != ESP_OK && ret != ESP_ERR_TIMEOUT) {
            ESP_LOGW(TAG, "usb_host_lib_handle_events ret=%s", esp_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(20));
        }
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            ESP_LOGI(TAG, "USB Host: all clients deregistered");
        }
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
            ESP_LOGI(TAG, "USB Host: all devices freed");
        }
    }
}

/* ---- CH340 厂商请求（Linux ch341 行为；managed VCP 组件的 0xA4 实现实测无效）---- */
static esp_err_t ch340_control_out(uint8_t req, uint16_t value, uint16_t index)
{
    return cdc_acm_host_send_custom_request(s_cdc_hdl,
            CH340_REQ_TYPE_OUT, req, value, index, 0, NULL);
}

/* SERIAL_INIT：Linux ch341_configure 在任何寄存器写之前发一次，
 * 不发的话 CH340 UART/modem 输出使能不完整（实测 DTR/RTS 无作用）*/
static esp_err_t ch340_serial_init(void)
{
    return ch340_control_out(CH340_REQ_SERIAL_INIT, 0, 0);
}

/* 寄存器对写：wValue=寄存器对（高/低字节各一个寄存器号），wIndex=16 位值。
 * LCR 0x2518 由 VCP 的 line_coding_set 代发；0x2727 是 FLOW_CTL 寄存器对。 */
static esp_err_t ch340_write_reg(uint16_t reg_pair, uint16_t value)
{
    return ch340_control_out(CH340_REQ_WRITE_REG, reg_pair, value);
}

/* 第二次 SERIAL_INIT（"magic init"）：wValue=0x501f/wIndex=0xd90a。
 * Windows WCH 驱动（以及据抓包复刻的 usb-serial-for-android）在 LCR 写完后、
 * 首次 0xA4 之前必发一次；缺它时实测 CH340 版本 0x31 对 0xA4 全部 ACK 但
 * DTR/RTS 物理引脚不动。WCH 厂商 Linux 驱动里没有这一发——克隆/新版芯片上
 * Linux 用户反复遇到的 auto-reset 失效即源于此。
 * 副作用：magic 会让波特率寄存器失效，之后必须重写一次波特率。*/
static esp_err_t ch340_magic_init(void)
{
    return ch340_control_out(CH340_REQ_SERIAL_INIT, 0x501f, 0xd90a);
}

/* 波特率必须成对写（Windows 抓包序列；VCP 组件只写 0x1312，magic 后会乱码）：
 *   0x9A 0x1312←val1（分频高半字 + divisor，bit7=立即生效）
 *   0x9A 0x0f2c←val2（分频低半字）
 * 系数公式复刻 usb-serial-for-android（CH340 datasheet 的 2^(12-3ps-fact) 分频）。*/
#define CH340_BAUD_FACTOR_NUM 1532620800u
static esp_err_t ch340_set_baudrate(uint32_t baud)
{
    uint32_t factor;
    uint32_t divisor;
    if (baud == 921600) {
        factor = 0xf300;
        divisor = 7;
    } else if (baud == 307200) {
        factor = 0xd900;
        divisor = 7;
    } else {
        if (baud == 0) return ESP_ERR_INVALID_ARG;
        factor = CH340_BAUD_FACTOR_NUM / baud;
        divisor = 3;
        while (factor > 0xfff0 && divisor > 0) {
            factor >>= 3;
            divisor--;
        }
        if (factor > 0xfff0) return ESP_ERR_INVALID_ARG;
        factor = 0x10000 - factor;
    }
    divisor |= 0x80; /* RX 定时器使能（版本>0x27 置位；0x31 克隆定时器不生效但保留） */
    esp_err_t r = ch340_control_out(CH340_REQ_WRITE_REG, 0x1312,
                                   (factor & 0xff00) | divisor);
    if (r != ESP_OK) return r;
    return ch340_control_out(CH340_REQ_WRITE_REG, 0x0f2c, factor & 0xff);
}

/* DTR/RTS：Linux 把 ~mcr 作为 u16 wValue，高字节必须 0xFF
 *（双断言 0xFF9F / 仅DTR 0xFFBF / 仅RTS 0xFFDF / 双释放 0xFFFF），
 * 只发低字节实测 CH340 整包忽略。*/
static esp_err_t ch340_set_control(bool dtr, bool rts)
{
    uint16_t value = 0xFFFF;
    if (dtr) value &= ~CH340_BIT_DTR;
    if (rts) value &= ~CH340_BIT_RTS;
    return ch340_control_out(CH340_REQ_MODEM_OUT, value, 0);
}

/* ---- Open: 阻塞直到 CDC-ACM 设备插入 ---- */
esp_err_t bridge_usb_host_open(uint32_t timeout_ms)
{
    if (!s_driver_installed) {
        ESP_LOGE(TAG, "not inited");
        return ESP_ERR_INVALID_STATE;
    }
    if (s_device_open) {
        return ESP_OK;
    }

    /* 第一选择：WCH CH340/CH341/CH342/CH343/CH9101/CH9102 等厂商类芯片——
     * 它们不响应标准 SET_LINE_CODING，必须走厂商请求设波特率。
     * CH34X_PID_AUTO 只匹配 VID 1A86 的受支持 PID（含 CH9102F 的 55D2/55D4）。*/
    cdc_acm_host_device_config_t vcp_cfg = {
        .connection_timeout_ms = timeout_ms ? timeout_ms : 10000,
        .out_buffer_size       = 256,   /* bulk OUT max packet */
        .in_buffer_size        = 256,   /* bulk IN max packet */
        .event_cb              = cdc_event_cb,
        .data_cb               = cdc_data_cb,
        .user_arg              = NULL,
    };
    esp_err_t ret = ch34x_vcp_open(CH34X_PID_AUTO, 0, &vcp_cfg, &s_cdc_hdl);
    if (ret == ESP_OK) {
        s_is_wch = true;
        ESP_LOGI(TAG, "Opened as WCH ch34x VCP device");
    } else {
        /* 回退：标准 CDC-ACM 设备（CH343 CDC 模式/CH9102 CDC 模式/其他原生 CDC）*/
        cdc_acm_host_open_config_t cfg = {
            .vid                     = CDC_HOST_ANY_VID,
            .pid                     = CDC_HOST_ANY_PID,
            .interface_idx           = 0,
            .dev_addr                = CDC_HOST_ANY_DEV_ADDR,
            .connection_timeout_ms   = timeout_ms ? timeout_ms : 10000,
            .out_buffer_size         = 256,
            .in_buffer_size          = 256,
            .event_cb                = cdc_event_cb,
            .data_cb                 = cdc_data_cb,
            .user_arg                = NULL,
        };
        ESP_LOGI(TAG, "not a WCH VCP (%s), trying generic CDC-ACM...",
                 esp_err_to_name(ret));
        ret = cdc_acm_host_open(&cfg, &s_cdc_hdl);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "cdc_acm_host_open failed: %s (no serial device?)",
                     esp_err_to_name(ret));
            return ret;
        }
        /* 防误认领（实测竞态）：多级 HUB 枚举慢时，ch34x_vcp_open 的三档 PID
         * 扫描会在设备就绪前全部 NOT_FOUND，而 ANY_VID/PID 的通用 open 随即成功——
         * 连 bInterfaceClass=0xFF 的 CH340 厂商专用接口也照单全收。通用 ops 发的
         * 标准 SET_CONTROL_LINE_STATE/LINE_CODING CH340 一概不理（只认 0xA4 厂商
         * 请求），会导致 DTR/RTS/波特率全部静默失效。发现这种错配必须立刻放手，
         * 当场重试 VCP open，不能让它以错误驱动锁死到下次热插拔。 */
        const usb_intf_desc_t *dintf = s_cdc_hdl->data.intf_desc;
        const usb_device_desc_t *ddesc = NULL;
        usb_host_get_device_descriptor(s_cdc_hdl->dev_hdl, &ddesc);
        uint16_t vid = ddesc ? ddesc->idVendor : 0;
        if (dintf && dintf->bInterfaceClass == USB_CLASS_VENDOR_SPEC &&
            vid == NANJING_QINHENG_MICROE_VID) {
            ESP_LOGW(TAG, "generic CDC mis-claimed WCH vendor-class intf (VID %04x/class 0xff) "
                     "— closing and retrying WCH VCP open", vid);
            cdc_acm_host_close(s_cdc_hdl);
            s_cdc_hdl = NULL;
            vTaskDelay(pdMS_TO_TICKS(200));
            ret = ch34x_vcp_open(CH34X_PID_AUTO, 0, &vcp_cfg, &s_cdc_hdl);
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "VCP retry still %s — leave for next scan cycle",
                         esp_err_to_name(ret));
                return ret;
            }
            s_is_wch = true;
            ESP_LOGI(TAG, "Opened as WCH ch34x VCP device (recovered from generic race)");
        } else {
            s_is_wch = false;
            ESP_LOGI(TAG, "Opened as generic CDC-ACM compliant device");
        }
    }

    if (s_is_wch) {
        /* Windows WCH 驱动 open 序列（据 usb-serial-for-android 抓包复刻；
         * VCP 组件只做了单次 0x1312 波特率写，缺 magic/流控/对写，实测
         * CH340 版本 0x31 上 0xA4 全部 ACK 但 DTR/RTS 引脚不动）：
         *   0xA1(0,0) → 波特率对写(0x1312+0x0f2c) → LCR 0x2518←0xC3(8N1)
         *   → 0xA1(0x501f,0xd90a) magic（使能 modem 引脚控制，同时复位波特率）
         *   → 波特率对写重发 → 0x9A 0x2727←0（FLOW_CTL 关，防 RTS 被 FIFO 接管）
         *   → 0xA4 释放两线。*/
        esp_err_t ir = ch340_serial_init();
        ESP_LOGI(TAG, "CH340 SERIAL_INIT(0xA1 0000): %s", esp_err_to_name(ir));
        esp_err_t rb1 = ch340_set_baudrate(s_line_coding.dwDTERate);
        esp_err_t rl = ch340_write_reg(0x2518, 0xC3); /* RX|TX enable + CS8 */
        esp_err_t rm = ch340_magic_init();
        ESP_LOGI(TAG, "CH340 magic 0xA1(501f/d90a): %s", esp_err_to_name(rm));
        esp_err_t rb2 = ch340_set_baudrate(s_line_coding.dwDTERate);
        esp_err_t rf = ch340_write_reg(0x2727, 0x0000);
        ESP_LOGI(TAG, "CH340 baud pre=%s lcr=%s post=%s flow=%s",
                 esp_err_to_name(rb1), esp_err_to_name(rl),
                 esp_err_to_name(rb2), esp_err_to_name(rf));
        ch340_set_control(false, false); /* 空闲释放两线：目标正常运行，不按住 EN */

        /* 克隆检测：读 0x5F 版本。0x31 = 已知克隆（RX 尾包 ≥64B 楔形卡死，
         * 芯片级硬件缺陷，固件无解）。0x95 读寄存器对克隆是致命操作，0x5F 安全。*/
        uint8_t ver[2] = {0};
        s_ch340_version = -1;
        s_ch340_clone = false;
        if (cdc_acm_host_send_custom_request(s_cdc_hdl, 0xC0,
                    CH340_REQ_READ_VERSION, 0, 0, sizeof(ver), ver) == ESP_OK) {
            s_ch340_version = ver[0];
            s_ch340_clone = (ver[0] == 0x31);
            if (s_ch340_clone) {
                ESP_LOGW(TAG, "========================================================");
                ESP_LOGW(TAG, "  CLONE CH340 DETECTED (version 0x%02X)", ver[0]);
                ESP_LOGW(TAG, "  Third-party counterfeit chip with a HARDWARE defect:");
                ESP_LOGW(TAG, "  RX tail packets >=64B wedge; bulk transfers stall.");
                ESP_LOGW(TAG, "  esptool read/write will likely fail.");
                ESP_LOGW(TAG, "  This is a chip fault, NOT an AirSerial compatibility");
                ESP_LOGW(TAG, "  issue. Use genuine CH340C/CH343, CP2102, or a");
                ESP_LOGW(TAG, "  native USB-Serial-JTAG target board instead.");
                ESP_LOGW(TAG, "========================================================");
            } else {
                ESP_LOGI(TAG, "CH340 version 0x%02X (genuine)", ver[0]);
            }
        } else {
            ESP_LOGW(TAG, "CH340 version read failed — treat as unknown");
        }
    } else {
        /* 配 line coding（默认 115200 8N1；s_line_coding 随后续 SET_BAUDRATE 更新）*/
        s_ch340_version = -1;
        s_ch340_clone = false;
        cdc_acm_host_line_coding_set(s_cdc_hdl, &s_line_coding);
        cdc_acm_host_set_control_line_state(s_cdc_hdl, false, false); /* 空闲态，不主动断言 */
    }

    ESP_LOGI(TAG, "CDC-ACM device OPENED! (%lu 8N1, control lines released)",
             (unsigned long)s_line_coding.dwDTERate);
    cdc_acm_host_desc_print(s_cdc_hdl);

    s_device_open = true;

    /* 重放主机侧最后一次线状态：目标可能是 EN 复位后 ROM 重新枚举上来的，
     * 掉线窗口里到达的 SET_CONTROL 只更新了 s_target_*，重开后补发。*/
    if (s_target_dtr || s_target_rts) {
        modem_emit(s_target_dtr, s_target_rts);
    }
    return ESP_OK;
}

/* —— RFC2217 带外控制转发：esptool 自动复位电路靠 RTS→EN / DTR→GPIO0 的快速翻转 —— */

/* 真正下发一次控制线状态（CH340 走 Linux 风格厂商请求 wValue=~mcr，其余走标准 CDC）*/
static esp_err_t modem_emit(bool dtr, bool rts)
{
    if (!s_cdc_hdl) return ESP_ERR_INVALID_STATE;
    /* 诊断：每条控制线写的返回值/耗时（经 HUB split 可能数 ms）*/
    static int64_t s_last_us;
    int64_t t0 = esp_timer_get_time();
    esp_err_t r = s_is_wch ? ch340_set_control(dtr, rts)
                           : cdc_acm_host_set_control_line_state(s_cdc_hdl, dtr, rts);
    int64_t t1 = esp_timer_get_time();
    ESP_LOGI(TAG, "MODEM_OUT dtr=%d rts=%d ret=%s cost=%lldus gap=%lldus",
             (int)dtr, (int)rts, esp_err_to_name(r),
             (long long)(t1 - t0), (long long)(s_last_us ? t0 - s_last_us : -1));
    s_last_us = t1;
    return r;
}

/* 控制线任务。
 * WCH CH340 交叉复位板：单线断言态立即下发；"双线同时断言"先压住
 * MODEM_BOTH_HOLD_MS，期间来了新状态就以最新态为准（跳过交叉电路上等于
 * 全释放的中间态）。
 * 非 WCH（标准 CDC / ESP 原生 USB-Serial-JTAG，直连电路）：RTS_ON 先压住
 * NATIVE_RTS_GUARD_MS 等 DTR_ON，把经典序列整形成 IO0 先低、EN 后落。*/
static void modem_coalesce_task(void *arg)
{
    (void)arg;
    for (;;) {
        xTaskNotifyWait(0, 0xFFFFFFFF, NULL, portMAX_DELAY);
        bool dtr = s_target_dtr;
        bool rts = s_target_rts;

        if (!s_is_wch && rts && !dtr) {
            /* EN 将被拉低：等经典序列尾随的 DTR_ON（网络送达实测可达 +210ms）*/
            bool boot_shape = false;
            bool rts_released = false;
            int waited_ms = 0;
            while (waited_ms < NATIVE_RTS_GUARD_MS) {
                int slice_ms = 20;
                if (xTaskNotifyWait(0, 0xFFFFFFFF, NULL,
                                    pdMS_TO_TICKS(slice_ms)) == pdTRUE) {
                    dtr = s_target_dtr;
                    rts = s_target_rts;
                    if (dtr) { boot_shape = true; break; }       /* 只要 IO0 参与=boot 意图 */
                    if (!rts) { rts_released = true; break; }    /* hard reset 释放/瞬态 */
                    /* dtr=0,rts=1 继续等 */
                }
                waited_ms += slice_ms;
            }
            if (boot_shape) {
                /* 复刻 Espressif 官方 usb_jtag_bootloader_reset 脉冲列
                 *（esp_pylib/serial_reset.py，C3/S3/C6 USB-Serial-JTAG 通用）：
                 * IO0 先低并建立 → EN 拉低 → 复位保持期内释放 IO0 → EN 抬起。
                 * 无间隔的两连发 S3 能过、C3 strapping 读到 0xc 正常启动，
                 * 必须给足建立/保持时间。主机随后补发的经典序残尾只动单线，无害。*/
                modem_emit(true, false);
                vTaskDelay(pdMS_TO_TICKS(NATIVE_IO0_SETUP_MS));
                modem_emit(true, true);
                modem_emit(false, true);   /* 复位保持期立刻释放 IO0（对齐官方序列）*/
                vTaskDelay(pdMS_TO_TICKS(NATIVE_EN_HOLD_MS));
                modem_emit(false, false);
                continue;
            }
            if (rts_released) {
                /* hard reset 的 EN 低电平被网络抖动挤进守卫窗口：
                 * 自己补发至少 100ms 的完整 EN 脉冲，不能漏复位 */
                modem_emit(false, true);
                vTaskDelay(pdMS_TO_TICKS(100));
                modem_emit(false, false);
                continue;
            }
            /* 守卫超时仍是 RTS 低电平（USB 外设 hard reset 持 200ms 的常态）：
             * 此刻才真正把 EN 拉低，保底保持 100ms 再按主机最新态释放 */
            modem_emit(false, true);
            vTaskDelay(pdMS_TO_TICKS(100));
            modem_emit(s_target_dtr, s_target_rts);
            continue;
        }

        while (s_is_wch && dtr && rts) {
            if (xTaskNotifyWait(0, 0xFFFFFFFF, NULL,
                                pdMS_TO_TICKS(MODEM_BOTH_HOLD_MS)) != pdTRUE) {
                break; /* 两线态被保持：少数直连电路确实需要，到时照发 */
            }
            dtr = s_target_dtr;
            rts = s_target_rts;
        }
        modem_emit(dtr, rts);
    }
}

esp_err_t bridge_usb_host_set_control(bool dtr, bool rts)
{
    /* 即使设备正处于复位重枚举窗口（s_cdc_hdl=NULL）也要记下主机期望态：
     * open 成功尾部会重放，整形任务也靠通知驱动时序。*/
    s_target_dtr = dtr;
    s_target_rts = rts;
    if (s_modem_task_hdl) xTaskNotifyGive(s_modem_task_hdl);
    return ESP_OK;
}

esp_err_t bridge_usb_host_set_baud(uint32_t baud)
{
    if (!s_cdc_hdl) return ESP_ERR_INVALID_STATE;
    if (baud == 0 || baud == s_line_coding.dwDTERate) return ESP_OK;
    s_line_coding.dwDTERate = baud;
    /* CH340 走 Windows 抓包的波特率对写（0x1312+0x0f2c）；标准 CDC 走 SET_LINE_CODING */
    return s_is_wch ? ch340_set_baudrate(baud)
                    : cdc_acm_host_line_coding_set(s_cdc_hdl, &s_line_coding);
}

/* 任意 vendor 控制传输（RFC2217 CLI "usbctrl" 后端，用于在线扫描芯片差异）。
 * 安全护栏：CH340 0x31 克隆执行 0x95 读寄存器后固件崩溃（掉电才能恢复），
 * 对 WCH 设备直接拒绝该请求。*/
esp_err_t bridge_usb_host_raw_ctrl(uint8_t bm_req_type, uint8_t b_req,
                                   uint16_t w_value, uint16_t w_index,
                                   uint8_t *in_buf, size_t in_len)
{
    if (!s_cdc_hdl) return ESP_ERR_INVALID_STATE;
    if (s_is_wch && b_req == CH340_REQ_READ_REG) {
        ESP_LOGW(TAG, "blocked CH340 0x95 read-reg (crashes 0x31 clone fw)");
        return ESP_ERR_NOT_SUPPORTED;
    }
    return cdc_acm_host_send_custom_request(s_cdc_hdl, bm_req_type, b_req,
                                            w_value, w_index, in_len, in_buf);
}

esp_err_t bridge_usb_host_close(void)
{
    if (!s_cdc_hdl) return ESP_OK;
    esp_err_t r = cdc_acm_host_close(s_cdc_hdl);
    s_cdc_hdl = NULL;
    s_device_open = false;
    s_ch340_version = -1;
    s_ch340_clone = false;
    ESP_LOGI(TAG, "CDC-ACM closed: %s", esp_err_to_name(r));
    return r;
}

/* 克隆 CH340 状态查询：version <0 表示非 WCH/未知；clone 仅 0x31 置真 */
int  bridge_usb_host_ch340_version(void) { return s_ch340_version; }
bool bridge_usb_host_ch340_is_clone(void) { return s_ch340_clone; }
bool bridge_usb_host_is_wch(void) { return s_is_wch && s_device_open; }

void bridge_usb_host_deinit(void)
{
    bridge_usb_host_close();
    if (s_driver_installed) {
        cdc_acm_host_uninstall();
        usb_host_uninstall();
        s_driver_installed = false;
        ESP_LOGI(TAG, "USB Host fully deinited");
    }
}

bool bridge_usb_host_is_open(void)
{
    return s_device_open;
}

const bridge_serial_transport_t *bridge_usb_host_transport(void)
{
    return &s_host_transport;
}
