/*
 * board_powerhub — M5Stack PowerHub 板级胶水
 *
 * 硬件:
 *   - ESP32-S3-N16R8 (16MB Flash + 8MB Octal PSRAM)
 *   - STM32G031G8U6 协处理器 (I2C 0x50, SDA=G45/SCL=G48)
 *     管理: 各口 5V 供电开关 / USB Switch / 6 路 INA226 / WS2812 LED / RTC / 按键
 *   - 底部 Type-C: USB-SERIAL-JTAG + OTG 共享 PHY → SHARED_PHY topo
 *   - 正面 Type-A/Type-C: 经 STM32 USB Switch (reg 0x10) 切数据线 + PC_USB (reg 0x01) 供 5V
 *
 * Tier 0: 无屏无键盘 → headless，状态全靠日志 + Web
 *
 * 参考: reference/powerhub/POWERHUB_DEV_REFERENCE.md
 */
#include "board.h"

#include "esp_log.h"
#include "esp_efuse.h"
#include "esp_efuse_table.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <string.h>

#define TAG "board_powerhub"

/* — STM32 协处理器 I2C — */
#define PH_I2C_NUM      I2C_NUM_0
#define PH_I2C_SDA_GPIO 45
#define PH_I2C_SCL_GPIO 48
#define PH_STM32_ADDR   0x50

/* — STM32 寄存器（详见 POWERHUB_DEV_REFERENCE.md 第二节）— */
#define PH_REG_PC_LED      0x00   /* WS2812 LED 供电 */
#define PH_REG_PC_USB      0x01   /* 正面 USB 口 5V 输出 */
#define PH_REG_PC_I2C      0x02   /* PORT.A Grove 5V */
#define PH_REG_PC_UART     0x03   /* PORT.C Grove 5V */
#define PH_REG_PC_BUS      0x04   /* EXT 总线电源 */
#define PH_REG_PC_VAMETER  0x05   /* 6 路 INA226 监测供电 */
#define PH_REG_USB_MODE    0x10   /* 0=SLAVE(底部C口) 1=HOST_TYPE_A 2=HOST_TYPE_C */
#define PH_REG_VM_BAT_V    0x30   /* 电池电压 mV (u16 LE) */
#define PH_REG_CHG_STATUS  0x50   /* 0=未充电 1=充电中 2=异常 */
#define PH_REG_EXT_POWER   0x51   /* 0=电池供电 ≠0=外部供电 */
#define PH_REG_FW_VERSION  0xFE   /* STM32 固件版本 */

/* — 全局 I2C 句柄 — */
static i2c_master_bus_handle_t s_i2c_bus = NULL;
static i2c_master_dev_handle_t s_stm32_dev = NULL;
static bool s_stm32_online = false;

/* — 设备描述 — */
static board_info_t s_info = {
    .name       = "M5Stack PowerHub",
    .target     = "esp32s3",
    .has_lcd    = false,
    .has_kbd    = false,
    .has_psram  = true,
    .cols       = 30,   /* headless，仅作占位 */
    .rows       = 8,
    .tier       = BOARD_TIER_0_HEADLESS,
};

const board_info_t *board_get_info(void) { return &s_info; }

/* — DTR/RTS 直控：完整实现见本文件后部（EXT 16P 排针 G4/G5） —
 * 正面口 USB Host 路线下 esptool 复位由目标板上 USB-UART 模块自身
 * DTR/RTS 完成；UART 兜底路线（PORT.C G1/G2）由本实现接管。 */

/* — USB 拓扑状态 — */
static board_usb_topo_t s_detected_topo = USB_TOPO_UNKNOWN;

board_usb_topo_t board_get_usb_topo(void) { return s_detected_topo; }

bool board_usb_host_init_safe(void)
{
    /* SHARED_PHY → Host 模式下 USB-Serial-JTAG console 断 */
    return s_detected_topo == USB_TOPO_INDEPENDENT;
}

void board_detect_usb_topo(void)
{
    ESP_LOGI(TAG, "=== USB topology detect ===");

    bool otg_disabled = esp_efuse_read_field_bit(ESP_EFUSE_DIS_USB_OTG);
    ESP_LOGI(TAG, "  efuse OTG disabled: %d", otg_disabled);
    ESP_LOGI(TAG, "  board declared: SHARED_PHY (底部C口共享PHY, 正面口经STM32 USB Switch)");

    if (otg_disabled) {
        s_detected_topo = USB_TOPO_NO_HOST;
        ESP_LOGW(TAG, "  ⚠️ efuse OTG disabled overrides → NO_HOST");
    } else {
        s_detected_topo = USB_TOPO_SHARED_PHY;
    }

    ESP_LOGI(TAG, "  ⇒ USB topo: %s",
             s_detected_topo == USB_TOPO_SHARED_PHY ? "SHARED_PHY" :
             s_detected_topo == USB_TOPO_NO_HOST    ? "NO_HOST (efuse)" : "UNKNOWN");
    ESP_LOGI(TAG, "=== USB topology detect done ===");
}

/* =============================================================
 *  STM32 协处理器 I2C 读写
 * ============================================================= */

static esp_err_t ph_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(s_stm32_dev, buf, 2, 100);
}

static esp_err_t ph_read_reg(uint8_t reg, uint8_t *val)
{
    return i2c_master_transmit_receive(s_stm32_dev, &reg, 1, val, 1, 100);
}

static esp_err_t ph_read_reg16_le(uint8_t reg, uint16_t *val)
{
    uint8_t buf[2];
    esp_err_t err = i2c_master_transmit_receive(s_stm32_dev, &reg, 1, buf, 2, 100);
    if (err == ESP_OK) {
        *val = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
    }
    return err;
}

/* — STM32 初始化 —
 * 时序要求：STM32 上电需 500-700ms 完成初始化；
 * 先 100kHz 探测到 0x50，再升 400kHz（官方 UserDemo 做法）。
 * 失败不 crash——STM32 挂了桥本身还能跑（UART 兜底路线）。
 * 返回 ESP_OK 表示 STM32 在线。 */
static esp_err_t board_stm32_init(void)
{
    ESP_LOGI(TAG, "=== STM32 coprocessor init ===");

    /* 0. 等 STM32 就绪（上电时序底线 500ms） */
    vTaskDelay(pdMS_TO_TICKS(500));

    /* 1. GPIO 先 input+pullup，确保 I2C 线空闲高 */
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pin_bit_mask = (1ULL << PH_I2C_SDA_GPIO) | (1ULL << PH_I2C_SCL_GPIO),
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    vTaskDelay(pdMS_TO_TICKS(10));

    /* 2. I2C bus @ 100kHz */
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = PH_I2C_NUM,
        .sda_io_num = PH_I2C_SDA_GPIO,
        .scl_io_num = PH_I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t err = i2c_new_master_bus(&bus_cfg, &s_i2c_bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C bus create failed: %s", esp_err_to_name(err));
        return err;
    }

    /* 3. 探测 0x50，最多 10 次 × 500ms（APRS 重试策略） */
    bool found = false;
    for (int i = 0; i < 10; i++) {
        if (i2c_master_probe(s_i2c_bus, PH_STM32_ADDR, 200) == ESP_OK) {
            found = true;
            break;
        }
        ESP_LOGW(TAG, "STM32 probe retry %d/10...", i + 1);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    if (!found) {
        ESP_LOGE(TAG, "STM32 not found at 0x%02X — coprocessor offline, "
                      "bridge continues without power management", PH_STM32_ADDR);
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "STM32 found at 0x%02X", PH_STM32_ADDR);

    /* 4. 添加设备 @ 400kHz */
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = PH_STM32_ADDR,
        .scl_speed_hz = 400000,
    };
    err = i2c_master_bus_add_device(s_i2c_bus, &dev_cfg, &s_stm32_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "STM32 device add failed: %s", esp_err_to_name(err));
        return err;
    }

    /* 5. 读固件版本确认通信 */
    uint8_t fw = 0;
    err = ph_read_reg(PH_REG_FW_VERSION, &fw);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "STM32 version read failed: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "STM32 firmware version: %u", fw);

    s_stm32_online = true;
    ESP_LOGI(TAG, "=== STM32 init done ===");
    return ESP_OK;
}

/* =============================================================
 *  电池 / 电源状态
 * ============================================================= */

/* — 电压→百分比查表（PowerHubAPRS 实测校准，2S LiPo 不满充延寿）— */
static int batt_volt_to_percent(uint16_t mv)
{
    static const uint16_t volts[] = {6800, 7000, 7200, 7400, 7600, 7800, 8000, 8100};
    static const uint8_t  pcts[]  = {   0,   10,   20,   35,   55,   75,   92,  100};
    const int n = sizeof(volts) / sizeof(volts[0]);

    if (mv <= volts[0])     return pcts[0];
    if (mv >= volts[n - 1]) return pcts[n - 1];

    for (int i = 0; i < n - 1; i++) {
        if (mv >= volts[i] && mv < volts[i + 1]) {
            return pcts[i] + (int)((uint32_t)(mv - volts[i]) *
                                   (pcts[i + 1] - pcts[i]) /
                                   (volts[i + 1] - volts[i]));
        }
    }
    return 0;
}

/* — 电源状态读取并打印 —
 * 需先开 PC_VAMeter 并等 ~200ms 让 INA226 完成首次转换。
 * INA226 未就绪时读数为 0，mv>1000 才算有效。 */
static void board_power_status_log(void)
{
    if (!s_stm32_online) return;

    /* 开 INA226 监测供电（做减法：只开 VAMeter，LED/其余端口默认全关） */
    esp_err_t err = ph_write_reg(PH_REG_PC_VAMETER, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "PC_VAMeter enable failed: %s", esp_err_to_name(err));
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(200));

    uint16_t bat_mv = 0;
    err = ph_read_reg16_le(PH_REG_VM_BAT_V, &bat_mv);
    if (err == ESP_OK && bat_mv > 1000) {
        ESP_LOGI(TAG, "Battery: %u mV (%d%%)", bat_mv, batt_volt_to_percent(bat_mv));
    } else if (err == ESP_OK) {
        ESP_LOGW(TAG, "Battery voltage invalid (%u mV) — no battery or INA226 not ready", bat_mv);
    } else {
        ESP_LOGE(TAG, "Battery read failed: %s", esp_err_to_name(err));
    }

    uint8_t chg = 0, ext = 0;
    if (ph_read_reg(PH_REG_CHG_STATUS, &chg) == ESP_OK) {
        ESP_LOGI(TAG, "Charge status: %s",
                 chg == 0 ? "not charging" : chg == 1 ? "charging" : "CHARGE ERROR");
    }
    if (ph_read_reg(PH_REG_EXT_POWER, &ext) == ESP_OK) {
        ESP_LOGI(TAG, "External power: %s", ext ? "YES" : "no (battery)");
    }
}

/* =============================================================
 *  USB Host 路线准备（board_usb_host_prepare 弱符号覆写）
 *  正面口工作需同时：PC_USB=1（5V 输出）+ USB_MODE=1/2（数据线切换）
 *  默认切 Type-A（HOST_MODE_TYPE_A），AirSerial 目标板多数经 A 口。
 *  ⚠️ 切走后底部 C 口数据线断开——console 必死，与 SHARED_PHY Host 切换同步发生。
 * ============================================================= */
static uint8_t s_usb_mode = 0;   /* 1=正面A 2=正面C，配合 USB_MODE 寄存器 */

esp_err_t board_usb_host_prepare(void)
{
    if (!s_stm32_online) {
        ESP_LOGE(TAG, "USB host prepare: STM32 offline, cannot route front port");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = ph_write_reg(PH_REG_PC_USB, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "PC_USB enable failed: %s", esp_err_to_name(err));
        return err;
    }
    err = ph_write_reg(PH_REG_USB_MODE, 1);  /* HOST_MODE_TYPE_A */
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "USB_MODE set failed: %s", esp_err_to_name(err));
        return err;
    }
    s_usb_mode = 1;

    ESP_LOGI(TAG, "USB routed: PC_USB=1 (5V on), USB_MODE=1 (front Type-A)");
    return ESP_OK;
}

/* — A/C 口自动回退：cdc_retry 连续失败时 1↔2 互切 —
 * mux 切换对 Host lib 等价一次拔插，retry 循环天然吃这个。 */
bool board_usb_host_switch_alt(void)
{
    if (!s_stm32_online) return false;

    uint8_t next = (s_usb_mode == 1) ? 2 : 1;
    esp_err_t err = ph_write_reg(PH_REG_USB_MODE, next);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "USB_MODE switch failed: %s", esp_err_to_name(err));
        return false;
    }
    s_usb_mode = next;
    ESP_LOGW(TAG, "USB switch: front port → Type-%s (auto fallback)",
             next == 1 ? "A" : "C");
    return true;
}

/* =============================================================
 *  VM_USB 监测 + 倒灌保护
 *  VM_USB = 正面 USB 口 5V 轨（INA226 @ STM32 寄存器 0x3C=V 0x3E=I）。
 *  电流符号约定：+=流出（正常供电给目标板） -=流入（外部 5V 倒灌特征）。
 *  采集期 1Hz 打日志标定；倒灌保护：≤-50mA 连续 2 采样（~2s）→ 切 PC_USB=0
 *  并闩锁报警（重启前不再开 PC_USB）。阈值实测后调整。
 * ============================================================= */
#define VMUSB_REG_V        0x3C   /* uint16 LE, mV */
#define VMUSB_REG_I        0x3E   /* int16  LE, mA */
#define BACKFEED_MA_TH     (-20)  /* 触发阈值：≤-20mA 视为倒灌（正常态只会是正/零，实测 PC 源注入 ≈-25mA） */
#define BACKFEED_SAMPLES   2      /* 连续命中次数（1Hz 采样 → ~2s） */

static volatile int16_t s_vmusb_ma;
static volatile uint16_t s_vmusb_mv;
static volatile bool s_backfeed_trip;
static int s_backfeed_streak;

bool board_powerhub_vmusb_get(int16_t *ma, uint16_t *mv)
{
    if (!s_stm32_online) return false;
    *ma = s_vmusb_ma;
    *mv = s_vmusb_mv;
    return true;
}

bool board_powerhub_backfeed_tripped(void)
{
    return s_backfeed_trip;
}

/* =============================================================
 *  WS2812 状态灯（STM32 代管，8 颗）
 *  颜色 0x60+idx*4（4 字节/颗，文档标 "BGR?" 待实测校正——
 *  字节序不符时表现为颜色互换，首灯测试后改 PH_LED_ORDER）；
 *  亮度 0x80+idx（每颗 1 字节）。PC_LED(0x00) 必须先开。
 * ============================================================= */
#define PH_REG_LED_COLOR   0x60   /* 8 颗 × 4B */
#define PH_REG_LED_BRI     0x80   /* 8 颗 × 1B */
#define PH_LED_COUNT       8

/* 颜色字节序：文档标 "BGR?"，先按 B,G,R,0 组包——首灯实测校正点 */
void board_led_raw(uint8_t idx, uint8_t r, uint8_t g, uint8_t b, uint8_t bri)
{
    if (!s_stm32_online || idx >= PH_LED_COUNT) return;

    uint8_t cbuf[5] = { (uint8_t)(PH_REG_LED_COLOR + idx * 4), b, g, r, 0x00 };
    i2c_master_transmit(s_stm32_dev, cbuf, 5, 100);
    uint8_t bbuf[2] = { (uint8_t)(PH_REG_LED_BRI + idx), bri };
    i2c_master_transmit(s_stm32_dev, bbuf, 2, 100);
}

bool board_powerhub_pwr_get(uint16_t *bat_mv, int *pct, uint8_t *chg, uint8_t *ext)
{
    if (!s_stm32_online) return false;
    uint16_t mv = 0;
    if (ph_read_reg16_le(PH_REG_VM_BAT_V, &mv) != ESP_OK) return false;
    *bat_mv = mv;
    *pct = batt_volt_to_percent(mv);
    ph_read_reg(PH_REG_CHG_STATUS, chg);
    ph_read_reg(PH_REG_EXT_POWER, ext);
    return true;
}

uint8_t board_powerhub_usb_mode_get(void)
{
    return s_usb_mode;
}

static void vmusb_monitor_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "vmusb_monitor: 1Hz collect start (backfeed arm: ≤%dmA × %d)",
             BACKFEED_MA_TH, BACKFEED_SAMPLES);

    while (1) {
        uint16_t mv = 0, ma_raw = 0;
        if (ph_read_reg16_le(VMUSB_REG_V, &mv) == ESP_OK &&
            ph_read_reg16_le(VMUSB_REG_I, &ma_raw) == ESP_OK) {
            s_vmusb_mv = mv;
            s_vmusb_ma = (int16_t)ma_raw;
            /* 采集模式：全量打点标定（后续版本收敛为变化上报） */
            ESP_LOGI(TAG, "VM_USB: %u mV, %d mA%s",
                     s_vmusb_mv, s_vmusb_ma,
                     s_backfeed_trip ? "  [TRIPPED]" : "");

            if (!s_backfeed_trip && s_vmusb_ma <= BACKFEED_MA_TH) {
                if (++s_backfeed_streak >= BACKFEED_SAMPLES) {
                    s_backfeed_trip = true;
                    esp_err_t err = ph_write_reg(PH_REG_PC_USB, 0);
                    ESP_LOGE(TAG, "⚠️ BACKFEED TRIP: %d mA ≤ %d mA × %d — PC_USB cut (%s)",
                             s_vmusb_ma, BACKFEED_MA_TH, BACKFEED_SAMPLES,
                             err == ESP_OK ? "ok" : esp_err_to_name(err));
                }
            } else {
                s_backfeed_streak = 0;
            }
        } else {
            ESP_LOGW(TAG, "VM_USB read failed — STM32 I2C?");
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* ================================================================
 *  DTR/RTS 带外控制（UART 兜底路线的 esptool 自动复位）
 *
 *  UART PORT（PORT.C 蓝 Grove）只有 G1/G2/5V/GND 四线，没有流控脚。
 *  DTR/RTS 走 EXT 16P 排针：G4 → 目标 IO0/BOOT，G5 → 目标 EN/RST。
 *  与 sticks3 同款开漏 + idle 释放（防探测期误夹目标板）。
 * ================================================================ */
#define BOARD_DTR_GPIO   4    /* 客户端 DTR → 目标板 IO0/BOOT (低=下载模式) */
#define BOARD_RTS_GPIO   5    /* 客户端 RTS → 目标板 EN/RST (低脉冲复位) */

static bool s_serial_ctrl_inited;

esp_err_t board_serial_ctrl_init(void)
{
    if (s_serial_ctrl_inited) return ESP_OK;

    gpio_config_t cfg = {
        .mode = GPIO_MODE_OUTPUT_OD,
        .pin_bit_mask = (1ULL << BOARD_DTR_GPIO) | (1ULL << BOARD_RTS_GPIO),
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&cfg));

    /* idle 状态: 释放(高阻), 电平由目标板板上拉定义 */
    gpio_set_level(BOARD_DTR_GPIO, 1);
    gpio_set_level(BOARD_RTS_GPIO, 1);

    s_serial_ctrl_inited = true;
    ESP_LOGI(TAG, "DTR/RTS init: DTR=G%d RTS=G%d (EXT 16P pin15/16, idle=release/OD)",
             BOARD_DTR_GPIO, BOARD_RTS_GPIO);
    return ESP_OK;
}

esp_err_t board_serial_ctrl_set(bool dtr_active, bool rts_active)
{
    if (!s_serial_ctrl_inited) {
        ESP_LOGE(TAG, "DTR/RTS not inited");
        return ESP_ERR_INVALID_STATE;
    }
    int dtr_level = dtr_active ? 0 : 1;
    int rts_level = rts_active ? 0 : 1;
    gpio_set_level(BOARD_DTR_GPIO, dtr_level);
    gpio_set_level(BOARD_RTS_GPIO, rts_level);
    return ESP_OK;
}

bool board_serial_ctrl_is_supported(void) { return true; }

esp_err_t board_init(void)
{
    ESP_LOGI(TAG, "board_init (M5Stack PowerHub)");
    ESP_LOGI(TAG, "  console: USB-SERIAL-JTAG (底部 Type-C)");
    ESP_LOGI(TAG, "  flash: 16MB, PSRAM: 8MB Octal");
    ESP_LOGI(TAG, "  coprocessor: STM32G031 @ I2C 0x%02X (SDA=G%d SCL=G%d)",
             PH_STM32_ADDR, PH_I2C_SDA_GPIO, PH_I2C_SCL_GPIO);

    /* STM32 协处理器握手（失败不致命，桥继续跑） */
    board_stm32_init();

    /* 设备态默认收口：关正面 5V + 数据线回底部 C 口。
     * STM32 寄存器跨 ESP 重启持久——上一轮 Host 会话残留的 PC_USB=1 /
     * USB_MODE=1/2 不清掉，会造成 console 无数据（COM 消失）+ 正面口持续带电。
     * Host 路线随后由 board_usb_host_prepare() 重新开启，不受影响。 */
    if (s_stm32_online) {
        ph_write_reg(PH_REG_PC_USB, 0);
        ph_write_reg(PH_REG_USB_MODE, 0);
        ph_write_reg(PH_REG_PC_LED, 1);   /* WS2812 供电开——状态灯是 headless 唯一物理出口 */
        ph_write_reg(PH_REG_PC_UART, 1);  /* UART PORT 5V 常开——PORT 插线即用（同 PORT.A 传感器语义） */
        ESP_LOGI(TAG, "device-mode cleanup: PC_USB=0, USB_MODE=0 (data → bottom C), PC_LED=1, PC_UART=1");
    }

    /* 电池/充电/外部供电状态 */
    board_power_status_log();

    /* USB 拓扑检测 */
    board_detect_usb_topo();

    /* PSRAM 能力检测 */
    board_fill_psram_cap(&s_info);

    /* VM_USB 监测 + 倒灌保护（VAMeter 已在 power_status_log 里开启） */
    xTaskCreate(vmusb_monitor_task, "vmusb_mon", 3072, NULL, 2, NULL);

    /* DTR/RTS（EXT 排针 G4/G5）：UART 兜底路线的 esptool 自动复位 */
    board_serial_ctrl_init();

    ESP_LOGI(TAG, "board_init done");
    return ESP_OK;
}
