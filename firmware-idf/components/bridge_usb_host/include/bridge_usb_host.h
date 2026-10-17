/*
 * bridge_usb_host — USB Host CDC-ACM 串口桥接（v0.1.0）
 *
 * 封装 espressif/usb + usb_host_cdc_acm managed components，
 * 提供 bridge_serial_transport_t 给 bridge_core 使用。
 *
 * 调用流程：
 *   1. bridge_usb_host_init()  — 装 USB Host Library + CDC-ACM driver
 *   2. bridge_usb_host_open()  — 阻塞等到 CDC-ACM 设备插入（VID/PID 任意）
 *   3. bridge_usb_host_transport() — 拿到 bridge_serial 用的 transport
 *   4. bridge_usb_host_deinit() — 关闭设备 + 卸载 driver（Host → Device 切换时）
 */
#pragma once

#include "esp_err.h"
#include "bridge_serial.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t bridge_usb_host_init(void);
esp_err_t bridge_usb_host_open(uint32_t timeout_ms);
esp_err_t bridge_usb_host_close(void);
void      bridge_usb_host_deinit(void);
bool      bridge_usb_host_is_open(void);

const bridge_serial_transport_t *bridge_usb_host_transport(void);

/* 注册设备拔出回调：CDC 设备断开时调用（主程序据此把 transport 切回 UART 兜底） */
typedef void (*bridge_usb_host_disconnect_cb_t)(void *arg);
void bridge_usb_host_set_disconnect_callback(bridge_usb_host_disconnect_cb_t cb, void *arg);

/* RFC2217 带外控制转发（esptool 自动复位：RTS→EN / DTR→GPIO0；SET_BAUDRATE 透传） */
esp_err_t bridge_usb_host_set_control(bool dtr, bool rts);
esp_err_t bridge_usb_host_set_baud(uint32_t baud);

/* 原始 vendor 控制传输（调试/适配用）：可发任意 bmRequestType/bRequest/wValue/wIndex，
 * in_len>0 时做 IN 读并把字节填回 in_buf。仅在 USB Host CDC 设备打开时可用。*/
esp_err_t bridge_usb_host_raw_ctrl(uint8_t bm_req_type, uint8_t b_req,
                                   uint16_t w_value, uint16_t w_index,
                                   uint8_t *in_buf, size_t in_len);

/* 分层诊断：CDC 通道实收/实发字节数与 TX 失败次数（USB Host 路线对应 UART 路线的 uart_rx 计数） */
void bridge_usb_host_debug(size_t *cdc_rx, size_t *cdc_tx, size_t *cdc_tx_fail);

/* 克隆 CH340 检测（open 时读 0x5F 版本寄存器）：
 * version <0 = 非 WCH/读取失败；clone 仅在版本 0x31（已知克隆，RX 尾包楔形卡死）时置真 */
int  bridge_usb_host_ch340_version(void);
bool bridge_usb_host_ch340_is_clone(void);
bool bridge_usb_host_is_wch(void);

#ifdef __cplusplus
}
#endif
