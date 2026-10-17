/*
 * bridge_uart.h — UART transport for AirSerial bridge
 *
 * StickS3 default pinout: UART1 TX=GPIO4, RX=GPIO5 (IOMUX direct, no matrix routing)
 * 115200 8N1 by default, configurable via Kconfig or bridge_uart_set_params()
 */
#ifndef BRIDGE_UART_H
#define BRIDGE_UART_H

#include "esp_err.h"
#include "bridge_serial.h"
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t baud_rate;
    uint8_t  data_bits;    /* 5/6/7/8 */
    uint8_t  stop_bits;    /* 1 or 2 */
    uint8_t  parity;       /* 0=none, 1=odd, 2=even */
    int      gpio_tx;      /* -1 = use default */
    int      gpio_rx;      /* -1 = use default */
} bridge_uart_params_t;

/* —— 默认参数（S3 StickS3）—— */
#define BRIDGE_UART_DEFAULT_TX_PIN   4
#define BRIDGE_UART_DEFAULT_RX_PIN   5
#define BRIDGE_UART_DEFAULT_BAUD     921600

/* Init UART driver + RX task. 调用时 transport 会自动替换 bridge_serial 里的当前 transport */
esp_err_t bridge_uart_init(void);

/* Init with custom params（可选），不传则用默认 */
esp_err_t bridge_uart_init_custom(const bridge_uart_params_t *params);

/* 运行时改参数（波特率/数据位等）—— 内部会重新配 UART，不重 init */
esp_err_t bridge_uart_set_params(const bridge_uart_params_t *params);

/* 重开 transport 连接（USB Device 切换回来后再启用等场景）*/
bool bridge_uart_is_ready(void);

uint32_t bridge_uart_get_baud(void);

/* RX 侧诊断：UART 实收字节数 / FIFO 溢出 / 驱动缓冲满 / 帧错 / 校验错 / BREAK（可传 NULL） */
void bridge_uart_rx_debug(size_t *bytes, size_t *fifo_ovf, size_t *buf_full,
                          size_t *frame_err, size_t *parity_err, size_t *brk);

bool bridge_uart_set_baud(uint32_t baud);

void bridge_uart_deinit(void);

/* 返回 transport 指针（和 bridge_usb_host_transport() 完全平行）*/
const bridge_serial_transport_t *bridge_uart_transport(void);

/* 当前参数（只读，用于 portal 展示）*/
const bridge_uart_params_t *bridge_uart_get_params(void);

#ifdef __cplusplus
}
#endif

#endif /* BRIDGE_UART_H */
