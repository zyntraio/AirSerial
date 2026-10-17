/*
 * bridge_web_dns —— 组件内部私有接口（portal.c 调用）。
 * 最小 captive DNS：UDP 53，所有 A 查询应答为热点 IP。
 */
#pragma once

#include "esp_err.h"

esp_err_t bridge_web_dns_start(const char *ap_ip);
esp_err_t bridge_web_dns_stop(void);
