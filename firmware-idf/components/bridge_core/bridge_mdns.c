/* bridge_mdns 实现，见头文件注释。 */
#include "bridge_mdns.h"

#include "mdns.h"
#include "esp_log.h"
#include "esp_mac.h"

#include <stdio.h>
#include <string.h>

#define TAG "bridge_mdns"

#define AIRSN_SERVICE_TYPE "_airserial_tcp"  /* 与 proto "_tcp" 组合为 _airserial_tcp._tcp */

static bool s_started = false;
static char s_hostname[20];   /* "airserial-XXXX" = 13 + 结尾，留余量 */
static char s_instance[24];   /* "AirSerial XXXX" */

const char *bridge_mdns_hostname(void)
{
    return s_started ? s_hostname : NULL;
}

esp_err_t bridge_mdns_start(uint16_t raw_port, uint16_t rfc_port)
{
    if (s_started) {
        ESP_LOGI(TAG, "mDNS already started — skip");
        return ESP_OK;
    }
    if (raw_port == 0 || rfc_port == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    /* 主机名取 STA MAC 后两字节（最后 4 个十六进制位），多台设备并存不冲突 */
    uint8_t mac[6] = {0};
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    snprintf(s_hostname, sizeof(s_hostname), "airserial-%02x%02x", mac[4], mac[5]);
    snprintf(s_instance, sizeof(s_instance), "AirSerial %02X%02X", mac[4], mac[5]);

    esp_err_t ret = mdns_init();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "mdns_init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = mdns_hostname_set(s_hostname);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "hostname_set(%s) failed: %s", s_hostname, esp_err_to_name(ret));
        return ret;
    }
    mdns_instance_name_set(s_instance);

    /* 两个端口统一服务类型，实例名和 TXT proto 区分；客户端按类型枚举即可看到全部 */
    mdns_txt_item_t rawTxt[] = {
        { "product", "AirSerial" },
        { "proto", "raw-tcp" },
    };
    ret = mdns_service_add(s_instance, AIRSN_SERVICE_TYPE, "_tcp", raw_port,
                           rawTxt, sizeof(rawTxt) / sizeof(rawTxt[0]));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "raw service_add failed: %s", esp_err_to_name(ret));
        return ret;
    }

    char rfc_instance[sizeof(s_instance) + 8];
    snprintf(rfc_instance, sizeof(rfc_instance), "%s RFC2217", s_instance);
    mdns_txt_item_t rfcTxt[] = {
        { "product", "AirSerial" },
        { "proto", "rfc2217" },
    };
    ret = mdns_service_add(rfc_instance, AIRSN_SERVICE_TYPE, "_tcp", rfc_port,
                           rfcTxt, sizeof(rfcTxt) / sizeof(rfcTxt[0]));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "rfc2217 service_add failed: %s", esp_err_to_name(ret));
        return ret;
    }

    s_started = true;
    ESP_LOGI(TAG, "mDNS up: %s.local, %s._tcp raw=%u/rfc2217=%u ('%s')",
             s_hostname, AIRSN_SERVICE_TYPE, raw_port, rfc_port, s_instance);
    return ESP_OK;
}
