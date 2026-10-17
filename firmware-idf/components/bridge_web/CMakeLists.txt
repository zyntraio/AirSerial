# bridge_web = 网页服务积木（纯核心）：captive portal（DNS 劫持 + httpd 占位页）
idf_component_register(SRCS "bridge_web_portal.c" "bridge_web_dns.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_http_server lwip bridge_wifi bridge_core)

# include 目录里还有组件内部头 bridge_web_dns.h——不对外暴露，
# portal.c 直接相对引用（同目录），无需额外 PUBLIC include
target_include_directories(${COMPONENT_LIB} PRIVATE .)
