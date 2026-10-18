# AirSerial — M5Stack StickS3 WiFi serial bridge (ESP-IDF v6, S3 built-in WiFi)
cmake_minimum_required(VERSION 3.16)

# 版本号单一出处：app_version.h 的 AIRSERIAL_VERSION。
# 这里提取出来喂给 PROJECT_VER（boot 日志 App version / esp_app_desc），
# 避免 PROJECT_VER 落到 git describe 与固件宏分裂（曾显示 v0.3.2-alpha-dirty）。
file(READ "${CMAKE_SOURCE_DIR}/components/bridge_core/include/app_version.h" _ver_h)
string(REGEX MATCH "AIRSERIAL_VERSION[ \t]+\"([^\"]+)\"" _ "${_ver_h}")
if(CMAKE_MATCH_1)
    set(PROJECT_VER "${CMAKE_MATCH_1}")
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(airserial)
