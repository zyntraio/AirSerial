/*
 * bridge_crash —— 崩溃遗言机制（v0.2.4）
 *
 * 背景：v0.2.2 修完三个分层 bug 后，剩下最大的风险是"设备死透时远端
 * 读不到任何带内状态"（panic 重启循环/断网/掉电，任何 HTTP/RFC2217/
 * MCP 全死）。遗言机制在"半死"之外，再覆盖"刚要变死"的最后瞬间：
 * panic handler 里把现场写 RTC 慢速内存（跨复位存活），下次正常
 * 启动时落 NVS + 打日志，供 v0.5.4 HTTP 状态 API 读取。
 *
 * 关键约束：
 *   - panic handler 里中断已禁、SPI flash 操作会死锁——禁止碰 flash/NVS/网络
 *     本模块的 panic-side 代码（esp_reset_reason_set_hint strong override）
 *     只做内存写入，零 flash 操作
 *   - IDF v6.0.3 panic.c 用 weak esp_reset_reason_set_hint() 作为 reboot
 *     前 hook，我们提供 strong override 覆盖它，时机正好
 *   - bridge_core 纯核心，不能依赖板级——这里只做内存/NVS/log，零板级调用
 *
 * 生命周期：
 *   bridge_crash_abort("原因")  ← 主动 abort 时用，先写 reason 到 RTC 再 abort()
 *   esp_reset_reason_set_hint() ← panic.c reboot 分支自动调用（strong override）
 *   bridge_crash_try_load_past()← app_main 早期（NVS init 后）调用，落 NVS + log + 清
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* app_main 早期调用（NVS init 后，WiFi/USB init 前）：
 * 若上次 panic 有遗言，打 WARNING 日志 + 落 NVS（namespace "airsn_crash", key "reason"/"uptime_us"/"abort"）。
 * 无遗言时静默返回。 */
void bridge_crash_try_load_past(void);

/* 主动 abort 时用：先把自定义 reason 写入 RTC 缓冲（供遗言读取），
 * 再调 abort() 进入 panic handler。reason 为空或 NULL 时只 abort。
 * 禁止在 panic handler 内调用（那时内存安全，但 abort 会重入 panic）。 */
void bridge_crash_abort(const char *reason);

#ifdef __cplusplus
}
#endif
