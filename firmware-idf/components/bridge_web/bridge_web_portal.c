/*
 * bridge_web_portal —— AirSerial 通用设置页 HTTP 服务（v0.3.2）。
 *
 * esp_http_server 单实例：端口 80，URL 空间约定：
 *   /           → 通用设置页（SPA，单 HTML 文件）
 *   /api/status → 聚合状态 + features 数组（功能隐藏驱动）
 *   /api/wifi/... -> WiFi 配网 API (scan/saved/connect/forget)
 *   /ws         → WebSocket 串口终端（v0.4.1 注册）
 *
 * 探针路径（各 OS 联网检测 URL）统一 302 回首页。
 * 未实现的功能 = handler 不注册 = /api/status 的 features 里没有 = 前端不显示 tab。
 */
#include "bridge_web.h"
#include "bridge_web_dns.h"
#include "bridge_wifi.h"
#include "bridge_serial.h"
#include "bridge_rfc2217.h"
#include "app_version.h"

#include "esp_log.h"
#include "esp_http_server.h"
#include "esp_system.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <string.h>
#include <stdio.h>
#include <inttypes.h>

#define TAG "bridge_web"

/* ---- 运行状态 ---- */
static httpd_handle_t s_httpd = NULL;
static volatile bool  s_portal_running;
static char           s_ap_ssid[33] = {0};

/* ---- WebSocket 终端 ---- */
#define WS_TASK_STACK   4096
#define WS_TASK_PRIO    4
#define WS_RX_BUF_SIZE  256

static volatile int  s_ws_client_count;
static int           s_ws_client_fd = -1;  /* 当前 WS 客户端 socket fd（单客户端简化）*/
static bridge_serial_subscriber_t *s_ws_sub;
static TaskHandle_t  s_ws_task_handle;
static volatile bool s_ws_task_running;
static char           s_ap_ip[16] = "192.168.4.1";

/* — 前向声明 — */
static esp_err_t ws_term_handler(httpd_req_t *req);
static esp_err_t ws_post_handshake_cb(httpd_req_t *req);
static void ws_forward_task(void *arg);

/* 探针路径表（OS 联网检测 URL），命中即 302 回首页 */
static const char *s_probe_paths[] = {
    "/generate_204",
    "/hotspot-detect.html",
    "/library/test/success.html",
    "/mobile/success.html",
    "/connecttest.txt",
    "/redirect",
    "/ncsi.txt",
    "/fwlink",
    "/canonical.html",
    "/success.txt",
};

/* ================================================================
 *  前端：通用设置页 SPA（单 HTML 文件，纯 CSS+JS，零外部依赖）
 *  侧栏 tab 导航，每个 tab 对应一个 section；
 *  section 可见性由 /api/status 返回的 features 数组驱动。
 *  当前版本（v0.3.2）只有 "wifi" tab，Serial/System/Tunnel/AI tab
 *  的 HTML 骨架已预留但 CSS 隐藏，等对应 API 落地后自动显示。
 * ================================================================ */
static const char s_html[] =
"<!DOCTYPE html><html><head><meta charset='utf-8'>"
"<meta name='viewport' content='width=device-width,initial-scale=1'>"
"<title>AirSerial</title>"
"<style>"
"*{box-sizing:border-box;margin:0;padding:0}"
":root{--bg:#0d0d14;--bg-2:#16161f;--bg-3:#1e1e2a;--fg:#e8e8ef;--fg-dim:#88889a;--accent:#8BE9FD;--danger:#FF5555;--ok:#50FA7B;--border:#2a2a38;--radius:8px}"
"html,body{height:100%;background:var(--bg);color:var(--fg);font-family:-apple-system,BlinkMacSystemFont,'PingFang SC','Microsoft YaHei','Segoe UI',Roboto,sans-serif;font-size:14px}"
"a{color:var(--accent);text-decoration:none}"
"button{cursor:pointer;font-family:inherit}"
".app{display:flex;height:100vh;overflow:hidden}"
/* --- Sidebar --- */
".sidebar{width:220px;background:var(--bg-2);border-right:1px solid var(--border);display:flex;flex-direction:column;flex-shrink:0}"
".brand{padding:20px 16px;border-bottom:1px solid var(--border)}"
".brand h1{font-size:18px;color:var(--accent);font-weight:600}"
".brand .ver{font-size:11px;color:var(--fg-dim);margin-top:2px}"
".tabs{flex:1;padding:8px 0;overflow-y:auto}"
".tab{display:flex;align-items:center;gap:10px;padding:10px 16px;color:var(--fg-dim);cursor:pointer;border-left:3px solid transparent;transition:all .15s}"
".tab:hover{background:var(--bg-3);color:var(--fg)}"
".tab.active{background:var(--bg-3);color:var(--accent);border-left-color:var(--accent)}"
".tab.hidden{display:none}"
".tab .icon{width:18px;text-align:center}"
".sidebar-footer{padding:12px 16px;border-top:1px solid var(--border);font-size:11px;color:var(--fg-dim)}"
/* --- Main --- */
".main{flex:1;display:flex;flex-direction:column;overflow:hidden}"
".topbar{height:48px;background:var(--bg-2);border-bottom:1px solid var(--border);display:flex;align-items:center;padding:0 20px;gap:16px;font-size:13px}"
".topbar .spacer{flex:1}"
".topbar .status-dot{display:inline-block;width:8px;height:8px;border-radius:50%;margin-right:6px;vertical-align:middle}"
".topbar .status-dot.on{background:var(--ok)}"
".topbar .status-dot.off{background:var(--danger)}"
".content{flex:1;overflow-y:auto;padding:24px}"
".section{display:none}"
".section.active{display:block}"
".section h2{font-size:16px;margin-bottom:16px;color:var(--fg)}"
/* --- WiFi page --- */
".wifi-scan{display:flex;gap:10px;margin-bottom:16px}"
".wifi-scan button{padding:8px 18px;background:var(--bg-3);color:var(--fg);border:1px solid var(--border);border-radius:var(--radius);font-size:13px}"
".wifi-scan button:hover{border-color:var(--accent)}"
".wifi-scan button.primary{background:var(--accent);color:var(--bg);border-color:var(--accent)}"
".wifi-list{display:flex;flex-direction:column;gap:4px}"
".wifi-item{display:flex;align-items:center;padding:12px 16px;background:var(--bg-2);border:1px solid var(--border);border-radius:var(--radius);cursor:pointer;transition:all .15s}"
".wifi-item:hover{border-color:var(--accent)}"
".wifi-item .ssid{flex:1;font-weight:500}"
".wifi-item .rssi{color:var(--fg-dim);font-size:12px;margin:0 12px}"
".wifi-item .lock{color:var(--fg-dim);width:20px;text-align:center}"
".wifi-item .saved{color:var(--accent);margin-left:8px}"
".wifi-item.active-connect{border-color:var(--accent);background:rgba(139,233,253,.08)}"
".saved-title{font-size:13px;color:var(--fg-dim);margin:24px 0 8px;padding-bottom:6px;border-bottom:1px solid var(--border)}"
/* --- Connect dialog --- */
".modal{display:none;position:fixed;inset:0;background:rgba(0,0,0,.6);align-items:center;justify-content:center;z-index:100}"
".modal.active{display:flex}"
".modal-content{background:var(--bg-2);border:1px solid var(--border);border-radius:var(--radius);padding:24px;width:340px}"
".modal-content h3{font-size:15px;margin-bottom:16px}"
".form-row{margin-bottom:14px}"
".form-row label{display:block;font-size:12px;color:var(--fg-dim);margin-bottom:6px}"
".form-row input{width:100%;padding:9px 12px;background:var(--bg);border:1px solid var(--border);border-radius:6px;color:var(--fg);font-size:14px;font-family:inherit}"
".form-row input:focus{outline:none;border-color:var(--accent)}"
".modal-actions{display:flex;gap:10px;justify-content:flex-end;margin-top:18px}"
".modal-actions button{padding:8px 18px;border-radius:var(--radius);font-size:13px;border:1px solid var(--border);background:var(--bg-3);color:var(--fg)}"
".modal-actions button.primary{background:var(--accent);color:var(--bg);border-color:var(--accent)}"
".modal-actions button.danger{background:transparent;color:var(--danger);border-color:var(--danger)}"
"/* --- Generic empty placeholder for future tabs --- */"
".placeholder{text-align:center;padding:60px 20px;color:var(--fg-dim)}"
".placeholder .big-icon{font-size:48px;margin-bottom:16px;opacity:.4}"
".placeholder p{line-height:1.6}"
"/* --- Clone CH340 warning banner --- */"
".warn-banner{display:none;margin-bottom:16px;padding:12px 16px;border:1px solid var(--danger);border-radius:var(--radius);color:var(--danger);font-size:13px;line-height:1.6;background:rgba(255,85,85,.07)}"
/* --- Mobile --- */
".menu-btn{display:none;width:32px;height:32px;background:var(--bg-3);border:1px solid var(--border);border-radius:6px;color:var(--fg);align-items:center;justify-content:center;margin-right:10px;font-size:18px}"
".overlay{position:fixed;inset:0;background:rgba(0,0,0,.5);z-index:150;display:none}"
".overlay.open{display:block}"
"@media(max-width:768px){"
".sidebar{position:fixed;left:-220px;top:0;bottom:0;z-index:200;transition:left .2s}"
".sidebar.open{left:0}"
".menu-btn{display:flex}"
".topbar .title{display:none}"
".wifi-item{flex-wrap:wrap}"
".content{padding:16px}"
"}"
"</style></head><body>"
"<div class='app'>"
"<aside class='sidebar' id='sidebar'>"
"<div class='brand'><h1>AirSerial</h1><div class='ver' id='ver'>--</div></div>"
"<nav class='tabs'>"
"<div class='tab active' data-tab='wifi'><span class='icon'>📶</span>WiFi</div>"
"<div class='tab' data-tab='serial'><span class='icon'>🔌</span>Serial</div>"
"<div class='tab hidden' data-tab='system'><span class='icon'>⚙️</span>System</div>"
"<div class='tab hidden' data-tab='tunnel'><span class='icon'>🔒</span>Tunnel</div>"
"<div class='tab hidden' data-tab='ai'><span class='icon'>🤖</span>AI</div>"
"</nav>"
"<div class='sidebar-footer'><span id='ap-info'>AP: AirSerial-XXXX</span></div>"
"</aside>"
"<div class='overlay' id='overlay'></div>"
"<div class='main'>"
"<div class='topbar'>"
"<button class='menu-btn' id='menuBtn'>☰</button>"
"<div><span class='status-dot off' id='wifiDot'></span><span id='wifiLabel'>Offline</span></div>"
"<div class='spacer'></div>"
"<div id='uptimeLabel'>Uptime: <span id='uptime'>--</span></div>"
"<button id='langBtn' style='margin-left:16px;padding:4px 10px;background:var(--bg-3);color:var(--fg);border:1px solid var(--border);border-radius:6px;font-size:12px;cursor:pointer'>中</button>"
"</div>"
"<div class='content'>"
"<div class='warn-banner' id='cloneBanner'></div>"
"<!-- WiFi -->"
"<div class='section active' data-section='wifi'>"
"<h2 data-i18n='wifi'>WiFi</h2>"
"<!-- Manual connect form (visible first, scan optional) -->"
"<div class='form-row' style='margin-bottom:12px'><label style='font-size:12px;color:var(--fg-dim);margin-bottom:4px;display:block' data-i18n='ssid'>SSID</label><input id='manualSsid' placeholder='Network name' style='width:100%;padding:9px 12px;background:var(--bg);border:1px solid var(--border);border-radius:6px;color:var(--fg);font-size:14px'></div>"
"<div class='form-row' style='margin-bottom:12px'><label style='font-size:12px;color:var(--fg-dim);margin-bottom:4px;display:block' data-i18n='password'>Password</label><input id='manualPw' type='password' placeholder='Leave blank for open networks' style='width:100%;padding:9px 12px;background:var(--bg);border:1px solid var(--border);border-radius:6px;color:var(--fg);font-size:14px'></div>"
"<div style='display:flex;gap:10px;margin-bottom:16px'>"
"<button id='manualConnectBtn' class='primary' style='flex:1;padding:8px 18px;background:var(--accent);color:var(--bg);border:1px solid var(--accent);border-radius:var(--radius);font-size:13px' data-i18n='connect'>Connect</button>"
"<button id='scanBtn' style='padding:8px 18px;background:var(--bg-3);color:var(--fg);border:1px solid var(--border);border-radius:var(--radius);font-size:13px' data-i18n='scan'>Scan</button>"
"</div>"
"<div class='wifi-list' id='wifiList'></div>"
"<div class='saved-title' id='savedTitleEl'>Saved Networks</div>"
"<div class='wifi-list' id='savedList'></div>"
"</div>"
"<!-- Serial Web Terminal -->"
"<div class='section' data-section='serial'>"
"<h2>Web Terminal</h2>"
"<!-- USB Host mode switch -->"
"<div style='display:flex;gap:10px;margin-bottom:14px;padding:12px 16px;background:var(--bg-2);border:1px solid var(--border);border-radius:var(--radius);align-items:center'>"
"<span id='uhDot' style='display:inline-block;width:10px;height:10px;border-radius:50%;background:var(--fg-dim);flex-shrink:0'></span>"
"<span id='uhMode' style='flex:1;font-size:13px;color:var(--fg-dim)'>USB mode</span>"
"<button id='uhEnable' style='padding:6px 14px;background:var(--bg-3);color:var(--fg);border:1px solid var(--border);border-radius:6px;font-size:12px;cursor:pointer'>Enable USB Host</button>"
"<button id='uhDisable' style='padding:6px 14px;background:var(--bg-3);color:var(--danger);border:1px solid var(--danger);border-radius:6px;font-size:12px;cursor:pointer;display:none'>Disable USB Host</button>"
"</div>"
"<div style='display:flex;gap:8px;margin-bottom:10px;align-items:center'>"
"<button id='wsConnect' style='padding:6px 14px;background:var(--accent);color:var(--bg);border:1px solid var(--accent);border-radius:6px;font-size:12px;cursor:pointer'>Connect</button>"
"<button id='wsDisconnect' style='padding:6px 14px;background:var(--bg-3);color:var(--fg);border:1px solid var(--border);border-radius:6px;font-size:12px;cursor:pointer;display:none'>Disconnect</button>"
"<span id='wsStatus' style='font-size:12px;color:var(--fg-dim)'>disconnected</span>"
"<span style='flex:1'></span>"
"<button id='termClear' style='padding:6px 12px;background:var(--bg-3);color:var(--fg-dim);border:1px solid var(--border);border-radius:6px;font-size:11px;cursor:pointer'>Clear</button>"
"</div>"
"<div id='termBox' style='background:#000;color:#0f0;font-family:Menlo,Monaco,Consolas,monospace;font-size:13px;line-height:1.5;padding:12px;border-radius:6px;border:1px solid var(--border);height:420px;overflow-y:auto;white-space:pre-wrap;word-break:break-all;caret-color:#0f0'></div>"
"<div style='display:flex;gap:8px;margin-top:8px'>"
"<textarea id='termInput' rows='2' placeholder='Type here, press Enter to send...' style='flex:1;padding:8px;background:var(--bg-2);color:var(--fg);border:1px solid var(--border);border-radius:6px;font-family:Menlo,Monaco,Consolas,monospace;font-size:13px;resize:none'></textarea>"
"<button id='termSend' style='padding:8px 18px;background:var(--accent);color:var(--bg);border:1px solid var(--accent);border-radius:6px;font-size:13px;cursor:pointer;align-self:flex-end'>Send</button>"
"</div>"
"<p style='font-size:11px;color:var(--fg-dim);margin-top:8px'>Tip: Press Ctrl+C / Ctrl+D / Ctrl+Z in the input box to send control characters. Ctrl+Enter sends raw newline.</p>"
"</div>"
"<!-- System (placeholder v0.5) -->"
"<div class='section' data-section='system'>"
"<h2>System</h2>"
"<div class='placeholder'><div class='big-icon'>⚙️</div><p>System settings coming soon — restart, OTA update, crash log export.</p></div>"
"</div>"
"<!-- Tunnel (placeholder v0.9) -->"
"<div class='section' data-section='tunnel'>"
"<h2>Remote Tunnel</h2>"
"<div class='placeholder'><div class='big-icon'>🔒</div><p>Remote access tunnel (VPN-like) coming in v0.9.</p></div>"
"</div>"
"<!-- AI (placeholder v3.x) -->"
"<div class='section' data-section='ai'>"
"<h2>AI Integration</h2>"
"<div class='placeholder'><div class='big-icon'>🤖</div><p>AI-powered features planned for future versions.</p></div>"
"</div>"
"</div></div></div>"
"<!-- Connect modal -->"
"<div class='modal' id='connectModal'>"
"<div class='modal-content'>"
"<h3 id='modalTitle'>Connect to WiFi</h3>"
"<div class='form-row'><label>SSID</label><input id='modalSsid' readonly></div>"
"<div class='form-row' id='pwRow'><label>Password</label><input id='modalPw' type='password' placeholder='Leave blank for open networks'></div>"
"<div class='modal-actions'>"
"<button id='modalCancel'>Cancel</button>"
"<button id='modalConnect' class='primary'>Connect</button>"
"</div></div></div>"
"<!-- Saved actions modal -->"
"<div class='modal' id='savedModal'>"
"<div class='modal-content'>"
"<h3 id='savedTitle'>Saved Network</h3>"
"<p style='color:var(--fg-dim);font-size:13px;margin-bottom:16px' id='savedInfo'></p>"
"<div class='modal-actions'>"
"<button id='savedForget' class='danger'>Forget</button>"
"<button id='savedClose'>Close</button>"
"</div></div></div>"
"<script>"
"const tabs=document.querySelectorAll(\".tab\"), secs=document.querySelectorAll(\".section\");"
"let features=new Set();"
"let currentAp=null;"
"let savedAp=null;"
"let lang=\"en\";"
"const T={"
"en:{wifi:\"WiFi\",serial:\"Serial\",system:\"System\",tunnel:\"Tunnel\",ai:\"AI\","
"scan:\"Scan\",scanning:\"Scanning...\",saved:\"Saved Networks\",connect:\"Connect to WiFi\","
"connecting:\"Connecting...\",connected:\"Connected\",offline:\"Offline\","
"hotspot:\"Hotspot: \",ap:\"AP: \",inactive:\"inactive\","
"ssid:\"SSID\",password:\"Password\","
"uptime:\"Uptime: \",days:\"d\",hours:\"h\",mins:\"m\",secs:\"s\","
"modal_cancel:\"Cancel\",modal_connect:\"Connect\","
"saved_title:\"Saved Network\",saved_password:\"Password: \",saved_open:\"Open network\",saved_hidden:\"Saved (hidden)\","
"saved_forget:\"Forget\",saved_close:\"Close\","
"serial_title:\"Serial Bridge\",serial_placeholder:\"Serial port settings coming soon.\","
"system_title:\"System\",system_placeholder:\"System settings coming soon.\","
"tunnel_title:\"Remote Tunnel\",tunnel_placeholder:\"Remote access tunnel v0.9.\","
"ai_title:\"AI Integration\",ai_placeholder:\"AI planned.\","
"err_connect:\"Connection failed: \",err_unknown:\"unknown\",scan_none:\"No networks found, retry?\","
"clone_warn:\"⚠ Clone CH340 detected (version %V). This counterfeit chip has a hardware defect: bulk transfers stall, so esptool read/write will likely fail. This is a fault of the chip itself, not an AirSerial compatibility issue. Use a genuine CH340C/CH343, CP2102, or a native USB-Serial-JTAG target board.\""
"},"
"zh:{wifi:\"WiFi\",serial:\"串口\",system:\"系统\",tunnel:\"远程隧道\",ai:\"AI\","
"scan:\"扫描\",scanning:\"扫描中...\",saved:\"已保存网络\",connect:\"连接\","
"connecting:\"连接中...\",connected:\"已连接\",offline:\"离线\","
"hotspot:\"热点: \",ap:\"AP: \",inactive:\"未启用\","
"ssid:\"WiFi 名称\",password:\"密码\","
"uptime:\"运行时间: \",days:\"天\",hours:\"时\",mins:\"分\",secs:\"秒\","
"modal_cancel:\"取消\",modal_connect:\"连接\","
"saved_title:\"已保存的网络\",saved_password:\"密码: \",saved_open:\"开放网络\",saved_hidden:\"已保存\","
"saved_forget:\"忘记\",saved_close:\"关闭\","
"serial_title:\"串口桥\",serial_placeholder:\"串口参数设置即将上线。\","
"system_title:\"系统\",system_placeholder:\"系统设置即将上线。\","
"tunnel_title:\"远程隧道\",tunnel_placeholder:\"远程隧道 v0.9 上线。\","
"ai_title:\"AI 集成\",ai_placeholder:\"AI 功能规划中。\","
"err_connect:\"连接失败: \",err_unknown:\"未知\",scan_none:\"没扫到网络,重试?\","
"clone_warn:\"⚠ 检测到克隆 CH340（版本 %V）。该仿制芯片存在硬件缺陷：批量传输会卡死，esptool 读写大概率失败。这是芯片本身的问题，并非 AirSerial 兼容性不足。建议更换正品 CH340C/CH343、CP2102，或原生 USB-Serial-JTAG 目标板。\"}"
"};"
"function L(k){return T[lang][k]||T.en[k]||k;}"
"function fmtUptime(s){if(s==null||s==undefined)return\"--\";const d=Math.floor(s/86400),h=Math.floor((s%86400)/3600),m=Math.floor((s%3600)/60),sec=s%60;let r=\"\";if(d)r+=d+L(\"days\");if(h)r+=h+L(\"hours\");r+=m+L(\"mins\")+sec+L(\"secs\");return r;}"
"function applyLang(){document.querySelectorAll(\"[data-i18n]\").forEach(el=>{const k=el.dataset.i18n;if(T[lang][k])el.textContent=T[lang][k]});document.querySelectorAll(\".tab\").forEach(t=>{const icon=t.firstChild.textContent;const map={wifi:\"wifi\",serial:\"serial\",system:\"system\",tunnel:\"tunnel\",ai:\"ai\"};t.textContent=L(map[t.dataset.tab]||t.dataset.tab);t.insertAdjacentHTML(\"afterbegin\",icon)});document.querySelectorAll(\".section\").forEach(s=>{const t=s.dataset.section;const h=s.querySelector(\"h2\");if(h)h.textContent=L(t)});const savedEl=document.getElementById(\"savedTitleEl\");if(savedEl)savedEl.textContent=L(\"saved\");const mt=document.getElementById(\"modalTitle\");if(mt)mt.textContent=L(\"connect\");document.getElementById(\"modalCancel\").textContent=L(\"modal_cancel\");document.getElementById(\"modalConnect\").textContent=L(\"modal_connect\");const st=document.getElementById(\"savedTitle\");if(st)st.textContent=L(\"saved_title\");document.getElementById(\"savedForget\").textContent=L(\"saved_forget\");document.getElementById(\"savedClose\").textContent=L(\"saved_close\");document.getElementById(\"scanBtn\").textContent=L(\"scan\");document.getElementById(\"langBtn\").textContent=lang==\"en\"?\"中\":\"EN\";const ul=document.getElementById(\"uptimeLabel\");if(ul.firstChild)ul.firstChild.textContent=L(\"uptime\");}"
"async function apiGet(p){try{const r=await fetch(p);return r.json();}catch(e){return null;}}"
"async function apiPost(p,b){try{const r=await fetch(p,{method:\"POST\",headers:{\"Content-Type\":\"application/json\"},body:JSON.stringify(b)});return r.json();}catch(e){return null;}}"
"async function apiDel(p,b){try{const r=await fetch(p,{method:\"DELETE\",headers:{\"Content-Type\":\"application/json\"},body:JSON.stringify(b)});return r.json();}catch(e){return null;}}"
"function showSection(n){tabs.forEach(t=>{t.classList.toggle(\"active\",t.dataset.tab===n)});secs.forEach(s=>{s.classList.toggle(\"active\",s.dataset.section===n)})}"
"function renderWifi(list,el,isOpen){el.innerHTML=\"\";if(!list||!list.length){const empty=document.createElement(\"div\");empty.style.cssText=\"text-align:center;color:var(--fg-dim);padding:20px;font-size:12px\";empty.textContent=L(\"scan_none\");el.appendChild(empty);return}list.forEach(a=>{const d=document.createElement(\"div\");d.className=\"wifi-item\";const ssid=a.ssid||\"(hidden)\";const lock=a.open?\"\":\"🔒\";d.innerHTML=`<span class=\\\"ssid\\\">${ssid}</span><span class=\\\"lock\\\">${lock}</span><span class=\\\"rssi\\\">${a.rssi}dBm</span>`;d.onclick=()=>pickSsid(a);el.appendChild(d)})}"
"async function doScan(){const btn=document.getElementById(\"scanBtn\");btn.disabled=true;btn.textContent=L(\"scanning\");await apiPost(\"/api/wifi/scan\",{});let retries=0,aps=[],scanning=true;while(retries<10&&scanning){await new Promise(r=>setTimeout(r,400));const r=await apiGet(\"/api/wifi/scan\");if(r){scanning=r.scanning;if(r.aps&&r.aps.length)aps=r.aps}retries++}btn.disabled=false;btn.textContent=L(\"scan\");renderWifi(aps,document.getElementById(\"wifiList\"))}"
"async function loadSaved(){const r=await apiGet(\"/api/wifi/saved\");if(r&&r.creds){r.creds.forEach(c=>c.rssi=c.open?-50:-60);renderWifi(r.creds,document.getElementById(\"savedList\"))}}"
"function pickSsid(ap){const ssidEl=document.getElementById(\"manualSsid\");const pwEl=document.getElementById(\"manualPw\");ssidEl.value=ap.ssid;pwEl.value=\"\";if(ap.open){pwEl.placeholder=\"Open network — password blank\";document.getElementById(\"manualConnectBtn\").focus()}else{pwEl.placeholder=\"Password for \"+ap.ssid;pwEl.focus();pwEl.select()}}"
"async function doConnect(ssid,pw){const btn=document.getElementById(\"manualConnectBtn\");btn.disabled=true;btn.textContent=L(\"connecting\");const r=await apiPost(\"/api/wifi/connect\",{ssid:ssid,password:pw});btn.disabled=false;btn.textContent=L(\"connect\");if(r&&r.ok){updateStatus();loadSaved()}else alert(L(\"err_connect\")+(r&&r.error?r.error:L(\"err_unknown\")))}"
"async function updateUsbStatus(){const u=await apiGet(\"/api/usb/host_status\");if(!u)return;const dot=document.getElementById(\"uhDot\");if(dot){const mode=document.getElementById(\"uhMode\");const en=document.getElementById(\"uhEnable\");const dis=document.getElementById(\"uhDisable\");if(u.host_enabled){dot.style.background=\"var(--danger)\";mode.textContent=\"Host mode (risky — monitor will disconnect)\";en.style.display=\"none\";dis.style.display=\"inline-block\"}else{dot.style.background=\"var(--ok)\";mode.textContent=\"Device mode (safe — monitor available)\";en.style.display=\"inline-block\";dis.style.display=\"none\"}}const d=await apiGet(\"/api/usb/debug\");const b=document.getElementById(\"cloneBanner\");if(d&&d.ch340_clone){b.textContent=L(\"clone_warn\").replace(\"%V\",d.ch340_ver||\"?\");b.style.display=\"block\"}else{b.style.display=\"none\"}}"
"async function enableUsbHost(){if(!confirm(\"Enable USB Host?\\n\\nThis will reboot the device and the monitor will disconnect.\\nThe serial console is lost until Host mode is disabled.\"))return;const r=await apiPost(\"/api/usb/host_on\",{});if(r&&r.ok){document.getElementById(\"uhEnable\").disabled=true;document.getElementById(\"uhEnable\").textContent=\"Rebooting...\"}else alert(\"Failed: \"+(r&&r.error?r.error:\"unknown\"))}"
"async function disableUsbHost(){const r=await apiPost(\"/api/usb/host_off\",{});if(r&&r.ok){document.getElementById(\"uhDisable\").disabled=true;document.getElementById(\"uhDisable\").textContent=\"Rebooting...\"}else alert(\"Failed: \"+(r&&r.error?r.error:\"unknown\"))}"
"async function updateStatus(){const s=await apiGet(\"/api/status\");if(!s)return;features=new Set(s.features||[]);tabs.forEach(t=>{const tab=t.dataset.tab;const map={wifi:\"wifi_connect\",serial:\"serial_config\",system:\"system_restart\",tunnel:\"tunnel\",ai:\"ai\"};t.classList.toggle(\"hidden\",!features.has(map[tab]||tab))});const dot=document.getElementById(\"wifiDot\");dot.classList.toggle(\"on\",s.sta_connected);dot.classList.toggle(\"off\",!s.sta_connected);document.getElementById(\"wifiLabel\").textContent=s.sta_connected?(s.sta_ssid||L(\"connected\")):(s.ap_active?(L(\"hotspot\")+s.ap_ssid):L(\"offline\"));document.getElementById(\"uptime\").textContent=fmtUptime(s.uptime);document.getElementById(\"ver\").textContent=s.version;document.getElementById(\"ap-info\").textContent=L(\"ap\")+(s.ap_active?(s.ap_ssid+\" (\"+s.ap_ip+\")\"):L(\"inactive\"))}"
"tabs.forEach(t=>{t.onclick=()=>{showSection(t.dataset.tab);if(t.dataset.tab===\"serial\")updateUsbStatus()}});"
"document.getElementById(\"scanBtn\").onclick=doScan;"
"document.getElementById(\"manualConnectBtn\").onclick=()=>{const ssid=document.getElementById(\"manualSsid\").value.trim();const pw=document.getElementById(\"manualPw\").value.trim();if(!ssid)return;doConnect(ssid,pw)};"
"document.getElementById(\"modalCancel\").onclick=()=>document.getElementById(\"connectModal\").classList.remove(\"active\");"
"document.getElementById(\"modalConnect\").onclick=()=>doConnect(currentAp.ssid,document.getElementById(\"modalPw\").value.trim());"
"document.getElementById(\"savedForget\").onclick=()=>forgetSaved();"
"document.getElementById(\"savedClose\").onclick=()=>document.getElementById(\"savedModal\").classList.remove(\"active\");"
"document.getElementById(\"menuBtn\").onclick=()=>{document.getElementById(\"sidebar\").classList.toggle(\"open\");document.getElementById(\"overlay\").classList.toggle(\"open\")};"
"document.getElementById(\"overlay\").onclick=()=>{document.getElementById(\"sidebar\").classList.remove(\"open\");document.getElementById(\"overlay\").classList.remove(\"open\")};"
"document.getElementById(\"langBtn\").onclick=()=>{lang=lang==\"en\"?\"zh\":\"en\";applyLang();updateStatus();};"
"const uhEn=document.getElementById(\"uhEnable\");if(uhEn)uhEn.onclick=enableUsbHost;"
"const uhDis=document.getElementById(\"uhDisable\");if(uhDis)uhDis.onclick=disableUsbHost;"
"applyLang();updateStatus();updateUsbStatus();loadSaved();setInterval(()=>{updateStatus();updateUsbStatus()},5000);"
"/* — WebSocket Terminal — */"
"let ws=null;"
"const termBox=document.getElementById('termBox');"
"const termInput=document.getElementById('termInput');"
"const wsStatus=document.getElementById('wsStatus');"
"function wsConnect(){if(ws){ws.close();ws=null}const proto=location.protocol==='https:'?'wss:':'ws:';"
"ws=new WebSocket(proto+'//'+location.host+'/ws');"
"ws.binaryType='arraybuffer';"
"wsStatus.textContent='connecting...';"
"wsStatus.style.color='var(--fg-dim)';"
"ws.onopen=()=>{"
"wsStatus.textContent='connected';"
"wsStatus.style.color='var(--ok)';"
"document.getElementById('wsConnect').style.display='none';"
"document.getElementById('wsDisconnect').style.display='inline-block';"
"termBox.textContent+='\\r\\n[AirSerial] WebSocket terminal connected\\r\\n';"
"termBox.scrollTop=termBox.scrollHeight;"
"};"
"ws.onclose=()=>{"
"wsStatus.textContent='disconnected';"
"wsStatus.style.color='var(--fg-dim)';"
"document.getElementById('wsConnect').style.display='inline-block';"
"document.getElementById('wsDisconnect').style.display='none';"
"termBox.textContent+='\\r\\n[AirSerial] WebSocket disconnected\\r\\n';"
"termBox.scrollTop=termBox.scrollHeight;"
"};"
"ws.onerror=()=>{wsStatus.textContent='error';wsStatus.style.color='var(--danger)'};"
"ws.onmessage=ev=>{"
"let data;"
"if(ev.data instanceof ArrayBuffer){data=new Uint8Array(ev.data);let s='';for(let i=0;i<data.length;i++)s+=String.fromCharCode(data[i]);termBox.textContent+=s.replace(/[\\u0000-\\u0008\\u000b\\u000c\\u000e-\\u001f]/g,'')}"
"else{termBox.textContent+=ev.data.replace(/[\\u0000-\\u0008\\u000b\\u000c\\u000e-\\u001f]/g,'')}"
"termBox.scrollTop=termBox.scrollHeight;"
"};"
"}"
"function wsDisconnect(){if(ws){ws.close();ws=null}}"
"document.getElementById('wsConnect').onclick=wsConnect;"
"document.getElementById('wsDisconnect').onclick=wsDisconnect;"
"document.getElementById('termClear').onclick=()=>{termBox.textContent=''};"
"function termSend(){if(!ws||ws.readyState!==1){alert('WebSocket not connected');return}let v=termInput.value;if(v){ws.send(v);termInput.value=''}}"
"document.getElementById('termSend').onclick=termSend;"
"termInput.addEventListener('keydown',e=>{"
"if(e.ctrlKey&&e.key==='Enter'){e.preventDefault();ws&&ws.readyState===1&&ws.send(termInput.value+'\\n');termInput.value='';return}"
"if(e.key==='Enter'&&!e.shiftKey&&!e.ctrlKey){e.preventDefault();termSend()}"
"});"
"/* Ctrl+C/D/Z 等控制字符 */"
"termInput.addEventListener('keydown',e=>{"
"if(!e.ctrlKey||e.shiftKey)return;"
"const map={'c':3,'d':4,'z':26,'a':1,'e':5,'l':12};if(map[e.key.toLowerCase()]){e.preventDefault();if(ws&&ws.readyState===1){ws.send(String.fromCharCode(map[e.key.toLowerCase()]));termBox.textContent+='[sent ^'+e.key.toUpperCase()+']';termBox.scrollTop=termBox.scrollHeight}}"
"});"
"</script></body></html>"
;

/* ================================================================
 *  轻量 JSON 生成（手写 snprintf，零 cJSON 依赖——省 ~20KB flash）
 *  只处理我们自己的简单结构：status 对象、wifi scan 数组、cred 数组
 * ================================================================ */

/* GET /api/status — 聚合状态 + features 数组 */
static esp_err_t api_status_get(httpd_req_t *req)
{
    bool connected = bridge_wifi_is_connected();
    const char *ip = bridge_wifi_get_ip_str();
    bool ap_active = bridge_wifi_ap_is_active();
    const char *ap_ssid = ap_active ? bridge_wifi_ap_get_ssid() : NULL;
    uint64_t uptime_s = esp_timer_get_time() / 1000000ULL;

    char body[512];
    int n = 0;
    n += snprintf(body + n, sizeof(body) - n,
        "{\"version\":\"%s\",\"uptime\":%" PRIu64,
        AIRSERIAL_VERSION, uptime_s);
    n += snprintf(body + n, sizeof(body) - n,
        ",\"sta_connected\":%s,\"sta_ssid\":null",
        connected ? "true" : "false");
    if (ip && ip[0]) {
        n += snprintf(body + n, sizeof(body) - n,
            ",\"sta_ip\":\"%s\"", ip);
    } else {
        n += snprintf(body + n, sizeof(body) - n,
            ",\"sta_ip\":null");
    }
    n += snprintf(body + n, sizeof(body) - n,
        ",\"ap_active\":%s", ap_active ? "true" : "false");
    if (ap_ssid && ap_ssid[0]) {
        n += snprintf(body + n, sizeof(body) - n,
            ",\"ap_ssid\":\"%s\"", ap_ssid);
    } else {
        n += snprintf(body + n, sizeof(body) - n,
            ",\"ap_ssid\":null");
    }
    n += snprintf(body + n, sizeof(body) - n,
        ",\"ap_ip\":\"%s\"", s_ap_ip);
    n += snprintf(body + n, sizeof(body) - n,
        ",\"features\":[\"wifi_scan\",\"wifi_connect\",\"wifi_saved\",\"wifi_forget\",\"usb_host\",\"serial_config\"]}");

    if (n < 0 || (size_t)n >= sizeof(body)) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "response too large");
        return ESP_FAIL;
    }
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, body, HTTPD_RESP_USE_STRLEN);
    ESP_LOGI(TAG, "[HTTP] GET /api/status");
    return ESP_OK;
}

/* GET /api/wifi/scan — 返回最近一次扫描结果 */
static esp_err_t api_wifi_scan_get(httpd_req_t *req)
{
    static bridge_wifi_ap_t s_aps[20];
    int n = bridge_wifi_get_scan_results(s_aps, 20);
    bool scanning = bridge_wifi_scan_in_progress();

    /* 预估容量：20 * (33 ssid + 20 rest) ≈ 1KB，buffer 开 1.5KB 留余量 */
    char *buf = malloc(1536);
    if (!buf) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "oom");
        return ESP_FAIL;
    }
    int off = snprintf(buf, 1536, "{\"scanning\":%s,\"count\":%d,\"aps\":[",
                       scanning ? "true" : "false", n);
    for (int i = 0; i < n && off > 0 && off < 1400; i++) {
        if (i > 0) buf[off++] = ',';
        int w = snprintf(buf + off, 1536 - off,
            "{\"ssid\":\"%s\",\"rssi\":%d,\"channel\":%u,\"open\":%s,\"hidden\":%s}",
            s_aps[i].ssid, s_aps[i].rssi, s_aps[i].channel,
            s_aps[i].open ? "true" : "false",
            s_aps[i].hidden ? "true" : "false");
        if (w < 0) break;
        off += w;
    }
    if (off > 0) snprintf(buf + off, 1536 - off, "]}");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
    free(buf);
    ESP_LOGI(TAG, "[HTTP] GET /api/wifi/scan (count=%d, scanning=%d)", n, scanning);
    return ESP_OK;
}

/* POST /api/wifi/scan — 触发异步扫描 */
static esp_err_t api_wifi_scan_post(httpd_req_t *req)
{
    int total = req->content_len;
    if (total > 0) {
        char dummy[64];
        httpd_req_recv(req, dummy, total > (int)sizeof(dummy) ? (int)sizeof(dummy) : total);
    }
    esp_err_t ret = bridge_wifi_scan_async();
    char body[64];
    snprintf(body, sizeof(body), "{\"ok\":%s}", ret == ESP_OK ? "true" : "false");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, body, HTTPD_RESP_USE_STRLEN);
    ESP_LOGI(TAG, "[HTTP] POST /api/wifi/scan -> %s", esp_err_to_name(ret));
    return ESP_OK;
}

/* GET /api/wifi/saved — NVS 凭证列表 */
static esp_err_t api_wifi_saved_get(httpd_req_t *req)
{
    static bridge_wifi_cred_t s_creds[BRIDGE_WIFI_MAX_CREDS];
    int n = bridge_wifi_load_all_creds(s_creds, BRIDGE_WIFI_MAX_CREDS);

    char *buf = malloc(512);
    if (!buf) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "oom");
        return ESP_FAIL;
    }
    int off = snprintf(buf, 512, "{\"count\":%d,\"creds\":[", n);
    for (int i = 0; i < n && off > 0 && off < 480; i++) {
        if (i > 0) buf[off++] = ',';
        int w = snprintf(buf + off, 512 - off,
            "{\"ssid\":\"%s\",\"open\":%s}",
            s_creds[i].ssid, s_creds[i].open ? "true" : "false");
        if (w < 0) break;
        off += w;
    }
    if (off > 0) snprintf(buf + off, 512 - off, "]}");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
    free(buf);
    ESP_LOGI(TAG, "[HTTP] GET /api/wifi/saved (count=%d)", n);
    return ESP_OK;
}

/* POST /api/wifi/connect — 发起异步连接 */
static esp_err_t api_wifi_connect_post(httpd_req_t *req)
{
    int total = req->content_len;
    if (total <= 0) total = 256;
    if (total > 512) total = 512;
    char *body = malloc(total + 1);
    if (!body) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "oom");
        return ESP_FAIL;
    }
    body[0] = '\0';
    int received = 0;
    while (received < total) {
        int r = httpd_req_recv(req, body + received, total - received);
        if (r <= 0) {
            /* chunked transfer: try one more time, then break */
            if (received == 0) { vTaskDelay(pdMS_TO_TICKS(10)); continue; }
            break;
        }
        received += r;
    }
    body[received] = '\0';

    /* 手写 JSON 解析：只提 ssid 和 password 两个 string 字段 */
    /* 修复版：value NUL 终止后手动 p=vend+1 跳过，避免 strchr 在 \0 处返回 NULL */
    const char *ssid = NULL, *password = NULL;
    char ssid_buf[33] = {0}, pass_buf[65] = {0};
    char *p = body;

    while (*p) {
        /* 找 key 开头引号 */
        if (*p != '"') { p++; continue; }
        char *key_start = p + 1;
        /* 找 key 结束引号 */
        char *key_end = strchr(key_start, '"');
        if (!key_end) break;
        /* 跳过 key 结束引号后的空格和冒号 */
        p = key_end + 1;
        while (*p == ' ') p++;
        if (*p != ':') { p++; continue; }
        p++;
        while (*p == ' ') p++;
        /* 找 value：应该是字符串 */
        if (*p != '"') {
            /* 非字符串（null/数字），跳过 */
            while (*p && *p != ',') p++;
            continue;
        }
        p++;  /* 跳过 value 开始引号 */
        char *val_start = p;
        char *val_end = strchr(p, '"');
        if (!val_end) break;

        /* 提 key 名字（临时打 NUL 在 key_end）*/
        char key[33] = {0};
        int klen = (int)(key_end - key_start);
        if (klen > (int)sizeof(key) - 1) klen = (int)sizeof(key) - 1;
        memcpy(key, key_start, klen);

        /* NUL 终止 value */
        *val_end = '\0';

        if (strcmp(key, "ssid") == 0) {
            int vlen = (int)(val_end - val_start);
            if (vlen > (int)sizeof(ssid_buf) - 1) vlen = (int)sizeof(ssid_buf) - 1;
            memcpy(ssid_buf, val_start, vlen);
            ssid = ssid_buf;
        } else if (strcmp(key, "password") == 0) {
            int vlen = (int)(val_end - val_start);
            if (vlen > (int)sizeof(pass_buf) - 1) vlen = (int)sizeof(pass_buf) - 1;
            memcpy(pass_buf, val_start, vlen);
            password = pass_buf;
        }

        /* 关键修复：手动移到 value 结束引号之后，找下一个 key 引号 */
        p = val_end + 1;
        while (*p && *p != '"') p++;
    }

    if (!ssid || !ssid[0]) {
        free(body);
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"missing ssid\"}");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "[HTTP] /api/wifi/connect raw body: '%s'", body);
    ESP_LOGI(TAG, "[HTTP] /api/wifi/connect parsed ssid='%s' password='%s' pass_len=%d",
             ssid ? ssid : "(null)",
             password ? password : "(null)",
             password ? (int)strlen(password) : 0);

    bridge_wifi_reset_conn_state();
    esp_err_t ret = bridge_wifi_connect_async_ex(ssid, (password && password[0]) ? password : NULL, 15000);

    /* 存凭证（连接发起时先存——无论成败都更新 last_used） */
    (void)bridge_wifi_save_cred(ssid, (password && password[0]) ? password : "",
                                !password || !password[0]);

    free(body);
    char resp[128];
    snprintf(resp, sizeof(resp), "{\"ok\":%s}", ret == ESP_OK ? "true" : "false");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    ESP_LOGI(TAG, "[HTTP] POST /api/wifi/connect ssid='%s' -> %s", ssid, esp_err_to_name(ret));
    return ESP_OK;
}

/* DELETE /api/wifi/saved — 删指定凭证 */
static esp_err_t api_wifi_saved_delete(httpd_req_t *req)
{
    int total = req->content_len;
    char body[256] = {0};
    int received = 0;
    while (received < total && received < (int)sizeof(body) - 1) {
        int r = httpd_req_recv(req, body + received, total - received);
        if (r <= 0) break;
        received += r;
    }
    /* 提 ssid（复用 connect 的极简解析） */
    const char *ssid = NULL;
    char *p = body;
    while ((p = strchr(p, '"')) != NULL) {
        p++;
        const char *k = p;
        if ((p = strchr(p, '"')) == NULL) break;
        char *colon = strchr(p + 1, ':');
        if (!colon) break;
        p = colon + 1;
        while (*p == ' ') p++;
        if (*p != '"') { while (*p && *p != ',') p++; if (p) p++; continue; }
        p++;
        const char *vstart = p;
        if ((p = strchr(p, '"')) == NULL) break;
        char key[32] = {0}; int i;
        for (i = 0; i < (int)(sizeof(key) - 1) && k < p; i++) key[i] = *k++;
        if (strcmp(key, "ssid") == 0) { ssid = vstart; *(char *)p = '\0'; break; }
        if ((p = strchr(p, ',')) == NULL) break;
        p++;
    }

    esp_err_t ret = ESP_OK;
    if (ssid && ssid[0]) {
        ret = bridge_wifi_forget_cred(ssid);
    }
    char resp[64];
    snprintf(resp, sizeof(resp), "{\"ok\":%s}", ret == ESP_OK ? "true" : "false");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    ESP_LOGI(TAG, "[HTTP] DELETE /api/wifi/saved ssid='%s' -> %s", ssid ? ssid : "(null)", esp_err_to_name(ret));
    return ESP_OK;
}

/* GET / — 通用设置页 SPA */
static esp_err_t root_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_send(req, s_html, HTTPD_RESP_USE_STRLEN);
    ESP_LOGI(TAG, "[HTTP] GET / (%d bytes)", (int)strlen(s_html));
    return ESP_OK;
}

/* 探针路径 → 302 回首页 */
static esp_err_t probe_get_handler(httpd_req_t *req)
{
    char loc[40];
    snprintf(loc, sizeof(loc), "http://%s/", s_ap_ip);
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", loc);
    httpd_resp_send(req, NULL, 0);
    ESP_LOGI(TAG, "[HTTP] probe %s -> %s", req->uri, loc);
    return ESP_OK;
}

/* 404 兜底：配网态拉回首页，其他态返回轻量 404 JSON 给 API 客户端 */
static esp_err_t fallback_handler(httpd_req_t *req, httpd_err_code_t err)
{
    if (err == HTTPD_404_NOT_FOUND) {
        if (strncmp(req->uri, "/api/", 5) == 0) {
            /* API 路径 → JSON 404（不是配网态） */
            httpd_resp_set_status(req, "404 Not Found");
            httpd_resp_set_type(req, "application/json");
            httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"not found\"}");
            return ESP_OK;
        }
        /* 非 API 路径 → 配网态 302 回首页（手机体验一致） */
        return probe_get_handler(req);
    }
    return ESP_FAIL;
}

/* ---- URI 注册表 ---- */
static const httpd_uri_t s_uris[] = {
    { .uri = "/",                   .method = HTTP_GET,    .handler = root_get_handler },
    { .uri = "/api/status",         .method = HTTP_GET,    .handler = api_status_get },
    { .uri = "/api/wifi/scan",      .method = HTTP_GET,    .handler = api_wifi_scan_get },
    { .uri = "/api/wifi/scan",      .method = HTTP_POST,   .handler = api_wifi_scan_post },
    { .uri = "/api/wifi/saved",     .method = HTTP_GET,    .handler = api_wifi_saved_get },
    { .uri = "/api/wifi/saved",     .method = HTTP_DELETE, .handler = api_wifi_saved_delete },
    { .uri = "/api/wifi/connect",   .method = HTTP_POST,   .handler = api_wifi_connect_post },
    /* WebSocket 串口终端（需 CONFIG_HTTPD_WS_SUPPORT=y）
     * IDF6 握手不进 handler → 连接建立走 ws_post_handshake_cb */
    { .uri = "/ws", .method = HTTP_GET, .handler = ws_term_handler,
      .is_websocket = true, .ws_post_handshake_cb = ws_post_handshake_cb },
};

esp_err_t bridge_web_portal_start(const char *ap_ssid, const char *ap_ip)
{
    if (s_portal_running) {
        ESP_LOGW(TAG, "portal already running");
        return ESP_ERR_INVALID_STATE;
    }

    if (ap_ssid && ap_ssid[0]) strlcpy(s_ap_ssid, ap_ssid, sizeof(s_ap_ssid));
    if (ap_ip && ap_ip[0]) strlcpy(s_ap_ip, ap_ip, sizeof(s_ap_ip));

    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port = BRIDGE_WEB_HTTP_PORT;
    cfg.ctrl_port = 32768 + BRIDGE_WEB_HTTP_PORT;
    cfg.max_open_sockets = 10;      /* 桌面+手机并存（各 2-3 keep-alive + 1 WS），6 会把手机 WS 挤死 */
    cfg.stack_size = 8192;          /* v0.5: 加大栈 */
    cfg.max_uri_handlers = 32;
    cfg.task_priority = 6;
    cfg.core_id = 1;                /* 扔 CPU1！跟 USB Host (CPU0) 物理隔离，永不饿死 */

    esp_err_t ret = httpd_start(&s_httpd, &cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start failed: %s", esp_err_to_name(ret));
        return ret;
    }

    for (size_t i = 0; i < sizeof(s_uris) / sizeof(s_uris[0]); i++) {
        httpd_register_uri_handler(s_httpd, &s_uris[i]);
    }
    for (size_t i = 0; i < sizeof(s_probe_paths) / sizeof(s_probe_paths[0]); i++) {
        httpd_uri_t u = { .uri = s_probe_paths[i], .method = HTTP_GET, .handler = probe_get_handler };
        httpd_register_uri_handler(s_httpd, &u);
    }
    httpd_register_err_handler(s_httpd, HTTPD_404_NOT_FOUND, fallback_handler);

#if 0   /* captive DNS v0.3.1 暂时关闭 */
    ret = bridge_web_dns_start(s_ap_ip);
#endif

    s_portal_running = true;
    ESP_LOGI(TAG, "[PORTAL] on: http://%s/ ssid='%s' HTML=%d bytes",
             s_ap_ip, s_ap_ssid[0] ? s_ap_ssid : "(none)", (int)strlen(s_html));
    return ESP_OK;
}

esp_err_t bridge_web_portal_stop(void)
{
    if (!s_portal_running) return ESP_OK;
#if 0
    bridge_web_dns_stop();
#endif

    /* 停 WebSocket */
    s_ws_task_running = false;
    s_ws_client_count = 0;
    if (s_ws_sub) {
        bridge_serial_unsubscribe(s_ws_sub);
        s_ws_sub = NULL;
    }

    esp_err_t ret = ESP_OK;
    if (s_httpd) {
        ret = httpd_stop(s_httpd);
        s_httpd = NULL;
    }
    s_portal_running = false;
    ESP_LOGI(TAG, "[PORTAL] off");
    return ret;
}

bool bridge_web_portal_is_running(void)
{
    return s_portal_running;
}

esp_err_t bridge_web_add_uri_handler(const httpd_uri_t *uri)
{
    if (!s_httpd) return ESP_ERR_INVALID_STATE;
    if (!uri || !uri->handler || !uri->uri) return ESP_ERR_INVALID_ARG;
    esp_err_t ret = httpd_register_uri_handler(s_httpd, uri);
    ESP_LOGI(TAG, "add_uri %s %s → %s",
             uri->method == HTTP_GET ? "GET" :
             uri->method == HTTP_POST ? "POST" : "?",
             uri->uri, esp_err_to_name(ret));
    return ret;
}

/* ================================================================
 *  WebSocket 终端 (/ws)
 *  浏览器 ↔ WebSocket ↔ bridge_serial ↔ bridge_uart ↔ 目标板
 * ================================================================ */

/* WebSocket 转发 task：从 bridge_serial subscriber 读 UART RX 数据，通过 WebSocket 发给客户端 */
static void ws_forward_task(void *arg)
{
    ESP_LOGI(TAG, "[WS] forward task started");
    uint8_t buf[WS_RX_BUF_SIZE];

    while (s_ws_task_running) {
        /* 没有客户端 fd 时让出 CPU */
        int fd = s_ws_client_fd;
        if (fd < 0) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        /* 订阅者从独立小环读数据 */
        size_t n = bridge_serial_sub_read(s_ws_sub, buf, sizeof(buf), 50);
        if (n > 0) {
            httpd_ws_frame_t frame;
            memset(&frame, 0, sizeof(frame));
            frame.payload = buf;
            frame.len = n;
            frame.type = HTTPD_WS_TYPE_TEXT;
            esp_err_t ret = httpd_ws_send_data(s_httpd, fd, &frame);
            if (ret != ESP_OK) {
                /* 客户端断了，清掉 fd */
                ESP_LOGW(TAG, "[WS] send failed: %s, client gone", esp_err_to_name(ret));
                s_ws_client_fd = -1;
                s_ws_client_count = 0;
            }
        }
    }

    ESP_LOGI(TAG, "[WS] forward task stopped");
    vTaskDelete(NULL);
}

static void ws_ensure_sub_and_task(void)
{
    if (!s_ws_sub) {
        s_ws_sub = bridge_serial_subscribe("webterm");
        ESP_LOGI(TAG, "[WS] subscriber registered: %p", s_ws_sub);
    }
    if (!s_ws_task_running) {
        s_ws_task_running = true;
        BaseType_t ret = xTaskCreate(ws_forward_task, "ws_fwd", WS_TASK_STACK, NULL, WS_TASK_PRIO, &s_ws_task_handle);
        if (ret != pdPASS) {
            ESP_LOGE(TAG, "[WS] xTaskCreate failed: %d", ret);
            s_ws_task_running = false;
        } else {
            ESP_LOGI(TAG, "[WS] forward task created");
        }
    }
}

/* IDF6：WS 握手由 httpd 内核完成（不进 handler）。
 * 连接建立后经 post-handshake 回调通知：记 fd + 启动转发链路。
 * 单客户端语义：新连接进来时踢掉旧 fd，防僵尸连接耗尽会话槽 */
static esp_err_t ws_post_handshake_cb(httpd_req_t *req)
{
    int fd = httpd_req_to_sockfd(req);
    int old = s_ws_client_fd;
    if (old >= 0 && old != fd && s_httpd) {
        ESP_LOGW(TAG, "[WS] replacing stale client fd=%d with fd=%d", old, fd);
        httpd_sess_trigger_close(s_httpd, old);
    }
    ESP_LOGI(TAG, "[WS] client connected, fd=%d", fd);
    s_ws_client_fd = fd;
    s_ws_client_count = 1;
    ws_ensure_sub_and_task();
    return ESP_OK;
}

/* 服务端→客户端 TEXT 帧（同步直发）。httpd_ws_send_data 内部是
 * queue_work + 无限等完成，而 work 只由 httpd 控制线程执行——
 * 在 ws_term_handler（控制线程）里调用它等于自己等自己，永久死锁。
 * CLI 回复就在 handler 上下文，必须用 httpd_socket_send 直接发。 */
static esp_err_t ws_send_text_frame(int fd, const uint8_t *payload, size_t len)
{
    uint8_t hdr[4];
    int hlen = 0;
    hdr[hlen++] = 0x81;                    /* FIN + TEXT */
    if (len < 126) {
        hdr[hlen++] = (uint8_t)len;
    } else {
        hdr[hlen++] = 126;
        hdr[hlen++] = (uint8_t)(len >> 8);
        hdr[hlen++] = (uint8_t)len;
    }
    if (httpd_socket_send(s_httpd, fd, (const char *)hdr, hlen, 0) < 0) {
        return ESP_FAIL;
    }
    return httpd_socket_send(s_httpd, fd, (const char *)payload, len, 0) < 0
           ? ESP_FAIL : ESP_OK;
}

/* CLI 回复 → WS TEXT 帧（bridge_cli_process 通道注入，handler 上下文同步发） */
static void ws_cli_reply(const char *line, void *arg)
{
    (void)arg;
    int fd = s_ws_client_fd;
    if (fd < 0 || !s_httpd) return;
    esp_err_t r = ws_send_text_frame(fd, (const uint8_t *)line, strlen(line));
    ESP_LOGI(TAG, "[WS] cli reply %zu bytes -> %s", strlen(line), esp_err_to_name(r));
}

/* 数据帧 handler：浏览器发来的串口输入（IDF6 帧到达时 method=0，靠帧类型判断）*/
static esp_err_t ws_term_handler(httpd_req_t *req)
{
    httpd_ws_frame_t ws_pkt;
    uint8_t *buf = NULL;
    memset(&ws_pkt, 0, sizeof(ws_pkt));

    /* 先读 header 拿到长度 */
    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "[WS] recv header failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* 只处理数据帧；控制帧（PING/PONG/CLOSE）由 httpd 内核自动处理 */
    if (ws_pkt.type != HTTPD_WS_TYPE_TEXT && ws_pkt.type != HTTPD_WS_TYPE_BINARY) {
        return ESP_OK;
    }

    if (ws_pkt.len > 0) {
        buf = malloc(ws_pkt.len + 1);
        if (!buf) return ESP_ERR_NO_MEM;
        ws_pkt.payload = buf;
        ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "[WS] recv payload failed: %s", esp_err_to_name(ret));
            free(buf);
            return ret;
        }
        /* 手动补 \0：recv_frame 不写终止符，CLI 的 strncmp/strlen 需要 C 字符串 */
        buf[ws_pkt.len] = '\0';
        /* 本地命令拦截：!airsn … 与 RFC2217 通道共用同一套分发；
         * 前缀不符（false）则原样穿透给目标板 */
        if (buf[0] == '!' &&
            bridge_cli_process((char *)buf, ws_cli_reply, NULL, "Web terminal")) {
            free(buf);
            return ESP_OK;
        }
        /* 浏览器输入 → bridge_serial TX 环 → UART → 目标板 */
        size_t written = bridge_serial_write(buf, ws_pkt.len, 100);
        ESP_LOGI(TAG, "[WS] → serial: %zu bytes (written %zu)", ws_pkt.len, written);
        free(buf);
    }

    return ESP_OK;
}
