# AirSerial

把一台 ESP32-S3 设备变成**无线串口桥**：目标开发板的串口经 WiFi 暴露到局域网，PC 端用 esptool / pyserial / 任意终端远程烧录固件、看日志、交互调试——目标板可以留在现场、机柜里或另一间屋子。

当前版本：**v0.3.1-alpha**（唯一出处 `firmware-idf/components/bridge_core/include/app_version.h`）

***

## 功能一览

| 能力              | 说明                                                         |
| --------------- | ---------------------------------------------------------- |
| RFC2217 桥       | 端口 **3334**,esptool / pyserial 原生支持，波特率/DTR/RTS 远程可控       |
| TCP 裸流桥         | 端口 **3333**，任意 telnet / nc / 脚本直连                          |
| 多 client fanout | RFC2217 + TCP + WebSocket 三路订阅者并行收数据，互不挤占                  |
| USB Host 模式     | USB OTG 直插目标板的 USB 串口（CDC-ACM / CH34x / CP210x / FTDI)     |
| UART 兜底         | 板载 GPIO 直连目标板 UART，无需任何 USB 串口芯片                           |
| DTR/RTS 自动复位    | esptool 经典复位序列整形转发，远程进下载模式免按键                              |
| Web Portal      | 配网、USB Host 开关、状态监控、WebSocket 终端（内嵌 JS 终端，支持 Ctrl+C 等控制字符） |
| LCD 状态页         | IP、client 数、波特率、PSRAM 余量实时显示；KEY2 切 Boot Terminal 看实时串口输出  |
| mDNS 发现         | `airserial-xxxx.local`，服务 `_airserial_tcp._tcp`            |
| 带外 CLI          | RFC2217 会话内发 `!airsn status` 等命令，不打断串口数据                   |
| 崩溃遗言            | panic 后重启自报上次死因并落 NVS，可远程读取                                |

***

## 支持硬件

### 桥本体（刷 AirSerial 的设备）

| 板                                     | 状态      | 说明                               |
| ------------------------------------- | ------- | -------------------------------- |
| **M5StickS3** | ✅ 全功能验证 | 有屏有 PSRAM,SHARED\_PHY 单 Type-C 口 |
| **N16R8 DevKit**(S3-WROOM-1U-N16R8公模)   | ✅ 全功能验证 | headless，无屏，有独立OTG接口             |
| PowerHub / Cardputer ADV              | 📋 计划   | —                                |

### 目标板侧（被远程烧录的设备）

- **UART 兜底路线**：任意 3.3V UART 目标板，三根线（TX/RX/GND）即可。
- **USB Host 路线**：目标板上是 **CDC-ACM(USB-JTAG 原生）、CH34x、CP210x、FTDI** 之一即可。
- ⚠️ **克隆 CH340（版本号 0x31）不支持**，见下文专章。

实测通过的目标板：ESP32-S3(USB-JTAG 原生 + UART)、ESP32-C3/C5/C6(USB-JTAG)、ESP8266（原装串口芯片板）。

***

## 快速开始

前置：桥固件已通过 USB 烧录到设备（构建/烧录见文末）。

### 1. 连接设备

首次开机自动起 SoftAP：`AirSerial-XXXX`(XXXX = MAC 后 4 位）。两种用法都行：

- **直连模式**：PC/手机连上该 AP，设备地址即 **192.168.4.1**，开箱可用，不依赖任何现有网络；
- **并入现有网络（可选）**：浏览器开 http://192.168.4.1/ 填 WiFi 凭证，重启后切到 STA 模式，设备从路由器取 IP(LCD 和 portal 首页会显示），同网段经 `airserial-xxxx.local` 或该 IP 访问。

### 2. 远程使用（以 esptool 为例）

下例主机名按连接方式替换：直连 AP 用 `192.168.4.1`;STA 模式用 `airserial-xxxx.local` 或路由器分配的 IP(Windows 解析不了 `.local` 时直接用 IP)。

**USB Host 路线**（目标板 USB 口直插桥的 OTG 口，默认握手已验证，自动进下载模式免按键；⚠️ StickS3 的 Type-C 口不对外供电，需自供电 OTG 方案，见下文"USB Host 模式说明"）:

```powershell
# 探芯片
esptool --port rfc2217://airserial-xxxx.local:3334 chip-id

# 烧录
esptool --port rfc2217://airserial-xxxx.local:3334 write-flash 0x10000 firmware.bin
```

**UART 接线路线 / 默认握手超时的高延迟链路**：先手动让目标板进下载模式（按住 BOOT → 点一下 RESET → 松开 BOOT)，再跳过握手：

```powershell
esptool --port rfc2217://192.168.x.x:3334 --before no-reset-no-sync --no-stub chip-id
```

pyserial 直接当本地串口用：

```python
import serial
s = serial.serial_for_url("rfc2217://airserial-xxxx.local:3334", baudrate=115200, timeout=1)
s.write(b"hello target\n")
```

裸 TCP（无协议开销，适合日志流）:`nc airserial-xxxx.local 3333`

WebSocket 终端：浏览器开 portal → Serial 页，即开即用。

### 3. 带外 CLI(RFC2217 会话内）

```
!airsn status            # 版本/板名/堆栈/连接数
!airsn set baud 921600   # 动态切目标串口波特率
!airsn usbhost on|off    # 同 portal 开关，写 NVS 后重启生效
!airsn restart           # 软重启
```

未知 `!airsn` 命令整行原样穿透给目标板，不会吃掉；目标板方向来的 `!airsn` 是串口数据，不拦截。

***

## USB Host 模式说明

- **StickS3 需要自供电 OTG 方案**：StickS3 的 Type-C 口 VBUS 只进不出（电池供电设备，无 5V 输出能力），目标板直插是**没有电的**。必须用带 5V 注入的 OTG 转接头/线（或供电 USB Hub），或者目标板独立供电——无论哪种方式，目标板 GND 都会经 USB 线与桥共地，无需额外地线。N16R8 DevKit 这类经 USB 口取电的开发板，OTG 口 VBUS 与供电口共轨，通常可直接输出 5V，无此限制。
- **默认 OFF**。SHARED\_PHY 拓扑板（StickS3 这类单 Type-C 口）上，USB-OTG 与控制台 USB-Serial-JTAG 共用同一个内置 PHY，开 Host 后**本机 USB 控制台（COM 口）会断开**——这是硬件拓扑决定的，不是 bug。桥功能（WiFi/RFC2217）不受影响。
- 开关方式：portal 的 USB Host 卡片，或 CLI `!airsn usbhost on|off`，均写 NVS 后**重启生效**。
- 关闭（host\_off）后软重启即可恢复 USB-Serial-JTAG 枚举（v0.3.1-alpha 起固件会在启动时强制把 PHY 归位，无需断电）。
- 独立串口芯片拓扑（INDEPENDENT，如自带 CH343 的板）开 Host 不断控制台，默认自动 ON。

## UART 兜底接线（StickS3 实测）

| StickS3 Hat2 | 目标板                   |
| ------------ | --------------------- |
| 脚2 G5（桥 RX)  | 目标 TX（如 S3-Zero IO43) |
| 脚4 G4（桥 TX)  | 目标 RX（如 S3-Zero IO44) |
| 脚1 GND       | GND                   |

- 桥 UART1 默认 115200 8N1，可经 RFC2217 / CLI 切到 921600。
- 自动复位信号（active-low）按 esptool 标准约定：**RTS=GPIO3 → 目标 EN**、**DTR=GPIO6 → 目标 GPIO0**，接好即可远程进下载模式免按键；目标板 EN 不可达时（如 S3-Zero 无焊盘），手动进下载模式（按住 BOOT→点 RESET→松）再 `--before no-reset-no-sync`。

***

## ⚠️ 克隆 CH340（版本 0x31）不兼容声明

市面上部分低价板（某些 ESP8266/NodeMCU 克隆板、山寨下载器）使用**克隆 CH340 芯片，固件版本号 0x31**。该芯片有硬件级 RX 定时器缺陷：当 RX 侧凑满 ≥64 字节的"楔形尾包"（例如 ESP8266 74880 boot log 垃圾字节 + esptool sync 应答）时，数据会在芯片内部**卡住约 3 秒**才被后续流量顶出。后果是 esptool 的 sync 应答永远迟到，握手必败——**这是芯片缺陷，不是 AirSerial 的兼容性问题，软件侧无法修复**。

AirSerial 的处理：

- 枚举到 WCH 设备后自动读 0x5F 版本寄存器，`0x31` 判定为克隆；
- 桥控制台打印八行 `CLONE CH340 DETECTED` 警告框；
- portal 顶部显示红色横幅（中英文）;
- `GET /api/usb/debug` 暴露 `"ch340_ver":"31","ch340_clone":true`，供上位机检测。

**建议**：换用原装 WCH CH340/CH343、Silabs CP2102/CP2104、FTDI 的板子。ESP8266 本身没有问题——同一套固件在原装串口芯片的 8266 板上验证通过。

***

## 构建

- 依赖 **ESP-IDF v6.0.3**（本项目锁定版本，勿用其他大版本）。用官方安装器装好后，在 "ESP-IDF PowerShell" 里操作；或手动执行 IDF 目录下的 `export.ps1` 激活环境。
- 在仓库 `firmware-idf/` 目录下构建 StickS3 release(**`SDKCONFIG` 必须给绝对路径**，否则会静默回退默认配置）:

```powershell
idf.py -B build-sticks3-release `
  -D SDKCONFIG="$PWD\sdkconfig-release-sticks3" `
  -D SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.defaults.sticks3" build
idf.py -p COMx -B build-sticks3-release flash
```

- 其他已验证板型：换对应的 `sdkconfig.defaults.<board>`（如 `n16r8`）即可。

## 项目结构

```
firmware-idf/
├── main/                  # 纯装配层:只调 init/start,零硬件细节
└── components/
    ├── bridge_core/       # RFC2217 / TCP / 串口管道 / mDNS / 崩溃遗言(零板级依赖)
    ├── bridge_uart/       # UART1 兜底 transport(GPIO4/5)
    ├── bridge_usb_host/   # USB Host CDC-ACM + CH34x/CP210x/FTDI + DTR/RTS 整形
    ├── bridge_wifi/       # WiFi 抽象(默认 S3 内置后端)
    ├── bridge_web/        # HTTP portal + DNS captive + WebSocket 终端
    ├── board/             # 板级子组件,一板一文件(sticks3 / n16r8 / cardputer...)
    ├── ui_view_manager/   # LCD 状态页 / Boot Terminal
    └── ui_cga/            # 5x7/8x16 字符渲染
```

## Alpha 状态与已知边界

- **无 OTA**。
- **esptool 握手限制（UART 接线路线）**：经高延迟 WiFi(RTT ~80ms+）时默认 ROM_SYNC 握手会稳定失败（重试无用），需 `--before no-reset-no-sync` 并手动让目标板进下载模式；USB Host 直插路线默认握手已验证通过。此问题计划在未来版本中优化。
- 克隆 CH340 0x31 不支持（见专章）。
- 版本号以 `app_version.h` 为准；portal HTML 里的版本显示如有出入以头文件为准。

## 许可

Copyright 2026 Zyntra

项目自有代码以 [Apache License 2.0](LICENSE) 发布；固件基于 ESP-IDF(Apache-2.0）及其托管组件构建，第三方组件许可证随构建产物附带。
