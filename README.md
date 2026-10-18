# N32H482-Gateway

基于国民技术 **N32H482RE** 的「4G + LoRa」工业网关固件。

- **本地**：LoRa（LLCC68 / SX1261，433 MHz）连接液位传感器与声光报警器
- **上云**：4G 模组（ML307R/ML307Y，UART AT）跑 MQTT，对接 RoadTwin 云协议
- **自治**：水位边缘评估 + 联动下发；断网时事件/遥测缓存、恢复后按原时间戳补报
- **运维**：`#` 串口控制台、OTA over HTTP（+MD5）、A/B 健康计数
- **产线**：不插 SIM 也能 `#bind` / `#set` / `#get` / `#unbind` 写出厂字段

> 架构、数据流、模块职责、约定与坑：见 **[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)**。

## 目录结构

```
N32H482-Gateway/
├── app/
│   ├── bsp/            板级支撑：GPIO / 定时器 / 串口
│   ├── common/         通用：CRC/UTF-8、Flash 存储、分区表、环形缓冲
│   ├── drivers/lora/   LLCC68 驱动 + 射频抽象 + 板级引脚
│   ├── links/g4/       4G 链路：AT 传输、URC、建链状态机
│   ├── links/lora/     LoRa 链路：帧协议、从机表、绑定/解绑/告警/字段
│   ├── links/serial/   RS232/RS485 通道（协议待定，仅建通道）
│   ├── cloud/          云协议：信封/21 条命令/上行构造、JSON 读写
│   ├── model/          配置模型：9 个配置域、掉电参数、档位、最后值
│   ├── service/        业务服务：水位评估、失联判定、事件/遥测缓存、本地告警
│   ├── ota/            OTA：HTTP 下载、分区写入、MD5、A/B 健康
│   ├── log/            日志口（USART1）
│   └── cjson/          cJSON + 静态池适配
├── src/                main.c（启动 + 主循环）、中断向量
├── inc/                公共头文件
├── bsp/                延时 / 日志接口（SDK 支撑层）
├── firmware/           N32H48x 标准外设库（勿改）
├── MDK-ARM/            Keil 工程（CDC.uvprojx）
├── _host_test/         主机测试（gcc，13 套件 / 1840 条断言）
├── archive/            已停用代码（不参与构建，见 archive/README.md）
├── docs/               架构文档
└── 相关资料/            4G / LoRa / MCU 原厂资料
```

## 开发环境

- Keil MDK V5.43a / ARMClang V6.24（target `N32H480` 与 `N32H48X` 现在完全一致）
- gcc（MinGW-w64）+ Python 3（只为主机测试与脚本）
- 芯片：N32H482RE（Cortex-M4，512 KB Flash，192 KB RAM，**栈 4 KB / 堆 512 B**）

## 构建

图形界面：打开 `MDK-ARM/CDC.uvprojx`，Build。

命令行（推荐用脚本，见下）：

```powershell
$log = "$env:TEMP\keil.log"
$p = Start-Process 'D:\Keil_v5\UV4\UV4.exe' `
     -ArgumentList '-j0','-b','MDK-ARM\CDC.uvprojx','-o',$log -Wait -PassThru
Get-Content $log -Tail 20
```

产物：`MDK-ARM\bin\CDC.bin`（`fromelf` 由 Keil 的 After-Build 命令生成）。

> **判断标准**：日志里必须是 `0 Error(s)`；退出码 `0`=无错误无警告，`1`=有警告，`≥2`=有错误。
> 同一个工程在 uVision 里开着时，命令行构建不可靠，先关掉。

## 测试（不用板子）

```powershell
powershell -ExecutionPolicy Bypass -File _host_test\run.ps1
```

- 13 个套件、1840 条断言，跑在 PC 上（假 ML307R / 假 Flash / 假 LoRa）
- 全绿时最后一行是 `all host tests passed`，退出码 0

一键验证（构建 + 测试）：

```powershell
powershell -ExecutionPolicy Bypass -File tools\verify.ps1
```

## ⚠️ 脚本必须存成 UTF-8 with BOM

PowerShell 5.1 对**没有 BOM** 的 `.ps1` 按 GBK 解码，中文注释行末的全角标点会把
**下一行整行吞掉**（曾导致测试套件的源文件列表少一项，测试照跑但少测）。
改完任何 `.ps1` 后跑一次：

```powershell
python _host_test\check_ps1.py     # 期望：危险行数 = 0
```

## 运行时模式

编译期三选一（见 `app/drivers/lora/project_config.h`，也可在 Keil 的
C/C++ → Define 里覆盖）：

| 宏 | 模式 | 说明 |
|---|---|---|
| `MAIN_MODE_CLOUD`（默认） | 云端联调 | 4G + LoRa + 云协议全套 |
| `MAIN_MODE_PAIRING` | 从机配对 | 只起射频，`0x04` 配对请求回 ACK |
| `MAIN_MODE_LORA_TEST` | 射频测试 | 每 5 s 发一帧，其余时间收 |

三者必须恰好选一个，多选/少选直接编译报错。
