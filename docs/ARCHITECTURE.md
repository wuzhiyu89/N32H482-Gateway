# 架构总览

> 这份文档回答四个问题：**系统由哪些层组成、一帧数据怎么流、每个目录负责什么、
> 改代码时必须守住哪些约束。**
> 面向第一次接手的人（或 AI）：读完这一篇，再按需看模块头部的注释。

---

## 1. 系统形态

一块 **N32H482RE** 网关板，同时接两个无线/移动网络：

```
液位传感器 ──LoRa──┐
声光报警器 ──LoRa──┤→ 本板 ──4G/MQTT──→ 云平台
                  │        ←──命令──
                  └── 本地联动下发 / 告警
```

- **不上 RTOS**：裸机 + 1 ms 时基（ATIM1）分频调度
- **两种从机**：液位（`0x02` 类型）、声光（`0x03` 类型）；地址统一是
  **13 字节 UID**（1 字节类型/环境 + 96 位芯片 UID，显示为 26 位 hex）
- **一条空口协议**：`AA 55 | ver | gw_uid[13] | dev_uid[13] | func | dlen | data | CRC16`
  （帧开销 33 B，data ≤ 207 B）
- **一条云协议**：`{cmd,msgId,ts,auth,data}` 下行 / `{event,...}` 上行，
  21 条命令、9 个配置域

## 2. 资源约束（会直接影响写法）

| 项 | 值 | 后果 |
|---|---|---|
| Flash | 512 KB | 当前镜像约 191 KB（36%） |
| RAM | 192 KB | 当前 RW+ZI ≈ 184.5 KB → **93.8%**，只剩约 12 KB |
| 栈 | **4 KB** | 任何 >256 B 的局部数组都不要放栈上 |
| 堆 | **512 B** | 禁止 malloc；cJSON 走静态池（`app/cjson/cjson_port.c`） |
| 主频 | 240 MHz | 1 ms 节拍下每圈只有几十万周期，别在主循环里做长耗时计算 |

**加静态缓冲前先算 RAM。** 这是本工程最容易"改完就起不来"的地方。

---

## 3. 分层与依赖规则

```
        ┌──────────────────────── app（编排）────────────────────────┐
        │ 启动序列 · 主循环调度 · 控制台 · 上报组包 · 下行路由 · 模式  │
        └───────┬───────────────┬──────────────────┬─────────────────┘
                │               │                  │
        ┌───────▼──────┐ ┌──────▼───────┐ ┌────────▼────────┐
        │   cloud      │ │  links/g4    │ │  links/lora     │
        │ 信封/命令/上行│ │ AT+URC+建链  │ │ 帧/从机/绑定     │
        └───────┬──────┘ └──────┬───────┘ └────────┬────────┘
                │               │                  │
        ┌───────▼───────────────▼──────────────────▼────────┐
        │  service（水位评估 / 失联 / 三种缓存 / 本地告警）    │
        ├───────────────────────────────────────────────────┤
        │  model（config_store / param_save / profile / last）│
        ├───────────────────────────────────────────────────┤
        │  platform（bsp / drivers/lora / common / log）      │
        └───────────────────────────────────────────────────┘
```

规则（新代码请遵守）：

1. **依赖只向下**。service/model 不 include cloud/links；links 不 include cloud。
2. **跨层用回调/钩子**，不要在底层直接调上层。现有的钩子都叫 `xxx_set_*_cb/hook`，
   注册集中在 `main()` 里（初始化顺序敏感，见 `src/main.c` 的注释）。
3. **能在 PC 上单测的模块不要引入硬件依赖**。`cloud` / `service` / `model` 都是
   这样设计出来的；`_host_test` 里用假 4G、假 Flash、假 LoRa 顶替。

> ⚠️ **现状偏差**：`cloud/cloud_proto.c` 仍 include `links/lora/lora_business.h`，
> 而 `lora_business.c` 也 include `cloud_proto.h`（双向）。这是历史遗留，
> 重构会逐步用事件接口切断，见第 12 节。

---

## 4. 模块表

| 目录 | 关键文件 | 职责 | 不做什么 |
|---|---|---|---|
| `app/bsp` | `bsp_gpio/tim/uart` | 板级外设：4G 电源脚 PA8、USART2、定时器时基 | 不含业务 |
| `app/common` | `utils` / `storage` / `flash_map` / `ring_buf` | CRC16/CRC32、UTF-8 工具、Flash 读写、分区地址、环形缓冲 | 无状态 |
| `app/drivers/lora` | `llcc68.c` / `radio.c` / `LLCC68N32H482-board.c` / `project_config.h` | 芯片寄存器、收发中断、引脚、**编译期模式开关** | 不懂帧格式 |
| `app/links/lora` | `lora_proto.c` | 帧编解码、CRC、连续接收守护、诊断 `#lora` | 不管从机业务 |
| | `lora_business.c` | 从机表、绑定/解绑/声光状态机、下发队列、字段读写、MQTT 桥 | 不组云 JSON |
| `app/links/g4` | `g4_io.c` | USART2 传输 + AT 命令引擎（非阻塞） | 不懂 MQTT 语义 |
| | `g4_urc.c` | URC 解析、MQTT 下行队列、puback 计数 | 不建链 |
| | `g4_at.c` | 建链状态机、发布子流程、断链恢复、诊断命令 | 不管业务 |
| `app/links/serial` | `serial_proto.c` | RS232/485 收发（协议待定） | — |
| `app/cloud` | `cloud_proto.c` | 信封解析/生成、21 条命令、msgId 幂等、7 类上行构造 | 不碰 MQTT 收发 |
| | `cloud_json.c` | 极简 JSON 读写（零堆） | 不是通用库 |
| `app/model` | `config_store.c` | 9 个配置域的校验/JSON/双区保存/CRC32 | 不决定"何时存" |
| | `param_save.c` | 掉电参数页：网关 UID、出厂字段、档位 | 不上云 |
| | `cloud_profile.c` | topic / QoS / retained 的"本地档 / 文档档" | — |
| | `last_value.c` | 每台设备的最后值快照（`sen.query`/`dev.query`） | — |
| `app/service` | `level_eval.c` | 纯算法：阈值→防抖→状态→档位→模板渲染 | 无 IO |
| | `level_supervisor.c` | 取数 + 串接算法 + "该不该下发/该不该报事件" | 不真发 |
| | `sensor_offline.c` | 失联判定的纯函数 | — |
| | `event_cache.c` | 断线事件缓存（紧凑记录，保留原始 ts） | 不组 JSON |
| | `telemetry_cache.c` | 断线逐条液位上报缓存 | 不组 JSON |
| | `data_cache.c` | 下发队列（16 条）+ 最后值快照 | — |
| | `local_alarm.c` | 本地告警位 | — |
| `app/ota` | `ota_http.c` | `ota.start` → `AT+MHTTP*` 下载 → MD5 | 不搬分区 |
| | `ota_port.c` | 分区擦写、续传记录、标记页 | — |
| | `ota_health.c` | A/B 槽的机会计数与回滚判定 | 不执行搬运 |
| | `md5.c` | 增量 MD5 | — |
| `src/main.c` | | **只做模式分发**（30 行） | 不再放业务逻辑 |
| `app/core` | `app_boot` / `app_loop` / `app_state` / `app_console` / `app_downlink` / `app_uplink` / `app_events` / `app_level` / `app_modes` | 编排层：启动序列、主循环、共享状态、日志口、下行路由、上行业务、工程模式 | 不含协议/驱动细节 |
| `app/core/app_config.h` | | 编译期开关与少量编译期参数的**唯一默认值源** | 不放可调参数（进 config_store） |

---

## 5. 启动流程（`app/core/app_boot.c` → `app_boot_run()`）

顺序是**有意**的，不要随意调整：

1. `param_save_set_uid_hook` / `config_store_set_uid_hook` —— 先注册芯片 UID 来源
2. `bsp_tim1_init_1ms()` —— 一切 `delay_ms/get_tick_ms` 的前提
3. `log_init()` → 开机横幅（打印**当前编译模式**）
4. `cjson_port_init()` —— 装静态池；必须早于任何 JSON 处理
5. `reset_cause_report()` —— 上次是谁把我复位的
6. 4G 上电（PA8）—— 与 LoRa 上电**错开 500 ms**，避免浪涌把板子拉复位
7. `local_alarm_init()` → `lora_business_init()`（内含 `param_save_init` + 射频 + 协议回调）
8. 挂绑定/字段/告警/失联/遥测回调（**少挂一个，对应功能就静默失效**）
9. `g4_at_init()` → 注入 broker/clientID/topic/LWT/keepalive/watchdog
10. `ota_http_init()` → `ota_health_boot_step()`（A/B 判定）
11. 进主循环

> 判断"死在哪一步"：串口最后一行 `[BOOT] ...` 就是现场证据。

## 6. 主循环节拍（`app/core/app_loop.c` → `app_loop_run()`）

| 周期 | 做什么 | 为什么是这个周期 |
|---|---|---|
| 每 1 ms | `lora_business_poll()`：射频中断处理、帧分发、ACK 超时/重传、下行队列 | 空口时序要求 |
| 每 5 ms | `g4_at_poll()` + `ota_http_poll()` | AT 状态机要推进；OTA 下载块 200 B/片，1 s 一次会太慢 |
| 每 10 ms | `console_poll()`：`#` 命令 | 人工输入，不着急 |
| 每 1 s | `status_report()`、水位评估与联动、事件/遥测补报、失联检查、字段重发 | 业务粒度 |
| 每 30 s* | 心跳 `status.report`（retained） | *实际周期读配置 `network.statusIntervalSeconds` |

主循环末尾用 `while (get_tick_ms() == now) {}` 与 1 ms 对齐；中断仍可抢占。

---

## 7. 三条主数据流

### 7.1 LoRa 上行（传感器 → 云）

```
radio RxDone → lora_proto_poll → 帧校验 → lora_business::on_frame
  ├─ 0x02 液位上报 → 回 0x80 → data_cache_up_push（最后值）
  │                → main_on_level_report → telemetry_cache（离线时）
  │                → 事件/遥测上行（在线时）
  ├─ 0x80/0x81/0x82/0x83 → 从机状态机（ACK/声光结果）
  └─ 0x04/0x08 → 绑定 / 出厂字段应答
```

**上行只有一条出口**：`main.c` 组 JSON → `g4_at_mqtt_publish()`（topic/QoS 由
`cloud_profile` 决定）。发布成功才出队。

### 7.2 云端下行（云 → 网关）

```
4G URC "publish" → g4_io 按 payload_len 收定长二进制 → g4_urc 队列
  → lora_business::lora_mqtt_recv_cb
      ├─ 以 '{' 开头 → main_cloud_downlink_try → cloud_proto 命令分发 → cmdresp
      └─ 否则按二进制帧 → 校验 → 下发队列 → 空口（等 ACK，重传 3 次）
```

下行 payload 是**原始二进制帧**（含 `0x00/0x0A/0x0D`），不能当字符串处理。

### 7.3 本地自治（水位 → 联动）

```
每秒：level_supervisor_poll()
  有效水位(max) → confirmCount 防抖 → 状态迁移 → 档位
    → 渲染 gears[] 模板（{level} 占位符）→ 是否该下发
    → lora_business_apply_gear() → 0x03 下发 → 等 0x82/0x83
    → 成功才 mark_sent（失败下次重试）
  同时：status_change 事件（受 minChangeCm / eventMinInterval 去抖）
       全失联 fail-safe（keep / alarm / off）
```

---

## 8. 配置模型

`config_store_t` 有 **9 个域**：

`system` · `network` · `sensors[5]` · `thresholds` · `devices[5]` ·
`gears[4]` · `rules` · `ota` · `meta`

- 存储：Flash 双区 + CRC32（`storage.c` + `config_store.c`）
- 生效方式：`cfg.set` 落盘后**立即**更新内存实例；`network` 域的 broker 类字段
  在**下次启动**才真正注入 4G 层
- 出厂字段（`#set`/`#get`）**不进** `config_store`，只存 `param_save`，
  所以 `cfg.query`/`status.report` 看不到它（这是刻意设计）

## 9. 运行模式与开关

编译期三选一（`app/drivers/lora/project_config.h`）：
`MAIN_MODE_CLOUD`（默认） / `MAIN_MODE_PAIRING` / `MAIN_MODE_LORA_TEST`。
三者恰好选一个，否则 `#error`。

其它常用开关（都在对应文件顶部，`#ifndef` 保护，可在 Keil Define 覆盖）：

| 开关 | 默认 | 作用 |
|---|---|---|
| `MAIN_G4_ENABLE` | 1 | 关掉只跑 LoRa（调 4G 时用） |
| `MAIN_LORA_ENABLE` | 1 | 关掉只跑 4G |
| `MAIN_G4_POWER_AT_BOOT` | 0 | 仅控制"是否提前上电 + 浪涌错峰"；4G 最终由 `g4_at_init()` 上电 |
| `MAIN_GW_ONLINE_ENABLE` | 1 | 周期心跳 |
| `MAIN_STATUS_JSON_ENABLE` | 1 | 心跳载荷用 `status.report` JSON（否则裸 `GW_ONLINE`） |
| `MAIN_AT_CONSOLE_ENABLE` | 1 | `#at` 等手敲 AT 控制台 |
| `MAIN_OTA_AUTO_REBOOT` | 0 | 模式 1 下无 BL，校验通过后**不复位**（台架用） |

## 10. 编码与约定

- 所有 `.c/.h/.md` 都是 **UTF-8（无 BOM）**；`.ps1` 必须 **UTF-8 with BOM**（见 README）
- 日志用 `log_info("[模块] ...")`，每条一行，现场靠串口判读
- 时间：`get_tick_ms()` 是毫秒 tick（会回绕，比较一律用无符号差值）
- 云端口径：Unix 秒；LoRa 帧内时间：月/日/时/分 打包
- 字符串：禁止 `strcpy`，一律 `snprintf` + 边界检查

## 11. 构建与验证

```powershell
powershell -ExecutionPolicy Bypass -File tools\verify.ps1
```

它做三件事：Keil 构建（必须 `0 Error(s)`）→ `check_ps1.py`（BOM 检查）→
`_host_test\run.ps1`（13 套件 / 1840 断言）。

**任何改动的最低验收**：构建 0/0 + 主机测试全绿。碰到底层（射频/AT/Flash）
还要上板看串口日志。

## 12. 重构进行中（2026-10-18 起）

目标：**同一行为、更小的文件、更清楚的依赖**。当前进度：

- [x] 死代码归档（USB Host、bootloader、旧 MQTT OTA、json_proto）
- [x] 目录按层重排（drivers / links / cloud / model / service）
- [x] `main.c` 拆分完成：`main.c` **4309 → 340 行**（只剩开关 + 模式分发）
      - 启动序列 → `app_boot.c`（`app_boot_run()`）
      - 主循环 → `app_loop.c`（`app_loop_run()`）
      - 控制台 / 周期状态 / 复位原因 → `app_console.c`
      - 下行路由 → `app_downlink.c`
      - 上报（status/telemetry/event/ota.progress/补报）→ `app_uplink.c`
      - 共享状态（时钟/最近档位）→ `app_state.c`
      - 两种工程模式 → `app_modes.c`
      - 开关与节拍常量 → `app_config.h`
- [x] `g4_at.c`：`g4_at_poll()` **1384 → 221 行**，25 个状态各拆成 `g4_st_*()`，
      状态迁移仍统一走 `g4_goto_state()`；`test_state` 35/35 通过
- [x] `cloud_proto.c` 拆分（进行中）：上行构造 → `cloud_uplink.c`；`sys` → `cloud_cmd_sys.c`；`ota` → `cloud_cmd_ota.c`；`cfg`/`sen`/`dev`/`rule` 待拆（用"锚点+大括号配对"提取，不能按行号——坏注释已把多行压成一行）
- [ ] `lora_business.c` 拆分：从机表 / 绑定 / 解绑 / 声光 / 字段 / 下行
- [ ] 切断 `cloud ↔ lora_business` 双向依赖
- [ ] 两套 JSON（`cloud_json` + `cJSON`）的边界写清楚，能合则合
- [ ] 修复被编码损坏的注释（`cloud_proto.c` 293 行、`radio.c` 76 处等）

每一步都单独提交，并且必须保持：**构建 0/0、图像尺寸不变或按预期变小、
主机测试 1840/1840**。
