# 重构状态（滚动更新）

> 目标：**行为不变**的前提下提高可读性与模块化。
> 每次提交都要求：Keil 构建 `0 Error / 0 Warning` + 主机测试 `1840/1840`。
> 一键验证：`powershell -ExecutionPolicy Bypass -File tools\verify.ps1`

## 一、已完成

### 1. 仓库与基线

- 仓库原本**无法从干净 clone 构建**：`.gitignore` 的 `log/` 把 `app/log/log.c` 一起忽略掉；
  两个 Keil target 的宏不一致（`LOG_ENABLE`）。
- 现已修正，并补上主机测试（13 套件 / 1840 条断言）与 OTA 靶子文件；
  `_host_test/run.ps1` 的双 BOM（导致全绿仍 exit 1）也已修掉。
- 新增 `.gitattributes`（文本 LF、二进制不动）与 `tools/verify.ps1`。

### 2. 清死代码（A/B）

移入 `archive/legacy/`（不参与构建，见 `archive/README.md`）：

- USB Host CDC demo（`usbh_*` / `usbhs_*`，从未进 Keil 工程）
- `main_bl.c` + `bootloader.*`（模式 2 才用；当前无 BL）
- 旧 MQTT 分包 OTA（`ota.c`，被 `ota_http.c` 取代）
- `json_proto.*`、LLCC68 示例、0 字节的 `app/common/g4_at.c`
- 需要的类型单独抽出：`app/ota/ota_meta.h`、`app/ota/ota_mark.h`

### 3. 目录按层重排

`app/{bsp,common,drivers/lora,links/{g4,lora,serial},cloud,model,service,ota,core}`，
根目录 `src/` 只放入口与中断向量。Keil 分组由脚本从目录树重新生成。

### 4. 巨型文件拆分

| 文件 | 之前 | 现在 |
|---|---:|---:|
| `src/main.c` | 4,309 | **268**（只剩模式分发） |
| `g4_at_poll()` 单函数 | 1,384 | **221**（25 个 `g4_st_*`） |
| `cloud_proto.c` | 3,576 | **1,049** |
| `app_uplink.c` | 1,491 | **830** |
| `lora_business.c` | 3,012 | **634**（拆出 5 个文件，见下） |
| `config_store.c` | 2,283 | **1,062**（解析 / 序列化拆出，见下） |

新增的编排单元：`app/core/{app_boot,app_loop,app_state,app_config,app_console,app_downlink,app_uplink,app_events,app_level,app_modes}.c`。

云协议按域拆：`cloud_uplink.c`、`cloud_cmd_{sys,cfg,rule,devices,ota}.c`，
共享接口集中在 `cloud_internal.h`（暂存缓冲 / commit / section 表 / 打印）。

LoRa 侧 2026-10-18 继续拆分（`lora_internal.h` 集中内部接口）：

| 文件 | 职责 |
|---|---|
| `lora_slave.c` | 从机表：hex 工具、装载、增删、在线状态 |
| `lora_bind.c` | 0x04 绑定窗口状态机 + 写配置 |
| `lora_unbind.c` | 0x05 解绑两阶段状态机 |
| `lora_alarm.c` | 0x03 执行结果状态机 + 档位联动下发 |
| `lora_downlink.c` | 下发队列 / ACK 重传 / 两步应答 / MQTT 下行 |
| `lora_business.c` | 帧分发、0x02 上报、失联检测、init / poll |

`lora_field.c` 之前已拆出（出厂字段）。

配置存储 2026-10-18 按"解析 / 序列化"拆开（内部接口在 `config_internal.h`）：

| 文件 | 职责 |
|---|---|
| `config_store.c` | 校验、出厂默认、双区 Flash 读/写、版本/恢复出厂、UID 工具 |
| `config_parse.c` | 取值助手 + 9 个域的解析 + 全局/逐域事务合并 |
| `config_serialize.c` | 9 个域 -> JSON、CRC 复算、诊断打印 |

解析与序列化共用同一份枚举名表（`config_internal.h` 里 extern），防止两边漂。

### 5. 文档与约定

- `docs/ARCHITECTURE.md`：分层、依赖规则、启动顺序、节拍表、三条数据流、约束；
- `docs/JSON.md`：两套 JSON 的边界（解析零堆 / 构造走 24KB 静态池，不做合并）；
- 编译期开关全部收敛到 `app/core/app_config.h`（`main.c` 已无 `#define MAIN_*`）；
- PowerShell 脚本 BOM 门禁（`_host_test/check_ps1.py`）。

### 6. 坏注释修复（完成）

编码事故留下的 `U+FFFD`，**`app/` 下已清零**（`tools/check_comments.py` 全部 0 块）：

- 早期：`radio.c`、`llcc68-board.h`、`cloud_cmd_rule.c`、`cloud_cmd_sys.c`；
- 本轮按文件逐个整块重写（每块 `(start,end)` 由脚本给出，代码行逐字节不变）：
  `cloud_uplink.c` 30 块 → 0、`cloud_cmd_devices.c` 28 块 → 0、
  `cloud_proto.c` 36 块 → 0（`cloud_cmd_cfg.c` 的 9 块已在上一提交清零）；
- 额外：`cloud_cmd_devices.c` 里两处 **log 格式串**中的 `U+FFFD` 已修
  （`dev.ctrl` 的两条日志），`cloud_internal.h` 注释里的 `app/cloud/*.c` 改成
  "app/cloud 目录下的 .c"，消掉 `-Wcomment`。

唯一剩下的 `U+FFFD` 在 `firmware/n32h47x_48x_std_periph_driver/src/n32h47x_48x_pga.c`
（厂商代码，按约定不改）。

## 二、未完成

（无 —— 代码侧与板上冒烟都已完成。见下面第五节本轮结果。）

> 主机测试覆盖不到 `main.c` / `app/core` / `app/links/lora`（`lora_business` 在测试里是替身）。
> 这几块的结构改动只有构建与人工核对兜底，所以板上冒烟是必要的一步。
> 本次 LoRa 拆分的额外核对：用脚本把拆分前 `lora_business.c` 的 **66 个函数逐个
> 与拆分后比对**（只允许改 `static` 与跨文件改名），结果 0 处差异；
> 只有 `on_frame` / `handle_ack_from_device` 两处是**有意**把"直接读 `g_bind`/`g_unbind`"
> 换成 `lora_bind_*` / `lora_unbind_*` 访问器。config_store 拆分同样比对 **62 个函数，0 差异**。

## 三、坏注释修复的做法（踩坑后固化）

编码事故把**多行注释压成了同一行**，所以**不能按单行或关键词替换** —— 很容易破坏
注释块的 `/* */` 边界，编译期直接报 `U+FFFD` 出现在注释外。

正确步骤：

1. 用脚本把目标文件切成「注释块 / 代码行」两类；
2. **只重写注释块**，代码行原样保留（逐字节比对）；
3. 每个文件处理完立即构建 + 跑 1840 条断言；
4. 一次只提交一个文件，出问题 `git checkout -- <file>` 即时回滚。
## 四、硬件验证记录（2026-10-18，J-Link + COM14）

烧录方式：JLink.exe -device N32H482RE -if SWD -speed 4000 + loadbin MDK-ARM\bin\CDC.bin, 0x08000000
（Keil 的 -f 会被正在运行的 uVision 占用，报 Target DLL has been cancelled）。

已验证通过：

| 项 | 证据 |
|---|---|
| 启动序列 | 横幅 模式=云端联调(MAIN_MODE_CLOUD) -> 各 [BOOT] -> 进入主循环 |
| LoRa | LLCC68IoInit init sucess、SX1261、网关 UID=113608、LoRa RX=在收 补开=0 |
| 4G 驻网 | IP=10.31.136.222、CEREG=1、CSQ=31(-51 dBm)、4G=ONLINE |
| MQTT | 上线并有心跳：status.report 247 B，broker 回 puback（puback=3） |
| 下行/命令/应答 | 串口注入 #json {"cmd":"sys.query_status",...} -> code=0、应答 285 B -> 发布到 .../cmdresp |
| 失实日志修复 | [BOOT] 4G 暂不提前上电(...), 稍后由 g4_at_init() 上电（旧文案是"不给 4G 上电"） |

未验证：

- 真实液位/声光模块的 [LoRa RX]（需要一台从机在发）；
- 产线流程 #bind / #set / #get（需要从机配合）；
- 云端真实下发（需要有平台侧发 t/gw/<uid>/cmd 的权限）。
### 追加：带真实液位从机的端到端验证（2026-10-18）

从机 UID 21360611303239363131017C42，网关 113608。

| 场景 | 结果 |
|---|---|
| 绑定 #bind sensor 60 + 从机发 0x04 | 进入 WAIT_DEV_ACK -> 回 0x80 -> 在线从机=1/1 |
| 传感器上报 0x02 | 	elemetry.report 241 B：levelCm 0 / rawMm 0 / battery 49% / rssi -22 |
| status.report | sensorsOnline:1, sensorsTotal:1, configVersion:4 |
| 写字段 #set TEST000001 | 3 条 0x06 内收到 0x80 -> 写入成功 |
| 读字段 #get（设备刚上报后） | 0x07 -> 设备回 0x08 "TEST000001" -> 读取成功 |
| #get（设备休眠时） | 12s 内无 0x08 -> 如实报失败 |

**运维结论（重要）**：液位从机是电池设备，只在**自己上报后的短暂窗口**内监听下行。
- #set 失败率低：0x06 会每 5 s 重发、最多 3 条；
- #get 只发一次 0x07，**必须在设备刚上报后立刻执行**，否则 12 s 超时。
  这不是网关缺陷，是"只发一次"策略与休眠设备的固有组合。
  **【用户已决定保持现状：不把 0x07 改成重发】**，后续重构不要顺手改这一行为。

## 五、重构后回归（2026-10-18 晚，HEAD=0ad89b9，J-Link + COM14）

烧录 `MDK-ARM\bin\CDC.bin`（`ALL CHECKS PASSED` 的同一份构建），J-Link `Program & Verify O.K.`。

| 场景 | 结果 |
|---|---|
| 4G / MQTT | `CEREG=1`、IP 已分配、`status.report` 连续拿到 `puback` |
| `#status` | status.report 248 B 已发 |
| `#json cfg.query` | code 0，应答 1837 B，9 域 + `meta.crc32=3A20AF04`，cmdresp 有 puback |
| `#json rule.query` | code 0，1050 B（gears + rules） |
| `#json sys.query_status` | code 0，286 B，puback |
| `#bind sensor 60`（同 UID 重绑） | 走 `config_store_rebind_device` 分支：`重新绑定(已在从机表, 不新增)`，`sensorsTotal` 仍为 1（**不新增条目**），event=bound |
| 从机 0x02 | `[LoRa RX] func=0x02` → `[LEVEL]` → `telemetry.report 241 B`（level 0 / bat 49% / rssi -24） |
| `#set TEST000001` | 0x06 第 3 条收到 0x80 → 写入成功 |
| `#get`（上报后立即） | 0x07 → 0x08 `"TEST000001"` → 读取成功 |
| 期间 4G 异常 | 一次 `+CME ERROR: 601`（MQTT 会话掉），既有快修逻辑自动恢复，puback 继续递增 |

### 已知的"诊断误报"（不是缺陷，勿按它改代码）

`#lora` 的"频率【回读 0x0880】"**不可信**：同一块板两次读到的值不同
（`03 00 00 57` / `00 00 00 57`），而空口实际工作正常（`RX_DONE=20`、
芯片 GetStats 收包 20、rssi -22~-27）。所以那条
`!! SET_RFFREQUENCY 没有生效` 是**误报**；判断射频好坏请看
`RX_DONE / PREAMBLE_DETECTED / 芯片 GetStats` 这三组累计值。
（`SYNCWORD_VALID` 恒 0 属正常：IRQ 掩码里没开它。）