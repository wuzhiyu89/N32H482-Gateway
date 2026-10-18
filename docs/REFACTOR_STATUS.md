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
| `lora_business.c` | 2,950 | **2,690**（字段状态机已拆出） |

新增的编排单元：`app/core/{app_boot,app_loop,app_state,app_config,app_console,app_downlink,app_uplink,app_events,app_level,app_modes}.c`。

云协议按域拆：`cloud_uplink.c`、`cloud_cmd_{sys,cfg,rule,devices,ota}.c`，
共享接口集中在 `cloud_internal.h`（暂存缓冲 / commit / section 表 / 打印）。

LoRa 侧新增 `lora_internal.h`，`lora_field.c` 独立。

### 5. 文档与约定

- `docs/ARCHITECTURE.md`：分层、依赖规则、启动顺序、节拍表、三条数据流、约束；
- `docs/JSON.md`：两套 JSON 的边界（解析零堆 / 构造走 24KB 静态池，不做合并）；
- 编译期开关全部收敛到 `app/core/app_config.h`（`main.c` 已无 `#define MAIN_*`）；
- PowerShell 脚本 BOM 门禁（`_host_test/check_ps1.py`）。

### 6. 坏注释修复（进行中）

编码事故留下的 `U+FFFD`。已清零：`radio.c`、`llcc68-board.h`、
`cloud_cmd_rule.c`、`cloud_cmd_sys.c`、`cloud_proto.c`（文件头 120 行）。
**剩余 235 行（103 个注释块）**，分布：
`cloud_proto.c` 36 块 / `cloud_cmd_devices.c` 28 块 / `cloud_uplink.c` 30 块 / `cloud_cmd_cfg.c` 9 块。

## 二、未完成

1. 坏注释剩余 239 行（按文件逐个重写，每批构建 + 1840 条断言）；
2. `lora_business.c`（2,690 行）继续拆：从机表 / 绑定 / 解绑 / 声光 / 下发队列；
3. `config_store.c`（2,283 行）按 9 个配置域拆解析/序列化；
4. **板上冒烟测试**（唯一未做的验证）：
   - 开机横幅 → `进入主循环`；
   - `===== MQTT ONLINE! =====` + 一条 `status.report 已发`；
   - 液位发一帧，确认 `[LoRa RX]` / `[LEVEL]`；
   - 产线流程 `#bind` → `#set` → `#get`。

> 主机测试覆盖不到 `main.c` / `app/core` / `lora_business.c`（后者在测试里是替身）。
> 这几块的结构改动只有构建与人工核对兜底，所以板上冒烟是必要的一步。

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
  这不是网关缺陷，是"只发一次"策略与休眠设备的固有组合；如要改善，可把 0x07 也改成和 0x06 一样的重发节奏。