# N32H482 4G+LoRa 网关 — 交接文档

> 生成时间：2026-10-27
> 仓库：`C:\Users\wzy886\Desktop\N32H482-Gateway`
> 分支：`main`，远端 `git@github.com:wuzhiyu89/N32H482-Gateway.git`（SSH，已同步）

---

## 0. 先看这里

- 这是**裸机**工程（无 RTOS），Keil MDK + ARMCLANG，1ms tick。
- 改动 C 代码后**必须真的编译**：`powershell -ExecutionPolicy Bypass -File tools\verify.ps1`，要求日志出现 `0 Error(s)` / `build OK (0 Error / 0 Warning)`，且 `ALL CHECKS PASSED`。
- 板子当前**未烧录**最近这批改动；所有新功能只过了编译 + 主机测试。
- 不要提交构建产物（`Objects/`、`*.bin`、`*.axf`、`*.map` 等）。

---

## 1. 常用工具 / 命令

| 用途 | 命令 / 路径 |
|---|---|
| Keil | `D:\Keil_v5\UV4\UV4.exe`（GUI 子系统，必须 `Start-Process -Wait`） |
| J-Link | `C:\Program Files\SEGGER\JLink_V974\JLink.exe` |
| Python | `E:\python\Python312\python.exe`（有 pyserial） |
| PowerShell | 5.1，**本机没有 pwsh 7** |
| 完整验证 | `powershell -ExecutionPolicy Bypass -File tools\verify.ps1` |
| 主机测试 | `powershell -ExecutionPolicy Bypass -File _host_test\run.ps1` |
| A/B 构建 | `powershell -ExecutionPolicy Bypass -File tools\build_ab.ps1 -Version 1.0.x` |

### 构建注意

- **先关闭 uVision**；uVision 会把 `CDC.uvprojx` 的 IROM 改回 `0x08000000/0x30000`，发现后执行：
  ```
  git checkout -- MDK-ARM/CDC.uvprojx
  ```
  正确值：IROM `0x08004000 / 0x34000`，VECT 在 `0x08004000`。
- A/B 版本约定：**末位奇数 → B 版，偶数 → A 版**；平台按 `status.slot` 下发另一槽的镜像。
- 串口常见：`COM12` JLink CDC、`COM14` CP210x、`COM24` CH340（插拔会变，先列端口）。

### 烧录

J-Link 脚本示例（路径分隔符按字面写）：

```text
r
loadbin "C:\...\MDK-ARM\bin\CDC_A_1.0.0.bin", 0x08004000
r
g
qc
```

- BL：`0x08000000`（16KB）
- A：`0x08004000`（208KB）
- B：`0x08038000`（208KB）
- 全片擦除后再烧 BL + App；配置区擦除会恢复出厂。

---

## 2. 项目结构 / 关键机制

### Flash 布局（模式 2）

| 区域 | 地址 | 大小 |
|---|---|---|
| Bootloader | `0x08000000` | 16KB |
| Slot A | `0x08004000` | 0x34000 |
| Slot B | `0x08038000` | 0x34000 |
| Meta（启动记录） | `0x0806C000` | 8KB |
| Param | `0x0806E000` | — |
| Config A | `0x08070000` | 8KB |
| Config B | `0x08072000` | 8KB |
| Event | `0x08074000` | — |

- 启动记录在 `meta + 512`，32B：`magic='BOOT'`、`seq`、`active`、`pending`、`confirmed`、`tries`、`crc`；**每次保存 seq 必须 +1**（BL 和 App 都一样）。
- BL：有 pending 且 tries>0 → 递减并跳 pending；tries=0 → 清 pending 跳 active；向量非法 → 回退；无记录 → 旧 OTAV 兼容路径。
- 配置双 Bank：头 32B（magic/version/crc32/updated_at/json_len/seq/rsv/hdr_crc），必须 8 字节对齐（`FLASH_ProgramdoubleWord`）；保存先 B 后 A，`seq = max(A,B)+1`；加载取 seq 大者。
- `CFG_JSON_MAX = 6144`；`CFG_GEAR_COUNT = 4`（4~7 见 TODO）。
- `config_store_t` 目前 4096B，有 live/staging/reset/校验多份实例，RAM 很紧（ZI ≈ 188KB / 192KB）。

### 云端协议（摘要）

- Topic：`rt/gw/{gwId}/cmd`（QoS1）、`cmdresp`、`telemetry`（QoS1）、`event`、`status`（retained）、`lwt`。
- `msgId` 去重：最近 4 个、60 秒窗口；平台重试请换新 msgId。
- 遥测周期 = `network.statusIntervalSeconds`（10~600，默认 30），首条立即发，失败 5s 重试。
- `effective.levelCm` = 在线且 enabled 传感器的平均值；`status`/`gear` 按最大值。
- 应答 > ~1024B 返回 `1011`；下行 JSON 上限约 6KB。

### 声光 0x03 载荷

`[0]` 电源（gear0=0 关屏）、`[1]` 闪光模式 0~3、`[2]` 音量、`[3]` LED 模式位、`[4]` 文本1长度 + 内容、`[5+N]` 文本2、`[6+N+M]` TTS 长度 + 内容。
- `flashMode`：0 关 / 1 常亮 / 2 慢闪 / 3 快闪；
- TTS 空 = 不播报；
- 文本按 UTF-8 字节：LED 每段 ≤48、TTS ≤88、合计 ≤199。

### 0x09 在线查询（新增）

- `func=0x09`，无 data；声光回 `0x80`（无 data）= 在线。
- 触发：有新液位样本、不需要发 0x03、非 manual、非 fail-safe → 全部 enabled 声光；
- 同一秒多台上报合并一轮；同一台等 ACK 不重复排队；待发 0x03 优先；
- 帧级 3 次 × 5 秒；超时 → `offline` + `event.device(offline)`；ACK → 在线，必要时补 `online`。

### OTA

- 平台按 `status.slot` 选另一槽镜像；`ota.start` 带 version/url/size/md5；仅 HTTP；
- 状态 `idle/downloading/verifying/installing/failed`，没有 `done`；成功看 `status.report.fwVersion`；
- 新固件试运行 8 次机会，起不来自动回退；
- `MAIN_OTA_AUTO_REBOOT=1`。

---

## 3. 最近已完成（提交链）

```
bc22a17 chore: replace NUL char literals so git treats app_level.c as text
9d18ab2 docs: document failsafe off/alarm/keep and Offline/null status
4d745ea fix(failsafe): keep fault-screen ACKs out of normal action events
55b5f3c feat(failsafe): off fault screen, Offline/null status, keep/alarm/off state machine
737ed85 fix(backfill): throttle 1 msg/s and hash-based msgId
f95910d fix(level): force one gear refresh when returning manual->auto
4c216f5 docs: sync manual with flashMode, masked creds, repeated offline events
c229f60 feat(lora): send 0x09 online probe to alarms when level unchanged
05984da feat(cfg): flashMode rename, username mask, boot->auto; share cfg scratch
b798dc0 feat(level): repeat sensor-offline event every 3x report period
```

对应功能：

1. **离线事件重复**：每台液位按自己的 3×reportPeriod 在 3×/6×/9×… 重复 `event.device(offline)`；恢复只补一次 `online`。
2. **flashMode**：`gears[].soundMode → flashMode`，**不兼容旧字段**；旧配置升级后 flashMode=0，需平台重下 `rule.set`。
3. **username 掩码**：`cfg.query` 的 username/password 都返回 `"*****"`；含掩码配置跳过 `meta.crc32` 自校验，平台用 `configVersion` 比对。
4. **上电强制 auto**：只改 RAM，不写 Flash、不抬 configVersion。
5. **0x09**：见上。
6. **manual→auto**：强制重发一次当前档位（`level_supervisor_force_dispatch()`），发送成功后才清标志。
7. **fail-safe**：
   - keep：进入不动；manual 结束仍失联 → 重发当前自动档位；恢复强制补发；
   - alarm：进入发 gear 3 一次，不周期重发；manual 结束补发；恢复强制补发；
   - off：专用故障画面（屏亮、不闪、音量 0、竖屏、两行居中、黄/黄、`设备故障`/`勿依赖提示`、无 TTS），每 `max(reportPeriod)×3` 重发；manual 期间不覆盖，结束后立即补发；恢复立即补发正常档位；
   - off 期间 `currentStatus="Offline"`、`currentGear=null`、`effective.status="Offline"`、`effective.gear=null`、`event.action.gear=null`、`trigger="failsafe"`，事件只在进入时一次。
8. **补报**：遥测 1 条/秒、事件 1 页/秒；补报 msgId 改为 `FNV1a(uid,ts,seq)` 8 位 hex，不再与在线 `'m'+seq` 共用。

---

## 4. 明确的设计决定（不要随意改）

- 遥测 QoS=1；心跳周期严格等于 `statusIntervalSeconds`；`effective.levelCm` 取平均，`status/gear` 取最大。
- 声光 0x01 的水位阈值固定 50mm（甲方确认）。
- 默认 `ledShowMode=1`（竖屏）。
- `manualTimeoutSeconds=0` = 不自动回 auto；上电强制回 auto。
- `soundMode` 不做兼容。
- `gears[]` 当前 4 档；4~7 预留，等 OTA，原因见 TODO。
- off 状态用 `"Offline"`（不是 null）；gear 用 null。
- 平台可按 `uid+ts` 去重遥测。

---

## 5. 已知风险 / TODO

### 需要上板验证

1. 0x09：发送、ACK 在线、超时离线、与 0x03 互斥；
2. off 故障画面、重发节奏、manual 结束补发、恢复补发；
3. manual→auto 强制刷屏（液位不变 → dev.ctrl 改文本 → 切回 auto → 屏应恢复）；
4. 离线事件重复（3×/6×/9×）。

### 代码风险

- 故障画面在等 ACK 时若被正常 0x03 覆盖，`alarm_fault_display` 可能吞掉一次普通 `event.action`。
- `event.action` failsafe 的 `results:[]` 且 4G 断线不入事件缓存；云端若要每台结果需再定。
- `currentStatus` 是新增字段，云端必须接受 `"Offline"`。
- `app_level` 状态机（manual 结束/恢复补发）没有 host 测试覆盖。
- `app_level.c` 的 NUL 字面量已修（`bc22a17`），以后不要再用实际 NUL 字符。

### gear 4~7（暂缓）

扩到 8 档时发现三重限制：
- `config_store_t=4096B` × 5 份实例 → RAM 只剩 ~1.5KB；
- 满配 JSON 超过 6144；
- cJSON 静态池也会耗尽（`cfg_serialize` 返回 0）。

已尝试缩缓存（事件 256 / 遥测 128 / JSON 8000）仍然不够，已回退。后续方案：
1. targets 从 27B UID 字符串改成设备索引/13B 二进制，省 ~4KB；
2. 合并 config 实例；
3. 扩 cJSON 池 + JSON 缓冲；
4. 或 4~7 只做精简占位。

### 工作区未提交文件（非本轮改动，未动）

`MDK-ARM/CDC.code-workspace`、`tools/A_B_bin_creat.py`、`docs/HANDOFF_AB_OTA.md`、`_host_test/provision_gen.c`、`tools/provision_devices.json`、`tools/provision_flash.ps1`、`.workbuddy/`、`tools/__pycache__/`。

---

## 6. 关键文件地图

| 文件 | 作用 |
|---|---|
| `app/core/app_loop.c` | 主循环；联动下发、0x09 触发、fail-safe 调用 |
| `app/core/app_level.c` | fail-safe 状态机、manual 回归、0x03/故障画面投递 |
| `app/core/app_uplink.c` | 遥测/补报/状态、0x09 记录、msgId 哈希、节流 |
| `app/core/app_events.c` | event.device/action/status_change、设备在线离线事件 |
| `app/service/level_supervisor.c` | 档位评估、manual 抑制、force dispatch |
| `app/service/sensor_offline.c` | 离线判定/重复事件 |
| `app/links/lora/lora_business.c` | 从机表、0x02/ACK、0x09 特殊处理 |
| `app/links/lora/lora_downlink.c` | 0x03/0x09 重传、超时、离线标记 |
| `app/links/lora/lora_alarm.c` | 0x03 三段式应答、故障画面 payload |
| `app/model/config_*.c` | 配置解析/序列化/校验/双 Bank |
| `app/cloud/cloud_cmd_*.c` | cfg/rule/dev/sen 命令 |
| `tools/verify.ps1` | 构建 + 主机测试 |
| `tools/build_ab.ps1` | A/B 双镜像构建 |
| `MDK-ARM/网关使用手册.md` | 给云端的对外手册 |

---

## 7. 建议的下一步顺序

1. 板子回来先烧 BL + 最新 A，验证基本启动；
2. 配 1 台液位 + 1 台声光，验证 0x09（拔声光电源 → 3×5s 后 offline 事件）；
3. 验证 off：拔液位 → 故障画面 → 每 3× 周期重发 → 恢复补发；
4. 验证 manual→auto 刷屏；
5. 再讨论 gear 4~7 的 OTA 方案。
