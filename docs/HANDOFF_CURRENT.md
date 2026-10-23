# 当前工作区交接（2026-10-23）

> 仓库：`C:\Users\wzy886\Desktop\N32H482-Gateway`
> 这份文档替代旧状态；A/B 双槽的设计背景仍见 `docs/HANDOFF_AB_OTA.md`。
> 工作区当前有 **36 个已修改文件 + 若干新文件，全部未提交**。

---

## 1. 我实际验证过的（不是转述）

| 项 | 结果 |
|---|---|
| `tools\verify.ps1` | **ALL CHECKS PASSED**，Keil 构建 `0 Error / 0 Warning` |
| 主机测试 | 13 个套件全绿（含 `test_ota_port` 新增 P0 用例、`test_cloud` 新增 `cj_get_bool` 用例） |
| `tools\build_bl.ps1` | **可以编**：`BL.bin = 3136 B`，0 Error |
| `FM_HAS_SLOT_B` | 双槽模式下为 `1`（`flash_map.h`），BL 的 A→B 兜底**会参与编译** |

> 更正上一会话的说法：仓库里**有** BL 构建脚本 `tools/build_bl.ps1`（armclang/armasm/armlink/fromelf），
> 不需要另找工程，直接 `powershell -File tools\build_bl.ps1` 即可出 `MDK-ARM\bin\BL.bin`。

---

## 2. P0 / P1 逐条核实结果

| 项 | 位置 | 核实 |
|---|---|---|
| P0 擦标记页保住启动记录 | `ota_port.c`：`meta_page_erase_keep_boot()`；`mark_valid` / `mark_invalid` / `resume_save` 三处均改走它 | ✅ 已实现，逻辑正确（擦前 load、擦后 save；写回失败靠 BL 兜底） |
| P0 BL 无记录时 A→B 兜底 | `bootloader.c`：`bl_jump_app()` 先 A 后 B；`bl_run()` 不再就地卡死 | ✅ 已实现，`FM_HAS_SLOT_B=1` 时生效；BL 编译通过 |
| P1-1 `config_store_save` 栈上大对象 | `config_store.c`：`static config_store_t chk` | ✅ 已改（2640 B → .bss） |
| P1-2 第一台 IDLE 声光挡住后面 | `lora_downlink.c`：`lora_try_send_downlink(i) == DOWNLINK_SENT` 才 break | ✅ 已改 |
| P1-3 `cj_get_bool` 不认数字 | `cloud_json.c`：`CJ_NUM` 按“非 0 即真” | ✅ 已改 |
| P1-4 AT 缓冲不够 | `g4_at.c`：`G4_TX_CMD_MAX 288` + `_Static_assert` | ✅ 已改 |
| P1-5 双区载入优先 A | `config_store.c`：peek 两区头，`updated_at` → `version` 取新 | ✅ 已改，但只是“缓解”：时间戳相同/都没对时时仍回退 A 优先 |
| P1-6 0x82 结果回来清错缓存 | `data_cache.c/.h`：`data_cache_down_del_if_match()`，0x80 确认/重传耗尽两处也改成条件删 | ✅ 已改，语义正确（载荷逐字节匹配才删） |

---

## 3. 审计中发现、需要你确认的问题

### ① 版本号冲突（必须先定）

- `tools/A_B_bin_creat.py` 当前写死：`VERSION = "1.0.15"`、`BUILD_SLOT = "A"`；
- `MDK-ARM\bin\CDC_A_1.0.15.bin` 已存在（**16:35 生成，是修复前的构建**）；
- 如果再出 1.0.15，就会出现两个内容不同的同版本固件。
  **建议：出 1.0.16**，槽位按板子当前 active 槽的“另一槽”来。

### ② BL 还没烧

P0 的 BL 那一半只在源码/新 `BL.bin` 里，**板子上跑的还是 10-21 的旧 BL**。
不重烧的话，P0 只有 App 侧生效，仍剩“擦完到写回之间掉电”的微秒级窗口。

### ③ 默认显示方向被改了（不在 P1 清单里）

`config_store_factory_default()` 里 4 个档位的 `led_show_mode` 从 `0`（横屏）改成了 `1`（竖屏），
注释也改成了“默认竖屏”。`test_cfg.c` 同步改了，所以测试是绿的——但这是**产品行为变化**，
请确认这是有意的（如果是，建议单独提交并写进变更说明）。

### ④ 工作区 36 个文件未提交

其中混着：P0/P1 修复、`src/main.c` 大改（-252 行）、新增 `power_mgr.c/.h`（只提供接口、未接 main）、
`app_events.c`、`sensor_offline.*`、`lora_business/bind`、`utils`、`cloud_cmd_*` 等。
**建议按主题拆成 3~5 个提交**（P0、P1、竖屏默认值、main/power 重构、文档/工具），不要一个 `git add -A`。

---

## 4. 板子状态（本次无法读取）

- `COM14` 不存在；`COM24`(CH340)、`COM12`(J-Link CDC) 都返回“访问被拒绝”（被其它程序占用）。
- 上次我这边的确定状态（10-21）：A 槽 1.0.7、`active=A`、`pending=无`。
- 上一会话称后来烧到了 **B 槽、active=B**，但未留下串口证据。
- **接手第一件事**：关掉占用串口的程序，用 COM24/COM12 读一次 `status.report` 的 `fwVersion` 与 `slot`。

---

## 5. 常用命令

```powershell
# 全量验证（先关闭 uVision；它会改坏 IROM）
powershell -ExecutionPolicy Bypass -File tools\verify.ps1

# 出指定槽/版本的固件
python tools\A_B_bin_creat.py      # 改文件头 VERSION / BUILD_SLOT
# 或
powershell -ExecutionPolicy Bypass -File tools\build_ab.ps1 -Version 1.0.16

# BootLoader
powershell -ExecutionPolicy Bypass -File tools\build_bl.ps1

# J-Link（脚本：r / loadbin / r / g / qc）
JLink.exe -device N32H482RE -if SWD -speed 4000 -autoconnect 1 -CommanderScript xxx.jlink
```

槽基址：A `0x08004000`、B `0x08038000`，各 `0x34000`；标记页 `0x0806C000`，启动记录从 `+512` 起，32 B/条。

---

## 6. 需要你拍板

1. **版本与槽**：出 1.0.16 吗？烧 A 还是 B（取决于当前 active 槽）？
2. **BL 是否现在重烧**（P0 才完整）？
3. **P1-5 是否根治**（给配置头加单调序号会改 Flash 格式，现有 broker/账号/密码会读不出、回出厂）？
4. **默认竖屏**是否为预期改动？
5. 工作区是否按上面的拆分提交（以及是否推送；当前网络 push 仍超时）？
