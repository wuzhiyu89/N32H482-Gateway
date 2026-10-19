# 完整 OTA（A/B 双 Slot + Bootloader）测试手册

> 为什么需要这份：出厂是 **模式 1**（`FM_MAP_MODE = FM_MAP_SINGLE_RESERVED`），
> 板上没有 bootloader，`ota.start` 只能做到"下载 + MD5 校验 + 写 OTAV 标记"，
> 校验通过后停在 `installing 100%`，**不会真的换固件**。
> 想做"下载 → 搬运 → 运行新固件"的完整闭环，必须切到 **模式 2** 并先烧 BL。

## 1. 分区布局（模式 2，`flash_map.h`）

| 区域 | 地址 | 大小 |
|---|---|---|
| BootLoader | `0x08000000` | 16 KB |
| App-A（运行区） | `0x08004000` | 208 KB |
| App-B（OTA 暂存 / A/B 镜像） | `0x08038000` | 208 KB |
| OTA/启动标记页 | `0x0806C000` | 8 KB |
| 掉电参数页 | `0x0806E000` | 8 KB |
| 配置 A/B | `0x08070000` / `0x08072000` | 各 8 KB |
| 事件日志 | `0x08074000` | 48 KB |

## 2. 构建

### 2.1 Bootloader

```powershell
powershell -ExecutionPolicy Bypass -File tools\build_bl.ps1
# -> MDK-ARM\bin\BL.bin  (约 2.5 KB, 链接在 0x08000000)
```

BL 源码在 `bootloader/`（`bootloader.c/h` + `main_bl.c` + `BL.sct`）。
它用 `FM_MAP_MODE=2` 编译，地址全部来自 `flash_map.h`，不写死。

### 2.2 App（模式 2）

Keil 工程 `MDK-ARM\CDC.uvprojx` 已经带上：

- `Define`: `..., FM_MAP_MODE=2, VECT_TAB_OFFSET=0x4000, MAIN_OTA_AUTO_REBOOT=1`
- `IROM`: `0x08004000`, 大小 `0x34000`

`VECT_TAB_OFFSET=0x4000` 是必须的：`SystemInit()` 会把 `SCB->VTOR` 设成
`FLASH_BASE | VECT_TAB_OFFSET`，否则 App 在 `0x08004000` 跑、中断却跳到 BL 的向量表。

### 2.3 OTA 载荷（版本号必须和当前不同）

在 `Define` 末尾临时加 `MAIN_FW_VERSION=\"1.0.3\"`（**必须带反斜杠转义引号**，
否则 ARMCLANG 会把它当成浮点数 `1.0.3` 报错），重新构建，把
`MDK-ARM\bin\CDC.bin` 另存为载荷；构建完把这段宏删掉。
载荷必须是**同一种模式**（也链接在 `0x08004000`），BL 只做整段复制。

## 3. 烧录

```text
r
erase
loadbin "...\MDK-ARM\bin\BL.bin",  0x08000000
loadbin "...\MDK-ARM\bin\CDC.bin", 0x08004000
r
g
```

`erase` 会清掉配置/参数/标记页 —— 首次上电回到出厂默认（`configVersion=0`、
从机表为空），需要重新绑定从机。

## 4. 跑一次完整 OTA

1. 把载荷放到 HTTP 服务器（**只支持 http://，HTTPS 在 ML307R 上不可靠**）；
2. 平台向 `rt/gw/<gwId>/cmd` 发：

```json
{"cmd":"ota.start","msgId":"ota-1","ts":0,
 "data":{"version":"1.0.3","url":"http://<host>/<file>.bin",
         "size":192848,"md5":"<32 位小写 hex>"}}
```

3. 串口应看到：`downloading` → `verifying` → `OTAV 标记已写` → `installing 100%`
   → 3 s 后自动复位 → BL 打印搬运 → 跳 App；
4. 重启后 `status.report.fwVersion` 变成 `1.0.3` 即成功（`ota.query` 会回到 idle）。

## 5. 回退到模式 1（出厂形态）

1. 删掉 `Define` 里的 `FM_MAP_MODE=2, VECT_TAB_OFFSET=0x4000, MAIN_OTA_AUTO_REBOOT=1`；
2. `IROM` 改回 `0x08000000` / `0x30000`；
3. **整片擦除**后再烧 App 到 `0x08000000` —— 如果 BL 还留在 `0x08000000`，
   它会按模式 2 跳到 `0x08004000`（空的）直接 HardFault。

## 6. 风险与兜底

- BL 写坏/搬运中断：J-Link 重新 `erase` + 烧 BL/App 即可，不会永久变砖；
- 模式切换会换掉 param/配置页地址 → 配置回出厂、从机需重绑；
- `ota_health` 的 A/B 健康计数在新固件连续几次未确认时会触发回滚，
  第一次跑要留意它的日志。
