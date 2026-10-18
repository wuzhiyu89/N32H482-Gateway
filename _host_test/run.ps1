# Host-side tests for the gateway firmware (no board / no ARM toolchain needed, only gcc).
#
# ===========================================================================
# !! 本文件必须保存为 "UTF-8 with BOM" -- 不要去掉 BOM !!
#
#   原因: Windows PowerShell 5.1 对【无 BOM】的脚本按 ANSI/GBK 解码, 而本文件
#   里有中文注释. 全角标点的 UTF-8 末字节 (0x82/0x8C/0x81/0x9A ...) 全落在
#   GBK 前导字节区间 0x81-0xFE 内, 解码器会把紧跟的 CR/LF 当成它的第二个字节
#   吃掉, 于是下一行整行被并进注释.
#
#   实测 (2026-10-02, 加 test_md5 套件时踩到): 一条以全角句号结尾的注释把紧跟
#   的 name='test_md5' 整行吃了, 套件名字变成空, 但测试本身照跑, 日志里只显示
#   "=== build & run  ===", 极难注意到.
#
#   自检(应输出 危险行数 = 0): python _host_test/check_ps1.py
# ===========================================================================
#
# Usage (from the demo directory):
#     powershell -ExecutionPolicy Bypass -File _host_test\run.ps1
#
# Suites:
#   test_rx    - byte-level framing of g4_io.c + downlink queue of g4_urc.c
#                (0x00/0x0A/0x0D/0x3E inside payload, split publish, truncated
#                 payload -> timeout + resync, the '>' prompt which is "\r\n> "
#                 with NO trailing newline, result delivery on the next tick)
#   test_state - g4_at.c state machine on a virtual millisecond clock with a fake
#                ML307R modem: full attach to MQTT ONLINE, heartbeat + puback,
#                binary cloud downlink, link loss -> 4G_RST power cycle -> reattach,
#                and dead module -> stop knocking AT -> 4G_RST.
#   test_cloud - cloud_json.c / cloud_proto.c / cloud_profile.c / config_store.c
#                / sensor_offline.c (P1-0/P1-1/P1-2 + P2-4 + P2-5):
#                JSON paths + arrays + escapes + \uXXXX, writer + overflow guard,
#                command envelope parse (msgId is a STRING <=16), 21-command
#                dispatch table, error codes incl. 1008 auth, the local/doc
#                profile topic + QoS + retained mapping, the cfg.set/cfg.query
#                handlers (1002 version conflict, applied/rejected, network refused,
#                cfg.query returns the whole config object), sys.factory_reset
#                (keepNetwork semantics, configVersion -> 0, arrays cleared, and
#                Flash failure -> 1007 never 0), and the liquid-level sensor
#                offline rule (3 x reportPeriodSeconds, tick wrap-around,
#                transition de-duplication).
#                Plus P1-6 msgId idempotency: a repeated msgId must NOT re-run the
#                command (proved by configVersion NOT advancing a second time), the
#                previous response is replayed byte-for-byte, an empty msgId is not
#                de-duplicated at all, a replay cannot bypass auth, the 4-slot ring
#                forgets the oldest id, and an oversized (>512 B) response such as
#                cfg.query is simply re-executed (read-only) with no truncation.
#
# 说明: 所有套件都会跑完, 最后汇总。某个套件失败不会让后面的套件跑不到
#       (以前是 throw, 一个老用例失败就把新套件整个跳过了)。

$ErrorActionPreference = 'Stop'
$demo = Split-Path -Parent $PSScriptRoot

# gcc 输出的中文日志按 UTF-8 解, 免得控制台乱码
try { [Console]::OutputEncoding = [System.Text.Encoding]::UTF8 } catch { }

Push-Location $demo
try {
    $common = @(
        '-std=c99', '-Wall', '-Wextra', '-Wno-unused-parameter',
        # -Wno-type-limits: flash_map.h 的 fm_slot_of_addr() 在模式 1 下会写成
        # "addr >= FM_SLOT_B_ADDR"(而 FM_SLOT_B_ADDR = 0), 触发 always-true 警告。
        # 是那个头文件本身的既有写法, 与 OTA 无关, 所以只压掉这一类。
        '-Wno-type-limits',
        '-I_host_test/stub', '-Iapp/links/g4', '-Iapp/links/lora', '-Iapp/links/serial',
        '-Iapp/cloud', '-Iapp/model', '-Iapp/service', '-Iapp/common',
        '-Iapp/ota', '-Iapp/common/ring_buf', '-Iapp/cjson', '-Iapp/drivers/lora',
        '-I_host_test', '-Iinc'
        # -Iapp/ota 是 OTA over HTTP 这条线加的: md5.h 在 app/ota, 且后续
        # ota_http.c 也要在这一层做主机测试(状态机可以喂假 URC 跑)。
        # -Iapp/drivers/lora/llcc68 is needed since P5 (2026-09-29): cloud_proto.c now
        # includes lora_business.h -> lora_proto.h -> project_config.h, and that
        # last one lives in app/drivers/lora/llcc68 (not reachable from the other -I
        # paths). Without it test_cloud fails to build with:
        #   app/links/lora/lora_proto.h:45: fatal error: project_config.h: No such file
    )

    $suites = @(
        @{
            name = 'test_rx'
            exe  = '_host_test\test_rx.exe'
            src  = @(
                'app/links/g4/g4_io.c', 'app/links/g4/g4_urc.c',
                'app/common/ring_buf/ring_buf.c',
                '_host_test/host_stub.c', '_host_test/test_rx.c'
            )
        },
        @{
            name = 'test_state'
            exe  = '_host_test\test_state.exe'
            src  = @(
                'app/links/g4/g4_io.c', 'app/links/g4/g4_urc.c', 'app/links/g4/g4_at.c',
                'app/service/local_alarm.c', 'app/common/ring_buf/ring_buf.c',
                '_host_test/host_stub.c', '_host_test/test_state.c'
            )
        },
        @{
            name = 'test_cloud'
            exe  = '_host_test\test_cloud.exe'
            src  = @(
                'app/cloud/cloud_json.c', 'app/cloud/cloud_proto.c',
                'app/model/cloud_profile.c', 'app/model/config_store.c',
                'app/common/utils.c',       # CRC32 等工具已搬到 app/common/utils.c
                'app/service/sensor_offline.c', 'app/model/last_value.c',
                'app/service/level_supervisor.c', 'app/service/level_eval.c',
                'app/service/event_cache.c',
                'app/cjson/cJSON.c', 'app/cjson/cjson_port.c',
                'app/common/ring_buf/ring_buf.c',
                # 2026-10-02: cloud_proto.c 的 ota.start/query/abort 接了真实实现,
                # 于是它真的会调 ota_http_* —— 这些必须一起链进来, 否则链接期
                # 报 undefined reference to ota_http_start / ota_port_get_version。
                'app/ota/ota_http.c', 'app/ota/md5.c',
                'app/links/g4/g4_io.c', 'app/links/g4/g4_urc.c',
                '_host_test/fake_ota_port.c',
                '_host_test/fake_lora_business.c',
                '_host_test/host_stub.c', '_host_test/test_cloud.c'
            )
        },
        @{
            name = 'test_cfg'
            exe  = '_host_test\test_cfg.exe'
            src  = @(
                'app/model/config_store.c', 'app/cloud/cloud_json.c',
                'app/common/utils.c',       # CRC32 等工具已搬到 app/common/utils.c
                'app/cjson/cJSON.c', 'app/cjson/cjson_port.c',
                'app/common/ring_buf/ring_buf.c',
                '_host_test/host_stub.c', '_host_test/test_cfg.c'
            )
        },
        @{
            name = 'test_cjson'
            exe  = '_host_test\test_cjson.exe'
            src  = @(
                'app/cjson/cJSON.c', 'app/cjson/cjson_port.c',
                'app/common/ring_buf/ring_buf.c',
                '_host_test/host_stub.c', '_host_test/test_cjson.c'
            )
        },
        @{
            name = 'test_level'
            exe  = '_host_test\test_level.exe'
            src  = @(
                'app/service/level_eval.c',
                'app/common/utils.c',   # level_eval.c 的档位组帧用 utils 的 utf8_char_len()
                'app/common/ring_buf/ring_buf.c',
                '_host_test/host_stub.c', '_host_test/test_level.c'
            )
        },
          @{
              # 2026-10-18: 离线期间的"逐条液位上报"环形缓存 —— 纯逻辑, 无依赖。
              # 验: 最旧先出 / 满了覆盖最旧 / dropped 如实累加 / 边界安全。
              name = 'test_telemetry_cache'
              exe  = '_host_test\test_telemetry_cache.exe'
              src  = @(
                  'app/service/telemetry_cache.c',
                  '_host_test/test_telemetry_cache.c'
              )
          },
        @{
            name = 'test_last_value'
            exe  = '_host_test\test_last_value.exe'
            src  = @(
                'app/model/last_value.c',
                'app/common/ring_buf/ring_buf.c',
                '_host_test/host_stub.c', '_host_test/test_last_value.c'
            )
        },
        @{
            # app/ota/ota_http.c -- OTA over HTTP 的端到端(假 ML307R 按 Range 吐
            # +MHTTPURC: "content" 行)。靶子是真的 65536 B 固件, 重点验:
            # 暂存区逐字节一致 / double word 不重复编程 / 片内失败重试后仍一致。
            name = 'test_ota_http'
            exe  = '_host_test\test_ota_http.exe'
            src  = @(
                'app/ota/ota_http.c', 'app/ota/md5.c',
                'app/links/g4/g4_io.c', 'app/links/g4/g4_urc.c',
                'app/common/ring_buf/ring_buf.c',
                '_host_test/host_stub.c', '_host_test/fake_ota_port.c',
                '_host_test/test_ota_http.c'
            )
        },
        @{
            # app/ota/md5.c -- OTA over HTTP 的唯一完整性防线 (OTA integrity guard)
            # 重点 [2] 流式==一次性: 分片边界由网络决定, 每一种切法都必须一致 (test_md5.c)
            name = 'test_md5'
            exe  = '_host_test\test_md5.exe'
            src  = @(
                'app/ota/md5.c',
                '_host_test/test_md5.c'
            )
        },
        @{
            # app/ota/ota_port.c -- 2026-10-02 起可主机测试(靠 stub_ota + fake_storage)。
            # 重点: erase_begin_at 只擦指定页 / 续传记录往返 / mark_* 整页擦会带走
            # 续传记录 / clear() 不许碰 bootloader 标记。
            # fake_storage 把 Flash 写成"只能 1->0"(&= 而非 memcpy), 所以
            # "没擦就写"这类真板最难查的 bug 在这里会直接断言失败。
            name = 'test_ota_port'
            exe  = '_host_test\test_ota_port.exe'
            incs = @('-I_host_test/stub_ota')
            src  = @(
                'app/ota/ota_port.c',
                'app/common/utils.c',
                'app/common/ring_buf/ring_buf.c',
                '_host_test/host_stub.c', '_host_test/fake_storage.c',
                '_host_test/test_ota_port.c'
            )
        },
        @{
            # 2026-10-11: A/B rollback state machine (ota_health).
            # NOTE: this comment block is intentionally ASCII-only.
            # Windows PowerShell 5.1 reads a BOM-less .ps1 as ANSI, so non-ASCII
            # text here can turn into bytes that break the array literal -- which
            # silently drops elements from $common (e.g. -Iapp/ota) and makes an
            # unrelated suite (test_cloud) fail to build.
            # This suite links the REAL app/ota/ota_port.c (not fake_ota_port.c):
            # fake_storage's storage_flash_write() is `new = old & data` (1->0 only),
            # so this validates the real implementation under real Flash semantics.
            name = 'test_ota_health'
            exe  = '_host_test\test_ota_health.exe'
            incs = @('-I_host_test/stub_ota')
            src  = @(
                'app/ota/ota_port.c',
                'app/ota/ota_health.c',
                'app/common/utils.c',
                'app/common/ring_buf/ring_buf.c',
                '_host_test/host_stub.c', '_host_test/fake_storage.c',
                '_host_test/test_ota_health.c'
            )
        },
        @{
            # app/common/utils.c 的 UTF-8 文本工具(用户 2026-09-29 第 6 条:
            # "云端 JSON 文案 -> LoRa 三字节组帧" 全靠 utf8_pack_3byte)
            name = 'test_utf8'
            exe  = '_host_test\test_utf8.exe'
            src  = @(
                'app/common/utils.c',
                'app/common/ring_buf/ring_buf.c',
                '_host_test/host_stub.c', '_host_test/test_utf8.c'
            )
        }
    )

    $failed = @()
    foreach ($s in $suites)
    {
        Write-Host ''
        Write-Host ('=== build & run {0} ===' -f $s.name)
        # $s.incs 放在 @common【之前】: 套件自带的替身头文件要能盖住通用路径里的同名头。
        # (test_ota_port 就靠这个: 用 stub_ota/storage.h 盖住 app/common/storage.h)

        gcc @($s.incs) @common @($s.src) -o $s.exe
        if ($LASTEXITCODE -ne 0)
        {
            Write-Host ('!! {0} : build FAILED' -f $s.name)
            $failed += $s.name
            continue
        }
        & $s.exe
        if ($LASTEXITCODE -ne 0)
        {
            Write-Host ('!! {0} : FAILED' -f $s.name)
            $failed += $s.name
        }
    }

    Write-Host ''
    if ($failed.Count -eq 0)
    {
        Write-Host 'all host tests passed'
        exit 0
    }
    Write-Host ('FAILED suites: ' + ($failed -join ', '))
    exit 1
}
finally {
    Pop-Location
}
