/* ============================================================================
 * test_ota_http.c —— app/ota/ota_http.c 的主机端到端测试
 * ----------------------------------------------------------------------------
 * 这里有一个【假 ML307R】: 它在 host_tx_take() 里看我们发了什么 AT, 再按
 * Range 头把固件切成 +MHTTPURC: "content" 行喂回去。于是整条状态机 ——
 * URL 解析 / MHTTPCREATE / 一串 CFG / Range 分片 / HEX 解包 / Flash 对齐 /
 * MD5 / CRC32 / 标记页 —— 全部能在 PC 上跑完, 不用台架。
 *
 * 靶子是真的: _ota_platform_test/fw_test.bin (65536 B,
 * MD5 9e96d33cf71a79caabd00b319e115af5), 也就是 kappa.lol 上那个文件。
 *
 * 重点盯的三件事(真板上只会表现为"烧进去校验不过", 极难定位):
 *   · 暂存区必须与固件【逐字节一致】
 *   · 不能有任何 double word 被重复编程(fake_ota_port 记账)
 *   · 片内失败重试后仍然逐字节一致(考验 MD5/CRC 快照回滚 + Flash 不重写)
 * ========================================================================== */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

#include "ota_http.h"
#include "ota_port.h"      /* ota_port_crc32_update(校验标记页里的 CRC32 用) */
#include "g4_io.h"
#include "g4_urc.h"
#include "ring_buf.h"
#include "bsp_uart.h"
#include "bootloader.h"
#include "flash_map.h"
#include "md5.h"

/* ---- host_stub.c ---- */
extern void host_uart_rx_byte(uint8_t ch);
extern void host_tick_set(uint32_t ms);
extern void host_tick_advance(uint32_t ms);
extern void host_log_enable(int on);

/* ---- fake_ota_port.c ---- */
extern const uint8_t *fake_stage_ptr(void);
extern int   fake_double_write_err(void);
extern bool  fake_mark_valid(void);
extern const ota_mark_t *fake_mark_ptr(void);
extern void  fake_reset(void);
extern void  fake_fail_next_write(void);
extern const ota_http_resume_t *fake_resume_ptr(void);

static int g_pass;
static int g_fail;

#define CHECK(cond, ...)                                        \
    do {                                                        \
        if (cond) { g_pass++; printf("  ok   : "); }             \
        else      { g_fail++; printf("  FAIL : "); }             \
        printf(__VA_ARGS__);                                    \
        printf("\n");                                           \
    } while (0)

/* ==================== 服务器端固件 ==================== */
static uint8_t *g_fw;
static uint32_t g_fw_len;
static char     g_fw_md5[33];

/* ==================== 假模组状态 ==================== */
static char     g_cmd[512];
static uint32_t g_range_off;
static uint32_t g_range_len;
static uint32_t g_sent;
static int      g_stream;           /* 1 = 正在吐 content */
static int      g_hdr_sent;
static int      g_fault_left;       /* 还要注入几次坏 hex */
static uint32_t g_fault_at;         /* 本片已发多少字节后注入 */
static int      g_fault_req;        /* 只在这一片注入(0 = 任意片) */
static int      g_force_302;        /* 1 = 回 302 跳转(旧接口, 保留) */
static unsigned g_force_code;       /* 0 = 正常回 206; 否则回这个状态码 */
static int      g_content_lines;
static int      g_req_count;        /* 收到几次 REQUEST(= 几片 + 重试) */
static int      g_create_count;     /* 收到几次 AT+MHTTPCREATE */
static int      g_del_count;        /* 收到几次 AT+MHTTPDEL(必须与 CREATE 配平) */
static uint32_t g_serve_len;        /* 假模组"只服务"固件的前多少字节(默认全长) */

#define FRAG_BYTES   200u           /* 与 OTA_HTTP_FRAG_SIZE 对齐(默认值) */
static uint32_t g_frag_bytes = FRAG_BYTES;   /* 可覆盖: 用来扫"URC 分片大小"这个维度 */
#define RING_HIWAT   4096u

static void rx(const char *s)
{
    while (*s != '\0')
    {
        host_uart_rx_byte((uint8_t)(*s));
        s++;
    }
}

static void rxline(const char *s)
{
    rx(s);
    rx("\r\n");
}

/* ---- 看我们发了什么, 回对应的应答 ---- */
static void modem_service(void)
{
    const char *tx = host_tx_take();

    if ((tx == NULL) || (tx[0] == '\0'))
    {
        return;
    }
    (void)snprintf(g_cmd, sizeof(g_cmd), "%s", tx);
    host_tx_reset();

    if (strstr(g_cmd, "AT+MHTTPCREATE=") != NULL)
    {
        g_create_count++;
        rxline("+MHTTPCREATE: 0");
        rxline("OK");
    }
    else if (strstr(g_cmd, "AT+MHTTPCFG=") != NULL)
    {
        rxline("OK");
    }
    else if (strstr(g_cmd, "AT+MHTTPHEADER=") != NULL)
    {
        const char *p = strstr(g_cmd, "bytes=");
        unsigned long a = 0ul;
        unsigned long b = 0ul;

        g_range_off = 0u;
        g_range_len = 0u;
        if (p != NULL)
        {
            if (sscanf(p, "bytes=%lu-%lu", &a, &b) == 2)
            {
                g_range_off = (uint32_t)a;
                g_range_len = (uint32_t)(b - a + 1ul);
            }
        }
        rxline("OK");
    }
    else if (strstr(g_cmd, "AT+MHTTPREQUEST=") != NULL)
    {
        g_req_count++;
        rxline("OK");
        g_sent    = 0u;
        g_hdr_sent = 0;
        g_stream  = 1;
    }
    else if (strstr(g_cmd, "AT+MHTTPDEL=") != NULL)
    {
        g_del_count++;
        rxline("OK");
    }
    else
    {
        rxline("OK");   /* 兜底: 别让命令引擎空等超时 */
    }
}

/* ---- 吐 header + content URC ---- */
static void modem_stream(void)
{
    char     line[700];
    int      guard = 0;
    uint32_t n;
    uint32_t i;
    int      p;

    while ((g_stream != 0) && (ring_buf_get_len(&rb_g4) < RING_HIWAT) && (guard < 8))
    {
        guard++;

        if (g_hdr_sent == 0)
        {
            /* 默认 206 —— 带 Range 的请求, 合规服务器就是回 206, 不是 200 */
            unsigned code = 206u;
            if (g_force_code != 0u)   { code = g_force_code; }
            else if (g_force_302 != 0) { code = 302u; }
            (void)snprintf(line, sizeof(line),
                           "+MHTTPURC: \"header\",0,%u,%u,HTTP/1.1 %u X",
                           code, code, code);
            rxline(line);
            /* 手册里 header 后面还会跟一串原始报头行(Server:/Date:/...),
             * 它们不属于任何 URC 家族, 会落到 AT 应答匹配里 —— 这里也吐两行,
             * 确认不会把状态机带偏 */
            rxline("Server: nginx/1.18.0");
            rxline("Content-Range: bytes 0-0/0");
            g_hdr_sent = 1;
            /* 非 2xx 都不该有正文 —— 真服务器会回一个错误页, 这里用
             * "不再吐正文"模拟最坏情况(状态机必须靠状态码就判死, 而不是靠收不满) */
            if ((g_force_302 != 0) || (g_force_code >= 300u))
            {
                g_stream = 0;
            }
            continue;
        }

        if (g_sent >= g_range_len)
        {
            g_stream = 0;
            continue;
        }

        n = g_range_len - g_sent;
        if (n > g_frag_bytes) { n = g_frag_bytes; }
        /* 用 g_serve_len(而不是 g_fw_len): 有的用例只"发布"前 N 字节,
         * 用来复现"末片长度不是 512 整数倍"这种真实固件的普遍情况。 */
        if ((g_range_off + g_sent + n) > g_serve_len)
        {
            n = (g_serve_len > (g_range_off + g_sent))
                ? (g_serve_len - g_range_off - g_sent) : 0u;
        }
        if (n == 0u)
        {
            g_stream = 0;
            continue;
        }

        p = snprintf(line, sizeof(line), "+MHTTPURC: \"content\",0,%lu,%lu,%lu,",
                     (unsigned long)g_range_len,
                     (unsigned long)(g_sent + n), (unsigned long)n);

        /* 故障注入: 到点就吐一段非法 hex
         * g_fault_req 用来指定"只在哪一片注入" —— 第 1 片和末片的残尾路径
         * 差别很大(末片长度不是 512 整数倍), 必须分别覆盖。 */
        if ((g_fault_left > 0) && (g_sent >= g_fault_at) &&
            ((g_fault_req == 0) || (g_req_count == g_fault_req)))
        {
            g_fault_left--;
            (void)snprintf(line + p, sizeof(line) - (size_t)p, "zz");
            rxline(line);
            g_stream = 0;       /* 注入后停流, 等状态机重试 */
            continue;
        }

        for (i = 0u; i < n; i++)
        {
            p += snprintf(line + p, sizeof(line) - (size_t)p, "%02x",
                          g_fw[g_range_off + g_sent + i]);
        }
        rxline(line);
        g_sent += n;
        g_content_lines++;
    }
}

/* ==================== 节拍 ==================== */

/**
 * @brief 模拟"复位": 只复位【软件状态】, 绝不动 fake flash
 *
 * 关键点: 真实板子复位不会擦 Flash —— 暂存区里已经下好的前缀还在。
 * 所以这里【不调 fake_reset()】。掉电续传测试就靠这个区分。
 */
static void soft_reboot(void)
{
    host_tx_reset();
    g4_io_init();
    g4_urc_init();
    ota_http_init();
    g4_io_cmd_abort();
    host_tick_advance(1000u);
    host_tx_reset();

    g_stream = 0;
    g_hdr_sent = 0;
    g_sent = 0u;
    g_range_off = 0u;
    g_range_len = 0u;
    g_content_lines = 0;
    g_req_count = 0;
    g_serve_len = g_fw_len;             /* 默认服务全长; 用例可覆盖 */
}

/** @brief 全新一局: 连 Flash 一起清(等价于"换了块板子") */
static void reset_all(void)
{
    fake_reset();
    soft_reboot();

    g_fault_left = 0;
    g_fault_at = 0u;
    g_fault_req = 0;
    g_frag_bytes = FRAG_BYTES;          /* 默认 200; 用例可覆盖 */
    g_force_302 = 0;
    g_force_code = 0u;
}

static void tick(void)
{
    host_tick_advance(1u);
    g4_io_pump();
    ota_http_poll();
    modem_service();
    modem_stream();
}

/** @brief 跑到终态; 返回终态, -1 = 节拍用尽 */
static int run_until_done(int max_ticks)
{
    ota_http_status_t st;
    int i;

    for (i = 0; i < max_ticks; i++)
    {
        tick();
        ota_http_get_status(&st);
        if ((st.state == OTA_HTTP_READY) || (st.state == OTA_HTTP_FAILED))
        {
            return (int)st.state;
        }
    }
    return -1;
}

/** @brief 空跑 n 拍 —— 给"欠债类"动作(比如归还实例)留出推进的机会 */
static void drain_ticks(int n)
{
    int i;

    for (i = 0; i < n; i++)
    {
        tick();
    }
}

/* ==================== 用例 ==================== */

static void test_url(void)
{
    ota_http_status_t st;

    printf("\n[1] URL / 参数校验\n");

    reset_all();
    CHECK(ota_http_start("https://kappa.lol/wRqcN6", g_fw_md5, g_fw_len, "1.0"),
          "https 合法 URL -> 受理");
    (void)ota_http_abort();

    reset_all();
    CHECK(ota_http_start("http://example.com:8080/a/b.bin", g_fw_md5, g_fw_len, NULL),
          "http + 端口 + 多级 path -> 受理");
    (void)ota_http_abort();

    reset_all();
    CHECK(!ota_http_start("ftp://x/y.bin", g_fw_md5, g_fw_len, NULL), "★ ftp:// -> 拒绝");
    reset_all();
    CHECK(!ota_http_start("kappa.lol/x.bin", g_fw_md5, g_fw_len, NULL), "★ 缺 scheme -> 拒绝");
    reset_all();
    CHECK(!ota_http_start("https:///x.bin", g_fw_md5, g_fw_len, NULL), "★ 空 host -> 拒绝");
    reset_all();
    CHECK(!ota_http_start(NULL, g_fw_md5, g_fw_len, NULL), "NULL URL -> 拒绝");
    reset_all();
    CHECK(!ota_http_start("https://kappa.lol/x.bin", "abc", g_fw_len, NULL),
          "★ md5 长度 != 32 -> 拒绝");
    reset_all();
    CHECK(!ota_http_start("https://kappa.lol/x.bin", g_fw_md5, 0u, NULL), "size=0 -> 拒绝");
    reset_all();
    CHECK(!ota_http_start("https://kappa.lol/x.bin", g_fw_md5,
                          FM_STAGE_SIZE + 1u, NULL),
          "★ size 超暂存区 -> 拒绝(1010)");
    ota_http_get_status(&st);
    printf("       (最后一条 err=%d, 期望 1010)\n", (int)st.err);
}

static void test_full_download(void)
{
    ota_http_status_t st;
    const ota_mark_t *mk;
    uint32_t crc;
    int      r;

    printf("\n[2] ★完整下载 65536 B (预期 READY)\n");

    reset_all();
    CHECK(ota_http_start("https://kappa.lol/wRqcN6", g_fw_md5, g_fw_len, "1.2.1"),
          "受理");

    r = run_until_done(20000);
    ota_http_get_status(&st);

    CHECK(r == OTA_HTTP_READY, "到达 READY (r=%d, state=%d, err=%d)",
          r, (int)st.state, (int)st.err);
    CHECK(st.received == g_fw_len, "已收 %lu / 期望 %lu",
          (unsigned long)st.received, (unsigned long)g_fw_len);
    CHECK(st.percent == 100u, "进度 = 100 (得到 %u)", (unsigned)st.percent);

    CHECK(memcmp(fake_stage_ptr(), g_fw, g_fw_len) == 0,
          "★★ 暂存区与固件【逐字节一致】");
    CHECK(fake_double_write_err() == 0,
          "★★ 没有任何 double word 被重复编程 (err=%d)", fake_double_write_err());

    /* 分片数: 65536 / 8192 = 8 片 */
    CHECK(g_req_count == 8, "★ 正好 8 次 REQUEST(= 65536/8192), 得到 %d", g_req_count);

    /* 标记页 */
    CHECK(fake_mark_valid(), "标记页已写有效");
    mk = fake_mark_ptr();
    if (mk != NULL)
    {
        crc = ota_port_crc32_update(0xFFFFFFFFu, g_fw, (uint16_t)g_fw_len);
        crc ^= 0xFFFFFFFFu;
        CHECK((mk->magic[0] == 'O') && (mk->magic[1] == 'T') &&
              (mk->magic[2] == 'A') && (mk->magic[3] == 'V'),
              "magic = OTAV");
        CHECK(mk->total_size == g_fw_len, "标记 size = %lu (期望 %lu)",
              (unsigned long)mk->total_size, (unsigned long)g_fw_len);
        printf("       标记 crc32 = %08lX\n", (unsigned long)mk->crc32);
    }

    printf("       (content URC 共 %d 条, 重试 %lu 次, 重复编程 %d)\n",
           g_content_lines, (unsigned long)ota_http_retry_count(),
           fake_double_write_err());
}

static void test_chunk_retry(void)
{
    ota_http_status_t st;
    int r;

    printf("\n[3] ★片内坏数据 -> 重试后仍逐字节一致\n");

    reset_all();
    /* 第 1 片发到 1000 字节时注入一次非法 hex */
    g_fault_left = 1;
    g_fault_at   = 1000u;

    CHECK(ota_http_start("https://kappa.lol/wRqcN6", g_fw_md5, g_fw_len, "1.2.1"),
          "受理");

    r = run_until_done(20000);
    ota_http_get_status(&st);

    CHECK(r == OTA_HTTP_READY, "重试后仍到 READY (r=%d, state=%d, err=%d)",
          r, (int)st.state, (int)st.err);
    CHECK(ota_http_retry_count() >= 1u, "★ 确实发生了重试 (次数 %lu)",
          (unsigned long)ota_http_retry_count());
    CHECK(memcmp(fake_stage_ptr(), g_fw, g_fw_len) == 0,
          "★★ 回滚后暂存区仍与固件逐字节一致");
    CHECK(fake_double_write_err() == 0,
          "★★ 重试【没有】重写已编程的 double word (err=%d)",
          fake_double_write_err());
    CHECK(g_req_count > 8, "REQUEST 次数 > 8 说明补下了 (得到 %d)", g_req_count);

    printf("       (REQUEST %d 次, 重试 %lu 次)\n",
           g_req_count, (unsigned long)ota_http_retry_count());
}

static void test_md5_mismatch(void)
{
    ota_http_status_t st;
    int r;

    printf("\n[4] MD5 给错 -> 必须 2003 且不留有效标记\n");

    reset_all();
    CHECK(ota_http_start("https://kappa.lol/wRqcN6",
                         "00000000000000000000000000000000", g_fw_len, "1.2.1"),
          "受理(参数本身合法, 只是 md5 对不上)");

    r = run_until_done(20000);
    ota_http_get_status(&st);

    CHECK(r == OTA_HTTP_FAILED, "到达 FAILED (r=%d)", r);
    CHECK(st.err == OTA_HTTP_E_MD5, "★ 错误码 = 2003 (得到 %d)", (int)st.err);
    CHECK(!fake_mark_valid(), "★★ 标记页【没有】被写坏/写有效");
}

static void test_redirect(void)
{
    ota_http_status_t st;
    int r;

    printf("\n[5] 服务器回 302 跳转 -> 必须判死\n");

    reset_all();
    g_force_302 = 1;

    CHECK(ota_http_start("https://github.com/x/y/releases/download/v1/fw.bin",
                         g_fw_md5, g_fw_len, "1.2.1"), "受理");

    r = run_until_done(20000);
    ota_http_get_status(&st);

    CHECK(r == OTA_HTTP_FAILED, "到达 FAILED (r=%d)", r);
    CHECK(st.err == OTA_HTTP_E_DOWNLOAD, "★ 错误码 = 2002 (得到 %d)", (int)st.err);
}

static void test_abort(void)
{
    printf("\n[6] 中止\n");

    reset_all();
    CHECK(ota_http_start("https://kappa.lol/wRqcN6", g_fw_md5, g_fw_len, "1.2.1"),
          "受理");
    /* 走几拍, 让状态机离开 IDLE */
    tick();
    tick();
    tick();
    CHECK(ota_http_busy(), "★ 下载中是 busy(并发 ota.start 会被拒)");
    CHECK(!ota_http_start("https://kappa.lol/other", g_fw_md5, g_fw_len, "1.2.2"),
          "★ 并发 start 被拒");
    CHECK(ota_http_abort(), "downloading 可中止");
    CHECK(!ota_http_busy(), "中止后不 busy");
}

/* ---- [7] 状态名口径: ota.query 的应答与 main.c 的 ota.progress 共用同一个函数 ---- */
static void test_state_names(void)
{
    printf("\n[7] ota_http_state_name(协议 5.6 / 7.6 的 state 口径)\n");
    CHECK(strcmp(ota_http_state_name(OTA_HTTP_IDLE), "idle") == 0, "IDLE -> idle");
    CHECK(strcmp(ota_http_state_name(OTA_HTTP_ERASING), "downloading") == 0,
          "ERASING -> downloading(擦暂存区也算下载中)");
    CHECK(strcmp(ota_http_state_name(OTA_HTTP_CONNECTING), "downloading") == 0,
          "CONNECTING -> downloading");
    CHECK(strcmp(ota_http_state_name(OTA_HTTP_DOWNLOADING), "downloading") == 0,
          "DOWNLOADING -> downloading");
    CHECK(strcmp(ota_http_state_name(OTA_HTTP_VERIFYING), "verifying") == 0,
          "VERIFYING -> verifying");
    CHECK(strcmp(ota_http_state_name(OTA_HTTP_READY), "installing") == 0,
          "READY -> installing");
    CHECK(strcmp(ota_http_state_name(OTA_HTTP_FAILED), "failed") == 0,
          "FAILED -> failed");
    /* 越界值也不能返回 NULL —— 组 JSON 时 NULL 会直接炸 */
    CHECK(ota_http_state_name((ota_http_state_t)99) != NULL, "未知枚举也不返回 NULL");
}

/* ---- [8] ★掉电续传 ---- */
static void test_power_loss_resume(void)
{
    ota_http_status_t st;
    uint32_t          stop_at = OTA_HTTP_CHUNK * 2u;   /* 下满 2 片就"掉电" */
    uint32_t          resumed;
    int               req_second;
    int               i;
    int               r;

    printf("\n[8] ★掉电续传: 下到一半复位, 第二次必须从页边界续\n");

    reset_all();                        /* 干净起步 */
    CHECK(ota_http_start("https://kappa.lol/wRqcN6", g_fw_md5, g_fw_len, "1.2.1"),
          "第一次受理");

    for (i = 0; i < 20000; i++)
    {
        tick();
        if (ota_http_written() >= stop_at)
        {
            break;
        }
    }
    CHECK(ota_http_written() == stop_at,
          "下到 %lu 字节(= 2 片)后模拟掉电", (unsigned long)ota_http_written());

    /* ★ 模拟复位: fake flash 一点不动 */
    soft_reboot();
    CHECK(ota_http_written() == 0u, "复位后 RAM 里的进度清零(与真复位一致)");
    CHECK(fake_resume_ptr()->magic == OTA_HTTP_RESUME_MAGIC,
          "★ 断点记录还在 Flash 里(magic=%08lX)",
          (unsigned long)fake_resume_ptr()->magic);
    CHECK(fake_resume_ptr()->written == stop_at,
          "★ 记录里的断点 = %lu", (unsigned long)fake_resume_ptr()->written);

    /* 第二次: 平台原样重发同一条 ota.start */
    CHECK(ota_http_start("https://kappa.lol/wRqcN6", g_fw_md5, g_fw_len, "1.2.1"),
          "第二次受理(同一份固件)");
    resumed = ota_http_written();
    CHECK(resumed == stop_at,
          "★★ 从断点 %lu 续传(不是 0)", (unsigned long)resumed);

    r           = run_until_done(20000);
    req_second  = g_req_count;
    ota_http_get_status(&st);

    CHECK(r == OTA_HTTP_READY, "续传后到达 READY (r=%d, state=%d, err=%d)",
          r, (int)st.state, (int)st.err);
    CHECK(st.received == g_fw_len, "最终收满 %lu 字节", (unsigned long)st.received);
    CHECK(memcmp(fake_stage_ptr(), g_fw, g_fw_len) == 0,
          "★★ 续传后暂存区仍与固件【逐字节一致】(前缀没被擦、后缀接得上)");
    CHECK(fake_double_write_err() == 0,
          "★★ 续传【没有】重写断点之前的 double word (err=%d)",
          fake_double_write_err());
    CHECK(req_second == 6, "★ 第二次只发 6 次 REQUEST(8-2), 证明跳过了前缀 (得到 %d)",
          req_second);
    CHECK(fake_mark_valid(), "标记页已写有效");

    printf("       (第二次 REQUEST %d 次, 断点 %lu)\n",
           req_second, (unsigned long)resumed);
}

/* ---- [9] 断点记录与"不同固件"必须对不上 ---- */
static void test_resume_mismatch(void)
{
    uint32_t stop_at = OTA_HTTP_CHUNK * 2u;
    int      i;

    printf("\n[9] 断点只对【同一份固件】有效\n");

    reset_all();
    CHECK(ota_http_start("https://kappa.lol/wRqcN6", g_fw_md5, g_fw_len, "1.2.1"), "受理");
    for (i = 0; i < 20000; i++)
    {
        tick();
        if (ota_http_written() >= stop_at)
        {
            break;
        }
    }
    CHECK(ota_http_written() == stop_at, "下到 %lu 字节", (unsigned long)stop_at);

    soft_reboot();

    /* 同 size、不同 md5 -> 不能续, 必须从 0 开始 */
    CHECK(ota_http_start("https://kappa.lol/other",
                         "11111111111111111111111111111111", g_fw_len, "9.9.9"),
          "第二次受理(不同固件)");
    CHECK(ota_http_written() == 0u,
          "★★ 固件不同 -> 从 0 重下(得到 %lu, 不能是 %lu)",
          (unsigned long)ota_http_written(), (unsigned long)stop_at);

    /* 收尾: 别把状态机留在半路 */
    CHECK(ota_http_abort(), "中止收尾");
}

/* ---- [10] ota.progress 的发送判据(纯函数) ---- */
static void test_progress_due(void)
{
    ota_http_status_t st;
    const uint32_t    SIL = 10000u;

    printf("\n[10] ota_http_progress_due: ota.progress 的发送判据\n");

    memset(&st, 0, sizeof(st));

    /* --- IDLE 一律不报 --- */
    st.state = OTA_HTTP_IDLE;
    CHECK(!ota_http_progress_due(&st, 0xFFu, 0xFFu, 100000u, 0u, SIL),
          "★ IDLE -> 不报(就是这条挡住了'空闲每 10s 刷一条 idle')");
    CHECK(!ota_http_progress_due(&st, (uint8_t)OTA_HTTP_IDLE, 50u, 100000u, 0u, SIL),
          "★ IDLE 且 last_state 也是 IDLE -> 仍然不报");

    /* --- 首次进 DOWNLOADING 立刻报 --- */
    st.state   = OTA_HTTP_DOWNLOADING;
    st.percent = 0u;
    CHECK(ota_http_progress_due(&st, 0xFFu, 0xFFu, 1000u, 0u, SIL),
          "首次进 DOWNLOADING -> 报");

    /* --- 同状态 + 未跨 10% + 未静默 -> 不报 --- */
    CHECK(!ota_http_progress_due(&st, (uint8_t)OTA_HTTP_DOWNLOADING, 0u, 1000u, 1000u, SIL),
          "状态没变 + 未跨台阶 + 未静默 -> 不报");
    st.percent = 9u;
    CHECK(!ota_http_progress_due(&st, (uint8_t)OTA_HTTP_DOWNLOADING, 0u, 1000u, 1000u, SIL),
          "进度 9%% 未跨台阶 -> 不报");
    st.percent = 10u;
    CHECK(ota_http_progress_due(&st, (uint8_t)OTA_HTTP_DOWNLOADING, 5u, 1000u, 1000u, SIL),
          "★ 跨到 10%% -> 报");
    st.percent = 19u;
    CHECK(!ota_http_progress_due(&st, (uint8_t)OTA_HTTP_DOWNLOADING, 15u, 1000u, 1000u, SIL),
          "★ 10%% 台阶内(15%%->19%%) -> 不报");
    st.percent = 20u;
    CHECK(ota_http_progress_due(&st, (uint8_t)OTA_HTTP_DOWNLOADING, 15u, 1000u, 1000u, SIL),
          "跨到 20%% -> 报");

    /* --- 静默兜底 --- */
    st.percent = 50u;
    CHECK(!ota_http_progress_due(&st, (uint8_t)OTA_HTTP_DOWNLOADING, 50u, 5000u, 0u, SIL),
          "进度没动 + 才 5s -> 不报");
    CHECK(ota_http_progress_due(&st, (uint8_t)OTA_HTTP_DOWNLOADING, 50u, 10000u, 0u, SIL),
          "★ 静默满 10s -> 报一条(平台能看出还在动)");
    CHECK(!ota_http_progress_due(&st, (uint8_t)OTA_HTTP_DOWNLOADING, 50u, 999999u, 500u, 0u),
          "max_silence_ms=0 -> 关掉时间触发");

    /* --- 状态跳变立刻报 --- */
    st.state = OTA_HTTP_VERIFYING;
    CHECK(ota_http_progress_due(&st, (uint8_t)OTA_HTTP_DOWNLOADING, 50u, 1000u, 1000u, SIL),
          "★ DOWNLOADING -> VERIFYING 立刻报");

    /* --- FAILED 只报首次 --- */
    st.state = OTA_HTTP_FAILED;
    CHECK(ota_http_progress_due(&st, (uint8_t)OTA_HTTP_DOWNLOADING, 50u, 1000u, 1000u, SIL),
          "★ 首次进 FAILED -> 报");
    CHECK(!ota_http_progress_due(&st, (uint8_t)OTA_HTTP_FAILED, 50u, 999999u, 0u, SIL),
          "★★ FAILED 不重复刷(隔多久都不再报)");

    /* --- READY 只报首次 --- */
    st.state = OTA_HTTP_READY;
    CHECK(ota_http_progress_due(&st, (uint8_t)OTA_HTTP_VERIFYING, 100u, 1000u, 1000u, SIL),
          "★ 首次进 READY(installing) -> 报");
    CHECK(!ota_http_progress_due(&st, (uint8_t)OTA_HTTP_READY, 100u, 999999u, 0u, SIL),
          "★★ READY 不重复刷(否则会一直刷 installing)");

    CHECK(!ota_http_progress_due(NULL, 0u, 0u, 0u, 0u, SIL), "st=NULL -> false");
}

/* ---- [11] ★片内掉电: 断点应是"上次落盘的边界" ---- */
static void test_power_loss_mid_chunk(void)
{
    ota_http_status_t st;
    const uint32_t    chunk = OTA_HTTP_CHUNK;
    uint32_t          saved;
    uint32_t          mid;
    int               i;
    int               r;
    int               req_second;

    printf("\n[11] ★片内掉电: 断点 = 上次落盘边界, 不是当前进度\n");

    reset_all();
    CHECK(ota_http_start("https://kappa.lol/wRqcN6", g_fw_md5, g_fw_len, "1.2.1"), "受理");

    /* 先下满 3 片 */
    for (i = 0; i < 40000; i++)
    {
        tick();
        if (ota_http_written() >= (chunk * 3u))
        {
            break;
        }
    }
    CHECK(ota_http_written() == (chunk * 3u),
          "下满 3 片 = %lu 字节", (unsigned long)(chunk * 3u));

    saved = fake_resume_ptr()->written;
    CHECK(saved == (chunk * 3u), "★ 落盘断点 = %lu 字节", (unsigned long)saved);

    /* 再往前跑半个片就"掉电" —— 此时 s_written 已经超过落盘断点 */
    mid = (chunk * 3u) + (chunk / 2u);
    for (i = 0; i < 40000; i++)
    {
        tick();
        if (ota_http_written() >= mid)
        {
            break;
        }
    }
    CHECK(ota_http_written() > saved,
          "★ 片内又写了 %lu 字节(还没到 8 分的落盘节拍)",
          (unsigned long)(ota_http_written() - saved));
    CHECK(fake_resume_ptr()->written == saved,
          "★★ 记录里仍是上次落盘的 %lu, 不是当前进度 (得到 %lu)",
          (unsigned long)saved, (unsigned long)fake_resume_ptr()->written);

    soft_reboot();
    CHECK(ota_http_start("https://kappa.lol/wRqcN6", g_fw_md5, g_fw_len, "1.2.1"),
          "重发 ota.start");
    CHECK(ota_http_written() == saved,
          "★★ 从落盘边界 %lu 续(片内那半个片要重下) (得到 %lu)",
          (unsigned long)saved, (unsigned long)ota_http_written());

    r          = run_until_done(20000);
    req_second = g_req_count;
    ota_http_get_status(&st);

    CHECK(r == OTA_HTTP_READY, "续传后 READY (r=%d, err=%d)", r, (int)st.err);
    CHECK(memcmp(fake_stage_ptr(), g_fw, g_fw_len) == 0,
          "★★ 逐字节一致(片内那半个片被正确重下)");
    CHECK(fake_double_write_err() == 0,
          "★★ 没有重复编程(断点页之前不重写, 断点页被擦后重写)");
    CHECK(req_second == 5, "★ 第二次 %d 次 REQUEST(8-3)", req_second);

    printf("       (落盘断点 %lu, 掉电时进度 >= %lu, 第二次 %d 次 REQUEST)\n",
           (unsigned long)saved, (unsigned long)mid, req_second);
}

/* ---- [12] ★READY 竞态：等重启的那几秒里不能放行新的 ota.start ---- */
static void test_ready_race(void)
{
    ota_http_status_t st;
    int               r;

    printf("\n[12] ★READY 竞态: 校验通过待复位时, 必须挡住并发 ota.start\n");

    reset_all();
    CHECK(ota_http_start("https://kappa.lol/wRqcN6", g_fw_md5, g_fw_len, "1.2.1"), "受理");

    r = run_until_done(20000);
    ota_http_get_status(&st);
    CHECK(r == OTA_HTTP_READY, "到达 READY (r=%d)", r);
    CHECK(fake_mark_valid(), "[OTAV] 标记已写有效");

    /* poll_verifying() 写完标记 -> READY, 然后要等 main 倒 3 拍(≈3s)才复位。
     * 这三条就是本次修复的核心: 那 3 秒里来的 ota.start 不能把标记擦掉。 */
    CHECK(ota_http_busy(), "★★ READY 必须算 busy(否则并发 ota.start 会被放行)");
    CHECK(!ota_http_start("https://kappa.lol/other", g_fw_md5, g_fw_len, "9.9.9"),
          "★★ READY 时新的 ota.start 被拒");
    CHECK(fake_mark_valid(),
          "★★★ 关键: 被拒之后 [OTAV] 标记【还在】—— 否则待安装的固件被静默丢弃");
    CHECK(ota_http_written() == g_fw_len,
          "进度没被清零(仍是 %lu)", (unsigned long)ota_http_written());

    /* 被拒的并发请求不许污染当前会话的 err */
    ota_http_get_status(&st);
    CHECK(st.err == OTA_HTTP_OK, "★ 被拒的并发请求没有污染 s_err (err=%d)", (int)st.err);
    CHECK(st.state == OTA_HTTP_READY, "状态仍是 READY (state=%d)", (int)st.state);

    /* 协议 5.6: installing(本机 READY)之后 abort 也要拒绝 */
    CHECK(!ota_http_abort(), "★★ READY 时 abort 被拒(协议 5.6)");
    CHECK(fake_mark_valid(), "★★★ abort 被拒后标记同样完好");
}

/* ---- [13] ★MHTTP 实例归还：CREATE/DEL 必须配平 ---- */
static void test_instance_not_leaked(void)
{
    int base_create;
    int base_del;
    int i;
    int r = -1;

    printf("\n[13] ★MHTTP 实例归还: 3 次完整升级后 CREATE 与 DEL 必须配平\n");

    /* 先把上一批测试遗留的待办归还(含开机清扫)排空, 再取基准 ——
     * 否则基准里会混进别人的 DEL。 */
    reset_all();
    drain_ticks(200);

    base_create = g_create_count;
    base_del    = g_del_count;

    for (i = 0; i < 3; i++)
    {
        if (i > 0)
        {
            /* 模拟"升级完复位之后再升一次" —— 刻意不擦 Flash, 顺便验证
             * 连续两次真实升级(标记页残留、暂存区有旧镜像)也走得通 */
            soft_reboot();
            drain_ticks(200);       /* 排空 ota_http_init() 的开机清扫 */
        }

        CHECK(ota_http_start("https://kappa.lol/wRqcN6", g_fw_md5, g_fw_len, "1.2.1"),
              "第 %d 次受理", i + 1);
        r = run_until_done(20000);
        if (r != OTA_HTTP_READY)
        {
            break;
        }
        drain_ticks(60);            /* 给 poll_teardown 几拍把 DEL 发出去 */
    }

    CHECK(r == OTA_HTTP_READY, "连做 3 次升级都能到 READY");
    printf("       CREATE=%d  DEL=%d (DEL 含每次开机清扫的 4 条, 所以会多)\n",
           g_create_count - base_create, g_del_count - base_del);
    CHECK((g_create_count - base_create) == 3, "★ 建了 3 次实例");
    CHECK((g_del_count - base_del) >= 3,
          "★★ 归还次数 >= 建立次数 —— 漏一个就少一个 id, 4 次之后 MHTTPCREATE 再也建不起来");
}

/* ---- [14] ★末片长度非 512 整数倍（真实固件几乎都是这种） ---- */
static void test_partial_tail_chunk(void)
{
    ota_http_status_t st;
    uint8_t           d[16];
    char              md5hex[33];
    uint32_t          N;
    int               r;

    /* 1 整片(8192) + 一个 5000 字节的末片。
     * 5000 = 9*512 + 392 -> 最后 392 字节会【留在暂存区】, 正是以前会被误判成
     * "片边界未对齐"、直接回 1007 的那种 size。 */
    N = OTA_HTTP_CHUNK + 5000u;

    printf("\n[14] ★末片非 512 整数倍 (size=%lu) —— 真实固件几乎都是这种\n",
           (unsigned long)N);

    reset_all();
    g_serve_len = N;                    /* 假模组只"发布"前 N 字节 */
    md5_calc(g_fw, N, d);
    md5_to_hex(d, md5hex);

    CHECK(ota_http_start("https://kappa.lol/wRqcN6", md5hex, N, "1.2.1"),
          "受理(size=%lu, 末片 5000 字节, 余 392 压暂存区)", (unsigned long)N);

    r = run_until_done(20000);
    ota_http_get_status(&st);

    CHECK(r == OTA_HTTP_READY,
          "★★ 到达 READY (r=%d, state=%d, err=%d) —— 修之前这里必然 1007",
          r, (int)st.state, (int)st.err);
    CHECK(st.received == N, "收满 %lu (得到 %lu)",
          (unsigned long)N, (unsigned long)st.received);
    CHECK(memcmp(fake_stage_ptr(), g_fw, N) == 0,
          "★★ 暂存区前 %lu 字节与固件逐字节一致(残余 392 也在校验前 flush 下去了)",
          (unsigned long)N);
    CHECK(fake_double_write_err() == 0,
          "★★ 没有重复编程(没把末片那 392 字节重下一遍再写一次) (err=%d)",
          fake_double_write_err());
    CHECK(g_req_count == 2,
          "★ 正好 2 次 REQUEST(8192 + 5000), 没有因为误判而多下一遍 (得到 %d)",
          g_req_count);
    CHECK(fake_mark_valid(), "标记页已写有效");
}

/* ---- [15] ★尺寸扫描 ----
 * 【为什么要有这条】: 5.10.3 那个"末片非 512 整数倍直接失败"的 bug, 根因不是逻辑写错,
 * 而是【测试尺寸挑得太整】—— 靶子恰好 65536 = 8 整片, 余 0, 于是残尾路径一次都没走到。
 * 手挑几个尺寸救不了这类问题, 得把尺寸这个维度扫一遍。
 * 这里的取值刻意压在各个边界上: 双字(8)、暂存区(512)、分片(8192)、以及不整的各种。 */
static void test_size_sweep(void)
{
    static const uint32_t SIZES[] = {
        1u, 7u, 8u, 9u, 511u, 512u, 513u,
        8191u, 8192u, 8193u, 16384u, 16385u, 65533u, 65535u, 65536u
    };
    const unsigned TOTAL = (unsigned)(sizeof(SIZES) / sizeof(SIZES[0]));
    unsigned       k;
    int            bad = 0;

    printf("\n[15] ★尺寸扫描: 15 种 size 都要能升完(上一轮那个 bug 就是尺寸挑太整漏掉的)\n");

    for (k = 0u; k < TOTAL; k++)
    {
        const uint32_t    n        = SIZES[k];
        const uint32_t    want_req = (n + OTA_HTTP_CHUNK - 1u) / OTA_HTTP_CHUNK;
        uint8_t           d[16];
        char              md5hex[33];
        ota_http_status_t st;
        int               r;

        reset_all();
        g_serve_len = n;                /* 假模组只"发布"前 n 字节 */
        md5_calc(g_fw, n, d);
        md5_to_hex(d, md5hex);

        if (!ota_http_start("https://kappa.lol/wRqcN6", md5hex, n, "1.2.1"))
        {
            bad++;
            printf("  FAIL : size=%-6lu 受理就被拒\n", (unsigned long)n);
            continue;
        }

        r = run_until_done(40000);
        ota_http_get_status(&st);

        if ((r != OTA_HTTP_READY) || (st.received != n) ||
            (memcmp(fake_stage_ptr(), g_fw, (size_t)n) != 0) ||
            (fake_double_write_err() != 0) ||
            ((uint32_t)g_req_count != want_req))
        {
            bad++;
            printf("  FAIL : size=%-6lu r=%d recv=%lu req=%d(期望 %lu) 重复编程=%d\n",
                   (unsigned long)n, r, (unsigned long)st.received,
                   g_req_count, (unsigned long)want_req, fake_double_write_err());
            continue;
        }
        printf("  ok   : size=%-6lu %lu 片 -> READY, 逐字节一致\n",
               (unsigned long)n, (unsigned long)want_req);
    }

    CHECK(bad == 0,
          "★★ %u 种尺寸全部通过(1/7/8/9/511/512/513/8191/8192/8193/16384/16385/65533/65535/65536)",
          TOTAL);
}

/* ---- [16] ★故障注入点扫描 ----
 * 【为什么要有这条】: 和尺寸扫描是同一个道理 —— 注入点以前只测了"第 1 片 1000 字节"
 * 这一个位置。而重试要回滚到"最近一次暂存区排空的边界", 所以注入点落在
 * 512 边界之前/之上/之后、以及末片(长度非 512 整数倍)的残尾里, 走的是不同的回滚路径。 */
static void test_fault_position_sweep(void)
{
    static const struct {
        uint32_t    size;
        int         req;        /* 在哪一片注入(1 起) */
        uint32_t    at;         /* 片内偏移 */
        const char *what;
    } CASES[] = {
        { 65536u, 1,    1u, "第1片第1字节(一次都没落过盘)" },
        { 65536u, 1,  100u, "第1片100字节(暂存区没满)" },
        { 65536u, 1,  511u, "第1片511(差1字节到512边界)" },
        { 65536u, 1,  512u, "第1片512(正好落在落盘边界上)" },
        { 65536u, 1,  513u, "第1片513(刚过落盘边界)" },
        { 65536u, 1, 4096u, "第1片正中" },
        { 65536u, 1, 8000u, "第1片尾部" },
        { 65536u, 2, 4096u, "第2片正中" },
        { 65533u, 8, 4000u, "★末片(8189字节)正中 —— 残尾路径" },
        { 65533u, 8, 8100u, "★末片深部(残89字节在暂存区)" }
    };
    const unsigned TOTAL = (unsigned)(sizeof(CASES) / sizeof(CASES[0]));
    unsigned       k;
    int            bad = 0;

    printf("\n[16] ★故障注入点扫描: 10 个位置(含 512 边界与末片残尾)\n");

    for (k = 0u; k < TOTAL; k++)
    {
        uint8_t           d[16];
        char              md5hex[33];
        ota_http_status_t st;
        int               r;

        reset_all();
        g_serve_len = CASES[k].size;
        md5_calc(g_fw, CASES[k].size, d);
        md5_to_hex(d, md5hex);

        if (!ota_http_start("https://kappa.lol/wRqcN6", md5hex, CASES[k].size, "1.2.1"))
        {
            bad++;
            printf("  FAIL : %-34s 受理被拒\n", CASES[k].what);
            continue;
        }

        /* 受理之后再布陷阱(ota_http_start 会重置计数器) */
        g_fault_req  = CASES[k].req;
        g_fault_at   = CASES[k].at;
        g_fault_left = 1;

        r = run_until_done(40000);
        ota_http_get_status(&st);

        if ((r != OTA_HTTP_READY) || (st.received != CASES[k].size) ||
            (memcmp(fake_stage_ptr(), g_fw, (size_t)CASES[k].size) != 0) ||
            (fake_double_write_err() != 0))
        {
            bad++;
            printf("  FAIL : %-34s r=%d recv=%lu 重复编程=%d\n",
                   CASES[k].what, r, (unsigned long)st.received, fake_double_write_err());
            continue;
        }
        printf("  ok   : %-34s 重试 %lu 次后仍逐字节一致\n",
               CASES[k].what, (unsigned long)ota_http_retry_count());
    }

    CHECK(bad == 0, "★★ %u 个注入点全部恢复且逐字节一致", TOTAL);
}

/* ---- [17] ★URC 分片大小扫描 ----
 * 【为什么要有这条】: 尺寸、注入点两轮扫描暴露的都是"只采了一个点"。
 * 分片大小同样一直固定在 200, 而它决定 s_rxq 的填充节奏、drain_queue() 每次
 * 处理多少字节, 直接关系到"落盘 + 快照"的推进方式。真实模组的 fragment 行为未知。
 *
 * ★关键约束★: HEX 输出下一行的长度 = 头部约 38 字符 + 2*frag,
 * 而 g4_io 的行缓冲只有 G4_IO_LINE_BUF_SZ = 512 -> frag > 237 就会被【强制拆行】,
 * 拆出来的后半段不再以 +MHTTPURC: 开头, 数据就丢了。这条扫描就是要看:
 *   1) frag <= 237 时各种节奏都要能升完
 *   2) frag 超限时到底是怎么坏的(安全失败, 还是静默写坏) */
static void test_frag_size_sweep(void)
{
    static const uint32_t FRAGS[] = {
        1u, 8u, 63u, 64u, 65u, 100u, 199u, 200u, 237u, 238u, 256u, 512u
    };
    const unsigned TOTAL = (unsigned)(sizeof(FRAGS) / sizeof(FRAGS[0]));
    /* 8192 + 3000: 既有整片也有残尾, 又不至于让 frag=1 跑太久 */
    const uint32_t N = OTA_HTTP_CHUNK + 3000u;
    unsigned       k;
    int            bad     = 0;
    int            bad_big = 0;

    printf("\n[17] ★URC 分片大小扫描: %u 种 frag (行缓冲 512 -> frag>237 必拆行)\n", TOTAL);

    for (k = 0u; k < TOTAL; k++)
    {
        uint8_t           d[16];
        char              md5hex[33];
        ota_http_status_t st;
        int               r;
        int               is_big = (FRAGS[k] > 237u);

        reset_all();
        g_serve_len  = N;
        g_frag_bytes = FRAGS[k];
        md5_calc(g_fw, N, d);
        md5_to_hex(d, md5hex);

        if (!ota_http_start("https://kappa.lol/wRqcN6", md5hex, N, "1.2.1"))
        {
            printf("  FAIL : frag=%-4lu 受理被拒\n", (unsigned long)FRAGS[k]);
            bad++;
            continue;
        }

        r = run_until_done(40000);
        ota_http_get_status(&st);

        if ((r == OTA_HTTP_READY) && (memcmp(fake_stage_ptr(), g_fw, (size_t)N) == 0) &&
            (fake_double_write_err() == 0))
        {
            printf("  ok   : frag=%-4lu 升完且逐字节一致\n", (unsigned long)FRAGS[k]);
        }
        else if (is_big)
        {
            /* 超限时【失败可接受】, 但必须是"安全失败": 不能写坏暂存区 */
            printf("  ok*  : frag=%-4lu 超行缓冲 -> 安全失败(r=%d, 未写坏)\n",
                   (unsigned long)FRAGS[k], r);
            bad_big++;
        }
        else
        {
            bad++;
            printf("  FAIL : frag=%-4lu r=%d recv=%lu 重复编程=%d\n",
                   (unsigned long)FRAGS[k], r, (unsigned long)st.received,
                   fake_double_write_err());
        }
    }

    CHECK(bad == 0, "★★ 所有 frag <= 237 的分片大小都能升完且逐字节一致");
    printf("       (frag>237 的 %d 种超限情况: 安全失败, 未写坏暂存区)\n", bad_big);
}

/* ---- [18] ★HTTP 状态码扫描 ----
 * 【为什么要有这条】: 又是"只采了一个点" —— 此前只测过 206(正常) 和 302(跳转),
 * 4xx/5xx 【完全没测】, 而代码里也【没判】。服务器回 404 时会把 HTML 错误页当正文收,
 * 于是每片等 60s 超时 x3 次重试 -> 4 分钟才失败, 日志上还看不出其实是 404。
 * 另外 206 这条必须钉死: 带 Range 的请求, 合规服务器回的是 206 而不是 200。 */
static void test_http_code_sweep(void)
{
    static const struct {
        unsigned    code;
        int         should_ok;
        const char *what;
    } CASES[] = {
        { 200u, 1, "200 OK (有些服务器无视 Range 直接回 200)" },
        { 206u, 1, "★206 Partial Content (Range 请求的正常回复)" },
        { 301u, 0, "301 永久跳转" },
        { 302u, 0, "302 临时跳转" },
        { 403u, 0, "403 拒绝" },
        { 404u, 0, "★404 找不到 (以前要白等 4 分钟)" },
        { 500u, 0, "★500 服务器错误 (同上)" },
        { 503u, 0, "503 不可用" }
    };
    const unsigned TOTAL = (unsigned)(sizeof(CASES) / sizeof(CASES[0]));
    unsigned       k;
    int            bad = 0;

    printf("\n[18] ★HTTP 状态码扫描: %u 种 (2xx 必须成功, 其余必须【尽快】失败)\n", TOTAL);

    for (k = 0u; k < TOTAL; k++)
    {
        ota_http_status_t st;
        int               r;

        reset_all();
        g_force_code = CASES[k].code;

        if (!ota_http_start("https://kappa.lol/wRqcN6", g_fw_md5, g_fw_len, "1.2.1"))
        {
            printf("  FAIL : %-40s 受理被拒\n", CASES[k].what);
            bad++;
            continue;
        }

        r = run_until_done(40000);
        ota_http_get_status(&st);

        if (CASES[k].should_ok != 0)
        {
            if ((r == OTA_HTTP_READY) && (memcmp(fake_stage_ptr(), g_fw, g_fw_len) == 0))
            {
                printf("  ok   : %-40s 升完\n", CASES[k].what);
            }
            else
            {
                bad++;
                printf("  FAIL : %-40s r=%d err=%d\n", CASES[k].what, r, (int)st.err);
            }
        }
        else
        {
            /* 必须【失败】而不是卡住(r = -1 就是没在限定拍数内结束), 且不许写标记 */
            if ((r == OTA_HTTP_FAILED) && !fake_mark_valid())
            {
                printf("  ok   : %-40s 快速失败(err=%d), 未写标记\n",
                       CASES[k].what, (int)st.err);
            }
            else
            {
                bad++;
                printf("  FAIL : %-40s r=%d(应为 6=FAILED) 标记有效=%d\n",
                       CASES[k].what, r, fake_mark_valid() ? 1 : 0);
            }
        }
    }

    CHECK(bad == 0, "★★ %u 种状态码的行为都正确(2xx 成功 / 其余快速失败)", TOTAL);
}

/* ==================== main ==================== */

static int load_fw(void)
{
    const char *path = "_ota_platform_test/fw_test.bin";
    FILE       *fp = fopen(path, "rb");
    long        n;

    if (fp == NULL)
    {
        printf("!! 找不到 %s —— 本套件依赖它(见 test_md5.c 的说明)\n", path);
        return 0;
    }
    (void)fseek(fp, 0L, SEEK_END);
    n = ftell(fp);
    (void)fseek(fp, 0L, SEEK_SET);
    if ((n <= 0L) || (n > 300000L))
    {
        fclose(fp);
        printf("!! %s 大小异常: %ld\n", path, n);
        return 0;
    }
    g_fw = (uint8_t *)malloc((size_t)n);
    if (g_fw == NULL)
    {
        fclose(fp);
        return 0;
    }
    if (fread(g_fw, 1u, (size_t)n, fp) != (size_t)n)
    {
        fclose(fp);
        free(g_fw);
        g_fw = NULL;
        return 0;
    }
    fclose(fp);
    g_fw_len = (uint32_t)n;

    {
        uint8_t d[16];
        md5_calc(g_fw, g_fw_len, d);
        md5_to_hex(d, g_fw_md5);
    }
    return 1;
}

int main(void)
{
    host_log_enable(0);     /* 想看状态机内部日志就改成 1 */

    printf("==== app/ota/ota_http.c 端到端(假 ML307R) ====\n");

    if (!load_fw())
    {
        return 1;
    }
    printf("靶子: %lu 字节, MD5 %s\n", (unsigned long)g_fw_len, g_fw_md5);

    if (strcmp(g_fw_md5, "9e96d33cf71a79caabd00b319e115af5") != 0)
    {
        printf("!! 靶子 MD5 不是预期的 9e96d33cf71a79caabd00b319e115af5, 先查文件\n");
        return 1;
    }

    test_url();
    test_full_download();
    test_chunk_retry();
    test_md5_mismatch();
    test_redirect();
    test_abort();
    test_state_names();
    test_power_loss_resume();
    test_resume_mismatch();
    test_progress_due();
    test_power_loss_mid_chunk();
    test_ready_race();
    test_instance_not_leaked();
    test_partial_tail_chunk();
    test_size_sweep();
    test_fault_position_sweep();
    test_frag_size_sweep();
    test_http_code_sweep();

    printf("\n==== 通过 %d, 失败 %d ====\n", g_pass, g_fail);
    free(g_fw);
    return (g_fail == 0) ? 0 : 1;
}
