/**
 * \file  ota_http.c
 * \brief OTA over HTTP(S) 下载状态机 —— 见 ota_http.h 的架构说明
 *
 * ============================ 三条硬约束(踩过才知道) ============================
 *
 * 1) 【URC 钩子在收包泵里跑, 绝不能在那里写 Flash】
 *    g4_io_pump() -> g4_io_process_line() -> g4_urc_input_line() -> 本文件钩子。
 *    在那里面调 ota_port_flash_write() 会 __disable_irq() + 等 Flash 编程完成,
 *    把泵卡住, rb_g4(8192 B) 有被灌爆的风险。
 *    → 所以钩子只做"解 HEX + 塞进 s_rxq", 写盘/喂 MD5 全部留给 ota_http_poll()。
 *      这与 g4_urc.h 里"publish 入队, 主循环投递"是同一个套路。
 *
 * 2) 【片边界必须天然 8 字节对齐】
 *    ota_port_flash_write() 内部按 8 字节补 0xFF, 被补过的 double word 不能
 *    再写第二次(Flash 只能 1->0)。所以暂存区只在攒满 512(是 8 的倍数)时下盘,
 *    而 OTA_HTTP_CHUNK 取 8192 = 16 x 512, 于是每片收完暂存区正好空,
 *    s_written 恒为 8 字节对齐, 下一片的 Range 偏移就是它。
 *
 * 3) 【失败重试要能回滚 MD5】
 *    MD5 是流式的, 算进去就退不回来。所以每片开始前把 md5_ctx_t 和 CRC32
 *    各存一份快照(s_md5_snap / s_crc_snap), 重试时恢复 —— md5_ctx_t 92 字节,
 *    存两份的代价可以忽略, 换来"片内任意点失败都能干净重来"。
 */

#include "ota_http.h"

#include <string.h>
#include <stdio.h>

#include "g4_io.h"
#include "g4_urc.h"
#include "ota_port.h"
#include "md5.h"
#include "flash_map.h"
#include "log.h"

/* ==================== 编译期自检 ==================== */

#if (OTA_HTTP_CHUNK % OTA_HTTP_STAGE_SZ) != 0u
#error "OTA_HTTP_CHUNK 必须是 OTA_HTTP_STAGE_SZ 的整数倍(片边界要对齐到 8 字节)"
#endif
#if (OTA_HTTP_STAGE_SZ % 8u) != 0u
#error "OTA_HTTP_STAGE_SZ 必须是 8 的倍数(Flash double word)"
#endif
#if (OTA_HTTP_CHUNK % 8u) != 0u
#error "OTA_HTTP_CHUNK 必须是 8 的倍数"
#endif

/* ★ 分片大小必须让整条 HEX URC 塞进 g4_io 的行缓冲 ★
 *
 * 2026-10-02 主机扫描(frag = 1/8/63/64/65/100/199/200/237/238/256/512)实测:
 *   frag <= 237  -> 全部能升完
 *   frag >= 238  -> g4_io 把行【强制拆开】, 后半段不再以 +MHTTPURC: 开头,
 *                   数据直接丢掉
 * 幸好拆行时是【安全失败】(收不满 -> 片超时 -> 3 次重试 -> 整单失败,
 * 暂存区没有被写坏), 不是静默损坏。但那个代价很难看:
 *   4 次尝试 x 60s 片超时 ≈ 4 分钟才失败一次。
 * 所以把这个约束提到编译期 —— 配错了直接编不过, 而不是现场跑 4 分钟才知道。
 *
 * 行格式: +MHTTPURC: "content",<id>,<total>,<offset>,<len>,<hex...>
 * 头部最坏情况(三个 10 位数)约 56 字符, 再加 2*frag 个 HEX 字符和结尾 NUL。
 * 这里按 64 留余量, 所以 frag 上限是 (512-64)/2 = 224。 */
#if ((OTA_HTTP_FRAG_SIZE * 2u) + 64u) > G4_IO_LINE_BUF_SZ
#error "OTA_HTTP_FRAG_SIZE 太大: HEX 行(2*frag+~64)会超过 G4_IO_LINE_BUF_SZ 而被强制拆行, 数据丢失"
#endif

/* ==================== 内部类型 ==================== */

typedef struct
{
    char     scheme[6];                 /* "http" / "https" */
    char     host[OTA_HTTP_URL_MAX / 2];/* host 或 host:port 里的 host 部分 */
    char     path[OTA_HTTP_URL_MAX / 2];/* 以 '/' 开头的 path */
    uint16_t port;                      /* 0 = 用默认端口(scheme 决定) */
    bool     tls;                       /* scheme == https */
} ota_http_url_t;

/* DOWNLOADING 的子步骤 */
typedef enum
{
    DL_SEND_HEADER = 0,     /* 发 AT+MHTTPHEADER(带 Range) */
    DL_SEND_REQUEST,        /* 发 AT+MHTTPREQUEST */
    DL_WAIT_DATA            /* 等 content URC 流 */
} ota_http_dl_step_t;

/* ==================== 内部状态 ==================== */

/* --- 会话参数(来自 ota_http_start) --- */
static char     s_url_raw[OTA_HTTP_URL_MAX];
static char     s_md5hex[33];
static char     s_version[24];
static uint32_t s_size;
static ota_http_url_t s_url;

/* --- 对外可见状态 --- */
static ota_http_state_t s_state;
static ota_http_err_t   s_err;

/* --- 实例 id(MHTTPCREATE 返回) --- */
static uint16_t s_httpid;
static bool     s_httpid_valid;     /* 本会话确实建成了实例(才需要归还) */

/* --- 实例归还(AT+MHTTPDEL) ---
 * ★ 为什么必须有 ★ —— 2026-10-02 复查时发现的资源泄漏:
 *   手册 §3.2: "最多可同时建立 4 个实例"(id 0~3), 而 MHTTPCREATE 每次调用都会
 *   占一个新的。早期实现从头到尾【没发过 AT+MHTTPDEL】, 于是:
 *     · 每完成/放弃一次升级, 就漏掉一个实例; 第 5 次 MHTTPCREATE 直接失败
 *     · 而 g4_at_init() 只是清软件状态, 【不给模组断电】—— 所以这个泄漏
 *       【跨 MCU 复位累积】, 只有链路恢复时的 4G_RST 断电才会清掉
 *   表现: "前几次升级好好的, 后来 ota.start 永远失败", 极难定位。
 * 做成"待办"而不是在 abort()/校验成功处直接发 AT: 协议层不该碰 AT,
 * 统一由 ota_http_poll() 按节拍推进(那里本来就是非阻塞的 AT 状态机)。 */
static uint16_t s_del_id;           /* 要归还哪个实例 */
static bool     s_del_needed;       /* 欠一条 AT+MHTTPDEL */

/* --- 连接阶段的命令序列 --- */
static uint8_t s_conn_step;

/* --- 擦除 --- */
static bool s_erase_started;

/* --- 下载 --- */
static ota_http_dl_step_t s_dl_step;
static uint32_t s_chunk_off;        /* 本片起始偏移(= 收片前的 s_written) */
static uint32_t s_chunk_len;        /* 本片希望收到的字节数 */
static uint32_t s_chunk_got;        /* 本片已收字节数 */
static uint32_t s_chunk_retry;      /* 本片已重试次数 */
static uint32_t s_retry_total;      /* 累计重试次数(诊断) */
static uint32_t s_rx_total;         /* 全程已收字节数(含尚未下盘的) */
static uint32_t s_written;          /* 已写进 Flash 的字节数(8 字节对齐) */
static uint32_t s_resume_from;      /* 断点续传起点(页对齐; 0 = 从头下)         */
static uint32_t s_saved_written;    /* 上次落盘断点时的 s_written(决定何时再存) */
static uint32_t s_tick_rx;          /* 最近一次收到 content 的时刻 */
static bool     s_chunk_bad;        /* 本片出过错, poll 里做重试 */
static bool     s_got_header;       /* 本片已见过 header URC */
static uint16_t s_http_code;        /* 本片 HTTP 状态码 */
static uint32_t s_srv_clen;         /* 服务器声明的 content 长度 */

/* ★2026-10-07 诊断: 前 3 条 content URC 的字段/字节数只打一次(见 urc_content 尾部)★
 * 必须声明在 urc_content() 【之前】。 */
static uint8_t  s_content_dbg;

/* ★2026-10-07: 下载阶段 "653 客户端忙" 的容忍参数(见 dl_cmd_busy_retry)★
 * 现场: 第一片收满后 19ms 就发下一片的 HEADER, 模组还忙着 -> 653。
 * 等 300ms 重发即可; 连续 10 次还忙就认为真出问题, 交给整片重试。 */
#ifndef OTA_HTTP_BUSY_RETRY_MS
#define OTA_HTTP_BUSY_RETRY_MS    300u
#endif
#ifndef OTA_HTTP_BUSY_RETRY_MAX
#define OTA_HTTP_BUSY_RETRY_MAX   10u
#endif
static uint32_t s_busy_hold_tick;   /* 非 0 = 正在等这个时刻之后再重发 */
static uint8_t  s_busy_retry;       /* 本次连续遇到 653 的次数 */

/* --- 校验累加器(含快照, 见文件头约束 3) ---
 * 快照点跟着【下盘边界】走, 不是片边界: 这样重试时 Flash 上已写的部分一个字节
 * 都不需要重写(Flash 只能 1->0, 重写同一 double word 虽然同值无害, 但没必要
 * 去踩那个边界)。s_snap_written 就是"快照对应的 Flash 已写字节数"。 */
static md5_ctx_t s_md5;
static uint32_t  s_crc;
static md5_ctx_t s_md5_snap;
static uint32_t  s_crc_snap;
static uint32_t  s_snap_written;

/* --- 对齐暂存 --- */
static uint8_t  s_stage[OTA_HTTP_STAGE_SZ];
static uint16_t s_stage_len;

/* --- URC -> poll 的字节队列(见文件头约束 1) ---
 * 大小依据: g4_io_pump() 一次会把 rb_g4(8192 B) 抽干, 折成 HEX 后最多约
 * 4096 字节有效数据, 所以 4096 能兜住一次泵; 真溢出也不丢数据 —— 标 s_rxq_ovf
 * 让 poll 重试整片(有快照可回滚)。 */
#define OTA_HTTP_RXQ_SZ  4096u
static uint8_t  s_rxq[OTA_HTTP_RXQ_SZ];
static uint16_t s_rxq_len;
static bool     s_rxq_ovf;

/* ==================== 前向声明 ==================== */
/* poll_connecting 收尾时要靠它划出第一片(定义在下面「下载」一节) */
static void chunk_begin(void);

/* ==================== 小工具 ==================== */

static uint32_t now_ms(void)
{
    return ota_port_get_tick_ms();
}

static void set_state(ota_http_state_t st)
{
    s_state = st;
}

static uint16_t s_del_id;           /* 下一个要归还的实例 id */
static uint16_t s_del_last;         /* 最后一个(含) —— 支持一次归还一段 */
static bool     s_del_needed;       /* 还欠着 AT+MHTTPDEL */

/** @brief 登记"归还 [first, last] 这一段实例"
 *  @note  只置标志、不发 AT —— 由 ota_http_poll() 按节拍推进, 这样协议层
 *         (abort) 和收尾路径都不必阻塞, 也不必自己去碰 AT 层。
 *         已经有待办时取并集: 多删几个不存在的实例只会多几条 CME ERROR,
 *         比漏删安全得多。 */
static void teardown_request_range(uint16_t first, uint16_t last)
{
    if (!s_del_needed)
    {
        s_del_id     = first;
        s_del_last   = last;
        s_del_needed = true;
        return;
    }
    if (first < s_del_id)   { s_del_id   = first; }
    if (last  > s_del_last) { s_del_last = last;  }
}

/** @brief 登记归还在用的那个实例(本会话建过才登记) */
static void teardown_request(void)
{
    if (s_httpid_valid)
    {
        teardown_request_range(s_httpid, s_httpid);
        s_httpid_valid = false;     /* 只登记一次 */
    }
}

static void fail(ota_http_err_t e, const char *why)
{
    s_err = e;
    s_state = OTA_HTTP_FAILED;
    /* 【必须归还实例】: 失败时实例可能已经建好了(比如下载失败)。不还就漏一个,
     * 4 次之后 MHTTPCREATE 再也建不起来 —— 见 s_del_needed 处的说明。 */
    teardown_request();
    log_info("[OTA_HTTP] !! 失败(%d): %s\r\n", (int)e, (why != NULL) ? why : "");
}

/** @brief 从 ota.start 的 url 里拆出 scheme/host/port/path 并校验
 *  @return true = 合法 */
static bool url_parse(const char *url, ota_http_url_t *out)
{
    const char *p;
    const char *host_end;
    const char *path;
    uint16_t    n;

    if ((url == NULL) || (out == NULL))
    {
        return false;
    }
    memset(out, 0, sizeof(*out));

    /* ---- scheme ---- */
    if (strncmp(url, "https://", 8u) == 0)
    {
        (void)memcpy(out->scheme, "https", 6u);
        out->tls = true;
        p = url + 8;
    }
    else if (strncmp(url, "http://", 7u) == 0)
    {
        (void)memcpy(out->scheme, "http", 5u);
        out->tls = false;
        p = url + 7;
    }
    else
    {
        return false;       /* 只认 http/https —— 协议 2001 */
    }

    if (*p == '\0')
    {
        return false;
    }

    /* ---- host[:port] ---- */
    host_end = p;
    while ((*host_end != '\0') && (*host_end != '/') && (*host_end != ':'))
    {
        host_end++;
    }
    n = (uint16_t)(host_end - p);
    if ((n == 0u) || (n >= sizeof(out->host)))
    {
        return false;
    }
    (void)memcpy(out->host, p, n);
    out->host[n] = '\0';

    /* host 里不允许出现空格/控制字符(拼进 AT 命令会破坏语法) */
    {
        uint16_t i;
        for (i = 0u; i < n; i++)
        {
            char c = out->host[i];
            if ((c <= ' ') || (c == '"') || (c == '\\'))
            {
                return false;
            }
        }
    }

    /* ---- :port ---- */
    if (*host_end == ':')
    {
        uint32_t v = 0u;
        host_end++;
        if ((*host_end < '0') || (*host_end > '9'))
        {
            return false;
        }
        while ((*host_end >= '0') && (*host_end <= '9'))
        {
            v = v * 10u + (uint32_t)(*host_end - '0');
            if (v > 65535u)
            {
                return false;
            }
            host_end++;
        }
        out->port = (uint16_t)v;
    }

    /* ---- path ---- */
    path = (*host_end == '/') ? host_end : "/";
    n = (uint16_t)strlen(path);
    if (n >= sizeof(out->path))
    {
        return false;
    }
    (void)memcpy(out->path, path, n);
    out->path[n] = '\0';

    /* path 里不能有引号(直接拼进 AT 命令的字符串参数) */
    if (strchr(out->path, '"') != NULL)
    {
        return false;
    }

    return true;
}

/* ==================== HEX 解码 ==================== */

static int hex_val(char c)
{
    if ((c >= '0') && (c <= '9')) { return (int)(c - '0'); }
    if ((c >= 'a') && (c <= 'f')) { return (int)(c - 'a') + 10; }
    if ((c >= 'A') && (c <= 'F')) { return (int)(c - 'A') + 10; }
    return -1;
}

/* hex_decode() 在 2026-10-02 删除: urc_content() 改成直接解到 s_rxq,
 * 不再需要先解到临时缓冲。hex_val() 仍在用, 保留。 */

/* ==================== 暂存 + 下盘 ==================== */

static bool stage_flush(void)
{
    if (s_stage_len == 0u)
    {
        return true;
    }
    if (!ota_port_flash_write(s_written, s_stage, s_stage_len))
    {
        return false;
    }
    s_written += s_stage_len;
    s_stage_len = 0u;
    return true;
}

/** @brief 把一段字节放进暂存区(**不落盘**)
 *  @note  调用者必须保证 n <= 剩余空间; 何时 flush 由 drain_queue() 决定,
 *         因为"落盘"与"md5/crc 推进"必须成对, 快照才能提交在落盘边界上。 */
static void stage_put(const uint8_t *d, uint16_t n)
{
    (void)memcpy(&s_stage[s_stage_len], d, n);
    s_stage_len = (uint16_t)(s_stage_len + n);
}

/* ==================== URC 解析 ==================== */

/** @brief 跳过一个逗号(可选空白)后取一个十进制数; 失败返回 false */
static bool num_next(const char **pp, uint32_t *out)
{
    const char *p = *pp;
    uint32_t v = 0u;
    bool     any = false;

    while ((*p == ' ') || (*p == '\t')) { p++; }
    if (*p == ',') { p++; }
    while ((*p == ' ') || (*p == '\t')) { p++; }

    while ((*p >= '0') && (*p <= '9'))
    {
        v = v * 10u + (uint32_t)(*p - '0');
        any = true;
        p++;
    }
    if (!any)
    {
        return false;
    }
    *pp = p;
    *out = v;
    return true;
}

/** @brief 取出 tag(引号内), 并把 p 推到 tag 之后 */
static bool tag_take(const char **pp, char *tag, uint16_t tagcap)
{
    const char *p = *pp;
    const char *q;
    uint16_t    n;

    while ((*p == ' ') || (*p == '\t')) { p++; }
    if (*p != '"')
    {
        return false;
    }
    p++;
    q = strchr(p, '"');
    if (q == NULL)
    {
        return false;
    }
    n = (uint16_t)(q - p);
    if (n >= tagcap)
    {
        return false;
    }
    (void)memcpy(tag, p, n);
    tag[n] = '\0';
    *pp = q + 1;
    return true;
}

/** @brief 取剩余部分(数据段) */
static const char *rest_of(const char *p, uint16_t *len)
{
    uint16_t n = (uint16_t)strlen(p);
    *len = n;
    return p;
}

/**
 * @brief 处理 +MHTTPURC 的 content 数据(在收包泵里跑, 只做解 HEX + 入队)
 *
 * 【为什么直接解到 s_rxq 而不先解到临时缓冲】: 本函数跑在收包泵内部, 而泵的调用
 * 链是 g4_at_poll -> g4_io_pump -> g4_io_process_line -> g4_urc_input_line -> 这里。
 * 早期写法在栈上开了 208 字节的 tmp 再 memcpy 进队列 —— 那 208 字节是白占的,
 * 而且在"下行最快、最需要泵跟得上"的时刻正好压栈。直接解到队列尾, 栈和拷贝都省了。
 *
 * 出错时的安全性: 若中途遇到非法 hex, 已经写进 s_rxq 的那几个字节不用回退 ——
 * 置 s_chunk_bad 后 poll 会重试整片, 重试时 s_rxq_len 归零(见 chunk_retry_or_fail)。
 */
static void urc_content(const char *p)
{
    uint32_t id = 0u;
    uint32_t clen = 0u;
    uint32_t sum = 0u;
    uint32_t cur = 0u;
    uint16_t dlen = 0u;
    uint16_t room;
    uint16_t n = 0u;
    uint16_t i;
    const char *data;

    /* ★★★2026-10-07: 没见过本片的 header URC, 就一条数据都不收 ★★★
     *
     * 【现场 19:16:37 —— 这是"字节总数对、内容错"的真正根因】
     * 第一片收到 8192 后, 第二片遇到 653 并重试成功(见 653 那条修复)。但随后
     *   19:16:37:379  !! content 本块长度不符: 模组声称 200, 实际解出 180 字节
     *   19:16:37:411  片重试 1/3 @9728  -> 立刻发新的 HEADER + REQUEST
     *   19:16:37:498  !! content 累计不连续: 模组声称 2119, 按已收应为 200
     *   19:16:37:587  !! content 累计不连续: 模组声称 2319, 按已收应为 200
     *   19:16:37:692  !! content 累计不连续: 模组声称 2519, 按已收应为 200
     * 那 2119/2319/2519 是【上一个请求还没吐完的残留 URC】(每块仍然 +200)。
     *
     * 老代码(没有校验时)把它们当成新片的数据照单全收 -> 镜像整体错位 ->
     * 一直到最后 MD5 才发现(19:10 那次就是这样: 头对尾对总长对, 只有 MD5 错)。
     * 我加了校验之后变成了"立刻重试 -> 又被残留污染 -> 又重试"的死循环,
     * 3 次耗尽后 fail(2002)。
     *
     * 【为什么这一条能干净解决】每个 AT+MHTTPREQUEST 都【必然】先来一条
     *   +MHTTPURC: "header",<id>,<code>,<hlen>
     * (现场每次都看得到, 而且 code=206)。所以"没见过 header 就不收 content"
     * 能把上一个请求的残留整体挡在门外 —— 既不用延时猜时间, 也不用清缓存。
     * chunk_begin() 会把 s_got_header 清成 false, urc_header() 再置 true。 */
    if (!s_got_header)
    {
        return;
    }

    if (!num_next(&p, &id))   { return; }
    if (!num_next(&p, &clen)) { return; }
    if (!num_next(&p, &sum))  { return; }
    if (!num_next(&p, &cur))  { return; }

    /* 数据段紧跟在第 4 个数字后面的逗号之后 */
    while ((*p == ' ') || (*p == '\t')) { p++; }
    if (*p == ',') { p++; }

    data = rest_of(p, &dlen);
    if (dlen == 0u)
    {
        return;
    }

    /* 去掉行尾可能的 CR/空白(行已按 CR/LF 切开, 这里只是保险) */
    while ((dlen > 0u) && ((data[dlen - 1u] == '\r') || (data[dlen - 1u] == ' ')))
    {
        dlen--;
    }
    if ((dlen & 1u) != 0u)
    {
        s_chunk_bad = true;
        log_info("[OTA_HTTP] !! content 段 hex 长度为奇数(%u)\r\n", (unsigned)dlen);
        return;
    }

    room = (uint16_t)(OTA_HTTP_RXQ_SZ - s_rxq_len);

    for (i = 0u; (uint16_t)(i + 1u) < dlen; i = (uint16_t)(i + 2u))
    {
        int hi = hex_val(data[i]);
        int lo = hex_val(data[i + 1u]);

        if ((hi < 0) || (lo < 0))
        {
            /* 片段本身畸形: 标坏, 让 poll 重试整片(有快照可回滚) */
            s_chunk_bad = true;
            log_info("[OTA_HTTP] !! content 段 hex 非法(第 %u 字节处)\r\n", (unsigned)i);
            return;
        }
        if (n >= room)
        {
            /* 队列装不下 —— 丢字节会让镜像错, 只能重试整片 */
            s_rxq_ovf = true;
            return;
        }
        s_rxq[s_rxq_len + n] = (uint8_t)((hi << 4) | lo);
        n++;
    }

    /* ★★2026-10-07: 用 URC 自带的计数校验本块 —— 这是"字节总数对、内容错"的唯一解药★★
     *
     * 【现场 19:10】收到 15444 字节(= content-length, 一个不多一个不少), 头 32B 是
     * 正确的 PNG 签名、尾 16B 是正确的 IEND, 可 MD5 就是不对(算出 dd4a7606...,
     * 云端 d9c8750b...)。也就是说【长度全对、头尾全对, 中间有一块错位】。
     *
     * 【为什么老代码查不出来】它把第 3、4 个数字读进 sum/cur 之后就地 (void) 掉了,
     * 只按【到达顺序】往 s_rxq 后面追加。于是只要 URC 丢一条、重一条、错一条,
     * 或者某一行的 hex 被串口丢字节啃掉一截, 后面的数据就会【整体前移/后移】——
     * 而字节总数是完全可能仍然凑得上的, 只有最后的 MD5 才暴露出来。
     *
     * 【现在】用模组自己给的计数做两道校验(实测字段语义, 见 urc_content 上面的诊断):
     *   +MHTTPURC: "content",<id>,<本片总长>,<累计已发>,<本块长度>,<hex...>
     *     content#1: n2=8192 n3=200 n4=200   本片累计 200
     *     content#2: n2=8192 n3=400 n4=200   本片累计 400   -> n3 是【累计】
     *   ① 本块长度(n4) 必须等于我们实际解出的字节数 -> 不符说明这一行的 hex 被截断/损坏
     *   ② 累计(n3) 必须等于"本片已收 + 本块" -> 不符说明丢了/重了/乱序了
     * 任一不符就标 s_chunk_bad, 由 poll 回滚本片重来 —— 宁可贵一次网络,
     * 也不能把错位的镜像写进 Flash 然后 MD5 才发现(那时已经白下了整片)。 */
    if ((uint32_t)n != cur)
    {
        log_info("[OTA_HTTP] !! content 本块长度不符: 模组声称 %lu, 实际解出 %u 字节 "
                 "(hex 段 %u 字符) —— 行被截断/串口丢字节, 本片重来\r\n",
                 (unsigned long)cur, (unsigned)n, (unsigned)dlen);
        s_chunk_bad = true;
        return;
    }
    if (sum != (s_chunk_got + (uint32_t)n))
    {
        log_info("[OTA_HTTP] !! content 累计不连续: 模组声称 %lu, 按已收应为 %lu "
                 "(本块 %u 字节) —— URC 丢/重/乱序, 本片重来\r\n",
                 (unsigned long)sum, (unsigned long)(s_chunk_got + (uint32_t)n),
                 (unsigned)n);
        s_chunk_bad = true;
        return;
    }

    s_rxq_len = (uint16_t)(s_rxq_len + n);
    s_srv_clen = clen;
    s_chunk_got += (uint32_t)n;
    s_tick_rx = now_ms();

    /* ★2026-10-07 诊断: 前 3 条 content URC 把四个数字字段和被解码的字节数打出来★
     *
     * 【为什么必须打】设计文档 §9.1 把"fragment 是否对 content URC 生效"列为
     * 【阻塞架构拍板】的未验证项, 而且这四字段的真实语义(id/total/offset/len)
     * 从手册节选里看不出来 —— 本函数的字段映射就是照着设计文档猜的。
     * 现场(18:45) 出现"字节数完全正确(15444 = content-length) 但 MD5 不符",
     * 那就必须先确认①字段到底怎么读 ②每块是不是真的 200 字节 ③offset 是不是 +200。
     * 只打前 3 条, 不刷屏。 */
    if (s_content_dbg < 3u)
    {
        s_content_dbg++;
        log_info("[OTA_HTTP] content#%u: id=%lu n2=%lu n3=%lu n4=%lu | 解出 %u 字节 "
                 "(hex 段 %u 字符), 本片累计 %lu\r\n",
                 (unsigned)s_content_dbg,
                 (unsigned long)id, (unsigned long)clen,
                 (unsigned long)sum, (unsigned long)cur,
                 (unsigned)n, (unsigned)dlen, (unsigned long)s_chunk_got);
    }

    (void)sum;
    (void)cur;
    (void)id;
}

/** @brief 处理 +MHTTPURC 的 header: 抓 HTTP 状态码 */
static void urc_header(const char *p)
{
    uint32_t id = 0u;
    uint32_t code = 0u;
    uint32_t hlen = 0u;

    if (!num_next(&p, &id))   { return; }
    if (!num_next(&p, &code)) { return; }
    if (!num_next(&p, &hlen)) { return; }

    s_http_code = (uint16_t)code;
    s_got_header = true;

    log_info("[OTA_HTTP] header code=%u hlen=%u\r\n",
             (unsigned)code, (unsigned)hlen);

    /* 【跨域跳转必须判死】
     * AT+MHTTPCREATE 绑定的是一台 host。服务器回 3xx 且 Location 指向别的
     * 域名时, 模组不会跨域跟过去 —— 只会把重定向页面当正文吐给我们。
     * 这类平台(GitHub Release / Gitee raw)从一开始就不能选, 见设计文档 §2。 */
    if ((code >= 300u) && (code < 400u))
    {
        s_chunk_bad = true;
        log_info("[OTA_HTTP] !! 服务器回 %u 跳转 —— 本模组的实例绑定单一 host, "
                 "跨域跳转取不到固件; 请换直链平台\r\n", (unsigned)code);
        return;
    }

    /* 【非 2xx 一律判死】—— 2026-10-02 复查时补的。
     *
     * 原本只判了 3xx, 4xx/5xx 是漏的。服务器回 404/500 时会把它那个 HTML
     * 错误页当正文收下来, 然后:
     *   · 错误页比片小 -> 片永远收不满 -> 每片等 60s 超时 x3 次重试
     *   · 错误页比片大 -> s_rx_total 对不上 -> 失败
     * 两种都"最终会失败", 但要走【4 分钟】, 而且日志上完全看不出其实是 404。
     * 直接判掉: 秒级失败, 而且把状态码原样打出来。
     *
     * ★ 200 和 206 都要接受 ★ —— 我们每片都带 Range 头, 合规服务器回的是
     * 【206 Partial Content】而不是 200。假模组一直只回 200, 所以这条路径
     * 此前【一次都没被验证过】: 真机联调时若只认 200, 会变成"每一片都失败"。 */
    if ((code < 200u) || (code >= 300u))
    {
        s_chunk_bad = true;
        log_info("[OTA_HTTP] !! 服务器回 %u (非 2xx) —— 取不到固件\r\n",
                 (unsigned)code);
    }
}

/** @brief 处理 +MHTTPURC 的 err */
static void urc_err(const char *p)
{
    uint32_t id = 0u;
    uint32_t ec = 0u;

    if (!num_next(&p, &id)) { return; }
    if (!num_next(&p, &ec)) { return; }

    /* 错误码表见《HTTP_HTTPS 用户手册 V4.0.0》§3.10 */
    log_info("[OTA_HTTP] !! 模组报错 error_code=%u (%s)\r\n", (unsigned)ec,
             (ec == 1u)  ? "域名解析失败" :
             (ec == 2u)  ? "连接服务器失败" :
             (ec == 3u)  ? "连接服务器超时" :
             (ec == 4u)  ? "SSL握手失败 -> 查 ssl_id / 证书 / MSSLCFG" :
             (ec == 5u)  ? "连接异常断开" :
             (ec == 6u)  ? "请求响应超时" :
             (ec == 7u)  ? "接收数据解析失败" :
             (ec == 8u)  ? "缓存空间不足" :
             (ec == 9u)  ? "数据丢包" :
             (ec == 10u) ? "写文件失败" : "未知");

    s_chunk_bad = true;
}

bool ota_http_on_urc(const char *line)
{
    char        tag[16];
    const char *p;

    if (line == NULL)
    {
        return false;
    }
    p = strstr(line, "+MHTTPURC:");
    if (p == NULL)
    {
        return false;
    }
    p += 10;    /* strlen("+MHTTPURC:") */

    /* 只在下载阶段关心这些 URC; 其它阶段看日志就够了 */
    if (!tag_take(&p, tag, sizeof(tag)))
    {
        return true;
    }

    if (strcmp(tag, "content") == 0)
    {
        if (s_state == OTA_HTTP_DOWNLOADING)
        {
            urc_content(p);
        }
    }
    else if (strcmp(tag, "header") == 0)
    {
        if (s_state == OTA_HTTP_DOWNLOADING)
        {
            urc_header(p);
        }
    }
    else if (strcmp(tag, "err") == 0)
    {
        urc_err(p);
    }
    else
    {
        /* recv / ind: 记一行日志即可(片完成判据用 sum==clen / 字节数) */
        log_info("[OTA_HTTP] %s\r\n", line);
    }

    return true;
}

/* ==================== 命令序列(连接阶段) ==================== */

/**
 * @brief 在连接阶段推进一条 CFG 命令
 * @return true = 这一条完成(可以走下一步); false = 还在途
 * @note  出错时置 FAILED 并返回 true(让调用方跳出)
 */
/* ★2026-10-07 安全网: "本步的命令到底发出去过没有"★
 *
 * 【为什么必须单独立这个闩锁】conn_cmd() 只回 true/false, 而"true"必须意味着
 * "这一条配置命令【真的发出去并且模组应答了】"。可是引擎有 IDLE/RUNNING/DONE
 * 三态, "拿到一个终态结果"并不自动等于"这是我刚发出去的那条命令的应答"。
 *
 * 【现场代价】2026-10-07 16:06 的升级日志里, CONNECTING 的第 1 步
 * (AT+MHTTPCFG="ssl",1,1,0)【整条没发出去】, 而状态机照样推进到了第 2 步:
 *     16:06:10:992 [G4_TX] AT+MHTTPCFG="cached",1,0     <- 直接从 cached 开始
 *     ...(encoding / fragment / timeout 都在, 唯独没有 ssl)
 *     16:06:11:142 [G4_TX] AT+MHTTPREQUEST=1,1,0,"/vlRcjZ.bin"
 *     16:06:11:180 [G4_RX] +CME ERROR: 655              <- 655 = SSL 未使用
 * 实例上没配 ssl, 拿 https 去请求当然被拒 —— 升级就是这么失败的。
 *
 * 所以本步只要【没把命令写出去】, 就一律不推进, 并且大声报出来。 */
static bool s_cfg_sent;             /* 当前 CONNECTING 步骤内, 命令是否已真的发出 */
static bool s_create_sent;          /* 同上, 专给 AT+MHTTPCREATE 那一步 */

/* ★2026-10-07: "等引擎"的可见性计时(见 ota_http_poll 开头)★ */
static uint32_t s_eng_wait_tick;
static bool     s_eng_wait_logged;

/* 下载到 100% 的那一刻 latch 一个事件, 由 app_uplink 强制补报 verifying/100%% */
static bool     s_dl_done_pending;

/* ★2026-10-08: CONNECTING 每条 CFG 的"连续超时重发"计数(见 conn_cmd 的超时分支)★
 * 现场教训: AT+MHTTPCFG="ssl" 超时后老代码只打"命令超时(继续)"就往下走,
 * 结果 ssl 根本没配上 -> 后面 https 请求回 +CME ERROR: 655(SSL 未使能) -> 白跑一轮。 */
#ifndef OTA_HTTP_CFG_RETRY_MAX
#define OTA_HTTP_CFG_RETRY_MAX   3u
#endif
static uint8_t  s_cfg_retry;

static bool conn_cmd(const char *cmd, const char *kw)
{
    /* ★2026-10-07: 用【带属主】的版本★ —— 见 g4_io.h 里"命令引擎的属主"整段说明。
     * 不加属主的话, 引擎上的命令会被 g4_at 的发布子流程"偷结果":
     *   我们发一条 CFG(超时 10s), 发布流程来 poll, 到点拿到【我们的】TIMEOUT
     *   当成它自己"等 '>' 超时" -> 连 3 次 -> 判链路已断 -> 4G_RST 断电重启模组
     *   -> 我们的实例没了 -> 后面每条 CFG 都回 +CME ERROR: 652 -> 失败(2002)。
     * 现场日志(log/新建 文本文档.txt) 2026-10-07 15:40 完整复现了这一串。 */
    uint32_t     tx0 = g4_io_cmd_tx_seq();
    g4_cmd_res_t r   = g4_io_cmd_run_owned(cmd, kw, OTA_HTTP_CMD_TMO_MS, G4_OWNER_OTA);

    if (r == G4_CMD_RUNNING)
    {
        /* 命令这一刻发出去没有? 发出去了就把闩锁立起来 —— 之后的终态才算我们的 */
        if (g4_io_cmd_tx_seq() != tx0) { s_cfg_sent = true; }
        return false;
    }

    /* ---- 拿到终态了。先确认它【确实是本步发出去的那条命令】的应答 ---- */
    if (!s_cfg_sent)
    {
        log_info("[OTA_HTTP] !! conn_cmd 拿到结果(%d)但本步命令没发出去, "
                 "不推进步骤, 下一拍重发: %s\r\n", (int)r, cmd);
        return false;               /* 不推进: 让本步重来 */
    }
    s_cfg_sent = false;             /* 本步用掉了这个闩锁 */

    if ((r != G4_CMD_OK) && (r != G4_CMD_TIMEOUT))
    {
        fail(OTA_HTTP_E_DOWNLOAD, "建立 HTTP 实例/配置失败");
        return true;
    }
    if (r == G4_CMD_TIMEOUT)
    {
        /* ★★2026-10-08: 超时【必须重试这一条】, 不能只打一行就往下走★★
         *
         * 【现场 14:00 的教训 —— 一个字节都不回】
         *   14:00:14.908  [G4_TX] AT+MHTTPCFG="ssl",0,1,0
         *                 ↓ 模组【10 秒一个字都没回】(刚发生过 '>' 丢失,
         *                   多半是把它当成上一条 MQTTPUB 的 payload 吃掉了)
         *   14:00:24.919  命令超时(继续)  <- 老代码只打这一行就往下走
         *   14:00:25.080  +CME ERROR: 655   <- 655 = SSL 未使能
         *   ⇒ ssl 没配上, 后面拿 https 去请求必然被拒, 整个升级白跑。
         *
         * 【为什么老代码写"继续"】注释说的是"CFG 类命令模组有时只回 OK 而不回关键字" ——
         *   但本函数的关键字本来就是 "OK", 模组回了 OK 就一定能匹配上。
         *   所以这里的 TIMEOUT 只有一种含义: 【这一条真的没生效】。
         *   对配置类命令, "没生效却继续"= 后面每一步都建立在错的前提上。
         *
         * 【现在】超时就重发同一条, 最多 OTA_HTTP_CFG_RETRY_MAX 次;
         *   重试用尽仍超时才判失败 —— 与其带着错的实例往下跑, 不如早点报 2002。 */
        s_cfg_retry++;
        if (s_cfg_retry > OTA_HTTP_CFG_RETRY_MAX)
        {
            log_info("[OTA_HTTP] !! %s 连续 %u 次超时(模组不应答), 判失败\r\n",
                     cmd, (unsigned)s_cfg_retry);
            s_cfg_retry = 0u;
            fail(OTA_HTTP_E_DOWNLOAD, "配置命令连续超时(模组不应答)");
            return true;
        }
        log_info("[OTA_HTTP] %s 超时(模组没回应) —— 第 %u/%u 次重发同一条"
                 "(不复用'继续', 否则 ssl 没配上后面必然 655)\r\n",
                 cmd, (unsigned)s_cfg_retry, (unsigned)OTA_HTTP_CFG_RETRY_MAX);
        return false;                   /* 不推进: 下一拍重发同一条 */
    }
    s_cfg_retry = 0u;                   /* 本条成功: 计数清零, 下一步重新算 */
    return true;
}

static void poll_connecting(void)
{
    char cmd[192];
    char hostpart[160];

    if (s_conn_step == 0u)
    {
        /* host[:port] 拼回去 */
        if (s_url.port != 0u)
        {
            (void)snprintf(hostpart, sizeof(hostpart), "%s:%u",
                           s_url.host, (unsigned)s_url.port);
        }
        else
        {
            (void)snprintf(hostpart, sizeof(hostpart), "%s", s_url.host);
        }

        (void)snprintf(cmd, sizeof(cmd), "AT+MHTTPCREATE=\"%s://%s\"",
                       s_url.scheme, hostpart);

        /* 【必须用 g4_io_cmd_run 而不是 g4_io_cmd_active 判断】
         * 引擎是【三态】: IDLE / RUNNING / DONE。收包泵里命中关键字后引擎进
         * DONE(结果待取走), 此时 g4_io_cmd_active() 返回 false —— 若用它判断,
         * 会以为"没命令在途"而重新 begin, 把刚拿到的结果覆盖成 RUNNING,
         * 于是每拍重发一次、永远取不到应答。g4_io_cmd_run() 内部三态都处理了。
         * (2026-10-02 实测踩到: 状态机永久卡在 CONNECTING。) */
        /* ★2026-10-07: 与 conn_cmd 同一道安全网★ —— 只有【本步真的把 CREATE 发出去过】
         * 才认它的应答。否则解析出来的实例号是从别人的应答里瞎抓的,
         * 后面所有 AT+MHTTPCFG / REQUEST 都会打到不存在的实例上。 */
        {
            uint32_t     tx0 = g4_io_cmd_tx_seq();
            g4_cmd_res_t r = g4_io_cmd_run_owned(cmd, "+MHTTPCREATE:",
                                                 OTA_HTTP_CMD_TMO_MS, G4_OWNER_OTA);

            if (r == G4_CMD_RUNNING)
            {
                if (g4_io_cmd_tx_seq() != tx0) { s_create_sent = true; }
                return;
            }
            if (!s_create_sent)
            {
                log_info("[OTA_HTTP] !! CREATE 拿到结果(%d)但本步命令没发出去, "
                         "不推进, 下一拍重发\r\n", (int)r);
                return;
            }
            s_create_sent = false;

            if (r != G4_CMD_OK)
            {
                fail(OTA_HTTP_E_DOWNLOAD, "AT+MHTTPCREATE 无应答/被拒");
                return;
            }
        }

        /* 从 "+MHTTPCREATE: <id>" 抓实例号 */
        {
            const char *r = g4_io_cmd_line();
            const char *c = (r != NULL) ? strchr(r, ':') : NULL;
            s_httpid = 0u;
            if (c != NULL)
            {
                while ((*c != '\0') && ((*c < '0') || (*c > '9'))) { c++; }
                if ((*c >= '0') && (*c <= '9')) { s_httpid = (uint16_t)(*c - '0'); }
            }
        }
        log_info("[OTA_HTTP] 实例已建: httpid=%u host=%s tls=%u\r\n",
                 (unsigned)s_httpid, hostpart, (unsigned)s_url.tls);
        s_httpid_valid = true;      /* 从此欠模组一条 AT+MHTTPDEL */
        s_conn_step = 1u;
        return;
    }

    if (s_conn_step == 1u)
    {
        (void)snprintf(cmd, sizeof(cmd), "AT+MHTTPCFG=\"ssl\",%u,%u,%u",
                       (unsigned)s_httpid,
                       (unsigned)(s_url.tls ? OTA_HTTP_SSL_ENABLE : 0u),
                       (unsigned)OTA_HTTP_SSL_ID);
        if (conn_cmd(cmd, "OK")) { s_conn_step = 2u; }
        return;
    }

    if (s_conn_step == 2u)
    {
        /* cached=0: 非缓存, 数据走 content URC(见 ota_http.h) */
        (void)snprintf(cmd, sizeof(cmd), "AT+MHTTPCFG=\"cached\",%u,0",
                       (unsigned)s_httpid);
        if (conn_cmd(cmd, "OK")) { s_conn_step = 3u; }
        return;
    }

    if (s_conn_step == 3u)
    {
        /* output_format=1 -> HEX(二进制安全) */
        (void)snprintf(cmd, sizeof(cmd), "AT+MHTTPCFG=\"encoding\",%u,0,1",
                       (unsigned)s_httpid);
        if (conn_cmd(cmd, "OK")) { s_conn_step = 4u; }
        return;
    }

    if (s_conn_step == 4u)
    {
        /* ★这是"每块 ≤512 字节"的关键, 待台架确认对 content 生效 */
        (void)snprintf(cmd, sizeof(cmd), "AT+MHTTPCFG=\"fragment\",%u,%u,%u",
                       (unsigned)s_httpid,
                       (unsigned)OTA_HTTP_FRAG_SIZE,
                       (unsigned)OTA_HTTP_FRAG_INTV);
        if (conn_cmd(cmd, "OK")) { s_conn_step = 5u; }
        return;
    }

    if (s_conn_step == 5u)
    {
        (void)snprintf(cmd, sizeof(cmd), "AT+MHTTPCFG=\"timeout\",%u,60,30,20",
                       (unsigned)s_httpid);
        if (conn_cmd(cmd, "OK")) { s_conn_step = 6u; }
        return;
    }

    /* 全部配好, 开始下载 */
    log_info("[OTA_HTTP] 实例配置完成, 开始下载 size=%lu chunk=%u\r\n",
             (unsigned long)s_size, (unsigned)OTA_HTTP_CHUNK);
    set_state(OTA_HTTP_DOWNLOADING);
    /* 【必须走 chunk_begin()】: 它负责算 Range 区间 + 存 MD5/CRC 快照。
     * 早期版本在这里手工置 s_chunk_len = 0, 结果拼出 "bytes=0-4294967295"
     * (无符号下溢), 服务器算出的长度是 0, 一个字节都不下来。
     * (2026-10-02 主机测试抓到的。) */
    chunk_begin();
}

/* ==================== 下载 ==================== */

/** @brief 开始新的一片: 定 Range 区间, 存快照 */
static void chunk_begin(void)
{
    uint32_t left = s_size - s_written;

    s_chunk_off = s_written;
    s_chunk_len = (left > OTA_HTTP_CHUNK) ? OTA_HTTP_CHUNK : left;
    s_chunk_got = 0u;
    s_got_header = false;
    s_http_code = 0u;
    s_srv_clen = 0u;
    s_rxq_len = 0u;
    s_rxq_ovf = false;
    s_chunk_bad = false;
    s_tick_rx = now_ms();

    /* ★2026-10-07: 新片开始 —— 清掉"等 653"的窗口和计数★ */
    s_busy_hold_tick = 0u;
    s_busy_retry     = 0u;

    /* 快照: 片内失败要能干净回滚 MD5/CRC(见文件头约束 3) */
    s_md5_snap = s_md5;
    s_crc_snap = s_crc;
    s_snap_written = s_written;

    s_dl_step = DL_SEND_HEADER;
}

/** @brief 本片失败: 回滚到最近一次下盘边界并重试, 或整单失败 */
static void chunk_retry_or_fail(const char *why)
{
    /* 回滚到【快照点】(最近一次暂存区排空的边界), 不是片起点 ——
     * 那里 Flash 已写的字节与 md5/crc 状态严格对应, 重试不会重写任何
     * 已编程的 double word。 */
    s_md5      = s_md5_snap;
    s_crc      = s_crc_snap;
    s_written  = s_snap_written;
    s_rx_total = s_snap_written;
    s_stage_len = 0u;
    s_rxq_len   = 0u;
    s_rxq_ovf   = false;

    s_chunk_retry++;
    s_retry_total++;

    if (s_chunk_retry > OTA_HTTP_CHUNK_RETRY)
    {
        fail(OTA_HTTP_E_DOWNLOAD, why);
        return;
    }

    log_info("[OTA_HTTP] 片重试 %u/%u @%lu (%s)\r\n",
             (unsigned)s_chunk_retry, (unsigned)OTA_HTTP_CHUNK_RETRY,
             (unsigned long)s_written, why);

    /* 从新的 s_written 重新划 Range, 并重新存快照 */
    chunk_begin();
}

/** @brief 把队列里的字节消化掉: 喂 MD5/CRC + 下盘 */
static bool drain_queue(void)
{
    uint32_t off = 0u;

    if (s_rxq_len == 0u)
    {
        return true;
    }

    /* ★ md5/crc 必须与"落盘"同步推进, 快照才能提交在落盘边界上 ★
     *
     * 2026-10-02 故障注入点扫描抓到的 bug:
     *   老写法先把【整段】s_rxq 喂给 md5_update(), 再 stage_feed() 下盘,
     *   最后只在"喂完暂存区【刚好】排空"时才提交快照。
     *   可每次喂进来的是 200 字节的 URC 碎片: 200k ≡ 0 (mod 512) 要求
     *   k ≡ 0 (mod 64) —— 一个 8192 的片里根本凑不出这种巧合。
     *   于是 s_snap_written 一直停在很早的位置, 片内一失败就回滚到那里,
     *   把【已经编程好的几千字节又写了一遍】: 实测把注入点放在第 1 片尾部
     *   时重写了 960 个 double word = 7680 字节。
     *   在 NOR 上重写同样的数据多数时候不致命(AND 相同值不变), 但它
     *     1) 违反"重试绝不重写已编程 double word"的设计承诺(文件头约束 3)
     *     2) 白耗 Flash 擦写寿命
     *     3) 让重试要多下好几 KB —— 每片重试从 ~0.1s 变成 ~2s,
     *        碰到弱网就更容易撞上 60s 的片超时
     *
     * 现在: 每放满 512 就落盘, 落盘后【立刻】提交快照 —— 此刻 md5/crc 恰好
     * 覆盖 s_written 个字节, 两者严格一致, 回滚就落在真正的落盘边界上。 */
    while (off < s_rxq_len)
    {
        uint16_t room = (uint16_t)(OTA_HTTP_STAGE_SZ - s_stage_len);
        uint16_t take = (uint16_t)(s_rxq_len - off);

        if (take > room)
        {
            take = room;
        }

        md5_update(&s_md5, &s_rxq[off], take);
        s_crc = ota_port_crc32_update(s_crc, &s_rxq[off], take);
        stage_put(&s_rxq[off], take);

        off        += take;
        s_rx_total += take;

        if (s_stage_len >= OTA_HTTP_STAGE_SZ)
        {
            if (!stage_flush())
            {
                s_rxq_len = 0u;
                fail(OTA_HTTP_E_INTERNAL, "Flash 写入失败");
                return false;
            }
            /* 【快照提交点】Flash 上的字节数与 md5/crc 状态此刻严格一致 */
            s_md5_snap     = s_md5;
            s_crc_snap     = s_crc;
            s_snap_written = s_written;
        }
    }

    s_rxq_len = 0u;
    return true;
}

/* ==================== 下载阶段的 AT 命令: ★653(客户端忙)要等一等再重试★ ====================
 *
 * 【官方手册《ML307_HTTP_HTTPS用户手册 V4.0.0》§5 错误码】
 *   650 未知错误      651 无空闲客户端   652 客户端未创建   653 客户端忙
 *   654 URL解析失败   655 SSL未使能      656 连接失败       657 数据发送失败
 *   658 打开文件失败
 *
 * 【现场 2026-10-07 18:58:00 —— 第一次下载就是死在这里】
 *   18:57:59  第一片(8192 字节)收满
 *   18:58:00.246  [G4_TX] AT+MHTTPHEADER=0,0,0,"Range: bytes=8192-15443"  <- 第二片的头
 *   18:58:00.265  [G4_RX] +CME ERROR: 653        <- 19ms 后: 模组 HTTP 客户端还忙着
 *   18:58:00.265  !! 失败(2002)                  <- 老代码直接判死整个升级
 *   然后用户重发一次 ota.start, 从 8192 续传, 【一次就过, MD5 校验通过】——
 *   可见这不是"连接坏了", 只是"模组忙了一下, 该等一等"。
 *
 * 【为什么不能走 conn_cmd()】它对 G4_CMD_ERR 一律 fail(2002), 而且错误消息
 *   还是"建立 HTTP 实例/配置失败"(在下载阶段发 HEADER, 那句话本身就误导)。
 *
 * 【为什么不用 chunk_retry_or_fail()】它会【回滚到快照点重下整片】。HEADER 被拒
 *   跟"片里的数据"毫无关系, 为了一个 653 去重下 8192 字节纯属浪费 —— 而且弱网下
 *   更容易撞上 60s 片超时。所以这里单独一条路: 等一小会儿, 重发同一条。
 *
 * 返回: 1 = 成功/超时, 可以走下一步; 0 = 还在途或在等 653 的窗口(下一拍继续);
 *       -1 = 已经 fail()(状态机已置 FAILED, 调用方直接返回即可)。 */
static int dl_cmd_busy_retry(const char *cmd, const char *tag)
{
    g4_cmd_res_t r;

    /* 正在"客户端忙"的等待窗口里: 到点之前【一个字节都不发】 */
    if (s_busy_hold_tick != 0u)
    {
        if ((uint32_t)(now_ms() - s_busy_hold_tick) < OTA_HTTP_BUSY_RETRY_MS)
        {
            return 0;
        }
        s_busy_hold_tick = 0u;
    }

    r = g4_io_cmd_run_owned(cmd, "OK", OTA_HTTP_CMD_TMO_MS, G4_OWNER_OTA);

    if (r == G4_CMD_RUNNING)
    {
        return 0;                       /* 在途 */
    }

    if (r == G4_CMD_ERR)
    {
        const char *resp = g4_io_cmd_resp();

        if ((resp != NULL) && (strstr(resp, "653") != NULL))
        {
            s_busy_retry++;
            if (s_busy_retry > OTA_HTTP_BUSY_RETRY_MAX)
            {
                log_info("[OTA_HTTP] !! %s 连续 %u 次回 653(客户端忙), 放弃本片\r\n",
                         tag, (unsigned)s_busy_retry);
                s_busy_retry = 0u;
                chunk_retry_or_fail("模组 HTTP 客户端一直忙(653)");
                return -1;
            }
            s_busy_hold_tick = now_ms();
            log_info("[OTA_HTTP] %s 回 653(客户端忙) —— 第 %u/%u 次: 等 %ums 重发同一条"
                     "(片进度不动, 不重下)\r\n",
                     tag, (unsigned)s_busy_retry, (unsigned)OTA_HTTP_BUSY_RETRY_MAX,
                     (unsigned)OTA_HTTP_BUSY_RETRY_MS);
            return 0;
        }

        /* 其它错误: 按整片重试(带快照回滚), 与老行为一致 */
        s_busy_retry = 0u;
        {
            char why[64];
            (void)snprintf(why, sizeof(why), "%s 被拒", tag);
            chunk_retry_or_fail(why);
        }
        return -1;
    }

    /* OK / TIMEOUT: 往下走 */
    s_busy_retry = 0u;
    return 1;
}

static void poll_downloading(void)
{
    char     cmd[192];
    uint32_t silent_ms;

    /* ---- 先把 URC 钩子塞进来的字节消化掉 ---- */
    if (s_rxq_ovf)
    {
        chunk_retry_or_fail("URC 队列溢出");
        if (s_state != OTA_HTTP_DOWNLOADING) { return; }
    }
    else if (s_chunk_bad)
    {
        chunk_retry_or_fail("模组报错/服务器跳转");
        if (s_state != OTA_HTTP_DOWNLOADING) { return; }
    }
    else if (!drain_queue())
    {
        return;     /* fail() 已经置状态 */
    }

    /* ---- 片收满了? ----
     * ★ 对齐/完成的判据必须用 s_rx_total(收到的)而不是 s_written(已下盘的) ★
     *
     * 2026-10-02 复查时抓到: 中间片的长度 = OTA_HTTP_CHUNK, 编译期已断言它是
     * STAGE 的整数倍, 所以中间片收完时暂存区【必定排空】, 两者相等。
     * 但【末片】长度 = size % CHUNK, 一般不是 512 的整数倍 —— 那时最后几百
     * 字节还压在暂存区里(由 poll_verifying() 开头统一 flush), 于是
     *     s_written != s_chunk_off + s_chunk_len
     * 就误判成"片边界未对齐"直接失败, 而且报的还是"检查 CHUNK/STAGE 配置"
     * 这种把人带偏的话。
     *
     * 例: size = 10000 -> 末片 1808 = 3*512 + 272 -> s_written 只有 9728,
     *     而 9728 != 10000 -> 必然失败。也就是说【任何 size % 8192 % 512 != 0
     *     的固件都升不了级】。台架靶子恰好是 65536(8 整片), 所以一直没暴露。 */
    if ((s_chunk_len > 0u) && (s_chunk_got >= s_chunk_len))
    {
        if (s_rx_total != (s_chunk_off + s_chunk_len))
        {
            fail(OTA_HTTP_E_INTERNAL, "片内收字节数与请求长度对不上");
            return;
        }

        s_chunk_retry = 0u;

        /* 整包判完也用 s_rx_total: 用 s_written 会把"末片尾巴还在暂存区"
         * 误判成"还没下完", 于是再请求一遍那几百字节 —— 既多下又重写 Flash。 */
        if (s_rx_total >= s_size)
        {
            log_info("[OTA_HTTP] 下载完成 %lu 字节(已下盘 %lu, 余下由校验前统一 flush), 开始校验\r\n",
                     (unsigned long)s_rx_total, (unsigned long)s_written);
            s_dl_done_pending = true;
            set_state(OTA_HTTP_VERIFYING);
            return;
        }

        /* ---- 断点落盘(掉电续传) ----
         * 【为什么不能每片都存】: ota_port_http_resume_save() 内部要【擦整页】
         * (8KB, 约 25ms), 每片都存等于给下载插 16 次停顿。改成进度每跨过
         * 1/8 个 size 存一次 —— 128KB 全程 8 次, 最坏掉电只需重下最后一个 1/8。
         * 存的位置=[0..31] 的标记页, 下载阶段那里没有有效标记, 所以擦页安全。 */
        {
            uint32_t step = s_size / 8u;
            if (step < OTA_HTTP_CHUNK)
            {
                step = OTA_HTTP_CHUNK;      /* 小固件也别退化成每片一存 */
            }
            if ((s_written - s_saved_written) >= step)
            {
                if (ota_port_http_resume_save(s_written, s_size, s_md5hex, s_version))
                {
                    s_saved_written = s_written;
                }
                else
                {
                    /* 存不下不致命 —— 只是这次掉电要从更早的地方重下 */
                    log_info("[OTA_HTTP] !! 断点落盘失败(不致命)\r\n");
                }
            }
        }

        chunk_begin();
        return;
    }

    /* ---- 片内超时 ---- */
    silent_ms = (uint32_t)(now_ms() - s_tick_rx);
    if (silent_ms > OTA_HTTP_CHUNK_TMO_MS)
    {
        chunk_retry_or_fail("片内超时无数据");
        return;
    }

    /* ---- 推命令 ---- */
    if (s_dl_step == DL_SEND_HEADER)
    {
        int rc;

        (void)snprintf(cmd, sizeof(cmd),
                       "AT+MHTTPHEADER=%u,0,0,\"Range: bytes=%lu-%lu\"",
                       (unsigned)s_httpid,
                       (unsigned long)s_chunk_off,
                       (unsigned long)(s_chunk_off + s_chunk_len - 1u));

        /* ★2026-10-07★ 走 dl_cmd_busy_retry: 653(客户端忙)等一等重发,
         * 不再像老代码那样把整次升级判死(见该函数上方整段说明)。 */
        rc = dl_cmd_busy_retry(cmd, "AT+MHTTPHEADER");
        if (rc == 1)
        {
            s_dl_step = DL_SEND_REQUEST;
        }
        return;
    }

    if (s_dl_step == DL_SEND_REQUEST)
    {
        int rc;

        (void)snprintf(cmd, sizeof(cmd), "AT+MHTTPREQUEST=%u,1,0,\"%s\"",
                       (unsigned)s_httpid, s_url.path);

        /* 同样走 653 容忍版: 上一片刚收完时模组也可能还忙着。
         * OK/TIMEOUT 都往下走 —— 正文是靠 +MHTTPURC 来的, 不靠这条命令的应答 */
        rc = dl_cmd_busy_retry(cmd, "AT+MHTTPREQUEST");
        if (rc == 1)
        {
            s_dl_step = DL_WAIT_DATA;
            s_tick_rx = now_ms();
        }
        return;
    }

    /* DL_WAIT_DATA: 什么都不用发, 等 URC。超时在上面判了 */
}

/* ==================== 校验 ==================== */

static void poll_verifying(void)
{
    uint8_t  digest[16];
    char     hex[33];
    ota_meta_t meta;

    if (!stage_flush())
    {
        fail(OTA_HTTP_E_INTERNAL, "Flash 收尾写失败");
        return;
    }

    md5_final(&s_md5, digest);
    md5_to_hex(digest, hex);

    if (!md5_equal_hex(digest, s_md5hex))
    {
        log_info("[OTA_HTTP] !! MD5 不符: 算出 %s, 云端给 %s\r\n", hex, s_md5hex);

        /* ★2026-10-07 诊断: 把"我们到底存进去了什么"的头尾原样打出来★
         *
         * 现场(18:45) 的现象很难查: 收到 15444 字节(= 服务器 content-length, 一个不多
         * 一个不少), 但 MD5 不符 —— 也就是【长度对、内容错】。
         * 光看长度分不出下面三种情况, 而它们的修法完全不同:
         *   ① 头部就错(整体错位/字段被多吃掉几个数字)  -> 看头 32 字节
         *   ② 尾部错(末片边界/暂存区 flush 少写多写)    -> 看尾 16 字节
         *   ③ 中间散落错(行被 512 截断/URC 丢块)        -> 头尾都对, 只有 MD5 不对
         * PNG 的头是固定 8 字节 89 50 4E 47 0D 0A 1A 0A、尾是 49 45 4E 44 AE 42 60 82,
         * 一眼就能读出是哪种。从 Flash 回读(而不是查内存), 顺便验了写盘这条路。 */
        {
            uint8_t head[32];
            uint8_t tail[16];
            char    hbuf[2u * 32u + 1u];
            char    tbuf[2u * 16u + 1u];
            uint16_t i;
            uint32_t tail_off = (s_written > (uint32_t)sizeof(tail))
                                    ? (s_written - (uint32_t)sizeof(tail)) : 0u;

            if (ota_port_flash_read(0u, head, (uint16_t)sizeof(head)))
            {
                for (i = 0u; i < (uint16_t)sizeof(head); i++)
                {
                    (void)snprintf(&hbuf[2u * i], 3u, "%02X", (unsigned)head[i]);
                }
                hbuf[2u * sizeof(head)] = '\0';
                log_info("[OTA_HTTP]   头 32B: %s\r\n", hbuf);
            }
            else
            {
                log_info("[OTA_HTTP]   !! 头 32B 回读失败\r\n");
            }

            if (ota_port_flash_read(tail_off, tail, (uint16_t)sizeof(tail)))
            {
                for (i = 0u; i < (uint16_t)sizeof(tail); i++)
                {
                    (void)snprintf(&tbuf[2u * i], 3u, "%02X", (unsigned)tail[i]);
                }
                tbuf[2u * sizeof(tail)] = '\0';
                log_info("[OTA_HTTP]   尾 16B (@%lu): %s\r\n",
                         (unsigned long)tail_off, tbuf);
            }
            else
            {
                log_info("[OTA_HTTP]   !! 尾 16B 回读失败\r\n");
            }

            log_info("[OTA_HTTP]   收到 %lu 字节 / 已下盘 %lu / size %lu / content-length %lu\r\n",
                     (unsigned long)s_rx_total, (unsigned long)s_written,
                     (unsigned long)s_size, (unsigned long)s_srv_clen);
        }

        ota_port_mark_invalid();
        fail(OTA_HTTP_E_MD5, "MD5 校验失败");
        return;
    }

    log_info("[OTA_HTTP] MD5 校验通过: %s\r\n", hex);

    /* bootloader 只认标记页里的 CRC32(bootloader.c 的 bl_calc_fw_crc),
     * 所以除了 MD5 还得把 CRC32 刷进去 —— 见 ota_http.h 顶部说明 */
    memset(&meta, 0, sizeof(meta));
    (void)memcpy(meta.version, s_version, sizeof(meta.version) - 1u);
    meta.total_size = s_size;
    meta.crc32      = s_crc ^ 0xFFFFFFFFu;

    if (!ota_port_mark_valid(&meta))
    {
        fail(OTA_HTTP_E_INTERNAL, "写 OTA 标记页失败");
        return;
    }

    log_info("[OTA_HTTP] 就绪: 已写标记(size=%lu crc32=%08lX), 等上层重启\r\n",
             (unsigned long)s_size, (unsigned long)meta.crc32);
    /* 升级已经走完(标记页也写了), HTTP 实例没用了 —— 登记归还, 免得漏一个 */
    teardown_request();
    set_state(OTA_HTTP_READY);
}

/* ==================== 擦除 ==================== */

/**
 * @brief 断点续传: 把已经下好在 Flash 里的前缀读回来, 重建 MD5/CRC32 累加器
 *
 * 【为什么必须重算】: 掉电会把 md5_ctx_t / s_crc 一起丢掉(它们只在 RAM 里)。
 * 如果从断点继续往后面写, 累加器里只有后半段 —— 最后 MD5 必然对不上,
 * 而"读到"的暂存区其实是完整的, 这种失败最难查。
 *
 * 代价: 最多回读 240 KB。片内 Flash 是 XIP 映射, 顺序读几百 KB 是毫秒级,
 * 相比"重下整个固件"便宜得多。
 */
static bool resume_rebuild_checksums(void)
{
    uint8_t  buf[256];
    uint32_t off = 0u;

    md5_init(&s_md5);
    s_crc = 0xFFFFFFFFu;

    while (off < s_resume_from)
    {
        uint32_t left = s_resume_from - off;
        uint16_t n = (left > (uint32_t)sizeof(buf)) ? (uint16_t)sizeof(buf)
                                                    : (uint16_t)left;

        if (!ota_port_flash_read(off, buf, n))
        {
            return false;
        }
        md5_update(&s_md5, buf, n);
        s_crc = ota_port_crc32_update(s_crc, buf, n);
        off += n;
    }

    /* 累加器与 Flash 状态必须严格一致 —— 快照点就设在断点上 */
    s_written      = s_resume_from;
    s_snap_written = s_resume_from;
    s_md5_snap     = s_md5;
    s_crc_snap     = s_crc;
    s_rx_total     = s_resume_from;

    log_info("[OTA_HTTP] 已回读 %lu 字节重建 MD5/CRC32, 从断点继续\r\n",
             (unsigned long)s_resume_from);
    return true;
}

static void poll_erasing(void)
{
    if (!s_erase_started)
    {
        uint32_t total_pages = (s_size + FM_PAGE_SIZE - 1u) / FM_PAGE_SIZE;
        uint32_t cap         = ota_port_stage_size() / FM_PAGE_SIZE;
        uint32_t first_page  = s_resume_from / FM_PAGE_SIZE;

        if (total_pages > cap)
        {
            fail(OTA_HTTP_E_NOSPACE, "固件大于 OTA 暂存区");
            return;
        }
        /* 保险: 断点落到末尾之外就当从头来(正常不会发生, 记录写入时已判过) */
        if (first_page >= total_pages)
        {
            first_page    = 0u;
            s_resume_from = 0u;
        }

        /* ★ 只擦"还要写"的那些页 ★
         * 断点之前的数据【必须留着】, 否则续传就无从谈起 —— 这正是
         * ota_port_flash_erase_begin_at() 存在的唯一理由(老 API 恒从第 0 页擦)。 */
        if (!ota_port_flash_erase_begin_at(first_page, total_pages - first_page))
        {
            fail(OTA_HTTP_E_INTERNAL, "擦除无法启动");
            return;
        }

        if ((s_resume_from > 0u) && !resume_rebuild_checksums())
        {
            fail(OTA_HTTP_E_INTERNAL, "读回已下载前缀失败");
            return;
        }

        s_erase_started = true;
        return;
    }

    if (!ota_port_flash_erase_step())
    {
        return;     /* 还要继续擦 */
    }
    if (!ota_port_flash_erase_ok())
    {
        fail(OTA_HTTP_E_INTERNAL, "擦除有页失败");
        return;
    }

    log_info("[OTA_HTTP] 暂存区已擦除(从第 %lu 字节起), 开始建实例\r\n",
             (unsigned long)s_resume_from);
    s_conn_step = 0u;
    set_state(OTA_HTTP_CONNECTING);
}

/* ==================== 对外接口 ==================== */

void ota_http_init(void)
{
    memset(&s_md5, 0, sizeof(s_md5));
    s_state = OTA_HTTP_IDLE;
    s_err   = OTA_HTTP_OK;
    s_size  = 0u;
    s_written = 0u;
    s_rx_total = 0u;
    s_rxq_len = 0u;
    s_rxq_ovf = false;
    s_stage_len = 0u;
    s_erase_started = false;
    s_retry_total = 0u;

    /* 注册到 g4_urc: 让 +MHTTPURC 行不走 AT 应答匹配 */
    g4_urc_set_http_cb(ota_http_on_urc);

    /* ★ 开机清扫 0~3 ★
     * 上一次运行可能在"已经 AT+MHTTPCREATE 了、还没归还"的时候就被复位了
     * (掉电 / 看门狗复位 / 4G 链路重建之外的软复位) —— 那笔债模组那边还记着,
     * 而我们的 s_del_needed 是 RAM, 复位就没了, 债不会自己消失。
     * 攒够 4 次这种复位, MHTTPCREATE 就再也建不起来, OTA 永久失效。
     * 所以开机无条件把 4 个 id 都删一遍: 不存在的实例最多多几条 CME ERROR,
     * 代价是开机多 ~4 条 AT 往返(约 200ms), 一次性开销。 */
    teardown_request_range(0u, 3u);

    log_info("[OTA_HTTP] init: chunk=%u frag=%u/%ums stage=%u\r\n",
             (unsigned)OTA_HTTP_CHUNK, (unsigned)OTA_HTTP_FRAG_SIZE,
             (unsigned)OTA_HTTP_FRAG_INTV, (unsigned)OTA_HTTP_STAGE_SZ);
}

bool ota_http_start(const char *url, const char *md5hex, uint32_t size,
                    const char *version)
{
    s_dl_done_pending = false;   /* 新一轮: 清掉上一轮的完成事件 */
    uint16_t n;

    if (ota_http_busy())
    {
        /* 【不碰 s_err】: 它属于【正在进行的那个会话】。早期写法在这里写
         * s_err = INTERNAL, 于是一次被拒的并发请求会把当前会话的错误状态
         * 污染成 internal error。云协议层本来就先判 ota_http_busy() 并回 1006,
         * 不需要靠 s_err 才知道为什么被拒。 */
        log_info("[OTA_HTTP] !! 已有升级在进行中, 拒绝新的 ota.start\r\n");
        return false;
    }

    s_err = OTA_HTTP_OK;

    /* ---- URL ---- */
    if ((url == NULL) || (strlen(url) >= OTA_HTTP_URL_MAX))
    {
        s_err = OTA_HTTP_E_URL;
        log_info("[OTA_HTTP] !! URL 为空或过长\r\n");
        return false;
    }
    if (!url_parse(url, &s_url))
    {
        s_err = OTA_HTTP_E_URL;
        log_info("[OTA_HTTP] !! URL 非法(只认 http:// 或 https:// 且 host 非空): %s\r\n", url);
        return false;
    }
    (void)memcpy(s_url_raw, url, strlen(url) + 1u);

    /* ---- size ---- */
    if ((size == 0u) || (size > ota_port_stage_size()))
    {
        s_err = OTA_HTTP_E_NOSPACE;
        log_info("[OTA_HTTP] !! size=%lu 超出暂存区(%lu)\r\n",
                 (unsigned long)size, (unsigned long)ota_port_stage_size());
        return false;
    }
    s_size = size;

    /* ---- md5(32 字符 hex) ---- */
    if (md5hex == NULL)
    {
        s_err = OTA_HTTP_E_URL;
        log_info("[OTA_HTTP] !! md5 缺失\r\n");
        return false;
    }
    n = (uint16_t)strlen(md5hex);
    if (n != 32u)
    {
        s_err = OTA_HTTP_E_URL;
        log_info("[OTA_HTTP] !! md5 长度 %u != 32\r\n", (unsigned)n);
        return false;
    }
    (void)memcpy(s_md5hex, md5hex, 33u);

    /* ---- version ---- */
    memset(s_version, 0, sizeof(s_version));
    if (version != NULL)
    {
        (void)snprintf(s_version, sizeof(s_version), "%s", version);
    }

    /* ---- 断点续传: 先【读】记录, 再擦标记页(顺序绝不能反) ----
     * ota_port_mark_invalid() 是【整页擦除】, 会把续传记录一起擦掉 ——
     * 所以必须先读、后擦。见 ota_port.h 里 ota_http_resume_t 的说明。 */
    s_resume_from = 0u;
    {
        /* 同 cloud_cmd_ota_start 的理由: 这条链(main -> ... -> ota.start)本来就紧,
         * 栈上没有余量摆 57 B 的字符串缓冲。反正单线程非重入。 */
        static char rmd5[33];
        static char rver[24];
        uint32_t    rw = 0u;
        uint32_t    rs = 0u;

        memset(rmd5, 0, sizeof(rmd5));
        memset(rver, 0, sizeof(rver));

        if (ota_port_http_resume_load(&rw, &rs, rmd5, (uint16_t)sizeof(rmd5),
                                      rver, (uint16_t)sizeof(rver)))
        {
            /* 判据: 总长一致 + MD5 一致 + 断点落在 (0, size) 之间
             * → 说明暂存区里那半个镜像【就是这份固件】, 可以接着下。
             * 不比对 URL: 换镜像源但内容相同, 续传依然正确(见设计文档 §6bis.3)。 */
            if ((rs == size) && (rw > 0u) && (rw < size) &&
                (strcmp(rmd5, s_md5hex) == 0))
            {
                s_resume_from = rw & ~(FM_PAGE_SIZE - 1u);   /* 页对齐(记录里已对齐, 再保险) */
                log_info("[OTA_HTTP] ★发现可用断点: 已下 %lu/%lu 字节 -> 从 %lu 续传\r\n",
                         (unsigned long)rw, (unsigned long)size,
                         (unsigned long)s_resume_from);
            }
            else
            {
                log_info("[OTA_HTTP] 有断点记录但与本次不符(size %lu vs %lu, md5 %s), 从头下\r\n",
                         (unsigned long)rs, (unsigned long)size, rmd5);
            }
        }
    }

    /* 擦标记页: ① 清掉上一次可能残留的 "OTAV" ② 保证续传记录可写(擦完才能写)。
     * 不依赖"上一次升级结束时留下的全 0xFF"这种隐式状态。 */
    ota_port_mark_invalid();

    /* 立刻把本次断点(written=续传起点)落盘 —— 这样"刚受理就掉电"下次也认得目标 */
    (void)ota_port_http_resume_save(s_resume_from, size, s_md5hex, s_version);

    /* ---- 起跑 ---- */
    md5_init(&s_md5);
    s_crc = 0xFFFFFFFFu;
    s_written = s_resume_from;
    s_snap_written = s_resume_from;
    s_saved_written = s_resume_from;
    s_rx_total = s_resume_from;
    s_stage_len = 0u;
    s_rxq_len = 0u;
    s_rxq_ovf = false;
    s_chunk_retry = 0u;
    s_erase_started = false;
    s_httpid = 0u;
    s_httpid_valid = false;     /* 【不碰 s_del_needed】: 上一次欠的归还还得先还掉 */
    s_conn_step = 0u;
    /* ★2026-10-07: 新会话必须把两个"本步发出去了吗"的闩锁清掉★
     * 上一轮失败时它们可能停在 true, 那会让新会话的第一步【没发出去也被放行】。 */
    s_cfg_sent    = false;
    s_create_sent = false;
    s_cfg_retry   = 0u;             /* 新会话: 超时重发计数清零 */
    s_eng_wait_tick = now_ms();
    s_eng_wait_logged = false;
    s_content_dbg = 0u;             /* 诊断: 下个会话重新打前 3 条 content */
    s_chunk_bad = false;
    s_got_header = false;
    s_http_code = 0u;
    s_srv_clen = 0u;

    log_info("[OTA_HTTP] 受理: version=%s size=%lu md5=%s\r\n",
             (s_version[0] != '\0') ? s_version : "(无)",
             (unsigned long)size, s_md5hex);
    log_info("[OTA_HTTP]   url=%s (scheme=%s host=%s port=%u path=%s)\r\n",
             s_url_raw, s_url.scheme, s_url.host,
             (unsigned)s_url.port, s_url.path);

    set_state(OTA_HTTP_ERASING);
    return true;
}

/**
 * @brief 推进"归还实例"(AT+MHTTPDEL)
 * @return true = 没有待办的归还, 或已经归还完(调用方继续往下走)
 *
 * 【为什么失败也算"归还完"】: 模组对不存在的实例会回 +CME ERROR, 网络断了会超时。
 * 这些都不该把状态机卡死 —— 还不上最多导致"下次 CREATE 换个 id"(到 4 个为止),
 * 而卡死是彻底不干活。所以 OK / ERR / TIMEOUT / NOPROMPT 一律收摊。
 */
static bool poll_teardown(void)
{
    char         cmd[48];
    g4_cmd_res_t r;

    if (!s_del_needed)
    {
        return true;
    }

    (void)snprintf(cmd, sizeof(cmd), "AT+MHTTPDEL=%u", (unsigned)s_del_id);
    r = g4_io_cmd_run_owned(cmd, "OK", OTA_HTTP_CMD_TMO_MS, G4_OWNER_OTA);

    if (r == G4_CMD_RUNNING)
    {
        return false;           /* 下一拍继续 */
    }

    if (r == G4_CMD_OK)
    {
        log_info("[OTA_HTTP] 实例已归还(AT+MHTTPDEL=%u)\r\n", (unsigned)s_del_id);
    }
    else
    {
        /* 不存在的实例回 CME ERROR、断网会超时 —— 都不该卡住状态机 */
        log_info("[OTA_HTTP] 归还实例未确认(AT+MHTTPDEL=%u, r=%d) —— 不重试\r\n",
                 (unsigned)s_del_id, (int)r);
    }

    if (s_del_id >= s_del_last)
    {
        s_del_needed = false;
        return true;
    }
    s_del_id++;                 /* 这一段还有下一个 */
    return false;
}

void ota_http_poll(void)
{
    /* ★2026-10-07: "拿不到引擎"必须看得见★
     *
     * 三个干活的状态里, 只要命令引擎被别人占着, 我们每一拍都是【静默 return】——
     * 串口上一个字都没有。18:21 那次死锁现场就是这样: "暂存区已擦除, 开始建实例"
     * 之后两分多钟【一条日志都没有】, 完全无法判断卡在哪。
     * 这里加一条限频日志(5 秒一次, 只报一次), 让"被谁挡住"变成看得见的证据。 */
    if ((s_state == OTA_HTTP_ERASING) || (s_state == OTA_HTTP_CONNECTING) ||
        (s_state == OTA_HTTP_DOWNLOADING))
    {
        g4_cmd_owner_t own = g4_io_cmd_owner();

        if ((own == G4_OWNER_NONE) || (own == G4_OWNER_OTA))
        {
            /* ★2026-10-08 修正★ 原来的判据是 `own == G4_OWNER_NONE` 才复位,
             * 于是【引擎上是 OTA 自己的命令在途】时也会走到下面的 else ——
             * 现场日志里就出现过自相矛盾的一行:
             *     [OTA_HTTP] !! 等了 5003ms 仍拿不到命令引擎(当前属主=1)
             * 而属主 1 就是 OTA 自己 —— 根本不是"拿不到", 是"我正在等自己那条命令"。
             * 这条误报会把人往"被外部流程占住"的方向带偏, 所以必须排除自己。 */
            s_eng_wait_tick    = now_ms();
            s_eng_wait_logged  = false;
        }
        else if ((!s_eng_wait_logged) &&
                 ((uint32_t)(now_ms() - s_eng_wait_tick) >= 5000u))
        {
            s_eng_wait_logged = true;
            log_info("[OTA_HTTP] !! 等了 %lums 仍拿不到命令引擎(当前属主=%d, "
                     "非 OTA 自己) —— 本步的 AT 发不出去; 引擎是死账的话会被 g4_io 强制收回\r\n",
                     (unsigned long)(uint32_t)(now_ms() - s_eng_wait_tick),
                     (int)own);
        }
    }

    /* 【先还债】: 无论当前什么状态, 只要欠着一条 AT+MHTTPDEL 就先发掉。
     * 必须放在 switch 之前 —— READY / IDLE / FAILED 三个分支都是直接 return 的,
     * 放进去就永远发不出去, 实例也就永远漏着。 */
    if (!poll_teardown())
    {
        return;
    }

    switch (s_state)
    {
    case OTA_HTTP_ERASING:
        poll_erasing();
        break;

    case OTA_HTTP_CONNECTING:
        poll_connecting();
        break;

    case OTA_HTTP_DOWNLOADING:
        poll_downloading();
        break;

    case OTA_HTTP_VERIFYING:
        poll_verifying();
        break;

    case OTA_HTTP_IDLE:
    case OTA_HTTP_READY:
    case OTA_HTTP_FAILED:
    default:
        /* ★2026-10-07: 交还引擎★
         * 这三个状态【不会再 poll 命令引擎】了。而带属主的命令,"结果只等属主一个人"——
         * 如果我们在这里不交还, 引擎上那条命令(或它没被取走的结果)就会一直挂着,
         * g4_at 的发布子流程此后每次都只拿到 RUNNING -> MQTT 永久发不出去。
         * (READY 之后 main.c 会复位, 但 FAILED / IDLE 是长期状态, 必须交还。) */
        g4_io_cmd_release(G4_OWNER_OTA);
        break;
    }
}

bool ota_http_abort(void)
{
    if ((s_state == OTA_HTTP_READY) || (s_state == OTA_HTTP_IDLE))
    {
        /* 协议 5.6: installing(这里对应 READY)之后拒绝中止 */
        return false;
    }

    g4_io_cmd_abort();
    ota_port_mark_invalid();
    /* 中止了也得把实例还给模组, 否则反复 start/abort 会把 4 个 id 用光 */
    teardown_request();

    log_info("[OTA_HTTP] 已中止(状态 %d -> IDLE)\r\n", (int)s_state);
    s_state = OTA_HTTP_IDLE;
    s_err   = OTA_HTTP_OK;
    s_stage_len = 0u;
    s_rxq_len = 0u;
    return true;
}

void ota_http_get_status(ota_http_status_t *out)
{
    if (out == NULL)
    {
        return;
    }
    out->state    = s_state;
    out->err      = s_err;
    out->size     = s_size;
    out->received = s_rx_total;
    out->percent  = ota_http_percent();
    (void)memcpy(out->version, s_version, sizeof(out->version));
}

uint8_t ota_http_percent(void)
{
    if (s_size == 0u)
    {
        return 0u;
    }
    if (s_rx_total >= s_size)
    {
        return 100u;
    }
    return (uint8_t)((s_rx_total * 100u) / s_size);
}

bool ota_http_busy(void)
{
    /* ★ READY 也算"忙" ★
     *
     * 这是一个真实的竞态, 2026-10-02 复查时发现:
     *   poll_verifying() 校验通过后会写 "OTAV" 标记 -> 进 READY,
     *   然后要等 main_ota_progress_poll() 倒数 3 拍(≈3s)才复位。
     * 如果这 3 秒里又来一条 ota.start:
     *   · 早期写法把 READY 当"不忙" -> cloud_cmd_ota_start 放行
     *   · ota_http_start() 第一件事就是 ota_port_mark_invalid() —— 整页擦除
     *   · 于是【刚校验通过、等着搬家的那个固件被静默丢弃】, 而平台以为新升级开始了
     * 后果是"升级明明校验通过了, 设备却一直跑旧版本", 现场极难定位。
     *
     * 所以只有 IDLE(空闲) 和 FAILED(已失败, 可以重来) 才算不忙。
     * READY = 正在安装, 必须挡住并发 ota.start。
     * (abort 不受影响: ota_http_abort() 自己判 READY 并拒绝, 见那里。) */
    return ((s_state != OTA_HTTP_IDLE) &&
            (s_state != OTA_HTTP_FAILED));
}

uint32_t ota_http_chunk_got(void)   { return s_chunk_got; }
uint32_t ota_http_written(void)     { return s_written; }
uint32_t ota_http_retry_count(void) { return s_retry_total; }

bool ota_http_take_download_done(void)
{
    bool pending = s_dl_done_pending;
    s_dl_done_pending = false;
    return pending;
}

const char *ota_http_state_name(ota_http_state_t st)
{
    switch (st)
    {
    case OTA_HTTP_IDLE:        return "idle";
    case OTA_HTTP_ERASING:      /* 擦暂存区 / 建实例 —— 都还没开始拿数据 */
    case OTA_HTTP_CONNECTING:
    case OTA_HTTP_DOWNLOADING: return "downloading";
    case OTA_HTTP_VERIFYING:   return "verifying";
    case OTA_HTTP_READY:       return "installing";
    case OTA_HTTP_FAILED:
    default:                   return "failed";
    }
}

bool ota_http_progress_due(const ota_http_status_t *st,
                           uint8_t last_state, uint8_t last_percent,
                           uint32_t now_ms, uint32_t last_ms,
                           uint32_t max_silence_ms)
{
    if (st == NULL)
    {
        return false;
    }

    /* 空闲: 没有任何可报的。调用方看到 false 会顺手复位节流状态。 */
    if (st->state == OTA_HTTP_IDLE)
    {
        return false;
    }

    /* 失败 / 就绪: 都只在【首次进入】时报一条, 不重复刷。
     * (失败反复刷会让平台以为设备一直在失败; 就绪反复刷也一样没意义。) */
    if ((st->state == OTA_HTTP_FAILED) || (st->state == OTA_HTTP_READY))
    {
        return (last_state != (uint8_t)st->state);
    }

    if ((uint8_t)st->state != last_state)
    {
        return true;        /* 状态跳变立刻报 */
    }
    if ((st->percent / 10u) != (last_percent / 10u))
    {
        return true;        /* 跨过 10% 台阶 */
    }
    if ((max_silence_ms != 0u) &&
        ((uint32_t)(now_ms - last_ms) >= max_silence_ms))
    {
        return true;        /* 静默兜底: 下载卡住时平台也能看出"还在动还是死了" */
    }
    return false;
}
