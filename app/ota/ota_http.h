#ifndef __OTA_HTTP_H__
#define __OTA_HTTP_H__

#include <stdint.h>
#include <stdbool.h>

/**
 * \file  ota_http.h
 * \brief OTA over HTTP(S) 下载状态机(数据源: ML307R 原生 AT+MHTTP* 指令族)
 *
 * ============================ 与 ota.h 的关系 ============================
 *
 *   ota.h/.c      已有: MQTT 分包版(按包收 + 位图 + CRC32), 保留但本线不用
 *   ota_http.h/.c 本文件: HTTP 拉取版(按字节偏移 + MD5), 走 AT+MHTTP*
 *   ota_port.h/.c 移植层: Flash 擦写 / CRC32 / 标记 / 重启 —— 【两版共用, 不改】
 *
 * 为什么校验要【同时算 MD5 和 CRC32】:
 *   · 协议 5.6 要求 ota.start 带 <md5>, 云端拿它对账 —— 所以 MD5 是给人的;
 *   · bootloader 只认标记页里的 <crc32>(bootloader.c 的 bl_calc_fw_crc)——
 *     它不认 MD5。所以刷进标记页的必须是 CRC32。
 *   两个都流式算, 各 0 额外读盘。
 *
 * ============================ 下载通路 ============================
 *
 * 依据《ML307A HTTP_HTTPS 用户手册 V4.0.0》§3.1/§3.5/§3.10, 以及
 * app/links/g4/g4_io.c 的两个既有缓冲上限:
 *
 *   G4_IO_LINE_BUF_SZ = 512   行超长在 512 处强制切行
 *   G4_IO_RESP_MAX    = 384   累积应答放不下就整行丢
 *
 * 因此【不能】用"缓存模式 + AT+MHTTPREAD 一次读一大块"——那条路上限只有
 * ~128 B/次, 128 KB 要 1024 次往返。改用:
 *
 *   AT+MHTTPCFG="cached",0,0          非缓存, 数据走 URC
 *   AT+MHTTPCFG="encoding",0,0,1      输出 HEX(二进制安全)
 *   AT+MHTTPCFG="fragment",0,200,50   每块 200 B / 间隔 50 ms
 *   AT+MHTTPREQUEST=...               → 模组主动吐:
 *     +MHTTPURC: "content",<id>,<clen>,<sum>,<cur>,<hex...>
 *
 * 200 B → 400 hex 字符, 加前缀约 40 → 卡在 512 行限内。
 * 带宽: 串口 115200 8N1 有效 5760 B/s(HEX 折半), 200B/50ms = 4000 B/s, 有余量。
 *
 * ⚠️ 【待台架确认】(见 OTA_HTTP_设计_v1.md §9.1): fragment 是否真的对
 *    content URC 生效。若每条 URC 的 data 不是 200 B, 就调 OTA_HTTP_FRAG_SIZE。
 */

/* ==================== 可调参数 ==================== */

#ifndef OTA_HTTP_CHUNK
#define OTA_HTTP_CHUNK        8192u   /* 每个 Range 请求的字节数。必须是
                                       * OTA_HTTP_STAGE_SZ 的整数倍, 这样每片
                                       * 收完暂存区正好清空, 片边界天然 8 字节对齐 */
#endif
#ifndef OTA_HTTP_FRAG_SIZE
#define OTA_HTTP_FRAG_SIZE    200u    /* AT+MHTTPCFG="fragment" 的 <frag_size> */
#endif
#ifndef OTA_HTTP_FRAG_INTV
#define OTA_HTTP_FRAG_INTV    50u     /* fragment 的 <interval>(ms), 用来压带宽 */
#endif
#ifndef OTA_HTTP_SSL_ID
#define OTA_HTTP_SSL_ID       0u      /* SSL 上下文索引 0~4 */
#endif
#ifndef OTA_HTTP_SSL_ENABLE
#define OTA_HTTP_SSL_ENABLE   1u      /* 1=HTTPS。改成 0 可拿 http:// 靶子先验协议 */
#endif
#ifndef OTA_HTTP_CHUNK_RETRY
#define OTA_HTTP_CHUNK_RETRY  3u      /* 单片最多重试次数 */
#endif
#ifndef OTA_HTTP_CHUNK_TMO_MS
#define OTA_HTTP_CHUNK_TMO_MS 60000u  /* 片内多久没有新数据判超时 */
#endif
#ifndef OTA_HTTP_CMD_TMO_MS
#define OTA_HTTP_CMD_TMO_MS   10000u  /* 单条 AT 命令超时 */
#endif
#ifndef OTA_HTTP_STAGE_SZ
#define OTA_HTTP_STAGE_SZ     512u    /* 对齐暂存; 必须能整除 OTA_HTTP_CHUNK */
#endif
#ifndef OTA_HTTP_URL_MAX
#define OTA_HTTP_URL_MAX      256u
#endif

/* ==================== 状态 ==================== */

typedef enum
{
    OTA_HTTP_IDLE = 0,      /* 空闲 */
    OTA_HTTP_ERASING,       /* 擦 OTA 暂存区(分页异步) */
    OTA_HTTP_CONNECTING,    /* MHTTPCREATE + 一串 MHTTPCFG */
    OTA_HTTP_DOWNLOADING,   /* 逐片 Range 拉取 + 收 content URC */
    OTA_HTTP_VERIFYING,     /* MD5 / CRC32 比对 */
    OTA_HTTP_READY,         /* 校验通过, 标记页已写, 等上层重启 */
    OTA_HTTP_FAILED         /* 失败, 等 abort 或下次 start */
} ota_http_state_t;

/* 失败原因 —— 与协议 8 章错误码一一对应 */
typedef enum
{
    OTA_HTTP_OK = 0,
    OTA_HTTP_E_URL        = 2001,   /* URL 非法 */
    OTA_HTTP_E_DOWNLOAD   = 2002,   /* 下载失败(重试耗尽/模组报错/服务器跳转) */
    OTA_HTTP_E_MD5        = 2003,   /* MD5 不符 */
    OTA_HTTP_E_NOSPACE    = 1010,   /* 暂存区装不下 */
    OTA_HTTP_E_INTERNAL   = 1007    /* 内部/Flash 错 */
} ota_http_err_t;

/* 状态快照(供 cloud_proto 组 ota.progress / ota.query) */
typedef struct
{
    ota_http_state_t state;
    ota_http_err_t   err;           /* 仅 FAILED 时有意义 */
    uint32_t         size;          /* 目标总字节数 */
    uint32_t         received;      /* 已收字节数 */
    uint8_t          percent;       /* 0..100 */
    char             version[24];
} ota_http_status_t;

/* ==================== 对外 API ==================== */

/** @brief 初始化(清状态, 不碰 Flash)。会把自己注册到 g4_urc 的 +MHTTPURC 钩子 */
void ota_http_init(void);

/**
 * @brief 主轮询(非阻塞, 每拍调一次)
 * @note  必须在 g4_io_pump() 【之外】调用(与 g4_urc_poll 同级)
 */
void ota_http_poll(void);

/**
 * @brief 发起一次升级
 * @param url     固件 HTTP(S) 地址(含 host 与 path)
 * @param md5hex  固件 MD5, 32 字符 HEX(大小写均可)
 * @param size    固件字节数
 * @param version 目标版本号(可 NULL)
 * @return true = 已受理(进入 ERASING); false = 参数非法或正忙(取 ota_http_get_status 看原因)
 *
 * @note 受理后【不阻塞】, 由 ota_http_poll() 推进。完成后状态到 READY,
 *       由调用方决定何时 ota_port_reboot()(建议先上报 ota.progress)。
 */
bool ota_http_start(const char *url, const char *md5hex, uint32_t size,
                    const char *version);

/**
 * @brief 中止下载(仅 DOWNLOADING/CONNECTING/ERASING 可用; READY 之后拒绝)
 * @return true = 已中止
 */
bool ota_http_abort(void);

/** @brief 取状态快照 */
void ota_http_get_status(ota_http_status_t *out);

/** @brief 是否正在升级(用于 cloud_proto 拒绝并发 ota.start) */
bool ota_http_busy(void);

/** @brief 下载进度百分比 0..100 */
uint8_t ota_http_percent(void);

/**
 * @brief g4_urc 的 +MHTTPURC 钩子(g4_urc_input_line 里调)
 * @return true = 这一行属于 MHTTP URC, 已被消费(不要再拿去做 AT 应答匹配)
 *
 * @note 必须在【泵之外】也能安全调用: 本函数只往环形/暂存里塞数据, 不发 AT。
 *       发 AT 的动作全部留给 ota_http_poll()。
 */
bool ota_http_on_urc(const char *line);

/**
 * @brief 状态枚举 -> 协议字符串
 * @return "idle" / "downloading" / "verifying" / "installing" / "failed"
 *
 * 【为什么要有这个函数】: 文档 5.6 的 ota.query 应答和 7.6 的 ota.progress 上报
 * 用的是【同一套 state 口径】。两处各写一个 switch 迟早会漂移 —— 平台按字符串
 * 判断进度, 一边给 "installing" 另一边给 "ready" 就直接对不上。所以收在这里。
 *
 * 映射说明: ERASING/CONNECTING/DOWNLOADING 都算 "downloading"(都还没拿到完整镜像);
 *           READY 映射成 "installing"(标记页已写, 等重启搬运);
 *           本机没有独立的 "done" 态 —— 升级成功与否由平台拿重启后的
 *           status.report.fwVersion 自己比对(与文档第 6 章时序图一致)。
 */
const char *ota_http_state_name(ota_http_state_t st);

/**
 * @brief ota.progress 的**发送判据**(纯函数 —— 逻辑不做进 main.c 就是为了能测)
 *
 * 文档 7.6 的口径: 每 10% **或** 10s 一条。这里再加上"状态变了立刻一条",
 * 因为 downloading -> verifying -> failed 这些跳变平台必须看得见。
 *
 * @param st             当前状态快照
 * @param last_state     上次发送时的状态(0xFF = 本次会话还没发过)
 * @param last_percent   上次发送时的进度(0xFF = 同上)
 * @param now_ms         当前时基
 * @param last_ms        上次发送的时基(首条传 0 即可)
 * @param max_silence_ms 最长静默(传 0 = 不按时间触发)
 * @return true = 这一拍该发一条
 *
 * 【为什么必须是独立函数】: 这段逻辑原本写在 main.c 的 main_ota_progress_poll()
 * 里, 而 main.c 在主机上编不了 —— 于是它成了整个 OTA 链路里唯一没有测试覆盖的
 * 判断。事实上那里已经写出过一个 bug: 空闲时只复位了 last_state 就往下走,
 * 而 IDLE(0) != 0xFF 恒成立, 结果是设备空闲时每 10s 往云端刷一条
 * {"state":"idle"}。抽出来之后这类错在主机上就能挡住。
 *
 * 语义约定:
 *   · IDLE            一律不报(调用方据此复位节流)
 *   · FAILED          只报【首次进入】那一条, 不重复刷
 *   · READY           报一次(installing), 之后由调用方走复位流程
 *   · 其余状态        状态变 / 跨 10% 台阶 / 静默超时 三者任一即发
 */
bool ota_http_progress_due(const ota_http_status_t *st,
                           uint8_t last_state, uint8_t last_percent,
                           uint32_t now_ms, uint32_t last_ms,
                           uint32_t max_silence_ms);

/* ---- 仅供主机单测/诊断 ---- */

/** @brief 本片已收到的字节数(诊断用) */
uint32_t ota_http_chunk_got(void);
/** @brief 已写进 Flash 的字节数(8 字节对齐), 也是续传偏移 */
uint32_t ota_http_written(void);
/** @brief 片级重试累计次数 */
uint32_t ota_http_retry_count(void);

#endif /* __OTA_HTTP_H__ */
