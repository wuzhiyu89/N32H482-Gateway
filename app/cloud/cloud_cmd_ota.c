/**
 * @file  cloud_cmd_ota.c
 * @brief 升级类命令：ota.start / ota.query / ota.abort（文档 5.6 第 19~21 条）
 *
 * 只是"命令层"：校验参数、翻译错误码、调用 ota_http 的接口。
 * 真正的下载/校验/写分区在 app/ota/ota_http.c 与 ota_port.c，进度上报在
 * app/core/app_uplink.c 的 app_ota_progress_poll()。
 */
#include <stdio.h>
#include <string.h>

#include "cloud_proto.h"
#include "cloud_internal.h"
#include "cloud_json.h"     /* cj_get_str / cj_get_uint */
#include "log.h"
#include "ota_http.h"
#include "ota_port.h"

/* ==================================================================
 * 19~21 OTA 类(文档 5.6 + 第 6 章) —— OTA over HTTP(S) + MD5
 * ==================================================================
 * 这一段是 2026-10-02 新增的"OTA over HTTP"独立线: 数据源从原来的
 * MQTT 分包(ota.c)换成 ML307R 原生 AT+MHTTP* 拉取(app/ota/ota_http.c)。
 * ota.start 三条命令此前恒回 1006, 现在接真实实现。
 *
 * 【错误码映射】(文档 8 章 + 5.6):
 *   URL 非法        -> 2001 CLOUD_EC_OTA_URL
 *   与当前版本相同   -> 1006 CLOUD_EC_STATE
 *   暂存区装不下     -> 1010 CLOUD_EC_NO_SPACE
 *   下载失败        -> 2002 CLOUD_EC_OTA_DOWNLOAD (由 ota.progress 上报, 见下)
 *   MD5 校验失败     -> 2003 CLOUD_EC_OTA_VERIFY
 *   并发 ota.start   -> 1006 CLOUD_EC_STATE
 */

/**
 * @brief ota.start — 发起升级(文档 5.6 第 19 条)
 *   必填: version(string) / url(string) / size(uint32) / md5(string, HEX 小写)
 *   响应: code 0 受理。受理后【不阻塞】, 由主循环的 ota_http_poll() 推进。
 */
int cloud_cmd_ota_start(const cloud_req_t *req, cJSON *data)
{
    /* 【必须 static, 不能放栈上】—— 与上面 s_data_buf 段同一个理由, 而且更急:
     * url 一个就 OTA_HTTP_URL_MAX = 256 B, 三个加起来 320 B。
     * 实测(armclang -fstack-usage)这条链是:
     *     main(2144) -> main_cloud_downlink_try(312) -> cloud_proto_handle(240)
     *     -> 本函数  -> ota_http_start(128) -> ota_port_http_resume_load(112)
     * 放栈上总额 ~3320 B / 4096 B(81%), 已经压过了既有的最坏路径
     * (main + main_publish_status_json = 3128 B), 还要给中断留余量。
     * 改 static 后本函数从 384 B 降到 ~64 B。
     *
     * 安全性: cloud_proto_handle() 只在主循环里被调, 单线程且不重入
     * (命令处理里也不会递归调用自己) —— 与 s_data_buf 的论证完全一致。 */
    static char       url[OTA_HTTP_URL_MAX];
    static char       version[24];
    static char       md5hex[40];
    uint32_t          size;
    const char       *cur;
    ota_http_status_t st;

    (void)data;                         /* 文档这条响应没有 data 字段 */

    if (req->data == NULL)
    {
        return CLOUD_EC_PARAM;
    }

    /* ---- 四个必填字段 ---- */
    if (cj_get(req->data, "size", NULL, 0u) != CJ_NUM)
    {
        /* 类型不对(给了字符串/对象)一律 1001, 不猜它想表达什么 */
        return CLOUD_EC_PARAM;
    }
    size = cj_get_uint(req->data, "size", 0u);

    if (!cj_get_str(req->data, "version", version, (uint16_t)sizeof(version)) ||
        !cj_get_str(req->data, "url",     url,     (uint16_t)sizeof(url))     ||
        !cj_get_str(req->data, "md5",     md5hex,  (uint16_t)sizeof(md5hex)))
    {
        return CLOUD_EC_PARAM;
    }

    /* ---- 与当前版本相同 -> 1006(文档 5.6 明写) ---- */
    cur = ota_port_get_version();
    if ((cur != NULL) && (cur[0] != '\0') && (strcmp(cur, version) == 0))
    {
        log_info("[CLOUD] ota.start: 目标版本 %s 与当前版本相同 -> 1006\r\n", version);
        return CLOUD_EC_STATE;
    }

    /* ---- 已有升级在途 -> 拒绝并发 ---- */
    if (ota_http_busy())
    {
        log_info("[CLOUD] ota.start: 已有升级在途 -> 1006\r\n");
        return CLOUD_EC_STATE;
    }

    if (!ota_http_start(url, md5hex, size, version))
    {
        ota_http_get_status(&st);
        switch (st.err)
        {
        case OTA_HTTP_E_URL:     return CLOUD_EC_OTA_URL;
        case OTA_HTTP_E_NOSPACE: return CLOUD_EC_NO_SPACE;
        default:                 return CLOUD_EC_PARAM;
        }
    }

    log_info("[CLOUD] ota.start 受理: version=%s size=%lu url=%s\r\n",
             version, (unsigned long)size, url);
    return CLOUD_EC_OK;
}

/**
 * @brief ota.query — 查询进度(文档 5.6 第 20 条)
 *   响应 data: {"state": ..., "percent": N, "targetVersion": "x.y.z"}
 *   state 取值: idle/downloading/verifying/installing/done/failed
 *
 * 【关于 "done"】本机没有单独的 done 态: 校验通过写标记后就 reboot 了,
 * 升级是否成功由平台拿重启后的 status.report.fwVersion 自己比对 —— 这与
 * 文档第 6 章的时序图一致。所以这里 READY 映射成 "installing"。
 */
int cloud_cmd_ota_query(const cloud_req_t *req, cJSON *data)
{
    ota_http_status_t st;

    (void)req;                          /* 文档这条没有参数 */

    ota_http_get_status(&st);

    /* 状态名由 ota_http 统一给 —— 与 main.c 的 ota.progress 上报必须同口径,
     * 两处各写一个 switch 迟早漂移(见 ota_http_state_name 的说明)。 */
    if ((cJSON_AddStringToObject(data, "state",
                                 ota_http_state_name(st.state)) == NULL) ||
        (cJSON_AddNumberToObject(data, "percent", (double)st.percent) == NULL))
    {
        return CLOUD_EC_INTERNAL;
    }
    if (st.version[0] != '\0')
    {
        if (cJSON_AddStringToObject(data, "targetVersion", st.version) == NULL)
        {
            return CLOUD_EC_INTERNAL;
        }
    }
    return CLOUD_EC_OK;
}

/**
 * @brief ota.abort — 中止(文档 5.6 第 21 条)
 *   downloading 状态可中止; installing(本机 READY)之后拒绝 -> 1006
 */
int cloud_cmd_ota_abort(const cloud_req_t *req, cJSON *data)
{
    (void)req;
    (void)data;                         /* 文档这条响应没有 data 字段 */

    if (ota_http_abort())
    {
        log_info("[CLOUD] ota.abort 已中止\r\n");
        return CLOUD_EC_OK;
    }
    log_info("[CLOUD] ota.abort: 当前状态不允许中止 -> 1006\r\n");
    return CLOUD_EC_STATE;
}
