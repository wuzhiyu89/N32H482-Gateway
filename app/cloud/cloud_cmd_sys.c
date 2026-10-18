/**
 * @file  cloud_cmd_sys.c
 * @brief 系统类命令：sys.query_status / sys.time_sync / sys.reboot（文档 5.1）
 *
 * 三条命令都依赖"只有 app 层才知道的东西"，所以用钩子反向注入，协议层保持可脱机测试：
 *   · query_status 需要信号 / 在线数 / 档位  -> cloud_proto_set_status_provider()
 *   · time_sync    需要向 4G 模组要时间        -> cloud_proto_set_time_sync_hook()
 *   · reboot       只登记时刻，真正复位由主循环调 cloud_proto_reboot_due()
 */
#include <string.h>

#include "cloud_proto.h"
#include "cloud_internal.h"
#include "cloud_json.h"
#include "log.h"
#include "event_cache.h"

/* ==================================================================
 * sys.reboot —— 文档 5.1 第 3 条（可延迟执行）
 * ==================================================================
 * 【为什么这里只"登记", 不直接复位】
 *   NVIC_SystemReset() 是目标机专属(主机单测链不到它), 直接在这里调会让
 *   cloud_proto 变成"不可脱机测试"的模块 —— 所以命令只记下"什么时候重启",
 *   真正的复位由主循环调 cloud_proto_reboot_due() 完成。这样这条命令的
 *   全部逻辑(参数校验 / 延迟计算 / 只触发一次)都能在主机上验证。
 * ================================================================== */
static uint32_t s_reboot_at_unix;       /* 重启时刻(Unix 秒) */
static bool     s_reboot_pending;       /* 是否有待执行的重启 */

bool cloud_proto_reboot_due(uint32_t now_unix)
{
    if (!s_reboot_pending)
    {
        return false;
    }
    /* 用有符号差值比较, Unix 秒回绕也不会判错 */
    if ((int32_t)(now_unix - s_reboot_at_unix) < 0)
    {
        return false;                   /* 还没到 */
    }
    s_reboot_pending = false;           /* 只触发一次: 取走就不再重复 */
    return true;
}

int cloud_cmd_sys_reboot(const cloud_req_t *req, cJSON *data)
{
    uint32_t delay_s = 0u;

    (void)data;                         /* 文档这条响应没有 data 字段 */

    if ((req->data != NULL) && (cj_get(req->data, "delaySeconds", NULL, 0u) != CJ_NOTFOUND))
    {
        /* 文档类型 uint16; 给了字符串/对象就回 1001, 不猜它想表达什么 */
        if (cj_get(req->data, "delaySeconds", NULL, 0u) != CJ_NUM)
        {
            return CLOUD_EC_PARAM;
        }
        delay_s = cj_get_uint(req->data, "delaySeconds", 0u);
        if (delay_s > 0xFFFFu)
        {
            return CLOUD_EC_PARAM;
        }
    }

    s_reboot_at_unix = cloud_proto_get_now_ts() + delay_s;
    s_reboot_pending = true;

    log_info("[CLOUD] sys.reboot 已登记 %lu 秒后软件复位(由主循环执行)\r\n",
             (unsigned long)delay_s);
    return CLOUD_EC_OK;
}

/* ==================================================================
 * sys.time_sync(文档 5.1 第 2 条) + 写时钟钩子
 * ================================================================== */
static cloud_time_sync_fn s_time_sync_fn;

void cloud_proto_set_time_sync_hook(cloud_time_sync_fn fn)
{
    s_time_sync_fn = fn;
}

int cloud_cmd_sys_time_sync(const cloud_req_t *req, cJSON *data)
{
    uint32_t ts;
    int32_t  diff;

    (void)data;                         /* 文档这条响应没有 data 字段 */

    if (req->data == NULL)
    {
        return CLOUD_EC_PARAM;
    }
    /* 参数 ts(uint32, 平台当前 Unix 秒)。类型不对就拒绝, 不猜 */
    if (cj_get(req->data, "ts", NULL, 0u) != CJ_NUM)
    {
        return CLOUD_EC_PARAM;
    }
    ts = cj_get_uint(req->data, "ts", 0u);
    if (ts == 0u)
    {
        return CLOUD_EC_PARAM;          /* Unix 0 当无效值 */
    }

    /* 文档: "网关时钟偏差 >5s 时才校时" —— 偏差不大就什么都不做(回 0)。
     * 用有符号差值比较, Unix 秒回绕也不会判错。 */
    diff = (int32_t)(ts - cloud_proto_get_now_ts());
    if (diff < 0)
    {
        diff = -diff;
    }
    if (diff <= 5)
    {
        return CLOUD_EC_OK;
    }

    if (s_time_sync_fn == NULL)
    {
        /* 确实需要校时, 但本机还没接写时钟的实现(main.c 还没注册) ——
         * 如实回 1006, 【不假装校准成功】, 否则平台会以为网关时间已经准了。 */
        log_info("[CLOUD] sys.time_sync: 偏差 %lds 需要校时 但本机还没注册写时钟钩子, 回 1006\r\n",
                 (long)diff);
        return CLOUD_EC_STATE;
    }

    if (!s_time_sync_fn(ts))
    {
        return CLOUD_EC_INTERNAL;       /* 写时钟失败 -> 1007 */
    }
    return CLOUD_EC_OK;
}

/* ==================================================================
 * sys.query_status(文档 5.1 第 1 条) + 运行时状态提供者
 * ==================================================================
 * 【为什么要"提供者"】应答里的 rsrp / uptime / 在线数 / 档位只有 app 层知道
 * (4G 信号在 g4_at、从机在线状态在 lora_business、档位在边缘评估)。直接在
 * cloud_proto 里 include 那些模块, 协议层就重新变成"不可脱机测试"了 ——
 * 所以由 main.c 注册一个回调, 主机测试里注册一个假的; 协议层只认这个接口。
 * ================================================================== */
static cloud_status_fn s_status_fn;

void cloud_proto_set_status_provider(cloud_status_fn fn)
{
    s_status_fn = fn;
}

int cloud_cmd_sys_query_status(const cloud_req_t *req, cJSON *data)
{
    cloud_status_t st;

    (void)req;                          /* 文档这条没有参数 */

    if (s_status_fn == NULL)
    {
        /* 还没接状态提供者: 如实回 1006, 【绝不】拿一堆 0 冒充状态发上去 */
        log_info("[CLOUD] sys.query_status: 本机还没注册状态提供者 -> 1006\r\n");
        return CLOUD_EC_STATE;
    }

    (void)memset(&st, 0, sizeof(st));
    s_status_fn(&st);

    if (!cloud_fill_status_json(data, &st))
    {
        return CLOUD_EC_INTERNAL;       /* 池满 */
    }

    /* 文档 5.1 第 1 条比 status.report 多这一个字段: bufferedEvents(尚未补报的缓存事件数)。
     * v59 起给【真值】★ —— 此前事件缓存还不存在(P4 未做), 只能"如实回 0";
     * 现在直接问缓存里还有多少条没补报出去。
     * 语义要点: 这是【待补报】的条数, 不是"历史上发生过多少"; 补报成功 drop 之后会变小 ——
     * 平台正是靠它在重连后判断"还要不要继续等补报"。 */
    if (cJSON_AddNumberToObject(data, "bufferedEvents",
                                (double)event_cache_count()) == NULL)
    {
        return CLOUD_EC_INTERNAL;
    }
    return CLOUD_EC_OK;
}
