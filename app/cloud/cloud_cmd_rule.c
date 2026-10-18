/**
 * @file  cloud_cmd_rule.c
 * @brief 联动规则类命令：rule.set / rule.query / rule.mode（文档 5.5 第 16~18 条）
 *
 * rule.set  只允许改 gears 域（夹带别的域直接 1001，不静默忽略）
 * rule.query 先写全量再删掉不该带的域，避免把字段表抄第二份
 * rule.mode  改 rules.mode；切回 auto 时强制边缘评估重算一次
 */
#include <string.h>

#include "cloud_proto.h"
#include "cloud_internal.h"
#include "cloud_json.h"
#include "config_store.h"
#include "level_supervisor.h"
#include "log.h"

/**
 * @brief rule.mode — 文档 5.5 第 18 条：auto / manual
 *   参数: mode(auto/manual, 必填), manualTimeoutSeconds(可选，本次覆盖默认值)
 *   响应: code 0 + data.configVersion(配置变了, 让平台能对上版本)
 *
 * @note 文档还要求"切回 auto 时立即按当前水位重评估一次并刷新联动状态"。
 *       这件事由主循环的 manual 窗口回归负责(app_manual_step -> level_supervisor_force_next),
 *       本命令只改配置并如实应答。
 */
int cloud_cmd_rule_mode(const cloud_req_t *req, cJSON *data)
{
    char mode[12];
    int  rc;

    if (req->data == NULL)
    {
        return CLOUD_EC_PARAM;
    }
    if (!cj_get_str(req->data, "mode", mode, (uint16_t)sizeof(mode)))
    {
        return CLOUD_EC_PARAM;
    }

    (void)memcpy(&cloud_cfg_staging_buf, config_store_get(), sizeof(cloud_cfg_staging_buf));

    if (strcmp(mode, "auto") == 0)
    {
        cloud_cfg_staging_buf.rules.mode = (uint8_t)CFG_RULE_AUTO;
    }
    else if (strcmp(mode, "manual") == 0)
    {
        cloud_cfg_staging_buf.rules.mode = (uint8_t)CFG_RULE_MANUAL;
    }
    else
    {
        return CLOUD_EC_PARAM;              /* 只认这两个值，不猜 */
    }

    /* manualTimeoutSeconds 可选；给了就覆盖本次的默认值 */
    if (cj_get(req->data, "manualTimeoutSeconds", NULL, 0u) != CJ_NOTFOUND)
    {
        uint32_t t = cj_get_uint(req->data, "manualTimeoutSeconds", 0u);

        if (t > 0xFFFFu)
        {
            return CLOUD_EC_PARAM;
        }
        cloud_cfg_staging_buf.rules.manual_timeout_s = (uint16_t)t;
    }

    rc = cloud_commit_config(&cloud_cfg_staging_buf);
    if (rc != CLOUD_EC_OK)
    {
        return rc;
    }

    if (cJSON_AddNumberToObject(data, "configVersion",
                                (double)config_store_version()) == NULL)
    {
        return CLOUD_EC_INTERNAL;           /* 池满 */
    }
    return CLOUD_EC_OK;
}

/**
 * @brief rule.query — 文档 5.5 第 17 条
 *   响应 data: {"gears":[...], "rules":{mode, manualTimeoutSeconds, sensorOfflinePolicy}}
 *
 * 【实现取巧但刻意如此】直接复用 config_store_write_members() 把 9 个域全写出来,
 * 再把这条命令【不该带】的 7 个域删掉 —— 比在这里把 gears/rules 的字段表再抄一遍
 * 稳妥得多: 抄一遍就意味着"以后改配置结构要改两处", 两处必然漂移。
 * 代价只是多建/多删几个节点(键序不变: gears 在 rules 之前)。
 */
int cloud_cmd_rule_query(const cloud_req_t *req, cJSON *data)
{
    static const char *const drop[] =
    {
        "system", "network", "sensors", "devices", "thresholds", "ota", "meta"
    };
    uint8_t i;

    (void)req;                              /* 文档这条没有参数 */

    if (!config_store_write_members(data, config_store_get()))
    {
        return CLOUD_EC_INTERNAL;           /* 池满 */
    }

    for (i = 0u; i < (uint8_t)(sizeof(drop) / sizeof(drop[0])); i++)
    {
        cJSON_DeleteItemFromObject(data, drop[i]);
    }
    return CLOUD_EC_OK;
}

/**
 * @brief rule.set — 文档 5.5 第 16 条：设置联动档位动作(gears 数组, 按 gear 合并)
 *   响应: code 0 + data.configVersion
 *
 * 【实现为什么复用 config_store_merge_sections】gears 域的"按 gear 字段定位槽位"
 * 已经在那里实现并测过(单测 [12]): 给一个 {gear:2,...} 只改 2 号档, 0 号档不动,
 * 而且槽位不变(gear[i]==i 仍成立)。自己再抄一份"按 gear 找槽位"必然漂移。
 *
 * 【只允许改 gears】这条命令的语义就是改档位。如果平台在 data 里顺手夹带了
 * thresholds 之类的域, 必须【明确拒绝】—— 不能"只挑 gears 生效、其余静默忽略",
 * 否则会让平台以为整份配置改成功了。所以先扫一遍已知域, 不是 gears 的出现了就回 1001。
 */
int cloud_cmd_rule_set(const cloud_req_t *req, cJSON *data)
{
    cfg_sections_result_t res;
    uint8_t               i;
    int                   rc;

    if (req->data == NULL)
    {
        return CLOUD_EC_PARAM;
    }

    if (cj_get(req->data, "gears", NULL, 0u) == CJ_NOTFOUND)
    {
        return CLOUD_EC_PARAM;              /* 必须带 gears */
    }
    for (i = 0u; i < (uint8_t)CLOUD_CFG_SECTION_N; i++)
    {
        if (strcmp(cloud_cfg_section_names[i], "gears") == 0)
        {
            continue;                       /* 只有 gears 是允许的 */
        }
        if (cj_get(req->data, cloud_cfg_section_names[i], NULL, 0u) != CJ_NOTFOUND)
        {
            log_info("[CLOUD] rule.set 只允许改 gears, data 里出现了 %s -> 1001\r\n",
                     cloud_cfg_section_names[i]);
            return CLOUD_EC_PARAM;
        }
    }

    (void)memset(&res, 0, sizeof(res));
    (void)memcpy(&cloud_cfg_staging_buf, config_store_get(), sizeof(cloud_cfg_staging_buf));

    if (config_store_merge_sections(&cloud_cfg_staging_buf, req->data, &res) != CFG_OK)
    {
        return CLOUD_EC_PARAM;              /* 入参非法 */
    }
    if ((res.applied & CFG_CH_GEARS) == 0u)
    {
        return CLOUD_EC_PARAM;              /* 被校验拒绝 / 没生效，不假装成功 */
    }

    rc = cloud_commit_config(&cloud_cfg_staging_buf);
    if (rc != CLOUD_EC_OK)
    {
        return rc;
    }

    if (cJSON_AddNumberToObject(data, "configVersion",
                                (double)config_store_version()) == NULL)
    {
        return CLOUD_EC_INTERNAL;           /* 池满 */
    }
    return CLOUD_EC_OK;
}
