/**
 * @file  cloud_cmd_cfg.c
 * @brief 配置类命令：cfg.set / cfg.query / sys.factory_reset（文档 5.2 / 5.1 第 4 条）
 *
 * 三条命令都围绕"整份配置"工作（约 1.7 KB），因此共用 cloud_proto.c 里的
 * 文件级暂存缓冲 cloud_cfg_staging_buf —— 单线程轮询、不会重入，放栈上会爆 4 KB。
 *
 * 写路径一律是：合并到副本 -> 校验 -> 落 Flash -> 成功才提交给当前实例。
 */
#include <stdio.h>
#include <string.h>

#include "cloud_proto.h"
#include "cloud_internal.h"
#include "cloud_json.h"
#include "cloud_profile.h"
#include "config_store.h"
#include "log.h"

/**
 * @brief cfg.set —— 文档 5.2
 *   data.expectVersion(uint32, 必填): 与当前 configVersion 不一致 -> 1002，且什么都不改；
 *   data.effectiveMode(string, 可选): immediate(默认) / reboot
 *   data.sections(object, 必填): 键为域名, 值为该域完整或部分字段
 *   应答 data: {"configVersion":7,"applied":["thresholds","sensors"],"rejected":[]}
 */
int cloud_cmd_cfg_set(const cloud_req_t *req, cJSON *data)
{
    const char           *sections;
    cfg_sections_result_t res;
    uint32_t              expect;
    uint32_t              cur;
    int                   rc = CLOUD_EC_OK;
    /* (用户决定不做 effectiveMode 回显 -> 这里不再需要记住它是 immediate 还是 reboot) */

    if (req->data == NULL)
    {
        return CLOUD_EC_PARAM;              /* 连 data 段都没有 */
    }

    /* ---- data.expectVersion(必填) ---- */
    if (cj_get(req->data, "expectVersion", NULL, 0) != CJ_NUM)
    {
        return CLOUD_EC_PARAM;              /* 缺失或不是数字 */
    }
    /* 超出 uint32 的巨型数字会落到这里给的哨兵值, 结果是与当前版本不符 -> 1002。
     * 1001/1002 都是"拒绝", 平台都看得懂, 不必为这种输入再分一档。 */
    expect = cj_get_uint(req->data, "expectVersion", 0xFFFFFFFFu);

    cur = config_store_version();
    if (expect != cur)
    {
        /* 【版本冲突必须什么都不改】否则平台会以为自己的旧配置被接受了 */
        return CLOUD_EC_VER_CONFLICT;
    }

    /* ---- data.effectiveMode(可选) ---- */
    if (cj_get(req->data, "effectiveMode", NULL, 0) != CJ_NOTFOUND)
    {
        char mode[16];

        if (!cj_get_str(req->data, "effectiveMode", mode, (uint16_t)sizeof(mode)))
        {
            return CLOUD_EC_PARAM;
        }
        if ((strcmp(mode, "immediate") != 0) && (strcmp(mode, "reboot") != 0))
        {
            return CLOUD_EC_PARAM;
        }
        /* 【本实现对两种模式的实际差别: 没有】����如实记档, 不假装有�         *   本工程的配置�丢�份结构体 + 立即�Flash", 运行期消费��阈��档位/规则)
         *   每拍都直接读� 扢��
         *     · immediate: 写完立即生效(与文档一�;
         *     · reboot   : 文档意��是"重启后再生效", 但本实现【不延迟应用】����         *                  值现在就落到运行期结构里了��差别只体现在那�本来就要
         *                  重启才生�的域�目前只有 network, �cfg.set 里那段说��         *   为什么不模拟 reboot: 那需要再存一�待生效副�并在启动时切� 对一�         *   单实例配置来说只会制�RAM �Flash 不一�的新隐患, 收益却只是让
         *   应答更贴文档。已在本条目的台账里记为【有意差异����         *   ⚠️ 本函数��不会��因�reboot 就自行重� 重启由平台显式下�sys.reboot�*/
        /* (只校验取值合法; 不再记住它 —— 用户决定不做回显) */
    }

    /* ---- data.sections(必填, 对象) ---- */
    /* cj_get_sub 返回的是指向原报文内部的指针(零拷�, 不保�NUL 结尾;
     * cloud_json 的路径解析是按括号配平走� 扢��子对象后面还有字�没关系��*/
    sections = cj_get_sub(req->data, "sections");
    if (sections == NULL)
    {
        return CLOUD_EC_PARAM;
    }

    /* ---- 逐域合并(每域独立事务: 合法的生效, 非法的被拒) ---- */
    (void)memcpy(&cloud_cfg_staging_buf, config_store_get(), sizeof(cloud_cfg_staging_buf));
    if (config_store_merge_sections(&cloud_cfg_staging_buf, sections, &res) != CFG_OK)
    {
        return CLOUD_EC_PARAM;
    }

    /* ---- 拼错的域名必须报出来, 不能默默忽略 ----
     * config_store_merge_sections() 只认 8 个已知域, sections 里其它键会被
     * 【静默丢弃】。于是平台把 "thresholds" 打成 "threshold", 我们回 code=0,
     * 而 applied / rejected 都是空的 —— 平台以为改成功了, 实际一个字没动。
     * cloud_json 没有"对象键遍历"接口, 所以这里用低成本判据:
     *   逐个数这 8 个已知域名有没有出现; 若【一个都没出现】而 sections 又不是空对象,
     *   那就是键名全拼错了 -> 回 1001, 让平台立刻看到。
     * 【已知残留】"部分拼错"(一个对 + 一个错)仍会被漏掉, 只表现为 applied 里少一项;
     *   要彻底解决需要给 cloud_json 加对象键遍历接口。 */
    {
        /* 注意 meta 也要"认识": 它是 9 个配置域之一, 只是【系统维护、不可写】。
         * 把它当拼写错误会让"只发 meta"的报文从 0 变成 1001 —— 那是两回事。 */
        static const char *const known[] = { "system", "network", "sensors", "thresholds",
                                             "gears", "devices", "rules", "ota", "meta" };
        uint8_t     n;
        uint8_t     present = 0u;
        const char *p;
        bool        is_empty_obj = false;

        for (n = 0u; n < 9u; n++)
        {
            if (cj_get(sections, known[n], NULL, 0) != CJ_NOTFOUND) { present++; }
        }

        /* sections 是不是空对象 {}(含空� 空对象是合法�仢�么都不改" */
        p = sections;
        while ((*p == ' ') || (*p == '\t') || (*p == '\n') || (*p == '\r')) { p++; }
        if (*p == '{')
        {
            p++;
            while ((*p == ' ') || (*p == '\t') || (*p == '\n') || (*p == '\r')) { p++; }
            if (*p == '}') { is_empty_obj = true; }
        }

        if ((present == 0u) && !is_empty_obj)
        {
            return CLOUD_EC_PARAM;
        }
    }

    /* ---- network � 【接受�� 按文�重启生效" ----
     * 文档 5.2 末原� "network 域修改后本网关重启生效，生效前旧连接保持"�     * 本工程的连接参数本来就是�config_store.network �—��main.c 弢�机时把它交给
     * g4_at_set_broker()(弢�机日志那�"[BOOT] broker 来自配置 network: ..." 就是�,
     * 而运行期【不会��热重连。所�写下�+ 重启生效 + 旧连接保�与文档完全一�
     * 应答也不撒谎: 平台重启网关后连的就是新地址�     *
     * ⚠️ 旧版本这里是�network 整域【拒掉��的, 理由�运行期读的是 param_save �     *    那一� —��那是 P2-7 统一之前的情况��现在读的就�config_store, 扢�以改判��     * ⚠️ 唯一必须守住的一� 【不要��在运行期热改连接参�—��那会打断正在跑的业务
     *    连接, 而文档明确要�生效前旧连接保持"。所以本函数丢�行都不碰 g4_at�*/
    if ((res.applied & CFG_CH_NETWORK) != 0u)
    {
        log_info("[CFG] cfg.set 改了 network �-> 已落� 【重启后生效�当前连接不受影响)\r\n");
    }

    /* ---- 有域生效才 +1 版本并落 Flash; 全部被拒时版本不动、Flash 不动 ---- */
    if (res.applied != 0u)
    {
        config_store_t *live;

        config_store_touch(&cloud_cfg_staging_buf, cloud_proto_get_now_ts());

        if (config_store_save(&cloud_cfg_staging_buf) != CFG_OK)
        {
            /* Flash 失败: 这些改动只存在于 RAM 副本� 掉电即失, 扢�以如实全�             * 记为 rejected 并回 1007。绝�应答说成功��实际没存住"�*/
            res.rejected |= res.applied;
            res.applied   = 0u;
            rc = CLOUD_EC_INTERNAL;
        }
        else
        {
            /* 落盘成功才提交给"当前配置"实例 —— 紧接着的 cfg.query 才能读到新值 */
            live = config_store_get_mut();
            (void)memcpy(live, &cloud_cfg_staging_buf, sizeof(*live));
        }
    }

    /* ---- 应答 data(文档 5.2: configVersion + applied[] + rejected[]) ---- */
    if ((cJSON_AddNumberToObject(data, "configVersion",
            (double)config_store_version()) == NULL) ||
        (!cloud_write_section_list(data, "applied",  res.applied)) ||
        (!cloud_write_section_list(data, "rejected", res.rejected)))
    {
        return CLOUD_EC_INTERNAL;       /* 池满: 明确失败, 绝不发半条应答 */
    }

    /* ★ 2026-09-30 用户决定: 【不做 rejectedDetail, 也不做 effectiveMode 回显】★
     *   · effectiveMode 在文档里本来就是【非必填】——收了就按"立即落盘"处理即可,
     *     不必回显; 本实现对 immediate/reboot 也无实际差别(见上面那段说明)。
     *   · rejected 只回域名数组(文档示例就是这个格式)。要给出原因的话需要把
     *     "哪个域、哪条规则"的原因从 config_store 逐级带出来, 而拒绝的可能太多,
     *     成本远大于收益 —— 用户明确不做。
     *   ⚠️ 所以 rejected 的非空只表示"这一域整域被拒"(文档 5.2 的语义),
     *      想知道具体原因要看网关日志。 */
    return rc;
}

int cloud_cmd_cfg_query(const cloud_req_t *req, cJSON *data)
{
    uint16_t mask = 0u;
    uint8_t  i;
    int      rc;

    rc = cloud_cfg_sections_mask(req, &mask);
    if (rc != CLOUD_EC_OK)
    {
        return rc;
    }

    /* data 对象已经由 cloud_proto_handle() obj_begin 过了, 这里只写成员 */
    if (!config_store_write_members(data, config_store_get()))
    {
        return CLOUD_EC_INTERNAL;           /* 池满 */
    }

    /* ★★2026-10-09 用户要求: 应答里的 network.password 固定返回 5 个 '*' ★★
     * --------------------------------------------------------------------------
     * 【为什么要掩码】broker 密码是敏感信息, 而 cfg.query 的应答会经 MQTT 下发,
     *   任何能发 cfg.query 或订阅 rt/gw/{gwId}/cmdresp 的一方都能看到明文
     *   (平台第 4 轮报告 §6.2 N6 也提了这条)。
     *
     * 【为什么改在这里, 而不是 config_store 的写出函数里】
     *   ⚠️ config_store_write_members() 很可能【同时】被"生成应答"和"序列化到
     *     Flash"两条路使用 —— 在那里掩码会把 '*****' 真的写进 Flash, 把真密码毁掉。
     *   改在 cfg.query 这一条路径上, 只影响"读出去的文本", 碰不到存储 ✓
     *
     * 【固定 5 个 *, 不是回显长度】用户口径: 固定 "*****"。
     *   用固定串而不是按长度生成, 是为了不泄露密码长度。
     *
     * ⚠️【配套风险, 见 cfg 的 set 侧】平台如果把这 5 个 '*' 原样回传 cfg.set,
     *   就会把密码真的设成 "*****"。⇒ set 侧必须把恰为 "*****" 的入参当作
     *   "不修改" 跳过(见 config_store.c 里 network.password 的取值处)。 */
    {
        cJSON *net = cJSON_GetObjectItem(data, "network");

        if (net != NULL)
        {
            cJSON_DeleteItemFromObject(net, "password");
            (void)cJSON_AddStringToObject(net, "password", "*****");
        }
    }

    /* �rule.query / sen.query 同一套办� 全写出来再删掉不要的�—��     * 这样字段名与键序永远与写入器同源, 不会漂��*/
    for (i = 0u; i < (uint8_t)CLOUD_CFG_SECTION_N; i++)
    {
        if ((mask & (uint16_t)(1u << i)) == 0u)
        {
            cJSON_DeleteItemFromObject(data, cloud_cfg_section_names[i]);
        }
    }
    return CLOUD_EC_OK;
}

/* 恢复出厂的工作副�+ 霢�要保留下来的 network 域�� * 都必须是文件�static: config_store_t ~1.7KB, �4KB 的栈上会� * (startup_n32h47x_48x.s �Stack_Size)。本工程是单线程轮询, 不会重入�*/
static config_store_t s_cfg_reset_tmp;
static cfg_network_t  s_keep_network;

int cloud_cmd_sys_factory_reset(const cloud_req_t *req, cJSON *data)
{
    bool         keep_network = true;       /* 文档默认值 true */
    cfg_result_t r;

    /* ---- data.keepNetwork(bool, 可�� 默认 true) ---- */
    if (req->data != NULL)
    {
        cj_type_t t = cj_get(req->data, "keepNetwork", NULL, 0u);

        if (t != CJ_NOTFOUND)
        {
            /* 文档类型是 bool; 也宽容接受 0/1(部分平台客户端把 bool 写成数字),
             * 但字符串/对象等一律 1001 —— 不猜它想表达什么。 */
            if ((t != CJ_BOOL) && (t != CJ_NUM))
            {
                return CLOUD_EC_PARAM;
            }
            keep_network = cj_get_bool(req->data, "keepNetwork", true);
        }
    }

    if (!keep_network)
    {
        /* 不保留 network: 直接用现成的 config_store_reset()
         * (= 填出厂默�+ updatedAt + �Flash), 与文�其余配置域恢复出厂默�
         * network 也一起回默认"完全丢�� 不必自己再拼丢�份��*/
        r = config_store_reset(&s_cfg_reset_tmp, cloud_proto_get_now_ts());
    }
    else
    {
        /* 【保�network 时为仢�么不能也�config_store_reset()�         * 它就�先填默认再落�, 会把 network 丢�起清� 若先 reset 再回�network
         * 并二次落� Flash 就会经历丢��network 被清�的中间��—��         * 那两次写盘之间掉� 网关恢复出厂后连不上 broker, 现场无法远程救回�         * 扢�以改� 填出厂默�-> 回填 network -> 【一次��写盘��*/
        const config_store_t *cur = config_store_get();

        s_keep_network = cur->network;              /* 先把要保留的域拷出来 */
        config_store_factory_default(&s_cfg_reset_tmp);
        s_cfg_reset_tmp.network         = s_keep_network;
        s_cfg_reset_tmp.meta.updated_at = cloud_proto_get_now_ts();   /* 与 config_store_reset 同一口径 */
        r = config_store_save(&s_cfg_reset_tmp);
    }

    if (r != CFG_OK)
    {
        /* 落盘失败 -> 1007, 且��内存里的当前配置一个字都不动��
         * 这样 RAM �Flash 不会出现"丢�个变了��一个没�的分裂状态��*/
        log_info("[CLOUD] !! sys.factory_reset 落 Flash 失败(code=%d), 回 1007, 当前配置未改动\r\n",
                 (int)r);
        return CLOUD_EC_INTERNAL;
    }

    /* 落盘成功才提交给"当前配置"实例 */
    (void)memcpy(config_store_get_mut(), &s_cfg_reset_tmp, sizeof(s_cfg_reset_tmp));

    /* ★★2026-10-11 补: 恢复出厂时【cloud_profile 也要回默认】★★
     * --------------------------------------------------------------------------
     * 【原来缺什么】`cloud_profile_reset()` 全工程【没有任何调用者】——
     *   本模块自己的注释(cloud_profile.c 顶部)就写着这件事, 一直没接上。
     *   ⇒ 后果: 平台发 sys.factory_reset 之后, config_store 那 9 个域确实回默认了,
     *     **但 cloud_profile 里的 broker / gwId / clientID 还是老的** ——
     *     现场表现是"恢复了出厂, 但网关还在连原来的 broker / 还用原来的 gwId",
     *     与"恢复出厂"的语义不符, 而且平台以为设备已经干净了。
     *
     * 【为什么放在这里(落盘成功之后)】
     *   与 config_store 一个口径: **先确认 Flash 落盘成功, 再动运行期状态**。
     *   前面任何一步失败都会 return 1007、当前配置一个字不改 —— cloud_profile
     *   也【不能】先重置, 否则会出现"config 没回默认、profile 却回默认了"的分裂状态。
     *
     * 【为什么不用管持久化】cloud_profile_reset() 自己只清内存里的 s_profile/s_loaded
     *   (见 cloud_profile.c), 下次用到时按出厂默认重新装载 —— 不需要额外写 Flash。
     *
     * ⚠️ 与 keepNetwork 无关: 那说的是 config_store.network 域; cloud_profile 是
     *   另一份(连到哪个 broker/用哪个 gwId), 恢复出厂就该一起回默认。 */
    cloud_profile_reset();
    log_info("[CLOUD] sys.factory_reset: cloud_profile 已一并回出厂默认(broker/gwId/clientID)\r\n");

    log_info("[CLOUD] sys.factory_reset 完成: keepNetwork=%s, configVersion=0, "
             "sensors=%u � devices=%u � 其余域已回出厂默认\r\n",
             keep_network ? "true" : "false",
             (unsigned)config_store_get()->sensor_count,
             (unsigned)config_store_get()->device_count);

    /* 文档这一条的响应只有 code 0, 没有 data 字段 —— 所以这里不往里写任何成员。 */
    (void)data;
    return CLOUD_EC_OK;
}
