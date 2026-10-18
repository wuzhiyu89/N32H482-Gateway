/**
 * @file  app_downlink.c
 * @brief 云端下行入口：一条 MQTT 消息该归谁处理
 *
 * 两条路，互斥：
 *   1. 以 '{' 开头 / 能被云协议层解析  -> cloud_proto（JSON 命令，回复 cmdresp）
 *   2. 其余（原始二进制帧）           -> 交给调用方继续走 LoRa 透传
 *
 * 这里不碰 AT：发布/回执都通过 g4_at_mqtt_publish() 登记，由 4G 状态机去发。
 */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

#include "log.h"
#include "app_core.h"
#include "app_state.h"
#include "cloud_proto.h"
#include "cloud_profile.h"
#include "config_store.h"
#include "g4_at.h"
#include "lora_business.h"
#include "ota_http.h"
#include "ota_port.h"

/* ==================== 云端 JSON 命令下行处理(P1-3) ====================
 *
 * 过渡期同一条下行通道上两种协议共存, 靠【内容嗅探】分流:
 *    以 '{' 开头          -> 云端 JSON 命令(RoadTwin), 走 cloud_proto, 回 cmdresp
 *    其它(0xAA 0x55 等)   -> 原有二进制 LoRa 帧透传, 交给 lora_business
 *
 * 【为什么用嗅探而不是按 topic 分】: "是不是 JSON" 与档位/topic 无关; 而且这样
 * 你在本地 broker 上往 rt/gw/{gwId}/cmd 发一条 JSON 就能直接验证整条链路。
 *
 * 【为什么要先拷进 NUL 结尾的缓冲】: MQTT 下发的 payload 是"字节流 + 长度",
 * 【不保证 NUL 结尾】(实测里含 0x00), 而 JSON 解析要求 NUL 结尾的 C 字符串。
 *
 * @return true = 已按云协议处理完(调用方不要再走 LoRa 透传)
 */
/* 【2026-09-27 放大: 384 -> CFG_JSON_MAX+64】
 * 原值 384 只装得下"改几个字段"的小下行; 而 cfg.set 允许"整份配置下发"
 * (实测 1203 B 默认 / 3584 B 满配), 超过 384 会在门口被丢掉 —— 而且【不回应答】,
 * 平台侧看起来就像网关死了。这里按单区配置的上限放大, 保证合法报文都进得来。
 * jsonbuf 本来就是 static, 代价只是 .bss; 改后整机 RAM 仍只用 ~26%, 很宽裕。 */
#define MAIN_CLOUD_JSON_MAX   (CFG_JSON_MAX + 64u)

/* (2026-10-08) 这里原来放过一版"字符串分片"(main_utf8_trim / main_json_escape /
 * main_publish_chunked) —— 因"必须转义引号"+"任一片丢就拼不齐"两个缺陷已删除。
 * 改为【按域分组】: 每片都是完整 JSON 对象, 零转义。见 cloud_proto 的分片接口。 */

bool app_downlink_try(const char *topic, const uint8_t *data, uint16_t len)
{
    static char jsonbuf[MAIN_CLOUD_JSON_MAX + 1u];
    /* 【必须 static, 不能用 CLOUD_RESP_MAX(512) 放栈上】(P2-4)
     * cfg.query 的应答正文 = 完整配置对象(默认 1203 B / 满配最坏 3584 B) + 信封,
     * 512 B 装不下(会溢出 -> 整个应答被判失败); 而 CFG_JSON_MAX+256 放栈上又会
     * 爆掉 4KB 的栈(startup_n32h47x_48x.s 的 Stack_Size)。
     * 本函数只在主循环里被调用, 不会重入, static 暂存安全。 */
    static char resp[CFG_JSON_MAX + 256u];
    char        rtopic[64];
    uint16_t    rlen = 0u;
    uint16_t    tlen;
    int         rc;

    /* ==================== 上板验证用: 每条 MQTT 下行的【原始正文】(v76) ====================
     * 【为什么必须打在这里、打在最前面】
     *   lora_business.c 的 MQTT 回调把【每一条】下行消息都先喂给本函数(前置过滤器),
     *   所以在这里打印 = 把"到底收没收到、收到的是什么字节"变成串口上看得见的东西。
     *   原来的写法有两种情况在串口上【完全没有日志】:
     *     · 报文不像 JSON -> 下面的 looks_like_json 直接 return false, 一行都不打;
     *     · 云端把下行发到了上行 topic(设备没订阅) -> 压根进不来这个函数
     *       (这种只能在 broker 侧看订阅关系, 设备侧无从证明)。
     *   ★ 可打印的按文本打; 含不可打印字节的改打 hex ★
     *   (下发给 LoRa 的 payload 是原始二进制, 用 %s 会被 0x00 截断/错位)
     *   ★ 打印上限 128 字节, 免得长报文把日志刷爆 ★ */
    if ((data != NULL) && (len > 0u))
    {
        uint16_t i;
        uint16_t show = (len > 128u) ? 128u : len;
        bool     printable = true;

        for (i = 0u; i < show; i++)
        {
            /* ★ v90: `\n` `\r` `\t` 也算"可打印" ★
             * 带缩进/换行的 JSON 是最常见的情形 —— 只有走**文本分支**才看得见内容。
             * 起因(v89 复盘): 那条 259 字节的下行含 `\n`, 于是整条被判为"不可打印"、
             * 走了 hex 分支、又被 48 字节上限截断, 我只能回头手工解码才看清是什么。
             * 真正的二进制帧(含 0x00/其它控制字节)仍然走 hex。 */
            if (((data[i] < 0x20u) && (data[i] != '\n') && (data[i] != '\r') &&
                 (data[i] != '\t')) || (data[i] > 0x7Eu))
            {
                printable = false;
                break;
            }
        }

        if (printable)
        {
            /* ★ 把 `\n` `\r` `\t` 转义成可见的两个字符再打 ★
             * 为什么不直接 %s: 含换行的 JSON 会把**一条日志切成好几行**,
             * 之后没法按行检索/过滤(本工程所有日志工具都是按行看的)。
             * 转义后既看得见内容, 又保住"一条日志一行"。 */
            char     tmp[2u * 129u + 1u];
            uint16_t n = 0u;

            for (i = 0u; i < show; i++)
            {
                if (data[i] == '\n')      { tmp[n] = '\\'; n++; tmp[n] = 'n'; n++; }
                else if (data[i] == '\r') { tmp[n] = '\\'; n++; tmp[n] = 'r'; n++; }
                else if (data[i] == '\t') { tmp[n] = '\\'; n++; tmp[n] = 't'; n++; }
                else                      { tmp[n] = (char)data[i]; n++; }
            }
            tmp[n] = '\0';
            log_info("[4G DOWN] topic=%s len=%u text: %s%s\r\n",
                     (topic != NULL) ? topic : "-", (unsigned)len, tmp,
                     (len > show) ? " ...(截断)" : "");
        }
        else
        {
            char     hex[3u * 48u + 1u];
            uint16_t n    = 0u;
            uint16_t hshow = (show > 48u) ? 48u : show;

            for (i = 0u; i < hshow; i++)
            {
                (void)snprintf(&hex[n], sizeof(hex) - n, "%02X ", data[i]);
                n += 3u;
            }
            hex[n] = '\0';
            log_info("[4G DOWN] topic=%s len=%u hex: %s%s\r\n",
                     (topic != NULL) ? topic : "-", (unsigned)len, hex,
                     (len > hshow) ? " ...(截断)" : "");
        }
    }

    if (!cloud_proto_looks_like_json(data, len))
    {
        /* ★★2026-10-14 安全加固: 云端【命令 topic】上只允许 JSON ★★
         * ------------------------------------------------------------------
         * 【为什么要加这道闸】下面那个 return false 会让调用方把它【空口透传】出去
         *   (lora_business 的"最简透传"路径)。而经历 2026-10-14 那次分包 bug 时,
         *   一条 1441 字节的 rule.set 被模块拆成 plen=489/512/440 三片, 后两片
         *   不以 '{' 开头 ⇒ 就走到了这里 ⇒ 差一点被当成 LoRa 载荷发出去:
         *       [4G DOWN] ^^ 这条【不像 JSON】, 交给 LoRa 透传路径
         *       [CLOUD->LoRa] 透传 512 字节到空口 ... !! 超过空口单包上限 240, 丢弃
         *   ⇒ 那次只是【侥幸】: 两片都 >240 字节, 被后面那道长度检查拦住了。
         *     只要哪一片 <=240 字节, 它就会被【真的发到空口】, 变成一条发给所有
         *     从机的无意义广播 —— 现场表现是"设备收到莫名其妙的指令"。
         *
         * 【判据】topic 以 "/cmd" 结尾 = 云端命令通道(rt/gw/{gwId}/cmd)。
         *   这里刻意【只看后缀】, 不去拼 CLOUD_TOPIC_CMD 再比较:
         *   少一个跨模块符号依赖, 而且命令通道的命名约定就是这样的后缀。
         *
         * 【丢弃而不是转发】命令通道上的"不像 JSON"只有两种来源 ——
         *   ① 平台发错了   ② 接收侧切碎了(分包/截断)。
         *   两种都不该被送上空口。⇒ 丢弃 + 留一条日志(便于现场取证)。
         *   ⚠️ 返回 true 表示"已处理(丢弃)", 调用方【不要】再走 LoRa 透传。 */
        size_t tlen = (topic != NULL) ? strlen(topic) : 0u;

        if ((tlen >= 4u) && (strcmp(topic + (tlen - 4u), "/cmd") == 0))
        {
            log_info("[4G DOWN] !! 命令 topic 上的下行【不像 JSON】(topic=%s, %u 字节) "
                     "-> 丢弃, 【不】空口转发(避免碎片被发到 LoRa 空口)\r\n",
                     topic, (unsigned)len);
            return true;
        }

        log_info("[4G DOWN] ^^ 这条【不像 JSON】, 交给 LoRa 透传路径\r\n");
        return false;
    }

    log_info("[CLOUD] 收到 JSON 下行(topic=%s, %u 字节)\r\n",
             (topic != NULL) ? topic : "-", (unsigned)len);

    if (len > (uint16_t)MAIN_CLOUD_JSON_MAX)
    {
        log_info("[CLOUD] !! JSON 下行超长(%u > %u), 拒绝\r\n",
                 (unsigned)len, (unsigned)MAIN_CLOUD_JSON_MAX);
        return true;
    }

    (void)memcpy(jsonbuf, data, len);
    jsonbuf[len] = '\0';

    app_refresh_epoch();
    rc = cloud_proto_handle(jsonbuf, app_now_unix(),
                            resp, (uint16_t)sizeof(resp), &rlen);
    log_info("[CLOUD] 处理完成 code=%d, 应答 %u 字节\r\n", rc, (unsigned)rlen);

    if (rlen == 0u)
    {
        return true;            /* 已处理(即使没生成出应答), 不要再去透传 */
    }

    /* 【弱网关键】把应答正文也打出来。现场 4G 常入不了网 / broker 连不上, 此时
     * MQTT 发不出去, 串口是唯一能看到"我们到底回了什么"的地方 —— 没有这一行,
     * 弱网下就只能看到"应答发布登记失败", 没法判断协议层本身对不对。 */
    log_info("[CLOUD] 应答正文: %s\r\n", resp);

    tlen = cloud_profile_build_topic(CLOUD_TOPIC_CMDRESP, rtopic, (uint16_t)sizeof(rtopic));
    if (tlen == 0u)
    {
        log_info("[CLOUD] !! 应答 topic 拼装失败, 应答丢弃\r\n");
        return true;
    }

    /* ★★2026-10-08 撤回: 原来这里调的 main_publish_chunked() 已删除★★
     * 那个实现把整份应答 JSON 【塞进一个字符串】("chunk":"<JSON 文本>")，有两个缺陷:
     *   ① 必须把 JSON 里的引号转义成 \" —— 平台要反转义, 而且显示很难看;
     *   ② N 条独立发布, 任一条丢就永远拼不齐。
     * 改为【按域分组】(每片是完整 JSON 对象, 零转义) —— 见 cloud_proto 的
     * 云命令应答分片接口。小应答仍然走下面的单条路径, 格式一字不变。 */
    if (g4_at_mqtt_publish_ex(rtopic, (const uint8_t *)resp, rlen,
                              cloud_profile_topic_qos(CLOUD_TOPIC_CMDRESP),
                              false) != SYS_OK)
    {
        /* ==========================================================================
         * ★★2026-10-08: 发布失败时【必须回一条错误应答】, 不能就这么算了★★
         * --------------------------------------------------------------------------
         * 【现场(2026-10-09)】平台发 `cfg.query sections:["all"]`, 网关算出 1551 字节
         *   应答, 但模组单条发布上限是 1024(超了会把模组 AT 应用层撑重启):
         *       [G4] !! 发布被拒: 1551 字节 > 单条上限 1024 —— 不硬发...
         *       [CLOUD] !! 应答发布登记失败(离线/队列满), 这条应答丢了
         *   ⇒ 平台【什么都收不到】, 只能干等到超时, 而且不知道该怎么办(改什么命令)。
         *
         * 【现在】改回一条【小的】错误应答: cmd/msgId 原样回带、code=1011、
         *   msg 明确告诉平台"应答过大, 请按域查询"。
         *   为什么重解析一次请求: cloud_proto_handle() 内部解析完就丢了, 而
         *   cloud_proto_build_resp() 需要一个 cloud_req_t 才能回带 cmd/msgId。
         *
         * 【为什么用独立的 static 小缓冲】错误应答只有 ~180 字节, 没必要占用
         *   上面那个 CFG_JSON_MAX 级的大缓冲; static 是为了不占本来就紧的栈。
         * ======================================================================== */
        static char eresp[256];
        cloud_req_t rq;
        uint16_t    elen;

        if (cloud_proto_parse_req(jsonbuf, &rq) == 0)
        {
            elen = cloud_proto_build_resp(eresp, (uint16_t)sizeof(eresp), &rq,
                                          CLOUD_EC_RESP_TOO_BIG,
                                          "response too large; query by section",
                                          NULL, app_now_unix());
            if ((elen > 0u) &&
                (g4_at_mqtt_publish_ex(rtopic, (const uint8_t *)eresp, elen,
                                       cloud_profile_topic_qos(CLOUD_TOPIC_CMDRESP),
                                       false) == SYS_OK))
            {
                log_info("[CLOUD] 已改回【错误应答】code=%d(%u 字节), 平台不会再干等: %s\r\n",
                         (int)CLOUD_EC_RESP_TOO_BIG, (unsigned)elen, eresp);
                return true;
            }
        }

        log_info("[CLOUD] !! 应答发布登记失败(离线/队列满/超长), 连错误应答也发不出去"
                 " —— 平台只能超时重试\r\n");
    }
    return true;
}

/* ==================== 4G 下行"只打印 + 回执"回调(LoRa 关闭时用) ====================
 *
 * 【为什么必须有】: MQTT 下行回调是在 lora_business_init() 里注册的
 * (g4_urc_set_recv_cb(lora_mqtt_recv_cb))。MAIN_LORA_ENABLE=0 时那个 init 不跑,
 * 于是【没有任何回调注册】—— 云端下发进来会被悄悄丢掉, 现场看起来就像
 * "4G 模组收不到下发消息", 其实是收到了没人处理(日志里也看不到)。
 *
 * 【回执机制 —— 现场要求把"定时上报"换成"收到就回执"】:
 * 定时上报是"我自己按节奏发", 云端收没收到全靠运气(而且实测很多都没到);
 * 回执是"一问一答": 云端既然发了指令, 就一定会等这条回执 —— 收不到就说明链路
 * 有问题, 责任和时机都清楚。回执里带上收到的原始字节(hex), 云端还能对上
 * 到底是哪一条指令被收到了。
 */
#if MAIN_LORA_ENABLE
static void app_downlink_log_cb(const char *topic, const uint8_t *data, uint16_t len)
{
    uint16_t i;
    char     ack[192];
    int      n = 0;
    sys_err_t r;

    /* P1-3: 先让云协议层试一次(JSON 命令)。归它管就到此为止, 不再往下走。 */
    if (app_downlink_try(topic, data, len))
    {
        return;
    }

    log_info("[4G DOWN] topic=%s, %u 字节\r\n",
             (topic != NULL) ? topic : "-", (unsigned)len);

    /* 【十六进制 + ASCII 双转储】: 下发的 payload 是【原始二进制】而不是 ASCII hex
     * (实测确认: "+MQTTURC: "publish",...,20,20," 后面直接跟 20 字节二进制,
     *  其中含 0x00/0x0A, 用 %s 打印会被截断/错位)。
     * 十六进制看"字节到底是什么", ASCII 看"能不能当文本读", 一起打才不会误判。 */
    if ((data != NULL) && (len > 0u))
    {
        log_info("[4G DOWN] HEX:");
        for (i = 0u; (i < len) && (i < 64u); i++)
        {
            log_info(" %02X", (unsigned)data[i]);
        }
        log_info("\r\n");

        log_info("[4G DOWN] ASC:");
        for (i = 0u; (i < len) && (i < 64u); i++)
        {
            uint8_t c = data[i];
            log_info("%c", ((c >= 0x20u) && (c < 0x7Fu)) ? (char)c : '.');
        }
        log_info("\r\n");
    }

    /* 回执: "ACK:" + 收到的字节(hex)。云端收到这条就知道指令到底有没有送达。 */
    n = snprintf(ack, sizeof(ack), "ACK:");
    if ((n > 0) && (data != NULL))
    {
        for (i = 0u; i < len; i++)
        {
            if ((n + 3) >= (int)sizeof(ack))
            {
                break;
            }
            n += snprintf(&ack[n], sizeof(ack) - (size_t)n, "%02X", (unsigned)data[i]);
        }
    }

    r = g4_at_mqtt_publish(NULL, (uint8_t *)ack, (uint16_t)n);
    if (r == SYS_OK)
    {
        log_info("[4G] 已回执: %s\r\n", ack);
    }
    else
    {
        log_info("[4G] !! 回执登记失败(发布忙/链路不在线), 这条回执丢了\r\n");
    }
}
#endif
