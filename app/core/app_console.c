/**
 * @file  app_console.c
 * @brief USART1 日志口：手敲 AT 控制台 + 云协议离线自测 + 周期状态 / 复位原因
 *
 * 这里只做"人机接口"：把串口上的一行 `#命令` 翻译成对业务层的调用，
 * 以及把当前状态打印成人能读的日志。真正的业务动作都在 cloud/links/service。
 *
 * ⚠ 本文件里的缓冲区都是 file-static：栈只有 4 KB（见 docs/ARCHITECTURE.md）。
 */
#include "n32h47x_48x.h"
#include "n32h47x_48x_gpio.h"
#include "n32h47x_48x_rcc.h"
#include "n32h47x_48x_usart.h"
#include "misc.h"
#include <string.h>
#include <stdio.h>

#include "log.h"
#include "bsp_tim.h"
#include "app_core.h"
#include "app_config.h"
#include "app_state.h"
#include "cloud_proto.h"
#include "cloud_profile.h"
#include "config_store.h"
#include "g4_at.h"
#include "lora_business.h"
#include "sensor_offline.h"
#include "last_value.h"
#include "param_save.h"  /* device_param_t / PARAM_FIELD_LEN (#set/#get) */
#include "bsp_gpio.h"   /* G4_PowerSwitch_* */
#include "g4_io.h"      /* g4_io_rx_total / ovf / 下行统计 */
#include "g4_urc.h"

/* ==================== 日志口手敲 AT 控制台 ====================
 * USART1 接收中断 → 环形缓冲 → app_console_poll 拼行 → g4_io_send_cmd 转发给 4G。
 * 便于台架上直接敲 AT+CSQ? / AT+MQTTCONN? 等命令排障。
 */
#if MAIN_AT_CONSOLE_ENABLE

/* 一行必须装得下真实的云命令 JSON：`#json {"cmd":"ota.start",...}` 约 200 字节。
 * 128 字节时会被静默截断 —— 表现为"敲了 #json 却什么都没发生"，
 * 连带 #json 离线自测根本无法注入 cfg.set / ota.start 这类真实命令。
 * RAM 代价：这块从 128 提到 384，两个缓冲共 +512 B（本工程 RW+ZI 仍有余量）。 */
#define CONSOLE_BUF_SIZE   384U

static volatile uint8_t  s_con_buf[CONSOLE_BUF_SIZE];
static volatile uint16_t s_con_wr = 0U;
static volatile uint16_t s_con_rd = 0U;
static char              s_con_line[CONSOLE_BUF_SIZE];

void USART1_IRQHandler(void)
{
    if (USART_GetIntStatus(USART1, USART_INT_RXDNE) != RESET)
    {
        uint8_t  ch   = (uint8_t)(USART_ReceiveData(USART1) & 0xFFU);
        uint16_t next = (uint16_t)((s_con_wr + 1U) % CONSOLE_BUF_SIZE);

        /* ★2026-10-23: 只收 ASCII 可打印字符与 \r\n。
         * 拔掉串口助手后 PA10 会悬空拾取噪声, 这些字节原来会被拼成随机"行",
         * 再被当成 AT 命令转发给 4G 模组 -> 模组被乱码干扰到重启(+MATREADY),
         * 同时乱码里的 \r 会让终端日志覆盖/错位。这里从源头丢掉不可打印字节,
         * 硬件上拉(log_init)是第二道防线。 */
        if (((ch < 0x20U) || (ch > 0x7EU)) && (ch != (uint8_t)'\r') && (ch != (uint8_t)'\n'))
        {
            USART_ClrFlag(USART1, USART_FLAG_RXDNE);
            return;
        }

        if (next != s_con_rd)   /* 满则丢弃, 不覆盖未处理数据 */
        {
            s_con_buf[s_con_wr] = ch;
            s_con_wr = next;
        }
        USART_ClrFlag(USART1, USART_FLAG_RXDNE);
    }
}

void app_console_init(void)
{
    USART_ConfigInt(USART1, USART_INT_RXDNE, ENABLE);
    NVIC_SetPriority(USART1_IRQn, NVIC_EncodePriority(5, 3, 3));
    NVIC_EnableIRQ(USART1_IRQn);
}

void app_console_poll(void)
{
    static uint16_t len = 0U;

    while (s_con_rd != s_con_wr)
    {
        char c = (char)s_con_buf[s_con_rd];

        s_con_rd = (uint16_t)((s_con_rd + 1U) % CONSOLE_BUF_SIZE);

        if ((c == '\r') || (c == '\n'))
        {
            if (len > 0U)
            {
                s_con_line[len] = '\0';

#if MAIN_G4_ENABLE
                /* 以 '#' 开头 = 本地诊断命令(#pwr? / #pwrswap / #cycle / #help...),
                 * 【不转发给模组】。现场排"模组没电/电压不稳/不回话"用, 见 g4_at.c。 */
                if (s_con_line[0] == '#')
                {
                    if (!g4_at_console_cmd(s_con_line))
                    {
                        log_info("[CONSOLE] 未知本地命令 %s (敲 #help 看列表)\r\n", s_con_line);
                    }
                    len = 0U;
                    continue;
                }
#endif
                /* ★2026-10-23: 只转发 AT 命令。噪声即使凑成可打印字符, 也几乎
                 * 不可能刚好以 "AT" 开头; 这条把随机行彻底挡在模组之外。
                 * (非 AT 行静默丢弃 —— 噪声场景下打印反而会刷屏。) */
                if (!(((s_con_line[0] == 'A') || (s_con_line[0] == 'a')) &&
                      ((s_con_line[1] == 'T') || (s_con_line[1] == 't'))))
                {
                    len = 0U;
                    continue;
                }

                /* 其它行: 整行当 AT 命令转发给 4G 模组。
                 * (以前这里还支持"以 { 开头当云端 JSON 喂给下行处理函数",
                 *  那个台架后门已经删掉了 —— 下行只走 4G/MQTT。) */
                log_info("[CONSOLE->4G] %s\r\n", s_con_line);
                g4_io_send_cmd(s_con_line);
                len = 0U;
            }
        }
        else if (len < (CONSOLE_BUF_SIZE - 1U))
        {
            s_con_line[len] = c;
            len++;
        }
    }
}

#else  /* !MAIN_AT_CONSOLE_ENABLE */

void app_console_init(void)  {}
void app_console_poll(void)  {}

#endif /* MAIN_AT_CONSOLE_ENABLE */

/* ==================== 状态打印 ==================== */

void app_status_report(void)
{
    uint8_t       online_cnt = 0U;
    uint8_t       total      = lora_business_get_slave_count();
    uint8_t       i;
    lora_slave_t *list = lora_business_get_slave_list();
    const g4_module_info_t *gi = g4_at_get_info();
    const g4_cereg_diag_t  *cd = g4_at_get_cereg_diag();
    char          vbat_text[16];
    char          hb_text[16];
    char          cereg_text[24];

    /* ★2026-10-18: 与 status.report / dev.query 统一口径(用 lora_business_device_state_bytes)
     *   以前这里直接看 is_online —— 那是"收到过帧/绑定过"的粘滞标志, 声光断电后
     *   仍然是 true, 于是串口显示"在线从机=1/1"而平台收到的 devicesOnline=0。
     *   现在三处(本行、status.report、dev.query)算的是同一件事。 */
    for (i = 0U; (i < total) && (list != NULL); i++)
    {
        if (lora_business_device_state_bytes(list[i].dev_uid) == LORA_DEV_STATE_ONLINE)
        {
            online_cnt++;
        }
    }

    /* VBAT 是【模组自己 ADC 测的供电】, 比万用表准(万用表读的是平均值)。
     * 规格 3.4~4.2V, 典型 3.8V。 */
    if (gi->vbat_valid)
    {
        snprintf(vbat_text, sizeof(vbat_text), "%umV", (unsigned)gi->vbat_mv);
    }
    else
    {
        vbat_text[0] = '-';
        vbat_text[1] = '\0';
    }

    /* 距上次成功发布(心跳)的秒数 —— 【判断保活是不是断了就看这个数】
     * 在线时它应该始终 < 30s 来回; 一旦只涨不回头, 就是心跳发不出去了。 */
    if (g4_at_pub_age_ms() != 0xFFFFFFFFu)
    {
        snprintf(hb_text, sizeof(hb_text), "%lus",
                 (unsigned long)(g4_at_pub_age_ms() / 1000u));
    }
    else
    {
        hb_text[0] = '-';
        hb_text[1] = '\0';
    }

    /* 驻网(CEREG)只读观测: 紧凑显示。
     * 【默认模式下 stat 恒为 0xFF】—— 因为出厂固件【不轮询】AT+CEREG?(ONLINE 期间
     * 不许发 AT, 理由见 g4_at.h 那段长注释), 只靠模组主动上报。所以这一栏在默认
     * 模式下的含义是"有没有把主动上报打开"(armed), 真正的证据在:
     *   · "[REG] 驻网行(原始)" / "[REG] !! PS 去附着" 那几行日志(事件发生时才有)
     *   · [STAT2] 里的 URC行 / CGEV / 去附着 累计计数
     * 台架若把 G4_CEREG_POLL_MS 打开, 这里就会显示实际轮询到的 stat。 */
    if (cd->stat != 0xFFu)
    {
        if (cd->bad_run != 0u)
        {
            snprintf(cereg_text, sizeof(cereg_text), "%u!!连续%u次",
                     (unsigned)cd->stat, (unsigned)cd->bad_run);
        }
        else
        {
            snprintf(cereg_text, sizeof(cereg_text), "%u", (unsigned)cd->stat);
        }
    }
    else if (cd->urc_armed)
    {
        snprintf(cereg_text, sizeof(cereg_text), "armed");
    }
    else
    {
        snprintf(cereg_text, sizeof(cereg_text), "-");
    }

    log_info("[STAT] 4G=%s | HB=%s(距上次发布成功) | PA8=%s(4G电源,%s) RX累计=%lu | "
             "IMEI=%s CFUN=%d CSQ=%u(%ddBm) IP=%s VBAT=%s CEREG=%s | "
             "LoRa=%s 在线从机=%u/%u | 下发缓存=%u | tick=%lu\r\n",
             g4_at_is_online() ? "ONLINE" : "offline",
             hb_text,
             G4_PowerSwitch_GetLevel() ? "H" : "L",
             G4_PowerSwitch_GetOnHigh() ? "高电平上电" : "低电平上电",
             (unsigned long)g4_io_rx_total(),
             (gi->imei[0] != '\0') ? gi->imei : "-",
             (int)g4_at_get_cfun(),      /* 1=全功能 0=最小功能(射频关,不会搜网) -1=没读到 */
             (unsigned)gi->csq, (int)gi->rssi_dbm,
             (gi->ip[0] != '\0') ? gi->ip : "-",
             vbat_text,
             cereg_text,
#if MAIN_LORA_ENABLE
             lora_business_radio_stuck() ? "模组无应答!" : "OK",
#else
             "关闭",
#endif
             (unsigned)online_cnt, (unsigned)total,
             (unsigned)data_cache_down_get_count(),
             (unsigned long)get_tick_ms());

    /* 【链路健康计数 —— 排"收不到 / 掉线"先看这一行】
     *   丢字节   : USART2 溢出 + 环形缓冲写满而丢掉的字节数。>0 = 接收真的被压垮过,
     *              下一步就是加大缓冲 / 降日志 / 让云端放慢下发, 而不是瞎猜。
     *   收包重同步: 下行收包中途被截断(超时/长度非法)而放弃的次数。>0 = 云端下行
     *              确实丢过消息, 但【不会再把整条 AT 流带偏】—— 这是新加的兜底。
     *   日志丢   : 收包路径日志被限流丢掉的条数。用来判断"日志本身是不是瓶颈"。
     *   下行     : 收到的云端消息条数; 括号里是下行队列满丢掉的条数。
     *   puback   : broker 确认过的上行条数 —— 【这才是"云端确实收到了"的证据】,
     *              模组回 OK 只代表它把数据交给了自己的 MQTT 栈。
     *   模组报就绪: 模组报过多少次 "+MATREADY"/"RDY", 也就是它的 AT 应用层重启过
     *              多少次 —— 【这才是"模组自己重启了没有"的可靠计数】。
     *              老版本按"离上次发送是否 >3s"分档, 而注册阶段每 2s 就问一条命令,
     *              于是真重启也被算进"应答习惯"那一档, 这个数一路是 0
     *              (现场日志 78.txt: +MATREADY 明明出现了 4 次)。现在给总数 +
     *              括号里的"紧随我们发送"那一档, 两者对比着看。
     *   '>'丢失重发: 发 MQTTPUB 时模组把数据模式提示 '>' 吞掉的次数。
     *              现在不再直接断电重启, 而是先敲 AT 探活、活着就重发(见 g4_at.h)。
     *   MQTT快修  : 只重连 MQTT(MQTTDISC->MQTTCONN->MQTTSUB)而【不给模组断电】的次数。
     *              现场病灶是"一发 PUBLISH 这个会话就死": 第 1 条收下但 broker 不回
     *              puback(云端收不到), 第 2 条起回 +CME ERROR: 601; 而模组【不发
     *              conn URC】, 状态机只能靠自己发现。快修 2~5 秒, 断电要 40~60 秒,
     *              所以现在优先快修, 连续多次救不回来才升级到断电。 */
    /* ---------- LoRa 接收状态 ----------
     * 【现场要求: 网关的 LoRa 一直在接收状态】。这一栏把它变成【可验证的】:
     *   · "在收" 是直接读【芯片状态字节】得出的(不是读软件缓存) ——
     *     芯片真的在 RX 才显示"在收";
     *   · "补开" 是"发现它不在接收并重新切回 RX"的累计次数。这个数一涨, 就说明
     *     确实漏过(发射超时 / 模式漂移), 而不是我们以为它一直在收。
     * ⚠️ 只有开了 LoRa 才读芯片: 没上电的芯片读状态会卡在 BUSY。 */
    {
        const char *lora_rx;
        uint32_t    lora_rearm;

#if MAIN_LORA_ENABLE
        lora_rx    = LoraApp_IsRxActive() ? "在收" : "!!没在收";
        lora_rearm = LoraApp_RxRearmCount();
#else
        lora_rx    = "关闭";
        lora_rearm = 0u;
#endif

        log_info("[STAT2] 丢字节=%lu 收包重同步=%lu 日志丢=%lu | "
                 "下行=%lu(队列满丢 %lu) puback=%lu | "
                 "模组报就绪=%lu(其中紧随发送的 %lu) '>'丢失重发=%lu MQTT快修=%lu | "
                 "CEREG轮询=%lu 未驻网=%lu URC行=%lu CGEV=%lu(去附着 %lu) | "
                 "断电重启=%lu 退避=%lus | "
                 "LoRa RX=%s 补开=%lu\r\n",
                 (unsigned long)g4_io_rx_ovf_count(),
                 (unsigned long)g4_io_dl_abort_count(),
                 (unsigned long)g4_io_rxlog_drop_count(),
                 (unsigned long)g4_urc_pub_rx_count(),
                 (unsigned long)g4_urc_pub_drop_count(),
                 (unsigned long)g4_urc_puback_count(),
                 (unsigned long)g4_io_ready_total(),
                 (unsigned long)g4_io_ready_after_tx_count(),
                 (unsigned long)g4_at_prompt_loss_count(),
                 (unsigned long)g4_at_mqtt_fast_count(),
                 (unsigned long)cd->poll_cnt,
                 (unsigned long)cd->bad_total,
                 (unsigned long)cd->urc_line_cnt,
                 (unsigned long)cd->cgev_cnt,
                 (unsigned long)cd->cgev_detach_cnt,
                 (unsigned long)g4_at_get_rst_count(),
                 (unsigned long)(g4_at_get_backoff_ms() / 1000u),
                 lora_rx,
                 (unsigned long)lora_rearm);
    }
}

/* ==================== 复位原因 ====================
 *
 * 【为什么第一件事就要打这个】: 现场出现"模组一开始工作, 板子就自己重启"时,
 * 唯一有价值的问题是"谁把我复位的"。本工程【没有任何看门狗】(全工程没有 IWDG/WWDG),
 * 所以复位只可能是这几种:
 *   BOR  欠压复位 → 电源被拉塌了(模组上电浪涌/供电能力不足) ← 最常见
 *   POR  上电复位 → 冷启动(正常)
 *   PIN  NRST 脚被外部拉低 → 板上复位电路/别人按着
 *   SOFT 软件复位 → 有代码主动 NVIC_SystemReset(本工程只有 OTA 里有, 默认关)
 *   IWDG/WWDG     → 看门狗(本工程没开, 出现就说明配置被改了)
 *
 * 读完就把标志清掉, 这样下一次复位的"真凶"不会被这一次的残留掩盖 ——
 * 反复重启时, 每一轮打印的就是【刚刚那次】的原因。
 */
void app_report_reset_cause(void)
{
    uint8_t bor  = (RCC_GetFlagStatus(RCC_FLAG_BORRSTF) != RESET) ? 1U : 0U;
    uint8_t por  = (RCC_GetFlagStatus(RCC_FLAG_PORRSTF) != RESET) ? 1U : 0U;
    uint8_t pin  = (RCC_GetFlagStatus(RCC_FLAG_PINRSTF) != RESET) ? 1U : 0U;
    uint8_t soft = (RCC_GetFlagStatus(RCC_FLAG_SFTRSTF) != RESET) ? 1U : 0U;
    uint8_t iwdg = (RCC_GetFlagStatus(RCC_FLAG_IWDGRSTF) != RESET) ? 1U : 0U;

    RCC_ClearResetFlag();

    log_info("[RST] 上次复位: BOR(欠压)=%u POR(上电)=%u PIN(NRST脚)=%u SOFT(软复位)=%u IWDG=%u\r\n",
             (unsigned)bor, (unsigned)por, (unsigned)pin, (unsigned)soft, (unsigned)iwdg);

    if ((bor != 0U) && (por == 0U))
    {
        log_info("[RST] !! 是【欠压复位】—— 电源被拉塌了, 不是代码问题。\r\n");
        log_info("[RST] !! 查: 模组上电浪涌 / 电源(电池)带载能力 / 模组 VCC 处的储能电容。\r\n");
    }
}

/* ==================== 日志口云协议离线自测(#json / #status / #topic / #offline) + 射频诊断(#lora) ====================
 *
 * 【为什么需要】现场 4G 常处于弱网: 模组入不了网 / broker 连不上, 于是
 * "云端下发 -> 网关应答"这条链路根本没法验。但云协议层的全部代码(信封解析、
 * 21 条命令表、授权校验、应答组包、档位 topic 映射)其实【一行都不依赖网络】,
 * 缺的只是一个把下行字节喂进 app_downlink_try() 的入口。
 *
 * 这三条命令就是那个入口, 而且【走的就是 MQTT 下行那一个函数】, 不另开分支 ——
 * 所以串口上测通 = 真链路也通, 唯一差别只是应答最后能不能发出去(发不出去也会
 * 打印出来, 见 app_downlink_try 里的"应答正文")。
 *
 * 这一段纯属台架调试, 量产可把 MAIN_AT_CONSOLE_ENABLE 改成 0 整段去掉。
 */
#if MAIN_AT_CONSOLE_ENABLE

/* 内置样例: 选 sys.query_status —— 它在 21 条命令表里, 但处理函数目前还是占位
 * (回 1006)。正好一次把"信封解析 + 授权 + 查表 + 应答组包"四步全走到,
 * 而且不用自己敲花括号和引号(串口终端常把引号吃掉)。 */
#define MAIN_CONSOLE_JSON_DEMO \
    "{\"cmd\":\"sys.query_status\",\"msgId\":\"t1\",\"ts\":1700000000}"

/** ★2026-10-18: #lorasend 用 —— 单个 hex 字符转值; 非 hex 返回 -1 */
static int console_hex_digit(char c)
{
    if ((c >= '0') && (c <= '9')) { return (int)(c - '0'); }
    if ((c >= 'a') && (c <= 'f')) { return (int)(c - 'a') + 10; }
    if ((c >= 'A') && (c <= 'F')) { return (int)(c - 'A') + 10; }
    return -1;
}

void app_console_help(void)
{
    log_info("[DIAG] #json [payload] 把下行 JSON 喂进云协议层(离线验证 P1 全链路; "
             "不带参数用内置样例)\r\n");
    log_info("[DIAG] #status 立刻组一条 status.report 并打印正文\r\n");
    log_info("[DIAG] #topic 打印当前档位的 5 个 topic / QoS / retained\r\n");
    log_info("[DIAG] #offline 打印各液位从机的上次上报/阈值/在线状态, 并立刻跑一次失联检查\r\n");
    log_info("[DIAG] #lora 射频诊断: 回读芯片实际配置(频率/同步字/模式/IRQ/统计/BUSY) + 一句话判读; "
             "只读不改, 不打断接收\r\n");
    log_info("[DIAG] #lorasend <hex> 空口【逐字节透传】发一串字节(不加帧头/不算 CRC), "
             "例: #lorasend AA5501AABB\r\n");
    log_info("[DIAG] #bind [sensor|device] [秒] 本地开绑定窗口 —— 拔掉 SIM 卡(无 4G)也能绑设备; "
             "默认 sensor 30 秒\r\n");
    log_info("[DIAG] #set <10字符> 空口写【设备】出厂字段(0x06 每5s一条, 最多3条); "
             "#set self <10字符> 写网关自己的(落 Flash, 云端不可见)\r\n");
    log_info("[DIAG] #get 空口读【设备】字段(0x07 只发一次, 等约12s; 设备会重发0x08); "
             "#get self 打印网关自己的\r\n");
    log_info("[DIAG] #unbind <uid> 本地解绑(与云端同一条流程: 发0x05 等设备 ack), 不需要 4G\r\n");
}

bool app_console_ext(const char *line)
{
    /* ---------------- #json ---------------- */
    if (strncmp(line, "#json", 5) == 0)
    {
        const char *p = line + 5;

        while (*p == ' ') { p++; }

        if (*p == '\0')
        {
            p = MAIN_CONSOLE_JSON_DEMO;
            log_info("[TEST] #json 未带参数, 用内置样例\r\n");
        }

        log_info("[TEST] 注入下行 JSON (%u 字节): %s\r\n", (unsigned)strlen(p), p);

        /* 和 MQTT 下行【同一个函数】, 不另开分支 */
        (void)app_downlink_try("(console)",
                                      (const uint8_t *)p, (uint16_t)strlen(p));

        log_info("[TEST] 注入结束(应答能否发出去取决于 4G 是否在线, 正文见上)\r\n");
        return true;
    }

    /* ---------------- #status ---------------- */
    if (strcmp(line, "#status") == 0)
    {
#if MAIN_STATUS_JSON_ENABLE
        log_info("[TEST] 立刻组一条 status.report:\r\n");
        (void)app_publish_status_json();
#else
        log_info("[TEST] MAIN_STATUS_JSON_ENABLE=0, status.report 未编译进来\r\n");
#endif
        return true;
    }

    /* ---------------- #topic ---------------- */
    if (strcmp(line, "#topic") == 0)
    {
        const cloud_profile_t *p = cloud_profile_get();
        char    t[80];
        uint8_t i;

        if (p == NULL)
        {
            log_info("[TEST] !! cloud_profile_get() 返回空\r\n");
            return true;
        }

        log_info("[TEST] 档位=%s host=%s:%u gwId=%s prefix=%s\r\n",
                 cloud_profile_is_doc() ? "DOC(文档)" : "LOCAL(本地)",
                 p->host, (unsigned)p->port, p->gw_id, p->topic_prefix);
        log_info("[TEST] 身份映射=%s 授权校验=%s\r\n",
                 p->id_map_enable ? "开" : "关",
                 p->auth_enable   ? "开" : "关");

        for (i = 0u; i < (uint8_t)CLOUD_TOPIC_COUNT; i++)
        {
            t[0] = '\0';
            if (cloud_profile_build_topic((cloud_topic_t)i, t, (uint16_t)sizeof(t)) == 0u)
            {
                (void)snprintf(t, sizeof(t), "(拼装失败)");
            }
            log_info("[TEST]   %s -> %s [QoS%u%s]\r\n",
                     cloud_profile_topic_name((cloud_topic_t)i), t,
                     (unsigned)cloud_profile_topic_qos((cloud_topic_t)i),
                     cloud_profile_topic_retained((cloud_topic_t)i) ? " retained" : "");
        }
        return true;
    }

    /* ---------------- #offline ---------------- */
    /* 液位失联检测的"看得见"入口(2026-09-27 用户补充要求第 3 条)。
     * 现场要验这条逻辑很不方便: 阈值默认是 3 × 上报周期(60s 周期 = 180s 没上报),
     * 而且弱网下事件根本发不出去。所以这里把【判定用的每个量】都打出来, 再顺手
     * 跑一次检查 —— 与 lora_business 内部用的是同一个纯函数(sensor_offline.c),
     * 串口上看到的结论就是真实结论, 不是另算一套。
     * ⚠ 只读+检查, 不伪造事件: 真到阈值了才会真的发(发不出去就是发不出去)。 */
    if (strcmp(line, "#offline") == 0)
    {
        const config_store_t *cfg = config_store_get();
        lora_slave_t         *sl  = lora_business_get_slave_list();
        uint8_t               cnt = lora_business_get_slave_count();
        uint32_t              now = get_tick_ms();
        uint8_t               i, shown = 0u;

        log_info("[TEST] 液位失联检测: 阈值 = 上报周期(config_store) × %u\r\n",
                 (unsigned)SENSOR_OFFLINE_PERIOD_MULT);

        for (i = 0u; (i < cnt) && (sl != NULL); i++)
        {
            char     uid[CFG_UID_STR_MAX];
            uint16_t period_s;

            if (sl[i].dev_type != SLAVE_TYPE_SENSOR) { continue; }

            cfg_uid_from_bytes(sl[i].dev_uid, uid);
            period_s = sensor_offline_period_for_uid(cfg, uid);

            log_info("[TEST]   %s 周期=%us(0/查不到则用默认%us) 阈值=%lums "
                     "上次上报=%lums 距今=%lums 在线=%s 已报失联=%s\r\n",
                     uid, (unsigned)period_s, (unsigned)SENSOR_OFFLINE_DEFAULT_PERIOD_S,
                     (unsigned long)sensor_offline_threshold_ms(period_s),
                     (unsigned long)sl[i].last_report_tick,
                     (unsigned long)(uint32_t)(now - sl[i].last_report_tick),
                     sl[i].is_online ? "是" : "否",
                     sl[i].offline_reported ? "是" : "否");
            shown++;
        }

        if (shown == 0u)
        {
            log_info("[TEST]   从机表里【没有】液位从机(dev_type=SENSOR), 无从判定\r\n");
        }

        log_info("[TEST] 立刻跑一次失联检查(该发的会真的发到 event topic):\r\n");
        lora_business_check_sensor_offline();
        log_info("[TEST] 检查结束(上面没有 '失联'/'恢复' 行 = 没有跳变)\r\n");
        return true;
    }

    /* ---------------- #lora ---------------- */
    /* 射频诊断(2026-09 现场: 网关自称"在收", 但从机永远不在线, 且 [LoRa RX] 与
     * [proto] !! PARSE FAILED 一条都不出现 -> 说明 RxDone 从未触发)。
     *
     * 【为什么放在 main.c 的扩展钩子里, 而不是加进 g4_at.c】
     *   1) g4_at.c 是 4G/AT 模块, 它的 include 列表里【没有】任何射频头文件, 而且
     *      它还要被主机单测(_host_test 的 test_state)编译 —— 往那里引 radio.h /
     *      LLCC68-board.h 会平白把射频层耦合进 4G 模块, 还要给主机测试造桩。
     *   2) 真正常识上的"组合根"就是本文件: lora_business_init() 在这里调,
     *      g4_at_set_console_ext() 也在这里注册。射频诊断放在这里, 既复用了现成的
     *      扩展钩子(不动 g4_at.c 的命令表), 又能直接调 LoRa 层的接口。
     *   3) 具体实现写在 lora_proto.c 的 LoraApp_DiagPrint(): 那里正是 LoRa 配置
     *      (project_config.h 的 LORA_FRE/SF/BW/CR)与 LoraApp_* 状态所在的一层。
     * 该命令【只读】: 只发 Get 类命令 + 读寄存器, 不发任何 Set/SetRx, 不打断接收。 */
#if MAIN_LORA_ENABLE
    /* ★2026-10-18 修正: 原来这两个分支写【反了】—— MAIN_LORA_ENABLE=1(有射频)时
     *   反而打"不可用: MAIN_LORA_ENABLE=0", 而 =0(没射频)时却去调 LoraApp_DiagPrint()。
     *   现场表现: 串口敲 #lora 永远只回一句"不可用", 射频诊断根本调不到。 */
    if (strcmp(line, "#lora") == 0)
    {
        LoraApp_DiagPrint();
        return true;
    }

    /* ---------------- #lorasend <hex> ----------------
     * 空口【逐字节透传】: 把十六进制串原样从 LoRa 发出去(不加帧头/不算 CRC)。
     *
     * 【为什么需要它 —— 纯 LoRa 台架/拔掉 SIM 卡时】
     *   本固件正常业务只发"协议帧"(AA55 + 网关/设备 UID + func + 数据 + CRC),
     *   没有"随手发一串字节"的入口。而现场排障/让设备进某个状态经常需要
     *   直接对空口喊一嗓子。lora_proto.c 里早就有 lora_proto_send_raw()
     *   (逐字节一致 + 等 TX_DONE + 发完回 RX), 只是一直没人从控制台调它。
     *
     * 用法: #lorasend AA5501...        (空格/逗号随意, 非 hex 字符会被跳过)
     * 上限: LORA_FRAME_RAW_MAX 字节(与协议帧同长, 240)。
     * 注意: 发射期间接收是停的(半双工), 与正常业务共用同一颗射频。 */
    if (strncmp(line, "#lorasend", 9) == 0)
    {
        static uint8_t buf[LORA_FRAME_RAW_MAX];   /* static: 栈只有 4 KB */
        const char *p = line + 9;
        uint8_t     n = 0u;
        int         hi;

        while (*p == ' ') { p++; }

        while ((*p != '\0') && (n < (uint8_t)sizeof(buf)))
        {
            hi = console_hex_digit(*p);
            if (hi < 0) { p++; continue; }        /* 跳过空格/逗号/其它分隔符 */
            p++;
            {
                int lo = console_hex_digit(*p);
                if (lo < 0) { break; }            /* 半字节: 不猜, 直接停 */
                p++;
                buf[n] = (uint8_t)(((uint32_t)hi << 4) | (uint32_t)lo);
                n++;
            }
        }

        if (n == 0u)
        {
            log_info("[TEST] #lorasend 需要 hex 参数, 例: #lorasend AA5501AABB\r\n");
            return true;
        }

        log_info("[TEST] #lorasend 透传 %u 字节(不做任何加工)\r\n", (unsigned)n);
        if (lora_proto_send_raw(buf, n))
        {
            log_info("[TEST] #lorasend 已发出\r\n");
        }
        else
        {
            log_info("[TEST] !! #lorasend 发送失败(射频忙/等 TX_DONE 超时, 详见上面 [TX-RAW] 行)\r\n");
        }
        return true;
    }

    /* ---------------- #bind [sensor|device] [window_s] ----------------
     * 【本地开门绑设备 —— 拔掉 SIM 卡也能绑】
     *
     * 【为什么必须有】绑定走的是"设备主动发 0x04 → 网关回 0x80 → 设备回 0x80"。
     *   但 on_frame 里那道门是:
     *       if (g_bind.state != BIND_IDLE && func == 0x04) { 处理; }
     *   ⇒ 【窗口没开, 0x04 直接当普通帧走白名单被丢掉】。而开窗只有一条路:
     *     云端下发 sen.bind / dev.bind ⇒ 没有 4G 就永远开不了窗 ⇒ 绑不了。
     *   本命令把 open_bind_window() 暴露到控制台, 于是纯 LoRa 台架也能绑。
     *
     * 用法: #bind                     (默认 sensor, 30 秒窗口)
     *       #bind device 60           (声光, 60 秒)
     * ⚠️ serial/name 留空 —— 那是平台侧的元数据, 离线时没有; 绑定后可用 cfg.set 补。
     * ⚠️ 声光默认 dev_type=3(一体机), 需要别的类型请用云端 dev.bind 下发。 */
    if (strncmp(line, "#bind", 5) == 0)
    {
        const char *p = line + 5;
        bool        is_sensor = true;
        unsigned    win_s     = 30u;

        while (*p == ' ') { p++; }

        if (strncmp(p, "device", 6) == 0)      { is_sensor = false; p += 6; }
        else if (strncmp(p, "sensor", 6) == 0) { is_sensor = true;  p += 6; }

        while (*p == ' ') { p++; }
        if ((*p >= '0') && (*p <= '9'))
        {
            win_s = 0u;
            while ((*p >= '0') && (*p <= '9'))
            {
                win_s = (win_s * 10u) + (unsigned)(*p - '0');
                p++;
            }
        }
        if ((win_s == 0u) || (win_s > 300u))
        {
            log_info("[TEST] #bind 窗口给 1~300 秒, 按 30 秒处理\r\n");
            win_s = 30u;
        }

        if (!config_store_can_add(is_sensor))
        {
            log_info("[TEST] !! #bind %s 已达上限(液位≤4 / 声光≤3 / 总≤5), 先解绑再绑\r\n",
                     is_sensor ? "液位传感器" : "声光设备");
            return true;
        }

        if (lora_business_open_bind_window("", "", (uint16_t)win_s, is_sensor, 3u))
        {
            log_info("[TEST] #bind 已开窗 %us type=%s —— 现在让设备发 0x04(绑定请求)\r\n",
                     win_s, is_sensor ? "sensor" : "device");
            log_info("[TEST] #bind 成功后会打 [BIND] ... 绑定成功; 失败/断链看 [BIND] !! 行\r\n");
        }
        else
        {
            log_info("[TEST] !! #bind 开窗失败(已有窗口在进行? 看 [BIND] 日志)\r\n");
        }
        return true;
    }

    /* ---------------- #set <10字符> / #set self <10字符> ----------------
     * 【产线专用: 写出厂字段, 全程本地, 云端不可见】
     *   #set YA26420001      -> 经空口写进设备(0x06, 每 5 秒一条, 最多 3 条)
     *   #set self WANGGUAN00 -> 写网关自己的字段(落 param_save, 不进 config_store)
     * 只校验"长度 == 10", 字符集/首字母【不校验】(用户要求)。
     * 目标设备 = 从机表里唯一那台(产线一次只接一台)。 */
    if (strncmp(line, "#set", 4) == 0)
    {
        const char *p = line + 4;

        while (*p == ' ') { p++; }

        /* #set self <10字符> */
        if ((strncmp(p, "self", 4) == 0) && ((p[4] == ' ') || (p[4] == '\0')))
        {
            device_param_t *dp;
            uint8_t         k;

            p += 4;
            while (*p == ' ') { p++; }

            if (strlen(p) != (size_t)PARAM_FIELD_LEN)
            {
                log_info("[FIELD] !! #set self 需要恰好 %u 字节, 例: #set self WANGGUAN00\r\n",
                         (unsigned)PARAM_FIELD_LEN);
                return true;
            }

            dp = param_get_ptr();
            if (dp == NULL)
            {
                log_info("[FIELD] !! 参数区不可用, 写不了\r\n");
                return true;
            }
            for (k = 0u; k < (uint8_t)PARAM_FIELD_LEN; k++) { dp->field[k] = p[k]; }
            dp->field[PARAM_FIELD_LEN] = '\0';

            if (param_save_store() == 0)
            {
                log_info("[FIELD] ★网关自身字段已落盘 = \"%s\"\r\n", dp->field);
            }
            else
            {
                log_info("[FIELD] !! 字段已改到 RAM, 但落 Flash 失败(重启会丢)\r\n");
            }
            return true;
        }

        /* #set <10字符> -> 写设备 */
        if (strlen(p) != (size_t)LORA_FIELD_LEN)
        {
            log_info("[FIELD] !! #set 需要恰好 %u 字节, 例: #set YA26420001\r\n",
                     (unsigned)LORA_FIELD_LEN);
            return true;
        }
        if (!lora_business_field_write_begin(p))
        {
            log_info("[FIELD] !! 写字段没启动(原因见上一行)\r\n");
        }
        return true;
    }

    /* ---------------- #get / #get self ----------------
     *   #get        -> 发 0x07 读【设备】的字段(0x07 只发一次, 等约 12 秒)
     *   #get self   -> 打印【网关自己】的字段(从 RAM, 不碰空口) */
    if (strncmp(line, "#get", 4) == 0)
    {
        const char *p = line + 4;

        while (*p == ' ') { p++; }

        if (strncmp(p, "self", 4) == 0)
        {
            device_param_t *dp = param_get_ptr();

            log_info("[FIELD] 网关自身字段 = \"%s\"\r\n",
                     (dp != NULL) ? dp->field : "(参数区不可用)");
            return true;
        }

        if (!lora_business_field_read_begin())
        {
            log_info("[FIELD] !! 读字段没启动(原因见上一行)\r\n");
        }
        return true;
    }

    /* ---------------- #unbind <uid> ----------------
     * 走【和云端完全一样】的解绑流程(用户要求: 不要变复杂):
     *   lora_business_unbind() -> 发 0x05 -> 等设备 ack -> 删 config_store + 从机表。
     * 不提供"强制删"的旁路。
     * ⚠️ 离线也能用: 整条流程只走空口, 不需要 4G。
     * ⚠️ 解绑【不碰出厂字段】—— 字段是出厂编号, 解绑后必须还在(用户已确认设备侧同步)。 */
    if (strncmp(line, "#unbind", 7) == 0)
    {
        const char *p = line + 7;
        char        uid[CFG_UID_STR_MAX];
        int         rc;

        while (*p == ' ') { p++; }
        if (*p == '\0')
        {
            log_info("[FIELD] !! #unbind 需要 uid, 例: #unbind 21000000000000000000000001\r\n");
            return true;
        }

        (void)snprintf(uid, sizeof(uid), "%s", p);
        rc = lora_business_unbind(uid);
        if (rc == 0)
        {
            log_info("[FIELD] 解绑流程已发起(等设备 ack, 与云端解绑同一条路)\r\n");
        }
        else
        {
            log_info("[FIELD] !! 解绑没发起 rc=%d (uid 不在从机表 / 已有解绑在进行)\r\n", rc);
        }
        return true;
    }
#else
    if (strcmp(line, "#lora") == 0)
    {
        log_info("[TEST] #lora 不可用: 本固件 MAIN_LORA_ENABLE=0, 没有射频\r\n");
        return true;
    }
#endif

    return false;
}
#endif /* MAIN_AT_CONSOLE_ENABLE */
