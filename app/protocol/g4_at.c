/**
 * \file  g4_at.c
 * \brief ML307R 4G 建链状态机 + MQTT 业务层 *
 * 架构分层:
 *   g4_io.c   - USART2 传输层+ AT 命令收发(隔离串口/USB 细节)
 *   g4_urc.c  - MQTT URC 解析 + 在线状态+ 下行回调(依据手册 V6.7.0)
 *   g4_at.c   - 本文件 建链状态机 + 发布子流程+ 业务封装
 *
 * ==================== 建链流程(仿照现场验证过的 ml307_app.c/ml307_init) ====================
 *
 *   AT_SYNC →ATE0 →MLPMCFG →CPIN →CGSN(IMEI) →CIMI(IMSI) →ICCID →CSQ
 *   →CREG/CEREG(等注册 →CGDCONT(APN, 三家轮换) →CGACT(激活PDP) →CGPADDR(查IP)
 *   →MQTT_DISC →MQTTCONN(clientID=IMEI) →WAIT_CONN →MQTT_SUB →ONLINE
 *
 *   与老流程的差别(都是"照抄现场跑通的那套"):
 *     1) 关深睡眠 AT+MLPMCFG="sleepmode",0,1 ——模组默认开着, 网关会被它 *        时不时睡一个, 表现就是下行偶尔收不到
 *     2) 【不再用 AT+MDIALUP 当主路。 那是 USB/RNDIS 拨号手册的命令【 *        UART AT 模式下走标准 3GPP 流程 ——APN(CGDCONT) + 激活PDP(CGACT)
 *        + 查IP(CGPADDR), ml307_app.c 里就是这么干的 且要拦CGPADDR 的 *        真实 IP 当通不通的判据空IP = PDP 没起来;
 *        MDIALUP 保留不PDP 拿不到IP 时的兜底【 *     3) APN 不写死 按IMSI 的MNC 猜运营商(移动 CMNET / 联通3GNET /
 *        电信 CTNET), 拿不到IP 就换下一家重来——现场还没定用哪家卡【 *     4) IMEI 当MQTT client ID(原来写死 mymodule_test_001): 固定 client ID
 *        会让同型号多台网关互相顶下线; IMEI 天然唯一【 *     5) 顺手读CPIN/CGSN/CIMI/ICCID/CSQ 做体检+日志, 现场排障直接看日志【 *
 *   开局: 上电/复位后先【只听不发】最大G4_BOOT_WAIT_MS(默认 30s)让模组预热—— *         听到 +MATREADY/RDY 就提前开始 听不到才等满。现场实测 预热不够时 *         模组一个字节都不回(累计收到=0), 光靠后面重试是补不回来的。 *
 *   失败恢复(2026-09 按现场要求改): 判定断网/与服务器失去连接/模组不理 AT
 *             → 【直接 4G_RST 断电重启模组】(g4_link_recover), 断电->预热->重新
 *             握手->重新入网->重新连 MQTT; 为避免网络抖一下就断电, 带最小间隔
 *             G4_RST_MIN_GAP_MS 与连续失败次数 G4_REBUILD_RST_CNT。
 */
#include "g4_at.h"
#include "g4_io.h"
#include "g4_urc.h"
#include "bsp_tim.h"
#include "bsp_gpio.h"
#include "local_alarm.h"
#include "data_cache.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "log.h"
#include "main.h"

/* #clk 诊断要用 RCC 寄存器与 RCC_GetClocksFreqValue()。
 * 只在目标构建引入(主机单测的 stub 里没有这些), 免得把 gcc 测试搞挂。 */
#if defined(N32H482)
#include "n32h47x_48x_rcc.h"
#endif

/* ======================== 用户配置化======================== */

/* ---- 供电极性自检 ----
 * ★现场已确认 PA8 低电平上电bsp_gpio.h 的G4_PWR_SW_ON_HIGH=0), 所以这里 *   默认【关掉自检】★【 *
 * 为什么确认之后反而要关掉: 自检的逻辑是听不到就翻到相反极性。极性一旦是对 * 的 反过来就是把模组的电断掉, 而且翻开之后不会再翻回来 ——结果是模组被永久
 * 断电, 比不开自检还糟。只有极性未知时才值得冒这个险【 *
 * 想手工试另一个方向临时再搜一遍 日志口敲 #pwrswap(翻一次 成#cycle(不翻只拔插【 * 真要让它在开机时自己搜 把G4_PWR_SEARCH 改成 1【*/
#ifndef G4_PWR_SEARCH
#define G4_PWR_SEARCH            0
#endif
#ifndef G4_PWR_SEARCH_MS
#define G4_PWR_SEARCH_MS         15000u  /* 每个极性先听这么久 */
#endif
#ifndef G4_PWR_SEARCH_MAX
#define G4_PWR_SEARCH_MAX        1u      /* 最多翻几次(1 = 两个极性各试一遍 */
#endif

/* 【预热窗口——持ml307_app.c 的写注 只等 +MATREADY, 不设"到点就发 AT"。 *
 * 现场那套跑通的 ml307_app.c 里是这么干的:
 *     ML307_PowerOn();
 *     ml307_Expect("+MATREADY", "OK", 5, resp, sizeof(resp));   // 等模组自己报 +MATREADY
 *     ...然后立刻 ATE / CPIN / CGSN ...
 * 也就是【等模组主动读+MATREADY, 而不是我们自己数秒数】。 *
 * 老做法是"等满 30s 就强行去发AT", 实测有两个后果
 *   · 模组起动慢冷启动刚上电供电开时 30s 一到就灌AT, 反而打扰它起动;
 *   · 日志变成"某条命令没应答不支持, 把排查方向带偏实测 AT+CEREG? 的应答 *     居然是+MATREADY ——模组当时在重启【 * 现在: 听到 +MATREADY/RDY 立刻开始握手 没听到就一直等, 只打进度;
 *       等够 G4_BOOT_WAIT_MS 才开始怀疑供电 断电重启一次再等
 *       重启次数用完了才退到AT_SYNC 去硬读那是真没辙了)。*/
#ifndef G4_BOOT_LISTEN_MS
#define G4_BOOT_LISTEN_MS        3000u   /* 上电后只安静听这么久(捡 banner), 然后主动敲 */
#endif
#ifndef G4_AT_PROBE_MS
#define G4_AT_PROBE_MS           500u    /* 主动敲 AT 的节奏 + 单次等应答的上限 */
#endif
#ifndef G4_BOOT_REPORT_MS
#define G4_BOOT_REPORT_MS        5000u   /* 进度行间隔, 免得现场看着像死机 */
#endif
#define G4_RESET_PULSE_MS        200u    /* 电源断电保持时间(从G4_POWER_CYCLE_AT_BOOT=1 用 */

/* ---- 上电时要不要给模组拔一次电 ----
 * 1(默认) = 开机先断电 G4_PWR_OFF_MS 再上电等效拔插一次电源:
 *           这样预热窗口里听到的 +MATREADY 一定是本次启动的 不会跟上一次残留混深
 *           也顺手把"MCU 复位了、模组还停在原来某个异常状态这件事清掉【 * 0 = 直接上电(适合模组由别的电路负责上下电的板子【*/
#ifndef G4_POWER_CYCLE_ON_BOOT
#define G4_POWER_CYCLE_ON_BOOT   1
#endif
#ifndef G4_PWR_OFF_MS
#define G4_PWR_OFF_MS            800u    /* 断电保持时间: 让模组输入电容真正放掉*/
#endif
/* "AT"→OK 最多敲多少次 —— 【要主动出击, 但要有个头】
 * 现场实测: 这块模组(ML307Y)不会稳定地自己吐 +MATREADY, 但一发 AT 就回。
 * 所以不傻等, 而是每隔 G4_AT_PROBE_MS 敲一次, 敲这么多次还不回就认输,
 * 交给断链恢复去断电重启模组(见 g4_link_recover)。
 *   60 × 500ms = 30s。
 * 【原来写的是 240(120s)】: 模组已经不理人了还把 AT 敲两分钟, 现场看着就是
 * "一直在发 AT 停不下来", 而且白白拖慢恢复。模组真死了, 断电重启比继续敲有用。 */
#ifndef G4_AT_SYNC_RETRY_MAX
#define G4_AT_SYNC_RETRY_MAX     60u
#endif

/* ==================== 断链恢复: 直接给模组断电重启(4G_RST) ====================
 *
 * 【现场要求】: 一旦判定"断网了"或者"与服务器失去连接", 就直接通过 4G_RST 给模组
 * 断电重启, 让它从头重新建链 —— 而不是在原地反复敲 AT、反复重连。
 *
 * 触发点(都走 g4_link_recover()):
 *   1) ONLINE 状态下收到 +MQTTURC: "conn",<state> 且 state != 0
 *      (3=服务器拒绝 / 4=服务器断开 / 5=ping超时 / 6=网络异常) —— 这是"与服务器
 *      失去连接"最权威的信号;
 *   2) 连续 G4_PUB_FAIL_MAX 次发布失败(模组假死、不再上报 URC 的典型形态);
 *   3) 敲满 G4_AT_SYNC_RETRY_MAX 次 AT 都不回 —— 模组已经不理人了;
 *   4) ERROR 里连续 G4_REBUILD_RST_CNT 次重建都没能回到 ONLINE。
 *
 * 【为什么要加最小间隔】: 网络"抖一下"也会走到这里。无脑断电会把模组一直按在
 * "断电-上电-重搜网"的循环里, 反而永远起不来(这是这份代码里踩过的坑)。
 * 所以 G4_RST_MIN_GAP_MS 之内已经重启过一次, 这一次就退回"只重新握手(不断电)"。
 */
#ifndef G4_RST_ON_LINK_LOSS
#define G4_RST_ON_LINK_LOSS      1        /* 1 = 断链直接断电重启(现场要求) */
#endif
#ifndef G4_RST_MIN_GAP_MS
#define G4_RST_MIN_GAP_MS        30000u   /* 两次断电重启的最小间隔 */
#endif
#ifndef G4_REBUILD_RST_CNT
#define G4_REBUILD_RST_CNT       2u       /* 连续几次重建失败后断电重启 */
#endif

/* ---- 建链失败 / 断链后要不要给模组断电(4G_RST) ----
 *
 * 【2026-09 按现场要求改过, 这里记一下来龙去脉】
 *
 * 老版本默认【不断电】, 理由是: 4G 入网(搜网→注册→激活 PDP)天生要十几秒到几十秒,
 * 进度全在模组里, 一断电就清零, 于是"一直失败、一直重启", 永远等不到注册上。
 * 那个顾虑本身是对的, 现场也踩过。
 *
 * 但现场现在的要求是: 【判定断网 / 与服务器失去连接, 就直接用 4G_RST 断电重启模组,
 * 重新建立连接】—— 因为模组假死(MQTT 掉了却连一条 URC 都不发)时, 原地重连根本
 * 救不回来, 只能断电。所以策略变成:
 *   · 断链(conn URC 报 3/4/5/6)、连续发布失败、模组不理 AT -> 直接 4G_RST;
 *   · 但加了【最小间隔 G4_RST_MIN_GAP_MS】和【连续失败次数 G4_REBUILD_RST_CNT】,
 *     避免网络抖一下就把模组按在"断电-上电-重搜网"的死循环里 —— 那正是上面那段
 *     顾虑要防的事。
 *
 * 开关和参数见下面 "断链恢复" 那一节(G4_RST_ON_LINK_LOSS 等)。 */

/* ---- AT 超时 ---- */
#define AT_CMD_TIMEOUT_MS        1000u   /* 普通AT 指令应答超时 */

/* 【等 MQTTPUB 的 '>' 提示 / 等数据发完的 OK —— 必须给够】
 * '>' 本该是模组本地立刻回的提示, 但实测 ML307Y 有时要 10 秒才吐出来(网络忙、
 * 刚订完主题时更慢)。原来这里只有 1s/5s, 于是每次心跳发布都超时 → 连续 3 次 →
 * 保活停滞检测判链路死 → 重建 → 【连接每 90 秒被拆一次, 云端下发自然收不到】。
 * 现场日志的铁证: "PUB fail(等 '>' 超时) x1", 而十几秒后又冒出 [URC] raw: "> " ——
 * 模组其实回应了, 只是我们等太短。 */
#ifndef G4_PUB_PROMPT_TIMEOUT_MS
#define G4_PUB_PROMPT_TIMEOUT_MS 15000u
#endif
#define AT_CONN_TIMEOUT_MS       5000u   /* MQTTCONN 指令 OK / 等'>' 超时 */
#define AT_URC_TIMEOUT_MS        60000u  /* 等conn URC(域名解析线10s) */
#define AT_PDP_TIMEOUT_MS        15000u  /* AT+CGACT=1,1 激活PDP(首次入网慢 */
/* 【去激活】快得多(本地操作, 不等网络), 但也不能给 1s —— 有些固件要 2~3s。
 * 超时只是白等一会儿, 不影响流程(本来没激活时也可能没 OK)。 */
#define AT_PDP_DEACT_TIMEOUT_MS  3000u
#define AT_DIALUP_TIMEOUT_MS     30000u  /* AT+MDIALUP=1,1 兜底拨号超时 */
#define G4_FAIL_RESTART_MS       15000u  /* 故障重启间隔 */
#define G4_MQTT_DISC_WAIT_MS     300u    /* 发完 MQTTDISC 等模组消化的时间 */

/* ---- 网络注册(AT+CEREG?/CGREG?/CREG?)的查询节好----
 * 【为什么必须限速【 网络注册要几秒到几十种 而g4_at_poll() 每5ms 就调一次 * 状态机。老代码在 CEREG 状态里"查一次→没注册→立刻再查", 实测 25ms 里连发了
 * 11 板 模组被命令灌满 注册反而更慢。现在每 2s 才查一条 一共给 30s。*/
#define AT_CEREG_POLL_MS         2000u   /* 两次注册查询之间的最小间隔*/
#define AT_CEREG_POLL_MAX        15u     /* 最多查多少次15 × 2s = 30s) */

/* ---- 分组基PDP)参数 ----
 * 【APN 为什么做三家轮换【 现场还没定最终用哪家运营商的单 把CMNET 写死等于
 * 联通电信的卡"注册得上位CGPADDR 拿不到IP"。这里按 IMSI 的MNC 猜运营商,
 * 猜错也不要紧 ——CGPADDR 没拿到IP 就换下一家的 APN 重来(候选表 3 不【*/
#ifndef G4_APN_MOBILE
#define G4_APN_MOBILE            "CMNET"  /* 移动 */
#endif
#ifndef G4_APN_UNICOM
#define G4_APN_UNICOM            "3GNET"  /* 联通个别卡是 UNINET) */
#endif
#ifndef G4_APN_TELECOM
#define G4_APN_TELECOM           "CTNET"  /* 电信 */
#endif
/* 手动指定 APN: 非空则优先于自动判断; 留空 = 交给 IMSI 判断 + 轮换 */
#ifndef G4_APN
#define G4_APN                   ""
#endif
#ifndef G4_APN_AUTO
#define G4_APN_AUTO              1        /* 0 = 不按 IMSI 猜 直接电G4_APN 成CMNET */
#endif
#ifndef G4_PDP_CID
#define G4_PDP_CID               1u      /* PDP 上下文编号1~15 */
/* 拨号(AT+MDIALUP=1,1)失败时重试几次, 还不行就退回老的 CGDCONT/CGACT/CGPADDR 流程 */
#ifndef G4_DIALUP_RETRY_MAX
#define G4_DIALUP_RETRY_MAX      3u
#endif
#endif
#define G4_PDP_ADDR_RETRY_MAX    3u      /* CGPADDR 拿不到IP 时的重试次数(= APN 候选数) */

/* ---- 只验收到"能上网就停 ----
 * 1 = 走完 AT →SIM →注册 →APN/PDP 拿到 IP 就停在那重不连 MQTT)。 *     这一档专门用来分阶段排障: 先确认和模组的沟通+ 入网"这一段是好的,
 *     免得 MQTT 连不上时状态机每15s 把一条已经建好的网络链路拆掉重来,
 *     现场看起来就像4G 一直在重启、什么都没通【 * 0 = 一路走到MQTT 上线(默认, 业务模式)。*/
#ifndef G4_DIAG_STOP_AFTER_PDP
#define G4_DIAG_STOP_AFTER_PDP   0
#endif

/* ==================== broker 选择: 公共(默认) / 私有 EMQX Cloud ====================
 *
 * ★2026-10-05 结论: 私有 EMQX Cloud(mqtts 8883)【当晚没连上】, 先回退到实测能用的公共 broker。★
 *
 * 【事实清单, 留给以后再攻时的起点】
 *   · broker.emqx.io:1883 明文匿名 —— ✅ 实测跑通(13 分钟 1 次会话 13 次 puback)
 *   · w90916ab...emqxsl.cn:8883 + TLS + 账号 —— ❌ 恒 conn state=3(服务器拒绝),
 *     距 MQTTCONN 约 605ms(够跑完 DNS+TCP+TLS+CONNECT, 像"连上了被拒", 不像握手失败)
 *   · 同一个私有 broker 的 1883 明文 —— ❌ 约 9.4s 后 state=1(正在重连) = TCP 没连上
 *     (很可能是 EMQX 部署只把 8883 暴露到公网, 1883 绑在 127.0.0.1)
 *   · 驻网/信号/IP 全程正常(CSEREG stat=1, CSQ 25~30, 拿到 10.x 私网 IP)
 *   · 手册与固件【不一致】: AT+MQTTSTATE? 回 +CME ERROR: 4(操作不支持) ——
 *     这块 ML307Y 是手册的【子集】, "手册上有就照着发"这条路不成立
 *   · 查询形态【不带 '?'】: AT+MQTTCFG="version"? 回 601(无效参数),
 *     而 AT+MQTTCFG="ssl",1 / AT+MQTTCFG="version",4 能正常回显
 *   · 所有 AT+MQTTCFG=... 都回 OK —— 【OK 不代表参数生效】, 必须用查询形态核对
 *
 * 【下次要再试私有 broker / TLS 的入口】
 *   a) 改这里: G4_MQTT_USE_PRIVATE_BROKER 1(或 -DG4_MQTT_USE_PRIVATE_BROKER=1)
 *   b) 或运行期不重编(日志口, 每改一次自动重连):
 *        #at AT+MQTTCFG="ssl",0            ← ★先确认 connect_id 0 到底开没开 TLS★
 *        #at AT+MQTTCFG="ssl",0,1,0        ← 按手册语法开
 *        #tls 1
 *        #broker <host> 8883 <user> <pass>
 *   c) 还没做过、但最该做的一步对照:
 *        #tls 1 ; #broker broker.emqx.io 8883 - -
 *      用【已知支持 TLS 的公共 broker】验证"模组的 TLS 到底能不能用"。
 *      通了 -> 我们 TLS 用法没问题, 问题在 EMQX Cloud 侧(多租户 SNI / 认证 / ACL);
 *      不通 -> 模组 TLS 本身有问题(加密套件/版本), 那就得从 MSSLCFG 那边查。
 */
#ifndef G4_MQTT_USE_PRIVATE_BROKER
#define G4_MQTT_USE_PRIVATE_BROKER  0    /* 0 = 甲方平台(当前目标); 1 = 旧 EMQX Cloud 备份档 */
#endif

#if G4_MQTT_USE_PRIVATE_BROKER
#ifndef G4_MQTT_HOST
#define G4_MQTT_HOST             "w90916ab.ala.cn-shenzhen.emqxsl.cn"
#endif
#ifndef G4_MQTT_PORT
#define G4_MQTT_PORT             "8883"   /* mqtts = TLS, 8883 */
#endif
#ifndef G4_MQTT_USER
#define G4_MQTT_USER             "N32H482_getway"
#endif
#ifndef G4_MQTT_PASS
#define G4_MQTT_PASS             "0809wzy"
#endif
#ifndef G4_MQTT_TLS_ENABLE
#define G4_MQTT_TLS_ENABLE       1
#endif
#else  /* ★ 甲方平台(2026-10-08 起的目标 broker) ★ */
/* 【broker 地址】甲方平台 223.75.207.139。
 *
 * 【端口为什么是 1883 而不是 18083 —— 实测证据, 不要改回去】
 *   甲方给的地址写的是 `223.75.207.139:18083`, 但现场实测(2026-10-08):
 *     · 对 18083 发 MQTT CONNECT  -> 收到 0 字节
 *     · HTTP GET http://223.75.207.139:18083/ -> 200 OK, server: Cowboy,
 *       <title>EMQX Dashboard</title>            <- 【18083 是 EMQX 控制台网页端口】
 *     · 对 1883  发 MQTT CONNECT  -> 收到 20 02 00 00 = CONNACK rc=0(连接被接受)
 *   ⇒ 18083 是 Dashboard(HTTP), 【不是 MQTT】。用 18083 模组必然连不上。
 *   若甲方确实把 MQTT 挪到了别的端口, 只改下面这一行的字符串即可。 */
#ifndef G4_MQTT_HOST
#define G4_MQTT_HOST             "223.75.207.139"
#endif
#ifndef G4_MQTT_PORT
#define G4_MQTT_PORT             "1883"
#endif
/* 【用户名 = "tuguan"(固定)】★2026-10-08 用户决定★
 * 原规则是"= 设备 ID(gwId)"。现在用户名与 topic 的 {gwId} 解耦:
 *   用户名恒为 tuguan(除非配置里显式给了 network.username),
 *   而 topic 里的 {gwId} 仍跟 system.gwId 走(出厂 110001, 可改)。
 * 运行期由 main.c 注入, 见启动段的 g4_at_set_broker()。 */
#ifndef G4_MQTT_USER
#define G4_MQTT_USER             "tuguan"
#endif
/* 【密码 = "tg123456!"】★2026-10-08 用户给定★
 * 配置里给了 network.password 就会被注入覆盖; 留空则用这个默认。
 * 注: 日志里【不打印明文密码】(只打"已设/空"), 免得串口日志外泄凭据。 */
#ifndef G4_MQTT_PASS
#define G4_MQTT_PASS             "tg123456!"
#endif
#ifndef G4_MQTT_TLS_ENABLE
#define G4_MQTT_TLS_ENABLE       0
#endif
#endif

/* ★★2026-10-08 用户决定: clientID / 用户名 / 密码 全部改成固定值★★
 *
 *   · clientID = "110001"  (原来的规则是"用模组 IMEI, 读不到才用这个兜底")
 *   · 用户名   = "tuguan"  (原来的规则是"= 设备 ID(gwId)")
 *   · 密码     = ""        (空)
 *   · topic 里的 {gwId} 仍然【跟 system.gwId 走】(出厂 110001, 可以改) ——
 *     即 clientID/用户名 与 topic 的 gwId 【不再绑定】, 这是用户明确要的。
 *
 * ⚠️⚠️ 多台网关必须警惕: 固定 clientID 会让【同一 broker 上】的网关互相顶下线 ⚠️⚠️
 *   本文件原来这句注释就是要提醒这件事:
 *       "clientID 用IMEI ——固定 client ID 会让多台网关互相顶下线"
 *   MQTT 规范: broker 收到一个新的 CONNECT 用了【已存在】的 clientID, 会把旧连接踢掉。
 *   所以两台网关都用 "110001" 时, 它们会反复互踢: 表现为两台都时通时断、A 上线 B 掉线。
 *   【单台测试没问题】; 一旦上第二台, 请把其中一台的 clientID 改成唯一值 ——
 *   两种做法:
 *     ① cfg.set network.clientId = "<唯一串>"   (main.c 会注入, 见启动段)
 *     ② cfg.set network.clientId = "IMEI"       (特殊值: 用模组 IMEI, 天然唯一 —— 见
 *        mqtt_build_conn_cmd 的说明)
 */
#ifndef G4_MQTT_CLIENT_ID
/* ★2026-10-18: 26 位(= "11" + 24 个 0)。真实值由 network.clientId 覆盖
 * (出厂 = "11" + 芯片 96 位 UID, 见 config_store 的 cfg_default_gw_id)。
 * 这里只作"配置里没给 clientId"时的兜底, 长度必须与 26 位 gwId 一致。 */
#define G4_MQTT_CLIENT_ID        "11000000000000000000000000"
#endif
/* ★2026-10-08: 下面两个"历史遗留 topic"已【整体删除】★
 *   · "mod/mod_upload" —— 旧的 LoRa 0x02 原始帧 hex 上行通道(与 mod/telemetry 重复)。
 *       甲方明确要求下线, 因此连同它的发布路径一起删掉了:
 *       `lora_business_publish_report()` 与 `data_cache_poll()` 都不再存在;
 *       `g4_at_mqtt_publish(NULL, ...)`(靠默认主题发的那条路)现在直接返回失败,
 *       杜绝"换个地方又把它发出去"。
 *   · "mod/pc_down" —— 旧的默认订阅 topic。现在订阅 topic【只能由 main.c 注入】
 *       (按 cloud_profile 算出 rt/gw/{gwId}/cmd), 不再有编译期兜底值。
 *       main.c 没注入时, 建链会明确打错误日志并【跳过订阅】, 不会静默用一个错主题。
 * ⇒ 两个宏(G4_MQTT_PUB_TOPIC / G4_MQTT_SUB_TOPIC)都不再存在, 不要再在别处 #ifndef 它们。 */
/* 下行订阅 topic 的缓冲长度(rt/gw/{gwId}/cmd) */
#ifndef G4_SUB_TOPIC_MAX
#define G4_SUB_TOPIC_MAX         96
#endif
#define G4_MQTT_SUB_QOS          "1"
/* ★上行必须用 QoS 1, 不能是 0 —— 这就是"假消息"的根因★
 *
 * QoS 0 的语义是"发出去就不管了": 模组把 PUBLISH 丢给 TCP 就立刻回 OK/"+MQTTPUB",
 * 【没有任何来自 broker 的确认】。所以只要 TCP 上的数据没能真正到达/被 broker 接受,
 * 我们这边照样显示"发送成功" —— 现场看到的正是"模组说发了, 云端没收到"。
 *
 * QoS 1 时模组会等 broker 的 PUBACK, 并吐出 URC:
 *     +MQTTURC: "puback",<id>,<msgid>
 * 这才是端到端的"云端确实收到了"的证据。
 * 现场那份成功率很高的 ml307_app.c 用的就是 QoS 1, 而且每一步都等 puback:
 *     AT+MQTTPUB=0,"topic",1,1,0,<len>  →  等 '>'  →  发数据  →  ml307_wait("+MQTTURC: \"puback\"")
 * 参数顺序 <id>,<topic>,<qos>,<retain>,<dup>,<msg_len> 用 "#at AT+MQTTPUB=?" 可以核对。 */
#define G4_MQTT_PUB_QOS          "1"

#define G4_MQTT_CONNECT_ID       "0"     /* ML307R 支持 0~5 跳*/

/* ==================== 模组【自己】的 MQTT 参数(2026-10-04 第 2 步) ====================
 *
 * 【为什么要做这一步】复盘 2026-10-04 现场日志得出的结论是: 我们一直在应用层
 * 手搓模组【本来就有】的能力, 而且搓得更差 ——
 *   · 模组自带 MQTT 自动重连(AT+MQTTCFG="reconn"), 我们从没下发过, 靠"连续失败就
 *     给整机断电 40~60 秒"来重连;
 *   · 模组自带心跳/保活(keepalive + pingreq + pingresp 回显), 从没下发过,
 *     network.keepAliveSeconds 是死配置;
 *   · 遗嘱(willoption/willpayload)只在"MQTT 快修"那条路上发过, 冷启动不发,
 *     而手册明确"模组掉电后配置参数不保存" -> 每次断电重启后遗嘱就没了。
 *
 * 手册依据: 《12_MQTT用户手册_V6.7.0》§3.1。参数语义(逐条抄自手册):
 *   +MQTTCFG="keepalive",<id>,<keepalive_time>
 *       保活时间。范围 0 或 60~65535; 默认 120; 单位 s。
 *       "在 1.5 倍的设置时间内, 若服务器未从客户端收到消息, 则默认客户端发送了
 *        DISCONNECT 消息, 因此服务器会断开客户端连接。" -> 0 = 不断开。
 *       ★所以网络侧的最小值是 60s, 配置里给 30~59 的必须夹到 60★
 *   +MQTTCFG="pingreq",<id>,<ping_interval>
 *       MQTT 心跳间隔(手册原文"配置MQTT心跳间隔"), 60~86400。
 *   +MQTTCFG="pingresp",<id>,<pingack>
 *       配置"MQTT心跳请求回显"。开 = 模组把心跳结果用
 *       +MQTTURC: "pingresp",<id>,<ping_ret> 报上来(<ping_ret> 0=成功/1=超时)。
 *       ★这是【模组级的保活探针】, 比我们自己的发布计数更贴近协议层★
 *   +MQTTCFG="reconn",<id>,<reconn_times>,<reconn_interval>,<mode>
 *       重连次数 0~3(默认3) / 重连间隔 20~60s(默认20) / 策略 0=固定间隔, 1=n倍间隔
 *       (例: ivl=20, times=3 -> 20-40-60s)。★这就是"模组自己会重连"★
 *
 * 【全部只在"MQTT 连接未创建"时下发】—— 手册对 keepalive/pingreq/reconn 都写了
 * 这个前提, 所以它们在 G4_STATE_MQTT_CFG 里发, 位置在 MQTTDISC 之后、MQTTCONN
 * 之前。每一次建链(含快修/断电重启之后)都会重走一遍: 模组掉电不保存这些设置。
 *
 * 【失败绝不影响建链】: 这只是让模组干得更专业一点。哪一条没被接受就打日志跳过,
 * 与加这一步之前的行为完全一致 —— 绝不能因为它连不上 broker。
 */
#ifndef G4_MQTT_KEEPALIVE_MIN_S
#define G4_MQTT_KEEPALIVE_MIN_S   60u    /* 手册下限: keepalive 要么 0, 要么 ≥60 */
#endif
#ifndef G4_MQTT_PINGREQ_S
#define G4_MQTT_PINGREQ_S         60u    /* MQTT心跳间隔(手册 60~86400); 取下限=最快发现 */
#endif
#ifndef G4_MQTT_RECONN_TIMES
#define G4_MQTT_RECONN_TIMES      3u     /* 重连次数上限(手册 0~3) */
#endif
#ifndef G4_MQTT_RECONN_IVL_S
#define G4_MQTT_RECONN_IVL_S      20u    /* 重连间隔(手册 20~60) */
#endif
#ifndef G4_MQTT_RECONN_MODE
#define G4_MQTT_RECONN_MODE       1u     /* 1 = n倍间隔 -> 20/40/60s 共 120s 自愈窗口 */
#endif
/* ★MQTT 消息发布格式(input_format)★ —— 2026-10-04 现场日志逼出来的一个真 bug:
 *
 * 现场云平台收到的遗嘱是(原文):
 *     SyntaxError: Expected property name or '}' in JSON at position 1 (line 1 column 2)
 *     {\"online":false}
 * 报错在【第 2 个字符】= 反斜杠 —— 也就是说【反斜杠真的发到 broker 了】, 遗嘱不是
 * 合法 JSON, 平台的在线判定拿到的是一段坏数据。
 *
 * 根因在手册里(12_MQTT用户手册 §AT+MQTTCFG 参数描述):
 *     <input_format> 整型，MQTT 消息发布格式… 对 MQTTPUB 的 <message> 参数和
 *                    MQTTCFG 的 <will_msg> 参数生效。
 *         0 = ASCII码字符串(默认)  1 = 十六进制字符串  2 = 转义字符串
 * 我们没设过它, 用的是默认 0 = ASCII —— 模组把 `\"` 当成两个普通字符(`\` 和 `"`)
 * 原样存下来, 所以遗嘱正文里带了反斜杠。
 * 改成 2(转义字符串) 之后 `\"` 才会被当成一个引号, 与我们的拼串方式一致。
 *
 * ⚠️ 这一条【必须在上位机台架确认】(发一条遗嘱 → 断电 → 看平台收到的是不是
 *    合法的 {"online":false})。若 2 不行, 备选是 1(十六进制): 把正文按 hex 写,
 *    完全绕开转义规则 —— 那就把 G4_LWT_PAYLOAD_HEX 用起来(见 g4_at_set_lwt)。
 * 它也顺带影响 AT+MQTTPUB 的 <message> 内联参数(我们目前走数据模式, 不用内联)。
 */
#ifndef G4_MQTT_ENC_INPUT
#define G4_MQTT_ENC_INPUT         2u     /* 0=ASCII(默认) 1=Hex 2=转义 */
#endif
#ifndef G4_MQTT_ENC_OUTPUT
#define G4_MQTT_ENC_OUTPUT        0u     /* 0=显示原始字符串 1=显示十六进制 */
#endif
/* ★clean_session★ —— 手册: <clean_session> 默认值 0
 *     0 = 服务端必须根据当前的会话状态【恢复】与客户端的通信
 *     1 = 服务端必须【清除】之前的会话, 启动一个新会话
 *
 * ★现场为什么必须改成 1★
 *   本工程的会话模型是"无状态 + 重连即全量重同步"(文档 9.4: 重连后先 status.report
 *   全量状态, 再 event.buffered 补报)。可 default 0 是【持久会话】: 于是每次重连
 *   broker 都会把上一次的会话状态(包括没被确认的 QoS1 报文与它的 packet-id)接回来。
 *   而现场恰恰是"同一个 clientId 反复重连"(2026-10-04 日志: 25 分钟 6 次), 两边都会
 *   被这份陈旧状态搅乱 —— 现象就是"云平台收到了发布, 但 PUBACK 不回到模组"。
 *   改成 1 之后每次连接都是干净会话, 与文档的"重连即全量重报"完全一致。
 *   ⚠️ 这条是【假设】, 要靠对照实验确认(见 4G断网定位_对照实验单.md)。
 */
#ifndef G4_MQTT_CLEAN_SESSION
#define G4_MQTT_CLEAN_SESSION     1u
#endif
/* 给"模组自己重连"的观察窗口。按上面的参数 = 20+40+60 = 120s, 留一点余量。 */
#ifndef G4_MQTT_RECONN_GRACE_MS
#define G4_MQTT_RECONN_GRACE_MS   130000u
#endif
/* G4_STATE_MQTT_CFG 里要下发几条(与 mqtt_cfg_build 的 step 对齐) */
#define G4_MQTT_CFG_STEPS         14u

/* MQTT 协议版本(手册: 默认值 4 = MQTT 3.1.1)。
 *
 * ★2026-10-05: 默认改成 0 = 【不下发】★
 *
 * 【为什么撤掉】现场实测出现了两个"手册与固件不一致"的硬证据:
 *   ① `#at AT+MQTTSTATE?` -> +CME ERROR: 4 (操作不支持) —— 手册明明有这条, 固件没有,
 *      说明这块 ML307Y 固件是手册的【子集】;
 *   ② `#at AT+MQTTCFG="version"?` -> +CME ERROR: 601 —— 【查询形态不支持】,
 *      于是我们【无法验证】version 到底被设成了多少。
 * 而网上 ML307A 连 EMQX 的实测教程用的语法是 `AT+MQTTCFG="version",4`
 * (【没有 connect_id】), 与手册的 `"version",<connect_id>[,<version>]` 不一致。
 * 两者都能回 OK —— 也就是说, 我们发的 `"version",0,4` 有可能被解析成
 * 【version = 0】(一个无效的协议版本), 那会让 broker 直接以
 * "unacceptable protocol version" 拒掉 CONNECT —— 正好也是 conn state=3。
 *
 * 结论: 这一条【无法验证、且有反向风险】, 不能留。撤掉它回到已知基线;
 * 台架上要试可以用运行期命令(不重编):
 *     #at AT+MQTTCFG="version",4      ← 教程语法
 *     #at AT+MQTTCFG="version",0,4    ← 手册语法
 * 然后看下一条 conn state。
 *
 * 0 = 不下发, 用模组默认(手册称默认 4);改成 4 就按手册语法下发。 */
#ifndef G4_MQTT_PROTO_VER
#define G4_MQTT_PROTO_VER         0u
#endif

/* ==================== TLS(mqtts) —— 2026-10-04 第 3 步 ====================
 *
 * 用户要求接自己的 EMQX Cloud: mqtts://w90916ab.ala.cn-shenzhen.emqxsl.cn:8883
 * (账号 N32H482_getway)。这是对 不做与偏差记录_v107 第一条"不做 TLS"的正式反转。
 *
 * 手册依据(两份):
 *   《12_MQTT用户手册 V6.7.0》§AT+MQTTCFG:
 *       AT+MQTTCFG="ssl",<connect_id>[,<ssl_enable>[,<ssl_id>]]
 *           <ssl_enable> 0=使用普通TCP连接(默认)  1=使用SSL TCP连接
 *           <ssl_id>     SSL 上下文索引(见 SSL 手册, 0~5)
 *       ★ 与 keepalive/will 一样, 必须在"MQTT 连接未创建"时下发。
 *   《ML307_SSL用户手册 V3.0.0》§3.1 AT+MSSLCFG:
 *       AT+MSSLCFG="auth",<ssl_id>,<cert_verify>
 *           <cert_verify> 0=无身份认证(默认) 1=单向认证 2=双向认证
 *       AT+MSSLCFG="cert",<ssl_id>[,<srv_cert>,<cli_cert>,<prv_key>]
 *           <srv_cert> 受信任 CA 证书名(≤64B) —— 单向认证时用它校验服务器证书
 *       AT+MSSLCFG="ignorestamp",<ssl_id>,<ignore_stamp>
 *           <ignore_stamp> 0=不忽略  1=忽略(默认)
 *       AT+MSSLCFG="ignoreverify",<ssl_id>,<ignore_verify>
 *           <ignore_verify> 0=不忽略认证结果(默认) 1=忽略
 *       AT+MSSLCFG="version",<ssl_id>,<ssl_version>
 *           0=SSL3.0 1=TLS1.0 2=TLS1.1 3=TLS1.2 255=全部(默认)
 *       手册原文: "需在建立 SSL 连接前进行证书输入和相应模式配置, 否则将无法生效"
 *       -> 所以这些必须在 MQTTCONN 之前下发(本工程放在 G4_STATE_MQTT_CFG 末尾)。
 *
 * ★关于证书时间戳(用户 2026-10-04 的说明: 模组联网后能算出时间)★
 *   模组注册之后 AT+CCLK? 确实能给出网络时间(现场日志里就报出了真实日期),
 *   所以理论上可以打开时间戳校验。但模组默认就是【忽略】(ignorestamp=1), 而
 *   忽略时间戳的代价只是"证书过期发现不了", 换来的是"绝不会因为时钟差异而握手
 *   失败"。对一台没有 RTC 电池、上电即 1970 的网关, 先把通道打通更重要 ——
 *   所以这里【显式写成 1】, 并把它做成开关: 想启用严格校验就把它改成 0,
 *   同时确认 MQTTCONN 之前模组时钟已经是准的。
 *
 * ★实施顺序(用户已同意: 先把通道打通, 有问题再慢慢修)★
 *   第 1 步(当前): G4_TLS_AUTH_MODE=0 —— 加密但不校验证书。不需要任何证书文件,
 *                  能立刻验证"TLS 通道通不通、handshake 要多久"。
 *   第 2 步: 拿 CA 证书 -> AT+MSSLCERTWR 写进模组 -> G4_TLS_AUTH_MODE=1
 *            + G4_TLS_CA_CERT "xxx.cer" -> 变成真正的单向认证。
 *            先敲 #at AT+MSSLLIST=1 看模组【出厂内置了哪些 CA】, 也许不用自己导。
 */
/* TLS 开关(G4_MQTT_TLS_ENABLE)与 broker 参数一起在文件上方按
 * G4_MQTT_USE_PRIVATE_BROKER 二选一定义 —— 不在这里重复定义。 */
#ifndef G4_TLS_SSL_ID
#define G4_TLS_SSL_ID             0u       /* SSL 上下文索引, 0~5 */
#endif
#ifndef G4_TLS_AUTH_MODE
#define G4_TLS_AUTH_MODE          0u       /* 0=不校验证书(第1步) 1=单向 2=双向 */
#endif
#ifndef G4_TLS_CA_CERT
#define G4_TLS_CA_CERT            ""       /* 单向认证要绑定的 CA 证书名(≤64B) */
#endif
#ifndef G4_TLS_IGNORE_STAMP
#define G4_TLS_IGNORE_STAMP       1u       /* 忽略证书时间戳(模组默认也是 1) */
#endif
#ifndef G4_TLS_IGNORE_VERIFY
#define G4_TLS_IGNORE_VERIFY      0u       /* 0=不忽略认证结果(默认) */
#endif

/* ★要不要预先配置 SSL 上下文(AT+MSSLCFG)? —— 2026-10-05 用户判断: 不要★
 *
 * 【为什么默认 0(不配)】用户 2026-10-05 指出: `AT+MQTTCFG="ssl",<enable>,<ssl_id>`
 * 本身就带 ssl_id, 不需要我们再拿 AT+MSSLCFG 去"配"那个上下文。翻手册也确实如此 ——
 * 《ML307_SSL用户手册》给出的 SSL 上下文【出厂默认】正好就是我们要的那一套:
 *     auth       默认 0 = 无身份认证(不校验证书)
 *     ignorestamp 默认 1 = 忽略证书时间戳
 *     ignoreverify 默认 0 = 不忽略认证结果
 * 也就是说: 我们原来那三条 AT+MSSLCFG 是【把默认值又写了一遍】, 纯属多余; 而多写一遍
 * 有可能覆盖掉模组出厂预置在这个上下文里的东西(例如内置 CA / 会话恢复状态), 反而
 * 把本来能通的 TLS 弄坏 —— 现场现象就是 AT+MQTTCONN 回 OK 但紧跟着 conn state=3
 * (服务器拒绝连接)。
 *
 * 【什么时候要改成 1】只有到"第 2 步: 单向认证"时才需要 —— 那时要显式下发
 *     AT+MSSLCFG="auth",0,1        (开启证书校验)
 *     AT+MSSLCFG="cert",0,"ca.cer" (绑定 CA 证书)
 * 所以这个开关就是"要不要碰 SSL 上下文"的总闸。 */
#ifndef G4_TLS_USE_MSSLCFG
#define G4_TLS_USE_MSSLCFG        0        /* 0 = 只用 MQTTCFG="ssl", 不碰 MSSLCFG */
#endif

/* step 编号(与 mqtt_cfg_build 的 switch 一一对应) */
#define G4_CFG_STEP_WILLOPT       0u
#define G4_CFG_STEP_ENCODING      1u
#define G4_CFG_STEP_WILLPAY       2u
#define G4_CFG_STEP_CLEAN         3u
#define G4_CFG_STEP_KEEPALIVE     4u
#define G4_CFG_STEP_PINGREQ       5u
#define G4_CFG_STEP_PINGRESP      6u
#define G4_CFG_STEP_RECONN        7u
#define G4_CFG_STEP_TLS_AUTH      8u
#define G4_CFG_STEP_TLS_STAMP     9u
#define G4_CFG_STEP_TLS_VERIFY    10u
#define G4_CFG_STEP_TLS_CERT      11u
#define G4_CFG_STEP_TLS_SSL       12u
#define G4_CFG_STEP_VERSION       13u

/* ============================================================ */

/* 模组状态机 */
typedef enum
{
    G4_STATE_RESET        = 0,  /* 电源复位模组(断电再上电 */
    G4_STATE_WAIT_MODULE  = 1,  /* 等模组起动仅电源复位后需要 */
    G4_STATE_AT_SYNC      = 2,  /* AT 握手 */
    G4_STATE_ATE0         = 3,  /* 关回是*/
    G4_STATE_MLPMCFG      = 4,  /* 关深睡眠 */
    G4_STATE_CPIN         = 5,  /* SIM 就绪 */
    G4_STATE_CGSN         = 6,  /* 读IMEI */
    G4_STATE_CIMI         = 7,  /* 读IMSI */
    G4_STATE_ICCID        = 8,  /* 读ICCID */
    G4_STATE_CSQ          = 9,  /* 读信号强应*/
    G4_STATE_CREG         = 10, /* 4G 网络注册 */
    G4_STATE_CGDCONT      = 11, /* 设APN */
    G4_STATE_CGACT        = 12, /* 激活PDP */
    G4_STATE_CGPADDR      = 13, /* 查PDP 分配的IP */
    G4_STATE_DIALUP       = 14, /* 兜底: AT+MDIALUP=1,1 拨号 */
    G4_STATE_MQTT_DISC    = 15, /* 清理残留连接 */
    G4_STATE_MQTT_CONN    = 16, /* 只MQTTCONN */
    G4_STATE_WAIT_CONN    = 17, /* 等+MQTTURC: "conn" */
    G4_STATE_MQTT_SUB     = 18, /* 订阅主题 */
    G4_STATE_ONLINE       = 19, /* 在线,处理发布/URC */
    G4_STATE_ERROR        = 20, /* 故障等待重启 */
    G4_STATE_DIAG_DONE    = 21, /* 自检档(G4_DIAG_STOP_AFTER_PDP=1)的终点: 已入网拿 IP */
    G4_STATE_REG_CHECK    = 22, /* 拨号成功之后"查一下 AT+CEREG?"做诊断(不当闸门) */
    G4_STATE_REG_ARM      = 23, /* 上线前给模组打开"驻网/PS 变化主动上报"(只观测) */
    G4_STATE_MQTT_CFG     = 24  /* 建 MQTT 之前下发【模组自己的】MQTT 参数(遗嘱/保活/重连) */
} g4_state_t;

/* 拿到 IP 之后往哪走: 自检档停在DIAG_DONE, 业务档继续连 MQTT */
#if G4_DIAG_STOP_AFTER_PDP
#define G4_STATE_AFTER_IP     G4_STATE_DIAG_DONE
#else
#define G4_STATE_AFTER_IP     G4_STATE_MQTT_DISC
#endif

/* 发布子状态 */
typedef enum
{
    PUB_IDLE = 0,
    PUB_CMD_SENT,       /* AT+MQTTPUB 已发, 等 '>' */
    PUB_PROBE,          /* '>' 丢了但收到了 OK: 先敲一条 AT 探活, 活着就重发本条 */
    PUB_DATA_SENT,      /* payload 已发, 等 OK */
    PUB_WAIT_ACK        /* 模组已收(OK), 再给 broker 的 puback 一点时间 */
} pub_state_t;

static g4_state_t   g_state;
static uint32_t     g_state_tick;
static uint16_t     g_retry_cnt;

/* 发 AT+MQTTCONN 那一刻的 conn URC 序号(定义见 g4_urc.c)。
 * WAIT_CONN 用它保证"等到的是【本次】的 conn 结果", 而不是上一轮残留的在线标志。
 * 少了这个判断, WAIT_CONN 一进来就会瞬间放行并打出
 *     [G4] MQTT conn ok (res=255 unknown)      ← 255 = 本次一条 conn URC 都还没收到
 * 然后 MQTTSUB 连吃 4 个 +CME ERROR: 607 再 rebuild —— 现场日志 78.txt 里两次。 */
static uint32_t     s_conn_seq_base;

/* 建链过程中采集到的模组信息IMEI/IMSI/ICCID/CSQ/注册状态IP) */
static g4_module_info_t g_info;

/* 注册查询命令轮换下标: 0=CEREG(LTE) 1=CGREG 2=CREG */
static uint8_t      g_reg_probe;
/* 连续建链失败次数: 到 G4_REBUILD_RST_CNT 就转 4G_RST 断电重启(见 g4_link_recover) */
static uint16_t     g_fail_cnt;
/* APN 候选表(0=首通 之后依次换另外两它 与当前下样*/
static const char  *g_apn_list[3] = { G4_APN_MOBILE, G4_APN_UNICOM, G4_APN_TELECOM };
static uint8_t      g_apn_idx;
/* PDP 拉IP 的重试计数不是否已经试过拨号兜底 */
static uint8_t      g_pdp_retry;
static bool         g_dialup_tried;

/* ---------------- 供电极性自检的状态----------------
 * g_pwr_ok     : 已经听到过模组说话→当前 PA8 极性是对的, 以后不再翻 * g_pwr_flips  : 已经翻了几次极性上限 G4_PWR_SEARCH_MAX)
 * g_boot_wait_ms: 预热窗口长度, 默认宏候 日志口"#listen N" 可以现场收 */
static bool         g_pwr_ok;
static uint8_t      g_pwr_flips;
static uint32_t     g_boot_wait_ms = G4_BOOT_LISTEN_MS;

/* ---------------- 非阻塞状态机的两个"进度"变量 ----------------
 * g_step      : 当前状态内的子步骤。有些状态要连着发好几条 AT(如 ATE0 → ATI →
 *               CGMR), 或者要等一段固定时间(如"注册被拒后 CFUN 等 10s"),
 *               全靠它记住走到哪一步。g4_goto_state() 会把它清零。
 * g_wait_tick : 子步骤的起始时刻, 用来代替老的 delay_ms()。
 *
 * 【为什么要把 delay_ms 全干掉】: delay_ms 期间没人收 USART2。115200 下
 * 4096 字节的环形缓冲只要 356ms 就被灌满, 一次 delay_ms(10000) 足以把云端
 * 下行整段丢掉 —— 现场"一下发就出事"就有它一份。
 */
static uint8_t      g_step;
static uint32_t     g_wait_tick;
/* 运行期拼出来的 AT 命令(AT+CGDCONT / AT+CGACT / AT+CGPADDR / AT+MQTTCONN...)。
 * 【必须用静态缓冲】: 非阻塞命令引擎会在多个主循环拍里持续用这条命令, 放在
 * case 的局部数组里, 下一拍就是未初始化的栈 —— 会发出乱码命令。 */
static char         g_tx_cmd[192];

/* ---- CFUN(射频功能档位) ----
 * 建链时读一次 AT+CFUN? 记在这里, 并打进状态行。
 *   1 = 全功能(正常)   0 = 最小功能(射频关, 不会搜网!)   -1 = 没读到
 * g_cfun_need_on: 读到 0 时置位, 下一步补发 AT+CFUN=1 打开射频。 */
static int8_t       g_cfun = -1;
static bool         g_cfun_need_on;
/* 进入 WAIT_MODULE 那一刻的"就绪计数", 用来判"模组开口了没有" */
static uint32_t     g_ready_base;
/* 进入 ONLINE 那一刻的"就绪计数" —— 用来盯"在线期间模组有没有偷偷重启过"。
 * 现场证据(日志 78.txt): 每次 +MATREADY 之后 MQTT 会话就死了, 而模组【不发
 * conn URC】告诉我们 —— 只能靠这个计数变化自己发现。 */
static uint32_t     g_ready_base_online;

/* ---- 驻网(CEREG)只读观测的状态 ----
 * 【为什么声明放在这里, 而函数体在下面】: g4_goto_state()(进 ONLINE 时)要复位
 * 这几个量, 它在文件靠前的位置 —— 声明必须早于它。函数体统一放在"保活看门狗"
 * 那一节, 那里离调用点(g4_at_poll)近, 也方便和 HB 判据放在一起看。
 * 详细设计见 g4_at.h 的 g4_cereg_diag_t 与下面的 g4_cereg_diag_poll()。 */
static bool             g_cereg_inflight;   /* 轮询在途(只在台架轮询模式下会为 true) */
static uint32_t         g_cereg_next_tick;  /* 下次允许轮询的时刻 */
static g4_cereg_diag_t  g_cereg;            /* 对外的观测数据 */

/* ---- 模组自己的 MQTT 参数 与 恢复策略(2026-10-04 第 2 步) ----
 * g_mqtt_keepalive_s : 由 main.c 从 network.keepAliveSeconds 注入(0 或 ≥60)
 * g_reconn_min/max_s : 由 main.c 从 network.reconnectMin/MaxSeconds 注入
 *                      —— 这两个字段以前是【死配置】, 现在真的喂给退避算法了
 * g_backoff_ms       : 当前退避窗口(0 = 还没退避过); 每次恢复动作后翻倍, 封顶 max
 * g_recover_hold    : 退避窗口结束时刻; 窗口内再要求"整链路恢复"就降级成"只重新握手"
 * g_rst_count        : 累计断电重启次数(状态行/诊断用)
 * g_mqtt_reconn_since: !=0 = 模组正在自己重连(conn state=1) 的起始时刻
 */
static uint16_t         g_mqtt_keepalive_s;             /* 0 = 不下发, 用模组默认 */
static uint16_t         g_reconn_min_s = 5u;
static uint16_t         g_reconn_max_s = 300u;
/* TLS(mqtts)开关。默认跟编译期宏; main.c 在配置里给了 broker 时会覆盖它。
 * ⚠️ 与 keepalive/reconnect 同一条约定: g4_at_init() 【不许】复位它 —— 否则
 *    "先注入再 init" 会静默丢掉配置(第 2 步就踩过这个坑)。 */
static bool             g_tls_enable = (G4_MQTT_TLS_ENABLE != 0);
static uint32_t         g_backoff_ms;
static uint32_t         g_recover_hold_tick;
static uint32_t         g_rst_count;
static uint32_t         g_mqtt_reconn_since;
static uint32_t         g_cgev_detach_seen;             /* 已处理过的 PS 去附着条数 */

/* ---------------- 发布队列 ----------------
 * ★原来是"单条": 上一条还没发完, 新的 g4_at_mqtt_publish() 直接返回 SYS_ERR。
 * 现场现象就是: 云端连着发几条, 回执只有第一条发得出去, 后面全被丢掉 ——
 *      [4G] !! 回执登记失败(发布忙/链路不在线), 这条回执丢了   ← 连着一堆
 * 这就是"我收到了、也回执了, 但云端收不到回应"的直接原因(不是模组的问题)。
 *
 * 现在改成一个小环形队列: 发布中时把新报文【排队】而不是丢掉。
 * 深度不用大 —— 一条发布事务几百毫秒就完了, 4 条足够吸收突发, 也不吃 RAM。
 */
#ifndef G4_PUB_QUEUE_DEPTH
#define G4_PUB_QUEUE_DEPTH       4u
#endif
#define G4_PUB_TOPIC_MAX         48u
/* 【2026-09-27 256 -> 4352: 原值会让 cfg.query 永远发不出去】
 * 实测(现场台架): 注入 cfg.query 得到 1279 字节应答, 但这里 >256 被直接拒绝,
 * 串口只留下"应答发布登记失败(离线/队列满), 这条应答丢了" —— 看起来像网络问题,
 * 实际是【本地缓冲上限】, 在结构上就发不出去。
 * cfg.query 的应答 = 信封(~120B) + 完整配置(实测默认 1203B / 满配最坏 3584B),
 * 所以至少要能装下 main.c 里那个 resp 缓冲的大小(CFG_JSON_MAX + 256 = 4352)。
 * ⚠ 代价: 本缓冲被"正在发"槽位和 G4_PUB_QUEUE_DEPTH 个队列项各占一份
 *   (g_pub_data + g_pub_q[4]), 静态 RAM 约 +16KB。当前整机 RAM 用量宽裕, 可接受。
 * ⚠ 注意: 4352 是【本地】上限; ML307Y 的 AT+MQTTUB 走 prompt(>) + 原始字节,
 *   理论能收长报文, 但【模块侧是否真的接受 1.3KB+ 尚未实测】—— 改完必须上板复验。 */
/* ★★2026-10-08: 由 4352 下调到 1024 —— 大包会把模组撑重启★★
 *
 * 【现场证据(2026-10-08 用户日志)】
 *   一条 1564 字节的按应答(`cfg.query sections:["all"]`)发出去后:
 *       [G4_TX] AT+MQTTPUB=0,"rt/gw/110001/cmdresp",1,0,0,1564
 *       [G4_RX] +MQTTPUB: 0,23623,1591     ← ★没有 '>' 提示★
 *       [G4_RX] OK                          ← 模组说"收下了"
 *       然后 puback 【永远没来】(STAT2 里 puback 一直停在 34)
 *       紧接着 [G4] ** +MATREADY           ← 模组 AT 应用层重启了
 *   ⇒ 模组把 payload 收进了 AT 缓冲, 但【装不下】, 于是没真正发出去,
 *     而且 AT 应用层被撑重启。同一时段 `'>'丢失重发=13` 与 `MQTT快修=13`
 *     几乎 1:1 —— 每次重发都在重新触发一次模组重启, 我们的重发逻辑在放大它。
 *
 * 【为什么定 1024】实测 293 字节正常、1564 字节必挂, 阈值在两者之间。
 *   先取 1024 保守兜底, 等 bisect 出准确值再调大(改这一个宏即可)。
 *   宁可让平台收到一条明确的 1007, 也不要让整条链路断 20 秒。
 *
 * 【超限会怎样】见 g4_at_mqtt_publish_ex(): 明确拒绝 + 打日志, 【绝不硬发】。 */
/* ==========================================================================
 * ★★2026-10-09 定案: 这个值【不是】模组的发布上限, 是【本地兜底】★★
 * --------------------------------------------------------------------------
 * 【曾经误判过一次, 记在这里免得后人重犯】
 *   2026-10-08 现场看到"1564 字节发布 -> 模组回 OK -> puback 不来 -> +MATREADY",
 *   又查到手册写 "ML307R 长度最大为 1024", 于是把这个值从 4352 降到 1024,
 *   并让超限走"错误码 1011"。当时的推理是"大包撑爆模组 AT 缓冲"。
 *
 *   ★2026-10-09 用户接了【外接电源】后实测: 1558 字节单条发布【完全正常】
 *   (AT+MQTTPUB -> OK -> puback 正常回来)。
 *   ⇒ 真因是【发射瞬间电源跌落】导致模组/整机复位, 与包长无关;
 *     手册那条 1024 指的是 AT 指令行长, 不是 MQTT payload 长度。
 *   ⇒ 所以值改回原设计, "大包"不再被无谓拒绝。
 *
 * 【那这个宏还有什么用】
 *   纯本地兜底: 满配配置(~4.3KB)加信封后可能超过它 -> 那时明确拒绝并回 1011,
 *   让平台按域查询。宁可拒绝一条, 也不去撞 RAM 边界。
 *   ⚠️ 真正的大包能不能发, 取决于【供电】—— 部署时必须保证电源带载能力,
 *      否则换了多大的值都会复位。
 * ======================================================================== */
#define G4_PUB_DATA_MAX          4352u

typedef struct
{
    char     topic[G4_PUB_TOPIC_MAX];
    uint8_t  data[G4_PUB_DATA_MAX];
    uint16_t len;
    uint8_t  qos;                        /* 逐条 QoS(文档: telemetry=0, 其余=1) */
    uint8_t  retain;                     /* 逐条 retained(文档: 只有 status) */
    bool     used;
} g4_pub_item_t;

static pub_state_t  g_pub_state;
static char         g_pub_topic[G4_PUB_TOPIC_MAX];   /* 正在发的这一条 */
static uint8_t      g_pub_data[G4_PUB_DATA_MAX];
static uint16_t     g_pub_len;
static bool         g_pub_pending;
/* 正在发的这一条的 QoS / retained。
 * 以前这两个值是写死的宏(G4_MQTT_PUB_QOS="1", retain 恒 0), 现在改为逐条携带 ——
 * 文档要求 telemetry 用 QoS0, 其余 QoS1, 且只有 status 需要 retained。 */
static uint8_t      g_pub_qos    = 1u;
static uint8_t      g_pub_retain = 0u;

/* 下行订阅 topic。
 * ★2026-10-08: 不再有编译期兜底值(mod/pc_down 已删除)★
 * 初始为空, 【必须】由 main.c 按 cloud_profile 注入 rt/gw/{gwId}/cmd。
 * g4_at 保持"只懂传输", 不依赖 cloud_profile —— 靠调用方注入。
 * 没注入时: 建链会打 !!(下行订阅 topic 为空) 并跳过订阅(上行不受影响)。 */
static char         g_sub_topic[G4_SUB_TOPIC_MAX];

/* ==================== v79: broker 参数可由 main.c 注入(配置域 network) ====================
 * 【为什么是"设置接口"而不是让 g4_at 直接读 config_store】
 *   直接读会把 config_store + cJSON 拖进 g4_at 的依赖, 而 **test_state 那套主机单测
 *   正好编译 g4_at.c**(它的 src 列表里没有 config_store) -> 直接【链接失败】;
 *   而 test_state 是本工程唯一带历史失败的套件, 不该为这个去动它。
 *   加一个设置接口则 g4_at 保持零配置依赖, 与既有的 g4_at_set_sub_topic() 完全同构。
 * 【默认行为不变】四个缓冲以原编译期常量初始化; main.c 只在配置里非空时才覆盖。
 * 【尺寸】按 config_store.h 的上限留足余量(host≤64 / user≤32 / pass≤64), 不做截断。 */
#define G4_BROKER_HOST_MAX   80
#define G4_BROKER_PORT_MAX   8
#define G4_BROKER_USER_MAX   40
#define G4_BROKER_PASS_MAX   80
static char g_broker_host[G4_BROKER_HOST_MAX] = G4_MQTT_HOST;
static char g_broker_port[G4_BROKER_PORT_MAX] = G4_MQTT_PORT;
static char g_broker_user[G4_BROKER_USER_MAX] = G4_MQTT_USER;
static char g_broker_pass[G4_BROKER_PASS_MAX] = G4_MQTT_PASS;

/* ★2026-10-08: clientID 也做成可注入(默认 "110001")★
 * 原来这里没有缓冲 —— 建链时直接 `(imei[0] ? imei : G4_MQTT_CLIENT_ID)`。
 * 现在默认固定成 "110001"(用户要求), 但保留"运行期能被配置覆盖"的能力:
 * main.c 会用 `network.clientId` 注入(空 = 不改, 用编译期默认)。 */
#define G4_CLIENT_ID_MAX   40
static char g_client_id[G4_CLIENT_ID_MAX] = G4_MQTT_CLIENT_ID;

static g4_pub_item_t g_pub_q[G4_PUB_QUEUE_DEPTH];    /* 排队等着发的 */
static uint8_t       g_pub_q_cnt;
static uint32_t      g_pub_drop_cnt;                 /* 队列满而丢掉的条数(要能看见) */

/* 上一条发完后取队列里的下一条(实现在文件后半, 这里先声明) */
static bool g4_pub_queue_next(void);

/* 恢复退避(实现在文件后半的"用上模组自己的能力"那一节)。
 * 【必须在这里前置声明】: g4_goto_state()(进 ONLINE 时归零退避)和
 * g4_link_recover_ex()(取退避窗口)都在它们之前。 */
static uint32_t g4_backoff_take(void);
static void     g4_backoff_reset(void);
/* 台架改 broker/账号/TLS 的命令处理(实现在日志口命令表【之后】,
 * 而 g4_at_console_cmd() 要调它 —— 所以在这里前置声明)。 */
static bool     g4_con_broker_cmd(const char *args, bool tls_cmd);

/* ==================== AT 应答解析小工具==================== */

/**
 * @brief 取字符串里第一段连续数字AT+CIMI / AT+CGSN 这类"应答即数据电
 * @return 取到的位数0 = 没有数字)
 */
static uint8_t at_take_digits(const char *s, char *out, uint8_t out_sz)
{
    uint8_t n = 0u;

    if ((s == NULL) || (out == NULL) || (out_sz == 0u))
    {
        return 0u;
    }
    out[0] = '\0';

    while ((*s != '\0') && ((*s < '0') || (*s > '9')))
    {
        s++;                       /* 跳过前导非数字命令回显/前缀) */
    }

    while ((*s >= '0') && (*s <= '9'))
    {
        if (n < (uint8_t)(out_sz - 1u))
        {
            out[n++] = *s;
        }
        s++;
    }
    out[n] = '\0';
    return n;
}

/**
 * @brief 依次取出字符串里的整数最大max 不
 * @return 取到的个数 */
static uint8_t at_parse_ints(const char *s, int *out, uint8_t max)
{
    uint8_t n = 0u;

    if ((s == NULL) || (out == NULL))
    {
        return 0u;
    }

    while ((*s != '\0') && (n < max))
    {
        if ((*s >= '0') && (*s <= '9'))
        {
            out[n++] = atoi(s);
            while ((*s >= '0') && (*s <= '9'))
            {
                s++;
            }
        }
        else
        {
            s++;
        }
    }
    return n;
}

/**
 * @brief 解析注册状态: 从 "+CEREG: ..." 里取出 <stat>
 *
 * ★这里踩过一个坑, 现场日志里留下过 `stat=94` 这种鬼数字★
 * 同一个前缀有【两种形态】, 而字段位置不一样:
 *     URC 主动上报 : +CEREG: <stat>[,<tac>,<ci>,<AcT>]
 *                     例: +CEREG: 2,"94B9","0F31EE15",7     ← 第 1 个数才是 stat
 *     轮询应答     : +CEREG: <n>,<stat>[,<tac>,<ci>,<AcT>]
 *                     例: +CEREG: 2,1,"94B9","094B4A46",7   ← 第 2 个数才是 stat
 * 老写法一律取"第 2 个数" —— 于是 URC 形态会去读引号里的 TAC 十六进制串
 * ("94B9" -> 94), 日志里就出现了 `wait reg 30s(stat=94)` 这种不可能的值,
 * 把方向彻底带偏(94 不是一个合法 stat)。
 *
 * 【怎么区分】只看【第 2 个字段的第一个字符是不是数字】:
 *     URC 形态的第 2 个字段是 <tac>, 它要么【不存在】, 要么是【带引号的串】;
 *     轮询应答的第 2 个字段是 <stat>, 一定是【数字】。
 * 实测三种都能判对:
 *     +CEREG: 2,0,,,                 -> 第2字段 "0"  是数字 -> poll, stat=0
 *     +CEREG: 2,1,"94B9",...,7       -> 第2字段 "1"  是数字 -> poll, stat=1
 *     +CEREG: 2,"94B9","0F31EE15",7  -> 第2字段 '"'  非数字 -> URC,  stat=2
 *     +CEREG: 2,0,,,  之后那条 URC   -> 同上
 *
 * @return stat; 0xFF = 应答里没有该字段
 */
static uint8_t at_reg_stat(const char *resp, const char *tag)
{
    const char *p;
    uint16_t    v0 = 0u;
    uint16_t    v1 = 0u;
    bool        has_v1 = false;

    if ((resp == NULL) || (tag == NULL))
    {
        return 0xFFu;
    }

    p = strstr(resp, tag);
    if (p == NULL)
    {
        return 0xFFu;
    }
    p = strchr(p, ':');
    if (p == NULL)
    {
        return 0xFFu;
    }
    p++;

    /* ---- 第 1 个字段必须存在且是数字 ---- */
    while ((*p == ' ') || (*p == '\t')) { p++; }
    if ((*p < '0') || (*p > '9'))
    {
        return 0xFFu;
    }
    while ((*p >= '0') && (*p <= '9'))
    {
        v0 = (uint16_t)((v0 * 10u) + (uint16_t)(*p - '0'));
        p++;
    }

    /* ---- 看第 2 个字段是不是数字 ---- */
    while ((*p == ' ') || (*p == '\t')) { p++; }
    if (*p == ',')
    {
        p++;
        while ((*p == ' ') || (*p == '\t')) { p++; }
        if ((*p >= '0') && (*p <= '9'))
        {
            has_v1 = true;
            while ((*p >= '0') && (*p <= '9'))
            {
                v1 = (uint16_t)((v1 * 10u) + (uint16_t)(*p - '0'));
                p++;
            }
        }
    }

    /* 第 2 个字段是数字 -> 轮询应答形态(<n>,<stat>); 否则 -> URC 形态(<stat>,...) */
    return (uint8_t)(has_v1 ? v1 : v0);
}

/**
 * @brief 从应答里挑出第一个形如a.b.c.d 的IPv4(排除 0.0.0.0)
 * @return true 取到
 */
/**
 * @brief 从 "+MDIALUP: 1,1,"10.42.18.188","","183.230.126.225","..." 里取出 IPv4
 *
 * 【为什么要专门写这个, 不能直接用 at_take_ipv4()】: AT+MDIALUP? 的应答有【两条】,
 * 一条 IPv4、一条 IPv6, 而且网关/DNS 字段可能为空串:
 *     +MDIALUP: 1,1,"10.42.18.188","","183.230.126.225","183.230.126.224"
 *     +MDIALUP: 1,1,"2409:8960:...","","2409:8060:...","..."
 * 光"找第一段 x.x.x.x"会把 DNS 的 183.230.126.225 当成本机 IP。
 * 所以这里只认【引号里的字段】: 逐个 "..." 取出来, 挑第一个"像 IPv4"的
 * (含点、不含冒号) —— 那就是手册里 <IPv4> 那个字段。
 *
 * @return true = 取到了
 */
static bool at_take_v4_quoted(const char *s, char *out, uint8_t out_sz)
{
    const char *p;

    if ((s == NULL) || (out == NULL) || (out_sz < 8u))
    {
        return false;
    }
    out[0] = '\0';

    p = s;
    while ((p = strchr(p, '"')) != NULL)
    {
        const char *e = strchr(p + 1, '"');
        uint8_t     len;
        uint8_t     i;
        bool        looks_v4 = true;

        if (e == NULL)
        {
            break;
        }
        len = (uint8_t)(e - (p + 1));
        if ((len >= 7u) && (len < out_sz))
        {
            for (i = 0u; i < len; i++)
            {
                char c = p[1 + i];

                if (((c < '0') || (c > '9')) && (c != '.'))
                {
                    looks_v4 = false;   /* IPv6 里的 ':' 或别的东西 */
                    break;
                }
            }
            if (looks_v4 && (strchr(p + 1, '.') != NULL) &&
                ((const char *)strchr(p + 1, '.') < e))
            {
                memcpy(out, p + 1, len);
                out[len] = '\0';
                return true;
            }
        }
        p = e + 1;
    }
    return false;
}

/** @brief 注册状态 <stat> 的人话解释(3GPP TS 27.007) */
static const char *g4_reg_stat_text(uint8_t stat)
{
    switch (stat)
    {
        case 0u:   return "未注册, 也没在搜";
        case 1u:   return "已注册(本地)";
        case 2u:   return "未注册, 正在搜网";
        case 3u:   return "★注册被拒★";
        case 4u:   return "未知";
        case 5u:   return "已注册(漫游)";
        default:   return "查询无应答";
    }
}

static bool at_take_ipv4(const char *s, char *out, uint8_t out_sz)
{
    const char *p;

    if ((s == NULL) || (out == NULL) || (out_sz < 8u))
    {
        return false;
    }
    out[0] = '\0';

    for (p = s; *p != '\0'; p++)
    {
        char        tmp[G4_IP_LEN];
        uint8_t     ti = 0u;
        uint8_t     seg;
        const char *q = p;

        for (seg = 0u; seg < 4u; seg++)
        {
            uint8_t d = 0u;

            while ((*q >= '0') && (*q <= '9') && (d < 3u))
            {
                if (ti < (uint8_t)(sizeof(tmp) - 1u)) { tmp[ti++] = *q; }
                q++;
                d++;
            }
            if (d == 0u)
            {
                break;
            }
            if (seg < 3u)
            {
                if (*q != '.')
                {
                    break;
                }
                if (ti < (uint8_t)(sizeof(tmp) - 1u)) { tmp[ti++] = *q; }
                q++;
            }
        }

        if (seg == 4u)
        {
            tmp[ti] = '\0';
            if (strcmp(tmp, "0.0.0.0") != 0)
            {
                uint8_t i;
                for (i = 0u; (i < ti) && (i < (uint8_t)(out_sz - 1u)); i++)
                {
                    out[i] = tmp[i];
                }
                out[i] = '\0';
                return true;
            }
        }
    }
    return false;
}

/* ==================== 状态机辅助 ==================== */

static const char *g4_state_name[] = {
    "RESET", "WAIT_MODULE", "AT_SYNC", "ATE0", "MLPMCFG", "CPIN", "CGSN",
    "CIMI", "ICCID", "CSQ", "CREG", "CGDCONT", "CGACT", "CGPADDR", "DIALUP",
    "MQTT_DISC", "MQTT_CONN", "WAIT_CONN", "MQTT_SUB", "ONLINE", "ERROR",
    "DIAG_DONE", "REG_CHECK", "REG_ARM", "MQTT_CFG"
};

/** @brief +MQTTURC: "conn",<id>,<state> 里<state> 的人话解重*/
static const char *g4_conn_state_text(uint8_t st)
{
    switch (st)
    {
        /* 建链失败时这行会打出来 直接区分"被服务器拒和网络不通 */
        case 0u:   return "connected";
        case 1u:   return "reconnecting";
        case 2u:   return "user-disconnect";
        case 3u:   return "server-reject(user/pass/clientID/ACL)";
        case 4u:   return "server-disconnect";
        case 5u:   return "ping-timeout(net down)";
        case 6u:   return "net-error";
        case 255u: return "unknown";
        default:   return "no-conn-urc-yet";
    }
}

static void g4_goto_state(g4_state_t st)
{
    /* 【切状态必须作废在途命令】: 否则上一条命令的应答会落到新状态的等待里,
     * 比如上一条的 "OK" 被新命令的关键字匹配到 —— 那就是"假成功"。
     * 非阻塞引擎下这一步是必须的(老的阻塞写法天然不会出现)。 */
    g4_io_cmd_abort();

    g_state      = st;
    g_state_tick = get_tick_ms();
    g_retry_cnt  = 0u;
    g_step       = 0u;
    g_wait_tick  = g_state_tick;

    if (st == G4_STATE_WAIT_MODULE)
    {
        /* 记下"进这个窗口时已经听到过几次 +MATREADY", 之后只要计数变了就说明
         * 模组开口了。收包泵在后台一直统计, 所以这里不用自己读缓冲区。 */
        g_ready_base = g4_io_ready_total();
    }

    if (st == G4_STATE_ONLINE)
    {
        /* 同上, 但这次是拿来盯"在线期间模组有没有偷偷重启过"(见 ONLINE 分支) */
        g_ready_base_online = g4_io_ready_total();

        /* 驻网观测: 进入 ONLINE 时把轮询状态收干净。
         * 【注意这里【不】下发 CEREG=2/CGEREP=1】—— 那两条在 G4_STATE_REG_ARM
         * 里发(上线之前), 因为本工程有一条被主机测试钉死的不变式:
         * ONLINE 稳态下不许发 AT。理由见 g4_cereg_diag_poll() 上面的长注释。 */
        g_cereg_inflight  = false;
        g_cereg_next_tick = get_tick_ms();

        /* 真的连上了: 恢复退避归零(下次出事从 min 重新开始), "正在重连"标记清除。
         * 【必须在这里清】否则一次退避之后窗口会一直挂着, 后面每次都降级成不断电。 */
        g4_backoff_reset();
        g_recover_hold_tick = 0u;
        g_mqtt_reconn_since = 0u;
    }

    log_info("[G4] -> %s\r\n", g4_state_name[st]);
}

/* ==================== 供电开关PA8)小工具==================== */

/** @brief PA8 的实际电平 给日志用("H"/"L") */
static const char *g4_pwr_level_text(void)
{
    return (G4_PowerSwitch_GetLevel() != 0u) ? "H" : "L";
}

/** @brief 当前"上电"电平的人读*/
static const char *g4_pwr_polarity_text(void)
{
    return (G4_PowerSwitch_GetOnHigh() != 0u) ? "PA8 高电平上电" : "PA8 低电平上电";
}

/**
 * @brief 给模组断电再上电, 并重新开始预热窗口 * @param why 日志前缀(谁触发的)
 * @note 断电期间 G4_PowerSwitch_Set(0) 会顺手把 PC8(TX) 放开, 免得反向灌电】 */
static void g4_pwr_cycle_and_listen(const char *why)
{
    G4_PowerCycle(G4_PWR_OFF_MS);
    g4_io_drain_rx();               /* 丢掉上一次上电残留的字节, 同时复位收包状态机 */
    g4_io_rx_reset();               /* 累计接收计数按"本次上电"重新算 */
    g_state_tick = get_tick_ms();   /* 预热窗口从重新上电这一刻起算 */
    log_info("[G4] %s: PA8=%s (%s), 已重新上电, 开始预热(只听不发)...\r\n",
             why, g4_pwr_level_text(), g4_pwr_polarity_text());
}

/* ---------------- 断链恢复(4G_RST 断电重启) ---------------- */

static uint32_t g_last_rst_tick;    /* 最近一次断电重启的时刻 */
static bool     g_rst_used;         /* 本次开机有没有用过断电重启 */

/** @brief 现在允许给模组断电重启吗(带最小间隔, 免得抖一下就一直断电) */
static bool g4_rst_allowed(void)
{
    if (!g_rst_used)
    {
        return true;
    }
    return ((uint32_t)(get_tick_ms() - g_last_rst_tick) >= G4_RST_MIN_GAP_MS);
}

/**
 * @brief 判定"断网了 / 与服务器失去连接 / 模组不理人"之后的统一恢复入口
 *
 * 【做的是什么】: 通过 4G_RST(PA8 电源开关)给模组断电 -> 重新上电 -> 重新预热
 * -> 重新握手 -> 重新入网 -> 重新连 MQTT。也就是现场要的"直接重启模块重建连接"。
 *
 * 【为什么不是每次都断电】: 网络抖一下也会走到这里; 无脑断电会把模组按在
 * "断电-上电-重搜网"的循环里, 反而永远建不起链。所以有 G4_RST_MIN_GAP_MS 最小
 * 间隔: 间隔内已经重启过, 这次就退回"只重新握手不断电"(ERROR->AT_SYNC)。
 *
 * @param force true = 【跳过最小间隔, 立刻断电】
 *        只在"AT 通道状态已经不可信"时用(例如数据模式提示 '>' 丢失) ——
 *        那时模组可能正卡在数据模式里吃我们的 AT 命令, 只有断电能把它拉回
 *        干净状态, 再等最小间隔没有意义。force 时 G4_RST_ON_LINK_LOSS=0 也照样断电。
 */
static void g4_link_recover_ex(const char *why, bool force)
{
    bool do_rst = force;

    log_info("[G4] !! %s\r\n", why);

#if G4_RST_ON_LINK_LOSS
    if (!do_rst && g4_rst_allowed())
    {
        do_rst = true;
    }
#endif

    if (do_rst)
    {
        /* ★退避(2026-10-04 第 2 步)★
         * 【为什么要它】现场 1.6 小时断电重启 27 次(≈每 3.5 分钟一次), 每次 40~60 秒,
         * 而且清掉模组的入网进度 —— 我们自己的恢复动作变成了新的故障源(网络抖一下
         * 就被我们按进"断电-上电-重搜网"的死循环, 这正是本文件开头那段顾虑)。
         * 文档要的指数退避一直没实现, 因为 network.reconnectMin/MaxSeconds 是死配置。
         * 现在: 真连上过就归零; 连续恢复时退避翻倍(首次 min, 封顶 max)。
         * 【窗口内怎么办】降级成"只重新握手(不断电)"—— 也就是还是让它重建 MQTT,
         * 但不碰 PA8。这样既不会把模组按在断电循环里, 也不会什么都不做。 */
        if ((int32_t)(get_tick_ms() - g_recover_hold_tick) < 0)
        {
            log_info("[G4] -> 退避窗口内(还剩 %lus, 当前退避 %lus): 这次【不断电】, "
                     "只重新握手\r\n",
                     (unsigned long)(((int32_t)(g_recover_hold_tick - get_tick_ms())) / 1000),
                     (unsigned long)(g_backoff_ms / 1000u));
            g4_goto_state(G4_STATE_ERROR);
            return;
        }

        g_recover_hold_tick = get_tick_ms() + g4_backoff_take();
        g_rst_count++;
        g_last_rst_tick = get_tick_ms();
        g_rst_used      = true;
        log_info("[G4] !! -> 【4G_RST 断电重启模组】PA8=%s, 断电 %ums 后重新上电重建连接"
                 "(累计第 %lu 次; 下次最早 %lus 后)\r\n",
                 g4_pwr_level_text(), (unsigned)G4_PWR_OFF_MS,
                 (unsigned long)g_rst_count,
                 (unsigned long)(g_backoff_ms / 1000u));
        g4_goto_state(G4_STATE_RESET);      /* RESET 里就是断电 + 预热 */
        return;
    }

#if G4_RST_ON_LINK_LOSS
    log_info("[G4] -> 距上次断电重启不足 %us, 这次只重新握手(不断电), 避免把模组"
             "按在断电循环里\r\n", (unsigned)(G4_RST_MIN_GAP_MS / 1000u));
#else
    log_info("[G4] -> 这次只重新握手(未开断链断电重启)\r\n");
#endif
    g4_goto_state(G4_STATE_ERROR);
}

/** @brief 常规恢复(带断电防抖) */
static void g4_link_recover(const char *why)
{
    g4_link_recover_ex(why, false);
}

static const char *reg_query_cmd(uint8_t probe)
{
    switch (probe % 3u)
    {
        case 0u:  return "AT+CEREG?";   /* E-UTRAN(LTE), 权威 */
        case 1u:  return "AT+CGREG?";   /* GPRS/分组基*/
        default:  return "AT+CREG?";    /* 电路基ml307_app.c 用的就是它 */
    }
}

static const char *reg_query_tag(uint8_t probe)
{
    switch (probe % 3u)
    {
        case 0u:  return "+CEREG:";
        case 1u:  return "+CGREG:";
        default:  return "+CREG:";
    }
}

/* ---------- APN 候通 按IMSI 的MNC 猜运营商 ---------- */

#if G4_APN_AUTO
/**
 * @brief IMSI(460 + MNC + MSIN)里的 MNC 对应的APN; 认不出返回NULL
 */
static const char *apn_guess_by_imsi(const char *imsi)
{
    uint8_t mnc;

    if ((imsi == NULL) || (strncmp(imsi, "460", 3u) != 0))
    {
        return NULL;    /* 不是中国号段就不猜*/
    }
    if ((imsi[3] < '0') || (imsi[3] > '9') || (imsi[4] < '0') || (imsi[4] > '9'))
    {
        return NULL;
    }

    mnc = (uint8_t)((imsi[3] - '0') * 10 + (imsi[4] - '0'));
    switch (mnc)
    {
        case 0u: case 2u: case 4u: case 7u: case 8u:  return G4_APN_MOBILE;
        case 1u: case 6u: case 9u:                     return G4_APN_UNICOM;
        case 3u: case 5u: case 11u:                    return G4_APN_TELECOM;
        default:                                       return NULL;
    }
}
#endif /* G4_APN_AUTO */

/**
 * @brief 生成 APN 候选表: 首通手动指定 > IMSI 判断 > 移动) + 另外两家
 */
static void apn_build_candidates(void)
{
    static const char *const all[3] = { G4_APN_MOBILE, G4_APN_UNICOM, G4_APN_TELECOM };
    const char  *first = NULL;
    uint8_t      i, n = 1u;

#if G4_APN_AUTO
    first = apn_guess_by_imsi(g_info.imsi);
#endif
    if ((G4_APN[0] != '\0') || (first == NULL))
    {
        first = (G4_APN[0] != '\0') ? G4_APN : G4_APN_MOBILE;
    }

    g_apn_list[0] = first;
    g_apn_idx     = 0u;

    for (i = 0u; i < 3u; i++)       /* 补齐另外两家(第一家就是它自己就跳过 */
    {
        if (strcmp(all[i], first) == 0)
        {
            continue;
        }
        g_apn_list[n++] = all[i];
        if (n >= 3u)
        {
            break;
        }
    }
}

/* ==================== MQTT 遗嘱(LWT) ====================
 * 文档 2 章 / 7.7: status topic 是 retained, 且要带 LWT 遗嘱 {"online":false};
 * 网关【异常掉线】(断电/拔卡/信号丢)时由 broker 代发这条, 平台靠它区分
 * "设备死了"和"只是没数据"。如果没有遗嘱, broker 只留着最后一条 retained 的
 * status.report{online:true} —— 平台会一直以为网关活着。
 *
 * 【为什么默认关闭】模组是否支持遗嘱参数【还没在台架上确认】:
 *   本工程手里的 4G 手册(ML307R_ATCommands.pdf, 29 页"通信流程示例")里
 *   【没有 MQTT 章节】, 工程内也没有任何遗嘱语法的记录(test.txt 记的 MQTTCONN
 *   也只有 6 个参数)。所以拼进去之前必须先跑 MQTT_LWT_台架测试.txt 的发现步骤。
 *
 * 【打开方式】MAIN_MQTT_LWT_ENABLE 改成 1; 默认 0 = MQTTCONN 与验证过的流程
 *   逐字节一致(谁也别在不确定的时候把现场能用的建链流程改坏)。
 *   ★ 参数写错会让 MQTTCONN 直接失败, 而"连不上"在现场极易被当成信号问题查半天。
 *
 * 【待确认的写法】mqtt_apply_lwt_config() 里那一行按台架结果改(g4_at.c 只有这一处)。
 *   给出的是最可能的一种(遗嘱 topic + qos + retain + 正文), 若模组用 MQTTCFG
 *   配置遗嘱, 则改成"MQTTCONN 之前先发一条 MQTTCFG"(见台架测试文档的方案 B)。
 */
#ifndef MAIN_MQTT_LWT_ENABLE
#define MAIN_MQTT_LWT_ENABLE   1
#endif

/** 遗嘱正文(文档 7.7 原文: LWT 遗嘱消息(retained): {"online": false}) */
#define G4_LWT_PAYLOAD_DEFAULT   "{\"online\":false}"

static char g_lwt_topic[80];     /* status topic; 空 = 没配遗嘱 */
static char g_lwt_payload[64];

void g4_at_set_lwt(const char *topic, const char *payload)
{
    if ((topic == NULL) || (topic[0] == '\0'))
    {
        g_lwt_topic[0]   = '\0';
        g_lwt_payload[0] = '\0';
        return;
    }

    (void)snprintf(g_lwt_topic, sizeof(g_lwt_topic), "%s", topic);
    (void)snprintf(g_lwt_payload, sizeof(g_lwt_payload), "%s",
                   ((payload != NULL) && (payload[0] != '\0')) ? payload
                                                               : G4_LWT_PAYLOAD_DEFAULT);
    log_info("[G4] LWT 遗嘱已备好: topic=%s payload=%s (启用=%s)\r\n",
             g_lwt_topic, g_lwt_payload,
#if MAIN_MQTT_LWT_ENABLE
             "是"
#else
             "否: 语法待台架确认, 见 MQTT_LWT_台架测试.txt"
#endif
             );
}

/** 拼 AT+MQTTCONN 的遗嘱参数部分; 未启用/没配 -> 空串(于是与旧流程逐字节一致) */
/**
 * @brief 按下发顺序组一条 "AT+MQTTCFG=..." 命令(2026-10-04 第 2 步)
 *
 * @param step 第几步(0 起); 与 G4_MQTT_CFG_STEPS / G4_STATE_MQTT_CFG 对齐
 * @param buf  输出缓冲
 * @param size 缓冲大小
 * @return true = buf 里有一条要发的命令; false = 这一步不需要发(跳过它)
 *
 * 【为什么改成"组命令"而不是原来的阻塞下发】原实现 mqtt_apply_lwt_config() 是
 * 阻塞封装(每条最长 1.5s), 而且【只在 MQTT 快修那条路上被调用】—— 冷启动建链
 * 从来不发遗嘱; 手册又明确"模组掉电后配置参数不保存", 于是每次 4G_RST 之后遗嘱
 * 就丢了。现在改成状态机的一步, 走正常建链路径, 每次建链都重下发一遍。
 *
 * 步骤表(与 main.c 注入的值一起决定实际命令):
 *   0 willoption   遗嘱开关+QoS+retain(只在配了遗嘱 topic 时才发)
 *   1 encoding     ★MQTT 消息格式: 必须放在 willpayload 之前★
 *                  现场日志证实: 默认(ASCII)把 `\"` 的反斜杠原样存下来, 遗嘱
 *                  到平台就是坏 JSON(见 G4_MQTT_ENC_INPUT 的注释)。
 *   2 willpayload  遗嘱正文(正文里的 " 转义成 \", 配合 input_format=2 才生效)
 *   3 clean        干净会话(默认 0=持久会话, 与本工程"重连即全量重报"不符)
 *   4 keepalive    保活时间(手册: 0 或 60~65535)
 *   5 pingreq      MQTT心跳间隔
 *   6 pingresp     心跳请求回显(开 = 能收到 +MQTTURC: "pingresp" 心跳结果)
 *   7 reconn       模组自带重连(次数/间隔/策略)
 */
static bool mqtt_cfg_build(uint8_t step, char *buf, uint16_t size)
{
    static char esc[128];
    int         n = -1;

    if ((buf == NULL) || (size == 0u))
    {
        return false;
    }
    buf[0] = '\0';

    switch (step)
    {
    case G4_CFG_STEP_WILLOPT:
        if ((g_lwt_topic[0] == '\0') || (g_lwt_payload[0] == '\0'))
        {
            return false;       /* 没配遗嘱 -> 跳过这几步 */
        }
        n = snprintf(buf, size, "AT+MQTTCFG=\"willoption\",%s,1,1,1",
                     G4_MQTT_CONNECT_ID);
        break;

    case G4_CFG_STEP_ENCODING:
        if ((g_lwt_topic[0] == '\0') || (g_lwt_payload[0] == '\0'))
        {
            return false;
        }
        /* ★必须在 willpayload 之前★: 它决定 <will_msg> 怎么被解释 */
        n = snprintf(buf, size, "AT+MQTTCFG=\"encoding\",%s,%u,%u",
                     G4_MQTT_CONNECT_ID,
                     (unsigned)G4_MQTT_ENC_INPUT, (unsigned)G4_MQTT_ENC_OUTPUT);
        break;

    case G4_CFG_STEP_WILLPAY:
        if ((g_lwt_topic[0] == '\0') || (g_lwt_payload[0] == '\0'))
        {
            return false;
        }
        {
            const char *q;
            uint16_t    w = 0u;

            for (q = g_lwt_payload;
                 (*q != '\0') && ((uint16_t)(w + 2U) < (uint16_t)sizeof(esc));
                 q++)
            {
                if (*q == '"') { esc[w++] = '\\'; }
                esc[w++] = *q;
            }
            esc[w] = '\0';
        }
        n = snprintf(buf, size, "AT+MQTTCFG=\"willpayload\",%s,\"%s\",\"%s\"",
                     G4_MQTT_CONNECT_ID, g_lwt_topic, esc);
        break;

    case G4_CFG_STEP_CLEAN:
        /* ★干净会话★ —— 本工程是"无状态 + 重连即全量重报"(文档 9.4), 不是持久会话。
         * 默认 0 会让 broker 把上一次没确认的 QoS1 报文与 packet-id 一起接回来, 而
         * 现场恰恰是同一个 clientId 反复重连 —— 见 G4_MQTT_CLEAN_SESSION 的注释。 */
        n = snprintf(buf, size, "AT+MQTTCFG=\"clean\",%s,%u",
                     G4_MQTT_CONNECT_ID, (unsigned)G4_MQTT_CLEAN_SESSION);
        break;

    case G4_CFG_STEP_KEEPALIVE:
        if (g_mqtt_keepalive_s == 0u)
        {
            return false;       /* 0 = 不干预, 用模组默认 */
        }
        n = snprintf(buf, size, "AT+MQTTCFG=\"keepalive\",%s,%u",
                     G4_MQTT_CONNECT_ID, (unsigned)g_mqtt_keepalive_s);
        break;

    case G4_CFG_STEP_PINGREQ:
        n = snprintf(buf, size, "AT+MQTTCFG=\"pingreq\",%s,%u",
                     G4_MQTT_CONNECT_ID, (unsigned)G4_MQTT_PINGREQ_S);
        break;

    case G4_CFG_STEP_PINGRESP:
        /* 开"心跳请求回显" —— 这样我们能收到 +MQTTURC: "pingresp",<id>,<ping_ret>,
         * 那是【模组级】的保活探针结果(0=成功 / 1=超时), 比应用层发布计数更贴近
         * 协议层, 是判断"会话真的还活着吗"最直接的一条证据。 */
        n = snprintf(buf, size, "AT+MQTTCFG=\"pingresp\",%s,1", G4_MQTT_CONNECT_ID);
        break;

    case G4_CFG_STEP_RECONN:
        /* ★模组自带重连★ mode=1 即 n 倍间隔(20/40/60s), 合计 120s 自愈窗口。
         * 有了它, 我们就【不该】在模组正重连时去断电/快修 —— 见 g4_at_poll 里
         * conn state=1 的处理。 */
        if (G4_MQTT_RECONN_TIMES == 0u)
        {
            return false;       /* 0 次 = 关掉模组重连(调试用) */
        }
        n = snprintf(buf, size, "AT+MQTTCFG=\"reconn\",%s,%u,%u,%u",
                     G4_MQTT_CONNECT_ID,
                     (unsigned)G4_MQTT_RECONN_TIMES,
                     (unsigned)G4_MQTT_RECONN_IVL_S,
                     (unsigned)G4_MQTT_RECONN_MODE);
        break;

    /* ---------------- TLS(mqtts): 全部只在开启时才发 ---------------- */

    case G4_CFG_STEP_TLS_AUTH:
        /* 【默认整段跳过】见 G4_TLS_USE_MSSLCFG 的注释: SSL 上下文的出厂默认
         * (auth=0 / ignorestamp=1 / ignoreverify=0) 正好就是我们要的, 多写一遍
         * 反而可能覆盖掉模组预置的东西。只有要做"单向认证"时才需要打开这个闸。 */
        if ((!g_tls_enable) || (G4_TLS_USE_MSSLCFG == 0))
        {
            return false;
        }
        n = snprintf(buf, size, "AT+MSSLCFG=\"auth\",%u,%u",
                     (unsigned)G4_TLS_SSL_ID, (unsigned)G4_TLS_AUTH_MODE);
        break;

    case G4_CFG_STEP_TLS_STAMP:
        if ((!g_tls_enable) || (G4_TLS_USE_MSSLCFG == 0))
        {
            return false;
        }
        n = snprintf(buf, size, "AT+MSSLCFG=\"ignorestamp\",%u,%u",
                     (unsigned)G4_TLS_SSL_ID, (unsigned)G4_TLS_IGNORE_STAMP);
        break;

    case G4_CFG_STEP_TLS_VERIFY:
        if ((!g_tls_enable) || (G4_TLS_USE_MSSLCFG == 0))
        {
            return false;
        }
        n = snprintf(buf, size, "AT+MSSLCFG=\"ignoreverify\",%u,%u",
                     (unsigned)G4_TLS_SSL_ID, (unsigned)G4_TLS_IGNORE_VERIFY);
        break;

    case G4_CFG_STEP_TLS_CERT:
        if ((!g_tls_enable) || (G4_TLS_USE_MSSLCFG == 0))
        {
            return false;
        }
        /* 只有单向/双向认证才需要绑定 CA 证书, 而且必须先把证书写进模组
         * (AT+MSSLCERTWR) —— 所以没给证书名就跳过, 不要发一条空名字的命令。 */
        if ((G4_TLS_AUTH_MODE == 0u) || (G4_TLS_CA_CERT[0] == '\0'))
        {
            return false;
        }
        n = snprintf(buf, size, "AT+MSSLCFG=\"cert\",%u,\"%s\"",
                     (unsigned)G4_TLS_SSL_ID, G4_TLS_CA_CERT);
        break;

    case G4_CFG_STEP_TLS_SSL:
        if (!g_tls_enable)
        {
            return false;
        }
        /* ★这一步才是真正开 TLS 的地方★
         * <ssl_enable> 0=普通TCP 1=SSL TCP; <ssl_id>=要用的 SSL 上下文。
         * 它自己就带 ssl_id, 所以不需要事先用 AT+MSSLCFG 去"配"那个上下文 ——
         * 模组会用它在该上下文上的出厂默认配置(正合我们要的)。 */
        n = snprintf(buf, size, "AT+MQTTCFG=\"ssl\",%s,1,%u",
                     G4_MQTT_CONNECT_ID, (unsigned)G4_TLS_SSL_ID);
        break;

    case G4_CFG_STEP_VERSION:
        /* 钉住 MQTT 协议版本 = 4 (3.1.1)。见 G4_MQTT_PROTO_VER 的说明:
         * conn state=3 的三个可能原因里, 协议版本是最容易排除的一个。 */
        if (G4_MQTT_PROTO_VER == 0u)
        {
            return false;       /* 0 = 不下发, 用模组默认 */
        }
        n = snprintf(buf, size, "AT+MQTTCFG=\"version\",%s,%u",
                     G4_MQTT_CONNECT_ID, (unsigned)G4_MQTT_PROTO_VER);
        break;

    default:
        return false;
    }

    return ((n > 0) && (n < (int)size));
}

void g4_at_set_keepalive_s(uint16_t sec)
{
    uint16_t want = sec;

    /* 【必须夹到手册下限】keepalive 只接受 0 或 60~65535。而文档 network.keepAlive
     * 允许 30~300 —— 若把 30 原样发下去, 模组会回 +CME ERROR, 于是"设了但没生效",
     * 而日志里只看到一条失败。这里夹紧并说清楚。 */
    if (sec != 0u)
    {
        if (sec < G4_MQTT_KEEPALIVE_MIN_S)
        {
            want = G4_MQTT_KEEPALIVE_MIN_S;
        }
    }
    g_mqtt_keepalive_s = want;
    log_info("[G4] 模组级 keepalive = %us%s\r\n",
             (unsigned)want,
             (want != sec) ? " (配置值小于模组下限 60s, 已夹紧)" : "");
}

void g4_at_set_reconnect(uint16_t min_s, uint16_t max_s)
{
    if (min_s == 0u) { min_s = 5u; }
    if (max_s < min_s) { max_s = min_s; }

    g_reconn_min_s = min_s;
    g_reconn_max_s = max_s;
    g_backoff_ms   = 0u;        /* 新参数生效: 退避从头算 */

    log_info("[G4] 恢复退避: 首次 %us, 每次翻倍, 封顶 %us(来自配置 network.reconnect*)\r\n",
             (unsigned)g_reconn_min_s, (unsigned)g_reconn_max_s);
}

void g4_at_set_tls(uint8_t enable)
{
    g_tls_enable = (enable != 0u);

    log_info("[G4] MQTT 传输 = %s (来自配置 network.tls)\r\n",
             g_tls_enable ? "★mqtts / TLS★" : "普通 TCP");
    if (g_tls_enable)
    {
        log_info("[G4]   SSL 上下文 id=%u, 只用 MQTTCFG=\"ssl\" 开(%s)\r\n",
                 (unsigned)G4_TLS_SSL_ID,
                 (G4_TLS_USE_MSSLCFG != 0)
                     ? "并预先下发 MSSLCFG; 认证方式=%u"
                     : "不碰 MSSLCFG, 用模组出厂默认");
        if (G4_TLS_USE_MSSLCFG != 0)
        {
            log_info("[G4]   认证方式=%u (0=不校验证书 1=单向 2=双向), 忽略证书时间戳=%u\r\n",
                     (unsigned)G4_TLS_AUTH_MODE, (unsigned)G4_TLS_IGNORE_STAMP);
        }
        log_info("[G4]   证书相关配置在建链时下发; #at AT+MSSLLIST=1 可看模组内置了哪些 CA\r\n");
    }
}

/** @brief 当前退避窗口(诊断/状态行用) */
uint32_t g4_at_get_backoff_ms(void)
{
    return g_backoff_ms;
}

/** @brief 累计断电重启(4G_RST)次数 —— 判"我们是不是在把模组按在断电循环里" */
uint32_t g4_at_get_rst_count(void)
{
    return g_rst_count;
}

/**
 * @brief 取"下一次整链路恢复"的退避窗口, 并把它翻倍备下次
 *
 * 【为什么必须退避】现场实测(2026-10-04 日志): 1.6 小时里断电重启 27 次
 * (≈每 3.5 分钟一次), 每次 40~60 秒, 且清掉模组的入网进度 —— 我们自己的恢复
 * 动作成了新的故障源。而文档要求的退避(min→max 指数)一直没实现, 因为
 * network.reconnectMin/MaxSeconds 两个字段【没有任何消费者】。
 * 现在接上: 第一次 min, 之后 2× 翻倍, 封顶 max; 一旦回到 ONLINE 就归零。
 */
static uint32_t g4_backoff_take(void)
{
    uint32_t v;
    uint32_t cap = (uint32_t)g_reconn_max_s * 1000u;

    if (g_backoff_ms == 0u)
    {
        v = (uint32_t)g_reconn_min_s * 1000u;
    }
    else if (g_backoff_ms >= cap)
    {
        v = cap;
    }
    else
    {
        v = g_backoff_ms * 2u;
    }
    if (v > cap) { v = cap; }
    if (v == 0u) { v = 1000u; }

    g_backoff_ms = v;
    return v;
}

/** @brief 真的连上了: 退避归零(下次出事从 min 重新开始) */
static void g4_backoff_reset(void)
{
    g_backoff_ms = 0u;
}

static bool mqtt_build_conn_cmd(char *buf, uint16_t size)
{
    /* AT+MQTTCONN=<id>,<host>,<port>,<clientID>,<user>,<passwd>[,<遗嘱参数>]
     *
     * ★clientID 的取值规则(2026-10-08 改)★
     *   默认 = g_client_id, 也就是编译期 "110001"(用户要求"固定使用 110001")。
     *   【特殊值 "IMEI"】: 配成这个字符串就用模组 IMEI ——
     *     多台网关接同一个 broker 时【必须】让 clientID 互不相同,
     *     否则 broker 会用新连接把旧连接顶掉, 两台互相踢、都时通时断。
     *     本文件原来那句注释("clientID 用IMEI ——固定 client ID 会让多台网关互相顶下线")
     *     说的就是这件事。有了这个逃生口, 单台可以用固定值、上多台时改一个字符串即可,
     *     不必重新烧固件。
     *   【IMEI 也读不到】: 退回编译期默认(绝不能发空 clientID —— 有些 broker 直接拒)。 */
    const char *cid = g_client_id;

    if ((strcmp(g_client_id, "IMEI") == 0) && (g_info.imei[0] != '\0'))
    {
        cid = g_info.imei;
    }
    if (cid[0] == '\0')
    {
        cid = G4_MQTT_CLIENT_ID;
    }

    int n = snprintf(buf, size, "AT+MQTTCONN=%s,\"%s\",%s,\"%s\",\"%s\",\"%s\"",
                     G4_MQTT_CONNECT_ID, g_broker_host, g_broker_port,
                     cid, g_broker_user, g_broker_pass);
    return (n > 0) && (n < (int)size);
}

static void g4_info_clear(void)
{
    memset(&g_info, 0, sizeof(g_info));
    g_info.csq        = 99u;
    g_info.reg_status = 0xFFu;
}

/* ==================== 模组自测供电电压 (AT+CBC) ====================
 *
 * 【为什么一定要有这个【 "4G 模组供电不稳、在 3.8~4.1V 之间跳 这类问题用万用表
 * 是量不准的——万用表直流档读的是平均值 采样又慢, 模组发射瞬间掉到 3.0V 它也照样
 * 给你显示 3.8V。而
 *   · AT+CBC 是【模组内部ADC 自己测的 VBAT【 由模组固件持续采样
 *     它报多少就是模组"看到"多少 ——这才是真正决定模组会不会复位的那个电压
 *   · 日志口#vbatsweep 连测 N 次取 min/max/avg, 就能看出到底有没有真跌落【 *
 * 应答格式(3GPP TS 27.007): +CBC: <bcs>,<bcl>
 *   <bcs> 0=未充电1=充电一2=充电完成 3=无电池 *   <bcl> 各厂商不一致 移远/中移这类模块给【mV【所以>1000 就按 mV 解释),
 *         也有实现给百分比(0~100)。这里按数值大小自动判断 并把原始应答打出来。 *
 * ⚠️ 【ML307Y 不支持这条【 现场实测本板模组是ML307Y, AT+CBC 直接回 *    "+CME ERROR: 4"(命令不支持。所以这个模组自测供电"在ML307Y 上拿不到候
 *    日志会打一行无应答模组可能不支持这条", 属于正常, 不影响联网【 *    要在 ML307Y 上看供电, 只能靠万用表 MIN/MAX 或示波器盯模组VCC【 *    (换型号时先查该型号手册有没有对应命令, 别把 ERROR 当故障查。
 */

/** @brief 取字符串里【逗号后面】那个数字"+CBC: 0,3980" →3980); 0 = 没有 */
static uint32_t at_take_second_number(const char *s)
{
    const char *p;

    if (s == NULL)
    {
        return 0u;
    }
    p = strchr(s, ',');
    if (p == NULL)
    {
        return 0u;
    }
    p++;
    while ((*p == ' ') || (*p == '\t'))
    {
        p++;
    }
    if ((*p < '0') || (*p > '9'))
    {
        return 0u;
    }
    return (uint32_t)atoi(p);
}

/**
 * @brief 读一次模组自测供电电压 *
 * 【命令用 ML307Y 支持的那条【 现场那份跑通的 ml307_app.c 里读电压用的是 *     AT+MCHIPINFO="vbat"   →  +MCHIPINFO: "vbat",<mV>
 * 而不是3GPP 的AT+CBC ——实测本板模组(ML307Y)对AT+CBC 直接回 * "+CME ERROR: 4"(命令不支持, 对MCHIPINFO 才给值。 * 所以这里的顺序是 全MCHIPINFO(该型号的正确命令), 再退回CBC(别的型号)。 *
 * @return true = 读到了mV
 * @note 【阻塞】最长1s×2。两条都不支持时只打一行日志 不影响别的功能【 */
static bool g4_query_vbat_mv(uint32_t *out_mv)
{
    char     resp[96];
    uint32_t v;

    if (out_mv == NULL)
    {
        return false;
    }

    /* ①ML307Y / OneMO 系列: AT+MCHIPINFO="vbat" →+MCHIPINFO: "vbat",3980 */
    if (g4_io_send_expect_resp("AT+MCHIPINFO=\"vbat\"", "+MCHIPINFO:",
                               AT_CMD_TIMEOUT_MS, NULL, resp, sizeof(resp)))
    {
        v = at_take_second_number(resp);
        if (v >= 1000u)
        {
            *out_mv = v;
            return true;
        }
    }

    /* ①兜底: 3GPP 的AT+CBC(部分型号支持, 固+CBC: <bcs>,<bcl>) */
    resp[0] = '\0';
    if (g4_io_send_expect_resp("AT+CBC", "+CBC:", AT_CMD_TIMEOUT_MS, NULL,
                               resp, sizeof(resp)))
    {
        v = at_take_second_number(resp);
        if (v >= 1000u)
        {
            *out_mv = v;
            return true;
        }
    }

    return false;
}

/** @brief 读一次并记进 g_info(带日志 */
static bool g4_read_vbat(void)
{
    uint32_t mv;

    if (!g4_query_vbat_mv(&mv))
    {
        log_info("[G4] 模组自测供电读不到本型号可能两条命令都不支持 不影响联网\r\n");
        return false;
    }

    g_info.vbat_mv    = (uint16_t)((mv > 65535u) ? 65535u : mv);
    g_info.vbat_valid = true;
    log_info("[G4] 模组自测供电 = %lu mV\r\n", (unsigned long)mv);

    /* 【低压告警 —— 本次故障的判据】
     * 4G(Cat.1)模组的 VBAT 规格一般是 3.4~4.2V(典型 3.8V)。模组【自己测】出来的
     * 电压掉到 3.4V 以下时, 它会冒出一整串"看起来像软件问题"的症状:
     *   · AT 引擎反复重启 → 日志里 +MATREADY 刷屏(实测能数到几十次);
     *   · 搜不到网 → CSQ=99;
     *   · 注册被拒 → +CEREG: 0,3;
     *   · 偶尔又能连上 → 于是更难查。
     * 现场就见过模组报 2555mV(2.555V) —— 那不是软件能修的, 必须动硬件。 */
    if (mv < 3400u)
    {
        log_info("[G4] !! !! 模组自测供电只有 %lumV —— 【低于 3.4V 最低工作电压】!\r\n",
                 (unsigned long)mv);
        log_info("[G4] !! 这就是\"AT 一直重来 / CSQ=99 / 注册被拒(stat=3)\"的根因, "
                 "不是代码问题\r\n");
        log_info("[G4] !! 按顺序查:\r\n");
        log_info("[G4] !!  1) 模组 VCC 处加 470uF 低阻电解 + 10uF + 100nF(紧贴引脚)\r\n");
        log_info("[G4] !!  2) 供电/电池能否给到 2A 峰值: 电池是否亏电、线是否够短够粗\r\n");
        log_info("[G4] !!  3) 万用表开 MIN/MAX 量模组 VCC: 搜网瞬间最低掉到多少\r\n");
        log_info("[G4] !!  4) PA8 控的那个开关(MOS/稳压器): 量开关【两侧】的电压, "
                 "差得多就是它自己压降大(没完全导通/Rds 太大)\r\n");
        log_info("[G4] !!  5) 模组是 3.8V 器件: 直接给 3.3V 本身就偏低\r\n");
    }
    return true;
}

/**
 * @brief 读一次模组温应AT+MCHIPINFO="temp")
 *
 * 【为什么顺手读它【 模组"用着用着自己重启"除了供电, 另一个常见原因是
 * 【过热保护】。ml307_app.c 里就是这么读的+MCHIPINFO: "temp",<℀),
 * 这里跟着读一次 重启时日志里就有温度可查, 不用瞎猜【 */
static void g4_read_temp(void)
{
    char     resp[96];
    uint32_t t;

    if (!g4_io_send_expect_resp("AT+MCHIPINFO=\"temp\"", "+MCHIPINFO:",
                                AT_CMD_TIMEOUT_MS, NULL, resp, sizeof(resp)))
    {
        return;     /* 不支持就算了, 不打扰日志*/
    }

    t = at_take_second_number(resp);
    if (t <= 200u)
    {
        g_info.temp_c     = (uint8_t)t;
        g_info.temp_valid = true;
        log_info("[G4] 模组温度 = %lu C\r\n", (unsigned long)t);
    }
}

/**
 * @brief 连测 N 次AT+CBC, 给出 min/max/avg ——专门用来把供电跳变"
 *
 * 【判据【 4G(Cat.1)模组的VBAT 规格一般是 3.4~4.2V, 典型 3.8V【 *   · 电压在3.7~4.1V 之间来回 →这是【负载调制【 属于正常(模组在搜网发射),
 *     模组自己也活得好好的 →不用为这个数字返已
 *   · min 掉到 3400mV 以下 →发射瞬间真的在塌, 那就是要加储能电容/ 换电源【 * @note 【阻塞】n × (1 次AT 往这+ 150ms), 期间 LoRa 会被拖慢 ——台架命令, 别常态用。 */
static void g4_vbat_sweep(uint16_t n)
{
    uint16_t i;
    uint32_t v_min = 0xFFFFFFFFu;
    uint32_t v_max = 0u;
    uint32_t sum   = 0u;
    uint32_t cnt   = 0u;

    if (n == 0u)  { n = 20u; }
    if (n > 200u) { n = 200u; }

    log_info("[DIAG] 连测 %u 次AT+CBC(间隔 150ms)抓供电跳发..\r\n", (unsigned)n);

    for (i = 0u; i < n; i++)
    {
        uint32_t v;

        if (g4_query_vbat_mv(&v))
        {
            if (v < v_min) { v_min = v; }
            if (v > v_max) { v_max = v; }
            sum += v;
            cnt++;
        }
        delay_ms(150u);
    }

    if (cnt == 0u)
    {
        log_info("[DIAG] 一次都没读到——本型号两条命令AT+MCHIPINFO=\"vbat\" / AT+CBC)都不支持?\r\n");
        return;
    }

    g_info.vbat_mv    = (uint16_t)((v_min > 65535u) ? 65535u : v_min);
    g_info.vbat_valid = true;

    log_info("[DIAG] 模组自测供电: min=%lumV max=%lumV avg=%lumV (全%lu 次 波动 %lumV)\r\n",
             (unsigned long)v_min, (unsigned long)v_max,
             (unsigned long)(sum / cnt), (unsigned long)cnt,
             (unsigned long)(v_max - v_min));

    if (v_min < 3400u)
    {
        log_info("[DIAG] !! min 已低了3400mV ——发射瞬间在塌: 模组 VCC 功470uF+10uF+100nF, 换供电\r\n");
    }
    else if ((v_max - v_min) > 300u)
    {
        log_info("[DIAG] 波动 >300mV 但最低点仍在规格内3.4~4.2V) ——属于负载调制, 不影响功能\r\n");
    }
    else
    {
        log_info("[DIAG] 最低点 %lumV 在规格内, 波动也不复——供电没问题 别为这个数字返工\r\n",
                 (unsigned long)v_min);
    }
}

/* ---------- 发布失败看门狗 ----------
 *
 * 【为什么需要】"跑一段时间后 4G 就不通了, 下发也收不到" 的典型形态是
 * 模组自己悄悄死了(MQTT 掉线/假死), 而它连 +MQTTURC: "conn" 都不再发 ——
 * 那么 g_online 永远不会变 false, g4_at_is_online() 一直返回 true,
 * 上层以为链路好好的, 继续把数据丢进一个死模组。
 *
 * 所以拿"我们自己发的 MQTTPUB 有没有被应答"当探针: 连续 N 次等不到 '>' /
 * 等不到 OK / 模组回 ERROR, 就当链路已死, 交给 g4_link_recover() 去
 * 4G_RST 断电重启模组。成功一次就把计数清零。
 *
 * ★2026-09 按现场要求收紧: 5 次 -> 2 次★
 *   原来 5 次 × 30s 心跳 = 最坏 150s 才发现断网, 云端那边早就看着这台设备
 *   不理人了。于是【连续 2 次发布失败就断电重启】—— 也就是"两次心跳上报都失败"。
 *   ⚠️ 现在值已改为 3 次(见下面 #define 处的 2026-09-22 说明)。
 *
 *   为了不让这个严格阈值被"本机问题"误触发, 失败被分成了两类(见
 *   g4_pub_mark_fail / g4_pub_abort):
 *     · 链路类失败(等 '>' 超时 / 等 OK 超时 / 模组回 ERROR) -> 计数, 到阈值就断电;
 *     · 本机类失败(命令组装失败 / 引擎被占用 / 串口写失败) -> 只放弃这一条,
 *       【不计入】链路失败, 否则一次代码 bug 会变成"一直给模组断电"。
 */
#ifndef G4_PUB_FAIL_MAX
/* ★2026-09-22 2 -> 3 (依据现场日志 78.txt)★
 * 一次"模组自己重启"要 40~60 秒才回到 ONLINE; 而连续 2 次发布失败(≈60s)就把整条
 * 链路断电重来 —— 等于在模组刚要回来的时候又把它按回去。放宽到 3 次, 给它留出
 * "自己回来"的时间。 */
#define G4_PUB_FAIL_MAX  3U
#endif
static uint8_t g_pub_fail_cnt;

/* ==================== "'>' 提示符丢失"的处理策略 ====================
 *
 * 【现场依据: 日志 78.txt】4 分钟里 +MATREADY 出现 4 次, 其中两次正是
 *     发 AT+MQTTPUB / AT+MQTTCONN -> 模组哑满 5 秒超时 -> 重试 -> "+MATREADY" + OK
 * 即【模组在那时候重启了】。
 *
 * 而原来的处理是: 等 '>' 时若先收到一行 OK, 就判"AT 状态不可信", 立刻给模组
 * 断电重启(g4_link_recover_ex(..., true))。
 * 【那个前提站不住】: 能走到这一步, 恰恰是因为收到了一整行 "OK"
 * (见 g4_io.c: s_cmd_prompt && strcmp(s_line,"OK")==0 那个判定) ——
 * 模组回了 OK 就说明它【已经把这条命令处理完、回到命令模式】了, 并没有在数据
 * 模式里等我们的 payload。所以"后面每条 AT 都会被当 payload 吃掉"这个担心,
 * 在"先收到 OK"这个场景里不成立。
 *
 * 代价对比: 一次空洞的断电重启 = 40~60 秒全网断(实测); 重发一条 = 一个命令来回。
 * 所以现在改成:
 *      '>' 丢失  ->  敲一条 AT 探活
 *                    AT 回 OK   -> AT 引擎活着, 直接重发本条(最多 G4_PUB_PROMPT_LOSS_MAX 次)
 *                    AT 不应答  -> 这才是真的"AT 通道不可信", 断电重启
 *      连续丢满 G4_PUB_PROMPT_LOSS_MAX 次 -> 真进不了数据模式, 断电重启
 * 于是"模组只是打了个喷嚏"不会再被我们升级成"拆掉整条链路"。
 */
#ifndef G4_PUB_PROMPT_LOSS_MAX
/* ★2026-10-08: 由 3 改成 2 —— 同一条包【最多重发 1 次】★
 *   第 1 次丢 '>' -> 探活 -> 重发本条(这一次是划算的, 见下面到阈值那段的说明)
 *   第 2 次丢 '>' -> 放弃本条(不重发、不断电)
 * 现场证据: 原阈值 3 意味着同一条大包最多发 3 遍, 而每发一遍都可能再撑重启一次模组
 * —— 22.5 分钟里 13 次重发对应 13 次快修, 1:1。 */
#define G4_PUB_PROMPT_LOSS_MAX  2U
#endif
static uint8_t  g_pub_prompt_loss;         /* 连续丢 '>' 次数(发布成功/换故障类型就清零) */
static uint32_t g_pub_prompt_loss_total;   /* 累计次数, 只给 [STAT2] 看趋势 */

/* ==================== MQTT 保活(MQTT keepalive) ====================
 *
 * 【先说清楚这套系统的"保活"是哪几条腿【 *
 *   1) MQTT 协议层keepalive(PINGREQ)
 *      由【模组内部】的 MQTT 栈负责。证换 断线 URC 里有 state=5
 *      "ping-timeout(net down)" 这一档——模组自己会发 PINGREQ, 发不通才报这个【 *      只是 AT+MQTTCONN 里我们没显式指定 keepalive 参数, 用的是模组默认值。 *      日志口敲 "AT+MQTTCONN=?" 可以看这条命令到底支不支持keepalive 参数【 *
 *   2) 应用层心跳本工程的主保活
 *      main.c 每 MAIN_GW_ONLINE_MS(30s) 往 rt/gw/{gwId}/status 发一条状态心跳
 *      (status.report)。 ★2026-10-08: 原来是发到 mod/mod_upload, 那条通道已删除★ *      任何 client→server 报文都会重置 broker 和中间NAT 的空闲计时器,
 *      所以只要这条发得出去 连接就不会因不闲着"被掐【 *
 *   3) 断线发现
 *      a. 模组主动把 +MQTTURC: "conn",0,<state> (2/3/4/5/6) →g4_urc 判离线→重建
 *      b. 发布失败计数: 连续 G4_PUB_FAIL_MAX 次发布失败→重建
 *      c. 【本段新增】保活停滞检测 在ONLINE 状态下, 只要"距上次成功发布
 *         超过 G4_HB_STALL_MS, 不管中间是什么原因 一律判链路已死并重建【 *
 * 【为什么必须有 c【 前两条都有盲化—— *   · 模组"假死"时它一条URC 都不发 a 永远不触发
 *   · b 要连续5 次失败 而心跳30s 一次 最均150s 才发现 更要命的是 *     如果 g4_at_mqtt_publish() 因为"上一条还没发完直接返回 SYS_ERR,
 *     连失败计数都不会消 【心跳可以永远静默地不再发出【 而日志里一个字都没有】 *   c 是纯时间判据, 对上面前两种失效模式都成立 也是现场唯一能一眼看出来的指标
 *   状态行里的 "HB=xs" 一直涨 = 保活断了。 */
/* 【时间兜底看门狗】距上次"发布成功"超过这么多毫秒 -> 不管什么原因, 一律判链路已死。
 *
 * ★2026-09 改成【运行时设定】, 由 main.c 按实际心跳周期算出来★
 *   原来是一个编译期宏且默认 0(关闭), 因为它有个坑: 没有周期性上行时必然误判。
 *   现在 main.c 在 g4_at_init() 之后调 g4_at_set_link_watchdog():
 *       心跳开着(MAIN_GW_ONLINE_ENABLE=1) -> 传 5 × 心跳周期(main.c 现设 30s×5 = 150s);
 *       心跳关着                          -> 传 0(彻底关掉, 不会误判)。
 *   这样"看门狗窗口"和"心跳周期"再也不会各说各话。
 *
 *   两条腿的分工:
 *     · 连续 G4_PUB_FAIL_MAX 次发布失败(现为 3) —— 快, 但要求"发布确实跑起来了";
 *     · 本看门狗(main.c 现按 5 × 心跳 = 150s 设)  —— 慢, 但连"发布根本没发出去/
 *       没登记"都能兜住, 而且窗口必须长于"模组自己重启 + 重连"的 40~60 秒, 否则
 *       它会变成又一个"自己拆自己链路"的来源(现场日志 78.txt)。 */
static uint32_t g_hb_stall_ms;              /* 0 = 关闭 */
static uint32_t g_last_pub_ok_tick;
static bool     g_last_pub_ok_valid;

/* ==================== 驻网(CEREG)只读观测 —— 2026-10-04 第 1 步 ====================
 *
 * 【这一步只做"看", 不做"判"】现场一直缺"网到底断没断"这一层证据: 现有判据
 * (conn URC / 连续发布失败 / 保活停滞)全在 MQTT 之上, 射频掉了但旧 IP 还在时
 * 一条都看不见 —— 实测要拖到 HB=119s 才发现(见 不做与偏差记录_v107 第五、六节)。
 *
 * 【做法: 靠模组主动上报, 而不是我们去轮询】
 *   ONLINE 之后下发一次 AT+CEREG=2 + AT+CGEREP=1, 之后模组自己会在驻网状态变化 /
 *   PS 去附着时吐 +CEREG: / +CGEV: —— g4_urc.c 记账并打原始行, 这里只管下发开关。
 *
 * ★为什么不按原计划"ONLINE 期间每 30s 轮询 AT+CEREG?"★
 *   本工程有一条【被主机测试钉死的不变式】: ONLINE 稳态下不许有任何多余的 AT
 *   (test_state.c 的 "ONLINE 稳态下 5 秒内发的 AT 命令数 = 0")。当初加这条是有
 *   代价换来的 —— 现场日志里 +MATREADY 出现 387 次, 【每一次都在我们发出字节后
 *   8~49ms 内】, 而且紧跟着 MQTT 会话就死了(第 1 条发布收下但不回 puback, 第 2 条
 *   起回 +CME ERROR: 601)。也就是说: 在 ONLINE 期间敲 AT, 本身就是把模组的 AT
 *   应用打到重启的诱因之一。
 *   所以"周期轮询"虽然读数直接, 却会亲手扰动被测对象 —— 对一个诊断功能来说这是
 *   最坏的失败方式(数据是假的, 而且把系统搞坏了)。
 *   改用"一次性开 URC + 被动接收": 开关只在【上线之前】发两条命令, ONLINE 期间
 *   我们一个字节都不发, 完全满足那条不变式。而且 URC 是边沿触发、立刻上报,
 *   比 30s 轮询更不容易漏掉短时掉网。
 *
 * 【开关必须在"上线之前"发】: ① 上面那条不变式; ② 手册要求 MQTTCFG 类命令在
 *    "MQTT 连接未创建" 时下发更稳。所以专门加了 G4_STATE_REG_ARM 状态, 位置在
 *   MQTT_SUB 成功之后、ONLINE 之前 —— 重启/快修之后每次都会重新走一遍
 *   (模组自己重启或整链路断电后, 这两个设置可能已被清掉)。
 *
 * 【兜底: 台架可选的轮询】G4_CEREG_POLL_MS > 0 时才在 ONLINE 里轮询 AT+CEREG?。
 *   默认 0(关闭) —— 打开它就会违反上面那条不变式, 只在台架上"想亲眼看一次
 *   轮询应答"时用, 不要带到现场。
 */
#ifndef G4_CEREG_DIAG_ENABLE
#define G4_CEREG_DIAG_ENABLE      1        /* 0 = 整个驻网观测不编译进去 */
#endif
#ifndef G4_CEREG_URC_ENABLE
#define G4_CEREG_URC_ENABLE       1        /* 1 = 上线前下发 CEREG=2 / CGEREP=1 收主动上报 */
#endif
#ifndef G4_CEREG_POLL_MS
#define G4_CEREG_POLL_MS          0u       /* 0 = 不轮询(默认); >0 = 台架轮询周期, 见上 */
#endif
#ifndef G4_CEREG_TMO_MS
#define G4_CEREG_TMO_MS           1000u    /* 单次轮询等应答上限(仅轮询模式用) */
#endif

/* 状态变量(g_cereg_inflight / g_cereg_next_tick / g_cereg)
 * 声明在文件靠前的"状态"块里 —— 因为 g4_goto_state() 要用它们。 */

/** @brief 驻网观测数据(只读; 见 g4_at.h) */
const g4_cereg_diag_t *g4_at_get_cereg_diag(void)
{
    /* 计数每次现取, 保证调用方拿到的总是最新值 */
    g_cereg.urc_line_cnt    = g4_urc_reg_line_count();
    g_cereg.cgev_cnt        = g4_urc_cgev_count();
    g_cereg.cgev_detach_cnt = g4_urc_cgev_detach_count();
    return &g_cereg;
}

/** @brief <stat> 的人话(3GPP TS 27.007); 供状态行/日志共用 */
const char *g4_at_reg_stat_text(uint8_t stat)
{
    return g4_reg_stat_text(stat);
}

#if (G4_CEREG_DIAG_ENABLE && (G4_CEREG_POLL_MS > 0u))
/**
 * @brief 现在能不能由驻网轮询占用 AT 命令引擎?(仅"台架轮询"模式用)
 *
 * 【为什么不能只看 g4_io_cmd_active()】引擎有 IDLE/RUNNING/DONE 三态, 而
 * g4_io_cmd_active() 只报 RUNNING —— DONE(命令已完成、结果还没被取走)会被
 * 误判成"空闲"。那时调 g4_io_cmd_run() 不会发送, 而是把【上一条命令的应答】
 * 交回来, 我们就会把它当成驻网数据 → 假数据比没数据更糟。
 * 所以这里显式排掉 DONE(排一次 poll: DONE→IDLE), 并留一行痕。
 *
 * @return true = 现在发命令一定是"我发的就是我读的"
 */
static bool g4_cereg_engine_available(void)
{
    if (g_pub_pending)
    {
        return false;               /* 有发布在途: 引擎归它 */
    }
    if (g4_io_cmd_idle())
    {
        return true;
    }
    if (g4_io_cmd_active())
    {
        return false;               /* 有在途命令: 等它做完 */
    }

    /* 只剩 DONE: 结果待取走。排掉它再抢 ——★2026-10-07 收紧★
     * 只有"无属主"的残结果才能丢。带属主的(现在只有 OTA)结果【只等它自己来取】,
     * 我们排掉就等于把它的 AT+MHTTPCFG 应答吞了 —— 它会以为命令没发出去。 */
    if (g4_io_cmd_owner() != G4_OWNER_NONE)
    {
        return false;               /* 结果归别的属主: 等它取走 */
    }
    (void)g4_io_cmd_poll();
    log_info("[CEREG] 排掉一处残留的命令结果(它不属于发布, 不影响链路)\r\n");
    return g4_io_cmd_idle();
}

/**
 * @brief 驻网轮询(仅当 G4_CEREG_POLL_MS > 0; 会违反"ONLINE 不发 AT"不变式)
 *
 * ⚠️ 默认不编译。打开前先读上面那段 ★为什么★ —— 它与本工程一条用主机测试钉死的
 *    不变式相冲突(ONLINE 稳态零 AT), 只适合台架上"想亲眼看一次应答"。
 */
static void g4_cereg_diag_poll(void)
{
    uint32_t     now = get_tick_ms();
    g4_cmd_res_t r;

    if (g_state != G4_STATE_ONLINE)
    {
        g_cereg_inflight  = false;
        g_cereg_next_tick = now;        /* 下次进 ONLINE 立刻查一次 */
        return;
    }

    /* ---- 在途: 推进并收结果 ---- */
    if (g_cereg_inflight)
    {
        r = g4_io_cmd_poll();
        if (r == G4_CMD_RUNNING)
        {
            return;
        }

        g_cereg_inflight  = false;
        g_cereg_next_tick = now + G4_CEREG_POLL_MS;

        if (g_cereg.poll_cnt < 0xFFFFFFFFu) { g_cereg.poll_cnt++; }

        if (r == G4_CMD_OK)
        {
            /* 轮询应答形态: +CEREG: <n>,<stat>[,...] —— at_reg_stat 取第二个数 */
            uint8_t stat = at_reg_stat(g4_io_cmd_resp(), "+CEREG:");

            if (stat != 0xFFu)
            {
                g_cereg.stat = stat;

                if ((stat == 1u) || (stat == 5u))
                {
                    g_cereg.bad_run = 0u;
                }
                else
                {
                    if (g_cereg.bad_run < 0xFFu)         { g_cereg.bad_run++; }
                    if (g_cereg.bad_total < 0xFFFFFFFFu) { g_cereg.bad_total++; }
                }
            }

            log_info("[CEREG] 台架轮询 #%lu: stat=%u (%s) 连续异常 %u 次"
                     "(这一步只记录, 不触发任何恢复动作)\r\n",
                     (unsigned long)g_cereg.poll_cnt,
                     (unsigned)g_cereg.stat,
                     g4_reg_stat_text(g_cereg.stat),
                     (unsigned)g_cereg.bad_run);
        }
        else
        {
            log_info("[CEREG] !! 台架轮询无应答(res=%d) —— 模组可能正在重启/忙"
                     "(这一步只记录)\r\n", (int)r);
        }
        return;
    }

    /* ---- 空闲: 能不能开一次新查询 ---- */
    if (!g4_cereg_engine_available())
    {
        return;
    }
    if ((int32_t)(now - g_cereg_next_tick) < 0)
    {
        return;
    }

    g_cereg_inflight = true;
    r = g4_io_cmd_run("AT+CEREG?", "+CEREG:", G4_CEREG_TMO_MS);
    if (r != G4_CMD_RUNNING)
    {
        /* 首拍就返回终态(极罕见) —— 不要把自己留在"在途"里 */
        g_cereg_inflight  = false;
        g_cereg_next_tick = now + G4_CEREG_POLL_MS;
    }
}
#else
/* 默认路径: 不轮询。ONLINE 期间一个字节都不发(不变式), 只被动收 URC。 */
static void g4_cereg_diag_poll(void)
{
    if (g_state != G4_STATE_ONLINE)
    {
        g_cereg_inflight = false;
    }
}
#endif

/**
 * @brief 设定"距上次发布成功多久算链路已死"(0 = 关闭这个判据)
 * @param max_silence_ms 建议 = 5 × 心跳周期(必须长于"模组重启+重连"的 40~60 秒);
 *                       没有周期心跳时必须传 0
 */
void g4_at_set_link_watchdog(uint32_t max_silence_ms)
{
    g_hb_stall_ms = max_silence_ms;
    log_info("[G4] 保活看门狗: %s\r\n",
             (max_silence_ms == 0u) ? "关闭"
                                    : "距上次发布成功超过该值即判链路断");
    if (max_silence_ms != 0u)
    {
        log_info("[G4]   (窗口 = %lus; 超过就走 4G_RST 断电重启模组)\r\n",
                 (unsigned long)(max_silence_ms / 1000u));
    }
}

/** @brief 距上次发布成功"过了多少毫秒(0xFFFFFFFF = 还没成功这 */
uint32_t g4_at_pub_age_ms(void)
{
    if (!g_last_pub_ok_valid)
    {
        return 0xFFFFFFFFu;
    }
    return (uint32_t)(get_tick_ms() - g_last_pub_ok_tick);
}

/** @brief 累计"等 '>' 时提示符丢失"的次数(诊断用; 处理策略见文件上方说明)
 *
 * >0 就说明模组确实发生过"回 OK 却不给 '>'": 配合 g4_io 那行
 * "[G4] ** 模组报出 \"+MATREADY\" ..." 一起看, 就能分清是"它自己重启了"
 * 还是"只是提示符被吞了"。 */
uint32_t g4_at_prompt_loss_count(void)
{
    return g_pub_prompt_loss_total;
}

/** @brief 记一次发布成功"(保活停滞检测的计时基准) */
static void g4_pub_mark_ok(void)
{
    g_last_pub_ok_tick  = get_tick_ms();
    g_last_pub_ok_valid = true;
}

/** @brief 模组射频功能档位(建链时读的 AT+CFUN?); 见 g4_at.h 的说明 */
int8_t g4_at_get_cfun(void)
{
    return g_cfun;
}

/* ==================== 模组重启检测====================
 *
 * 【为什么必须专门做这件事【 现场日志里出现过这样一段—— *     [G4_TX] AT+CEREG?
 *     [G4_RX] +MATREADY          →这是模组"起动完成"的URC!
 *     [G4_RX] OK
 *     [G4_TIMEOUT] cmd="AT+CEREG?"
 *     [G4] AT+CEREG? unsupported, next
 * 也就是说: 我们问它注册状态 它回了一台我刚启动完 ——模组在那时候【重启了】。 * 而原来的代码把这条无声无息地扔掉, 于是现场看到的只是某条命令没应答, 很容是 * 往"命令不支持"网络不好"方向查 越查越远【 *
 * "+MATREADY"/"RDY" 一次上电只该出现一次。第二次出现 = 模组重启过。这时候
 *   · 手里的建链进度、在途的 AT 命令、没发完的发布【全部作废【
 *   · 继续对它灌AT 只会打扰它起动实测: 模组会一直不起好);
 *   · 正确动作是回到预热窗口 老老实实等它把 +MATREADY 打完再重新握手【 */
static uint32_t s_ready_urc_seen = 0u;

/**
 * @brief 检测模组是否【真的】重启过; 是则作废链路并回到预热窗口
 * @return true = 检测到重启并已重置状态(调用方本次应直接返回)
 *
 * ⚠️ 【这个判据踩过大坑, 现在只认"无缘无故"的那一类】
 *   现场实测这块模组(ML307Y)会【在每条 AT 命令的应答里再报一次 +MATREADY】:
 *       发 "AT"   →  "\r\n+MATREADY\r\nOK\r\n"
 *       发 "ATE0" →  又一条 +MATREADY
 *   一次开机能攒到几十次(现场日志里已经数到第 24、25 次)。早先按"第 2 次
 *   +MATREADY = 重启"判, 结果: 刚 AT_SYNC OK → 收到第 2 次 → 判重启 → 作废 →
 *   回 WAIT_MODULE → 又敲 → 又"重启"…… 【无限循环, 链路永远建不起来】。
 *   所以现在:
 *     · 紧跟在我们发出的命令后面的那批 → g4_io 已经单独归类, 不计入重启计数;
 *     · WAIT_MODULE / AT_SYNC 期间本来就在等它起动 → 收到多少次都不算重启;
 *     · 只有"我们安静着, 它自己冒出来"的才判重启。
 */
static bool g4_check_module_reboot(void)
{
    uint32_t cnt    = g4_io_ready_urc_count();
    uint32_t in_tx  = g4_io_ready_after_tx_count();

    if (cnt == s_ready_urc_seen)
    {
        return false;
    }
    s_ready_urc_seen = cnt;

    /* 【正在等它起动】的阶段收到 +MATREADY 是好事, 不是重启 */
    if ((g_state == G4_STATE_WAIT_MODULE) || (g_state == G4_STATE_AT_SYNC))
    {
        return false;
    }

    log_info("\r\n[G4] !! 模组【自己】报了一次 \"+MATREADY\"/\"RDY\""
             "(无缘无故累计 %lu 次; 另有 %lu 次是紧跟命令后面的, 那些是它的应答习惯)\r\n",
             (unsigned long)cnt, (unsigned long)in_tx);
    log_info("[G4] !! 常见原因: 供电塌(发射瞬间压降) / 模组过热保护 / 看门狗自复位\r\n");
    log_info("[G4] !! 作废这条链路, 回到静听窗口重新等它起动(不再灌 AT)\r\n\r\n");

    g_pub_state   = PUB_IDLE;       /* 没发完的发布作废 */
    g_pub_pending = false;
    g4_io_drain_rx();
    g4_io_rx_reset();               /* 字节计数按"本次启动"重新算 */
    g4_goto_state(G4_STATE_WAIT_MODULE);
    return true;
}

/**
 * @brief 发布失败【本机类】: 放弃这一条, 但【不计入】链路失败计数
 *
 * 用于"命令组装失败 / 命令引擎被占用 / 串口写失败"这类跟模组和网络无关的问题。
 * 这类问题如果也去计数, 一次代码 bug 就会变成"一直给模组断电", 那才是灾难。
 */
static void g4_pub_abort(const char *why)
{
    g_pub_pending = false;
    log_info("[G4] 发布放弃(%s) —— 本机/组装问题, 不计入链路失败计数\r\n", why);
    (void)g4_pub_queue_next();      /* 队列里还有的接着发 */
}

/**
 * @brief 发布失败【链路类】: 计数, 连续 G4_PUB_FAIL_MAX 次就判链路已死
 *
 * ★判定已经收紧: 连续 G4_PUB_FAIL_MAX(现为 3)次就断电重启★ —— 也就是"连着几次
 * 心跳上报都失败"。
 * 注意判定顺序: 【先判阈值, 再排空队列】。老写法是"队列里还有就先去发下一条、
 * 直接 return", 于是队列非空时这个阈值永远不会被评估 —— 云端连发几条时,
 * 失败可以积累一堆却不触发恢复。现在计数严格按"连续失败"来。
 */
static void g4_pub_mark_fail(const char *why)
{
    g_pub_pending     = false;
    g_pub_prompt_loss = 0u;     /* 换了失败类型: 连续丢 '>' 的计数从头算 */
    if (g_pub_fail_cnt < 0xFFU)
    {
        g_pub_fail_cnt++;
    }

    log_info("[G4] PUB fail(%s) —— 连续 %u/%u 次\r\n",
             why, (unsigned)g_pub_fail_cnt, (unsigned)G4_PUB_FAIL_MAX);

    if (g_pub_fail_cnt >= G4_PUB_FAIL_MAX)
    {
        log_info("[G4] !! 连续 %u 次发布失败 -> 判链路已断(模块收不到也发不出)\r\n",
                 (unsigned)g_pub_fail_cnt);
        g_pub_fail_cnt = 0U;
        g4_link_recover("连续发布失败(疑似断网/模组假死)");
        return;
    }

    /* 还没到阈值: 队列里还有的接着发(不然云端连发时后面的回执又全丢) */
    (void)g4_pub_queue_next();
}

/* ==================== MQTT 会话"快修"(只重连 MQTT, 不断电模组) ====================
 *
 * 【为什么必须单独有这一条】现场日志 78.txt 的铁证 —— 每个 MQTT 会话形态固定:
 *     CONNECT 成功 + SUBSCRIBE 成功(suback 收到)
 *     -> 第 1 条 MQTTPUB 被模组收下("+MQTTPUB: 0,<mid>,29/60" + OK),
 *        但 broker 【永远不回 puback】, 云端一条也收不到;
 *     -> 从第 2 条起 AT+MQTTPUB 直接回 "+CME ERROR: 601"(= MQTT 未连接)。
 *   也就是说: 【一发 PUBLISH, 这个 MQTT 会话就死了】。而模组【不发任何 conn URC】,
 *   状态机于是一直以为还在线, 继续往一个死会话里灌数据。
 *
 *   老代码对这种情况只有一条出路: 连续 G4_PUB_FAIL_MAX 次发布失败 -> 4G_RST
 *   断电重启【整个模组】—— 代价 40~60 秒。可要修的只是【MQTT 这一层】:
 *   MQTTDISC -> MQTTCONN -> MQTTSUB, 2~5 秒, 完全不用碰 PA8。
 *
 * 升级策略: 连续 G4_MQTT_FAST_MAX 次快修都没救回来 -> 才交给 g4_link_recover()
 *          (断电重启), 因为那说明问题不只在 MQTT 层。
 *
 * 触发点(三个, 都来自现场日志):
 *   a) AT+MQTTPUB 回 "+CME ERROR: 601"          -> MQTT 会话已经没了
 *   b) 连续 G4_PUB_NOACK_MAX 条发布拿不到 puback -> 云端根本没收到(判据只认 puback)
 *   c) ONLINE 期间模组报了 "+MATREADY"           -> 会话可疑, 下次失败就走快修
 */
#ifndef G4_MQTT_FAST_MAX
#define G4_MQTT_FAST_MAX  3U
#endif

/* ★★2026-10-08 新增: 快修路径专用的 WAIT_CONN 超时★★
 *
 * 【为什么必须比 AT_URC_TIMEOUT_MS 短得多】
 *   AT_URC_TIMEOUT_MS = 60000(60 秒) —— 那是给【冷启动建链】用的:
 *   模组要搜网+注册+激活 PDP+DNS, 60 秒是合理的。
 *   但【快修】的前提是"驻网和 PDP 都还在, 只重建 MQTT" —— 它本来就该 2~5 秒。
 *
 * 【现场(2026-10-09 78.txt)】模组欠压重启(+MATREADY)后 MQTT 栈处于坏状态:
 *     [G4_TX] AT+MQTTCONN=0,...        -> [G4_RX] OK
 *     [G4] -> WAIT_CONN                 -> 之后【60 秒一条 AT 都没发】
 *   模组对 MQTTCONN 回 OK, 但【既不报连上也不报失败】—— 快修白等 60 秒才升级。
 *   而 G4_MQTT_FAST_MAX=3 ⇒ 最坏 3 × 60 = 180 秒(3 分钟)才轮到断电重启。
 *
 * 【8 秒怎么定的】快修正常只要 2~5 秒; 8 秒已经给了 60% 以上余量,
 *   拿到不到 conn URC 就说明"不是慢, 是坏了"。 */
#ifndef AT_URC_TIMEOUT_FAST_MS
#define AT_URC_TIMEOUT_FAST_MS   8000u
#endif

/* true = 当前这一轮建链是【快修】发起的(不是冷启动)。
 * 用途只有一个: 让 G4_STATE_WAIT_CONN 用 8 秒而不是 60 秒判超时。
 * 置位: g4_mqtt_fast_rebuild();  清零: MQTT SUB 成功(说明快修确实救回来了)。 */
static bool g_mqtt_fast_active;
/* 连续多少条"模组收下了但 broker 不回 puback"才判定会话已死。
 * 取 2 是给"puback 偶尔慢一点"留一次余量。 */
#ifndef G4_PUB_NOACK_MAX
#define G4_PUB_NOACK_MAX  2U
#endif

static uint8_t  g_mqtt_fast_cnt;      /* 连续走了几次快修还没回到 ONLINE */
static bool     g_mqtt_suspect;       /* ONLINE 期间看到 +MATREADY -> 会话可疑 */
static uint8_t  g_pub_noack_cnt;      /* 连续"收下了但没 puback"的次数 */
static uint32_t g_mqtt_fast_total;    /* 累计快修次数(诊断/STAT2) */

/** @brief 累计走了多少次 MQTT 快修(诊断用) */
uint32_t g4_at_mqtt_fast_count(void)
{
    return g_mqtt_fast_total;
}

static void g4_mqtt_fast_rebuild(const char *why)
{
    if (g_mqtt_fast_cnt < 0xFFu)   { g_mqtt_fast_cnt++; }
    if (g_mqtt_fast_total < 0xFFFFFFFFu) { g_mqtt_fast_total++; }

    log_info("[G4] !! %s\r\n", why);

    /* 【旁证】: 如果 ONLINE 期间模组报过 +MATREADY(AT 应用层重启过), 那这条会话
     * 多半就是那么死的 —— 这是解释"为什么突然 601"最有用的一句话, 值得单独打。
     * (它【不】作为触发条件: 判"链路该不该重建"只看 601/conn URC, 不看这个计数 ——
     *  这块模组会把 +MATREADY 附在几乎每条应答后面, 拿它当判据会草木皆兵。) */
    if (g_mqtt_suspect)
    {
        log_info("[G4]    (旁证: ONLINE 期间模组报过 +MATREADY —— AT 应用层重启过)\r\n");
    }

    if (g_mqtt_fast_cnt > G4_MQTT_FAST_MAX)
    {
        log_info("[G4] !! 连续 %u 次 MQTT 快修都没救回来 -> 交给整链路恢复(断电重启)\r\n",
                 (unsigned)g_mqtt_fast_cnt);
        g_mqtt_fast_cnt = 0u;
        g_mqtt_suspect  = false;
        g4_link_recover("MQTT 快修连续失败");
        return;
    }

    log_info("[G4] -> 【MQTT 快修 #%u】只重连 MQTT(MQTTDISC->MQTTCONN->MQTTSUB), "
             "【不断电模组】—— 预计 2~5 秒(整链路恢复要 40~60 秒)\r\n",
             (unsigned)g_mqtt_fast_cnt);

    /* 把 MQTT 侧状态全部作废, 免得"上一轮的在线标志"被当成成功(见 WAIT_CONN 的注释) */
    g4_urc_set_offline();
    g4_urc_reset_conn_state();
    /* 【这里不再单独下发遗嘱】: 下面走的 MQTT_DISC -> MQTT_CFG -> MQTT_CONN 这条链
     * 里, G4_STATE_MQTT_CFG 会把【整套】模组级参数(遗嘱/keepalive/pingreq/pingresp/
     * reconn)重下发一遍 —— 模组掉电不保存这些设置, 每轮建链都得补。 */
    g_pub_fail_cnt  = 0U;
    g_pub_noack_cnt = 0u;
    g_mqtt_suspect  = false;
    /* ⚠️ 【不动 g_pub_pending】: 这次没发出去的那条会停在槽位里, 等状态机回到
     * ONLINE 后由 pub_poll() 接着发(见那里的"不在线就不发")—— 数据不丢。 */
    /* ★2026-10-08: 标记"这轮建链是快修发起的" —— WAIT_CONN 据此用 8s 而不是 60s 判超时 */
    g_mqtt_fast_active = true;
    g4_goto_state(G4_STATE_MQTT_DISC);
}

/* ==================== MQTT 发布子流程(非阻塞三阶段) ====================
 *
 * 老版本是在这里 while(g4_io_read_line()) 自己抢字节流等 '>' / OK:
 *   · 期间 URC 扫描被整个跳过 → 云端下行没人解析, payload 被"按行"读掉;
 *   · 用 strchr(line,'>') 判提示 → 下行二进制里任意一个 0x3E 都会误触发, 于是在
 *     模组还没进数据模式时就把 payload 当 AT 数据发出去 → 被模组当命令解析,
 *     下一条 AT+MQTTPUB 和这堆垃圾粘在一起 → ERROR / AT 引擎乱掉 → 掉线;
 *   · 每次都要等满 15s, 主循环被这条子流程整个占住。
 *
 * 现在它只是状态机的一个普通子流程, 走统一的非阻塞命令引擎:
 *   PUB_CMD_SENT  → 引擎等【严格整行】的 '>' 提示
 *   PUB_DATA_SENT → 发 payload, 等模组的 OK
 *   PUB_WAIT_ACK  → 再给 broker 的 puback 一点时间(端到端确认)
 * 期间收包泵照常跑, 云端下行照常入队, 不再互相踩。
 */

static char     g_pub_cmd[112];     /* 正在发的这条发布的 AT 命令 */
static uint32_t g_pub_ack_base;     /* 本次事务开始时的 puback 计数 */

/* ★2026-10-07: "在命令引擎门口排了多久"★
 * 引擎被别的属主(目前只有 ota_http)占着时, 我们这条 AT+MQTTPUB 【一个字节都没发出去】。
 * 那种情况绝不能算"等 '>' 超时"(现场实测就是这么把链路判死、给模组断电的),
 * 但也不能无限等 —— 单开一个计时, 只用来"别被一条卡死的 OTA 永久饿死 MQTT"。
 * 只影响这一条发布(走 g4_pub_abort, 不计链路失败), 见 pub_poll 的 PUB_CMD_SENT。 */
#ifndef G4_PUB_ENGINE_WAIT_MAX_MS
#define G4_PUB_ENGINE_WAIT_MAX_MS  30000u
#endif
static uint32_t g_pub_engine_wait_tick;

/* 【等 '>' 的时间必须给够 —— 现场就是死在这里】
 * '>' 本该是模组【本地】立刻回的提示, 但实测 ML307Y 有时要 10 秒才吐出来
 * (网络忙 / 刚订完主题时更慢)。原来只等 1s, 于是: 发布超时 -> 失败计数+1 ->
 * 连续 5 次 -> 判链路死 -> 重建 -> 【连接每 90 秒被拆一次, 云端下发自然收不到】。 */
#ifndef G4_PUB_PROMPT_TIMEOUT_MS
#define G4_PUB_PROMPT_TIMEOUT_MS 15000u
#endif
/* 数据交给模组(OK)之后, 最多再等这么久等 broker 的 puback。
 * ★2026-10-04: 2000 -> 5000★ 现场日志里"发布未获 broker 确认"占了很大比例, 而
 * 同一批发布在云平台上是【收到了】的(说明 broker 确实收到并回了 puback, 只是我们
 * 2 秒窗口太短就放弃了)。4G 公网 + 公共 broker 的 RTT 完全可能 >2s。 */
#ifndef G4_PUB_ACK_WAIT_MS
#define G4_PUB_ACK_WAIT_MS       5000u
#endif

/** @brief 组装发布命令 + 记录本次事务的 puback 基准(真正发送在 PUB_CMD_SENT 里做) */
static void pub_start(void)
{
    int n;

    /* AT+MQTTPUB=<id>,<topic>,<qos>,<retain>,<dup>,<msg_len>
     * 省略 <message> 进入数据模式, 模组返回 '>' 提示后发送 msg_len 字节。
     * QoS/retain 现在是逐条携带的(g_pub_qos / g_pub_retain), 不再是写死的宏。
     * 参数顺序务必用 "#at AT+MQTTPUB=?" 核对一次。 */
    n = snprintf(g_pub_cmd, sizeof(g_pub_cmd), "AT+MQTTPUB=%s,\"%s\",%u,%u,0,%u",
                 G4_MQTT_CONNECT_ID, g_pub_topic,
                 (unsigned)g_pub_qos, (unsigned)g_pub_retain, (unsigned)g_pub_len);
    if ((n <= 0) || (n >= (int)sizeof(g_pub_cmd)))
    {
        g4_pub_abort("命令组装失败");
        return;
    }

    log_info("[G4] 发布 payload(%u 字节, qos=%u, retain=%u): %.*s\r\n",
             (unsigned)g_pub_len, (unsigned)g_pub_qos, (unsigned)g_pub_retain,
             (int)((g_pub_len < 200u) ? g_pub_len : 200u), (const char *)g_pub_data);

    g_pub_state    = PUB_CMD_SENT;
    g_state_tick   = get_tick_ms();
    g_pub_engine_wait_tick = g_state_tick;   /* ★排队计时从这一刻起★ */
}

/** @brief 发布事务收尾: 成功(并区分"有没有拿到 broker 确认") */
static void pub_finish_ok(const char *how)
{
    log_info("[G4] 发布完成(%s), 距上次成功 %lus\r\n",
             how,
             (g4_at_pub_age_ms() == 0xFFFFFFFFu)
                 ? 0UL : (unsigned long)(g4_at_pub_age_ms() / 1000u));
    g_pub_fail_cnt    = 0U;
    g_pub_prompt_loss = 0u;         /* 发出去了: 连续丢 '>' 的计数清零 */
    g_pub_noack_cnt   = 0u;         /* 而且拿到了 puback: "云端未确认"的计数也清零 */
    g_mqtt_suspect    = false;      /* 端到端确认过了, 会话不再可疑 */
    g4_pub_mark_ok();
    g_pub_pending = false;
    (void)g4_pub_queue_next();      /* 队列里还有就接着发 */
}

/* 发布子流程: 返回 true 表示"这一条还在进行中" */
static bool pub_poll(void)
{
    g4_cmd_res_t r;

    if (!g_pub_pending)
    {
        return false;
    }

    /* 【与驻网(CEREG)轮询互斥】: 两条都走【同一个】AT 命令引擎。
     * 轮询在途时让这条发布等一拍 —— 绝不能去抢应答: 那会把诊断轮询的应答
     * (比如一个 OK)判成"发布成功"(同 payload 发出后那段注释里警告的坑)。
     * 轮询最长只占 G4_CEREG_TMO_MS(1s), 而且只在没有发布在途时才开。
     * ⚠️ 默认 G4_CEREG_POLL_MS=0(不轮询), 所以这道闸门在出厂固件里恒不生效;
     *    它是给台架打开轮询时兜底用的(见 g4_cereg_diag_poll 的说明)。 */
    if (g_cereg_inflight)
    {
        return false;
    }

    /* 【不在线就不发】—— 重建链路期间(MQTT_DISC/CONN/SUB 或更早)命令引擎归状态机用,
     * 这里绝不能插队去抢模组的应答。所以那条没发出去的发布【停在槽位里等着】,
     * 等状态机回到 ONLINE 再接着发 —— 这样"MQTT 快修"期间的数据不会丢。
     * (MQTT 快修就是从 pub_poll 里触发的, 所以这个判断也是它能安全返回的前提。) */
    if (g_state != G4_STATE_ONLINE)
    {
        return false;
    }

    if (g_pub_state == PUB_IDLE)
    {
        pub_start();
        return g_pub_pending;
    }

    /* ---- 收到 broker 的 puback 才算端到端成功 ----
     * 【基准必须在"我们的字节真的上了线"之后取】: puback 只可能是 broker 收到
     * 本次 PUBLISH 之后才回来的, 所以从 send_all 那一刻起的任何 puback 都属于本条。
     * 老写法在 pub_start()(还没发一个字节)就取基准, 于是一条【上一条的迟到确认】
     * 会被算成本条成功 —— 假成功, 而且会把真正丢失的那条永久掩盖掉。
     * 可认账的状态也一并收窄到"payload 已发出"之后的两态。 */
    if (((g_pub_state == PUB_DATA_SENT) || (g_pub_state == PUB_WAIT_ACK)) &&
        (g4_urc_puback_count() != g_pub_ack_base))
    {
        g4_io_cmd_abort();
        pub_finish_ok("broker 已 puback, 云端确实收到了");
        return false;
    }

    switch (g_pub_state)
    {
    case PUB_CMD_SENT:
        r = g4_io_cmd_run_prompt(g_pub_cmd, G4_PUB_PROMPT_TIMEOUT_MS);
        if (r == G4_CMD_RUNNING)
        {
            /* ★2026-10-07: 排队【不算】超时★
             *
             * 【现场证据】log/新建 文本文档.txt:
             *   15:40:03.809  OTA 发 AT+MHTTPCFG="ssl",0,1,0(超时 10s)
             *   15:40:03.828  我们进来发 AT+MQTTPUB —— 但引擎被 OTA 占着,
             *                 【这条命令一个字节都没发出去】(同一段时间里没有任何
             *                 [G4_TX] AT+MQTTPUB= 日志)
             *   15:40:13.814  PUB fail(等 '>' 超时) —— 正好 03.809+10.005s,
             *                 也就是我们拿到了【OTA 那条命令的】TIMEOUT 当成自己的失败
             *   15:40:28/43   再来两次 -> 连续 3 次 -> 判"链路已断" -> 4G_RST
             *                 -> 正在配 HTTP 实例的 OTA 收到 +CME ERROR: 652 -> 失败 2002
             *
             * 属主保护(g4_io.h)已经把"偷别人的 TIMEOUT"堵死了, 这里只剩两种情况:
             *   · 引擎被别人占着 -> 我们还在排队, 只单独记"排了多久"防饿死, 不判失败
             *   · 引擎上是我们的命令 -> 交给 g4_io 自己的超时判定, 逻辑一字不变
             */
            if (g4_io_cmd_owner() != G4_OWNER_NONE)
            {
                if ((uint32_t)(get_tick_ms() - g_pub_engine_wait_tick)
                        >= G4_PUB_ENGINE_WAIT_MAX_MS)
                {
                    /* 只这一条发布放弃, 【不计入链路失败计数】—— 这是"本机排队",
                     * 跟模组和网络都没关系, 绝不能拿它去给模组断电。
                     * (见 g4_pub_abort() 的说明) */
                    g4_pub_abort("命令引擎被外部流程(OTA)长时间占用");
                    return g_pub_pending;
                }
            }
            else
            {
                g_pub_engine_wait_tick = get_tick_ms();     /* 轮到自己了: 重新起算 */
            }
            return true;
        }

        g_pub_engine_wait_tick = get_tick_ms();
        /* 【提示符 '>' 丢失 —— 先探活, 再决定要不要断电】(完整论证见文件上方
         *  "'>' 提示符丢失的处理策略")
         *
         * 现场实测(日志 78.txt): 发 AT+MQTTPUB 之后模组回 "+MATREADY" + "OK",
         * '>' 被吞了。老代码据此判"AT 状态不可信", 立刻给模组断电 —— 但那要
         * 40~60 秒才回得来, 而模组往往只是自己重启了一下(它给的那行 OK 说明
         * AT 引擎已经回来了)。所以现在先敲一条 AT 确认引擎还在不在。 */
        if (r == G4_CMD_NOPROMPT)
        {
            if (g_pub_prompt_loss_total < 0xFFFFFFFFu) { g_pub_prompt_loss_total++; }
            if (g_pub_prompt_loss < 0xFFu)             { g_pub_prompt_loss++; }

            if (g_pub_prompt_loss >= G4_PUB_PROMPT_LOSS_MAX)
            {
                /* ★★2026-10-08: 到阈值【不再给模组断电】, 改为放弃这一条发布★★
                 *
                 * 【为什么改】现场 22.5 分钟的统计:
                 *     '>'丢失重发 = 13    MQTT快修 = 13    模组报就绪(+MATREADY) = 34
                 *   13 次重发与 13 次快修几乎 1:1 —— 每次"探活+重发同一条包"都在
                 *   重新触发一次模组重启。而断电重启要 40~60 秒才回来, 比"丢一条"
                 *   的代价大两个数量级。
                 * ⇒ 同一条包【最多重发 1 次】(阈值=2); 再失败就认赔:
                 *   放弃这一条, 记录一次发布失败, 【不碰链路】。
                 *   链路真有问题的话, 发布失败计数/保活看门狗那两条腿会兜住,
                 *   不需要这里抢着断电。
                 *
                 * 【为什么不干脆不重发】'>' 丢失有时确实是瞬时的(模组刚重启完,
                 *   它的 AT 引擎已经回来了, 见上面那行 OK 的注释) —— 探活+重发 1 次
                 *   能救回相当一部分, 试一次是划算的。 */
                log_info("[G4] !! '>' 连续丢失 %u 次(>= %u) —— 本条发布【放弃】"
                         "(不重发、不断电): 链路交由失败计数/保活看门狗兜底\r\n",
                         (unsigned)g_pub_prompt_loss, (unsigned)G4_PUB_PROMPT_LOSS_MAX);
                g_pub_prompt_loss = 0u;
                g4_pub_abort("'>' 连续丢失, 放弃本条(不硬发)");
                return g_pub_pending;
            }

            if (!g4_io_cmd_idle())
            {
                /* ★2026-10-07★ 引擎被【外部属主】(OTA)占着 = 我们只是在排队,
                 * 跟模组状态没关系 —— 绝不能因此给模组断电(那就是现场那条
                 * "OTA 正在配实例 -> 我们断电 -> 652 -> 失败 2002" 的成因)。
                 * g4_pub_abort 不计链路失败计数。 */
                if (g4_io_cmd_owner() != G4_OWNER_NONE)
                {
                    g4_pub_abort("'>' 丢失后引擎被外部流程(OTA)占用");
                    return g_pub_pending;
                }
                /* 老路径不变: 无属主的命令卡着 = 引擎状态不对, 按老办法断电重启。
                 * 理论上到不了这里(上一条命令的结果已经被取走)。 */
                log_info("[G4] !! 探活前发现命令引擎仍被占用 —— 按老办法断电重启\r\n");
                g_pub_pending     = false;
                g_pub_prompt_loss = 0u;
                g4_link_recover_ex("'>' 丢失且命令引擎被占用", true);
                return false;
            }

            log_info("[G4] '>' 丢失(第 %u/%u 次, 累计 %lu 次): 模组回的是 OK => 它还在"
                     "命令模式, 并没有在等 payload -> 敲一条 AT 探活, 活着就【重发本条】"
                     "(不再直接断电)\r\n",
                     (unsigned)g_pub_prompt_loss, (unsigned)G4_PUB_PROMPT_LOSS_MAX,
                     (unsigned long)g_pub_prompt_loss_total);

            g4_io_cmd_begin("AT", "OK", AT_CMD_TIMEOUT_MS);
            g_pub_state  = PUB_PROBE;
            g_state_tick = get_tick_ms();
            return true;
        }

        if (r != G4_CMD_OK)
        {
            /* 【+CME ERROR: 601 = MQTT 未连接】—— 这不是"这一条发布失败了", 而是
             * "整个 MQTT 会话已经没了"。现场日志 78.txt: 每个会话从第 2 条 MQTTPUB
             * 起全是 601。对症的修法是重连 MQTT(2~5 秒), 不是攒 3 次去给模组断电。 */
            if ((r == G4_CMD_ERR) && (strstr(g4_io_cmd_resp(), "601") != NULL))
            {
                g4_mqtt_fast_rebuild("AT+MQTTPUB 回 +CME ERROR: 601 —— MQTT 会话已经没了");
                return g_pub_pending;
            }
            g4_pub_mark_fail((r == G4_CMD_ERR) ? "模组回 ERROR(命令头)" : "等 '>' 超时");
            return g_pub_pending;
        }

        /* 收到 '>' 提示(严格整行判定), 发原始 payload */
        if (!g4_io_send_all(g_pub_data, g_pub_len))
        {
            g4_pub_abort("payload 写串口失败");
            return g_pub_pending;
        }
        /* ★puback 计数基准在这里取★: 字节已经上线, 此后任何 puback 都属于这一条
         * (在 pub_start 里取就太早了, 会把上一条的迟到确认算到这一条头上)。 */
        g_pub_ack_base = g4_urc_puback_count();
        /* 【立刻开等, 不要等下一拍】: 模组的 OK 可能与 payload 几乎同时到。
         * ⚠️ 开之前必须确认引擎【真正空闲】: 万一被别的阻塞调用(如 #at / CCLK / OTA)
         *    占着, begin 会静默失败, 之后 g4_io_cmd_poll() 返回的就是【别人的】结果 ——
         *    那会把一次失败的发布判成成功。
         * ★2026-10-07: 判据从 g4_io_cmd_active()(只报 RUNNING) 换成 g4_io_cmd_idle()
         *   —— DONE(别人的结果还没取走)同样不能碰, 否则我们会把它的应答当成自己的。 */
        if (!g4_io_cmd_idle())
        {
            g4_pub_abort("命令引擎被占用(payload 已发出)");
            return g_pub_pending;
        }
        g4_io_cmd_begin(NULL, "OK", G4_PUB_PROMPT_TIMEOUT_MS);
        g_pub_state  = PUB_DATA_SENT;
        g_state_tick = get_tick_ms();
        return true;

    case PUB_PROBE:
    {
        /* '>' 丢失之后的探活: 只问一件事 —— "AT 引擎还在吗" */
        r = g4_io_cmd_poll();
        if (r == G4_CMD_RUNNING)
        {
            return true;
        }

        if (r == G4_CMD_OK)
        {
            /* AT 有 OK = 引擎活着, 而且此刻就在命令模式 -> 用同一份 payload 重发。
             * 回到 PUB_IDLE 会让 pub_start() 重新组装同一条命令(数据槽并没被清过)。 */
            log_info("[G4] AT 探活 OK —— AT 引擎正常, 【重发本条发布】"
                     "(累计丢 '>' %lu 次, 本次已重发 %u 次)\r\n",
                     (unsigned long)g_pub_prompt_loss_total,
                     (unsigned)g_pub_prompt_loss);
            g_pub_state  = PUB_IDLE;
            g_state_tick = get_tick_ms();
            return true;
        }

        /* AT 都不应答(超时/ERROR): 这才是真的"AT 通道不可信" -> 断电重启 */
        log_info("[G4] !! AT 探活都没应答(res=%d) —— AT 引擎确实不在了, 断电重启\r\n",
                 (int)r);
        g_pub_pending     = false;
        g_pub_prompt_loss = 0u;
        g4_link_recover_ex("探活 AT 无应答(AT 引擎不可信)", true);
        return false;
    }

    case PUB_DATA_SENT:
        r = g4_io_cmd_poll();
        if (r == G4_CMD_RUNNING)
        {
            return true;
        }
        if (r == G4_CMD_ERR)
        {
            /* payload 已经发出去之后才回 601: 同样说明 MQTT 会话没了(见上一条注释) */
            if (strstr(g4_io_cmd_resp(), "601") != NULL)
            {
                g4_mqtt_fast_rebuild("MQTTPUB(payload 已发) 回 +CME ERROR: 601 —— "
                                     "MQTT 会话已经没了");
                return g_pub_pending;
            }
            g4_pub_mark_fail("模组回ERROR(数据)");
            return g_pub_pending;
        }
        if (r != G4_CMD_OK)
        {
            g4_pub_mark_fail("等 OK 超时");
            return g_pub_pending;
        }
        /* 模组已把数据交给它的 MQTT 栈。QoS 1 下 broker 还会回 puback,
         * 再给它 G4_PUB_ACK_WAIT_MS —— 那才是"云端确实收到了"的证据。 */
        log_info("[G4] 模组已接收数据, 等 broker 确认(puback)...\r\n");
        g_wait_tick = get_tick_ms();
        g_pub_state = PUB_WAIT_ACK;
        return true;

    case PUB_WAIT_ACK:
        if ((uint32_t)(get_tick_ms() - g_wait_tick) < G4_PUB_ACK_WAIT_MS)
        {
            return true;
        }
        /* 【没等到 puback 不能再算成功 —— 这就是"云端收不到"的藏身处】
         * 现场日志 78.txt: 整份日志 puback=0, 而日志里一排
         *     "发布完成(模组已接收, 但未等到 puback...)"
         * 老代码在这个分支调 pub_finish_ok(), 于是它【清零失败计数、重置 HB 计时】——
         * 设备自我感觉发布成功了, 云端其实一条都没收到, 而且永远不会触发任何恢复。
         * 所以: 不算成功, 计数 + 大声报。
         *
         * ★2026-10-04 第 2 步: 【不再因此动链路】★
         *   用户口径(不做与偏差记录_v107 第六节 问题2 第③条): 驻网正常 + 会话在,
         *   但拿不到 puback -> 判"送达失败", 它死在【模组 MQTT 栈 ↔ broker】之间,
         *   既不是我们的代码也不是 AT 链路; 换 broker/看平台才能定位。
         *   快修改变不了那一段, 断电更改变不了 —— 只会白折腾一轮并把恢复时间拉长。
         *
         *   那"会话真的死了"靠什么发现? 靠【硬证据】: 下一条发布如果回
         *   +CME ERROR: 601(= 没有活的 MQTT 会话), 上面 PUB_CMD_SENT/PUB_DATA_SENT
         *   的分支就会走 MQTT 快修。所以这里只记账完全够用, 不会漏掉真死。 */
        if (g_pub_noack_cnt < 0xFFu) { g_pub_noack_cnt++; }

        log_info("[G4] !! 发布未获 broker 确认(puback): 连续 %u 次(旧阈值 %u 已不再触发"
                 "任何恢复动作)。模组回 OK 只说明【它自己收下了】, 不代表云端收到了。"
                 "按约定【不动链路】(驻网正常+会话在 => 死在模组栈与 broker 之间), "
                 "只记录; 下一条若回 601 才走 MQTT 快修\r\n",
                 (unsigned)g_pub_noack_cnt, (unsigned)G4_PUB_NOACK_MAX);

        g_pub_pending = false;
        (void)g4_pub_queue_next();      /* 接着发下一条(它才是"会话还活着吗"的探针) */
        return g_pub_pending;

    default:
        g_pub_state = PUB_IDLE;
        break;
    }
    return g_pub_pending;
}

/* ==================== 对外接口 ==================== */

void g4_at_init(void)
{
    g4_io_init();       /* 初始化4G AT 串口(USART2 PC8/PC9) */
    g4_urc_init();      /* URC 解析状态清零*/

    g_pub_state    = PUB_IDLE;
    g_pub_pending  = false;
    g_pub_fail_cnt = 0U;
    g_pub_prompt_loss       = 0u;
    g_pub_prompt_loss_total = 0u;
    s_conn_seq_base         = 0u;
    g_mqtt_fast_cnt         = 0u;
    g_mqtt_fast_total       = 0u;
    /* ★2026-10-08: 冷启动【必须】清掉快修标记 —— 否则上电后第一次建链会误用
     * 8 秒的短超时(冷启动要搜网+注册+DNS, 8 秒根本不够), 白白多断一次。 */
    g_mqtt_fast_active      = false;
    g_mqtt_suspect          = false;
    g_pub_noack_cnt         = 0u;
    g_ready_base_online     = 0u;
    g_reg_probe    = 0u;
    g_pdp_retry    = 0u;
    g_dialup_tried = false;
    g_apn_idx      = 0u;
    g_fail_cnt     = 0u;
    g_pwr_ok       = false;
    g_pwr_flips    = 0u;
    g_boot_wait_ms = G4_BOOT_LISTEN_MS;
    g_last_pub_ok_valid = false;
    g_last_pub_ok_tick  = 0u;
    g_rst_used     = false;         /* 本次开机还没用过断电重启 */
    g_last_rst_tick = 0u;
    g_cfun         = -1;            /* 还没读 CFUN */
    g_cfun_need_on = false;

    /* 驻网(CEREG)只读观测: 全部清零 */
    memset(&g_cereg, 0, sizeof(g_cereg));
    g_cereg.stat          = 0xFFu;  /* 还没读到 */
    g_cereg_inflight      = false;
    g_cereg_next_tick     = 0u;

    /* 模组自己的 MQTT 参数 与 恢复退避(2026-10-04 第 2 步)
     * ⚠️ 【这里【不许】复位"由配置注入"的量】—— g_mqtt_keepalive_s / g_reconn_*_s
     *    来自 main.c 的 g4_at_set_keepalive_s()/g4_at_set_reconnect()。谁先谁后
     *    不该影响结果: 本函数若清零它们, "先注入再 init" 就会【静默丢掉配置】
     *    (探针 probe_at_cfg 第一次就是这么把 keepalive 整条漏掉的)。
     *    已有的 g4_at_set_broker()/set_sub_topic()/set_lwt() 也是同样的约定:
     *    init 不动它们。这里只复位【运行时状态】。 */
    g_backoff_ms         = 0u;
    g_recover_hold_tick  = 0u;
    g_rst_count          = 0u;
    g_mqtt_reconn_since  = 0u;
    g_cgev_detach_seen   = 0u;

    g4_io_rx_reset();
    g4_info_clear();

    /* 【必须显式初始化 PA8!【     * 引脚定义与极性在 app/bsp/bsp_gpio.h 顶部(换板子改那里):
     *   G4_PWR_SW_*      →模组【电源开关】本板: PA8; 模组没有引出 RST)
     * 这段初始化原来藏在usbhs_bsp.c 的USB_BSP_Init() 里 而4G 改走串口之后
     * 根本不会调USB_BSP_Init() ——MCU 复位后GPIO 是输入悬空 悬空的PA8 让模组     * 供电处于"半通状态 症状就是万用表量到的电压来回跳+ 模组一个字节都不回】*/
    G4_ModulePWR_GPIO_Init();   /* PB13 已改用途→空实现不要动电池脚) */
    G4_PowerSwitch_Init();      /* PA8: 配成推挽输出并按当前极性上电*/

#if G4_POWER_CYCLE_ON_BOOT
    /* 开机先"拔插一次电": 保证后面听到的+MATREADY 一定是本次启动的*/
    g4_pwr_cycle_and_listen("开机拔插");
#else
    log_info("[G4] 直接上电: PA8=%s (%s)\r\n", g4_pwr_level_text(), g4_pwr_polarity_text());
    g4_io_rx_reset();
#endif

    log_info("[G4] 先安静听 %us 捡 banner, 然后【主动敲 AT】(每 %ums 一次, 最多 %u 次); "
             "日志口敲 #help 有诊断命令\r\n",
             (unsigned)(g_boot_wait_ms / 1000u),
             (unsigned)G4_AT_PROBE_MS, (unsigned)G4_AT_SYNC_RETRY_MAX);
    /* 【为什么只短听一下就主动敲】: 现场实测这块模组(ML307Y)不会稳定地自己吐
     * +MATREADY —— 30s 里一个字都没有, 可一发 AT 它就把 +MATREADY 和 OK 一起回了。
     * 所以被动等注定等不到, 必须主动出击(见 AT_SYNC 分支)。 */
    g4_goto_state(G4_STATE_WAIT_MODULE);
}

void g4_at_set_recv_cb(g4_mqtt_recv_cb_t cb)
{
    g4_urc_set_recv_cb(cb);
}

/**
 * @brief 设置下行订阅 topic(main.c 按 cloud_profile 注入 rt/gw/{gwId}/cmd)
 *
 * ★2026-10-08: 不再有编译期兜底值★ 传 NULL/空串 = 【把订阅 topic 清空】,
 * 之后建链会打 !! 并跳过订阅 —— 而不是悄悄回落成 "mod/pc_down"(那个遗留主题已删除)。
 * 这样"没注入"是一个看得见的错误, 不是一个静默的错误主题。
 *
 * ⚠ 只在【下一次 MQTT 建链】时生效 —— 本函数不会主动重订阅。
 *   运行期改 gwId/主题需要让 MQTT 重连(或后续补一条重订阅逻辑)。
 */
void g4_at_set_sub_topic(const char *topic)
{
    if ((topic == NULL) || (topic[0] == '\0'))
    {
        g_sub_topic[0] = '\0';          /* 清空: 明确表示"没有订阅主题" */
        return;
    }
    strncpy(g_sub_topic, topic, sizeof(g_sub_topic) - 1u);
    g_sub_topic[sizeof(g_sub_topic) - 1u] = '\0';
}

const char *g4_at_get_sub_topic(void)
{
    return g_sub_topic;
}

/* ==================== v79: broker 参数注入(配置域 network) ====================
 * 文档 3.2 写"network 整域【重启生效】", 但此前 g4_at 只认编译期常量 —— 配置域是死配置。
 * 由 main.c 在启动时把配置灌进来(见 main.c 的 [BOOT] broker 日志)。
 * 【约定】传 NULL 或空串 = **不覆盖该项** —— 这样 main.c 不必自己判断,
 *   也保证"没配的项仍用编译期默认"这一行为不被悄悄改掉。 */
void g4_at_set_broker(const char *host, const char *port,
                      const char *user, const char *pass)
{
    if ((host != NULL) && (host[0] != '\0'))
    {
        strncpy(g_broker_host, host, sizeof(g_broker_host) - 1u);
        g_broker_host[sizeof(g_broker_host) - 1u] = '\0';
    }
    if ((port != NULL) && (port[0] != '\0'))
    {
        strncpy(g_broker_port, port, sizeof(g_broker_port) - 1u);
        g_broker_port[sizeof(g_broker_port) - 1u] = '\0';
    }
    if ((user != NULL) && (user[0] != '\0'))
    {
        strncpy(g_broker_user, user, sizeof(g_broker_user) - 1u);
        g_broker_user[sizeof(g_broker_user) - 1u] = '\0';
    }
    if ((pass != NULL) && (pass[0] != '\0'))
    {
        strncpy(g_broker_pass, pass, sizeof(g_broker_pass) - 1u);
        g_broker_pass[sizeof(g_broker_pass) - 1u] = '\0';
    }
}

const char *g4_at_get_broker_host(void)
{
    return g_broker_host;
}

const char *g4_at_get_broker_port(void)
{
    return g_broker_port;
}

/* ★2026-10-08 新增★ 取当前生效的 MQTT 用户名(默认 "tuguan")。
 * 启动日志要能一眼看出"到底用的哪个用户名"(连不上时这是第一个要看的字段)。 */
const char *g4_at_get_broker_user(void)
{
    return g_broker_user;
}

/* ★2026-10-08 新增★ 取当前生效的 MQTT 密码 —— 只给日志/诊断判"设没设"用。
 * ⚠️ 调用方【不要把它打进日志】: 串口日志会外泄凭据。启动日志只打"已设/空"。 */
const char *g4_at_get_broker_pass(void)
{
    return g_broker_pass;
}

/* ★2026-10-08 新增★ 【注入 clientID】
 *
 * 默认是编译期的 "110001"(用户要求固定)。main.c 用 `network.clientId` 覆盖它 ——
 * 这样一来"多台网关 clientID 冲突"不用改固件就能解决:
 *     ① cfg.set network.clientId = "<唯一串>"
 *     ② cfg.set network.clientId = "IMEI"     <- 特殊值, 用模组 IMEI, 天然唯一
 * 传 NULL/空串 = 不覆盖(保留当前值)。
 * ⚠ 只在【下一次 MQTT 建链】时生效。 */
void g4_at_set_client_id(const char *cid)
{
    if ((cid == NULL) || (cid[0] == '\0'))
    {
        return;                             /* 空 = 不覆盖 */
    }
    strncpy(g_client_id, cid, sizeof(g_client_id) - 1u);
    g_client_id[sizeof(g_client_id) - 1u] = '\0';
}

const char *g4_at_get_client_id(void)
{
    return g_client_id;
}

const g4_module_info_t *g4_at_get_info(void)
{
    return &g_info;
}

/* ==================== 时钟诊断 (#clk) ====================
 *
 * 【为什么需要它】板上是 24MHz 外部晶振(PH0/PH1), 而两个构建配置里的 HSE_VALUE
 * 分别是 8MHz(EIDE) 和 16MHz(Keil), 【都对不上】。
 * system_n32h47x_48x.c 的 SetSysClock() 在 BOARD_USE_HSE=1 时按 HSE_VALUE 推算
 * PLL 分频系数; 声明的值与真实晶振不符时, 要么 PLL 锁不住, 要么主频差好几倍。
 * 编译期宏只是"声称", 光看代码推不出来 —— 必须上板把寄存器读出来看:
 *   · HSERDF=0 -> 外部晶振根本没起振, SetSysClock 走失败分支把 SystemCoreClock
 *                 置成 HSI_VALUE(8MHz), 整机按 8MHz 跑(能用但慢几十倍);
 *   · SCLKSTS  -> 现在到底是谁在当系统时钟(HSI / HSE / PLL)。
 * 用法: 串口敲 "#clk"。
 */
void g4_at_print_clock(void)
{
#if defined(N32H482)
    uint32_t       ctrl = RCC->CTRL;
    uint32_t       cfg  = RCC->CFG;
    uint32_t       pll  = RCC->PLLCTRL;
    uint32_t       mul  = (((cfg & RCC_CFG_PLLMULFCT) >> 3) * 2u) + 4u;
    const char    *src;
    RCC_ClocksType clk;

    switch (cfg & RCC_CFG_SCLKSTS)
    {
        case RCC_CFG_SCLKSTS_HSI: src = "HSI(内部RC)";   break;
        case RCC_CFG_SCLKSTS_HSE: src = "HSE(外部晶振)"; break;
        case RCC_CFG_SCLKSTS_PLL: src = "PLL";           break;
        default:                  src = "?保留?";        break;
    }

    memset(&clk, 0, sizeof(clk));
    RCC_GetClocksFreqValue(&clk);

    log_info("[CLK] SystemCoreClock=%lu Hz (%lu MHz)\r\n",
             (unsigned long)SystemCoreClock,
             (unsigned long)(SystemCoreClock / 1000000u));
    log_info("[CLK] 系统时钟来源 = %s (SCLKSTS=0x%08lX)\r\n",
             src, (unsigned long)(cfg & RCC_CFG_SCLKSTS));
    log_info("[CLK] HSI: EN=%u RDY=%u | HSE: EN=%u RDY=%u | PLL: EN=%u RDY=%u\r\n",
             (unsigned)((ctrl & RCC_CTRL_HSIEN)  ? 1u : 0u),
             (unsigned)((ctrl & RCC_CTRL_HSIRDF) ? 1u : 0u),
             (unsigned)((ctrl & RCC_CTRL_HSEEN)  ? 1u : 0u),
             (unsigned)((ctrl & RCC_CTRL_HSERDF) ? 1u : 0u),
             (unsigned)((ctrl & RCC_CTRL_PLLEN)  ? 1u : 0u),
             (unsigned)((ctrl & RCC_CTRL_PLLRDF) ? 1u : 0u));
    log_info("[CLK] PLL: 源=%s 分频=%s 倍频=%lu | CTRL=0x%08lX CFG=0x%08lX PLLCTRL=0x%08lX\r\n",
             ((cfg & RCC_CFG_PLLSRC) ? "HSE" : "HSI"),
             ((cfg & RCC_CFG_PLLHSEPRES) ? "输入/2" : "输入/1"),
             (unsigned long)mul,
             (unsigned long)ctrl, (unsigned long)cfg, (unsigned long)pll);
    log_info("[CLK] 驱动算出: SYSCLK=%luHz HCLK=%luHz PCLK1=%luHz PCLK2=%luHz\r\n",
             (unsigned long)clk.SysclkFreq, (unsigned long)clk.HclkFreq,
             (unsigned long)clk.Pclk1Freq, (unsigned long)clk.Pclk2Freq);
#ifdef HSE_VALUE
    /* ---- 声明值 vs 真实晶振(2026-09-27 加) ----
     * 实际主频【只取决于真实晶振】, 与 HSE_VALUE 无关; HSE_VALUE 只决定
     * SetSysClock 按哪个分频/倍频组合去配 PLL。
     * 两者不一致 = 主频错, 而这种错【不会报任何错】, 只会让整机时序全偏。
     * 所以把这张反推表直接打在板上, 结论不依赖任何推理。
     *
     * 实测依据(本次排障): 串口 USART1 在 115200 下文本干净 ->
     *   USART_Init() 用 RCC_GetClocksFreqValue() 的 Pclk2Freq 算分频, 那是
     *   "按声明值反推的以为值"; 若真实主频是以为值的 1.5 倍, 实际波特率会变成
     *   172800, 115200 下必然是乱码。文本干净 = 真实主频就是以为的那个值。 */
    log_info("[CLK] 本固件编译时声明 HSE_VALUE=%lu Hz (%lu MHz) —— 它必须等于板上真实晶振\r\n",
             (unsigned long)HSE_VALUE, (unsigned long)(HSE_VALUE / 1000000u));
    {
        static const uint32_t cand[] = { 8000000u, 12000000u, 16000000u,
                                         24000000u, 25000000u, 26000000u };
        uint8_t i;

        log_info("[CLK] 按当前寄存器的分频/倍频反推【各种晶振下的实际主频】"
                 "(N32H482 上限 240MHz):\r\n");
        for (i = 0u; i < 6u; i++)
        {
            uint32_t in  = ((cfg & RCC_CFG_PLLHSEPRES) ? (cand[i] / 2u) : cand[i]);
            uint32_t out = in * mul;

            log_info("[CLK]   晶振 %-2luMHz -> 实际 %-4luMHz %-10s%s\r\n",
                     (unsigned long)(cand[i] / 1000000u),
                     (unsigned long)(out / 1000000u),
                     (out > 240000000u) ? "★超规格!" : "符合规格",
                     (out == (uint32_t)SystemCoreClock) ? " <=与固件自报一致" : "");
        }
        log_info("[CLK] 判据: 你的晶振那一行必须是\"符合规格\"且\"与固件自报一致\";\r\n");
        log_info("[CLK]       若打出\"超规格\", 说明声明值与真实晶振不符, 整机正在超频跑\r\n");
    }
#else
    log_info("[CLK] 本固件未声明 HSE_VALUE\r\n");
#endif
    log_info("[CLK] 判读: 外部晶振 %s -> %s\r\n",
             ((ctrl & RCC_CTRL_HSERDF) ? "已起振" : "1111【没起振】"),
             ((ctrl & RCC_CTRL_HSERDF)
                  ? "PLL 按声明的 HSE_VALUE 配置; 声明值与真实晶振不符则主频不对"
                  : "SetSysClock 走失败分支, 整机按 HSI 8MHz 跑(能用但慢 30 倍)"));
#else
    log_info("[CLK] 本次是主机构建, 无 RCC 寄存器可读\r\n");
#endif
}

/* ==================== 日志口命令扩展钩子 ====================
 *
 * 主程序把云协议的离线自测命令(#json / #status / #topic)挂在这里, 复用同一个
 * "整行输入 + 不转发给模组"的通道。不注册时行为与本改动前【完全一致】。
 * 见 g4_at.h 的说明。 */
static g4_console_ext_fn  g_con_ext;
static g4_console_help_fn g_con_help;

void g4_at_set_console_ext(g4_console_ext_fn ext, g4_console_help_fn help)
{
    g_con_ext  = ext;
    g_con_help = help;
}

/* ==================== 日志口本地诊断命令 ====================
 *
 * 用法: 在日志串口(USART1, 115200)上敲一整行、回车。
 *   从 '#' 开头的行【不转发给模组】, 本地执行; 其它行照旧当 AT 命令转发。
 * 现场排"模组没电 / 电压不稳 / 一个字节都不回"就靠这几条:
 *   #help           列出命令
 *   #pwr?           真 PA8 实际电平 + 当前认为的上电"电平" + 累计收到字节数
 *   #pwron/#pwroff  手工给模组上电/断电(断电会自动放开 PC8, 不反向灌电)
 *   #pwrswap        把上电"电平"翻到相反一边, 断电重上电并重新预热
 *                   ★拿万用表盯模组电源脚, 敲一次 #pwrswap 看一次电压;
 *                     哪一次电压变成 3.3~3.8V, 那次的上电电平就是对的
 *   #cycle          按当前极性断电重上电(等于拔插一次)
 *   #listen <秒>    改预热窗口长度并重新开始, 例如 "#listen 60"
 *   #probe          跳过预热, 立刻开始 AT 握手
 *   #at <命令>      等价于直接敲 <命令>(转发给模组, 并把应答原样打出来)
 *   #clk            打印系统时钟来源 + HSE/PLL 寄存器实况
 *   #vbat / #vbatsweep <次数>
 *
 * 另外, 主程序可通过 g4_at_set_console_ext() 追加自己的命令(云协议离线自测,
 * 以及射频诊断 #lora —— 见 main.c 的 main_console_ext()/main_console_help()),
 * 那些命令的用法由 #help 末尾一并列出。
 * 【射频诊断为什么不在这个文件】: 本文件属于 4G/AT 模块, include 里没有任何射频
 * 头文件, 而且要被主机单测(_host_test/test_state)编译 —— 把 radio.h/LLCC68-board.h
 * 引进来会把射频层耦合到 4G 模块并破坏主机测试。所以 #lora 走 main.c 的扩展钩子,
 * 实现放在 app/protocol/lora_proto.c 的 LoraApp_DiagPrint()。
 */
bool g4_at_console_cmd(const char *line)
{
    if ((line == NULL) || (line[0] != '#'))
    {
        return false;
    }

    if (strcmp(line, "#help") == 0)
    {
        log_info("[DIAG] #pwr? #pwron #pwroff #pwrswap #cycle(=#rst, 4G_RST 断电重启) #listen <秒> #probe\r\n");
        log_info("[DIAG] #clk (打印系统时钟来源 + HSE/PLL 寄存器实况, 查外部晶振有没有起振)\r\n");
        log_info("[DIAG] #vbat (读一次模组自测供电) / #vbatsweep <次数> (连测抓跳变)\r\n");
        log_info("[DIAG] #at <命令> 会把模组应答原样打出来, 例如: #at AT+MQTTCONN=?\r\n");
        log_info("[DIAG] #broker [#broker <host> <port> [user] [pass]] 台架改 broker 并立刻重连\r\n");
        log_info("[DIAG] #tls <0|1> 台架开关 TLS 并重连 / #reconn 只强制重连一次\r\n");
        log_info("[DIAG] #mqttcfg 逐条查询模组【实际生效】的 MQTT 参数(查 TLS 有没有真开)\r\n");
        log_info("[DIAG] #dial (手工试 AT+MDIALUP=1,1 拨号 —— 注意它会把模组 AT 引擎搞乱, "
                 "默认流程已不用它)\r\n");
        log_info("[DIAG] 不以 # 开头的行照旧原样转发给模组(不回显应答, 查参数请用 #at)\r\n");

        /* 主程序追加的自定义命令(如云协议离线自测) */
        if (g_con_help != NULL)
        {
            g_con_help();
        }
    }
    else if (strcmp(line, "#clk") == 0)
    {
        g4_at_print_clock();
    }
    else if (strcmp(line, "#dial") == 0)
    {
        /* 【手工拨号 —— 仅供台架验证】
         * 实测这条命令会让模组 AT 引擎乱掉(之后只回 +MATREADY+OK, 连 +CEREG: 都没有),
         * 所以【自动建链流程已经不用它了】。留在这里是为了现场想亲眼看一次它的行为,
         * 或者核对 AT+MDIALUPCFG="mode" 当前是 RNDIS 还是别的。 */
        char resp[256];

        log_info("[DIAG] 手工拨号: 先看当前拨号模式配置...\r\n");
        resp[0] = '\0';
        (void)g4_io_send_expect_resp("AT+MDIALUPCFG=\"mode\"", "OK", AT_CMD_TIMEOUT_MS,
                                     NULL, resp, sizeof(resp));
        if (resp[0] != '\0') { log_info("[DIAG] MDIALUPCFG: %s\r\n", resp); }

        log_info("[DIAG] 发 AT+MDIALUP=1,1 (最长等 30s)...\r\n");
        resp[0] = '\0';
        (void)g4_io_send_expect_resp("AT+MDIALUP=1,1", "+MDIALUP:", AT_DIALUP_TIMEOUT_MS,
                                     NULL, resp, sizeof(resp));
        log_info("[DIAG] 拨号应答: %s\r\n", (resp[0] != '\0') ? resp : "(无)");

        resp[0] = '\0';
        (void)g4_io_send_expect_resp("AT+MDIALUP?", "+MDIALUP:", AT_CMD_TIMEOUT_MS,
                                     NULL, resp, sizeof(resp));
        log_info("[DIAG] 拨号状态: %s\r\n", (resp[0] != '\0') ? resp : "(无)");
        log_info("[DIAG] 手工拨号结束。如果之后 AT 开始只回 +MATREADY+OK, 敲 #cycle "
                 "断电重上电一次就恢复\r\n");
    }
    else if (strcmp(line, "#pwr?") == 0)
    {
        log_info("[DIAG] PA8 驱动=%s 引脚读回=%s | 判据:%s | 累计收到=%lu字节 | 静听=%lus | 已翻极性%u次\r\n",
                 g4_pwr_level_text(),
                 G4_PowerSwitch_ReadPin() ? "H" : "L",
                 g4_pwr_polarity_text(),
                 (unsigned long)g4_io_rx_total(),
                 (unsigned long)(g_boot_wait_ms / 1000u),
                 (unsigned)g_pwr_flips);
    }
    else if (strcmp(line, "#pwron") == 0)
    {
        G4_PowerSwitch_Set(1u);
        log_info("[DIAG] 已上电 PA8=%s (%s)\r\n",
                 g4_pwr_level_text(), g4_pwr_polarity_text());
    }
    else if (strcmp(line, "#pwroff") == 0)
    {
        G4_PowerSwitch_Set(0u);
        log_info("[DIAG] 已断电 PA8=%s (PC8/TX 已放开, 不会反向灌电)\r\n",
                 g4_pwr_level_text());
    }
    else if (strcmp(line, "#pwrswap") == 0)
    {
        g_pwr_ok    = false;
        g_pwr_flips = 0u;
        G4_PowerSwitch_SwapOnHigh();
        log_info("[DIAG] 上电电平翻成: %s\r\n", g4_pwr_polarity_text());
        log_info("[DIAG] (本板正确极性是 PA8 低电平上电 翻错了再敲一次#pwrswap 即可翻回)\r\n");
        g4_pwr_cycle_and_listen("手工翻转极性");
        g4_goto_state(G4_STATE_WAIT_MODULE);
    }
    else if (strcmp(line, "#cycle") == 0 || strcmp(line, "#rst") == 0)
    {
        /* #rst 是 #cycle 的别名 —— 现场说的"4G_RST"就是这条: 断电 -> 上电 -> 重新建链 */
        g4_pwr_cycle_and_listen("手工断电重上电(4G_RST)");
        g4_goto_state(G4_STATE_WAIT_MODULE);
    }
    else if (strncmp(line, "#listen", 7) == 0)
    {
        uint32_t sec = (uint32_t)atoi(&line[7]);

        if (sec == 0u)
        {
            sec = G4_BOOT_LISTEN_MS / 1000u;
        }
        g_boot_wait_ms = sec * 1000u;
        log_info("[DIAG] 静听窗口改为 %lus, 重新开始\r\n", (unsigned long)sec);
        g4_pwr_cycle_and_listen("改预热窗口");
        g4_goto_state(G4_STATE_WAIT_MODULE);
    }
    else if (strcmp(line, "#probe") == 0)
    {
        log_info("[DIAG] 跳过预热, 立刻试AT 握手\r\n");
        g4_goto_state(G4_STATE_AT_SYNC);
    }
    else if (strcmp(line, "#vbat") == 0)
    {
        /* 模组内部 ADC 自己测的 VBAT ——比万用表出万用表读的是平均值 */
        (void)g4_read_vbat();
    }
    else if (strncmp(line, "#vbatsweep", 10) == 0)
    {
        /* 抓供电跳发 连测 N 次给 min/max/avg, 直接回答"3.8~4.1 跳到底要不要算 */
        uint16_t n = (uint16_t)atoi(&line[10]);

        log_info("[DIAG] 注意: 这一步会阻塞主循环几种LoRa 会被拖慢)\r\n");
        g4_vbat_sweep(n);
    }
    else if (strcmp(line, "#mqttcfg") == 0)
    {
        /* ---- 逐条把模组【实际生效】的 MQTT 参数查出来 ----
         * 【为什么必须有这条】2026-10-05 的一夜教训:
         *   `AT+MQTTCFG=xxx` 【不管参数对不对都回 OK】—— 我们按手册发
         *   `AT+MQTTCFG="version",0,4` 也回 OK, 而实际正确的查询形态是 `"version",4`。
         *   也就是说: 只看 OK 根本不知道参数有没有生效。
         * 【查询形态的两条规矩】(现场实测, 与手册不同):
         *   ① 不带 '?' —— 带 '?' 回 +CME ERROR: 601(无效参数);
         *   ② 省略可选参数 —— 例如查 connect_id 0 的 ssl 就是 `AT+MQTTCFG="ssl",0`。
         * 这条命令把常用的 9 条一次查完, 台架排查 TLS/遗嘱/保活时一眼就能看到真实值。 */
        static const char *const q[] = {
            "AT+MQTTCFG=\"version\"",
            "AT+MQTTCFG=\"ssl\",0",
            "AT+MQTTCFG=\"clean\",0",
            "AT+MQTTCFG=\"keepalive\",0",
            "AT+MQTTCFG=\"pingreq\",0",
            "AT+MQTTCFG=\"pingresp\",0",
            "AT+MQTTCFG=\"reconn\",0",
            "AT+MQTTCFG=\"willoption\",0",
            "AT+MQTTCFG=\"willpayload\",0",
            "AT+MQTTSTATE? "
        };
        static char rbuf[256];
        uint8_t     qi;

        log_info("[DIAG] 逐条查询模组实际生效的 MQTT 参数(每条最长等 1.5s)...\r\n");
        for (qi = 0u; qi < (uint8_t)(sizeof(q) / sizeof(q[0])); qi++)
        {
            rbuf[0] = '\0';
            (void)g4_io_send_expect_resp(q[qi], "\x01NEVERMATCH\x02", 1500u, NULL,
                                         rbuf, (uint16_t)sizeof(rbuf));
            log_info("[DIAG]   %-30s -> %s\r\n", q[qi],
                     (rbuf[0] != '\0') ? rbuf : "(无应答: 该参数/命令可能不被固件支持)");
        }
        log_info("[DIAG] 说明: ssl 应为 \"ssl\",1 (TLS 开); 若为 0 说明【TLS 从来没开过】。\r\n");
    }
    else if (strncmp(line, "#broker", 7) == 0)
    {
        (void)g4_con_broker_cmd((line[7] != '\0') ? &line[8] : "", false);
    }
    else if (strncmp(line, "#tls", 4) == 0)
    {
        (void)g4_con_broker_cmd((line[4] != '\0') ? &line[5] : "", true);
    }
    else if (strcmp(line, "#reconn") == 0)
    {
        log_info("[DIAG] 手工触发一次 MQTT 重连(只重连 MQTT, 不断电)...\r\n");
        g4_mqtt_fast_rebuild("台架手工触发重连");
    }
    else if (strncmp(line, "#at ", 4) == 0)
    {
        char resp[512];

        /* 【必须把应答打出来。 原来这里只是 g4_io_send_cmd() 把命令发出去就完了——         * 模组的回复会被主循环的URC 扫描吃掉, 而扫描只打印它认识的 URC 表 于是
         * "敲了 AT 一点反应都没有", 根本没法用它查参数表【         * 现在改成阻塞收发: 用一个永远不会匹配的关键字 等满 1.5s, 把累积到的应答         * 原样打出来。查 "AT+MQTTCONN=?" 这类参数表就靠它【*/
        log_info("[DIAG] 发送并等应答最长1.5s): %s\r\n", &line[4]);
        resp[0] = '\0';
        (void)g4_io_send_expect_resp(&line[4], "\x01NEVERMATCH\x02", 1500u, NULL,
                                     resp, sizeof(resp));
        if (resp[0] != '\0')
        {
            log_info("[DIAG] 应答:\r\n%s\r\n", resp);
        }
        else
        {
            log_info("[DIAG] 1.5s 内没有任何应答命令不存在会回ERROR, 那也是一种答案\r\n");
        }
    }
    else if ((g_con_ext == NULL) || (!g_con_ext(line)))
    {
        /* 内建命令表没有, 且主程序的扩展钩子也不认 —— 交给调用方打"未知命令" */
        return false;
    }

    return true;
}

/* ==================== 台架: 运行期改 broker / 账号 / TLS ====================
 *
 * 【为什么要有这几条】(2026-10-05, 接私有 EMQX Cloud 时)
 *   拿到 conn state=3(服务器拒绝连接) 之后要快速做 A/B: 换端口试明文 / 换账号 /
 *   开关 TLS / 钉协议版本 —— 每一样都要"改一个参数再看一次"。而本板配置存储两区
 *   都坏了(`[CFG] 两区都不可用`), cfg.set 存不住; 重编译+重烧一轮要好几分钟,
 *   现场根本没法试。这几条命令让一组参数几秒钟就能换掉, 而且【不用断电】——
 *   改完自动走一次 MQTT 快修(只重连 MQTT)。
 *
 * 用法(日志口, 115200, 每行以 # 开头):
 *   #broker                                  打印当前生效的参数
 *   #broker <host> <port>                    只改地址+端口(账号密码不动), 立刻重连
 *   #broker <host> <port> <user> <pass>      四个一起写, 立刻重连
 *                                             ★token 写 "-" 表示【空串】★
 *                                             (去连无认证的公共 broker 单独试 TLS 时,
 *                                              必须能把私有账号清掉, 否则实验是假的)
 *   #tls <0|1>                               关/开 TLS, 立刻重连
 *   #reconn                                  不改参数, 只强制重连一次
 *
 * ⚠️ 这些只改 RAM 里的值, 【不写配置】—— 断电/复位就回到编译期默认。
 *    这是刻意的: 台架试参数不该污染"出厂配置"。
 */
#define G4_CON_ARG_MAX   5

/** @brief 把一行按空格切成最多 max 个 token; 返回切到的个数(原地改 buffer) */
static uint8_t g4_con_split(char *line, char *tok[], uint8_t max)
{
    uint8_t n = 0u;

    while ((*line != '\0') && (n < max))
    {
        while ((*line == ' ') || (*line == '\t')) { line++; }
        if (*line == '\0') { break; }
        tok[n++] = line;
        while ((*line != '\0') && (*line != ' ') && (*line != '\t')) { line++; }
        if (*line != '\0') { *line++ = '\0'; }
    }
    return n;
}

/** @brief 台架专用: 与 g4_at_set_broker 不同, 它【允许写空】。
 *
 * 【为什么不能直接用 g4_at_set_broker】那个函数的约定是"空串 = 不覆盖" —— 那是给
 * main.c 注入配置用的(配置里没填的字段不该把编译期默认冲掉)。但台架做 A/B 时
 * 【必须能把账号密码清掉】: 例如去连无认证的公共 broker 单独试 TLS 通道, 若还带着
 * 私有 broker 的账号密码, broker 会把我们拒掉 —— 那这个对照实验就是假的。
 * 语义不同, 所以分开, 不混在一起。 */
static void g4_con_set_broker(const char *host, const char *port,
                              const char *user, const char *pass)
{
    (void)snprintf(g_broker_host, sizeof(g_broker_host), "%s",
                   (host != NULL) ? host : "");
    (void)snprintf(g_broker_port, sizeof(g_broker_port), "%s",
                   (port != NULL) ? port : "");
    (void)snprintf(g_broker_user, sizeof(g_broker_user), "%s",
                   (user != NULL) ? user : "");
    (void)snprintf(g_broker_pass, sizeof(g_broker_pass), "%s",
                   (pass != NULL) ? pass : "");
}

static bool g4_con_broker_cmd(const char *args, bool tls_cmd)
{
    char        buf[192];
    char       *tok[G4_CON_ARG_MAX];
    uint8_t     n;

    if ((args == NULL) || (strlen(args) >= sizeof(buf)))
    {
        return false;
    }
    (void)snprintf(buf, sizeof(buf), "%s", args);
    n = g4_con_split(buf, tok, (uint8_t)G4_CON_ARG_MAX);

    /* token 写 "-" 表示【空串】—— 因为"用户名留空"正是 conn state=3 的一个主要
     * 嫌疑(EMQX Cloud 若只配了密码没有用户名, 我们带上用户名就会被拒)。
     * 命令行里没法打空 token, 所以约定一个占位符。 */
    {
        uint8_t i;
        for (i = 0u; i < n; i++)
        {
            if ((tok[i][0] == '-') && (tok[i][1] == '\0'))
            {
                tok[i][0] = '\0';
            }
        }
    }

    if (tls_cmd)
    {
        if (n < 1u)
        {
            log_info("[DIAG] 用法: #tls <0|1>  (当前 TLS=%s)\r\n",
                     (G4_MQTT_TLS_ENABLE != 0) ? "开" : "关");
            return true;
        }
        g4_at_set_tls((uint8_t)((tok[0][0] == '1') ? 1u : 0u));
        log_info("[DIAG] 已改 TLS 开关, 触发 MQTT 重连...\r\n");
        g4_mqtt_fast_rebuild("台架手工改 TLS 开关");
        return true;
    }

    if (n < 2u)
    {
        log_info("[DIAG] 当前生效: broker=%s:%s tls=%s user=%s pass=%s\r\n",
                 g_broker_host, g_broker_port,
                 g_tls_enable ? "开" : "关",
                 (g_broker_user[0] != '\0') ? g_broker_user : "(空)",
                 (g_broker_pass[0] != '\0') ? "(已设)" : "(空)");
        log_info("[DIAG] 用法: #broker <host> <port>              只改地址端口(账号不动)\r\n");
        log_info("[DIAG]       #broker <host> <port> <user> <pass> 四个一起写; '-' 表示空串\r\n");
        return true;
    }

    if (n >= 4u)
    {
        /* 四个都给: 【允许写空】—— 见 g4_con_set_broker 的说明 */
        g4_con_set_broker(tok[0], tok[1], tok[2], tok[3]);
        log_info("[DIAG] 已改 broker=%s:%s user=%s pass=%s, 触发 MQTT 重连...\r\n",
                 g_broker_host, g_broker_port,
                 (g_broker_user[0] != '\0') ? g_broker_user : "(空)",
                 (g_broker_pass[0] != '\0') ? "(已设)" : "(空)");
    }
    else
    {
        /* 只给 host/port: 账号密码保持不动 */
        g4_con_set_broker(tok[0], tok[1], g_broker_user, g_broker_pass);
        log_info("[DIAG] 已改 broker=%s:%s (账号不变: user=%s), 触发 MQTT 重连...\r\n",
                 g_broker_host, g_broker_port,
                 (g_broker_user[0] != '\0') ? g_broker_user : "(空)");
    }
    g4_mqtt_fast_rebuild("台架手工改 broker");
    return true;
}
/**
 * @brief 4G 主轮询: 收包泵 + 云端下行投递 + 非阻塞 AT 状态机
 *
 * 【与老版本最大的区别】: 这里不再有任何"发一条 AT 然后原地睡等应答"的循环。
 *   · 接收: g4_io_pump() 一次性把 rb_g4 消费干净(行/publish 定长二进制/重同步);
 *   · AT : g4_io_cmd_run() 每次只推进一步, 状态不变就下一拍再来;
 *   · 下行: 收全的 publish 进队列, 由 g4_urc_poll() 在主循环里投给业务层。
 * 于是"阻塞窗口把接收灌爆"和"多个消费者抢字节流"这两类问题从结构上消失。
 */
void g4_at_poll(void)
{
    uint32_t now = get_tick_ms();

    /* 1. 【最高优先级】把 USART2 收到的字节全部消费掉。
     *    统一接收泵负责: 行 -> URC 层 / AT 应答匹配; publish -> 按 <payload_len>
     *    收定长原始二进制。老版本这里是空的 —— 由各个"等应答"的循环各自去抢
     *    同一条字节流, 那正是一切"下发被劈开 / 永久失步 / 日志把接收顶爆"的根源。 */
    g4_io_pump();

    /* 2. 模组是不是在我们眼皮底下重启了?
     *    必须在发布子流程和状态机之前 —— 重启之后所有在途的命令/发布都作废了。 */
    if (g4_check_module_reboot())
    {
        return;
    }

    /* 3. 把队列里的云端下行投递给业务层。
     *    【必须在收包泵之外】: 业务回调里可能做几百毫秒的组帧/空口发送, 放在泵里
     *    会把接收卡住。放在发布子流程【之前】, 免得一次最长 15s 的发布把下行憋住。 */
    (void)g4_urc_poll();

    /* 4. 驻网(CEREG)只读观测轮询 —— 只记日志, 【不改变任何链路行为】。
     *    【必须在 pub_poll() 之前】: 两条共用同一个 AT 命令引擎, 先让这一拍决定
     *    "要不要占用引擎", 再由 pub_poll 决定要不要发(它开头有互斥闸门)。 */
    g4_cereg_diag_poll();

    /* 5. 发布子流程(自己走非阻塞命令引擎, 不会再抢走 '>' / OK 应答) */
    if (pub_poll())
    {
        return;
    }

    /* 6. 断链判定 / 恢复(2026-10-04 第 2 步: 分层, 并且用上模组自己的重连)
     *
     * ★这里以前是"任何 state != 0 都判'与服务器失去连接' -> 4G_RST 断电"。★
     * 那是把三种完全不同的"断"混成了一种(见日志分析):
     *   ① 驻网断了(射频/网络)        -> 才该整链路断电重建
     *   ② MQTT 会话死了(模组栈/服务器) -> MQTT 快修 2~5 秒, 不该断电
     *   ③ 端到端没送达(会话还在)     -> 只记诊断, 什么链都不该动
     * 尤其 state=1: 手册明写是"正在重连" —— 模组【自己】正在干活, 我们一断电
     * 就把它掐死了(现场 1.6h 断电 27 次, 67% 的 MQTTCONN 拿不到 conn URC)。
     *
     * 现在的口径:
     *   state == 1  -> 【不动链路】, 给模组 G4_MQTT_RECONN_GRACE_MS 自己回来;
     *                  超时才升级到 MQTT 快修(仍然不断电)。
     *   其它非 0    -> MQTT 会话已经结束(服务器拒绝/断开/ping超时/网络异常),
     *                  模组不会再自己救 -> 走模块级恢复入口(里面有退避;
     *                  真断网才断电, 否则只重新握手)。
     */
    if ((!g4_urc_is_online()) && (g_state == G4_STATE_ONLINE))
    {
        uint8_t st = g4_urc_get_conn_state();

        local_alarm_trigger(ALARM_4G_OFFLINE);

        if (st == 1u)
        {
            /* ---- 模组正在自己重连: 让它干完(这就是"用上模组自己的能力") ---- */
            if (g_mqtt_reconn_since == 0u)
            {
                g_mqtt_reconn_since = now;
                log_info("[G4] MQTT 正在重连(conn state=1) —— 【不动链路】, 给模组 "
                         "%us 自己回来(它配了 reconn: %u 次 × %us, n 倍间隔)\r\n",
                         (unsigned)(G4_MQTT_RECONN_GRACE_MS / 1000u),
                         (unsigned)G4_MQTT_RECONN_TIMES,
                         (unsigned)G4_MQTT_RECONN_IVL_S);
            }
            else if ((uint32_t)(now - g_mqtt_reconn_since) >= G4_MQTT_RECONN_GRACE_MS)
            {
                log_info("[G4] !! 模组自己重连已超 %us 仍未回来 -> 交给 MQTT 快修"
                         "(只重连 MQTT, 【不断电】)\r\n",
                         (unsigned)(G4_MQTT_RECONN_GRACE_MS / 1000u));
                g_mqtt_reconn_since = 0u;
                g4_mqtt_fast_rebuild("模组自己重连超时");
                return;
            }
            /* 【不 return】: 继续走主状态机 —— ONLINE 里还有保活看门狗和诊断要看。
             * (看门狗本身也会让开"正在重连"的时段, 见 G4_STATE_ONLINE。) */
        }
        else
        {
            log_info("[G4] MQTT down(state=%u %s)\r\n",
                     (unsigned)st, g4_conn_state_text(st));
            g_mqtt_reconn_since = 0u;
            g4_link_recover("与服务器失去连接(MQTT 断开)");
            return;
        }
    }
    else
    {
        g_mqtt_reconn_since = 0u;
    }

    /* 6b. 真·断网: 模组上报 PS 去附着
     * 手册原文: "+CGEV: NW DETACH —— 网络强制 PS 去附着, 这意味着所有激活的上下文
     * 都被去激活"。这是比 MQTT 更底层的铁证, 只有它才值得整链路断电重建。 */
    {
        uint32_t detach = g4_urc_cgev_detach_count();

        if (detach != g_cgev_detach_seen)
        {
            g_cgev_detach_seen = detach;
            if (g_state == G4_STATE_ONLINE)
            {
                log_info("[G4] !! 收到 PS 去附着(+CGEV DETACH, 累计 %lu 次) —— "
                         "这是【真断网】, 走整链路重建(带退避)\r\n",
                         (unsigned long)detach);
                g4_link_recover("PS 去附着(真断网)");
                return;
            }
        }
    }

    /* 7. 主状态机 */
    switch (g_state)
    {
    case G4_STATE_RESET:
        /* 假死/反复连不上时的最后手段: 拔电重来 */
        log_info("[G4] power reset (PA8=%s, %s)...\r\n",
                 g4_pwr_level_text(), g4_pwr_polarity_text());
        G4_ModulePWR_GPIO_Init();
        g4_pwr_cycle_and_listen("断电复位");
        g4_info_clear();
        g_pdp_retry    = 0u;
        g_dialup_tried = false;
        g4_goto_state(G4_STATE_WAIT_MODULE);
        break;

    case G4_STATE_WAIT_MODULE:
    {
        /* 【短听 + 主动出击】
         * 为什么不自己读缓冲: 收包泵已经在后台统计 +MATREADY/RDY 了。这里只查计数,
         * 好处是泵照常把每一行喂给 URC 层 —— 老版本"WAIT_MODULE 期间不敢扫 URC"
         * 的限制没有了。
         * 为什么不等满 30s 自己发 AT: 现场实测这块模组(ML307Y)不会稳定地自己吐
         * +MATREADY —— 30s 里一个字都没有, 可一发 AT 它就把 +MATREADY 和 OK 一起
         * 回了。所以只安静听一小会儿捡 banner, 到点就主动敲。 */
        if (g4_io_ready_total() != g_ready_base)
        {
            log_info("[G4] module ready (+MATREADY), 等了 %lus —— 立刻开始握手\r\n",
                     (unsigned long)((now - g_state_tick) / 1000u));
            g4_goto_state(G4_STATE_AT_SYNC);
            break;
        }

        /* 只要听到过任何字节(哪怕是一行乱码), 就说明供电/接线至少有一项是对的,
         * 把当前极性记死, 以后不再翻 —— 否则会把一个已经能通的模组反复断电。 */
        if ((!g_pwr_ok) && (g4_io_rx_total() > 0u))
        {
            g_pwr_ok = true;
            log_info("[G4] == 模组开口说话了 -> 确认供电极性: %s (PA8=%s) ==\r\n",
                     g4_pwr_polarity_text(), g4_pwr_level_text());
        }

#if G4_PWR_SEARCH
        /* 一个字都没听到, 且还没试过相反极性 -> 头号嫌疑: PA8 极性猜反了 */
        if ((!g_pwr_ok) && (g_pwr_flips < G4_PWR_SEARCH_MAX) &&
            ((now - g_state_tick) > G4_PWR_SEARCH_MS))
        {
            g_pwr_flips++;
            G4_PowerSwitch_SwapOnHigh();
            log_info("[G4] !! %us 内一个字节都没听到 -> 怀疑供电极性反了, 翻成 %s 再试\r\n",
                     (unsigned)(G4_PWR_SEARCH_MS / 1000u), g4_pwr_polarity_text());
            g4_pwr_cycle_and_listen("极性翻转");
            break;
        }
#endif

        /* 安静听的时间到 -> 主动出击(不再无限等) */
        if ((now - g_state_tick) > g_boot_wait_ms)
        {
            log_info("[G4] 静听 %us 结束(累计收到 %lu 字节) -> 开始主动敲 AT\r\n",
                     (unsigned)(g_boot_wait_ms / 1000u),
                     (unsigned long)g4_io_rx_total());
            g4_goto_state(G4_STATE_AT_SYNC);
        }
        break;
    }

    case G4_STATE_AT_SYNC:
    {
        /* 【主动出击的锤子】每隔 G4_AT_PROBE_MS(500ms) 敲一次 "AT"。
         * 单次超时就是 G4_AT_PROBE_MS, 所以敲的节奏稳定, 不会被长超时拖成 1Hz;
         * 每 10 次打一行进度 + 【累计收到】字节数。 */
        g4_cmd_res_t r = g4_io_cmd_run("AT", "OK", G4_AT_PROBE_MS);
        if (r == G4_CMD_RUNNING)
        {
            break;
        }

        if (r == G4_CMD_OK)
        {
            log_info("AT_SYNC: OK (第 %u 次敲, 从开始敲到应答 %lus)\r\n",
                     (unsigned)(g_retry_cnt + 1u),
                     (unsigned long)((now - g_state_tick) / 1000u));
            g4_goto_state(G4_STATE_ATE0);
            break;
        }

        g_retry_cnt++;

        if ((g_retry_cnt % 10u) == 0u)
        {
            log_info("[G4] 主动敲 AT %u/%u 次, 累计收到 %lu 字节(PA8=%s)\r\n",
                     (unsigned)g_retry_cnt, (unsigned)G4_AT_SYNC_RETRY_MAX,
                     (unsigned long)g4_io_rx_total(), g4_pwr_level_text());
        }

        if (g_retry_cnt > G4_AT_SYNC_RETRY_MAX)
        {
            /* 敲了这么久还不回 —— 按【累计收到】分两种完全不同的故障:
             *   累计=0 -> 模组一个字节都没发过: 供电/TX-RX 接反/波特率/没起来
             *   累计>0 -> 它会说话, 是内容或波特率对不上(看上面的 [G4_RX]) */
            log_info("[G4] !! 敲了 %u 次 AT(%us) 都不回, 累计收到=%lu 字节, PA8=%s\r\n",
                     (unsigned)(g_retry_cnt - 1u),
                     (unsigned)(((g_retry_cnt - 1u) * G4_AT_PROBE_MS) / 1000u),
                     (unsigned long)g4_io_rx_total(), g4_pwr_level_text());
            if (g4_io_rx_total() == 0u)
            {
                log_info("[G4] !! 模组一个字节都没发过 -> 查供电(3.4~4.2V 且不跳)/"
                         "VCC 处 470uF+10uF+100nF / PC8-PC9 是否接反 / TX 空闲电平\r\n");
            }
            else
            {
                log_info("[G4] !! 它会说话但听不懂 -> 优先查波特率, 再对比上面的 [G4_RX] 原始行\r\n");
            }
            /* 模组已经不理人了: 继续敲 AT 是白费, 直接 4G_RST 断电重启它 */
            g4_link_recover("模组长时间不应答 AT");
        }
        break;
    }

    case G4_STATE_ATE0:
    {
        /* 子步骤: ATE0 -> ATI -> CGMR -> CFUN? (-> CFUN=1), 每条都是一个非阻塞命令 */
        g4_cmd_res_t r;

        switch (g_step)
        {
        case 0u:
            /* 关回显只是"锦上添花": 成不成功都往下走, 不在这里耗重试机会。
             * (ml307_app.c 里写的是 ATE1=开回显 —— 那是给"靠回显做同步"的解析器用的;
             *  我们按关键字匹配应答, 回显只会把日志刷满, 所以用 ATE0。) */
            r = g4_io_cmd_run("ATE0", "OK", AT_CMD_TIMEOUT_MS);
            if (r == G4_CMD_RUNNING) { break; }
            if (r != G4_CMD_OK)
            {
                log_info("[G4] ATE0 no OK(continue)\r\n");
            }
            g_step = 1u;
            break;

        case 1u:
            r = g4_io_cmd_run("ATI", "OK", AT_CMD_TIMEOUT_MS);
            if (r == G4_CMD_RUNNING) { break; }
            if ((r == G4_CMD_OK) && (g4_io_cmd_resp()[0] != '\0'))
            {
                log_info("[G4] 模组 ATI: %s\r\n", g4_io_cmd_resp());
            }
            g_step = 2u;
            break;

        case 2u:
            r = g4_io_cmd_run("AT+CGMR", "OK", AT_CMD_TIMEOUT_MS);
            if (r == G4_CMD_RUNNING) { break; }
            if ((r == G4_CMD_OK) && (g4_io_cmd_resp()[0] != '\0'))
            {
                log_info("[G4] 固件版本 AT+CGMR: %s\r\n", g4_io_cmd_resp());
            }
            g_step = 3u;
            break;

        case 3u:
            /* ------------------------------------------------------------------
             * 【CFUN: 模组的射频功能档位 —— 排"能读卡、能应答、就是不上网"必查】
             *
             * 现场现象: AT+CPIN? 回 READY(卡读得到)、ATI/CGSN/CIMI/ICCID 全正常,
             * 但 AT+CSQ 回 99,99(无信号) 且 AT+CEREG? 回 0,0
             * —— 注意 0,0 是"没注册【也没有在搜网】", 跟"在搜但搜不到(0,2)"
             *    完全不同。这个组合除了"没信号/供电塌"之外, 还有一个非常容易被
             *    忽略的原因: 模组被留在 AT+CFUN=0(最小功能, 射频关着), 这时它
             *    【根本不会去搜网】, 于是永远 CSQ=99 / CEREG=0,0。
             *
             * 谁会把它留在 0: 上一轮实验敲过 AT+CFUN=0、异常掉电、或者模组自己
             * 因为欠压保护进了最小功能。CFUN 在有些固件里是【掉电保持】的,
             * 所以"之前能连、现在不能"也可能只是这一档。
             *
             * 处理: 读一次 AT+CFUN?, 把值打进日志; 是 0 就发 AT+CFUN=1 打开射频。
             * 注意用 AT+CFUN=1(不带第二个参数) —— 那是"设为全功能, 不复位";
             * AT+CFUN=1,1 会复位模组, 这里不要用。
             * ------------------------------------------------------------------ */
            r = g4_io_cmd_run("AT+CFUN?", "+CFUN:", AT_CMD_TIMEOUT_MS);
            if (r == G4_CMD_RUNNING) { break; }

            if (r == G4_CMD_OK)
            {
                int     v[2];
                uint8_t nv = at_parse_ints(g4_io_cmd_resp(), v, 2u);

                g_cfun = (nv > 0u) ? (int8_t)v[0] : -1;

                if (g_cfun == 0)
                {
                    log_info("[G4] !! CFUN=0 —— 模组处于【最小功能/射频关闭】, 它不会去搜网!\r\n");
                    log_info("[G4] !! 这正好能解释 CSQ=99 + CEREG 0,0(没注册也没在搜); "
                             "下面发 AT+CFUN=1 把射频打开\r\n");
                    g_cfun_need_on = true;
                }
                else
                {
                    log_info("[G4] CFUN=%d (射频正常开启, 1=全功能)\r\n", (int)g_cfun);
                }
            }
            else
            {
                log_info("[G4] CFUN 读不到(模组不支持或没应答, continue); "
                         "若一直连不上, 手工敲: #at AT+CFUN?\r\n");
                g_cfun = -1;
            }
            g_step = 4u;
            break;

        case 4u:
            if (!g_cfun_need_on)
            {
                g4_goto_state(G4_STATE_MLPMCFG);
                break;
            }
            r = g4_io_cmd_run("AT+CFUN=1", "OK", 5000u);
            if (r == G4_CMD_RUNNING) { break; }

            if (r == G4_CMD_OK)
            {
                log_info("[G4] 已切到 CFUN=1(全功能, 射频开) —— 稍后看 CSQ/CEREG 有没有起来\r\n");
                g_cfun = 1;
            }
            else
            {
                log_info("[G4] !! AT+CFUN=1 没回 OK —— 手工敲 #at AT+CFUN=1 再看\r\n");
            }
            g_cfun_need_on = false;
            g4_goto_state(G4_STATE_MLPMCFG);
            break;

        default:
            g4_goto_state(G4_STATE_MLPMCFG);
            break;
        }
        break;
    }

    case G4_STATE_MLPMCFG:
    {
        /* 关掉模组默认打开的深睡眠(ml307_app.c: AT+MLPMCFG="sleepmode",0,1)。
         * 深睡眠下模组会周期性不应答 AT, 表现就是"时不时收不到云端下行"。
         * 失败不影响联网, 继续往下。 */
        g4_cmd_res_t r = g4_io_cmd_run("AT+MLPMCFG=\"sleepmode\",0,1", "OK",
                                       AT_CMD_TIMEOUT_MS);
        if (r == G4_CMD_RUNNING) { break; }
        if (r != G4_CMD_OK)
        {
            log_info("[G4] MLPMCFG no OK(continue)\r\n");
        }
        g4_goto_state(G4_STATE_CPIN);
        break;
    }

    case G4_STATE_CPIN:
    {
        /* SIM 是否就绪只查一次; 不 READY 也照常往下试 —— 后面注册超时会走恢复流程 */
        g4_cmd_res_t r = g4_io_cmd_run("AT+CPIN?", "+CPIN:", AT_CMD_TIMEOUT_MS);
        if (r == G4_CMD_RUNNING) { break; }

        if (r == G4_CMD_OK)
        {
            const char *resp = g4_io_cmd_resp();

            if (strstr(resp, "READY") != NULL)
            {
                log_info("[G4] SIM ready\r\n");
            }
            else
            {
                /* 没插卡/需要 PIN 时注册必然失败, 但还是走完整条流程看日志 */
                log_info("[G4] SIM not READY: %s\r\n", resp);
            }
        }
        else
        {
            log_info("[G4] CPIN no resp(continue)\r\n");
        }
        g4_goto_state(G4_STATE_CGSN);
        break;
    }

    case G4_STATE_CGSN:
    {
        /* AT+CGSN=1 读 IMEI: 应答 "+CGSN: <imei>" */
        g4_cmd_res_t r = g4_io_cmd_run("AT+CGSN=1", "+CGSN:", AT_CMD_TIMEOUT_MS);
        if (r == G4_CMD_RUNNING) { break; }

        if ((r == G4_CMD_OK) &&
            (at_take_digits(g4_io_cmd_resp(), g_info.imei, G4_IMEI_LEN) >= 14u))
        {
            log_info("[G4] IMEI=%s\r\n", g_info.imei);
        }
        else
        {
            g_info.imei[0] = '\0';
            /* ★2026-10-08: clientID 已不再默认用 IMEI(现在固定 "110001")★,
             * 所以这里只说"IMEI 读不到"(它仍然影响: 诊断日志、以及 clientId 配成
             * 特殊值 "IMEI" 时的取值)。 */
            log_info("[G4] IMEI 读不到(不影响 clientID=%s)\r\n", g_client_id);
        }
        g4_goto_state(G4_STATE_CIMI);
        break;
    }

    case G4_STATE_CIMI:
    {
        /* AT+CIMI 的应答【就是 IMSI 本身】: 一行 15 位数字 —— 关键字只能用来
         * 判定"收完了", 所以用 "OK", 真正的 IMSI 从累积应答里取第一段数字。 */
        g4_cmd_res_t r = g4_io_cmd_run("AT+CIMI", "OK", AT_CMD_TIMEOUT_MS);
        if (r == G4_CMD_RUNNING) { break; }

        if ((r == G4_CMD_OK) &&
            (at_take_digits(g4_io_cmd_resp(), g_info.imsi, G4_IMSI_LEN) >= 14u))
        {
            log_info("[G4] IMSI=%s\r\n", g_info.imsi);
        }
        else
        {
            g_info.imsi[0] = '\0';
            log_info("[G4] IMSI none\r\n");
        }
        g4_goto_state(G4_STATE_ICCID);
        break;
    }

    case G4_STATE_ICCID:
    {
        /* ML307R 是 AT*ICCID?, 个别固件是 AT+ICCID?, 两个都试一次, 都不行就算了 */
        g4_cmd_res_t r;

        if (g_step == 0u)
        {
            r = g4_io_cmd_run("AT*ICCID?", "ICCID", AT_CMD_TIMEOUT_MS);
            if (r == G4_CMD_RUNNING) { break; }

            if ((r == G4_CMD_OK) &&
                (at_take_digits(g4_io_cmd_resp(), g_info.iccid, G4_ICCID_LEN) >= 18u))
            {
                log_info("[G4] ICCID=%s\r\n", g_info.iccid);
                g4_goto_state(G4_STATE_CSQ);
                break;
            }
            g_step = 1u;            /* 换第二条命令再试 */
            break;
        }

        r = g4_io_cmd_run("AT+ICCID?", "ICCID", AT_CMD_TIMEOUT_MS);
        if (r == G4_CMD_RUNNING) { break; }

        if ((r == G4_CMD_OK) &&
            (at_take_digits(g4_io_cmd_resp(), g_info.iccid, G4_ICCID_LEN) >= 18u))
        {
            log_info("[G4] ICCID=%s\r\n", g_info.iccid);
        }
        else
        {
            g_info.iccid[0] = '\0';
            log_info("[G4] ICCID none\r\n");
        }
        g4_goto_state(G4_STATE_CSQ);
        break;
    }

    case G4_STATE_CSQ:
    {
        /* AT+CSQ → "+CSQ: <rssi>,<ber>"; rssi 99 = 无信号/未知 */
        g4_cmd_res_t r = g4_io_cmd_run("AT+CSQ", "+CSQ:", AT_CMD_TIMEOUT_MS);
        if (r == G4_CMD_RUNNING) { break; }

        if (r == G4_CMD_OK)
        {
            char num[8];

            if (at_take_digits(g4_io_cmd_resp(), num, (uint8_t)sizeof(num)) > 0u)
            {
                uint8_t csq = (uint8_t)atoi(num);

                if ((csq > 31u) || (csq == 99u))
                {
                    g_info.csq      = 99u;
                    g_info.rssi_dbm = 0;
                    log_info("[G4] CSQ=99 (no signal)\r\n");
                }
                else
                {
                    g_info.csq      = csq;
                    g_info.rssi_dbm = (int16_t)(-113 + 2 * (int)csq);
                    log_info("[G4] CSQ=%u (%d dBm)\r\n",
                             (unsigned)g_info.csq, (int)g_info.rssi_dbm);
                }
            }
        }
        else
        {
            log_info("[G4] CSQ none\r\n");
        }

        /* AT+QENG="servingcell" 读 RSRP —— CSQ=99 不可靠(99 既可能真无信号,
         * 也可能模组还没测到/射频被 AT+CFUN=0 关着), RSRP 是 LTE 参考信号接收功率,
         * 比 CSQ 更能定位"到底是没信号还是模组没在测"。命令不支持/没注册也没关系,
         * 只打日志不拦建链(同 CSQ 做法)。响应格式因模组而异, 原样打进日志,
         * 现场 RSRP/RSRQ/RSSI 数值直接看原文(通常在 LTE 模式响应的倒数第 3~4 字段)。 */
        {
            char qeng[160];
            if (g4_io_send_expect_resp("AT+QENG=\"servingcell\"", "+QENG:",
                                       AT_CMD_TIMEOUT_MS, NULL,
                                       qeng, (uint16_t)sizeof(qeng)))
            {
                log_info("[G4] QENG servingcell: %s\r\n", qeng);
            }
            else
            {
                log_info("[G4] QENG servingcell none (未注册或模组不支持)\r\n");
            }
        }

        /* 顺手读一次模组自测供电(AT+MCHIPINFO="vbat", 兜底 AT+CBC)和模组温度:
         * 现场"供电稳不稳、模组是不是过热重启"看这两行就够。命令不支持也没关系,
         * g4_read_vbat()/g4_read_temp() 自己会打日志然后返回, 不影响建链。
         * (这两个是阻塞封装, 但内部跑的是同一个收包泵, 所以不会丢接收。) */
        (void)g4_read_vbat();
        g4_read_temp();

        /* 【回到"等注册"这条主路 —— 拨号(MDIALUP)被证明是帮倒忙, 已撤下】
         * 现场日志的铁证: 执行 AT+MDIALUP=1,1 之后, 模组的 AT 引擎就乱了 ——
         * 之后每条命令只回 "+MATREADY" + "OK", 连 +CEREG: 都不给。
         * 现场那份成功率很高的 ml307_app.c 从头到尾没调过 MDIALUP, 走的就是
         * 标准 3GPP: CREG 等注册 → CGDCONT → CGACT → CGPADDR。 */
        g4_goto_state(G4_STATE_CREG);
        break;
    }

    case G4_STATE_CREG:
    {
        const char  *cmd;
        const char  *tag;
        uint8_t      stat = 0xFFu;
        g4_cmd_res_t r;

        /* ---- 注册被拒(stat==3)之后的恢复: 发 CFUN 软复位 -> 等 10s -> 回 AT_SYNC ----
         * 【为什么不用 delay_ms(10000)】: 那 10 秒里没人收 USART2, 云端下行会把
         * 环形缓冲顶爆 —— 正是"一下发就出事"的成因之一。现在只是"这一拍什么都不做"。 */
        if (g_step >= 2u)
        {
            if (g_step == 2u)
            {
                r = g4_io_cmd_run("AT+CFUN=1,1", "OK", 8000u);
                if (r == G4_CMD_RUNNING) { break; }
                g_step      = 3u;
                g_wait_tick = now;
                break;
            }
            if ((uint32_t)(now - g_wait_tick) >= 10000u)
            {
                g4_goto_state(G4_STATE_AT_SYNC);
            }
            break;
        }

        /* 【限速】没到下一次查询时间就什么都不发(进入本状态时 g_retry_cnt=0,
         * 所以第一条注册查询立刻发; 之后每 AT_CEREG_POLL_MS 一条)。
         * ⚠️ 只有 g_step==0(没有在途命令)时才允许被限速挡住, 否则会把自己
         *    在途的那条命令也一起"限"住, 永远收不到应答。 */
        if ((g_step == 0u) && (g_retry_cnt != 0u) &&
            ((now - g_state_tick) < AT_CEREG_POLL_MS))
        {
            break;
        }

        if (g_step == 0u)
        {
            g_state_tick = now;     /* 记住这次查询时刻, 管住下一次 */
            g_retry_cnt++;
            g_step = 1u;
        }

        cmd = reg_query_cmd(g_reg_probe);
        tag = reg_query_tag(g_reg_probe);

        r = g4_io_cmd_run(cmd, tag, AT_CMD_TIMEOUT_MS);
        if (r == G4_CMD_RUNNING) { break; }
        g_step = 0u;                /* 这一拍结束, 下一拍按限速再发 */

        if (r == G4_CMD_OK)
        {
            stat              = at_reg_stat(g4_io_cmd_resp(), tag);
            g_info.reg_status = stat;

            /* 1=已注册(本地) 5=已注册(漫游) */
            if ((stat == 1u) || (stat == 5u))
            {
                log_info("[G4] registered (%s stat=%u)\r\n", cmd, (unsigned)stat);
                g_pdp_retry    = 0u;
                g_dialup_tried = false;
                apn_build_candidates();   /* 注册上了 -> 按 IMSI 定第一个 APN 候选 */
                g4_goto_state(G4_STATE_CGDCONT);
                break;
            }
            /* 0=没注册也没在搜 / 2=没注册但在搜 / 3=被拒 —— 前两种都是"再等等" */
        }
        else if (r == G4_CMD_ERR)
        {
            /* 模组【明确回 ERROR】才当不支持这条命令 -> 换另一种注册域 */
            log_info("[G4] %s 不支持(模组回了 ERROR) -> 换下一种\r\n", cmd);
            g_reg_probe++;
            if (g_reg_probe > 2u)
            {
                g_reg_probe = 0u;
            }
        }
        else
        {
            /* 【一句应答都没有 ≠ 不支持 —— 这是之前踩过的误判】
             * 现场实测: 发 AT+CEREG? 回来的却是 "+MATREADY"(模组刚重启), 于是这条
             * 命令"没有应答", 老代码直接判成不支持就把查询方式换掉了。
             * 现在: 没应答只当"这一拍没问到", 【保持同一种查询方式】按节奏重试。 */
            log_info("[G4] %s 无应答(模组可能正在起动/重新入网, 不是不支持) -> 稍后重试\r\n",
                     cmd);
        }

        if (g_retry_cnt >= AT_CEREG_POLL_MAX)
        {
            /* 【注册查询超时 ≠ 不能上网 —— 不再发 CFUN=1,1 去软复位模组】
             * 原来这里是 CFUN + 等 10s + 从头握手: 那等于把模组刚攒起来的入网进度
             * 又清掉一次。而且 3GPP 里"注册不上"不等于"PDP 起不来" —— 有些网络/卡
             * 就是 CEREG 报得慢, 但 CGDCONT/CGACT 照样能激活并分到 IP。
             * 所以现在: 报清楚, 然后【照常往下走】去 CGDCONT/CGACT/CGPADDR 试,
             * 拿不拿到 IP 才是唯一判据 —— 这正是 ml307_app.c 的做法。 */
            log_info("[G4] 注册查询超时 %us(最后 stat=%u '%s') -> 不再复位模组, "
                     "继续去激活 PDP 试\r\n",
                     (unsigned)((AT_CEREG_POLL_MAX * AT_CEREG_POLL_MS) / 1000u),
                     (unsigned)stat, g4_reg_stat_text(stat));
            log_info("[G4] (提示: 注册状态难看时, 先看 VBAT 和 CSQ —— "
                     "欠压/无信号才是常见原因)\r\n");
            g4_goto_state(G4_STATE_CGDCONT);
            break;
        }

        if ((g_retry_cnt % 5U) == 0U)
        {
            /* 每 5 次(10s)报一次进度就够了, 免得日志被"还没注册"刷屏。
             * 【顺便刷新一次模组自测供电】: 注册阶段正是模组"使劲搜网/发射"的时候,
             * 供电最容易塌 —— 现场就是靠这一行看到 VBAT 掉到 2.5V 的。 */
            log_info("[G4] wait reg %us(stat=%u CSQ=%u VBAT=%umV)...\r\n",
                     (unsigned)((g_retry_cnt * AT_CEREG_POLL_MS) / 1000u),
                     (unsigned)stat, (unsigned)g_info.csq, (unsigned)g_info.vbat_mv);
            (void)g4_read_vbat();       /* 读一次并更新状态行/上面的值 */
        }

        /* 【注册被拒 = stat 3】: 网络明确说"不让你注册", 和"还在搜"完全不同。
         * 但先看供电 —— 模组在欠压时也会报 3(它压根没力气把入网流程走完)。 */
        if (stat == 3u)
        {
            log_info("[G4] !! 注册被【拒绝】(stat=3) —— 不是没信号, 是网络不接受\r\n");
            log_info("[G4] !! 先看供电: 模组自测 VBAT=%umV (正常要 3400mV 以上)——"
                     "欠压时模组会一直重来, 表现就包括 stat=3\r\n",
                     (unsigned)g_info.vbat_mv);
            log_info("[G4] !! 供电正常再查: SIM 是否欠费/停机 / 该卡是否被限了 4G 权限 / "
                     "模组是否只认某个网络制式\r\n");
            g_step = 2u;        /* 下一拍开始: CFUN 软复位 -> 等 10s -> 回 AT_SYNC */
        }
        break;
    }

    case G4_STATE_CGDCONT:
    {
        /* 设 APN(ml307_app.c: AT+CGDCONT=1,"IP","CMNET")。失败(比如已经设过)
         * 不影响后续, 继续去激活 PDP。 */
        g4_cmd_res_t r;

        if (g_step == 0u)
        {
            /* 【先清旧 PDP 上下文, 再设 APN】
             * MQTT/TCP 异常断开、或者上一轮换 APN 试错之后, 同一个 CID 的 PDP
             * 上下文常常留着"脏"状态。直接再 CGDCONT/CGACT 会偶发激活失败, 严重时
             * 射频侧卡住 —— 表现就是 AT 还能应答、但 CSQ=99 搜不到网。
             * AT+CGACT=0,1 是标准 3GPP 的"去激活 CID 1", 先放一条把上下文放掉。
             * 本来就没激活时它可能回 ERROR 或没 OK, 都【不影响流程】, 只看日志。 */
            r = g4_io_cmd_run("AT+CGACT=0,1", "OK", AT_PDP_DEACT_TIMEOUT_MS);
            if (r == G4_CMD_RUNNING) { break; }

            if (r == G4_CMD_OK)
            {
                log_info("[G4] 旧 PDP 上下文已去激活(CGACT=0,%u)\r\n",
                         (unsigned)G4_PDP_CID);
            }
            else
            {
                log_info("[G4] CGACT=0,%u 无 OK(本来没激活时正常), 继续\r\n",
                         (unsigned)G4_PDP_CID);
            }
            g_step = 1u;
            break;
        }

        if (g_step == 1u)
        {
            const char *apn = g_apn_list[g_apn_idx];

            snprintf(g_tx_cmd, sizeof(g_tx_cmd), "AT+CGDCONT=%u,\"IP\",\"%s\"",
                     (unsigned)G4_PDP_CID, apn);
            g_step = 2u;
        }

        r = g4_io_cmd_run(g_tx_cmd, "OK", AT_CMD_TIMEOUT_MS);
        if (r == G4_CMD_RUNNING) { break; }

        if (r == G4_CMD_OK)
        {
            log_info("[G4] APN %u/3: %s\r\n",
                     (unsigned)(g_apn_idx + 1u), g_apn_list[g_apn_idx]);
        }
        else
        {
            log_info("[G4] APN no OK(continue)\r\n");
        }
        g4_goto_state(G4_STATE_CGACT);
        break;
    }

    case G4_STATE_CGACT:
    {
        /* 激活 PDP: 首次入网可能要十几秒, 给 15s(和 ml307_app.c 一致) */
        g4_cmd_res_t r;

        if (g_step == 0u)
        {
            snprintf(g_tx_cmd, sizeof(g_tx_cmd), "AT+CGACT=1,%u", (unsigned)G4_PDP_CID);
            g_step = 1u;
        }

        r = g4_io_cmd_run(g_tx_cmd, "OK", AT_PDP_TIMEOUT_MS);
        if (r == G4_CMD_RUNNING) { break; }

        if (r == G4_CMD_OK)
        {
            log_info("[G4] PDP activated\r\n");
        }
        else
        {
            log_info("[G4] %s no OK, check IP\r\n", g_tx_cmd);
        }
        g4_goto_state(G4_STATE_CGPADDR);
        break;
    }

    case G4_STATE_CGPADDR:
    {
        /* 【拿 IP 才是真的通了】: CGACT 回 OK 只说明命令被接受, PDP 是否真起来
         * 要看 CGPADDR(ml307_app.c 里也是用空 IP 判 PDP 失败)。 */
        g4_cmd_res_t r;

        /* 换 APN 之前等 1s(原来是 delay_ms(1000), 现在只算时间戳, 不阻塞) */
        if (g_step == 5u)
        {
            if ((uint32_t)(now - g_wait_tick) >= 1000u)
            {
                g4_goto_state(G4_STATE_CGDCONT);
            }
            break;
        }

        if (g_step == 0u)
        {
            snprintf(g_tx_cmd, sizeof(g_tx_cmd), "AT+CGPADDR=%u", (unsigned)G4_PDP_CID);
            g_step = 1u;
        }

        r = g4_io_cmd_run(g_tx_cmd, "+CGPADDR:", AT_CMD_TIMEOUT_MS);
        if (r == G4_CMD_RUNNING) { break; }
        g_step = 0u;

        if ((r == G4_CMD_OK) &&
            at_take_ipv4(g4_io_cmd_resp(), g_info.ip, G4_IP_LEN))
        {
            g_info.pdp_ok = true;
            log_info("[G4] PDP got IP: %s\r\n", g_info.ip);
            g4_goto_state(G4_STATE_AFTER_IP);
            break;
        }

        g_pdp_retry++;
        if (g_pdp_retry < G4_PDP_ADDR_RETRY_MAX)
        {
            /* 拿不到 IP 最常见的原因就是 APN 不对(尤其换运营商的卡):
             * 换下一家的 APN, 重新 CGDCONT -> CGACT -> CGPADDR 走一遍 */
            g_apn_idx = (uint8_t)((g_apn_idx + 1u) % 3u);
            log_info("[G4] no IP -> 换 APN %s\r\n", g_apn_list[g_apn_idx]);
            g_step      = 5u;           /* 先等 1s 再回 CGDCONT */
            g_wait_tick = now;
            break;
        }

        /* 【注意】拨号在开局就已经试过了, 走到这里说明两条路都没拿到 IP ——
         * 再回去拨一次没有意义, 直接进 ERROR 走恢复流程。 */
        log_info("[G4] no IP(拨号与 PDP 都失败) -> rebuild\r\n");
        g4_goto_state(G4_STATE_ERROR);
        break;
    }

    case G4_STATE_DIALUP:
    {
        /* 【说明】现场最终走的是"标准 3GPP: CEREG -> CGDCONT -> CGACT -> CGPADDR"
         * 这条主路(见 G4_STATE_AFTER_IP), G4_STATE_DIALUP 目前【没有任何地方会
         * 切进来】—— 保留它只是为了台架手工核对 AT+MDIALUP 的行为(日志口 #dial)。
         * 这里同样改成非阻塞子步骤, 免得哪天有人把它接回主路时又带进 delay_ms。 */
        g4_cmd_res_t r;

        if (g_step == 3u)       /* 拨号之后等 3s 再查结果 */
        {
            if ((uint32_t)(now - g_wait_tick) >= 3000u)
            {
                g_step = 4u;
            }
            break;
        }

        if (g_step == 0u)
        {
            r = g4_io_cmd_run("AT+MDIALUP?", "+MDIALUP:", AT_CMD_TIMEOUT_MS);
            if (r == G4_CMD_RUNNING) { break; }

            if ((r == G4_CMD_OK) &&
                at_take_v4_quoted(g4_io_cmd_resp(), g_info.ip, G4_IP_LEN))
            {
                g_info.pdp_ok = true;
                log_info("[G4] 拨号已在连接状态(AT+MDIALUP?), IP=%s\r\n", g_info.ip);
                g4_goto_state(G4_STATE_REG_CHECK);
                break;
            }
            g_step = 1u;
            break;
        }

        if (g_step == 1u)
        {
            r = g4_io_cmd_run("AT+MDIALUP=1,1", "OK", AT_DIALUP_TIMEOUT_MS);
            if (r == G4_CMD_RUNNING) { break; }

            if (r == G4_CMD_OK)
            {
                log_info("[G4] AT+MDIALUP=1,1 已下发, 等拨号结果...\r\n");
            }
            else
            {
                log_info("[G4] AT+MDIALUP=1,1 没回 OK(第 %u 次)\r\n",
                         (unsigned)(g_retry_cnt + 1u));
            }
            g_step      = 3u;       /* 给它 3s 让 URC 回来 */
            g_wait_tick = now;
            break;
        }

        /* g_step == 4: 再查一次拨号状态 */
        r = g4_io_cmd_run("AT+MDIALUP?", "+MDIALUP:", AT_CMD_TIMEOUT_MS);
        if (r == G4_CMD_RUNNING) { break; }
        g_step = 0u;

        if ((r == G4_CMD_OK) &&
            at_take_v4_quoted(g4_io_cmd_resp(), g_info.ip, G4_IP_LEN))
        {
            g_info.pdp_ok = true;
            log_info("[G4] 拨号成功: IP=%s (第 %u 次)\r\n",
                     g_info.ip, (unsigned)(g_retry_cnt + 1u));
            g4_goto_state(G4_STATE_REG_CHECK);
            break;
        }

        g_retry_cnt++;
        if (g_retry_cnt <= G4_DIALUP_RETRY_MAX)
        {
            log_info("[G4] 拨号没拿到 IP(第 %u/%u 次), 下一轮再试\r\n",
                     (unsigned)g_retry_cnt, (unsigned)G4_DIALUP_RETRY_MAX);
            g_step      = 3u;
            g_wait_tick = now;      /* 也等 2~3s, 不阻塞主循环 */
            break;
        }

        log_info("[G4] !! 拨号 %u 次都没拿到 IP -> 退回老流程(CEREG/CGDCONT/CGACT/CGPADDR)\r\n",
                 (unsigned)G4_DIALUP_RETRY_MAX);
        log_info("[G4] !! 提示: 拨号失败多半还是网络侧(注册被拒/无信号/SIM 状态), "
                 "先看下面 CEREG 的 stat 和 VBAT\r\n");
        g4_goto_state(G4_STATE_CREG);
        break;
    }

    case G4_STATE_REG_CHECK:
    {
        /* 【拨号之后的诊断检查 —— 只查、只打印, 不当闸门】
         * 拨号已经把 IP 拿到手了, 所以无论 stat 是什么都继续往下走(去连 MQTT)。 */
        g4_cmd_res_t r;

        if (g_step == 0u)
        {
            r = g4_io_cmd_run("AT+CEREG?", "+CEREG:", AT_CMD_TIMEOUT_MS);
            if (r == G4_CMD_RUNNING) { break; }

            if (r == G4_CMD_OK)
            {
                g_info.reg_status = at_reg_stat(g4_io_cmd_resp(), "+CEREG:");
                log_info("[G4] GSM/PS 注册状态 AT+CEREG? stat=%u (%s) | CSQ=%u IP=%s VBAT=%umV\r\n",
                         (unsigned)g_info.reg_status,
                         g4_reg_stat_text(g_info.reg_status),
                         (unsigned)g_info.csq,
                         (g_info.ip[0] != '\0') ? g_info.ip : "-",
                         (unsigned)g_info.vbat_mv);
                g_step = 2u;
                break;
            }
            g_step = 1u;
            break;
        }

        if (g_step == 1u)
        {
            r = g4_io_cmd_run("AT+CGREG?", "+CGREG:", AT_CMD_TIMEOUT_MS);
            if (r == G4_CMD_RUNNING) { break; }

            if (r == G4_CMD_OK)
            {
                g_info.reg_status = at_reg_stat(g4_io_cmd_resp(), "+CGREG:");
            }
            log_info("[G4] GSM/PS 注册状态 AT+CGREG? stat=%u (%s) | CSQ=%u IP=%s VBAT=%umV\r\n",
                     (unsigned)g_info.reg_status,
                     g4_reg_stat_text(g_info.reg_status),
                     (unsigned)g_info.csq,
                     (g_info.ip[0] != '\0') ? g_info.ip : "-",
                     (unsigned)g_info.vbat_mv);
            g_step = 2u;
            break;
        }

        (void)g4_read_vbat();       /* 顺手刷新供电, 低压会自己告警 */
        g4_goto_state(G4_STATE_AFTER_IP);
        break;
    }

    case G4_STATE_MQTT_DISC:
    {
        /* 清理残留连接, 无论成败继续。
         * 关键字传 NULL: 引擎只等 G4_MQTT_DISC_WAIT_MS 让模组消化一下,
         * 到点返回 TIMEOUT, 正是我们要的"等一会儿就往下走"。 */
        g4_cmd_res_t r = g4_io_cmd_run_silent("AT+MQTTDISC=" G4_MQTT_CONNECT_ID,
                                              G4_MQTT_DISC_WAIT_MS);
        if (r == G4_CMD_RUNNING) { break; }
        /* 【先配模组参数, 再建连接】: 手册要求 MQTTCFG 系列在"MQTT 连接未创建"时
         * 下发 —— 此刻刚 DISC 完, 正是这个条件。 */
        g4_goto_state(G4_STATE_MQTT_CFG);
        break;
    }

    case G4_STATE_MQTT_CFG:
    {
        /* ---- 把【模组自己的能力】配好(2026-10-04 第 2 步) ----
         * 遗嘱 / keepalive / pingreq / pingresp / reconn 一条条下发。
         * 【每轮建链都重下发】手册明确"模组掉电后配置参数不保存"。
         * 【任何一条失败都只记日志并继续】—— 这是"让模组干得更专业", 不是建链前提;
         * 绝不能因为它连不上 broker。 */
        static char cfg_cmd[256];
        g4_cmd_res_t r;

        if (g_step < (uint8_t)G4_MQTT_CFG_STEPS)
        {
            if (!mqtt_cfg_build((uint8_t)g_step, cfg_cmd, (uint16_t)sizeof(cfg_cmd)))
            {
                g_step++;           /* 这一步不需要发(例如没配遗嘱) -> 跳过 */
                break;
            }

            r = g4_io_cmd_run(cfg_cmd, "OK", AT_CMD_TIMEOUT_MS);
            if (r == G4_CMD_RUNNING) { break; }

            if (r == G4_CMD_OK)
            {
                log_info("[4G] 模组参数已接受: %s\r\n", cfg_cmd);
            }
            else
            {
                log_info("[4G] !! 模组参数未接受(res=%d): %s —— 跳过, 不影响建链\r\n",
                         (int)r, cfg_cmd);
            }
            g_step++;
            break;
        }

        g4_goto_state(G4_STATE_MQTT_CONN);
        break;
    }

    case G4_STATE_MQTT_CONN:
    {
        g4_cmd_res_t r;

        if (g_step == 0u)
        {
            g4_urc_reset_conn_state();   /* 清掉上一次的 conn 结果, 便于失败时看原因 */
            /* 【记下"发这条 MQTTCONN 之前"的 conn URC 序号】: 之后 WAIT_CONN 只认
             * 序号变新了的那条 conn 结果 —— 时序上必须在这里取基准(不是在收到 OK
             * 之后), 否则"命令在途时到达的 conn URC"会被我们当成旧的而漏掉。 */
            s_conn_seq_base = g4_urc_conn_seq();
            if (!mqtt_build_conn_cmd(g_tx_cmd, sizeof(g_tx_cmd)))
            {
                log_info("[G4] MQTTCONN 命令组装失败 -> rebuild\r\n");
                g4_goto_state(G4_STATE_ERROR);
                break;
            }
            g_step = 1u;
        }

        r = g4_io_cmd_run(g_tx_cmd, "OK", AT_CONN_TIMEOUT_MS);
        if (r == G4_CMD_RUNNING) { break; }

        if (r == G4_CMD_OK)
        {
            g4_goto_state(G4_STATE_WAIT_CONN);
            break;
        }

        if (++g_retry_cnt > 3U)
        {
            log_info("[G4] MQTTCONN rejected -> rebuild\r\n");
            g4_goto_state(G4_STATE_ERROR);
        }
        else
        {
            g_step = 0u;                /* 下一拍重新组装再试 */
        }
        break;
    }

    case G4_STATE_WAIT_CONN:
        /* conn URC 由收包泵->g4_urc 维护在线状态, 这里只查结果。
         * ⚠️ 必须【同时】要求"本次 MQTTCONN 之后新到了一条 conn URC"(序号变了)。
         *    只看 g4_urc_is_online() 会被上一轮的在线标志骗过去 —— 现场日志 78.txt
         *    就是这么白烧 30~40 秒的:
         *        [G4] MQTT conn ok (res=255 unknown)   ← 255 = 本次一条 conn URC 都没收到
         *        [G4] -> MQTT_SUB
         *        +CME ERROR: 607 ×4  ->  MQTTSUB fail -> rebuild
         */
        if ((g4_urc_conn_seq() != s_conn_seq_base) && g4_urc_is_online())
        {
            log_info("[G4] MQTT conn ok (res=%u %s)\r\n",
                     (unsigned)g4_urc_get_conn_state(),
                     g4_conn_state_text(g4_urc_get_conn_state()));
            g4_goto_state(G4_STATE_MQTT_SUB);
        }
        else if ((now - g_state_tick) >
                 (g_mqtt_fast_active ? AT_URC_TIMEOUT_FAST_MS : AT_URC_TIMEOUT_MS))
        {
            uint8_t st = g4_urc_get_conn_state();

            /* ★★2026-10-08: 快修路径拿不到 conn URC ⇒ 【立刻断电】, 不再白等★★
             *
             * 【现场现象】模组欠压重启后, MQTTCONN 回 OK 但一条 conn URC 都没有
             *   (既不报连上也不报失败)。这是"模组 MQTT 栈已坏"的特征 ——
             *   轻量的 DISC/CFG/CONN/SUB 重建【救不回来】, 只有整颗模组断电重启才行。
             *   实测: 最后就是靠 4G_RST 断电才重连成功的。
             *
             * 【为什么不再走 G4_STATE_ERROR】那条路会重试, 而重试用的是同一个坏栈,
             *   结果是"60 秒 x 最多 3 次 = 3 分钟"的纯白等。既然已经能判定
             *   "模组栈坏了", 就直接跳到唯一有效的处置 —— 断电重置。
             *
             * 【代价可控】断电恢复要 40~60 秒, 但【一定成功】; 白等 3 分钟然后
             *   照样断电, 是纯亏。 */
            if (g_mqtt_fast_active)
            {
                log_info("[G4] !! 快修在 WAIT_CONN 等了 %us 都没有 conn URC(last=%u/%s)"
                         " —— 模组 MQTT 栈已坏, 轻量重建救不回来, 【直接断电重置模组】\r\n",
                         (unsigned)(AT_URC_TIMEOUT_FAST_MS / 1000u),
                         (unsigned)st, g4_conn_state_text(st));
                g_mqtt_fast_cnt    = 0u;
                g_mqtt_suspect     = false;
                g_mqtt_fast_active = false;
                g4_link_recover("快修在 WAIT_CONN 拿不到 conn URC(模组 MQTT 栈已坏)");
                break;
            }

            log_info("[G4] conn URC timeout, last=%u(%s)\r\n",
                     (unsigned)st, g4_conn_state_text(st));
            g4_goto_state(G4_STATE_ERROR);
        }
        break;

    case G4_STATE_MQTT_SUB:
    {
        /* 订阅 topic 是运行期可变的(见 g4_at_set_sub_topic), 所以这里拼一次命令。
         * 用静态缓冲: g4_io_cmd_run() 在调用期间会用到这个指针。
         * ★2026-10-08: 没有编译期兜底主题了 —— 空 = main.c 没注入, 明确报错并跳过。 */
        static char   subcmd[G4_SUB_TOPIC_MAX + 48];
        g4_cmd_res_t  r;

        if (g_sub_topic[0] == '\0')
        {
            log_info("[G4] !! 下行订阅 topic 为空 —— main.c 没有注入(应为本机 rt/gw/{gwId}/cmd)"
                     ", 【跳过订阅】: 上行仍可发, 但收不到任何云端命令\r\n");
            g4_urc_set_online();
            g4_pub_mark_ok();
            g4_goto_state(G4_STATE_REG_ARM);
            break;
        }

        (void)snprintf(subcmd, sizeof(subcmd), "AT+MQTTSUB=%s,\"%s\",%s",
                       G4_MQTT_CONNECT_ID, g_sub_topic, G4_MQTT_SUB_QOS);

        r = g4_io_cmd_run(subcmd, "OK", AT_CMD_TIMEOUT_MS);
        if (r == G4_CMD_RUNNING) { break; }

        if (r == G4_CMD_OK)
        {
            g4_urc_set_online();
            local_alarm_clear(ALARM_4G_OFFLINE);
            g_fail_cnt = 0u;        /* 真的连上了: 失败预算清零 */
            g_pub_prompt_loss = 0u; /* 新链路: 连续丢 '>' 的计数也清零 */
            g_mqtt_fast_cnt   = 0u; /* 回来了: MQTT 快修连败计数清零 */
            /* ★2026-10-08: 快修标记也在这里清 —— 订阅成功才算"快修真的救回来了"。
             * 放在这里而不是 DISCONN/ONLINE: 万一后来 SUB 失败又走了别的恢复路径,
             * 标记留着不会有害(下次冷启动建链时会重新走 WAIT_CONN 的 60 秒分支吗?
             * 不会 —— 见下面 g4_at_init 里的清零), 但清了更干净。 */
            g_mqtt_fast_active = false;
            g_pub_noack_cnt   = 0u;
            g_mqtt_suspect    = false;
            log_info("===== MQTT ONLINE! =====\r\n");
            /* 保活计时从"上线"这一刻起算, 免得把建链花掉的时间算进停滞窗口 */
            g4_pub_mark_ok();
            /* ★2026-10-08: 这里原来还打了 pub=mod/mod_upload —— 那条通道已删除。
             * 现在打三个真正生效的值: clientID / user / sub(下行主题)。
             * clientID 必须打【实际发出去的那个】—— 配成特殊值 "IMEI" 时它不是 g_client_id,
             * 所以这里按 mqtt_build_conn_cmd 的同一套规则算一遍(否则日志会骗人)。 */
            log_info("[G4] clientID=%s user=%s sub=%s\r\n",
                     ((strcmp(g_client_id, "IMEI") == 0) && (g_info.imei[0] != '\0'))
                         ? g_info.imei
                         : ((g_client_id[0] != '\0') ? g_client_id : G4_MQTT_CLIENT_ID),
                     (g_broker_user[0] != '\0') ? g_broker_user : "-",
                     g_sub_topic);
            log_info("[G4] IMEI=%s IMSI=%s ICCID=%s CSQ=%u IP=%s\r\n",
                     (g_info.imei[0]  != '\0') ? g_info.imei  : "-",
                     (g_info.imsi[0]  != '\0') ? g_info.imsi  : "-",
                     (g_info.iccid[0] != '\0') ? g_info.iccid : "-",
                     (unsigned)g_info.csq,
                     (g_info.ip[0]    != '\0') ? g_info.ip    : "-");
            if (g_info.vbat_valid)
            {
                log_info("[G4] 模组自测供电 = %u mV\r\n", (unsigned)g_info.vbat_mv);
            }
            /* 【去 G4_STATE_REG_ARM 而不是直接 ONLINE】: 上线之前把驻网/PS 变化的
             * 主动上报打开。放在这里(而不是 ONLINE 里)是本工程的一条硬不变式 ——
             * ONLINE 稳态下不许发 AT, 详见 g4_cereg_diag_poll() 上面的长注释。 */
            g4_goto_state(G4_STATE_REG_ARM);
            break;
        }

        if (++g_retry_cnt > 3U)
        {
            log_info("[G4] MQTTSUB fail -> rebuild\r\n");
            g4_goto_state(G4_STATE_ERROR);
        }
        break;
    }

    case G4_STATE_REG_ARM:
    {
        /* ---- 上线前最后一次配置: 让模组主动报"驻网 / PS 去附着"变化 ----
         *
         * 【为什么必须在这里做】
         *   ① 本工程有一条被主机测试钉死的不变式: ONLINE 稳态下不许发任何多余 AT
         *      (test_state.c 的 "ONLINE 稳态下 5 秒内发的 AT 命令数 = 0")。现场日志
         *      也证实"在 ONLINE 里敲 AT"会诱发模组 AT 应用重启(+MATREADY 387 次,
         *      全部落在我们发字节后 8~49ms)并打死 MQTT 会话。所以这两条必须在
         *      上线【之前】发。
         *   ② 手册要求 MQTTCFG 类命令在"MQTT 连接未创建"时下发更稳 —— 此刻还没
         *      MQTTCONN, 正合适。
         *   ③ 每轮建链(MQTT 快修/断电重启之后)都会重新走一遍: 模组自己重启或整链路
         *      断电后这两个设置可能已被清掉。
         *
         * 【失败了绝不影响建链】: 这只是观测手段。发不出去就退化成"没有主动上报",
         * 与加这个功能之前的行为一致 —— 绝不能因为它连不上 broker。
         *
         * 【只读性质】这两条命令只改变"模组报不报", 不改变我们的任何判定逻辑;
         * 本工程此刻还没有任何一处消费 g_cereg.stat(第 2 步才阈值化)。 */
#if (G4_CEREG_DIAG_ENABLE && G4_CEREG_URC_ENABLE)
        {
            g4_cmd_res_t r;

            if (g_step == 0u)
            {
                r = g4_io_cmd_run("AT+CEREG=2", "OK", AT_CMD_TIMEOUT_MS);
                if (r == G4_CMD_RUNNING) { break; }
                if (r == G4_CMD_OK)
                {
                    log_info("[CEREG] AT+CEREG=2 已下发 —— 模组会主动上报驻网变化\r\n");
                }
                else
                {
                    log_info("[CEREG] !! AT+CEREG=2 没成功(res=%d) —— 没有驻网主动上报,"
                             "不影响建链\r\n", (int)r);
                }
                g_step = 1u;
                break;
            }

            if (g_step == 1u)
            {
                r = g4_io_cmd_run("AT+CGEREP=1", "OK", AT_CMD_TIMEOUT_MS);
                if (r == G4_CMD_RUNNING) { break; }
                if (r == G4_CMD_OK)
                {
                    g_cereg.urc_armed = true;
                    log_info("[CEREG] AT+CGEREP=1 已下发 —— PS 去附着会主动上报"
                             "(+CGEV: NW DETACH 是最强的断网证据)\r\n");
                }
                else
                {
                    log_info("[CEREG] !! AT+CGEREP=1 没成功(res=%d) —— 没有 PS 事件上报,"
                             "不影响建链\r\n", (int)r);
                }
                g_step = 2u;
                break;
            }

            if (g_step == 2u)
            {
                /* 【上线前把当前驻网状态量一次】—— 这是唯一一次可靠地拿到 <stat> 的
                 * 机会: 开机之后只有这里能问 (ONLINE 稳态不许发 AT, 见上面的长注释)。
                 * URC 只在"变化时"才来, 没有这个种子值就不知道起点是好是坏。
                 * 用 at_reg_stat() 按【轮询应答形态】解析 (<n>,<stat>)。 */
                r = g4_io_cmd_run("AT+CEREG?", "+CEREG:", AT_CMD_TIMEOUT_MS);
                if (r == G4_CMD_RUNNING) { break; }

                if (r == G4_CMD_OK)
                {
                    uint8_t stat = at_reg_stat(g4_io_cmd_resp(), "+CEREG:");

                    if (stat != 0xFFu)
                    {
                        g_cereg.stat = stat;
                        log_info("[CEREG] 上线前驻网状态: stat=%u (%s)\r\n",
                                 (unsigned)stat, g4_reg_stat_text(stat));
                    }
                    else
                    {
                        log_info("[CEREG] 上线前 AT+CEREG? 应答里没有 stat, 原样: %s\r\n",
                                 g4_io_cmd_resp());
                    }
                }
                else
                {
                    log_info("[CEREG] !! 上线前 AT+CEREG? 没应答(res=%d) —— 驻网状态未知\r\n",
                             (int)r);
                }
                g_step = 3u;
                break;
            }
        }
#endif
        g4_goto_state(G4_STATE_ONLINE);
        break;
    }

    case G4_STATE_ONLINE:
    {
        /* 【先看: 在线期间模组偷偷重启过没有?】
         * 现场证据(日志 78.txt):
         *   1) 每一次 "+MATREADY" 都紧跟在我们的命令之后(距上次发送 10~41ms);
         *   2) 每个会话都是"第 1 条 MQTTPUB 被收下但 broker 不回 puback, 第 2 条起回 601";
         *   3) 模组【不发 conn URC】, 所以状态机一直以为还在线。
         * 结论: 模组 AT 应用层一重启, MQTT 会话就没了, 而且没有任何人告诉我们。
         * 处理: 标记"会话可疑"并大声报出来 —— 下一次发布只要失败或拿不到 puback,
         *       pub_poll 里就会走 MQTT 快修(2~5 秒), 而不是攒够失败去断电(40~60 秒)。 */
        if (g4_io_ready_total() != g_ready_base_online)
        {
            g_ready_base_online = g4_io_ready_total();
            g_mqtt_suspect      = true;
            log_info("[G4] !! ONLINE 期间模组报出 \"+MATREADY\"(AT 应用层重启过, 累计 %lu 次)"
                     " —— MQTT 会话多半已经没了, 标记【可疑】; 下一次发布失败就走 MQTT 快修\r\n",
                     (unsigned long)g_ready_base_online);
        }

        /* 【保活停滞看门狗】: 详见 g4_at_set_link_watchdog() 的说明。
         * 窗口由 main.c 按实际心跳周期设定(心跳开着 = 5 × 周期, 关着 = 0 即不用)。
         * ⚠️ 【必须用有符号比较 —— 这里踩过一次, 就是"中间那次自动重启"的原因】
         * 无符号相减在 tick 反超时会得到 0xFFFFFFFF(≈4294967 秒), 于是"保活停滞"
         * 当场误报, 链路被无端重建。
         * ★2026-10-04 第 2 步: 模组正在自己重连(conn state=1)时【不开这条枪】★
         *   模组此刻是在干活的, 而且它配了 reconn(默认 20/40/60s)。看门狗这时候
         *   开火, 等于在它刚要回来时又把它按下去 —— 现场那 27 次断电里有一大半
         *   就是这么来的。让开的时限由 G4_MQTT_RECONN_GRACE_MS 兜住(超过它
         *   g4_at_poll 里的 conn 分支会走 MQTT 快修)。 */
        if ((g_hb_stall_ms != 0u) && g_last_pub_ok_valid &&
            (g_mqtt_reconn_since == 0u) &&
            ((int32_t)((uint32_t)now - g_last_pub_ok_tick) > (int32_t)g_hb_stall_ms))
        {
            log_info("[G4] !! 保活停滞: 已经 %lus 没有一次发布成功"
                     "(窗口 %lus) -> 判链路已死\r\n",
                     (unsigned long)(((uint32_t)now - g_last_pub_ok_tick) / 1000u),
                     (unsigned long)(g_hb_stall_ms / 1000u));
            log_info("[G4] !! 常见原因: 模组假死(MQTT 掉了但不发 URC) / 中间 NAT 把连接掐了\r\n");
            /* 计数清零后交给统一恢复入口(4G_RST 断电重启, 带最小间隔防抖) */
            g_pub_fail_cnt = 0U;
            g4_link_recover("保活停滞(长时间没有一次发布成功)");
        }
        /* 保活由模组内部 PINGREQ + 应用层 GW_ONLINE 心跳共同维持;
         * 断线由 g4_urc 的 conn URC 和发布失败计数检测;
         * 上行是否真的到云端, 由 QoS 1 的 puback 说话(g4_urc_puback_count)。 */
        break;
    }

    case G4_STATE_DIAG_DONE:
        /* 【自检档的终点】G4_DIAG_STOP_AFTER_PDP=1 时停在这里: 只验收到
         * "AT 沟通正常 + 已入网拿到 IP", 不碰 MQTT。每 30s 重查一次信号,
         * 既能持续证明链路是活的, 又不会因为 MQTT 连不上把网络链路拆掉重来。 */
        if ((now - g_state_tick) > 30000u)
        {
            g4_cmd_res_t r;

            g_state_tick = now;
            log_info("[G4] [自检] 模组沟通正常 IMEI=%s IP=%s CSQ=%u%s%u mV\r\n",
                     (g_info.imei[0] != '\0') ? g_info.imei : "-",
                     (g_info.ip[0] != '\0') ? g_info.ip : "-",
                     (unsigned)g_info.csq,
                     g_info.vbat_valid ? " 供电=" : "",
                     (unsigned)g_info.vbat_mv);

            r = g4_io_cmd_run("AT+CSQ", "+CSQ:", AT_CMD_TIMEOUT_MS);
            if (r == G4_CMD_RUNNING) { break; }

            if (r == G4_CMD_OK)
            {
                log_info("[G4] [自检] AT+CSQ 应答: %s\r\n", g4_io_cmd_resp());
                /* 模组是不是被供电搞死最直接的一条: 每轮自检都读一次它的自测电压 */
                (void)g4_read_vbat();
            }
            else
            {
                log_info("[G4] [自检] !! AT+CSQ 不应答了: 模组可能掉电/死机, 重新建链\r\n");
                g4_goto_state(G4_STATE_ERROR);
            }
        }
        break;

    case G4_STATE_ERROR:
    default:
        g4_urc_set_offline();
        g_pub_pending = false;
        g_info.pdp_ok = false;
        g_info.ip[0]  = '\0';
        local_alarm_trigger(ALARM_4G_OFFLINE);
        if ((now - g_state_tick) > G4_FAIL_RESTART_MS)
        {
            g_fail_cnt++;

            /* 【连续 G4_REBUILD_RST_CNT 次重建都没能回到 ONLINE】
             * 说明"只重新握手"这条路走不通了(模组卡在某个状态、或者网络侧根本没
             * 恢复), 这时候按现场要求上 4G_RST: 断电重启模组。 */
            if (g_fail_cnt >= G4_REBUILD_RST_CNT)
            {
                g_fail_cnt = 0u;
                g4_link_recover("连续建链失败(重新握手已试过多次)");
                break;
            }

            /* 第一次失败不碰电源: 让模组连着供电把起动/入网走完
             * (每次失败就断电 = 模组刚起动到一半被拉闸, 永远起不来) */
            log_info("[G4] rebuild #%u (no power cycle)\r\n", (unsigned)g_fail_cnt);
            g4_goto_state(G4_STATE_AT_SYNC);
        }
        break;
    }
}

bool g4_at_is_online(void)
{
    return (g4_urc_is_online() && (g_state == G4_STATE_ONLINE));
}

bool g4_at_set_clock_ymdhms(uint16_t year, uint8_t month, uint8_t day,
                            uint8_t hour, uint8_t minute, uint8_t second)
{
    char       cmd[48];
    char       *resp = NULL;        /* g4_io_send_expect 要的是 char** —— 不能加 const */
    int        n;

    /* 参数范围先自检: 月/日/时/分/秒越界就别往下发, 免得模组拒绝后
     * 只能靠 AT 应答猜原因(而且写坏模组时钟比"没写成"更糟)。 */
    if ((month < 1u) || (month > 12u) || (day < 1u) || (day > 31u) ||
        (hour > 23u) || (minute > 59u) || (second > 59u))
    {
        log_info("[G4] CCLK set 参数越界: %u-%02u-%02u %02u:%02u:%02u\r\n",
                 (unsigned)year, month, day, hour, minute, second);
        return false;
    }

    /* 模组不在网时它的时钟没有意义, 也不值得占用 AT 引擎 */
    if (!g4_at_is_online())
    {
        log_info("[G4] CCLK set skip: offline\r\n");
        return false;
    }

    /* 格式与读侧一致: yy/MM/dd,hh:mm:ss+zz
     * zz 固定 +32(1/4 小时 = 北京 +8h) —— 与模组自己上报的写法一致(见头文件说明)。 */
    n = snprintf(cmd, sizeof(cmd), "AT+CCLK=\"%02u/%02u/%02u,%02u:%02u:%02u+32\"",
                 (unsigned)(year % 100u), (unsigned)month, (unsigned)day,
                 (unsigned)hour, (unsigned)minute, (unsigned)second);
    if ((n <= 0) || (n >= (int)sizeof(cmd)))
    {
        return false;
    }

    if (!g4_io_send_expect(cmd, "OK", AT_CMD_TIMEOUT_MS, &resp))
    {
        log_info("[G4] CCLK set 无 OK 应答\r\n");
        return false;
    }

    log_info("[G4] CCLK set ok -> 20%02u-%02u-%02u %02u:%02u:%02u(+8h)\r\n",
             (unsigned)(year % 100u), (unsigned)month, (unsigned)day,
             (unsigned)hour, (unsigned)minute, (unsigned)second);
    return true;
}

/* ---------- AT+CCLK? 取模组时间---------- */

/* 从*pp 读两位十进制数 成功推进指针 */
static bool _cclk_two_digits(const char **pp, uint8_t *out)
{
    const char *p = *pp;

    if ((p[0] < '0') || (p[0] > '9') || (p[1] < '0') || (p[1] > '9'))
    {
        return false;
    }
    *out = (uint8_t)((p[0] - '0') * 10 + (p[1] - '0'));
    *pp  = p + 2;
    return true;
}

/* ---------- 日期 / 星期小工具(时区换算与星期计算要用) ---------- */

/** @brief 2000+yy 年 mm 月的天数(mm 保证 1~12) */
static uint8_t _days_in_month(uint8_t yy, uint8_t mm)
{
    static const uint8_t dm[12] = {31U,28U,31U,30U,31U,30U,31U,31U,30U,31U,30U,31U};

    if (mm == 2U)
    {
        /* 年份按 2000+yy 处理: 20xx 里 yy 能被 4 整除就是闰年(2000 本身是闰年) */
        return ((yy % 4U) == 0U) ? 29U : 28U;
    }
    return dm[mm - 1U];
}

/** @brief 日期 +1 天(带月/年进位) */
static void _date_add_one_day(uint8_t *yy, uint8_t *mm, uint8_t *dd)
{
    if (*dd < _days_in_month(*yy, *mm)) { (*dd)++; return; }

    *dd = 1U;
    if (*mm < 12U) { (*mm)++; return; }

    *mm = 1U;
    *yy = (uint8_t)((*yy + 1U) % 100U);
}

/** @brief 日期 -1 天(带月/年借位) */
static void _date_sub_one_day(uint8_t *yy, uint8_t *mm, uint8_t *dd)
{
    if (*dd > 1U) { (*dd)--; return; }

    if (*mm > 1U) { (*mm)--; }
    else { *mm = 12U; *yy = (uint8_t)((*yy + 99U) % 100U); }

    *dd = _days_in_month(*yy, *mm);
}

/**
 * @brief 由年月日算出星期(1=周一 .. 7=周日)
 *
 * 【为什么网关要自己算】AT+CCLK? 的应答里【没有星期字段】, 原来代码固定填周五。
 * 现场对不上: 2026-09-22 实际是周二, 空口上发的却是 w5。
 * (液位设备自己不读这个字节、按日期重算了, 所以之前没暴露; 但别的设备/云端
 *  只要读它就会拿到错的星期。)
 *
 * 用 Zeller 同余; 年份按 2000+yy 处理。编码与协议样例一致:
 *   样例 "26, 9, 18, 5, ..." = 2026-09-18, 那天确实是周五 -> 5。
 */
static uint8_t _weekday_from_date(uint8_t yy, uint8_t mm, uint8_t dd)
{
    int y = 2000 + (int)yy;
    int m = (int)mm;
    int d = (int)dd;
    int k, j, h;

    if (m < 3) { m += 12; y -= 1; }     /* 1、2 月按上一年的 13、14 月算 */

    k = y % 100;
    j = y / 100;
    h = (d + (13 * (m + 1)) / 5 + k + k / 4 + j / 4 + 5 * j) % 7;
    /* Zeller 的 h: 0=周六 1=周日 2=周一 ... 6=周五 */
    return (uint8_t)(((h + 5) % 7) + 1);   /* -> 1=周一 .. 7=周日 */
}

bool g4_at_get_clock(g4_clock_t *out)
{
    char       *resp = NULL;
    const char *p;
    uint8_t     yy, mm, dd, hh, mi;

    if (out == NULL) return false;

    /* 【模组不在网就别问了。 g4_io_send_expect 会白阻塞 1s 还必然超时】     * 调用方约定 拿不到时间就【不下发】带校时字段的0x01【*/
    if (!g4_at_is_online())
    {
        log_info("[G4] CCLK skip: offline\r\n");
        return false;
    }

    if (!g4_io_send_expect("AT+CCLK?", "+CCLK:", AT_CMD_TIMEOUT_MS, &resp))
    {
        log_info("[G4] CCLK no resp\r\n");
        return false;
    }
    if (resp == NULL)
    {
        log_info("[G4] CCLK resp null\r\n");
        return false;
    }

    /* 期望: +CCLK: "21/12/27,06:56:20+32"  ——yy/MM/dd,hh:mm:ss+zz */
    p = strchr(resp, '"');
    if (p == NULL)
    {
        log_info("[G4] CCLK no quote: %s\r\n", resp);
        return false;
    }
    p++;

    if (!_cclk_two_digits(&p, &yy) || (*p != '/')) goto bad;
    p++;
    if (!_cclk_two_digits(&p, &mm) || (*p != '/')) goto bad;
    p++;
    if (!_cclk_two_digits(&p, &dd) || (*p != ',')) goto bad;
    p++;
    if (!_cclk_two_digits(&p, &hh) || (*p != ':')) goto bad;
    p++;
    if (!_cclk_two_digits(&p, &mi)) goto bad;

    /* 基本合法性 月份 1~12 是关键——设备端app_cmd.c 里month==0 成>12
     * 会直接判 CMD_RET_ERR_PARAM 并回 0x81, 校时失败等于整条 0x01 白下【*/
    if ((mm < 1U) || (mm > 12U) || (dd < 1U) || (dd > 31U) ||
        (hh > 23U) || (mi > 59U))
    {
        log_info("[G4] CCLK bad time: 20%02u-%02u-%02u %02u:%02u\r\n",
                 (unsigned)yy, (unsigned)mm, (unsigned)dd,
                 (unsigned)hh, (unsigned)mi);
        return false;
    }

    /* ---------- 【时区换算: 统一成北京时间】 ----------
     *
     * AT+CCLK? 的应答是 "yy/MM/dd,hh:mm:ss±zz", 其中 zz 的单位是
     * 【四分之一小时】(3GPP TS 27.007): +32 = +8 小时 = 北京, +00 = UTC。
     * 本模块实测应答形如:  +CCLK: "26/09/22,16:20:41+32"
     *
     * ⚠️ 这个字段以前被整个丢掉了, 于是"模组报什么时区, 我们就当北京时间发下去"。
     *    第一轮修(按 ±zz 换算)之后现场发现【仍然差 8 小时】:
     *        本机真实北京时间 : 00:20   (串口日志时间戳)
     *        模组自报        : 16:20 且 ±zz = +32   ← 声称是北京, 数字却是 UTC
     *    也就是说【这个 ±zz 不可信: 模组把 UTC 的数字标成了北京】。
     *    所以现在默认【无视 ±zz, 一律按 UTC 解释】, 由网关 +8h 补成北京。
     *    开关见 g4_at.h 的 G4_CCLK_TZ_QUARTERS(0x00 = 无视 / 0x20 = 相信)。
     *
     * 协议约定: 发给设备的 6 字节校时字段是【北京时间】(设备端 app_time 按 -8h
     * 转 UTC 存), 所以这里统一换算到 +08 —— 需要时连带日期进位/借位。 */
    {
        int     tz_reported = 0;      /* 应答里 ±zz 的值(单位 1/4 小时) */
        int     tz_used;
        int     delta_min;
        bool    tz_ok    = false;
        bool    trust_tz = false;     /* 本次是否真的用了模组自报的 ±zz */
        uint8_t ss       = 0U;

        /* 跳过秒(值不关心, 但要把指针移过去)。有的固件可能省略, 所以不强制 */
        if (*p == ':')
        {
            p++;
            if (!_cclk_two_digits(&p, &ss)) goto bad;
        }

        /* 读 ±zz (1/4 小时: +32 = +8h = 北京, +00 = UTC) */
        if ((*p == '+') || (*p == '-'))
        {
            int     sign = (*p == '-') ? -1 : 1;
            uint8_t zz   = 0U;

            p++;
            if (_cclk_two_digits(&p, &zz))
            {
                tz_reported = sign * (int)zz;
                tz_ok       = true;
            }
        }

        /* 【用哪个时区解释这些数字】由 G4_CCLK_TZ_QUARTERS 决定(见 g4_at.h):
         *   0x00(默认) = 无视 ±zz, 当 UTC;  0x20 = 相信 ±zz。
         * 现场实测元组自报 +32 却给出 UTC 数字, 所以默认不信它。 */
        tz_used = (int)G4_CCLK_TZ_QUARTERS;

        if (!tz_ok)
        {
            /* 应答里干脆没有时区字段: 无论配成什么, 都只能当 UTC */
            tz_used = 0;
        }
        else if (tz_used == 0x20)
        {
            tz_used  = tz_reported;   /* 相信模组 */
            trust_tz = true;
        }

        /* 换算到北京(+32 个 1/4 小时): 正 = 往后推, 负 = 往前推 */
        delta_min = (32 - tz_used) * 15;

        if (delta_min != 0)
        {
            int tot = (int)hh * 60 + (int)mi + delta_min;

            while (tot < 0)     { tot += 1440; _date_sub_one_day(&yy, &mm, &dd); }
            while (tot >= 1440) { tot -= 1440; _date_add_one_day(&yy, &mm, &dd); }

            hh = (uint8_t)(tot / 60);
            mi = (uint8_t)(tot % 60);
        }

        if (tz_ok)
        {
            log_info("[G4] CCLK 模组自报时区 %s%u/4h; 本次按【%s】解释 -> "
                     "换算到北京时间(%s%u 分钟)\r\n",
                     (tz_reported < 0) ? "-" : "+",
                     (unsigned)((tz_reported < 0) ? -tz_reported : tz_reported),
                     trust_tz ? "模组的 ±zz" : "UTC, 无视 ±zz",
                     (delta_min < 0) ? "-" : "+",
                     (unsigned)((delta_min < 0) ? -delta_min : delta_min));
        }
        else
        {
            log_info("[G4] CCLK 应答里没有时区字段 -> 按 UTC 处理, "
                     "换算到北京时间(%s%u 分钟)\r\n",
                     (delta_min < 0) ? "-" : "+",
                     (unsigned)((delta_min < 0) ? -delta_min : delta_min));
        }
    }

    out->year   = yy;
    out->month  = mm;
    out->day    = dd;
#if G4_CLOCK_COMPUTE_WEEK
    out->week   = _weekday_from_date(yy, mm, dd);   /* 由归一化后的日期算真实星期 */
#else
    out->week   = G4_CLOCK_DEFAULT_WEEK;            /* 退回"固定周五" */
#endif
    out->hour   = hh;
    out->minute = mi;

    log_info("[G4] CCLK -> 20%02u-%02u-%02u w%u %02u:%02u\r\n",
             (unsigned)out->year, (unsigned)out->month, (unsigned)out->day,
             (unsigned)out->week, (unsigned)out->hour, (unsigned)out->minute);
    return true;

bad:
    log_info("[G4] CCLK format err: %s\r\n", resp);
    return false;
}

/** @brief 把一条报文放进"正在发"的槽位(队列空且空闲时用) */
static void g4_pub_slot_load(const char *topic, const uint8_t *data, uint16_t len,
                             uint8_t qos, uint8_t retain)
{
    strncpy(g_pub_topic, topic, sizeof(g_pub_topic) - 1u);
    g_pub_topic[sizeof(g_pub_topic) - 1u] = '\0';
    memcpy(g_pub_data, data, len);
    g_pub_len     = len;
    g_pub_qos     = (qos > 2u) ? 2u : qos;
    g_pub_retain  = (retain != 0u) ? 1u : 0u;
    g_pub_pending = true;
    g_pub_state   = PUB_IDLE;
}

/**
 * @brief 上一条发完/失败后, 从队列里取下一起来发
 * @return true = 又启动了一条(调用方保持"发布中"), false = 队列空了
 */
static bool g4_pub_queue_next(void)
{
    while (g_pub_q_cnt > 0u)
    {
        uint8_t i;

        for (i = 0u; i < G4_PUB_QUEUE_DEPTH; i++)
        {
            if (g_pub_q[i].used)
            {
                g4_pub_slot_load(g_pub_q[i].topic, g_pub_q[i].data, g_pub_q[i].len,
                                 g_pub_q[i].qos, g_pub_q[i].retain);
                g_pub_q[i].used = false;
                g_pub_q_cnt--;
                return true;
            }
        }
        break;      /* 计数和实际不一致时兜底, 免得死循环 */
    }
    return false;
}

sys_err_t g4_at_mqtt_publish_ex(const char *topic, const uint8_t *payload,
                                uint16_t payload_len, uint8_t qos, bool retain)
{
    if (qos > 2u)
    {
        qos = 2u;               /* MQTT 只有 0/1/2 */
    }
    if (!g4_at_is_online())
    {
        return SYS_ERR;
    }
    if ((payload == NULL) || (payload_len == 0u))
    {
        return SYS_ERR;
    }
    if (payload_len > G4_PUB_DATA_MAX)
    {
        /* ★2026-10-08: 原来这里是【静默】返回 SYS_ERR —— 平台什么都收不到,
         * 只能干等超时, 而现场看日志也看不出"是被长度挡了"。
         * 现在明确打出来。为什么不硬发: 见 G4_PUB_DATA_MAX 上面那段证据 ——
         * 超限硬发的代价是模组 AT 应用层重启, 整条链路断 20 秒。 */
        log_info("[G4] !! 发布被拒: %u 字节 > 单条上限 %u —— 不硬发(硬发会撑重启模组)"
                 ", 请平台改用按域查询/分片\r\n",
                 (unsigned)payload_len, (unsigned)G4_PUB_DATA_MAX);
        return SYS_ERR;     /* 单条超长, 明确拒绝 */
    }
    /* ★2026-10-08: 不再有"默认发布主题"★
     * 原来 topic==NULL 会回落成 G4_MQTT_PUB_TOPIC(mod/mod_upload) —— 那是 LoRa 原始帧
     * hex 上行的遗留通道。该通道已按甲方要求【整体下线】, 所以这里改成明确失败:
     * 任何调用方都必须显式给出主题(rt/gw/{gwId}/telemetry / event / status / cmdresp)。
     * 这样"忘了给主题"是一个看得见的错误, 而不是又悄悄发到一条废弃主题上。 */
    if ((topic == NULL) || (topic[0] == '\0'))
    {
        log_info("[G4] !! g4_at_mqtt_publish 没给 topic(默认主题 mod/mod_upload 已删除)\r\n");
        return SYS_ERR;
    }

    /* 空闲 → 直接占用"正在发"的槽位 */
    if (!g_pub_pending)
    {
        g4_pub_slot_load(topic, payload, payload_len, qos, (uint8_t)(retain ? 1u : 0u));
        return SYS_OK;
    }

    /* 【正在发 → 排队, 不再丢掉】
     * 现场就是这里丢的回执: 云端连着发几条, 从第二条起全部 SYS_ERR 被上层丢掉,
     * 日志里一排 "回执登记失败"。现在排队, 由 g4_pub_queue_next() 顺次发出去。 */
    if (g_pub_q_cnt < G4_PUB_QUEUE_DEPTH)
    {
        uint8_t i;

        for (i = 0u; i < G4_PUB_QUEUE_DEPTH; i++)
        {
            if (!g_pub_q[i].used)
            {
                strncpy(g_pub_q[i].topic, topic, sizeof(g_pub_q[i].topic) - 1u);
                g_pub_q[i].topic[sizeof(g_pub_q[i].topic) - 1u] = '\0';
                memcpy(g_pub_q[i].data, payload, payload_len);
                g_pub_q[i].len    = payload_len;
                g_pub_q[i].qos    = qos;
                g_pub_q[i].retain = (uint8_t)(retain ? 1u : 0u);
                g_pub_q[i].used   = true;
                g_pub_q_cnt++;
                return SYS_OK;
            }
        }
    }

    /* 队列也满了: 只能丢, 但必须留下痕迹(否则又是"静默丢消息") */
    g_pub_drop_cnt++;
    log_info("[G4] !! 发布队列满(深度 %u), 丢弃一条; 累计丢弃 %lu 条\r\n",
             (unsigned)G4_PUB_QUEUE_DEPTH, (unsigned long)g_pub_drop_cnt);
    return SYS_ERR;
}

/**
 * @brief 兼容包装: 既有的 3 个调用点(在线心跳 / LoRa 上报 / 云端回执)照旧使用
 *
 * 等价于 g4_at_mqtt_publish_ex(..., qos=1, retain=false)
 * —— 与改造前拼出来的 AT 命令逐字节一致, 所以现网行为不变。
 */
sys_err_t g4_at_mqtt_publish(char *topic, uint8_t *payload, uint16_t payload_len)
{
    return g4_at_mqtt_publish_ex(topic, payload, payload_len, 1u, false);
}

/* 说明: 原来这里还有一个mqtt_pack_publish_sensor_data() ——把上行记录拼成 * JSON 再发。云端已改成直接收协议帧的 hex 文本", 该函数随之删除
 * 组帧逻辑挪到了业务层: lora_business_publish_report() (lora_business.c)。 * 这样 4G 层只算把一段字节发出去", 不再知道业务载荷长什么样【*/
