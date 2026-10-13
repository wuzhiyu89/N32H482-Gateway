#ifndef __G4_AT_H__
#define __G4_AT_H__

#include <stdint.h>
#include <stdbool.h>
#include "data_cache.h"
#include "g4_urc.h"   /* g4_mqtt_recv_cb_t 在此定义,本头不再重复 typedef */

typedef enum
{
    SYS_ERR = 0U,
    SYS_OK = 1U,
} sys_err_t;


/**
 * @brief 4G模块初始化，复位、启动AT状态机
 */
void g4_at_init(void);

/**
 * @brief 4G主轮询，放在while(1)，消费rb_g4环形缓冲区，非阻塞
 */
void g4_at_poll(void);

/**
 * @brief 获取4G模块MQTT是否联网成功
 * @return true在线；false离线
 */
bool g4_at_is_online(void);

/**
 * @brief 注册 MQTT 下行接收回调(可空)
 *        类型 g4_mqtt_recv_cb_t 见 g4_urc.h
 */
void g4_at_set_recv_cb(g4_mqtt_recv_cb_t cb);

/**
 * @brief 设置下行 MQTT 订阅 topic
 * @param topic NULL 或空串 -> 【清空订阅主题】(不再有编译期兜底值)
 * @note  只在【下一次 MQTT 建链】时生效, 本函数不主动重订阅。
 *        ★2026-10-08★ 旧的编译期兜底值 "mod/pc_down" 已随甲方要求整体删除,
 *        所以现在【必须】由 main.c 注入 —— 按 cloud_profile 算出
 *        `rt/gw/{gwId}/cmd`(出厂 gwId=110001)。没注入时建链会打 !!
 *        并【跳过订阅】(上行仍可发, 但收不到任何云端命令)。
 */
void g4_at_set_sub_topic(const char *topic);

/** 取当前下行订阅 topic(诊断/日志用) */
const char *g4_at_get_sub_topic(void);

/**
 * @brief 注入 MQTT broker 参数(来自配置域 network; 文档 3.2"整域重启生效")
 * @param host 主机名/IP; **NULL 或空串 = 不覆盖**(沿用编译期默认 223.75.207.139)
 * @param port 端口字符串; 同上
 * @param user 用户名;   NULL 或空串 = 不覆盖
 *             ★甲方平台约定: 用户名 = 设备 ID(gwId)★ —— main.c 会把
 *             `config_store.system.gwId`(出厂 "110001") 传进来
 * @param pass 密码;     NULL 或空串 = 不覆盖
 * @note  ★ 由 main.c 在【启动时】调用一次(配置改了要重启才生效, 与文档一致) ★
 *        用"设置接口"而不是让 g4_at 直接读 config_store: 后者会把 config_store+cJSON
 *        拖进依赖, 让 test_state 那套主机单测(g4_at.c 在其中)直接链接失败。
 */
void g4_at_set_broker(const char *host, const char *port,
                      const char *user, const char *pass);

/** 取当前生效的 broker host/port/user(诊断/日志用)
 *  ★2026-10-08 新增 user★: 甲方平台约定"用户名 = 设备 ID(gwId)",
 *  启动日志要能一眼看出到底用了哪个用户名 —— 连不上时这是第一个要核对的字段。 */
const char *g4_at_get_broker_host(void);
const char *g4_at_get_broker_port(void);
const char *g4_at_get_broker_user(void);
/** 取当前密码 —— 只用来判断"设没设"(启动日志打"已设/空")。
 *  ⚠️ 不要把返回值打进日志: 串口日志会外泄凭据。 */
const char *g4_at_get_broker_pass(void);

/**
 * @brief ★2026-10-08 新增★ 注入 MQTT clientID(默认编译期 "110001")
 *
 * ★为什么需要它★ 固定 clientID 会让【同一 broker 上】的多台网关互相顶下线
 * (MQTT 规范: 新 CONNECT 用了已存在的 clientID 会踢掉旧连接)。
 * 有了这个注入口, 上多台时改配置即可, 不必重新烧固件:
 *     cfg.set network.clientId = "<唯一串>"
 *     cfg.set network.clientId = "IMEI"   —— 特殊值: 用模组 IMEI, 天然唯一
 *
 * @param cid NULL 或空串 = 不覆盖(保留当前值); 只在【下一次建链】生效
 */
void g4_at_set_client_id(const char *cid);

/** 取当前 clientID(诊断/日志用)。注意若配的是特殊值 "IMEI", 实际发出去的是模组 IMEI */
const char *g4_at_get_client_id(void);

/**
 * @brief 发送 MQTT-PUBLISH 报文(载荷是任意字节流, 不做任何编码/转义)
 * @param topic mqtt 主题字符串; ★必须显式给出★
 *              (2026-10-08 起【没有默认主题了】: 旧的 mod/mod_upload 已删除,
 *               传 NULL/空串会直接返回 SYS_ERR 并打日志)
 * @param payload 载荷(各业务自己组好的字节流)
 * @param payload_len 载荷长度
 * @return SYS_OK 成功; SYS_ERR 离线/上一条还没发完/超长/没给主题
 * @note  非阻塞: 只登记, 真正的两阶段发布由 g4_at_poll() 里的子流程做
 */
sys_err_t g4_at_mqtt_publish(char *topic, uint8_t *payload, uint16_t payload_len);

/**
 * @brief 发送 MQTT-PUBLISH 报文(可指定 QoS / retained)
 * @param topic  mqtt 主题; ★必须显式给出★(NULL/空串 -> SYS_ERR, 见上)
 * @param qos    0/1/2(>2 会被夹到 2)
 * @param retain true = 让 broker 保留这条(retained)
 * @return SYS_OK 已登记(空闲则立即占用槽位, 否则排队); SYS_ERR 离线/超长/队列满/没给主题
 * @note  g4_at_mqtt_publish() 是本函数的兼容包装(qos=1, retain=false),
 *        拼出来的 AT 命令与改造前逐字节一致。
 *        文档(RoadTwin)要求: telemetry QoS0, cmd/cmdresp/event/status QoS1,
 *        且只有 status 带 retained。
 */
sys_err_t g4_at_mqtt_publish_ex(const char *topic, const uint8_t *payload,
                                uint16_t payload_len, uint8_t qos, bool retain);

/* ==================== 模组信息(建链过程中采集) ====================
 *
 * 仿照现场验证过的 ml307_app.c/ml307_init: 建链时顺手把 IMEI/IMSI/ICCID/CSQ
 * 读出来。IMEI 有两个用途:
 *   1) 当 MQTT client ID —— 现场多台网关同时用同一个固定 client ID 会被
 *      broker 互相顶下线(现象: 一台在线另一台就掉), 用 IMEI 天然唯一;
 *   2) 上行组帧 / 云端识别设备。
 * CSQ 换算: rssi_dbm = -113 + 2 * csq (3GPP TS 27.007)。
 */
#define G4_IMEI_LEN     16U   /* 15 位 + '\0' */
#define G4_IMSI_LEN     16U
#define G4_ICCID_LEN    21U   /* 20 位 + '\0' */
#define G4_IP_LEN       16U

typedef struct
{
    char     imei[G4_IMEI_LEN];    /* 空串 = 没读到 */
    char     imsi[G4_IMSI_LEN];
    char     iccid[G4_ICCID_LEN];
    uint8_t  csq;                  /* 0~31; 99 = 未知/无信号 */
    int16_t  rssi_dbm;             /* 由 csq 换算, csq=99 时为 0 */
    uint8_t  reg_status;           /* 最近一次注册查询的 <stat>: 1/5 = 已注册, 0xFF = 未知 */
    char     ip[G4_IP_LEN];        /* PDP/RNDIS 分配到的本机 IP, 空串 = 没有 */
    bool     pdp_ok;               /* true = 已确认拿到 IP */

    /* ---- 模组自己测到的供电电压(AT+CBC) ----
     * 【为什么要读它】: 模组供电稳不稳, 拿万用表量是不准的 —— 万用表直流档读的是
     * 平均值/慢采样, 发射瞬间掉到 3.0V 它也只给你一个 3.8。AT+CBC 是【模组内部 ADC
     * 测的 VBAT】, 由模组自己持续采样, 能反映真实水位。
     * 日志口 "#vbat" 读一次, "#vbatsweep 30" 连测 30 次给出 min/max/avg —— 抓跳变用。 */
    uint16_t vbat_mv;              /* 模组自测供电(mV, 常见实现); 0 = 没读到 */
    bool     vbat_valid;           /* true = 读到的确实是 mV 值 */
    uint8_t  temp_c;               /* 模组温度(℃, AT+MCHIPINFO="temp") */
    bool     temp_valid;           /* true = 温度有效 */
} g4_module_info_t;

/**
 * @brief 读模组信息(IMEI/IMSI/ICCID/CSQ/注册状态/IP)
 * @return 指向内部静态结构, 永远非 NULL; 建链未完成时字段为空/0
 */
const g4_module_info_t *g4_at_get_info(void);

/**
 * @brief 距上次【发布成功】过了多少毫秒
 *
 * 【这是判断"保活还活着吗"的唯一可靠指标】: 本工程的保活 = 每 30s 发一条
 * GW_ONLINE 心跳 + 模组内部 PINGREQ。只要这个数一直涨不回头, 就说明心跳发不出去
 * 了(哪怕 g4_at_is_online() 还返回 true)。
 *
 * 状态行里的 "HB=xs" 就是它。在线时它应该始终远小于 30s。
 * @return 毫秒数; 0xFFFFFFFF = 本次开机还没成功发布过
 */
uint32_t g4_at_pub_age_ms(void);

/**
 * @brief 累计"等 '>' 时被模组吞掉提示符"的次数(诊断用)
 *
 * 现场日志 78.txt 里这是最高频的故障: 发 AT+MQTTPUB 后模组只回
 * "+MATREADY" + "OK", 那个 '>' 提示符没了。
 * 现在遇到它不再直接给模组断电, 而是敲一条 AT 探活:
 *   · AT 回 OK  -> AT 引擎正常, 直接重发本条(最多 G4_PUB_PROMPT_LOSS_MAX 次);
 *   · AT 不应答 -> 才真的断电重启。
 * 本计数 = 重发过多少次(只增不减, 开机清零)。
 */
uint32_t g4_at_prompt_loss_count(void);

/**
 * @brief 累计走了多少次"MQTT 快修"(只重连 MQTT, 【不断电模组】)
 *
 * 【病灶, 来自现场日志 78.txt】一发 PUBLISH, 那个 MQTT 会话就死了:
 *   第 1 条 MQTTPUB 被模组收下但 broker 永远不回 puback, 云端一条也收不到;
 *   从第 2 条起 AT+MQTTPUB 直接回 "+CME ERROR: 601"(= MQTT 未连接)。
 * 而模组【不发 conn URC】, 所以状态机一直以为还在线, 继续往死会话里灌数据。
 *
 * 要修的只是 MQTT 这一层(MQTTDISC -> MQTTCONN -> MQTTSUB, 2~5 秒),
 * 所以现在遇到这种情况【不再给整个模组断电】(那是 40~60 秒)。
 * 只有连续多次快修都救不回来, 才升级到 g4_link_recover() 的断电重启。
 *
 * 本计数 = 累计快修次数(只增不减, 开机清零)。它一涨就说明"会话又死了一次"。
 */
uint32_t g4_at_mqtt_fast_count(void);

/**
 * @brief 模组的射频功能档位(建链时读的 AT+CFUN?)
 *
 * 【为什么值得看它】: "卡读得到、AT 全正常、就是不上网"这一档里, 除了没信号,
 * 还有一个很容易被忽略的原因 —— 模组被留在 AT+CFUN=0(最小功能, 射频关着),
 * 这时它【根本不会去搜网】, 表现就是 AT+CSQ 一直 99,99 且 AT+CEREG? 回 0,0
 * (没注册【也没在搜】)。g4_at 建链时读到 0 会自动补一条 AT+CFUN=1 打开射频。
 *
 * @return 1 = 全功能(正常) / 0 = 最小功能(射频关) / -1 = 还没读到或模组不支持
 */
int8_t g4_at_get_cfun(void);

/* ==================== 断网判定(两条腿) ====================
 *
 *  1) 【发布失败计数】连续 G4_PUB_FAIL_MAX(默认 3)次发布失败 —— 等 '>' 超时 /
 *     等 OK 超时 / 模组回 ERROR —— 就判链路已断。(本机类失败如命令组装失败
 *     不计入, 见 g4_at.c 的 g4_pub_abort。)
 *     注: ">' 丢失后探活成功并重发"不算失败, 不计入这里。
 *  2) 【保活停滞看门狗】距上次"发布成功"超过 g4_at_set_link_watchdog() 设的窗口,
 *     不管什么原因一律判链路已死 —— 这条能兜住"发布根本没登记/没跑起来"的情况。
 *
 * 两条都走同一个恢复入口: 4G_RST 给模组断电重启(g4_link_recover), 并带
 * G4_RST_MIN_GAP_MS 最小间隔防抖。
 */

/**
 * @brief 设定"距上次发布成功多久算链路已死"(0 = 关闭这条判据)
 *
 * 【必须由 main.c 按实际心跳周期来设】, 不要用固定值:
 *     g4_at_set_link_watchdog(MAIN_GW_ONLINE_ENABLE ? (5U * MAIN_GW_ONLINE_MS) : 0U);
 *   · 有周期心跳(本工程默认 30s) -> 传 5 × 周期 = 150s: 连着几次心跳都没成功才触发;
 *     ⚠️ 窗口必须【明显长于】"模组自己重启 + 重新入网 + 重连 MQTT"的一轮(实测
 *        40~60 秒), 否则看门狗会在模组刚要回来时又把它拆一次(现场日志 78.txt)。
 *   · 没有周期心跳               -> 必须传 0, 否则必然误判(没有上行还指望它发成功)。
 *
 * @param max_silence_ms 毫秒; 0 = 关闭
 */
void g4_at_set_link_watchdog(uint32_t max_silence_ms);

/* ==================== MQTT 遗嘱(LWT) ====================
 * 文档 2 章 / 7.7: status topic 是 retained, 且要带 LWT 遗嘱 {"online":false} ——
 * 网关【异常掉线】时由 broker 代发, 平台靠它区分"设备死了"和"只是没数据"。
 *
 * 【当前默认不启用】模组是否支持遗嘱参数【还没在台架上确认】:
 *   4G 手册(ML307R_ATCommands.pdf)里没有 MQTT 章节, 工程内也没有记录。
 *   调用本函数只是把遗嘱"备好"(开机日志会说明), 真正把它拼进 AT+MQTTCONN
 *   由 g4_at.c 的 MAIN_MQTT_LWT_ENABLE 控制(默认 0 = 与验证过的流程逐字节一致)。
 *   ★ 打开前先做 MQTT_LWT_台架测试.txt 里的发现步骤 ★
 *
 * @param topic   遗嘱 topic(status topic, 例 rt/gw/<gwId>/status); NULL 或空 = 取消
 * @param payload 遗嘱正文(例 {"online":false}); NULL = 用内置默认
 */
void g4_at_set_lwt(const char *topic, const char *payload);

/* ==================== 日志口本地诊断命令 ====================
 *
 * 在日志串口(USART1, 115200)上敲以 '#' 开头的一整行 —— 这些行【不转发给模组】,
 * 而是本地执行, 专门用来排"模组没电 / 电压不稳 / 一个字节都不回"。
 * 命令表见 g4_at.c 里 g4_at_console_cmd() 的注释, 敲 "#help" 也能列出来。
 *
 * @param line 已按行切好、以 '\0' 结尾的整行
 * @return true = 是本地命令并且已执行(调用方不要转发给模组);
 *         false = 不是本地命令(调用方按原逻辑转发给模组)
 */
bool g4_at_console_cmd(const char *line);

/* ---- 日志口命令扩展钩子(云协议离线自测) ----
 *
 * 【为什么要有】现场 4G 常在弱网: 模组入不了网 / broker 连不上, 于是
 * "云端下发 -> 网关应答"这条链路根本没法验。但云协议层的代码(信封解析、命令表、
 * 应答组包、档位 topic 映射)其实【一行都不依赖网络】, 缺的只是一个把下行字节
 * 喂进业务处理函数的入口。主程序用这两个钩子把自己的命令挂到同一个日志口上。
 *
 * @param ext   分发钩子: 收到不认识的 '#' 行时调用; 返回 true = 已处理
 * @param help  #help 时追加打印自定义命令的用法(可为 NULL)
 */
typedef bool (*g4_console_ext_fn)(const char *line);
typedef void (*g4_console_help_fn)(void);

void g4_at_set_console_ext(g4_console_ext_fn ext, g4_console_help_fn help);

/**
 * @brief 打印"时钟真相": 当前系统时钟来源 + HSE/PLL 寄存器实况(串口敲 "#clk")
 *
 * 用于判定板上 24MHz 外部晶振到底有没有起振、整机实际按哪个时钟源跑
 * —— 因为两个构建配置里的 HSE_VALUE(8MHz / 16MHz)都与 24MHz 对不上。
 */
void g4_at_print_clock(void);

/* ==================== 4G 模组时间(AT+CCLK?) ==================== */

/* 星期默认固定值 —— 只在 G4_CLOCK_COMPUTE_WEEK=0 时用作兜底。
 * AT+CCLK? 的应答里没有星期, 编码是 1=周一 ... 7=周日。 */
#define G4_CLOCK_DEFAULT_WEEK   5U

/* 1(默认) = 由【年月日】自己算出真实星期;
 * 0       = 用上面那个固定值(周五=5)。
 *
 * 【为什么改成自己算】现场对不上: 2026-09-22 实际是周二, 空口上发的却是 w5。
 * (液位设备自己不读这个字节、按日期重算了, 所以之前没暴露出来; 但别的设备/
 *  云端只要读它就会拿到错的星期。) 要退回固定周五就把这里改成 0。 */
#ifndef G4_CLOCK_COMPUTE_WEEK
#define G4_CLOCK_COMPUTE_WEEK   1
#endif

/* AT+CCLK? 的"数字"该按哪个时区解释(单位: 1/4 小时):
 *     0x00(默认) = 无视应答里的 ±zz, 一律当成 UTC, 再由网关 +8h 补成北京;
 *     0x20       = 相信应答里的 ±zz(即 +32 = +8h = 北京, 标准做法)。
 *
 * 【为什么默认不信 ±zz】现场实测(2026-09-22 深夜, 连续两轮):
 *   模组应答里的 ±zz 报的是 +32(声称北京时间), 但它的数字本身比真实北京时间
 *   慢整整 8 小时 —— 也就是【这个字段不可信, 它的数字其实是 UTC】。
 *   它自报"北京 16:20", 而真实北京是 00:20。设备端按协议把 16:20 当北京时间
 *   换算成 UTC, RTC 就慢了 8 小时(显示 08:20)。
 *   既然字段不可信, 就直接按 UTC 解释它的数字, 由网关补成北京。
 *
 * ⚠️ 换模组 / 换卡 / 换网络之后, 如果发现时间变成"快 8 小时", 说明模组开始
 *    老实报本地时间了 —— 把这个改回 0x20。
 *    日志里 "[G4] CCLK 模组自报时区 ..." 那行会同时打出模组报的值和本次
 *    采用的解释方式, 一眼能对上。
 */
#ifndef G4_CCLK_TZ_QUARTERS
#define G4_CCLK_TZ_QUARTERS     0x00
#endif

typedef struct
{
    uint8_t year;    /* 2000+ 的两位, 如 "21" -> 21 */
    uint8_t month;   /* 1~12 */
    uint8_t day;     /* 1~31 */
    uint8_t week;    /* 1=周一 .. 7=周日(由日期算出, 见 G4_CLOCK_COMPUTE_WEEK) */
    uint8_t hour;    /* 0~23 */
    uint8_t minute;  /* 0~59 */
} g4_clock_t;

/**
 * @brief 向 4G 模组查一次当前时间: 发 "AT+CCLK?", 解析应答
 *        应答形如: +CCLK: "21/12/27,06:56:20+32"
 *                  yy/MM/dd,hh:mm:ss+zz
 * @param out 输出年月日时分 + 星期(由日期算出)
 * @return true 取到并解析成功; false 模组不在线 / 无应答 / 格式不对
 *
 * @note ★时区★ 应答里的 ±zz 单位是【四分之一小时】(3GPP TS 27.007):
 *       +32 = +8 小时 = 北京, +00 = UTC。
 *       本函数会按它把时间【统一换算成北京时间】—— 因为协议约定发给设备的
 *       6 字节校时字段就是北京时间(设备端 app_time 按 -8h 转 UTC)。
 *       以前这个字段被整个丢掉: 模组报 UTC 时我们当北京发下去, 对方的 RTC
 *       直接慢了 8 小时。现在会自动补齐(必要时连带日期/月/年进位)。
 *
 * @note 【这是阻塞调用】内部走 g4_io_send_expect(), 最长阻塞 1s。
 *       模组不在网时直接返回 false, 不会白等。
 */
bool g4_at_get_clock(g4_clock_t *out);

/**
 * @brief 把日历时间写进 4G 模组时钟（`AT+CCLK="yy/MM/dd,hh:mm:ss±zz"`）
 * @param year 完整年份(如 2026，内部取后两位); month 1~12; day 1~31; hour/min/sec
 * @return true 模组回 OK; false 参数非法 / 离线 / 模组拒绝
 *
 * 【为什么要它】`sys.time_sync` 要把平台给的 Unix 秒写进模组时钟，
 * 而此前 `g4_at.h` **只有读**(`g4_at_get_clock`)，没有写入口 —— 见 v82 说明。
 *
 * 【时区口径】写进去的偏移量固定为 **+32(1/4 小时 = 北京 +8h)**，
 * 与模组**自己上报**的格式一致（实测 `+CCLK: "26/09/22,16:20:41+32"`）。
 * 所以调用方应当传**北京时间**的日历分量（例如用
 * `cloud_proto_unix_to_calendar(ts, +28800, ...)` 换算）。
 *
 * 【★ Unix->日历的换算刻意不放在这里 ★】放 `main.c` 做（用 cloud_proto 的
 * `unix_to_calendar`）。若让 g4_at 直接依赖 cloud_proto，会把 cloud_proto+cJSON
 * 拖进它的依赖，而 **test_state 那套主机单测正好编译 g4_at.c** -> 链接失败。
 *
 * 【实现方式】与 `g4_at_get_clock()` 同款的阻塞 AT 用法
 * (`g4_io_send_expect(..., "OK", AT_CMD_TIMEOUT_MS, ...)`)，不新开一种风险。
 * 离线时直接返回 false（模组不在网，时钟没有意义）。
 */
bool g4_at_set_clock_ymdhms(uint16_t year, uint8_t month, uint8_t day,
                            uint8_t hour, uint8_t minute, uint8_t second);

/* ==================== 驻网(CEREG)只读观测 —— 2026-10-04 第 1 步 ====================
 *
 * 【为什么单独有这一层】现有断链判据(conn URC / 连续发布失败 / 保活停滞)全在
 * MQTT 之上。射频掉了但旧 IP 还在时, 三条一条都看不见 —— 现场实测要拖到
 * HB=119s 才发现, 期间还一直往死会话里灌发布, 把模组的 AT 应用打到反复重启。
 * 驻网状态(AT+CEREG 的 <stat>)与 PS 去附着(+CGEV)是比 MQTT 更底层、更权威的证据。
 *
 * 【这一步只"看", 不"判"】判据命中也不断电、不重连 —— 只记日志。
 * 拿到"CEREG 到底会不会掉、掉多久、与 +MATREADY 什么关系"的数据后,
 * 再按已定的分层去阈值化(第 2 步)。本工程目前【没有任何一处】消费 stat。
 *
 * 【做法: 让模组主动上报, 而不是我们去轮询】上线之前(新加的 G4_STATE_REG_ARM
 * 状态)下发一次 AT+CEREG=2 + AT+CGEREP=1, 之后模组自己吐 +CEREG: / +CGEV:,
 * 由 g4_urc.c 记账并打 "[REG] ..." 原始行。
 *
 * ★为什么不是"ONLINE 期间每 30s 轮询 AT+CEREG?"(原计划)★
 *   本工程有一条【被主机测试钉死的不变式】: ONLINE 稳态下不许有任何多余的 AT
 *   (test_state.c: "ONLINE 稳态下 5 秒内发的 AT 命令数 = 0")。这条不是洁癖 ——
 *   现场日志里 +MATREADY 出现 387 次, 【每一次都在我们发出字节后 8~49ms 内】,
 *   紧接着 MQTT 会话就死(第 1 条发布收下但不回 puback, 第 2 条起 +CME ERROR: 601)。
 *   即"在 ONLINE 里敲 AT"本身就会扰动模组。用轮询去观测断网, 会亲手制造断网 ——
 *   对诊断功能来说这是最坏的失败方式(数据是假的, 而且把系统搞坏了)。
 *   被动收 URC 则是边沿触发、立刻上报, 既满足不变式, 也比 30s 轮询更不容易漏掉
 *   短时掉网。
 *   台架上若确实想看一眼轮询应答, 可把 G4_CEREG_POLL_MS 设成 >0(默认 0 = 关闭),
 *   那会违反上述不变式, 不要带到现场。
 *
 * <stat> 取值(3GPP TS 27.007 / ML307R AT 手册 §7.9):
 *   0 未注册也没在搜 / 1 已注册(本地) / 2 未注册正在搜 / 3 注册被拒 /
 *   4 未知(不在 E-UTRAN 覆盖内) / 5 已注册(漫游) / 6,7 仅短消息 / 8 紧急 /
 *   9,10 CSFB 不优先 / 11 RLOS
 *   本工程口径: {1,5} = 驻网正常, 其余 = 异常。
 *
 * 编译开关(都在 g4_at.c, 可用 -D 覆盖):
 *   G4_CEREG_DIAG_ENABLE=0   整个驻网观测不编译进去
 *   G4_CEREG_URC_ENABLE=0    不下发 CEREG=2 / CGEREP=1(那就只剩 URC 表里原有的东西)
 *   G4_CEREG_POLL_MS=30000   台架轮询模式(默认 0; 会违反"ONLINE 零 AT"不变式)
 */
typedef struct
{
    uint8_t  stat;             /* 最近一次轮询到的 <stat>; 0xFF = 还没读到
                                * ⚠️ 默认(不轮询)模式下恒为 0xFF —— 被动 URC 的
                                *    原始行在 g4_urc 那边, 见下面的 urc_line_cnt */
    uint8_t  bad_run;          /* 连续 stat ∉ {1,5} 的次数(恢复即清零); 仅轮询模式 */
    bool     urc_armed;        /* true = CEREG=2 / CGEREP=1 在本轮建链已成功下发 */
    uint32_t poll_cnt;         /* 累计轮询 AT+CEREG? 次数(仅台架轮询模式; 默认恒 0) */
    uint32_t bad_total;        /* 累计 stat ∉ {1,5} 的次数(仅轮询模式) */
    uint32_t urc_line_cnt;     /* 累计 +CEREG/+CGREG/+CREG 行数(含轮询应答) */
    uint32_t cgev_cnt;         /* 累计 +CGEV 行数 */
    uint32_t cgev_detach_cnt;  /* 其中 DETACH 类 —— 最强的断网证据 */
} g4_cereg_diag_t;

/**
 * @brief 取驻网观测数据(只读)
 * @return 指向内部静态结构, 永远非 NULL。urc_line_cnt/cgev_* 每次调用时从
 *         g4_urc 同步过来, 所以拿到的总是最新值。
 */
const g4_cereg_diag_t *g4_at_get_cereg_diag(void);

/** @brief <stat> 的人话解释(3GPP TS 27.007); 0xFF 返回"查询无应答" */
const char *g4_at_reg_stat_text(uint8_t stat);

/* ==================== 用上【模组自己】的 MQTT 能力 —— 2026-10-04 第 2 步 ====================
 *
 * 复盘结论: 我们一直在应用层手搓模组本来就有的能力, 而且搓得更差 ——
 * 模组自带自动重连/心跳/保活/遗嘱, 一个都没下发过, 靠"连续失败就给整机断电
 * 40~60 秒"来重连(现场 1.6 小时断电 27 次, 反而成了新的故障源)。
 *
 * 现在每次建链(MQTTCONN 之前)都会下发这一组(手册《12_MQTT用户手册 V6.7.0》§3.1):
 *   AT+MQTTCFG="willoption"  / "willpayload"   遗嘱(文档 7.7 要求)
 *   AT+MQTTCFG="keepalive"                    保活时间(手册: 0 或 60~65535)
 *   AT+MQTTCFG="pingreq"                      MQTT心跳间隔(60~86400)
 *   AT+MQTTCFG="pingresp"  开心跳回显          -> 能收到 +MQTTURC: "pingresp" 心跳结果
 *   AT+MQTTCFG="reconn"                        ★模组自带重连(次数/间隔/n倍退避)★
 * 【手册明确"模组掉电后配置参数不保存"】, 所以每次断电重启/快修之后都会重下发。
 * 任何一条失败都只记日志并继续 —— 这是"让模组更专业", 不是建链前提。
 *
 * 配套的判据变化(同一批改动):
 *   · conn state=1(手册: "正在重连")-> 【不动链路】, 给模组 G4_MQTT_RECONN_GRACE_MS
 *     自己回来; 超时才升级到 MQTT 快修(仍不断电)。以前是立刻 4G_RST, 等于把它掐死。
 *   · 保活停滞看门狗在"正在重连"期间不开枪(否则在它刚要回来时又按下去)。
 *   · 拿不到 puback 不再动链路(判"送达失败", 死在模组栈与 broker 之间, 快修/断电
 *     都无效); 只有 601 才说明会话真死 -> MQTT 快修。
 *   · 真断网(网络强制 PS 去附着 +CGEV DETACH)才整链路重建。
 *   · 整链路恢复带【指数退避】: 首次 min, 每次翻倍, 封顶 max —— 为此把一直闲置的
 *     network.reconnectMinSeconds / reconnectMaxSeconds 真正接上了。
 */

/**
 * @brief 注入模组级 MQTT 保活时间(来自配置 network.keepAliveSeconds)
 * @param sec 秒; 0 = 不干预(用模组默认 120s); 1~59 会被夹到 60(手册下限)
 * @note  ★由 main.c 在启动时调用一次★(network 域是"重启生效", 与文档一致)
 */
void g4_at_set_keepalive_s(uint16_t sec);

/**
 * @brief 注入恢复退避参数(来自配置 network.reconnectMin/MaxSeconds)
 * @param min_s 首次退避(秒); 0 会被当成 5
 * @param max_s 退避封顶(秒); 小于 min 时按 min 处理
 * @note  ★由 main.c 在启动时调用一次★
 *        这两个字段以前是【死配置】(只有 config_store 读写, 无消费者), 现在接上了。
 */
void g4_at_set_reconnect(uint16_t min_s, uint16_t max_s);

/** @brief 当前退避窗口(毫秒); 0 = 还没退避过(诊断/状态行用) */
uint32_t g4_at_get_backoff_ms(void);

/** @brief 累计 4G_RST 断电重启次数 —— 判"我们是不是把模组按在断电循环里" */
uint32_t g4_at_get_rst_count(void);

/* ==================== TLS / mqtts —— 2026-10-04 第 3 步 ====================
 *
 * 用户要求接自己的 EMQX Cloud(mqtts://…:8883 + 账号密码), 这是对
 * 不做与偏差记录_v107 第一条"不做 TLS"的正式反转。模组侧全部用原生指令:
 *
 *   建链时(G4_STATE_MQTT_CFG, 都在 MQTTCONN 之前)按顺序下发:
 *     AT+MSSLCFG="auth",<ssl_id>,<mode>          0=不校验证书 1=单向 2=双向
 *     AT+MSSLCFG="ignorestamp",<ssl_id>,<0|1>    是否忽略证书时间戳
 *     AT+MSSLCFG="ignoreverify",<ssl_id>,<0|1>   是否忽略认证结果
 *     AT+MSSLCFG="cert",<ssl_id>,"<ca.cer>"      单向/双向才发(要先把证书写进模组)
 *     AT+MQTTCFG="ssl",0,1,<ssl_id>              ★把 MQTT 会话绑到 SSL 上下文★
 *
 * 【当前档位: 第 1 步 = 加密但不校验证书(G4_TLS_AUTH_MODE=0)】
 *   不需要任何证书文件, 烧上去就能验证"TLS 通道通不通、handshake 要多久"。
 *   第 2 步拿到 CA 证书后再:
 *     AT+MSSLCERTWR="ca.cer",0,<len>   → 把 CA 写进模组文件系统
 *     改 G4_TLS_AUTH_MODE=1 + G4_TLS_CA_CERT "ca.cer"
 *   先敲 `#at AT+MSSLLIST=1` 看模组【出厂内置了哪些 CA】, 也许根本不用自己导。
 *
 * 【证书时间戳】模组默认 ignorestamp=1(忽略)。网关没有 RTC 电池、上电即 1970,
 *   忽略时间戳换来的是"绝不会因时钟差异握手失败", 代价只是"证书过期发现不了"。
 *   要启用严格校验就把 G4_TLS_IGNORE_STAMP 改成 0, 并确认 MQTTCONN 之前
 *   模组时钟已经通过网络校准(用户在 2026-10-04 已确认联网后时间可得)。
 *
 * 【配置接线】network.tls 这个字段以前是【死配置】(config_store 解析了但没人读),
 *   现在由 main.c 调本函数接上了。
 */

/**
 * @brief 注入 MQTT 是否走 TLS(来自配置 network.tls)
 * @param enable 0 = 普通 TCP; 非 0 = mqtts(TLS)
 * @note  ★必须在 g4_at_init() 【之后】调用★ —— init 不复位配置注入量, 但按
 *        main.c 的既有顺序(broker/subtopic/lwt 都在 init 之后)保持一致更安全。
 */
void g4_at_set_tls(uint8_t enable);

#endif /* __G4_AT_H__ */