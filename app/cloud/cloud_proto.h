/**
 * @file  cloud_proto.h
 * @brief 云平台协议路由层 —— P1-2
 *
 * 职责(只做这三件事, 不碰 MQTT 收发):
 *   1. 解析下行命令信封  {cmd, msgId, ts, auth, data}
 *   2. 鉴权 + 按命令名分发到处理函数
 *   3. 生成应答信封      {cmd, msgId, code, msg, ts, data}
 *   以及生成上行事件信封  {event, msgId, ts, data}
 *
 * 报文格式依据: gateway-protocol.docx (RoadTwin V0.1)
 *   - msgId 是【字符串】(<=16 字节), 不是数字 —— 应答原样带回
 *   - ts 是 Unix 秒
 *   - 21 条下行命令 / 7 条上行报文 / 14 个错误码
 *
 * 与 MQTT 的分界:
 *   MQTT 收发(topic 订阅、QoS、retained/LWT)属于 P1-3/P1-4/P1-5,
 *   本模块只负责"字节 -> 结构化 -> 字节", 因此可以在 PC 上用 gcc 脱机单测,
 *   也可以在没有 4G 的情况下用构造报文验证。
 *
 * 时间来源:
 *   本模块【不】自己去读时钟, 由调用方把当前 Unix 秒(now_ts)传进来。
 *   这样模块无外部依赖, 主机测试可以直接喂固定值。
 */
#ifndef __CLOUD_PROTO_H__
#define __CLOUD_PROTO_H__

#include <stdint.h>
#include <stdbool.h>
#include "cloud_json.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==================== 文档 8 章错误码 ==================== */
typedef enum
{
    CLOUD_EC_OK            = 0,     /* 成功 */
    CLOUD_EC_PARAM         = 1001,  /* 参数错误/缺失 (本实现也用于"未知命令") */
    CLOUD_EC_VER_CONFLICT  = 1002,  /* 配置版本冲突 expectVersion 不一致 */
    CLOUD_EC_NO_DEVICE     = 1003,  /* 目标设备不存在 addr 未绑定 */
    CLOUD_EC_DEV_OFFLINE   = 1004,  /* 目标设备离线 */
    CLOUD_EC_LORA_TIMEOUT  = 1005,  /* 子设备 LoRa 响应超时 */
    CLOUD_EC_STATE         = 1006,  /* 当前状态不允许执行 (也用作"尚未实现"占位) */
    CLOUD_EC_INTERNAL      = 1007,  /* 网关内部/存储错误 */
    CLOUD_EC_AUTH          = 1008,  /* 应用层密码校验失败 */
    CLOUD_EC_LIMIT         = 1009,  /* 子设备数量已达上限 */
    CLOUD_EC_NO_SPACE      = 1010,  /* 存储空间不足 */
    /* ★★2026-10-08 新增: 应答过大, 发不出去★★
     * 【为什么需要这个码】`cfg.query sections:["all"]` 的应答有 1551 字节, 而模组
     *   单条 MQTT 发布上限是 1024(超了会把模组 AT 应用层撑重启, 见 g4_at.c 的
     *   G4_PUB_DATA_MAX 注释)。原来超限是"发布被拒 -> 平台什么都收不到",
     *   只能干等超时, 而且不知道该改什么。
     * 现在网关会回一条【小的】错误应答: cmd/msgId 原样回带, code=1011,
     *   msg = "response too large; query by section"。
     * 【平台该怎么做】把 sections 拆开按域查询(见《网关使用手册》第 3 章 cfg.query)。 */
    CLOUD_EC_RESP_TOO_BIG  = 1011,
    CLOUD_EC_OTA_URL       = 2001,  /* OTA URL 非法 */
    CLOUD_EC_OTA_DOWNLOAD  = 2002,  /* OTA 下载失败 */
    CLOUD_EC_OTA_VERIFY    = 2003   /* OTA 校验失败 */
} cloud_ec_t;

/* ==================== 字段上限 ==================== */
/* 断线补报要把缓存里的紧凑记录渲染成事件对象, 所以这里要用 ec_record_t。
 * (放在这一行是因为它必须出现在下面的函数声明之前。) */
#include "event_cache.h"

#define CLOUD_CMD_MAX      24   /* 含 '\0'; 最长命令名 "sys.factory_reset" = 17 */
#define CLOUD_MSGID_MAX    20   /* 文档要求 <=16 字节, 留余量 */
#define CLOUD_AUTHMSG_MAX  40
#define CLOUD_DATA_MAX     320  /* 【小载荷】命令 data 段工作缓冲(status.report 等) */
#define CLOUD_RESP_MAX     512  /* 【小载荷】应答信封缓冲建议值(status.report 等) */

/* ⚠ P2-4 追加说明(别再用上面两个宏去装 cfg.query 的应答):
 *   cfg.query 的应答 data = 【完整配置对象】, 实测默认 1203 B、满配最坏 3584 B,
 *   再加信封 ≈ 3.7KB。所以:
 *     · cloud_proto_handle() 内部已改用文件级 static 的 CFG_JSON_MAX+64 缓冲
 *       (放栈上会爆掉 4KB 的栈, 见 startup 的 Stack_Size);
 *     · 调用方的 resp 缓冲也要给到 CFG_JSON_MAX + 256 左右, 并且同样【必须 static】
 *       —— 见 src/main.c 的 main_cloud_downlink_try()。
 *   这两个宏保留原值, 因为 status.report / 其它小应答仍在用, 改成 4KB 会无谓地吃 RAM。 */

/* ==================== 解析结果 ==================== */
typedef struct
{
    char        cmd[CLOUD_CMD_MAX];
    char        msg_id[CLOUD_MSGID_MAX];
    uint32_t    ts;
    char        auth[CLOUD_AUTHMSG_MAX];
    const char *data;           /* 指向原 JSON 内 data 对象起点; 无则 NULL */
    bool        has_ts;
} cloud_req_t;

/* ==================== 信封解析 / 生成 ==================== */

/**
 * @brief 解析下行命令信封
 * @return 0 成功; CLOUD_EC_PARAM 缺 cmd 或结构非法
 */
int cloud_proto_parse_req(const char *json, cloud_req_t *out);

/**
 * @brief 生成命令应答信封
 * @param ts        网关当前 Unix 秒(文档示例中应答的 ts 是网关时间)
 * @param data_json data 段的 JSON 文本(可为 NULL -> 输出空对象 {})
 * @return 写入长度; 0 表示失败(缓冲不足或参数错)
 */
uint16_t cloud_proto_build_resp(char *out, uint16_t cap, const cloud_req_t *req,
                                int code, const char *msg, const char *data_json,
                                uint32_t ts);

/**
 * @brief 生成上行事件信封 {event,msgId,ts,data}
 * @param event     如 "status.report"
 * @param msg_id    <=16 字节
 * @param data_json data 段 JSON 文本(可为 NULL -> {})
 * @return 写入长度; 0 失败
 */
uint16_t cloud_proto_build_uplink(char *out, uint16_t cap, const char *event,
                                  const char *msg_id, uint32_t ts, const char *data_json);

/* ==================== status.report 数据段(文档 7.7) ==================== */

/**
 * status.report 的字段(文档字段: online / fwVersion / configVersion / rsrp /
 * uptimeSeconds / mode / currentGear / sensorsOnline / sensorsTotal /
 * devicesOnline / devicesTotal)
 *
 * ⚠ 三处口径说明(实现里也标了):
 *   · rsrp:  本工程只有 AT+CSQ 换算出的 RSSI(3GPP TS 27.007: rssi=-113+2*csq),
 *            【没有真正的 RSRP】。这里按文档字段名填 RSSI 值, 后续换模组/换 AT
 *            命令拿到真 RSRP 再替换。
 *   · configVersion: 配置系统(P2)落地前恒为 0。
 *   · mode / currentGear: 边缘评估(P3)落地前恒为 "auto" / 0。
 */
typedef struct
{
    bool        online;
    const char *fw_version;
    const char *slot;          /* "A" / "B" A/B 槽位(云端据此选镜像) */
    uint32_t    config_version;
    int16_t     rsrp_dbm;
    uint32_t    uptime_s;
    const char *mode;
    const char *current_status;   /* Normal/Warning/Alarm/Offline; NULL -> Normal */
    bool        gear_null;        /* true -> currentGear 输出 JSON null */
    uint8_t     current_gear;
    uint8_t     sensors_online;
    uint8_t     sensors_total;
    uint8_t     devices_online;
    uint8_t     devices_total;
} cloud_status_t;

/**
 * @brief 生成 status.report 的 data 段 JSON 对象
 * @return 写入长度; 0 失败
 */
uint16_t cloud_proto_build_status_data(char *out, uint16_t cap, const cloud_status_t *st);

/* ==================== telemetry.report 数据段(文档 7.1) ==================== */

/** 一台传感器的遥测数据。uid 用本工程口径(26 位 hex 串), 文档里写的是 addr。 */
typedef struct
{
    const char *uid;
    int32_t     level_mm;       /* 网关换算后的水深, 单位 mm; 输出成 levelCm = mm/10 */
    uint32_t    raw_mm;         /* 传感器原始上报值(mm) */
    uint8_t     battery_pct;    /* ★ 电量【百分比】0~100 ★
                                 * 口径(3): 保持 lowBatteryPercent, **不用**文档的
                                 * lowVoltageMv/voltageMv。所以 telemetry 也照百分比发,
                                 * 字段名用 batteryPercent —— 与 sen.query/dev.query(v53)
                                 * 完全一致。
                                 * ★ 把百分比塞进一个叫 voltageMv 的字段是单位撒谎
                                 *   (平台会当毫伏用), 比缺字段更糟 —— 故改名。★
                                 * 这属于"需另行同步云平台"的偏差(D-5), 已在任务清单记录。 */
    int32_t     rssi_dbm;       /* 该帧的信号强度 */
    uint32_t    ts;             /* 该台【上报发生时】的 Unix 秒(补报也不改) */
} cloud_telemetry_sensor_t;

/** telemetry.report 的 data 段 */
typedef struct
{
    uint8_t                          sensor_count;
    const cloud_telemetry_sensor_t  *sensors;
    int32_t                          effective_level_mm;  /* 有效水位: 在线且 enabled 的取 max */
    const char                      *effective_status;    /* Normal/Warning/Alarm/Resolved/Offline */
    bool                             gear_null;           /* true -> effective.gear 输出 JSON null */
    uint8_t                          effective_gear;      /* 0~3 */
} cloud_telemetry_t;

/**
 * @brief 生成 telemetry.report 的 data 段 JSON 对象
 * @return 写入长度; 0 失败
 * @note  单位说明: levelCm 由 level_mm/10 得出(文档示例是 12.3 这种一位小数),
 *        cJSON 用 %1.15g 打印, 12.3 会原样输出成 12.3。
 */
uint16_t cloud_proto_build_telemetry_data(char *out, uint16_t cap,
                                          const cloud_telemetry_t *t);

/* ==================== event.status_change 数据段(文档 7.2) ==================== */

/** 告警状态迁移。from/to 取值: Normal / Warning / Alarm / Resolved(文档 3.4 的四个状态) */
typedef struct
{
    const char *from;
    const char *to;
    uint8_t     gear;           /* 迁移后所在档位 0~3 */
    int32_t     level_mm;       /* 迁移时刻的有效水位(mm); 输出成 levelCm = mm/10 */
} cloud_status_change_t;

uint16_t cloud_proto_build_status_change_data(char *out, uint16_t cap,
                                              const cloud_status_change_t *e);

/* ==================== event.device 数据段(文档 7.4) ==================== */

/** 子设备事件。uid 是本工程口径(26 位 hex 串); dev_type 用 "sensor"/"device" */
typedef struct
{
    const char *uid;
    const char *dev_type;
    const char *event;          /* bound / bind_failed / unbound / online / offline / low_voltage */
    /* ★ 第 12 条(2026-09-30)新增: 设备序列号 ★
     * 文档 7.4 的示例只有 addr/devType/event(没有 serial), 但用户清单明确要求
     * "event.device 补 serial/addr/devType" —— 有了它平台不必反查配置就知道
     * "是哪一台设备出的事"(serial 是现场贴在设备上的号)。
     * 可空: NULL 时【整个字段不输出】(不编空串冒充)。
     * addr 那一项: 本工程用 uid 代替(偏差 D-4, 用户已确认) —— uint16 的 addr
     * 装不下 13 字节 UID。 */
    const char *serial;
} cloud_device_event_t;

uint16_t cloud_proto_build_device_event_data(char *out, uint16_t cap,
                                             const cloud_device_event_t *e);

/* ==================== event.action 数据段(文档 7.3) ==================== */

/** 单台设备的执行结果。uid 是本工程口径(文档写 addr) */
typedef struct
{
    const char *uid;
    int         code;           /* 0 = 成功; 非 0 见错误码表 */
} cloud_action_result_t;

typedef struct
{
    uint8_t                      gear;
    bool                         gear_null;   /* true -> gear 输出 JSON null */
    const char                  *trigger;    /* auto / manual / failsafe */
    uint8_t                      result_count;
    const cloud_action_result_t *results;
} cloud_action_event_t;

uint16_t cloud_proto_build_action_data(char *out, uint16_t cap,
                                       const cloud_action_event_t *e);

/* ==================== event.buffered 数据段(文档 7.5) ==================== */

/*! 文档 7.5: 单条不超过 50 个事件, 超出【分多条】发送。
 *  所以这里只负责"装下不超过 50 条", 分页是调用方的事(超了明确返回 0, 不静默截断)。*/
#define CLOUD_BUFFERED_MAX  50u

typedef struct
{
    uint8_t            count;           /* 1 ~ CLOUD_BUFFERED_MAX */
    const char *const *events_json;     /* 每条是一段【完整的】事件对象文本,
                                         * 形如 {"event":..,"msgId":..,"ts":..,"data":{..}} */
} cloud_buffered_event_t;

/**
 * @brief 生成 event.buffered 的 data 段: {"events":[ <原始事件对象>, ... ]}
 * @return 写入长度; 0 失败(含 count==0 / count>CLOUD_BUFFERED_MAX / events_json==NULL)
 * @note  内嵌用的是 cJSON_Raw —— 原样拷贝、不加引号也不二次转义(见 cJSON 的 print_value),
 *        所以每条事件对象【保留它自己的 event/msgId/ts/data】, 补报不会改成重连时刻。
 */
uint16_t cloud_proto_build_buffered_data(char *out, uint16_t cap,
                                         const cloud_buffered_event_t *e);

/* ==================== 断线补报的单条渲染 ==================== */

/**
 * @brief 把事件缓存里的一条【紧凑记录】渲染成【完整的】事件对象文本
 * @param out          输出缓冲
 * @param cap          容量
 * @param rec          缓存记录(见 app/service/event_cache.h)
 * @param msg_id       本次补报给这条事件分配的 msgId(<=16 字节, 由调用方生成 ——
 *                     记录里【不存】msgId, 否则每条至少多 16 B, 容量直接减半)
 * @param results      仅 EC_KIND_ACTION 用: 同一次动作的多台设备执行结果
 *                     (由调用方按 seq 把同组记录收集好再传进来)
 * @param result_count results 的条数
 * @return 写入长度; **0 = 渲染不出来**(调用方应【跳过并计数】, 不要截断、不要发半条)
 *
 * 【什么情况返回 0】
 *   · 参数为空 / `event_cache_validate()` 不通过(码值不在已定义空间内);
 *   · `EC_KIND_ACTION` 但 results 为空 —— 动作事件没有执行结果是没有意义的;
 *   · 数据段或信封装不下(cap 太小 / 设备太多)—— 宁可这一条不补报,
 *     也**绝不截断**: 截断会产出"看起来正常但少报几台设备"的事件。
 *   ts 用记录里的**原始发生时刻**, 补报不改它(文档要求)。
 */
uint16_t cloud_proto_render_cached_event(char *out, uint16_t cap,
                                        const ec_record_t *rec,
                                        const char *msg_id,
                                        const cloud_action_result_t *results,
                                        uint8_t result_count);

/**
 * @brief 取下一个「补报专用」的 msgId
 * @param out 输出缓冲
 * @param cap 容量
 * @return true 已写好; false 参数错 / cap 太小(绝不写半截 id)
 *
 * 【为什么需要它】缓存记录里**不存 msgId**(存它每条至少多 16 B, 512 条的容量直接减半),
 * 所以补报时必须**重新分配**一个。工程里此前没有 msgId 生成器 —— 直发的 id 由调用方
 * 自己拼, 补报没有归属, 于是加这一个。
 *
 * 【格式】"b" + 十进制序号(如 "b17")。
 *   · 前缀 "b" 是刻意的: **补报与直发在平台侧应当能区分**(补报的 msgId 与首次发送的
 *     必然不同, 这是 v57 就写明的保真边界);
 *   · 序号**进程内单调递增**, 重启后从头开始 —— 平台的幂等记忆本就有时效,
 *     且补报本身就发生在重连之后。
 * ★ 文档只规定"字符串、≤16 字节"; 具体前缀是本工程的实现决定, **需与文档口径核对**。★
 */
bool cloud_proto_next_backfill_msg_id(char *out, uint16_t cap);

/**
 * @brief 把事件缓存里最旧的一页渲染成 `event.buffered` 的 data 段(断线补报)
 * @param out          输出缓冲
 * @param cap          容量(一批 16 条约 3~4 KB, 调用方要保证够)
 * @param batch_max    本页最多处理几条记录(0 -> 直接返回; 超过内部上限会被压到上限)
 * @param out_consumed 输出: 本页【推进过】多少条记录(**含被跳过的**)
 * @param out_events   输出: 本页真正装进了几条事件对象
 * @param out_skipped  输出: 其中有多少条被跳过(渲染不出来)
 * @return 写入长度; 0 = 本页没有可发的内容(此时仍可能 out_consumed > 0, 见下)
 *
 * 【调用方必须照这个契约做 —— 否则补报会卡死】
 *   · 返回 > 0: 发布。**成功**后 `event_cache_drop()` × `*out_consumed`;
 *     失败则一条都不要 drop(下次重来)。
 *   · 返回 0 且 `*out_consumed` > 0: 这些条**渲染不出来**(比如动作组收不齐、
 *     单个事件对象超过内部文本上限)。**必须照样 drop `*out_consumed` 次** ——
 *     不出队的话它们永远堵在队头, 补报永远发不完(后面的事件全被堵住)。
 *     ★ 但要按 `*out_skipped` 记日志/计数: 可以丢, 不许静默丢。★
 *   · 返回 0 且 `*out_consumed` == 0: 缓存空, 或这一页整体装不下 cap(下次再来)。
 *
 * 【为什么批量是 16 而不是文档上限 50】
 *   `build_buffered_data()` 要的是"指向完整事件对象文本的指针数组", 文本必须先实体化;
 *   50 × 256 B ≈ 12.8 KB 只能进静态区, 而本工程**栈只有 4 KB**。所以内部用
 *   16 × 256 B = 4 KB 的 static scratch, 512 条 -> 32 批。
 *
 * `ts` 用记录里的**原始发生时刻**(补报不改); `msgId` 由内部按 `b` 前缀重新分配。
 */
uint16_t cloud_proto_build_backfill_data(char *out, uint16_t cap, uint8_t batch_max,
                                        uint8_t *out_consumed, uint8_t *out_events,
                                        uint8_t *out_skipped);

/**
 * @brief 状态码(事件缓存的 ec_state_t / level_state_t 前四个) -> 文档里的状态名
 * @param code 1=Resolved 2=Normal 3=Warning 4=Alarm
 * @return 字符串; **NULL = 不认识**(调用方应当放弃这次上报, 不要拿 "" 冒充)
 *
 * 【为什么要公开这个映射】
 *   `event.status_change`(文档 7.2)的 from/to 是**字符串**, 而主流程手里只有
 *   状态码。这套字符串本来就在协议层(private), 若让主流程自己再写一份,
 *   两处迟早漂 —— 所以公开一个取值函数, 只保留一份实现。
 */
const char *cloud_proto_state_name(uint8_t code);

/* ==================== 时间换算 ==================== */

/**
 * @brief 日历 -> Unix 秒(文档里的 ts 字段)
 * @param year  完整年份, 如 2026
 * @param tz_offset_sec 输入时间相对 UTC 的偏移秒数(北京 = +28800)
 * @return Unix 秒(1970-01-01 00:00:00 UTC 起算)
 * @note  纯函数, 不含时区数据库; 调用方决定偏移量。
 *        网关的 g4_at_get_clock() 给的是【北京时间】, 所以传 +28800。
 */
uint32_t cloud_proto_calendar_to_unix(uint16_t year, uint8_t month, uint8_t day,
                                      uint8_t hour, uint8_t minute, uint8_t second,
                                      int32_t tz_offset_sec);

/**
 * @brief 日历 <- Unix 秒(上面那个函数的逆运算)
 * @param unix_ts       Unix 秒(1970-01-01 00:00:00 UTC 起算)
 * @param tz_offset_sec 输入时间相对 UTC 的偏移秒数(北京 = +28800)
 * @param year..second  输出(任一为 NULL -> 返回 false)
 * @return true 成功; false 参数为空
 *
 * 【为什么要它】`sys.time_sync` 要把平台给的 Unix 秒写进 4G 模组的时钟,
 * 而模组要的是 `yy/MM/dd,hh:mm:ss±zz` 这种**日历**形式 —— 工程里此前只有
 * 反向的 `cloud_proto_calendar_to_unix()`, 缺这一半(见 v82 说明)。
 *
 * 【自检方式很强】与 `calendar_to_unix()` **互为逆运算**:
 *      calendar_to_unix(unix_to_calendar(t)) == t   (同一个 tz_offset_sec)
 * 单测就靠这个往返恒等 + 几个定义性锚点(如 unix 0 = 1970-01-01 00:00:00),
 * 比逐个手写期望值可靠得多。
 *
 * 【纯函数】不碰硬件/AT/libc 的 gmtime —— 可在主机上脱机验;
 * 也不用 libc 的 time 函数(嵌入式环境下不可控)。
 */
bool cloud_proto_unix_to_calendar(uint32_t unix_ts, int32_t tz_offset_sec,
                                  uint16_t *year, uint8_t *month, uint8_t *day,
                                  uint8_t *hour, uint8_t *minute, uint8_t *second);

/* ==================== 命令处理 ==================== */

/**
 * @brief 处理一条下行命令: 解析 -> 鉴权 -> 分发 -> 生成应答
 * @param json     下行报文
 * @param now_ts   网关当前 Unix 秒(用于应答的 ts)
 * @param resp     应答输出缓冲
 * @param resp_cap resp 容量(建议 CLOUD_RESP_MAX)
 * @param resp_len 输出: 应答长度(可能为 0, 例如报文无法解析时)
 * @return 本次处理得到的错误码(0 成功); 同时已写入 resp
 * @note  即使返回非 0 也会尽量写出应答, 便于平台看到失败原因。
 */
int cloud_proto_handle(const char *json, uint32_t now_ts,
                       char *resp, uint16_t resp_cap, uint16_t *resp_len);

/**
 * @brief 取最近一次 cloud_proto_handle() 调用时传入的 now_ts
 * @return 网关当前 Unix 秒; 若还没调过 cloud_proto_handle 则返回 0
 * @note  供 LoRa 侧(绑定/解绑)在异步路径上取"网关当前时间"写 meta.updatedAt。
 *        单线程轮询, 无重入; 值在每次 cloud_proto_handle 入口处刷新。
 */
uint32_t cloud_proto_get_now_ts(void);

/* ==================== P1-6 msgId 幂等 ==================== */

/**
 * @brief 清空"最近处理过的 msgId"记忆
 *
 * @note  上电时这块内存本来就是 0, 【固件不需要主动调用】。它是给主机单测用的:
 *        测试之间要拿到确定的初始状态。
 *        刻意【不】在 MQTT 重连时调用 —— 断链重连恰恰是云端重发最密集的时刻,
 *        清掉记忆只会让同一条命令被多执行一次, 与幂等的目标正好相反。
 */
void cloud_proto_dedup_reset(void);

/**
 * @brief 幂等命中的累计次数(重复 msgId 被挡下、没有重复执行)
 * @note  只增不减的累计值, 供现场诊断用(沿用本工程"累计计数器优于瞬时状态"的做法)。
 */
uint32_t cloud_proto_dedup_hit_count(void);

/* ==================== sys.reboot 的延迟重启 ==================== */

/**
 * @brief 是否有"到点了、该执行软件复位"的 sys.reboot
 *
 * @param now_unix 网关当前 Unix 秒(main.c 的 main_now_unix())
 * @return true = 现在就复位(本函数【一次性】: 取走之后再调返回 false); false = 不用
 *
 * @note  ★ 为什么复位不由本模块自己调 NVIC_SystemReset() ★
 *        NVIC_SystemReset() 是目标机专属(主机单测链不到), 直接调会让 cloud_proto
 *        变成"不可脱机测试"的模块。所以命令只登记时刻, 复位由 main.c 主循环执行:
 *            if (cloud_proto_reboot_due(main_now_unix())) { ... NVIC_SystemReset(); }
 *        这样这条命令的全部逻辑(参数校验/延迟计算/一次性)都能在主机上验。
 */
bool cloud_proto_reboot_due(uint32_t now_unix);

/* ==================== sys.query_status 的运行时状态提供者 ==================== */

/**
 * @brief 状态提供者: 由 app 层(main.c)注册, 用来填 sys.query_status 应答 data 段
 * @param out 调用方已清零; 实现方按 cloud_status_t 的字段填
 *
 * @note  【为什么不让协议层自己去取】应答里的 rsrp / uptime / 在线数 / 档位只有
 *        app 层知道(4G 信号在 g4_at、从机在线状态在 lora_business、档位在边缘评估)。
 *        协议层 include 那些模块就重新变成"不可脱机测试"了, 所以走这个回调;
 *        主机测试注册一个假的即可。
 *        ★ 没注册时 sys.query_status 明确回 1006 —— 绝不用一堆 0 冒充状态。
 */
typedef void (*cloud_status_fn)(cloud_status_t *out);
void cloud_proto_set_status_provider(cloud_status_fn fn);

/* ==================== sys.time_sync 的"落地"钩子 ==================== */

/**
 * @brief 把平台给的 Unix 秒真正写进网关时钟(由 app 层提供)
 * @param unix_ts 平台时间(Unix 秒)
 * @return true = 已校准成功
 *
 * @note  【为什么又是个钩子】写时钟要发 AT+CCLK=... 给 4G 模组 —— 而 g4_at.h 目前
 *        【只有读】(`g4_at_get_clock`), 没有写入口; 直接在这里调会把协议层绑到 AT 层
 *        (也就不能脱机单测了), 而那属于用户正在排障的 4G 域。
 *        所以: 命令的【全部逻辑】(参数校验、"偏差 >5s 才校准"、调用、错误码)都在
 *        协议层做完并测好, 只在 main.c 里留一行注册 —— 等 4G 侧稳了、写入口有了就接上。
 *        ★ 没注册时 sys.time_sync 在"确实需要校准"的情况下明确回 1006 ★
 *          (偏差本来就在 5s 内时仍然回 0, 因为那时候什么都不用做)。
 */
typedef bool (*cloud_time_sync_fn)(uint32_t unix_ts);
void cloud_proto_set_time_sync_hook(cloud_time_sync_fn fn);

/* ==================== sen.bind / dev.bind 的配对后端钩子 ==================== */

/**
 * @brief 打开 LoRa 配对窗口(由 app / LoRa 层提供)
 * @param serial     待绑定设备的序列号(8~16 字节, 已校验)
 * @param name       平台给的名称(可为空串; 已校验长度) —— 配对成功写 sensors[]/devices[] 时用
 * @param window_s   窗口秒数
 * @param is_sensor  true = 液位传感器(写入 sensors[]), false = 联动设备(写入 devices[])
 * @return true = 窗口已开
 *
 * @note  【为什么是钩子】开配对窗口要经 LoRa 下发/监听, 属用户正在排障的链路。
 *        把命令的【全部可判逻辑】(参数校验 8~16 字节 / 容量上限 1009 / 窗口秒数)
 *        留在协议层做完并测好, 只在 main.c 注册一行 —— LoRa 侧稳了再接。
 *        ★ 没注册时 sen.bind / dev.bind 明确回 1006 ★
 *          —— 绝不"假装窗口已开"(平台会一直等 event.device{bound}, 永远等不到)。
 */
/* ★ 第 11 条(2026-09-30): 多一个 dev_type —— dev.bind 的 type(1 声光/2 LED/3 一体)
 *   必须一路传到"写 config_store 的那一刻", 否则 devices[].type 只能硬编码,
 *   而该字段文档标"只读", 平台改不了、只能解绑重绑。sen.bind 时该参数无意义(传 0)。 */
typedef bool (*cloud_pair_open_fn)(const char *serial, const char *name,
                                   uint16_t window_s, bool is_sensor,
                                   uint8_t dev_type);
void cloud_proto_set_pair_hook(cloud_pair_open_fn fn);

/** 命令名是否在 21 条命令表内 */
bool cloud_proto_is_known_cmd(const char *cmd);

/** 已注册的命令条数(应为 21) */
uint8_t cloud_proto_cmd_count(void);

/** 错误码 -> 固定英文短消息(用于应答的 msg 字段) */
const char *cloud_proto_ec_msg(int code);

/**
 * @brief 判断一段下行载荷是不是 JSON 文本(内容嗅探)
 *
 * 【为什么需要】过渡期同一条下行通道上要同时容纳两种协议:
 *   云端 JSON 命令(RoadTwin) 与 原有的二进制 LoRa 帧透传。
 *   二进制帧以 0xAA 0x55 开头, JSON 命令以 '{' 开头, 首字节即可区分。
 * 判定条件(刻意保守, 宁可漏判也不误判):
 *   跳过前导空白后第一个字符是 '{', 且后面还能找到 '}' 或 '"'。
 * @return true = 看起来是 JSON
 */
bool cloud_proto_looks_like_json(const uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* __CLOUD_PROTO_H__ */
