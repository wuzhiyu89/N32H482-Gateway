/**
 * \file  lora_business.c
 * \brief LoRa 业务层 —— 从机表 + 帧分发 + ACK 重传 + data_cache + MQTT 下行
 *
 * 本模块吸收了原 local_gateway.c 的全部业务逻辑:
 *   - lora_gateway_find_slave_by_uid → _find_slave (★ 从机表为动态表, 从
 *     config_store 的 sensors[]/devices[] 装载, 运行期支持 0x04 绑定/0x05 解绑)
 *   - lora_gateway_handle_up_frame → on_frame (协议层已做 CRC, 只做业务分发)
 *   - lora_gateway_handle_down_logic → _handle_down_logic (ACK 超时重传)
 *   - lora_gateway_send_req → lora_business_send_req (存 data_cache)
 *   - uart_send_bytes(USART2, ...) → lora_proto_send_frame()
 *   - 自己实现的 parse_frame → 删除(协议层 lora_proto_poll 统一做)
 *   - 自己维护的 rb_lora → 删除(协议层内部单缓冲)
 *
 * MQTT 对接(2026-09 改版: 【不再用 JSON】):
 *   g4_urc_set_recv_cb(lora_mqtt_recv_cb) 注册回调 —— 全模块唯一的下行入口
 *   下行 payload: 【一整个协议帧的原始二进制】(AA 55 开头, 含 CRC), 例如
 *       AA 55 01 11 00 01 21 00 01 01 07 05 00 0A 00 00 DC 05 98 E0
 *     ⚠️【2026-09 更正 —— 这一处原来写的 CRC 是 38 1B, 是错的】
 *        正确算法: crc16_modbus 覆盖【版本字节 .. data 末尾】共 16 字节,
 *        结果低字节在前。crc16_modbus(01 11 00 01 21 00 01 01 07 05 00
 *        0A 00 00 DC 05) = 0xE098 -> 帧尾写 98 E0。
 *        写成 38 1B 时液位设备端 CRC 校验不过, 会回 "CRC error" 丢弃整帧
 *        (设备日志: [cmd] CRC error: expect 98 E0, got 38 1B)。
 *        工具自检: crc16_modbus("123456789") 必须 = 0x4B37。
 *     校验 lora_proto_parse_raw()(帧头/版本/长度/CRC) → 转发到空口。
 *     同时保留旧格式 ASCII hex 文本的兼容分支(不以 AA55 开头才走 hex 解码)。
 *   上行 payload: 协议帧的 ASCII hex 文本(lora_business_publish_report) ——
 *     上行仍是 hex 文本, 这一条没变(云端按文本收)。
 *
 *   【台架联调开关 LORA_DOWNLINK_RAW_FORWARD】(见 lora_mqtt_recv_cb 上方)
 *     1 = 粗略直转: 校验通过就原样发到空口(不校时/不解释 func);
 *     0 = 走原业务解释(0x01 需要 4G 校时, 0x03 立即下发)。
 */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "lora_business.h"
#include "lora_internal.h"
#include "lora_proto.h"
#include "data_cache.h"
#include "param_save.h"        /* param_get_ptr() — 网关 UID 从 Flash 参数区读 */
#include "config_store.h"      /* P2-5 起: cfg_uid_from_bytes / 查上报周期(失联检测) */
#include "sensor_offline.h"    /* 液位失联判定的纯逻辑(用户补充要求第 3 条) */
#include "cloud_proto.h"       /* P5 绑定/解绑: cloud_proto_get_now_ts() 用于 config_store 落盘时间戳 */
#include "level_eval.h"        /* 档位联动: level_gear_*() —— 组帧/选目标是纯逻辑, 在那边单测 */
/* 2026-10-14: 重绑设备时清该 UID 的历史快照(last_value_clear_uid) —— 新设备不继承旧值 */
#include "last_value.h"

/* ★★2026-10-18: UID 长度只有一个真相源 ★★
 * 同一件事在三个头文件里各有一个宏(因为它们互相不 include, 避免循环依赖):
 *     config_store.h : CFG_UID_BYTES     = 13
 *     param_save.h   : PARAM_UID_BYTES   = 13
 *     lora_proto.h   : LORA_UID_BYTES    = 13
 * ⇒ 这里(同时能看到 CFG_UID_BYTES 与 LORA_UID_BYTES 的唯一地方)用静态断言钉住,
 *   将来谁只改一处、忘了另外两处, 【编译就失败】, 而不是让帧悄悄错位。
 *   (param_save 那个宏在 param_save.c 里另有一条断言盯着 —— 那文件看不到 config_store.h。) */
_Static_assert(CFG_UID_BYTES == LORA_UID_BYTES,
               "CFG_UID_BYTES and LORA_UID_BYTES must match");
#include "g4_urc.h"
#include "g4_at.h"
#include "bsp_tim.h"
#include "log.h"

/* ========== 内部: 从机表(动态) ==========
 * 【2026-09-29 改造, P5】从机表不再写死, 改为从 config_store 的 sensors[]/devices[]
 * 装载; 运行期还支持绑定(0x04)/解绑(0x05)动态增删。
 *   - LORA_SLAVE_MAX: RAM 数组上限 = 传感器槽 + 设备槽(够装即可, 真正的容量
 *     约束在 config_store_can_add 的「总数≤5/液位≤4/声光≤3」)
 *   - g_slave_count: 运行期实际台数(由 _slave_table_load / _slave_table_add /
 *     _slave_table_del 维护), 取代了原来的编译期 LORA_SLAVE_CNT。
 * (LORA_SLAVE_MAX 定义在 .c 里: .h 不 include config_store.h, 所以那里只留注释。) */
#define LORA_SLAVE_MAX  (CFG_SENSOR_MAX + CFG_DEVICE_MAX)
static lora_slave_t g_slave_list[LORA_SLAVE_MAX];
static uint8_t      g_slave_count = 0U;
static uint32_t g_last_poll_tick;
/* 网关 UID 在 lora_business_init 里从 param_save 读入 (支持后续本地/云端修改) */
uint8_t g_gw_uid[CFG_UID_BYTES];

/* ========== 内部: 设备绑定窗口状态机(P5, LoRa 0x04) ==========
 *
 * 【为什么绑定只在云端开窗口时才监听 0x04】(用户 2026-09-29 决策):
 *   平时 0x04 是个"陌生设备广播" —— 任何附近设备都可能发, 网关常驻监听就
 *   会被无关帧打扰并回 ACK(空口被刷)。所以常态是 BIND_IDLE, 只有当云端
 *   下发 dev.bind{serial,name,pairWindowSeconds}、经 pair_hook 调
 *   lora_business_open_bind_window() 时才进入 BIND_LISTENING 打开窗口,
 *   到点自动回 IDLE。
 *
 * 【状态转移】(tick 用 get_tick_ms, 32 位 ms, 用无符号差值判回绕):
 *   BIND_IDLE         -- open_bind_window -->  BIND_LISTENING
 *   BIND_LISTENING    -- 收到 0x04      -->  BIND_WAIT_DEV_ACK (记 pending_uid, 立即发一次 0x80)
 *   BIND_WAIT_DEV_ACK -- 收到对端 0x80 -->  BIND_IDLE  + 写 config_store + 触发 BOUND
 *   BIND_WAIT_DEV_ACK -- 每 5s          -->  重发 0x80 ACK(给设备机会重收)
 *   任意 LISTENING/WAIT_DEV_ACK -- 窗口到点 -->  BIND_IDLE (WAIT_DEV_ACK 时还要触发 BIND_FAILED)
 *
 * 【只允许一个绑定并发】: 同一时刻只跟踪一个发起绑定的设备(pending_uid),
 *   窗口里第二个不同地址的 0x04 直接被 0x81 拒掉(不打断当前握手)。
 */
typedef enum
{
    BIND_IDLE         = 0,   /* 未开窗口 */
    BIND_LISTENING    = 1,   /* 窗口已开, 在等 0x04 */
    BIND_WAIT_DEV_ACK = 2,   /* 已回 0x80, 等设备回 0x80 确认 */
} bind_state_t;

static struct
{
    bind_state_t state;
    uint32_t     deadline_tick;      /* 窗口到期 tick(进入 LISTENING 时设定) */
    uint32_t     last_ack_tx_tick;   /* 上次向设备发 0x80 ACK 的 tick */
    char         expected_serial[CFG_SERIAL_MAX + 1U];   /* 落盘用, 来自云端 dev.bind */
    char         expected_name[CFG_NAME_MAX + 1U];
    bool         is_sensor;          /* sen.bind=true 液位 / dev.bind=false 声光 */
    uint8_t      dev_type;           /* ★ 第 11 条: dev.bind 给的联动设备类型
                                      * (cfg_dev_type_t 1/2/3); sen.bind 时无意义 */
    uint8_t      pending_uid[CFG_UID_BYTES];     /* 发起绑定的设备地址(收到 0x04 后登记) */
} g_bind;

/* ========== 解绑两阶段状态机(2026-09-30 用户最终口径) ==========
 *
 * 【为什么 g_unbind 完全独立、不复用从机 pending_func 重传逻辑】
 *   原来的解绑走的是从机表的 pending_func=0x05 + _handle_down_logic 5s 重试/3 次
 *   耗尽强制删, 那套口径是"等不到 ACK 就强制删设备" —— 与用户最终口径冲突:
 *     - 阶段1 等不到 ack 是【失败上报】, 不删设备
 *     - 阶段2 收不到 ack 才是【成功】, 收到 ack 才是失败
 *   而且阶段切换(等15s / 阶段2 循环)用从机 pending_func 表达不了。
 *   所以 g_unbind 单独维护状态机, 只把从机表的 state/pending_func 当"锁":
 *   启动时把目标 slave 置 WAIT_ACK+pending_func=0x05, _handle_down_logic 看到
 *   pending_func==0x05 就 continue 跳过(不干扰); 结束时清回 IDLE。
 *
 * 时序(详见 lora_business.h 上方常量注释):
 *   阶段1(30s): 每 3s 发 0x05, 收到 ack 立即停发 → 等15s
 *     30s 没收到 ack → 上报 UNBIND_FAILED(从机记录保留)
 *   等15s: 给设备清自己保存的网关值
 *   阶段2(10s, 最多循环3次): 每 3s 发 0x05
 *     10s 没收到 ack → 解绑成功 → 删 config_store + 从机表 → 上报 UNBOUND
 *     收到 ack → 设备没清 → 循环阶段2 的 10s(不再等15s)
 *     3 次都收到 ack → 设备故障 → 上报 UNBIND_FAULT(从机记录保留不删)
 *
 * ★ tick 比较一律用 (uint32_t)(now - start) >= DURATION 无符号差值(自动处理回绕) */
typedef enum
{
    UNBIND_IDLE        = 0,   /* 未在解绑 */
    UNBIND_PHASE1      = 1,   /* 阶段1: 30s 内每 3s 发 0x05, 收 ack 立即停 */
    UNBIND_WAIT_CLEAR  = 2,   /* 等 15s 给设备清网关值 */
    UNBIND_PHASE2      = 3,   /* 阶段2: 10s 内每 3s 发 0x05, 收 ack 循环/故障 */
} unbind_state_t;

static struct
{
    unbind_state_t state;
    uint8_t        uid[CFG_UID_BYTES];            /* 解绑目标设备 UID(13B) */
    bool           is_sensor;         /* 设备类型(sensor=true / alarm=false), 用于事件回调 */
    uint8_t        slave_idx;         /* 从机表索引(0xFF = 无效); 解绑期间作"锁"用 */
    uint32_t       phase_start_tick;  /* 当前阶段开始 tick */
    uint32_t       last_send_tick;    /* 上次发 0x05 的 tick */
    uint8_t        phase2_retry;      /* 阶段2 已收 ack 的次数(>=3 = 设备故障) */
} g_unbind =
{
    .state        = UNBIND_IDLE,
    .slave_idx    = 0xFFU,
    .phase2_retry = 0U,
};

static lora_bind_event_fn g_bind_event_cb = NULL;

/* 字符 → 4bit 数值，非法返回 0xFF */
static uint8_t _hex_char2nibble(char c)
{
    if (c >= '0' && c <= '9') return (uint8_t)(c - '0');
    if (c >= 'A' && c <= 'F') return (uint8_t)(c - 'A' + 10);
    if (c >= 'a' && c <= 'f') return (uint8_t)(c - 'a' + 10);
    return 0xFFU;
}

/**
 * \brief hex 字符串 → 二进制数组
 * \param hex      输入 hex 字符串（NULL 结尾）
 * \param hex_len  字符串长度（不含 NULL）
 * \param out      输出缓冲（必须 >= hex_len/2）
 * \param out_max  out 缓冲最大字节数
 * \return 实际转换的字节数; -1 输入长度为奇数或非法 hex
 */
static int _hex_str2bin(const char *hex, int hex_len, uint8_t *out, int out_max)
{
    if (hex == NULL || (hex_len & 1) != 0) return -1;   /* 奇数长度 hex 无法成字节 */
    int byte_cnt = hex_len / 2;
    if (byte_cnt > out_max) return -1;

    int bi = 0;
    for (int i = 0; i < byte_cnt; i++)
    {
        uint8_t hi = _hex_char2nibble(hex[i * 2]);
        uint8_t lo = _hex_char2nibble(hex[i * 2 + 1]);
        if (hi == 0xFFU || lo == 0xFFU) return -1;
        out[bi++] = (uint8_t)((hi << 4) | lo);
    }
    return byte_cnt;
}

/**
 * \brief 二进制数组 → ASCII 十六进制字符串(大写)
 * \param in       输入字节
 * \param in_len   字节数
 * \param out      输出缓冲, 需要 (in_len*2 + 1) 字节
 * \param out_max  out 的容量(含结尾 '\0')
 * \return 写出的字符数(不含 '\0'); 0 = 参数非法/放不下
 *
 * 云端↔网关的 MQTT 载荷就是这种"文本 hex": 一帧 20 字节 → 40 个字符。
 * 走文本而不是裸二进制, 是因为 4G 模组的 +MQTTURC: "publish" 本身就是
 * 按行输出的文本通道 —— 用文本不用改底层驱动, 日志里也能直接看懂。
 */
static uint16_t _hex_bin2str(const uint8_t *in, uint16_t in_len, char *out, uint16_t out_max)
{
    static const char HEX[] = "0123456789ABCDEF";
    uint16_t i;
    uint16_t n = 0U;

    if ((in == NULL) || (out == NULL)) return 0U;
    if (((uint32_t)in_len * 2U + 1U) > (uint32_t)out_max) return 0U;

    for (i = 0U; i < in_len; i++)
    {
        out[n++] = HEX[(in[i] >> 4) & 0x0FU];
        out[n++] = HEX[in[i] & 0x0FU];
    }
    out[n] = '\0';
    return n;
}

/* ---- 以上为内部辅助函数 ---- */

/* ==================== ★2026-10-08: 上行 hex 通道已整体删除 ★ ====================
 *
 * 【删掉的是什么】
 *   原来这里有个 lora_business_publish_report(): 把 0x02 上报帧【整帧原样】组出来、
 *   转成 ASCII hex、再用 g4_at_mqtt_publish(NULL, ...) 发到那条"默认主题"
 *   (mod/mod_upload)。那是早期"云端自己解帧"的设计。
 *
 * 【为什么删】
 *   甲方明确要求: 不再使用 mod/mod_upload, 历史残留一并清除。
 *   而且它本来就和 mod/telemetry 是【两条并行的同一份数据】——
 *   0x02 收到时的 data_cache_up_push() 已经把值写进 last_value 快照表,
 *   由 main.c 组装成 JSON 走 rt/gw/{gwId}/telemetry 上报。留着那条 hex 通道
 *   只会让同一份液位在两个主题上各来一遍, 平台侧还要提防"两处数据不一致"。
 *
 * 【删干净了哪些地方】(避免以后有人以为还有一条上行)
 *   · 本文件的 lora_business_publish_report() 与 g_up_hex 缓冲
 *   · data_cache.c 的 data_cache_poll() 及其调用(lora_business_poll 里那处)
 *   · g4_at.c 的 G4_MQTT_PUB_TOPIC 宏; 并且 g4_at_mqtt_publish(NULL,...)
 *     现在【直接返回失败】—— 杜绝换个地方又发到废弃主题上
 *   · g4_at.c 的 AT+MQTTSUB 默认主题 G4_MQTT_SUB_TOPIC(mod/pc_down)
 *
 * 【不存在"数据丢失"】: 0x02 的解析、并入 last_value、以及 ACK 应答全都没动,
 *   rt/gw/{gwId}/telemetry 的上报路径完全不受影响。
 * ========================================================================= */

/* ==================== 设备类型的判断规则(现场规则) ====================
 *
 * 【从机设备地址的"首位"就是设备类型】
 *     1xxxxx = 网关      2xxxxx = 液位传感器      3xxxxx = 声光报警器
 *   设备 ID 一共 16 个(首位 0~F)。
 *
 * 地址是 3 字节【BCD】(21100001 这种 6 位十进制数), 例如
 *     21 00 01 -> "210001"   31 00 01 -> "310001"
 * 所以"首位"就是 uid[0] 的【高半字节】。
 *
 * 【为什么要有这条规则】绑定/解绑场景里, 收到的 0x04 广播帧的 dev_uid 就是
 * 发起绑定的设备自己, 网关靠这条规则把它分到液位/声光槽里, 不用设备自报类型。
 */
#define LORA_UID_TYPE_GW      1U
#define LORA_UID_TYPE_LEVEL   2U
#define LORA_UID_TYPE_ALARM   3U

static uint8_t _dev_type_of_uid(const uint8_t uid[CFG_UID_BYTES])
{
    return (uint8_t)(uid[0] >> 4);
}

/* 运行期从机表: dev_uid/dev_type 由 _slave_table_load 从 config_store 装载,
 * 其余字段跑起来才有意义。增删见 _slave_table_add / _slave_table_del。 */
/* (g_slave_list / g_slave_count 在文件开头已声明) */

/* ---------- 从机查找 ----------
 * 【只按地址匹配, 不看 is_online】: 在不在线都要能找到, 否则云端刚下发的指令
 * 会因为设备"暂时离线"而找不到目标被丢掉。 */
static uint8_t _find_slave(const uint8_t uid[CFG_UID_BYTES])
{
    for (uint8_t i = 0U; i < g_slave_count; i++)
    {
        if (memcmp(g_slave_list[i].dev_uid, uid, CFG_UID_BYTES) == 0)
        {
            return i;
        }
    }
    return 0xFFU;
}

/* ---------- 收到帧时"摸一下"从机: 标记在线 + 记时间戳 ----------
 * 地址不在表里就返回 0xFF, 调用方据此丢弃该帧。
 *
 * ⚠【这里故意不碰 last_report_tick】: 本函数对【任何】帧(含 0x80/0x81 ACK)都调用,
 *   而失联检测要的是"上次【液位上报】时间"。假如在这里顺手更新它, 一台只回 ACK、
 *   再也不上报的液位设备就会被永远当成"刚上报过", 失联事件永远发不出来。 */
static uint8_t _touch_slave(const uint8_t uid[CFG_UID_BYTES])
{
    uint8_t idx = _find_slave(uid);
    if (idx == 0xFFU) return idx;
    g_slave_list[idx].is_online     = true;
    g_slave_list[idx].last_recv_tick = get_tick_ms();
    return idx;
}

/* ---------- _slave_table_add: 往运行期表末尾追加一台 ----------
 * 由绑定流程在收到 0x04 广播帧且网关回了 ACK、设备也回 ACK 确认后调用。
 * 容量约束(总数≤5/液位≤4/声光≤3)由调用前 config_store_can_add 兜底,
 * 这里只保证 RAM 表不越界、不重复。返回索引; -1=表满/重复。 */
static int _slave_table_add(const uint8_t uid[CFG_UID_BYTES], slave_type_t type)
{
    if (g_slave_count >= LORA_SLAVE_MAX) return -1;
    if (_find_slave(uid) != 0xFFU)       return -1;   /* 已在表里 */

    uint8_t idx = g_slave_count;
    lora_slave_t *s = &g_slave_list[idx];
    memset(s, 0, sizeof(*s));
    memcpy(s->dev_uid, uid, CFG_UID_BYTES);
    s->dev_type  = type;
    s->state     = SLAVE_STATE_IDLE;
    s->is_online = true;                          /* 绑定当场就在线 */
    s->retry_cnt = 0U;
    g_slave_count++;
    return (int)idx;
}

/* ---------- _slave_table_del: 删除指定地址的从机 ----------
 * 把末尾元素前移填补空洞, 保持数组紧凑。由解绑流程在收到 0x80 ACK(对 0x05)
 * 或重试耗尽后调用。返回 0=成功; -1=不在表里。 */
static int _slave_table_del(const uint8_t uid[CFG_UID_BYTES])
{
    uint8_t idx = _find_slave(uid);
    if (idx == 0xFFU) return -1;

    uint8_t last = (uint8_t)(g_slave_count - 1U);
    if (idx != last)
    {
        /* 把最后一台搬到被删的那一格 —— 比 memmove 整段前移省事, 且本表
         * 外部只用 _find_slave 按地址查, 不认索引顺序, 搬动不影响别的逻辑。 */
        g_slave_list[idx] = g_slave_list[last];
    }
    memset(&g_slave_list[last], 0, sizeof(g_slave_list[last]));
    g_slave_count--;
    return 0;
}

/* ---------- 从 config_store 装载运行期表(init 时调一次) ----------
 * config_store 里的 sensors[](液位) 和 devices[](声光等联动设备) 是真相源,
 * 这里把它们的 uid 解成 13 字节填进 g_slave_list。
 * 顺序: 先 sensors 再 devices, 与 _slave_table_add 的末尾追加语义一致。 */
static void _slave_table_load(void)
{
    const config_store_t *cfg = config_store_get();
    uint8_t i;

    memset(g_slave_list, 0, sizeof(g_slave_list));
    g_slave_count = 0U;

    /* 液位传感器 → SLAVE_TYPE_SENSOR */
    for (i = 0U; i < cfg->sensor_count && i < CFG_SENSOR_MAX; i++)
    {
        if (g_slave_count >= LORA_SLAVE_MAX) break;
        uint8_t uid3[CFG_UID_BYTES];
        if (!cfg_uid_to_bytes(cfg->sensors[i].uid, uid3)) continue;
        lora_slave_t *s = &g_slave_list[g_slave_count];
        memcpy(s->dev_uid, uid3, CFG_UID_BYTES);
        s->dev_type  = SLAVE_TYPE_SENSOR;
        s->state     = SLAVE_STATE_IDLE;
        s->is_online = false;
        g_slave_count++;
    }

    /* 联动设备(声光等) → SLAVE_TYPE_ALARM_OUT */
    for (i = 0U; i < cfg->device_count && i < CFG_DEVICE_MAX; i++)
    {
        if (g_slave_count >= LORA_SLAVE_MAX) break;
        uint8_t uid3[CFG_UID_BYTES];
        if (!cfg_uid_to_bytes(cfg->devices[i].uid, uid3)) continue;
        lora_slave_t *s = &g_slave_list[g_slave_count];
        memcpy(s->dev_uid, uid3, CFG_UID_BYTES);
        s->dev_type  = SLAVE_TYPE_ALARM_OUT;
        s->state     = SLAVE_STATE_IDLE;
        s->is_online = false;
        g_slave_count++;
    }

    log_info("[GW] 从机表装载完成, 共 %u 台:\r\n", (unsigned)g_slave_count);
    for (i = 0U; i < g_slave_count; i++)
    {
        log_info("[GW]   [%u] dev=%02X%02X%02X type=%s\r\n", (unsigned)i,
                 g_slave_list[i].dev_uid[0], g_slave_list[i].dev_uid[1],
                 g_slave_list[i].dev_uid[2],
                 (g_slave_list[i].dev_type == SLAVE_TYPE_SENSOR) ? "SENSOR(液位)"
                                                                 : "ALARM_OUT(声光)");
    }
}

/* ========== 内部: 设备绑定窗口状态机(P5, LoRa 0x04) ==========
 *
 * 下列 7 个静态函数配合 on_frame / lora_business_poll 实现:
 *   _bind_fire()              —— 统一出口: 触发事件回调(没注册只打日志)
 *   _bind_send_ack_to_pending()—— 向发起绑定的设备发 0x80 ACK
 *   _bind_handle_0x04()       —— 收到 0x04 绑定请求
 *   _bind_handle_dev_ack()    —— 收到设备回的 0x80 确认(绑定成立)
 *   _bind_logic()             —— 周期调用: 重发 ACK + 窗口到期
 *   lora_business_set_bind_event_cb() / lora_business_open_bind_window() —— 对外 API
 *
 * 时序概要(详见 g_bind 上方注释):
 *   设备发 0x04 → 网关回 0x80 → 设备回 0x80 → 绑定成功。
 *   网关每 5s 重发一次 0x80(LORA_BIND_ACK_INTERVAL_MS), 直到设备回 0x80 或窗口到期。
 */

/* 统一的事件出口。uid_hex 必须是 6 字符大写 hex 串, 或 NULL 表示"无具体设备"(如超时) */
static void _bind_fire(const char *uid_hex, bool is_sensor, lora_bind_event_t event)
{
    const char *ev_str =
        (event == LORA_BIND_EVENT_BOUND)       ? "bound"       :
        (event == LORA_BIND_EVENT_BIND_FAILED) ? "bind_failed" :
        (event == LORA_BIND_EVENT_UNBOUND)     ? "unbound"     : "unknown";

    log_info("[BIND] event=%s uid=%s type=%s\r\n",
             ev_str,
             (uid_hex != NULL) ? uid_hex : "(none)",
             is_sensor ? "sensor" : "device");

    if (g_bind_event_cb != NULL)
    {
        g_bind_event_cb(uid_hex, is_sensor, event);
    }
}

/* 向发起绑定的设备(g_bind.pending_uid)发 0x80 ACK(应答它的 0x04 绑定请求) */
static void _bind_send_ack_to_pending(void)
{
    (void)lora_business_send_ack(g_gw_uid, g_bind.pending_uid,
                                 LORA_FUNC_BIND_REQ, true, 0U);
    g_bind.last_ack_tx_tick = get_tick_ms();
    log_info("[BIND] 已向 %02X%02X%02X 发 0x80 ACK(应答 0x04 绑定请求)\r\n",
             g_bind.pending_uid[0], g_bind.pending_uid[1], g_bind.pending_uid[2]);
}

/* 收到 0x04 绑定请求: 检查容量 → 登记 pending_uid → 立即回 0x80 */
static void _bind_handle_0x04(const lora_frame_t *frame)
{
    /* 容量检查(总数≤5 / 液位≤4 / 声光≤3) —— 不够直接回 0x81, 不进入握手 */
    if (!config_store_can_add(g_bind.is_sensor))
    {
        log_info("[BIND] !! %02X%02X%02X 请求绑定, 但%s已达上限, 回 0x81 拒绝\r\n",
                 frame->dev_uid[0], frame->dev_uid[1], frame->dev_uid[2],
                 g_bind.is_sensor ? "液位传感器" : "声光设备");
        (void)lora_business_send_ack(g_gw_uid, frame->dev_uid,
                                     LORA_FUNC_BIND_REQ, false, 0x06U);  /* err 0x06 = 容量满 */
        return;
    }

    /* 已在等另一台设备的 ACK → 直接拒掉这次(单绑定并发) */
    if (g_bind.state == BIND_WAIT_DEV_ACK)
    {
        log_info("[BIND] !! %02X%02X%02X 请求绑定, 但已在等 %02X%02X%02X 回 ACK, 回 0x81 拒绝\r\n",
                 frame->dev_uid[0], frame->dev_uid[1], frame->dev_uid[2],
                 g_bind.pending_uid[0], g_bind.pending_uid[1], g_bind.pending_uid[2]);
        (void)lora_business_send_ack(g_gw_uid, frame->dev_uid,
                                     LORA_FUNC_BIND_REQ, false, 0x05U);  /* err 0x05 = 忙 */
        return;
    }

    /* 登记 pending_uid → 转入 WAIT_DEV_ACK → 立即发 0x80 */
    memcpy(g_bind.pending_uid, frame->dev_uid, CFG_UID_BYTES);
    g_bind.state = BIND_WAIT_DEV_ACK;
    log_info("[BIND] 收到 %02X%02X%02X 的 0x04 绑定请求(type=%s), 进入 WAIT_DEV_ACK\r\n",
             g_bind.pending_uid[0], g_bind.pending_uid[1], g_bind.pending_uid[2],
             g_bind.is_sensor ? "sensor" : "device");
    _bind_send_ack_to_pending();
}

/* 收到设备回的 0x80 确认: 写 config_store + 追加运行期从机表 + 触发 BOUND */
static void _bind_handle_dev_ack(const lora_frame_t *frame)
{
    /* 校验是不是 pending_uid 回的(避免别的设备 0x80 蹭进来) */
    if (memcmp(frame->dev_uid, g_bind.pending_uid, CFG_UID_BYTES) != 0)
    {
        log_info("[BIND] !! 收到 %02X%02X%02X 的 0x80, 但 pending 是 %02X%02X%02X, 忽略\r\n",
                 frame->dev_uid[0], frame->dev_uid[1], frame->dev_uid[2],
                 g_bind.pending_uid[0], g_bind.pending_uid[1], g_bind.pending_uid[2]);
        return;
    }

    /* 1) hex 串形式(供 config_store / 事件回调用) */
    char uid_hex[CFG_UID_STR_MAX];
    /* cfg_uid_from_bytes 把 13 字节转 hex, 不会失败(非全 0 即有效) */
    (void)cfg_uid_from_bytes(g_bind.pending_uid, uid_hex);

    /* 2) 追加到运行期从机表(失败就别写 config_store, 免得两边不一致) */
    slave_type_t type = g_bind.is_sensor ? SLAVE_TYPE_SENSOR : SLAVE_TYPE_ALARM_OUT;
    int idx = _slave_table_add(g_bind.pending_uid, type);
    if (idx < 0)
    {
        /* ==================================================================
         * ★★2026-10-14 重绑: 已在从机表 ⇒ 允许再绑, 但【不新增】条目 ★★
         * ------------------------------------------------------------------
         * 【用户场景】设备坏了换新, 重新烧录后回到【同一默认地址】
         *   ⇒ 网关必须允许同 UID 再绑一次; 若拒绝, 现场换设备就必须先解绑,
         *     很难操作。且从机表里【不能出现两条同 UID】。
         *
         * 【用户口径, 三条】
         *   ① serial/name 用本次绑定窗口里的新值【覆盖】(换了设备, 标识也该换);
         *   ② 【不】动 report_period_s 等运行参数 —— 平台一定会再下发一次上报周期,
         *      等于给新设备设一遍(动了反而把平台设的值悄悄改回默认 60);
         *   ③ 【不继承】旧设备的历史值 ⇒ 清掉该 UID 的快照。
         *      理由: 否则新设备还没上报时 sen.query 就回旧水位、level_supervisor
         *      会拿旧水位算档位(新设备一上电就按旧水位点灯)、失联判定也可能因
         *      "缓存里的 rx_tick_ms 很新"而误判在线。
         *
         * ⚠️ 这里【不能】用 config_store_bind_device() —— 它对已存在的 UID 直接
         *    返回 CFG_ERR_RANGE(见该函数开头), 会静默什么都不改。
         * ⚠️ 也【不能】用 config_store_unbind_device() + bind "先删再建" ——
         *    unbind 会连带清掉 gears[].target_uid 里指向该 UID 的条目,
         *    那台设备以后收不到联动指令(能绑定、能上报, 就是不动灯)。
         *
         * ⚠️⚠️ 【本分支目前没有主机单测覆盖】⚠️⚠️
         *   _host_test 里没有任何套件链接【真的】app/links/lora/lora_business.c
         *   (test_cloud 用的是 _host_test/fake_lora_business.c 那个假实现),
         *   因为本文件依赖 LoRa/radio 时序。⇒ 改这段代码【必须上板验证】,
         *   "主机单测全绿"对它没有意义。
         *   已有的主机覆盖只到它调用的两个新函数:
         *     · config_store_rebind_device() -> test_cfg 用例 [14]
         *     · last_value_clear_uid()       -> test_last_value 用例 [10]
         * ================================================================== */
        if ((idx == -1) && (_find_slave(g_bind.pending_uid) != 0xFFU))
        {
            cfg_result_t rb;

            log_info("[BIND] %s %s 重新绑定(已在从机表, 不新增): serial=%s name=%s\r\n",
                     g_bind.is_sensor ? "液位传感器" : "声光设备",
                     uid_hex,
                     g_bind.expected_serial[0] ? g_bind.expected_serial : "(none)",
                     g_bind.expected_name[0]    ? g_bind.expected_name    : "(none)");

            /* ① 覆盖 serial/name(设备侧连 type 一起更新; 运行参数一律不动) */
            rb = config_store_rebind_device(uid_hex, g_bind.expected_serial,
                                            g_bind.expected_name, g_bind.dev_type,
                                            cloud_proto_get_now_ts());
            if (rb != CFG_OK)
            {
                /* 落盘失败: 运行期表已经是"已存在"状态, 不必回滚, 但要让现场看见 */
                log_info("[BIND] !! 重绑写 config_store 失败(code=%d), 本次只更新了运行期表\r\n",
                         (int)rb);
            }

            /* ③ 清该 UID 的历史快照(新设备不继承旧值; 其它 UID 不受影响) */
            (void)last_value_clear_uid(g_bind.pending_uid);

            _bind_fire(uid_hex, g_bind.is_sensor, LORA_BIND_EVENT_BOUND);
            g_bind.state = BIND_IDLE;
            return;
        }

        /* 到这里就是真失败: 从机表【满了】 */
        log_info("[BIND] !! 从机表【已满】, 无法绑定 %s(idx=%d) —— 请先解绑不用的设备\r\n",
                 uid_hex, idx);
        _bind_fire(uid_hex, g_bind.is_sensor, LORA_BIND_EVENT_BIND_FAILED);
        g_bind.state = BIND_IDLE;
        return;
    }

    /* 3) 写 config_store(失败回滚运行期表)
     * ★★ 必须与 CFG_OK(== 0) 比较, 不能用 ! 取反 ★★
     * cfg_result_t 的语义是"0 = 成功"(CFG_OK=0, 其余全是错误码), 写成
     * `if (!config_store_bind_device(...))` 会把【每一次绑定成功】都当成失败:
     *   刚加进运行期表的设备被回滚掉、事件报 bind_failed, 但 config_store 内部
     *   其实已经 save 落 Flash 了 —— 结果"云端看到绑定失败, 重启后设备又冒出来"。
     * (2026-09-29 评审修正) */
    uint32_t now_ts = cloud_proto_get_now_ts();
    if (config_store_bind_device(uid_hex, g_bind.expected_serial,
                                 g_bind.expected_name, g_bind.is_sensor,
                                 g_bind.dev_type,
                                 now_ts) != CFG_OK)
    {
        log_info("[BIND] !! config_store_bind_device 失败, 回滚运行期表\r\n");
        (void)_slave_table_del(g_bind.pending_uid);
        _bind_fire(uid_hex, g_bind.is_sensor, LORA_BIND_EVENT_BIND_FAILED);
        g_bind.state = BIND_IDLE;
        return;
    }

    /* 4) 绑定成功 */
    log_info("[BIND] %s %s 绑定成功(uid=%s, serial=%s, name=%s), 已写 config_store + 从机表\r\n",
             g_bind.is_sensor ? "液位传感器" : "声光设备",
             uid_hex, uid_hex,
             g_bind.expected_serial[0] ? g_bind.expected_serial : "(none)",
             g_bind.expected_name[0]    ? g_bind.expected_name    : "(none)");
    _bind_fire(uid_hex, g_bind.is_sensor, LORA_BIND_EVENT_BOUND);

    g_bind.state = BIND_IDLE;
    memset(g_bind.pending_uid, 0, sizeof(g_bind.pending_uid));
}

/* 周期调用(挂在 lora_business_poll):
 *   - 窗口未到期时, 若在 WAIT_DEV_ACK 且距上次发 0x80 已过 5s → 重发 0x80
 *   - 窗口到期: 若在 WAIT_DEV_ACK → 触发 BIND_FAILED; 一律回 IDLE
 * 用 (int32_t)(now - deadline_tick) >= 0 判到期 —— tick 32 位无符号回绕时
 * "已过 deadline" 的差值是小正数, 未过时是大负数(强转 int32 为负), 这样判才对。
 */
static void _bind_logic(void)
{
    if (g_bind.state == BIND_IDLE) return;

    uint32_t now = get_tick_ms();

    /* 窗口未到期: 看是否该重发 0x80 */
    if ((int32_t)(now - g_bind.deadline_tick) < 0)
    {
        if (g_bind.state == BIND_WAIT_DEV_ACK)
        {
            if ((uint32_t)(now - g_bind.last_ack_tx_tick) >= LORA_BIND_ACK_INTERVAL_MS)
            {
                _bind_send_ack_to_pending();
            }
        }
        return;
    }

    /* 窗口到期 */
    if (g_bind.state == BIND_WAIT_DEV_ACK)
    {
        char uid_hex[CFG_UID_STR_MAX];
        /* cfg_uid_from_bytes 不会失败, 直接走失败回调 */
        (void)cfg_uid_from_bytes(g_bind.pending_uid, uid_hex);
        _bind_fire(uid_hex, g_bind.is_sensor, LORA_BIND_EVENT_BIND_FAILED);
        log_info("[BIND] 窗口到期, 未收到 %02X%02X%02X 的确认, 绑定失败\r\n",
                 g_bind.pending_uid[0], g_bind.pending_uid[1], g_bind.pending_uid[2]);
    }
    else
    {
        log_info("[BIND] 窗口到期, 期间没有设备发起 0x04, 绑定失败(无具体设备)\r\n");
        _bind_fire(NULL, g_bind.is_sensor, LORA_BIND_EVENT_BIND_FAILED);
    }

    g_bind.state = BIND_IDLE;
    memset(g_bind.pending_uid, 0, sizeof(g_bind.pending_uid));
}

/* ========== 内部: 解绑收尾(P5, LoRa 0x05) ==========
 *
 * 在两处被调用:
 *   1) handle_ack_from_device: 设备回 ACK, 正常解绑成功
 *   2) _handle_down_logic 重试耗尽: 用户口径"重试耗尽也强制删除"
 *      (设备可能已经离线/掉电, 不能因为它不回 ACK 就永远删不掉)
 *
 * 【调用方必须在调本函数前清掉自己的 s->state/pending_func】,
 * 本函数只负责 config_store + 运行期表 + 事件, 不碰 s->state(因为
 * _slave_table_del 会把末尾从机搬到当前格, s 已经不一定指向原来了)。
 *
 * 返回 true = 已删除; false = uid 解 hex 失败(运行期表仍删, 但 config_store 没动) */
static bool _unbind_finalize(const uint8_t uid[CFG_UID_BYTES], bool is_sensor)
{
    char  uid_hex[CFG_UID_STR_MAX];
    /* cfg_uid_from_bytes 把 13 字节转 hex, 不会失败(非全 0 即有效) */
    (void)cfg_uid_from_bytes(uid, uid_hex);
    (void)config_store_unbind_device(uid_hex, cloud_proto_get_now_ts());
    log_info("[UNBIND] %02X%02X%02X(%s) 已删除 config_store, 触发 UNBOUND 事件\r\n",
             uid[0], uid[1], uid[2], uid_hex);
    (void)_slave_table_del(uid);
    _bind_fire(uid_hex, is_sensor, LORA_BIND_EVENT_UNBOUND);
    return true;
}

/* ========== 内部: 下行指令处理 ========== */

/**
 * \brief 组装 lora_frame_t 并发送 (用协议层 lora_proto_send_frame)
 * \return true 成功提交
 */
/* ==================== ★2026-10-08 声光(0x03)执行结果状态机的公共定义 ====================
 * 放在 _transmit_downlink 之前: 它是发送咽喉, 每次真的发出去都要记账。
 * 状态机主体在下面 handle_ack_from_device 之前(见 _alarm_on_fail)。 */
#define LORA_ALARM_TIMEOUT_MS   15000u   /* 单条 0x03 等 0x82/0x83 的窗口(用户指定 15s) */
#define LORA_ALARM_MAX_ATTEMPTS 3u       /* 单条动作最多发几条(用户指定 3 条) */
#define LORA_ALARM_RETRY_GAP_MS 500u     /* 重发前的最小间隔(给设备射频恢复时间) */

/* 正在下发的这条动作是哪个档位。
 * 【为什么要这个全局】_transmit_downlink 只知道 slave(uid/func/data), 不知道档位 ——
 *   档位只存在于 gears[] 的索引里, 不在空口载荷里。而上报 event.action 需要它。
 *   单线程 + 一条指令从入队到发出只隔最多一个 poll 周期, 所以这里存一份是安全的。
 *   (由 lora_business_apply_gear_ex() 在入队时写。) */
static uint8_t g_alarm_gear_inflight;

/* 前置声明: 实现在 handle_ack_from_device 之前那段, 但 _handle_down_logic 先用到它们 */
static void _alarm_fire_result(const lora_slave_t *s, bool ok, bool gave_up, bool recovered);
static void _alarm_on_fail(lora_slave_t *s);

static bool _transmit_downlink(uint8_t idx)
{
    lora_slave_t *s = &g_slave_list[idx];
    lora_frame_t frame;
    bool         ok;

    memset(&frame, 0, sizeof(frame));
    memcpy(frame.gw_uid,  g_gw_uid, CFG_UID_BYTES);
    memcpy(frame.dev_uid, s->dev_uid, CFG_UID_BYTES);
    frame.func      = s->pending_func;
    frame.data_len  = s->pending_data_len;
    if (frame.data_len > 0U)
    {
        memcpy(frame.data, s->pending_data, frame.data_len);
    }
    ok = lora_proto_send_frame(&frame);

    /* ==========================================================================
     * ★★2026-10-08: 声光(0x03)每次【真的发出去】就记一次尝试, 并启动 15 秒窗口★★
     * --------------------------------------------------------------------------
     * 为什么记在这里: 这是【唯一的发送咽喉】(_handle_down_logic 的重发和
     *   _try_send_alarm_idle 的首次发送都走它), 记在这里不会漏也不会重复。
     * 为什么只在 ok 时记: TX 忙导致的发送失败不算一次尝试 —— 那条指令还在缓存里,
     *   下一拍会被 _try_send_alarm_idle 再捡起来, 不该白扣一次 3 条的配额。
     * ======================================================================== */
    if (ok && (s->pending_func == (uint8_t)LORA_FUNC_SET_ALARM))
    {
        s->alarm_attempts++;
        s->alarm_pending = true;
        s->alarm_tick    = get_tick_ms();
        s->alarm_gear    = g_alarm_gear_inflight;
        log_info("[GEAR] -> %02X%02X%02X 发出第 %u/%u 条 0x03(档位 %u), 等 0x82/0x83 %us\r\n",
                 s->dev_uid[0], s->dev_uid[1], s->dev_uid[2],
                 (unsigned)s->alarm_attempts, (unsigned)LORA_ALARM_MAX_ATTEMPTS,
                 (unsigned)s->alarm_gear, (unsigned)(LORA_ALARM_TIMEOUT_MS / 1000u));
    }
    return ok;
}

/* ========== 解绑两阶段状态机 helpers(2026-09-30 用户最终口径) ==========
 *
 * 6 个 helper 配合 g_unbind 状态机:
 *   _unbind_send_frame()        —— 用从机表里目标 slave 的 pending_func=0x05 发一帧
 *   _unbind_fire_event()        —— 把 g_unbind.uid 转 hex + 调 _bind_fire 上报事件
 *   _unbind_clear_slave_state() —— 清从机表里目标格的 state/pending_func(不删条目)
 *   _unbind_reset()             —— 清 g_unbind 回 IDLE(释放锁)
 *   _unbind_on_ack()            —— 收到设备 ack 时按 state 处理
 *   _unbind_logic()             —— 周期驱动(挂在 lora_business_poll)
 *
 * 【锁的语义】解绑期间目标 slave 一直是 WAIT_ACK+pending_func=0x05:
 *   - _handle_down_logic 看到 pending_func==0x05 直接 continue 跳过(不干扰)
 *   - handle_ack_from_device 顶部拦截 g_unbind.state!=IDLE 且 slave_idx 匹配的 ack,
 *     路由到 _unbind_on_ack, 不走旧逻辑
 *   - 结束(成功/失败/故障)都清回 IDLE+pending_func=0 释放锁
 *
 * ★ 成功路径(_unbind_finalize 调 _slave_table_del 会让 slave_idx 失效)必须
 *   先拷贝 uid/is_sensor 到本地, 再 _unbind_reset, 最后 _unbind_finalize。
 *   失败/故障路径不删从机记录, 只清 slave 状态 + _unbind_reset + 上报事件。 */

/* 用从机表里目标 slave 的 pending_func=0x05 发一帧 0x05。
 * pending_func/pending_data_len 在 lora_business_unbind 启动时已设好,
 * 这里只负责发送 + 刷新时间戳。失败(TX 忙)也更新 last_send_tick, 3s 后再试。 */
static void _unbind_send_frame(void)
{
    if (g_unbind.slave_idx >= g_slave_count) return;
    lora_slave_t *s = &g_slave_list[g_unbind.slave_idx];
    (void)_transmit_downlink(g_unbind.slave_idx);
    uint32_t now              = get_tick_ms();
    s->send_req_tick          = now;
    g_unbind.last_send_tick   = now;
}

/* 把 g_unbind.uid 转 hex + 调 _bind_fire 上报事件 */
static void _unbind_fire_event(lora_bind_event_t event)
{
    char uid_hex[CFG_UID_STR_MAX];
    /* cfg_uid_from_bytes 把 13 字节转 hex, 不会失败(非全 0 即有效) */
    (void)cfg_uid_from_bytes(g_unbind.uid, uid_hex);
    _bind_fire(uid_hex, g_unbind.is_sensor, event);
}

/* 清从机表里目标 slave 的 state/pending_func —— 不删条目(失败/故障路径用,
 * 从机记录保留); 成功路径走 _unbind_finalize 的 _slave_table_del 删条目。
 * 校验 uid 匹配: 防止 _slave_table_del/add 改了布局后索引错位误清别的从机。 */
static void _unbind_clear_slave_state(void)
{
    if (g_unbind.slave_idx < g_slave_count)
    {
        lora_slave_t *s = &g_slave_list[g_unbind.slave_idx];
        if (memcmp(s->dev_uid, g_unbind.uid, CFG_UID_BYTES) == 0)
        {
            s->state            = SLAVE_STATE_IDLE;
            s->retry_cnt        = 0U;
            s->pending_func     = 0U;
            s->pending_data_len = 0U;
        }
    }
}

/* 清 g_unbind 回 IDLE(释放锁) */
static void _unbind_reset(void)
{
    g_unbind.state        = UNBIND_IDLE;
    g_unbind.slave_idx    = 0xFFU;
    g_unbind.phase2_retry = 0U;
    memset(g_unbind.uid, 0, sizeof(g_unbind.uid));
}

/* 收到设备 ack(0x80) 时由 handle_ack_from_device 路由进来。
 * - PHASE1: 立即停发(本函数本身就是收到 ack 的瞬间, 不会再发) → 等 15s
 * - PHASE2: phase2_retry++; >=3 → 设备故障(上报 UNBIND_FAULT, 从机记录保留);
 *           否则循环阶段2 的 10s(不再等 15s): 重置 phase_start_tick + 立即发一帧 */
static void _unbind_on_ack(void)
{
    uint32_t now = get_tick_ms();

    if (g_unbind.state == UNBIND_PHASE1)
    {
        log_info("[UNBIND] 阶段1 收到 %02X%02X%02X 的 ack, 立即停发, 等 15s 让设备清网关值\r\n",
                 g_unbind.uid[0], g_unbind.uid[1], g_unbind.uid[2]);
        g_unbind.state            = UNBIND_WAIT_CLEAR;
        g_unbind.phase_start_tick = now;
    }
    else if (g_unbind.state == UNBIND_PHASE2)
    {
        g_unbind.phase2_retry++;
        if (g_unbind.phase2_retry >= LORA_UNBIND_PHASE2_MAX_RETRY)
        {
            /* 3 次都收到 ack = 设备没清网关值 = 设备故障 */
            log_info("[UNBIND] !! 阶段2 第 %u 次收到 ack(>= %u), 设备故障, "
                     "上报 UNBIND_FAULT(从机记录保留)\r\n",
                     (unsigned)g_unbind.phase2_retry,
                     (unsigned)LORA_UNBIND_PHASE2_MAX_RETRY);
            _unbind_fire_event(LORA_BIND_EVENT_UNBIND_FAULT);
            _unbind_clear_slave_state();
            _unbind_reset();
        }
        else
        {
            /* 设备没清 → 循环阶段2 的 10s(不再等 15s): 重置阶段计时 + 立即发一帧 */
            log_info("[UNBIND] 阶段2 第 %u 次收到 ack(< %u), 设备没清, "
                     "循环阶段2(不再等 15s)\r\n",
                     (unsigned)g_unbind.phase2_retry,
                     (unsigned)LORA_UNBIND_PHASE2_MAX_RETRY);
            g_unbind.phase_start_tick = now;
            g_unbind.last_send_tick   = now;
            _unbind_send_frame();
        }
    }
    /* WAIT_CLEAR / IDLE 状态收到 ack 不可能(没发帧), 忽略 */
}

/* 周期驱动(挂在 lora_business_poll, 每轮调):
 * - PHASE1: 30s 超时未收 ack → 失败上报(从机记录保留); 3s 间隔发 0x05
 * - WAIT_CLEAR: 15s 到 → 进 PHASE2(立即发)
 * - PHASE2: 10s 超时未收 ack → 成功(删表 + 上报 UNBOUND); 3s 间隔发 0x05
 *
 * ★ tick 比较一律用 (uint32_t)(now - start) >= DURATION 无符号差值(自动处理回绕) */
static void _unbind_logic(void)
{
    if (g_unbind.state == UNBIND_IDLE) return;

    uint32_t now = get_tick_ms();

    if (g_unbind.state == UNBIND_PHASE1)
    {
        /* 30s 没收到 ack → 失败上报(从机记录保留) */
        if ((uint32_t)(now - g_unbind.phase_start_tick) >= LORA_UNBIND_PHASE1_MS)
        {
            log_info("[UNBIND] !! 阶段1 %us 没收到 ack, 上报 UNBIND_FAILED(从机记录保留)\r\n",
                     (unsigned)(LORA_UNBIND_PHASE1_MS / 1000U));
            _unbind_fire_event(LORA_BIND_EVENT_UNBIND_FAILED);
            _unbind_clear_slave_state();
            _unbind_reset();
            return;
        }
        /* 3s 间隔发 0x05(其余时间接收) */
        if ((uint32_t)(now - g_unbind.last_send_tick) >= LORA_UNBIND_PHASE1_INTERVAL_MS)
        {
            _unbind_send_frame();
        }
    }
    else if (g_unbind.state == UNBIND_WAIT_CLEAR)
    {
        /* 等 15s 给设备清网关值, 到了进阶段2 并立即发一帧 */
        if ((uint32_t)(now - g_unbind.phase_start_tick) >= LORA_UNBIND_WAIT_CLEAR_MS)
        {
            log_info("[UNBIND] 等 15s 完成, 进入阶段2(10s, 最多循环 %u 次)\r\n",
                     (unsigned)LORA_UNBIND_PHASE2_MAX_RETRY);
            g_unbind.state            = UNBIND_PHASE2;
            g_unbind.phase_start_tick = now;
            g_unbind.last_send_tick   = now;
            _unbind_send_frame();
        }
    }
    else if (g_unbind.state == UNBIND_PHASE2)
    {
        /* 10s 没收到 ack = 设备已清网关值 = 解绑成功 */
        if ((uint32_t)(now - g_unbind.phase_start_tick) >= LORA_UNBIND_PHASE2_MS)
        {
            /* 成功路径: 先拷 uid/is_sensor, 再 _unbind_reset(清 g_unbind 释放锁),
             * 最后 _unbind_finalize —— 它会 _slave_table_del 让 slave_idx 失效,
             * 所以必须先 reset 释放索引引用, 再用 uid_copy 找条目删 */
            uint8_t uid_copy[CFG_UID_BYTES];
            bool    is_sensor = g_unbind.is_sensor;
            memcpy(uid_copy, g_unbind.uid, CFG_UID_BYTES);
            log_info("[UNBIND] 阶段2 %us 没收到 ack, 设备已清网关值, 解绑成功\r\n",
                     (unsigned)(LORA_UNBIND_PHASE2_MS / 1000U));
            _unbind_reset();
            (void)_unbind_finalize(uid_copy, is_sensor);
            return;
        }
        /* 3s 间隔发 0x05(其余时间接收) */
        if ((uint32_t)(now - g_unbind.last_send_tick) >= LORA_UNBIND_PHASE2_INTERVAL_MS)
        {
            _unbind_send_frame();
        }
    }
}

/* _try_send_downlink() 的结果, 让调用方能把日志说清楚 */
typedef enum
{
    DOWNLINK_SENT        =  1,   /* 真的发出去了 */
    DOWNLINK_BUSY        =  0,   /* 该从机正在等 ACK, 这轮不发 */
    DOWNLINK_NO_CACHE    = -1,   /* 下发缓存里没有这条设备的指令 */
    DOWNLINK_TX_FAIL     = -2,   /* 组帧/射频发送失败 */
    DOWNLINK_BAD_PAYLOAD = -3,   /* 缓存项格式不对(连 func 都没有) */
} downlink_result_t;

/**
 * \brief 收到上行帧后: 检查该设备是否有待下发指令, IDLE 状态则发送
 * \return downlink_result_t
 */
static int _try_send_downlink(uint8_t idx)
{
    lora_slave_t *s = &g_slave_list[idx];
    if (s->state != SLAVE_STATE_IDLE) return DOWNLINK_BUSY;

    cache_down_item_t item;
    if (!data_cache_down_fetch_by_uid(s->dev_uid, &item)) return DOWNLINK_NO_CACHE;

    /* payload 打包约定: payload[0] = func, payload[1..] = data
     * (lora_business_send_req 在存入 data_cache 时就这么拼好了) */
    if (item.pay_len < 1U) return DOWNLINK_BAD_PAYLOAD;

    s->pending_func     = item.payload[0];
    s->pending_data_len = (uint8_t)(item.pay_len - 1U);
    if (s->pending_data_len > 0U)
    {
        memcpy(s->pending_data, &item.payload[1], s->pending_data_len);
    }

    /* 【0x01 的校时必须在"要发的这一刻"重新向 4G 取】—— 现场协议要求:
     *   时间是【网关 lora 发送数据时】向 4G 模块取来组帧的。
     *
     *   为什么不能沿用"收到云端下行时"取的那个时间戳:
     *   液位是休眠设备, 上报间隔最长可配到 65535 秒(≈18.2 小时) —— 云端完全可能在
     *   它上报前很久就把参数发下来了。用那时的时间戳会把设备 RTC 校偏那么久。
     *
     *   取不到怎么办: 【沿用缓存里的旧值】并告警, 而不是把这条指令丢掉 ——
     *   时间戳略旧也好过"参数永远下不去"。
     *
     *   载荷布局(与设备端 app_cmd.h 一致, 共 13 字节, 16 位字段低字节在前):
     *     [0-1] 检测间隔 [2-3] 上报间隔 [4] 相同液位上报表 [5-6] 阈值
     *     [7]年 [8]月 [9]日 [10]周 [11]时 [12]分          <- 这 6 字节是网关补的 */
    if ((s->pending_func == LORA_FUNC_SET_SENSOR_PARAM) && (s->pending_data_len >= 13U))
    {
        g4_clock_t clk;

        if (g4_at_get_clock(&clk))
        {
            s->pending_data[7]  = clk.year;
            s->pending_data[8]  = clk.month;
            s->pending_data[9]  = clk.day;
            s->pending_data[10] = clk.week;     /* 4G 应答里没有星期, 固定周五(5) */
            s->pending_data[11] = clk.hour;
            s->pending_data[12] = clk.minute;

            log_info("[GW] 0x01 校时按【发送时刻】重取: 20%02u-%02u-%02u w%u %02u:%02u\r\n",
                     (unsigned)clk.year, (unsigned)clk.month, (unsigned)clk.day,
                     (unsigned)clk.week, (unsigned)clk.hour, (unsigned)clk.minute);
        }
        else
        {
            log_info("[GW] !! 发送前取不到 4G 时间(AT+CCLK?), 沿用缓存里的旧校时 "
                     "20%02u-%02u-%02u 继续下发\r\n",
                     (unsigned)s->pending_data[7], (unsigned)s->pending_data[8],
                     (unsigned)s->pending_data[9]);
        }
    }

    /* 打印下发帧内容(调试用) */
    log_info("[LoRA TX] dev=%02X%02X%02X func=0x%02X data_len=%u data=",
             s->dev_uid[0], s->dev_uid[1], s->dev_uid[2],
             s->pending_func, s->pending_data_len);
    for (uint8_t di = 0; di < s->pending_data_len && di < 32; di++) {
        log_info("%02X ", s->pending_data[di]);
    }
    log_info("\r\n");

    if (!_transmit_downlink(idx)) return DOWNLINK_TX_FAIL;

    s->state       = SLAVE_STATE_WAIT_ACK;
    s->retry_cnt   = 0U;
    s->send_req_tick = get_tick_ms();

    return DOWNLINK_SENT;
}

/**
 * \brief 下行 ACK 超时重传检测 (轮询调用)
 *
 * 【重传耗尽 = 丢弃这条指令】—— 这是 2026-09 修掉的一个严重逻辑错误。
 *
 * 老实现耗尽后只把状态改回 IDLE、缓存【留着】, 注释写的是"等设备下次上报再试"。
 * 但声光报警器是【常驻 RX、且从不主动上报】的设备:
 *   _try_send_alarm_idle() 每秒扫一遍 IDLE 的报警器, 一看缓存里还有这条指令
 *   → 又发一轮(3 次重传) → 又耗尽 → 状态回 IDLE → 下一秒再发 …
 * 结果就是【网关永远在定时重发同一条 0x03】:
 *   1) 空口被这条帧刷满, 液位传感器的上报被挤掉 / 收到也来不及应答;
 *   2) 每次发送都要阻塞 ~500ms(LORA_TX_SETTLE_MS + 等 TX_DONE), 主循环被反复
 *      按停, 这期间 g4_at_poll() 不跑, 云端下行堆在串口缓冲里溢出丢失;
 *   3) 报警器那边收到的是同一条指令的无限副本。
 *
 * 正式业务逻辑: 3 次重传都没 ACK = 这次下发失败, 丢掉缓存并明确打日志;
 * 需要再试由云端重新下发(或等液位下次上报搭车 —— 液位缓存也一起丢,
 * 保持"两类设备语义一致", 免得又埋一个"某些设备会偷偷无限重试"的坑)。
 */
static void _handle_down_logic(void)
{
    uint32_t now = get_tick_ms();
    for (uint8_t i = 0U; i < g_slave_count; i++)
    {
        lora_slave_t *s = &g_slave_list[i];
        if (s->state != SLAVE_STATE_WAIT_ACK) continue;

        /* 【解绑(0x05)的 slave 由 g_unbind 状态机独立驱动, 不走这里的重传逻辑】
         *   g_unbind 启动时把目标 slave 置 WAIT_ACK+pending_func=0x05 当"锁",
         *   防止 _try_send_downlink / 本函数干扰; 这里直接 continue 跳过,
         *   发送/重发/超时/阶段切换全由 _unbind_logic() + _unbind_on_ack() 处理。
         *   (2026-09-30 改造: 原来走的 5s/3 次耗尽强制删路径与用户最终两阶段口径
         *   冲突, 已作废; 详见 g_unbind 上方注释) */
        if (s->pending_func == LORA_FUNC_UNBIND) continue;

        /* ==========================================================================
         * ★★2026-10-08: 声光(0x03)走【业务级】的 15 秒 / 3 条逻辑, 不走下面的帧级重传★★
         * --------------------------------------------------------------------------
         * 为什么必须分开:
         *   · 帧级重传(下面那套)语义是"发出去的帧没被解出来 ⇒ 重发同一串字节";
         *   · 声光的 0x80 只表示"收到了", 真正的结论在 0x82/0x83 ——
         *     所以"没结果"要重发的是【整条业务动作】, 而 0x80 到了也不能算成功。
         *   混在一起会出现: 设备回了 0x80 但 10 秒后才回 0x82, 中间被帧级重传
         *   又发了一遍 ⇒ 平台收到两条 event.action、设备执行两次。
         * ======================================================================== */
        if (s->pending_func == (uint8_t)LORA_FUNC_SET_ALARM)
        {
            if (!s->alarm_pending) continue;                /* 不在等结果: 还没发/已收尾 */
            if ((now - s->alarm_tick) < LORA_ALARM_TIMEOUT_MS) continue;

            log_info("[GEAR] !! %02X%02X%02X 等了 %us 没收到 0x82/0x83 -> 本条算失败"
                     "(已发 %u/%u)\r\n",
                     s->dev_uid[0], s->dev_uid[1], s->dev_uid[2],
                     (unsigned)(LORA_ALARM_TIMEOUT_MS / 1000u),
                     (unsigned)s->alarm_attempts, (unsigned)LORA_ALARM_MAX_ATTEMPTS);
            _alarm_on_fail(s);
            continue;
        }

        if ((now - s->send_req_tick) < LORA_ACK_TIMEOUT_MS) continue;

        s->retry_cnt++;
        if (s->retry_cnt > LORA_MAX_RETRY)
        {
            /* 重传耗尽: 丢弃下发缓存, 回到 IDLE(不会再被 _try_send_alarm_idle 捡起来重发) */
            s->state     = SLAVE_STATE_IDLE;
            s->retry_cnt = 0U;
            data_cache_down_del_by_uid(s->dev_uid);

            log_info("[GW] !! %02X%02X%02X 连发 %u 次没等到 ACK, 丢弃这条下发指令"
                     "(要重发请从云端再下发)\r\n",
                     s->dev_uid[0], s->dev_uid[1], s->dev_uid[2],
                     (unsigned)(LORA_MAX_RETRY + 1U));
        }
        else
        {
            _transmit_downlink(i);
            s->send_req_tick = now;
        }
    }
}

/**
 * \brief 检查是否有任意从机正在等待 ACK (无线电忙)
 *        有从机在 WAIT_ACK 时不能抢发, 否则 TX 会打断接收 ACK
 */
static bool _any_slave_waiting_ack(void)
{
    for (uint8_t i = 0U; i < g_slave_count; i++)
    {
        if (g_slave_list[i].state == SLAVE_STATE_WAIT_ACK)
            return true;
    }
    return false;
}

/**
 * \brief 扫描所有 IDLE 的声光报警器从机, 尝试发送缓存中的待下发指令
 *        声光报警器常驻 RX, 可以随时主动发; 液位传感器休眠, 只能等上报搭车
 *        无线电忙(有从机在等ACK)时跳过, 避免打断接收 ACK
 *
 * \note 这里能捡起来的只可能是【云端刚下发、还没发出去过】的那一条:
 *       一旦发出去进了 WAIT_ACK, 要么收到 ACK 清缓存, 要么重传耗尽被
 *       _handle_down_logic() 删缓存 —— 两种结局都不会让它再回到这里。
 *       (老版本耗尽后不删缓存, 于是这条函数每秒给它重发一轮, 永不停歇。)
 */
static void _try_send_alarm_idle(void)
{
    if (_any_slave_waiting_ack())
        return;

    for (uint8_t i = 0U; i < g_slave_count; i++)
    {
        if (g_slave_list[i].state == SLAVE_STATE_IDLE &&
            g_slave_list[i].dev_type == SLAVE_TYPE_ALARM_OUT)
        {
            _try_send_downlink(i);
            /* 发了一台就停, 该从机进入 WAIT_ACK, 下次周期再发下一台 */
            break;
        }
    }
}
/* 前置声明: 实现在 handle_ack_from_device 之前那段注释下面 */
static void _schedule_default_level_downlink(const uint8_t dev_uid[CFG_UID_BYTES]);

/* ==================== 液位失联检测(2026-09-27 用户补充要求第 3 条) ====================
 *
 * 用户口径: 传感器【最多三个】; 不用断言它还在, 只记录【上次上报时间】;
 *           每次收到一路液位上报, 就把所有液位从机的上次上报时间拿出来比,
 *           超过【3 倍上报周期】没上报的, 就向云端发"传感器失联"。
 *
 * 【判定/边界/回绕/去重的算法全在 app/service/sensor_offline.c】, 那是一个
 * 不依赖硬件的纯模块(主机单测直接覆盖); 这里只有状态与胶水。
 *
 * 三处口径说明(代码里也标了, 免得以后有人"顺手改一下"改错):
 *   1) 时间戳一律 get_tick_ms(), 比较必须走 sensor_offline 里的无符号差值 ——
 *      tick 会回绕, 写 now > last + 阈值 会在回绕那一下判错;
 *   2) 上报周期取自 config_store 里该 UID 的 reportPeriodSeconds; 查不到 / 没配 (0)
 *      用 SENSOR_OFFLINE_DEFAULT_PERIOD_S(60s)兜底, 阈值 = 180s;
 *   3) 只报【跳变】: 在线->失联报一次; 恢复靠它自己的下一次 0x02 上报, 报一次"恢复"。
 *      失联后不管来多少帧别人的上报都不会重发(offline_reported 挡住)。
 */

/* 失联/恢复事件的上报回调 —— 真正组事件 JSON + 发 MQTT 的实现在 src/main.c
 * (本模块不依赖云协议层, 见 lora_business.h 的说明)。不注册只打日志。 */
static lora_sensor_event_fn g_sensor_event_cb;

void lora_business_set_sensor_event_cb(lora_sensor_event_fn fn)
{
    g_sensor_event_cb = fn;
}

/* ★2026-10-18: 每次液位上报(0x02)的原始数据回调 —— 供主程序做"离线补报" */
static lora_level_report_fn g_level_report_cb;

void lora_business_set_level_report_cb(lora_level_report_fn fn)
{
    g_level_report_cb = fn;
}

/* 把一次跳变交给上层; 没注册回调时如实打日志(弱网/未挂载时现场要能看出来) */
static void _sensor_event_fire(const lora_slave_t *s, bool online)
{
    char uid[CFG_UID_STR_MAX];

    cfg_uid_from_bytes(s->dev_uid, uid);

    if (g_sensor_event_cb != NULL)
    {
        g_sensor_event_cb(uid, online);
    }
    else
    {
        log_info("[GW] !! 液位 %s %s, 但主程序没注册上报回调 -> 这条事件只记在本机\r\n",
                 uid, online ? "恢复上报" : "失联");
    }
}

void lora_business_check_sensor_offline(void)
{
    uint32_t              now_ms = get_tick_ms();
    const config_store_t *cfg    = config_store_get();
    uint8_t               i;

    for (i = 0U; i < g_slave_count; i++)
    {
        lora_slave_t *s = &g_slave_list[i];
        char          uid[CFG_UID_STR_MAX];
        uint16_t      period_s;
        sensor_offline_action_t act;

        if (s->dev_type != SLAVE_TYPE_SENSOR) { continue; }

        cfg_uid_from_bytes(s->dev_uid, uid);
        period_s = sensor_offline_period_for_uid(cfg, uid);

        /* got_report = false: 本帧不是它的上报, 只是"顺带检查"。
         * 它自己上报那条路径在 handle_sensor_report() 里(got_report = true)。 */
        act = sensor_offline_update(&s->is_online, &s->offline_reported,
                                    now_ms, s->last_report_tick, period_s, false);

        if (act == SENSOR_OFFLINE_ACT_REPORT_OFFLINE)
        {
            log_info("[GW] 液位 %s 失联: 已 %lums 没有 0x02 上报"
                     "(阈值 %lums = 上报周期 %us × %u)\r\n",
                     uid, (unsigned long)(uint32_t)(now_ms - s->last_report_tick),
                     (unsigned long)sensor_offline_threshold_ms(period_s),
                     (unsigned)period_s, (unsigned)SENSOR_OFFLINE_PERIOD_MULT);
            _sensor_event_fire(s, false);
        }
    }
}

/* ==================================================================
 * ★2026-10-18: 设备在线/离线状态查询(给 sen.query / dev.query 的 online 用)
 * ------------------------------------------------------------------
 * 【为什么不能继续用 last_value 快照】那张表只有液位 0x02 会写, 声光从不进表
 *   ⇒ 声光的 online 恒 false, 平台会一直以为它掉了。
 * 【为什么声光不用 last_recv_tick】声光平时不收帧(只有下发 0x03 时才回 0x80/
 *   0x82/0x83), 用"多久没收到帧"判它失联, 会在没下发动作时【一直误判离线】。
 *   所以对它只认一条证据: 3 条 0x03 都没成功(alarm_offline_reported)。
 * ================================================================== */
lora_dev_state_t lora_business_device_state_bytes(const uint8_t uid[LORA_UID_BYTES])
{
    uint8_t i;

    if (uid == NULL)
    {
        return LORA_DEV_STATE_UNKNOWN;
    }

    for (i = 0u; i < g_slave_count; i++)
    {
        const lora_slave_t *s = &g_slave_list[i];

        if (memcmp(s->dev_uid, uid, LORA_UID_BYTES) != 0) { continue; }

        if (s->dev_type == SLAVE_TYPE_SENSOR)
        {
            return s->is_online ? LORA_DEV_STATE_ONLINE : LORA_DEV_STATE_OFFLINE;
        }

        /* 声光: 只有"3 条 0x03 都没成功"这一条确凿证据 */
        return s->alarm_offline_reported ? LORA_DEV_STATE_OFFLINE
                                         : LORA_DEV_STATE_ONLINE;
    }

    return LORA_DEV_STATE_UNKNOWN;      /* 不在从机表里 */
}

lora_dev_state_t lora_business_device_state(const char *uid_hex)
{
    uint8_t uid[CFG_UID_BYTES];

    if ((uid_hex == NULL) || (!cfg_uid_to_bytes(uid_hex, uid)))
    {
        return LORA_DEV_STATE_UNKNOWN;
    }

    return lora_business_device_state_bytes(uid);
}


/* ---------- 0x02 传感器上报: 拆包 + 上行缓存 + 4G 上报 ---------- */
static void handle_sensor_report(const lora_frame_t *frame, uint8_t slave_idx,
                                 int8_t rssi_dbm)
{
    /* 载荷 7 字节, 与设备端 app_sensor.c 的 app_sensor_build_level_report() 一一对应:
     *   [0..1] level_mm   大端 (设备端写的是 (level>>8), (level&0xFF))
     *   [2]    bat_pct    电量百分比 %
     *   [3]    month      [4] day      [5] hour      [6] minute   ← 北京时间
     */
    if (frame->data_len < 7U) return;

    /* ---- 失联检测: 记下"上次上报时间", 并判断这是不是一次"恢复上报" ----
     * 【为什么放在长度校验之后】: 长度不对的残帧不算一次有效上报, 否则一台
     * 一直在发坏帧的设备会被判成"还活着"。
     * 【为什么不用 last_recv_tick】: 那个是"收到任何帧"都刷新的, 用它判失联会把
     * "只回 ACK 不上报"的设备永远算成在线(见 lora_business.h 的字段说明)。 */
    {
        lora_slave_t *s      = &g_slave_list[slave_idx];
        uint32_t      now_ms = get_tick_ms();
        sensor_offline_action_t act;

        s->last_report_tick = now_ms;

        /* got_report = true: 本帧就是这台设备的 0x02 上报。
         * 这里不需要周期(阈值只用于"没上报"的那一侧), 传 0 不影响结果。 */
        act = sensor_offline_update(&s->is_online, &s->offline_reported,
                                    now_ms, s->last_report_tick, 0u, true);
        if (act == SENSOR_OFFLINE_ACT_REPORT_ONLINE)
        {
            char uid[CFG_UID_STR_MAX];

            cfg_uid_from_bytes(s->dev_uid, uid);
            log_info("[GW] 液位 %s 恢复上报(之前已判失联, 发一条恢复事件)\r\n", uid);
            _sensor_event_fire(s, true);
        }
    }

    const uint8_t *d = frame->data;
    uint16_t level_mm  = ((uint16_t)d[0] << 8) | (uint16_t)d[1];   /* 大端 */
    uint16_t bat_pct   = d[2];        /* 注意: 旧 local_gateway 里叫 bat_mv(电压), 协议里实际是电量% */

    /* 【时间不用液位报的, 用 4G 的】—— 现场要求
     *   液位自己的 RTC 常年可能是没校时的(现场日志里它一直打 "NOT-SYNCED",
     *   上报的 4 个时间字节全是 00), 所以【不能相信它的时间】。
     *   规则: 【收到液位上报之后, 向 4G 模块取时间】来组这一帧。
     *   这里只要 月/日/时/分 4 个字节(协议里网关上报的载荷布局)。
     *   取不到(模组没在网/忙)时退回用液位报的那个, 但把告警打出来 ——
     *   宁可时间可能不准, 也不该把整条液位记录丢掉。 */
    uint8_t    mo, dd, hh, mi;
    g4_clock_t clk;

    if (g4_at_get_clock(&clk))
    {
        mo = clk.month; dd = clk.day; hh = clk.hour; mi = clk.minute;
    }
    else
    {
        mo = d[3]; dd = d[4]; hh = d[5]; mi = d[6];
        log_info("[LoRa UP] !! 取不到 4G 时间(AT+CCLK?) -> 这一条退回用液位报的时间\r\n");
    }

    /* 打包成 0xMMDDHHMM: cache_up_item_t.time_mdhhm 与
     * lora_business_publish_report() 都按 字节0=月 字节1=日 字节2=时 字节3=分 解读。 */
    uint32_t time_mdhhm = ((uint32_t)mo << 24) | ((uint32_t)dd << 16)
                        | ((uint32_t)hh << 8)  |  (uint32_t)mi;

    log_info("[LoRa UP] dev=%02X%02X%02X level=%umm bat=%u%% time=%02u-%02u %02u:%02u(4G)"
             " | 液位自报 %02u-%02u %02u:%02u(不用)\r\n",
             frame->dev_uid[0], frame->dev_uid[1], frame->dev_uid[2],
             (unsigned)level_mm, (unsigned)bat_pct,
             (unsigned)mo, (unsigned)dd, (unsigned)hh, (unsigned)mi,
             (unsigned)d[3], (unsigned)d[4], (unsigned)d[5], (unsigned)d[6]);

    /* 1. 存入上行缓存 —— 【上行只有这一个出口】:
     *    lora_business_poll() 里的 data_cache_poll() 会在 4G 在线时取出、
     *    组帧、发 MQTT, 发成功了才出队。
     *
     *    这里【故意不再】"立即发一遍": 以前既立即发又让 data_cache_poll 发,
     *    同一份数据发两次; 而且两次都去抢模组的 g_pub_pending, 抢输的那次
     *    直接 SYS_ERR(记录已被 pop) —— 现象就是"液位偶尔少一条"。 */
    data_cache_up_push(frame->dev_uid, level_mm, bat_pct, time_mdhhm, rssi_dbm);

    /* ★2026-10-18: 通知主程序"又收到一条液位上报" —— 它会在 4G 离线时把这条
     *   存进 telemetry_cache, 重连后按【此刻】的时间戳逐条补报。
     *   与上一行的分工: 上行 data_cache_up_push 是 last_value 快照的漏斗(在线
     *   聚合遥测要用), 这一条是"离线补报"的漏斗, 两者互不影响。
     *   放在长度校验之后 —— 残帧不算一次有效上报(否则补报里会混进垃圾值)。 */
    if (g_level_report_cb != NULL)
    {
        g_level_report_cb(frame->dev_uid, level_mm, (uint8_t)bat_pct, rssi_dbm);
    }

    /* 2. 液位告警检测 (暂留空, 未对接 local_alarm)
     *   if (level_mm > threshold) lora_business_send_req(slave->dev_uid, SLAVE_TYPE_ALARM_OUT,
     *                                                     LORA_FUNC_SET_ALARM, alarm_payload, len); */

    /* 3. 给这台液位传感器补一条"本地默认参数"进下发缓存
     *    见下面 LORA_LEVEL_PUSH_DEFAULT 那段说明。 */
    _schedule_default_level_downlink(frame->dev_uid);

    /* 4. 用户口径: 【每收到一路液位上报, 就顺带检查全部液位从机】是否失联。
     *    刚才这台已经在上面的 sensor_offline_update() 里刷过时间戳, 不会被误判;
     *    这里检查的是"别人"。不另起定时器。
     *    本函数在主循环路径上被调用(lora_proto_poll → on_frame), 不在中断上下文。 */
    lora_business_check_sensor_offline();

    (void)slave_idx;
}

/* ---------- 液位传感器: 上报后补一条默认参数进下发缓存 ----------
 *
 * 【2026-09 云端改成下发协议帧后, 这个默认值仍然默认【关闭】】
 *   正常路径是云端下发 0x01 帧, 载荷里的校时取自 AT+CCLK?(模组在网才有)。
 *   4G 不在网时按约定【不下发 0x01】, 而不是拿一个写死的时间去骗设备。
 *   这里保留开关只是给"纯 LoRa 台架、完全没有 4G"时应急用:
 *   打开后会在设备上报且下发缓存为空时补一条固定参数(含写死的时间)。
 *
 * 【什么时候补】只在该设备下发缓存为空时补(非破坏性 fetch 查)。
 * 云端刚下发过参数就按云端的走, 不覆盖。
 *
 * ⚠️ 打开后设备 RTC 每轮上报都会被校到下面那个固定时间。
 *
 * 载荷布局(13 字节, 小端, 与设备端 app_cmd.h / app_cmd.c 完全一致):
 *   off0-1 检测间隔 1200 秒 = 20 分钟 -> 0x04B0
 *   off2-3 上报间隔 1800 秒 = 30 分钟 -> 0x0708
 *   （★单位是【秒】, 网关不换算, 设备端也按秒用 —— 见 LORA_SENSOR_PERIOD_MIN_S ★）
 *   off4   相同液位上报 1
 *   off5-6 液位告警阈值 500mm -> 0x01F4
 *   off7 year off8 month off9 day off10 week off11 hour off12 minute
 *   ⚠️ 16 位字段必须【低字节在前】, 写成大端会被设备解成 512/256/62465 这种乱值。
 */
#ifndef LORA_LEVEL_PUSH_DEFAULT
#define LORA_LEVEL_PUSH_DEFAULT   0     /* 1 = 台架无 4G 时补固定参数 */
#endif

/* 单位【秒】(2026-10-05 改版, 与上面的 LORA_SENSOR_PERIOD_MIN_S 同一口径):
 *   检测间隔 1200s = 20 分钟, 上报间隔 1800s = 30 分钟, 阈值 500mm。 */
static const uint8_t LORA_LEVEL_DEFAULT_DATA[13] =
{
    0xB0U, 0x04U,        /* off0-1  检测间隔 1200 秒 = 20 分钟 (0x04B0 小端) */
    0x08U, 0x07U,        /* off2-3  上报间隔 1800 秒 = 30 分钟 (0x0708 小端) */
    0x01U,               /* off4    相同液位也上报 */
    0xF4U, 0x01U,        /* off5-6  告警阈值 500mm (0x01F4 小端) */
    26U, 9U, 18U, 5U, 13U, 25U   /* off7-12 校时: 2026-09-18 周五 13:25 */
};

static void _schedule_default_level_downlink(const uint8_t dev_uid[CFG_UID_BYTES])
{
#if LORA_LEVEL_PUSH_DEFAULT
    cache_down_item_t probe;

    /* 已经有待发指令(云端刚下发的)就不动它 */
    if (data_cache_down_fetch_by_uid((uint8_t *)dev_uid, &probe))
    {
        return;
    }

    if (lora_business_send_req(dev_uid, SLAVE_TYPE_SENSOR, LORA_FUNC_SET_SENSOR_PARAM,
                               LORA_LEVEL_DEFAULT_DATA,
                               (uint8_t)sizeof(LORA_LEVEL_DEFAULT_DATA)) == 0)
    {
        log_info("[GW] 该设备下发缓存为空, 已补一条默认 0x01 参数(1200s/1800s/500mm), "
                 "稍后随应答一起下发\r\n");
    }
    else
    {
        log_info("[GW] !! 默认参数补入下发缓存失败(缓存满/载荷过长)\r\n");
    }
#else
    (void)dev_uid;
#endif
}

/* ---------- 0x80 / 0x81 从机 ACK 应答: 更新状态 + 清除下发缓存 ---------- */
/* ==========================================================================
 * ★★2026-10-08 新增: 声光(0x03)执行结果的上报与重试(用户 A1/A3)★★
 * --------------------------------------------------------------------------
 * 用户口径:
 *   · 发出去后 15 秒内要收到 0x82(成功) 或 0x83(失败);
 *   · 超时算失败, 0x83 也算失败, 【都要计数并重发】;
 *   · 【总共发 3 条】(首次 + 2 次重试);
 *   · 3 条都没成功 ⇒ 上报"声光报警器离线"。
 * ======================================================================== */

/* 声光的超时/次数/间隔宏定义在文件前半(见 _transmit_downlink 之前),
 * 因为发送咽喉也要用它们 —— 这里不再重复定义。 */

static lora_alarm_result_fn g_alarm_result_cb = NULL;

void lora_business_set_alarm_result_cb(lora_alarm_result_fn fn)
{
    g_alarm_result_cb = fn;
}

/** 拿到 uid 的 hex 串并回调主程序(没注册就只打日志) */
static void _alarm_fire_result(const lora_slave_t *s, bool ok, bool gave_up,
                               bool recovered)
{
    char uid_hex[CFG_UID_STR_MAX];

    (void)cfg_uid_from_bytes(s->dev_uid, uid_hex);

    log_info("[GEAR] 上报执行结果 uid=%s gear=%u ok=%u gave_up=%u recovered=%u\r\n",
             uid_hex, (unsigned)s->alarm_gear, ok ? 1u : 0u, gave_up ? 1u : 0u,
             recovered ? 1u : 0u);

    if (g_alarm_result_cb != NULL)
    {
        g_alarm_result_cb(uid_hex, s->alarm_gear, ok, gave_up, recovered);
    }
}

/**
 * @brief 这一条 0x03 失败了(收到 0x83 或 15s 超时): 还有配额就重发, 用完就收尾
 *
 * 【为什么重发是"重新走一遍下发"而不是"重发同一串字节"】
 *   0x82/0x83 是【设备执行完】的结论, 重发同一帧是没意义的(设备已经执行过了);
 *   真正要做的是把这条业务动作【再试一次】。所以这里回到 IDLE + 保留下发缓存,
 *   由 _try_send_alarm_idle() 捡起来重发 —— 与既有的重发路径完全一致。
 *
 * 【为什么清 alarm_pending 但保留 attempts】
 *   attempts 是"这一轮动作已经试了几条", 跨重发保留; alarm_pending 表示
 *   "正在等结果", 清掉后 _handle_down_logic 就不会再对它判超时。
 */
static void _alarm_on_fail(lora_slave_t *s)
{
    s->alarm_pending = false;
    s->state         = SLAVE_STATE_IDLE;    /* 让 _try_send_alarm_idle 能再捡起来 */
    s->retry_cnt     = 0U;                  /* 帧级重传计数与业务重试无关, 一并归零 */

    if (s->alarm_attempts >= LORA_ALARM_MAX_ATTEMPTS)
    {
        /* ★重试用尽 ⇒ 收尾并报"设备离线"★ 缓存也要丢掉, 否则会被无限重发 */
        log_info("[GEAR] !! %02X%02X%02X 连续 %u 条都没执行成功 -> 判【声光报警器离线】\r\n",
                 s->dev_uid[0], s->dev_uid[1], s->dev_uid[2],
                 (unsigned)s->alarm_attempts);
        data_cache_down_del_by_uid(s->dev_uid);
        /* ★2026-10-18 (丙): 记下"已报离线", 等它下次 0x82 成功时补报 recovered ★ */
        s->alarm_offline_reported = true;
        _alarm_fire_result(s, false, true, false);  /* ok=false, gave_up=true */
        s->alarm_attempts = 0U;
        return;
    }

    /* 还有配额: 记一次"这一条失败了", 等 _try_send_alarm_idle 重发 */
    s->send_req_tick = get_tick_ms() - (LORA_ALARM_TIMEOUT_MS - LORA_ALARM_RETRY_GAP_MS);
    log_info("[GEAR] %02X%02X%02X 本条失败, 准备重发(已发 %u/%u)\r\n",
             s->dev_uid[0], s->dev_uid[1], s->dev_uid[2],
             (unsigned)s->alarm_attempts, (unsigned)LORA_ALARM_MAX_ATTEMPTS);
}

static void handle_ack_from_device(const lora_frame_t *frame, uint8_t slave_idx)
{
    /* 【g_unbind 拦截】解绑期间目标 slave 的 ack 由 g_unbind 状态机接管,
     *   不走下面的 data_cache / _reply_schedule 旧逻辑。
     *   必须在 s->state 检查之前拦: 解绑 slave 一直是 WAIT_ACK(锁), 旧逻辑
     *   会判成"等 ACK 状态"然后走进去误处理; 这里先把 g_unbind 在跟踪的 ack
     *   路由到 _unbind_on_ack, 由状态机按阶段1/阶段2 语义处理(收到 ack 不一定
     *   是成功 —— 阶段2 收到 ack 反而是失败)。 */
    if (g_unbind.state != UNBIND_IDLE && slave_idx == g_unbind.slave_idx)
    {
        (void)frame;   /* ack 内容不区分 0x80/0x81, 都视为设备应答 */
        _unbind_on_ack();
        return;
    }

    /* ★2026-10-18: 字段写(0x06)的 ack 归属 —— 必须在 slave 状态机之前判。
     *   字段操作【不走】SLAVE_STATE_WAIT_ACK(state 一直是 IDLE), 否则下面
     *   那句 `if (s->state != SLAVE_STATE_WAIT_ACK) return;` 会把 0x80 丢掉,
     *   于是"设备明明确认了, 串口却一直等到 3 次发完报失败"。
     *   0x80 = 写成功; 0x81 = 设备明确拒绝(立即判失败, 不再等剩下两条)。 */
    if (lora_field_on_ack(frame))
    {
        return;
    }

    lora_slave_t *s = &g_slave_list[slave_idx];
    if (s->state != SLAVE_STATE_WAIT_ACK) return;   /* 不是等 ACK 状态, 忽略 */

    /* ==========================================================================
     * ★★2026-10-08: 声光(0x03)是"三段式"应答, 0x80 不再是终态★★
     * --------------------------------------------------------------------------
     * 设备对 0x03 回两条:
     *     立即       0x80  = "我收到了"(不是执行成功!)
     *     约10秒后   0x82  = 执行成功   /   0x83 = 执行失败
     * 【为什么必须按 pending_func 分流】0x01(设液位参数)等其它功能码没有第二段,
     *   它们的 0x80 就是终态。把 0x80 一律当"还没完成"会让那些命令永远等不到结果。
     * ======================================================================== */
    if (s->pending_func == (uint8_t)LORA_FUNC_SET_ALARM)
    {
        if (frame->func == (uint8_t)LORA_FUNC_RESP_OK)          /* 0x80: 只是收到了 */
        {
            /* 【不结束】继续等 0x82/0x83; 但刷新一下时间戳 ——
             * 设备明确回话了, 说明链路是通的, 给它重新算 15 秒更公平。 */
            s->alarm_tick = get_tick_ms();
            log_info("[GEAR] %02X%02X%02X 已收下指令(0x80), 继续等执行结果(0x82/0x83)\r\n",
                     s->dev_uid[0], s->dev_uid[1], s->dev_uid[2]);
            return;
        }
        if (frame->func == (uint8_t)LORA_FUNC_ALARM_DONE)       /* 0x82: 成功 */
        {
            s->state          = SLAVE_STATE_IDLE;
            s->retry_cnt      = 0U;
            s->alarm_pending  = false;
            s->alarm_attempts = 0U;
            data_cache_down_del_by_uid(s->dev_uid);
            log_info("[GEAR] ★%02X%02X%02X 执行成功(0x82)★ 档位 %u\r\n",
                     s->dev_uid[0], s->dev_uid[1], s->dev_uid[2], (unsigned)s->alarm_gear);
            /* ★2026-10-18 (丙): 之前判过离线? 这一拍成功就是"恢复" ★
             * 带 recovered=true 回调, 主程序会补一条 event.device{"online"}。 */
            {
                bool was_offline = s->alarm_offline_reported;

                s->alarm_offline_reported = false;
                _alarm_fire_result(s, true, false, was_offline);
            }
            return;
        }
        if (frame->func == (uint8_t)LORA_FUNC_ALARM_FAIL)       /* 0x83: 失败 */
        {
            log_info("[GEAR] %02X%02X%02X 回执行失败(0x83), 第 %u/3 条\r\n",
                     s->dev_uid[0], s->dev_uid[1], s->dev_uid[2],
                     (unsigned)s->alarm_attempts);
            _alarm_on_fail(s);
            return;
        }
        return;     /* 其它功能码: 不动这个状态机 */
    }

    s->state       = SLAVE_STATE_IDLE;
    s->retry_cnt   = 0U;

    if (frame->func == LORA_FUNC_RESP_OK)
    {
        /* 成功 —— 清除下发缓存 */
        data_cache_down_del_by_uid(s->dev_uid);
    }
    /* 如果是 ERR —— 保留缓存, 让下次设备上报时 _try_send_downlink 再发 */
}

/* ========== 业务帧分发 (注册给协议层的 LoraFrameCallback_t) ========== */

/* ==================== 收到请求帧后的"分两步错开"应答时序 ====================
 *
 * 【为什么不能收到就立刻回】
 *   对端刚把那一包发完, 它的射频正在从 TX 切回 RX(还要重新上电接收链路)。
 *   网关这时立刻回 ACK / 下发指令, 前导码正好压在对端接收链路还没稳定的那段,
 *   对端解不出来 —— 现象就是"网关明明发了, 从机却没反应"。
 *   所以把它拆成两步, 每步之间留一段空档:
 *
 *     收到液位上报(0x02) / 其他请求帧
 *          └─ 等 GW_REPLY_ACK_DELAY_MS(3s)      → 发 0x80 ACK
 *               └─ 等 GW_REPLY_CMD_DELAY_MS(3s) → 从下发缓存取这条指令发出, 进 WAIT_ACK
 *
 * 【为什么用时间戳而不是 delay_ms()】
 *   delay_ms(3000) 是阻塞的, 两步加起来会把整个主循环按死 6s ——
 *   这 6s 里 lora_proto_poll() 不跑(别的从机的帧只能靠单缓冲暂存, 会被覆盖),
 *   4G AT 轮询也停摆(而且 g4_at_poll 自己最长还要阻塞 1s)。
 *   用"状态 + 时间戳"就完全没有这个问题: 主循环照常转, 到点了才动手。
 *
 * 全局只有一份待应答记录: 同一时刻先来后到, 后来的会覆盖前面的。
 * 从机不会高频上报, 台架实测够用; 真要多台并发再把这里改成队列。
 */
#define GW_REPLY_ACK_DELAY_MS   3000U   /* 收到请求帧后, 隔多久回 ACK */
#define GW_REPLY_CMD_DELAY_MS   3000U   /* 回完 ACK 后, 隔多久下发指令 */

typedef enum
{
    GW_REPLY_IDLE = 0,
    GW_REPLY_WAIT_ACK_TX,   /* 已收到请求帧, 等 GW_REPLY_ACK_DELAY_MS 再回 ACK */
    GW_REPLY_WAIT_CMD_TX,   /* ACK 已发, 等 GW_REPLY_CMD_DELAY_MS 再下发指令 */
} gw_reply_state_t;

static gw_reply_state_t g_reply_state = GW_REPLY_IDLE;
static uint32_t         g_reply_tick  = 0U;
static uint8_t          g_reply_dev[CFG_UID_BYTES];
/* ★★2026-10-18 修复(严重): 这里原来是 `[3]`★★
 *   _reply_schedule() 里是 `memcpy(g_reply_dev, dev_uid, CFG_UID_BYTES)`,
 *   UID 扩到 13 字节后, 往 3 字节数组里写 13 字节 ⇒
 *     ① 越界写坏紧跟其后的全局变量;
 *     ② 3 秒后 _reply_logic() 用 g_reply_dev 发 0x80 ACK 时,
 *        只有前 3 字节是对的, 后 10 字节是垃圾 ⇒ 地址根本不存在。
 *   现场表现(2026-10-18 实测): 设备上报 0x02 后, 网关的 ACK 发错地址,
 *   设备以为没收到, 连续重发 5 条 0x80(约 2 秒一条), 最后放弃。
 *   ⇒ 所有"存 13 字节 UID 的静态数组"都必须用 CFG_UID_BYTES 定尺寸。 */

/* ==================================================================
 * ★2026-10-18: UID 尺寸【静态断言】—— 防止"改了长度忘改缓冲"再次发生
 * ==================================================================
 * 【为什么要这一块】上面 g_reply_dev 那个故障的本质是:
 *     声明 3 字节数组, 却 memcpy 13 字节 ⇒ 越界写坏别的全局变量 +
 *     发出去的帧地址是垃圾, 而【编译期零告警、主机单测也测不到】,
 *     只有上板收发才会暴露。
 * 【为什么编译器抓不到】数组作函数参数会退化成指针, memcpy 的形参是 void* ——
 *   这一整条链路上没有任何类型/尺寸检查。
 * ⇒ 所以在这里把"存 UID 的结构体成员 / 文件级静态数组"的尺寸【钉死】。
 *   以后谁再改 UID 长度而漏改某个缓冲, 【这里编译就失败】, 不用等上板。
 * ================================================================== */
_Static_assert(sizeof(((lora_frame_t *)0)->gw_uid)        == CFG_UID_BYTES,
               "lora_frame_t.gw_uid 必须等于 CFG_UID_BYTES");
_Static_assert(sizeof(((lora_frame_t *)0)->dev_uid)       == CFG_UID_BYTES,
               "lora_frame_t.dev_uid 必须等于 CFG_UID_BYTES");
_Static_assert(sizeof(((lora_slave_t *)0)->dev_uid)       == CFG_UID_BYTES,
               "lora_slave_t.dev_uid 必须等于 CFG_UID_BYTES");
_Static_assert(sizeof(((cache_down_item_t *)0)->dev_uid)  == CFG_UID_BYTES,
               "cache_down_item_t.dev_uid 必须等于 CFG_UID_BYTES");
_Static_assert(sizeof(((device_param_t *)0)->gw_uid)      == CFG_UID_BYTES,
               "device_param_t.gw_uid 必须等于 CFG_UID_BYTES");
_Static_assert(sizeof(((param_slave_uid_t *)0)->uid)      == CFG_UID_BYTES,
               "param_slave_uid_t.uid 必须等于 CFG_UID_BYTES");
_Static_assert(sizeof(((last_value_t *)0)->uid)           == CFG_UID_BYTES,
               "last_value_t.uid 必须等于 CFG_UID_BYTES");
_Static_assert(sizeof(g_gw_uid)              == CFG_UID_BYTES, "g_gw_uid 必须 13 字节");
_Static_assert(sizeof(g_reply_dev)           == CFG_UID_BYTES, "g_reply_dev 必须 13 字节(2026-10-18 踩过坑)");
_Static_assert(sizeof(g_bind.pending_uid)    == CFG_UID_BYTES, "g_bind.pending_uid 必须 13 字节");
_Static_assert(sizeof(g_unbind.uid)          == CFG_UID_BYTES, "g_unbind.uid 必须 13 字节");
/* (tc_record_t / ec_record_t 的断言放在各自的 .c 里 —— 本文件不 include 那两个头) */
static uint8_t          g_reply_func  = 0U;
static uint8_t          g_reply_idx   = 0xFFU;

/* on_frame 里调用: 只登记"要回谁", 真正的发送交给 _reply_logic() 分两步做 */
static void _reply_schedule(const uint8_t dev_uid[CFG_UID_BYTES], uint8_t func, uint8_t idx)
{
    if (g_reply_state != GW_REPLY_IDLE)
    {
        log_info("[GW] !! 上一条待应答还没走完(dev=%02X%02X%02X), 被 %02X%02X%02X 覆盖\r\n",
                 g_reply_dev[0], g_reply_dev[1], g_reply_dev[2],
                 dev_uid[0], dev_uid[1], dev_uid[2]);
    }

    memcpy(g_reply_dev, dev_uid, CFG_UID_BYTES);
    g_reply_func  = func;
    g_reply_idx   = idx;
    g_reply_tick  = get_tick_ms();
    g_reply_state = GW_REPLY_WAIT_ACK_TX;
}

/* lora_business_poll() 每轮调用一次(1ms 精度) */
static void _reply_logic(void)
{
    uint32_t waited;

    if (g_reply_state == GW_REPLY_IDLE)
    {
        return;
    }

    /* ---- 第一步: 等够 GW_REPLY_ACK_DELAY_MS 后回 0x80 ---- */
    if (g_reply_state == GW_REPLY_WAIT_ACK_TX)
    {
        waited = (uint32_t)(get_tick_ms() - g_reply_tick);
        if (waited < GW_REPLY_ACK_DELAY_MS)
        {
            return;
        }

        log_info("[GW] 收到 %02X%02X%02X 的帧后等 %lums, 现在回 0x80 ACK\r\n",
                 g_reply_dev[0], g_reply_dev[1], g_reply_dev[2],
                 (unsigned long)waited);
        (void)lora_business_send_ack(g_gw_uid, g_reply_dev, g_reply_func, true, 0);

        g_reply_tick  = get_tick_ms();
        g_reply_state = GW_REPLY_WAIT_CMD_TX;
        return;
    }

    /* ---- 第二步: 再等 GW_REPLY_CMD_DELAY_MS 后下发指令 ---- */
    waited = (uint32_t)(get_tick_ms() - g_reply_tick);
    if (waited < GW_REPLY_CMD_DELAY_MS)
    {
        return;
    }

    g_reply_state = GW_REPLY_IDLE;

    if (g_reply_idx == 0xFFU)
    {
        log_info("[GW] 该从机没登记进从机表, 无从下发\r\n");
        return;     /* 从机表没登记成功, 无从下发 */
    }

    log_info("[GW] 回完 ACK 再等 %lums, 现在尝试下发 %02X%02X%02X 的待发指令\r\n",
             (unsigned long)waited,
             g_reply_dev[0], g_reply_dev[1], g_reply_dev[2]);

    /* 该从机 IDLE 且有缓存指令才会真的发出去。
     * 【这里必须把结果打出来】: 缓存为空时它是"什么都不做"的,
     * 不打日志的话现象就是"从机收到 ACK 之后就没下文了、直接睡过去",
     * 完全看不出是"网关没指令可发"还是"发了但从机没收到"。 */
    {
        int r = _try_send_downlink(g_reply_idx);

        switch (r)
        {
        case DOWNLINK_SENT:
            log_info("[GW] 下发指令已发出, 等从机 ACK\r\n");
            break;
        case DOWNLINK_NO_CACHE:
            log_info("[GW] !! 下发缓存里没有 %02X%02X%02X 的指令 -> 这一步什么都没发\r\n",
                     g_reply_dev[0], g_reply_dev[1], g_reply_dev[2]);
            break;
        case DOWNLINK_BUSY:
            log_info("[GW] 该从机还在等上一条指令的 ACK, 这轮跳过\r\n");
            break;
        case DOWNLINK_TX_FAIL:
            log_info("[GW] !! 下发发送失败(没等到 TX_DONE), 查射频/供电\r\n");
            break;
        default:
            log_info("[GW] !! 下发缓存项格式不对(连 func 都没有)\r\n");
            break;
        }
    }
}

/* ========== 业务帧分发 (注册给协议层的 LoraFrameCallback_t) ========== */

static void on_frame(const lora_frame_t *frame, int8_t rssi, int8_t snr)
{
    if (frame == NULL) return;

    /* 收到什么就打什么: 帧头/地址/功能码/原始字节, 台架联调排障全靠它 */
    log_info("[LoRa RX] rssi=%d snr=%d gw=%02X%02X%02X dev=%02X%02X%02X func=0x%02X dlen=%u len=%u:",
             (int)rssi, (int)snr,
             frame->gw_uid[0], frame->gw_uid[1], frame->gw_uid[2],
             frame->dev_uid[0], frame->dev_uid[1], frame->dev_uid[2],
             frame->func, (unsigned)frame->data_len, (unsigned)frame->raw_len);
    for (uint8_t i = 0U; i < frame->raw_len; i++)
    {
        log_info(" %02X", frame->raw[i]);
    }
    log_info("\r\n");

    /* ============ 绑定窗口拦截(P5, LoRa 0x04) ============
     * 必须在 _touch_slave / 白名单之前 —— 0x04 绑定请求的目标地址是广播
     * FFFFFF, dev_uid 是发起绑定的设备自己, 它还不在从机表里, 会被白名单
     * 丢弃。这里先拦下来走 _bind_handle_0x04, 由状态机决定是回 0x80 进入
     * 握手还是回 0x81 拒绝。
     *
     * 同样, 在 BIND_WAIT_DEV_ACK 期间, 对端回的 0x80 是"对 0x04 的确认",
     * 不能再走 handle_ack_from_device(那个是处理"对 0x01/0x03 下发的 ACK")。
     */
    if (g_bind.state != BIND_IDLE && frame->func == LORA_FUNC_BIND_REQ)
    {
        _bind_handle_0x04(frame);
        return;
    }
    if (g_bind.state == BIND_WAIT_DEV_ACK
        && frame->func == LORA_FUNC_RESP_OK
        && memcmp(frame->dev_uid, g_bind.pending_uid, CFG_UID_BYTES) == 0)
    {
        _bind_handle_dev_ack(frame);
        return;
    }

    /* ==========================================================================
     * ★★2026-10-08 修: 配对窗口【没开】时收到 0x04 —— 一律丢弃, 【绝不回 ACK】★★
     * --------------------------------------------------------------------------
     * 【现场现象(用户报告)】声光报警器解绑完之后不是关机, 而是进入"等待绑定"状态,
     *   于是它会持续发 0x04 绑定请求。而网关【给它回了 0x80】—— 设备会以为自己
     *   又被绑上了(或者反过来导致它一直停在绑定态)。
     *
     * 【为什么会回】原来的路径是:
     *     1615 行: 窗口没开 -> 不走配对状态机
     *     1629 行: _touch_slave() 摸一下 -> 如果这台设备【还在从机表里】, idx 有效
     *     1648 行: 通用 ACK 分支对"除 0x80/0x81 之外的所有 func"都回 ACK
     *   ⇒ 0x04 就搭上了这趟车。而 0x04 的语义是"我想加入", 不是一个业务请求 ——
     *     它【只】该由配对窗口状态机应答; 窗口没开就代表"没人在等它"。
     *
     * 【为什么必须早于 _touch_slave 拦】_touch_slave 的副作用是"标记在线 + 刷新时间戳",
     *   对一台刚被解绑的设备做这件事本身就会污染在线状态。
     *
     * 【为什么不顺手回 0x81(拒绝)】设备端没定义"绑定被拒"的处理路径, 回了反而可能
     *   让它改变状态。静默丢弃是最安全的: 设备该怎么退绑定态是【设备侧】的事。
     * ⚠️ 现场若看到这行日志反复刷, 说明设备一直在求绑定 —— 那是设备侧的问题
     *   (解绑后没关机/没退出绑定态), 需要设备侧修, 不是网关能替它解决的。 */
    if (frame->func == LORA_FUNC_BIND_REQ)
    {
        log_info("[BIND] 收到 %02X%02X%02X 的绑定请求(0x04), 但配对窗口未开"
                 " —— 丢弃, 不回 ACK(避免设备误以为已绑定)\r\n",
                 frame->dev_uid[0], frame->dev_uid[1], frame->dev_uid[2]);
        return;
    }

    /* 在从机表里"摸一下"这台设备(标记在线 + 记时间戳)。 */
    uint8_t idx = _touch_slave(frame->dev_uid);

    /* 【白名单】: 地址不在从机表里 -> 只打印, 一律丢弃。
     * 不回 ACK、不解析载荷、不入缓存 —— 陌生设备在这一层就被挡住。
     * ⚠ 0x04 绑定请求是特例(目标地址=广播 FFFFFF, dev_uid 是发起绑定的设备自己,
     *   不在表里) —— 步骤4 要在 _touch_slave 之前先判 func==0x04 且在绑定窗口,
     *   直接走 0x04 分支, 否则会被这里白名单丢弃。 */
    if (idx == 0xFFU)
    {
        log_info("[GW] !! 未知从机 %02X%02X%02X (不在从机表里), 丢弃该帧\r\n",
                 frame->dev_uid[0], frame->dev_uid[1], frame->dev_uid[2]);
        return;
    }

    /* 【不再立刻回】只登记, 由 _reply_logic() 等 3s 后回 ACK, 再等 3s 下发指令。
     *
     * ⚠️【0x80/0x81 绝不能回】: 那是"对方在应答我们下发的那条指令"。
     *    如果连它也要回一个 ACK, 两边就会互相 ACK 永远停不下来
     *    (设备侧一旦也这么实现, 空口就被这两帧刷满, 正常业务全被挤掉)。
     *
     * ★★2026-10-10: 0x82/0x83 同样【绝不能回】★★
     *   它们和 0x80/0x81 是同一类东西 —— "设备在应答/汇报我们下发的那条指令"。
     *   给它们回 ACK 就是同一场 ACK 风暴。
     *   (现场 2026-10-10: 声光回了 0x82, 走 default 打"未处理的功能码" —— 因为
     *    下面的 switch 里根本没有它的 case, 所以既没处理、也差点被这条 if 排上 ACK。) */
    if ((frame->func != LORA_FUNC_RESP_OK)    && (frame->func != LORA_FUNC_RESP_ERR) &&
        (frame->func != LORA_FUNC_ALARM_DONE) && (frame->func != LORA_FUNC_ALARM_FAIL) &&
        /* ★2026-10-18: 0x08(字段应答)也【不走】3 秒延迟 ACK ——
         * 设备侧等 ack 的窗口只有 5 秒, handle_field_resp() 里已经【立刻】回了。 */
        (frame->func != LORA_FUNC_FIELD_RESP))
    {
        _reply_schedule(frame->dev_uid, frame->func, idx);
    }

    switch (frame->func)
    {
    case LORA_FUNC_SENSOR_REPORT:     /* 0x02 — 液位传感器上报 */
        handle_sensor_report(frame, idx, rssi);
        break;
    case LORA_FUNC_FIELD_RESP:        /* 0x08 — 设备回报出厂字段(必须回 ack) */
        lora_field_handle_resp(frame, idx);
        break;
    case LORA_FUNC_RESP_OK:           /* 0x80 — 从机对我们下发的 ACK_OK */
    case LORA_FUNC_RESP_ERR:          /* 0x81 — 从机回错误/重传 */
    case LORA_FUNC_ALARM_DONE:        /* 0x82 — 声光: 执行成功 */
    case LORA_FUNC_ALARM_FAIL:        /* 0x83 — 声光: 执行失败 */
        /* ★2026-10-10: 后两个原来【漏了】, 于是 0x82/0x83 掉进 default 被当"未处理"。
         * 三者都由 handle_ack_from_device 分派:
         *   0x80 -> 只刷新时间戳(还需要等真正的执行结果)
         *   0x82 -> 上报 event.action{code=0} 执行成功
         *   0x83 -> 重发一次 0x03(配额内) */
        handle_ack_from_device(frame, idx);
        break;
    default:
        /* 未知/暂不处理的功能码 —— ACK 已由 _reply_logic() 排上。
         * ★ 2026-09-27 修复: 原来这里是【静默】的, 结果现场现象是
         *   "收包计数在涨、日志里却什么都没有", 排障时无从下手。
         *   现在把功能码打出来 —— 一眼就能看出对端发的是不是我认的那几个。 */
        log_info("[GW] !! 未处理的功能码 func=0x%02X (只认 0x02=液位上报/0x80=ACK_OK"
                 "/0x81=ACK_ERR/0x82=声光成功/0x83=声光失败)"
                 " dev=%02X%02X%02X dlen=%u\r\n",
                 (unsigned)frame->func,
                 frame->dev_uid[0], frame->dev_uid[1], frame->dev_uid[2],
                 (unsigned)frame->data_len);
        break;
    }

    /* 注意: 这里【不再】调 _try_send_downlink()。
     * 下发改由 _reply_logic() 的第二步做(回完 ACK 再等 3s),
     * 这样"回 ACK"和"下发指令"两条时间线不会各算各的。 */
}


/* ========== MQTT 下行处理: 云端直接下发【协议帧】 ==========
 *
 * 【2026-09 改版 —— JSON 全部删除】
 *   以前云端下发的是嵌套 JSON({"target_dev":"210001","cmd":"1","data":{...}}),
 *   为了在 MCU 上解析它写了一整套字符串查找 + 一堆字段名常量, 光字段名字符串
 *   就占了可观的 Flash, 而且云端字段一旦拼错/改名, 现象只是"静默什么都不做"。
 *   现在改成: 云端把【一个和 LoRa 空口一模一样的帧】直接发下来(原始二进制),
 *   网关做 lora_proto_parse_raw()(帧头/版本/长度/CRC 四重校验) → 按功能码处理。
 *   少一整层解析, 也少一整类"解析失败却不报错"的坑。
 *
 *   (2026-09 早期版本用的是 ASCII hex 文本承载, 现在云端已改成原始二进制;
 *    代码里 hex 分支保留做兼容 —— 只有 payload 不是 AA55 开头时才走它。)
 *
 *   线上收到的 payload 格式就是这 20 个字节(CRC 已按协议重算):
 *     AA 55 01 11 00 01 21 00 01 01 07 05 00 0A 00 00 DC 05 98 E0
 *   对应的帧:
 *     AA 55 | 01 | 11 00 01 | 21 00 01 | 01 | 07 | 05 00 0A 00 00 DC 05 | 98 E0
 *     └头┘  ver  └─ 网关 ─┘  └─ 设备 ─┘  fc  dlen  └──── data 7B ────┘  └CRC┘
 *   ⚠️【2026-09 更正】原来这里写的是 ...DC 05 38 1B, 那个 CRC 是错的 ——
 *     现场抓包(见 _rev_GOOD_2026-09-22\v4...\说明.txt 第五节, 当时
 *     MAIN_LORA_ENABLE=0 只跑 4G, 所以没人校验过它)里云端发下来的就是
 *     38 1B。正确值是 98 E0; 照抄 38 1B 会让设备端 CRC 不过、整帧被丢弃,
 *     现象就是"参数下发了但设备一直没同步时间(上报帧里月日时分全 00)"。
 *     算法: crc16_modbus(版本字节..data 末尾), 结果低字节在前。
 *
 * 转发规则(《液位监测系统通讯协议说明》第五节):
 *   1) 0x01(设置液位): 云端的 data 只有前 7 字节, 【没有】那 6 字节校时;
 *      网关转发前必须补上 年 月 日 周 时 分 —— 取自 AT+CCLK?, 星期固定填周五。
 *   2) 0x03(设置声光): 一个字节都不改, 直接打包转发。
 *   3) 0x02 是【上行】功能码, 云端不会下发; 0x80/0x81 若收到只当"云端应答"记一笔。
 */

/* 【单位: 秒 —— ★2026-10-05 用户口径改版★】
 *   平台侧配置(sen.set 的 reportPeriodSeconds)本来就是【秒】, 所以网关
 *   【不做任何换算】, 原样填进 0x01 帧的对应字段; 设备端按秒解析
 *   (设备侧由用户自己对接)。
 *
 *   ⚠ 这条与 2026-09 的"分钟"版本【不兼容】, 网关与设备必须同一次改版:
 *     旧版: 协议字段=分钟, 网关原样转发, 设备端自己 ×60 换成内部秒;
 *     现在: 协议字段=秒,   网关原样转发, 设备端按秒直接用 —— 谁都不换算。
 *     任何一侧漏改 => 时间差 60 倍(平台 5 分钟 -> 设备 5 小时)。
 *
 *   16 位字段上限 65535 秒 ≈ 18.2 小时; 平台侧 reportPeriodSeconds 的配置范围
 *   是 5~3600 秒(config_store_validate), 落在里面。 */
#define LORA_SENSOR_PERIOD_MIN_S     1U      /* 间隔最小 1 秒 */

/* 液位传感器是【休眠】设备: 默认不立即发, 等它上报时搭车发。
 * 改成 1 就在收到下行后立刻发(只有它正好在接收窗口内才有意义)。 */
#ifndef LORA_SENSOR_SEND_IMMEDIATE
#define LORA_SENSOR_SEND_IMMEDIATE   0
#endif

/* 下行失败统一出口: 云端下发是强约束指令, 丢弃必须留下原因, 否则只能靠猜 */
static void _downlink_drop(uint8_t func, const char *why)
{
    log_info("[CLOUD->LoRa] !! func=0x%02X %s, 丢弃\r\n", (unsigned)func, why);
}

/* ---------- 0x01 参数的公共落地点(两条下行共用) ----------
 *
 * 【为什么要抽出来】0x01 这条参数有【两个入口】:
 *     ① 平台直接下发 0x01 原始帧(云端自己拼帧);
 *     ② 平台下发 JSON sen.set -> cloud_proto 调 lora_business_set_sensor_params()。
 *   两个入口的 范围检查 / 补校时 / 投递 / 日志 必须完全一样 ——
 *   否则就会出现"用原始帧能下发、用 JSON 下发不了"这种最难查的不一致。
 *   所以两条路都走本函数, 只有日志里的来源标签(src)不同。
 *
 * 参数单位全部是【秒】(见上面的改版说明)。
 * 返回 0 成功; -1 载荷过长; -2 地址不在从机表; -3 间隔越界。
 */
static int _sensor_params_commit(const uint8_t uid[CFG_UID_BYTES], uint16_t detect_s,
                                 uint16_t report_s, bool same, uint16_t thresh_mm,
                                 const char *src)
{
    uint8_t    payload[13];
    g4_clock_t clk;
    int        ret;

    if ((detect_s < LORA_SENSOR_PERIOD_MIN_S) || (report_s < LORA_SENSOR_PERIOD_MIN_S))
    {
        log_info("[CLOUD->LoRa] !! 0x01(%s) 间隔越界(检测/上报都必须 >= %us): "
                 "detect=%us report=%us\r\n",
                 src, (unsigned)LORA_SENSOR_PERIOD_MIN_S,
                 (unsigned)detect_s, (unsigned)report_s);
        return -3;
    }

    /* 13 字节载荷, 16 位字段一律【低字节在前】, 间隔字段原样填秒值(不换算) */
    payload[0]  = (uint8_t)(detect_s & 0xFFU);
    payload[1]  = (uint8_t)((detect_s >> 8) & 0xFFU);
    payload[2]  = (uint8_t)(report_s & 0xFFU);
    payload[3]  = (uint8_t)((report_s >> 8) & 0xFFU);
    payload[4]  = same ? 1U : 0U;
    payload[5]  = (uint8_t)(thresh_mm & 0xFFU);
    payload[6]  = (uint8_t)((thresh_mm >> 8) & 0xFFU);

    /* 校时(6 字节): 先按"现在"填一份。真正发出去之前 _try_send_downlink() 还会
     * 【再取一次刷新】——液位是休眠设备, 参数可能在它上报前很久就排好了队,
     * 用排队时刻的时间戳会把设备 RTC 校偏(最坏偏一个上报周期)。
     * 现在取不到【不丢弃】: 填 0 并告警, 到了真正发送那一刻还有机会取到
     * (旧版这里是直接丢弃, 那会让 sen.set 在 4G 刚断开时静默失效)。 */
    if (g4_at_get_clock(&clk))
    {
        payload[7]  = clk.year;  payload[8]  = clk.month; payload[9]  = clk.day;
        payload[10] = clk.week;  payload[11] = clk.hour;  payload[12] = clk.minute;

        log_info("[CLOUD->LoRa] 0x01(%s) 液位 %02X%02X%02X detect=%us report=%us "
                 "same=%u thresh=%umm time=20%02u-%02u-%02u w%u %02u:%02u\r\n",
                 src, uid[0], uid[1], uid[2],
                 (unsigned)detect_s, (unsigned)report_s, (unsigned)payload[4],
                 (unsigned)thresh_mm,
                 (unsigned)clk.year, (unsigned)clk.month, (unsigned)clk.day,
                 (unsigned)clk.week, (unsigned)clk.hour, (unsigned)clk.minute);
    }
    else
    {
        (void)memset(&payload[7], 0, 6U);
        log_info("[CLOUD->LoRa] !! 0x01(%s) 现在取不到 4G 时间(AT+CCLK?), 校时先填 0"
                 "(真正发出前会再取一次)\r\n", src);
    }

    ret = lora_business_send_req(uid, SLAVE_TYPE_SENSOR,
                                 LORA_FUNC_SET_SENSOR_PARAM,
                                 payload, (uint8_t)sizeof(payload));
    if (ret != 0)
    {
        /* 把真实原因分开报 —— 地址没登记和缓存满完全是两个方向的排查 */
        log_info("[CLOUD->LoRa] !! 0x01(%s) 投递失败: %s\r\n", src,
                 (ret == -2) ? "该地址不在固定从机表里(液位必须先在表里登记)" :
                 (ret == -1) ? "载荷过长(>239字节)" : "下发缓存满/写入失败");
        return ret;
    }
    return 0;
}

/* ---------- 0x01 入口②: 平台 JSON(sen.set) ----------
 * 见 lora_business.h 的说明。单位【秒】, 不换算。 */
bool lora_business_set_sensor_params(const char *uid_hex,
                                     uint16_t detect_period_s,
                                     uint16_t report_period_s,
                                     bool     report_same_level,
                                     uint16_t alarm_thresh_mm)
{
    uint8_t uid[CFG_UID_BYTES];

    if (uid_hex == NULL) { return false; }
    if (!cfg_uid_to_bytes(uid_hex, uid))
    {
        log_info("[CLOUD->LoRa] !! sen.set: uid \"%s\" 不是合法的 6 位 hex 串\r\n", uid_hex);
        return false;
    }
    return (_sensor_params_commit(uid, detect_period_s, report_period_s,
                                  report_same_level, alarm_thresh_mm, "sen.set") == 0);
}

/* ---------- 0x01 入口①: 云端直接下发的原始帧 ----------
 *
 * 云端 data(7 字节, 单位【秒】, 见上面的改版说明):
 *   off0-1 检测间隔(小端) off2-3 上报间隔(小端) off4 相同液位是否上报
 *   off5-6 液位告警阈值 mm(小端)
 *
 * 下发给设备的 13 字节(app_cmd.h):
 *   off0-1 检测间隔(小端, 秒) off2-3 上报间隔(小端, 秒) off4 相同上报表
 *   off5-6 阈值(小端) off7 年 off8 月 off9 日 off10 周 off11 时 off12 分
 *   ⚠️ 16 位字段必须【低字节在前】, 写成大端会被设备解成 512/256/62465 这种乱值。
 */
static void _downlink_set_sensor(const lora_frame_t *frame)
{
    uint32_t detect_s, report_s, thresh_mm;
    int      ret;

    if (frame->data_len < 7U)
    {
        _downlink_drop(frame->func, "载荷不足 7 字节");
        return;
    }

    detect_s  = ((uint32_t)frame->data[0] | ((uint32_t)frame->data[1] << 8));
    report_s  = ((uint32_t)frame->data[2] | ((uint32_t)frame->data[3] << 8));
    thresh_mm = ((uint32_t)frame->data[5] | ((uint32_t)frame->data[6] << 8));

    ret = _sensor_params_commit(frame->dev_uid, (uint16_t)detect_s, (uint16_t)report_s,
                                (frame->data[4] != 0U), (uint16_t)thresh_mm,
                                "云端原始帧");
    if (ret != 0)
    {
        return;     /* 具体原因 _sensor_params_commit() 已经打过了, 不重复刷日志 */
    }

#if LORA_SENSOR_SEND_IMMEDIATE
    if (!_any_slave_waiting_ack())
    {
        uint8_t idx = _find_slave(frame->dev_uid);
        if (idx != 0xFFU)
        {
            (void)_try_send_downlink(idx);
        }
    }
    log_info("[CLOUD->LoRa] 已立即下发(液位立即模式)\r\n");
#else
    log_info("[CLOUD->LoRa] 已缓存, 等液位下次上报时随应答下发\r\n");
#endif
}

/* ---------- 0x03 云端设置声光报警器: 原样转发 ----------
 *
 * 云端 data 顺序即线上字节顺序, 与设备端完全一致 —— 网关【不解析、不加工】:
 *   power_ctrl 1B | light_ctrl 1B | volume 1B | led_mode 1B | led_brightness 1B |
 *   led_scroll_speed 1B | led_text_len 1B | led_text(len 字节 GB2312) |
 *   tts_text_len 1B | tts_text(len 字节 GB2312)
 *
 * 报警器常驻 RX, 缓存完立刻发; 无线忙就留在缓存里, 由 poll 里的
 * _try_send_alarm_idle() 捡起来发。
 */
static void _downlink_set_alarm(const lora_frame_t *frame, uint8_t idx)
{
    int ret;

    if (frame->data_len == 0U)
    {
        _downlink_drop(frame->func, "载荷为空");
        return;
    }

    /* 【不在固定从机表里 -> 直接转发】(现场规则: 地址首位是 3 就是声光报警器)
     *   报警器【常驻接收】, 不必等它上报, 所以直接上发即可 ——
     *   而且数据【一个字节都不改】: 现场要求"声光报警的数据我们不处理, 只判断"。
     *   (在表里的走下面那条"缓存 + 立即发 + ACK 重传"的路径, 多一层保障。) */
    if (idx == 0xFFU)
    {
        log_info("[CLOUD->LoRa] 0x03 声光 %02X%02X%02X 不在固定表里 -> 【直接转发】"
                 "%u 字节(数据不改)\r\n",
                 frame->dev_uid[0], frame->dev_uid[1], frame->dev_uid[2],
                 (unsigned)frame->data_len);

        /* 这帧就是从云端解出来的原帧; 组帧是幂等的(字段不变 -> CRC 不变),
         * 所以重发一次等于把云端那串字节原样送出去。
         * 去 const 是安全的: 实参是 lora_mqtt_recv_cb 里的非 const 局部变量。 */
        if (lora_proto_send_frame((lora_frame_t *)frame))
        {
            log_info("[CLOUD->LoRa] 直接转发完成\r\n");
        }
        else
        {
            log_info("[CLOUD->LoRa] !! 直接转发失败(射频没发出去) -> 查 LoRa 供电/天线\r\n");
        }
        return;
    }

    /* 下发缓存每台设备只有 64 字节(其中 1 字节给 func), 所以 data 最多 63 字节。
     * 声光那条带 LED(12B) + TTS(32B) 的实测载荷是 52 字节, 够用。 */
    ret = lora_business_send_req(frame->dev_uid, SLAVE_TYPE_ALARM_OUT,
                                 LORA_FUNC_SET_ALARM, frame->data, frame->data_len);
    if (ret != 0)
    {
        _downlink_drop(frame->func, "投递失败(载荷>63字节或缓存满)");
        return;
    }

    log_info("[CLOUD->LoRa] 0x03 声光 %02X%02X%02X %u 字节已缓存\r\n",
             frame->dev_uid[0], frame->dev_uid[1], frame->dev_uid[2],
             (unsigned)frame->data_len);

    if (!_any_slave_waiting_ack())
    {
        (void)_try_send_downlink(idx);
    }
}

/* ---------- MQTT 下行回调(由 lora_business_init 注册给 g4_urc) ----------
 *
 * 全模块【唯一】的下行入口: 4G URC → g4_io 收包泵 → g4_urc 队列 → g4_urc_poll → 这里。
 * 没有对外函数, 也没有"串口手敲当云端下发"的台架后门。
 *
 * 【2026-09 台架联调改版】云端现在直接下发【原始二进制帧】(AA 55 ... CRC),
 * 不再是 ASCII hex 文本 —— 所以这里先按二进制收, 同时保留 hex 兼容分支。
 *
 * 【LORA_DOWNLINK_RAW_FORWARD = "粗略直转"开关】
 *   1(台架联调用) = 校验通过后【原样把这一帧发到空口】: 不查白名单、不解释
 *       func、不取 4G 时钟、不等从机上报搭车。目的只有一个: 先确认
 *       "云端 -> 网关 -> 空口" 这条路通不通, 别被业务解释(校时/参数范围/等待
 *       上报)干扰判断。发出去的字节和云端发下来的完全一样(parse 后重新组帧,
 *       字段一致 -> CRC 一致)。
 *   0 = 走原来的业务解释: 0x01 设置液位传感器参数(需要 AT+CCLK? 校时),
 *       0x03 设置声光报警器(常驻 RX, 立即下发)。
 *
 * ★2026-09-22 默认从 1 改回 0★ —— 台架联调结束, 回到正式协议流程:
 *   云端下发 0x01(7 字节, 不带时间) -> 网关补齐 6 字节校时 -> 等液位下次上报
 *   时随应答下发。详见下面 LORA_DOWNLINK_PASSTHROUGH 的说明。
 */

/* 解出来的二进制帧放静态区: 一次 253 字节, 不占本来就紧张的栈 */
static uint8_t s_down_bin[LORA_FRAME_RAW_MAX];

#ifndef LORA_DOWNLINK_RAW_FORWARD
#define LORA_DOWNLINK_RAW_FORWARD   0
#endif

/* ==================== 【最简透传开关 LORA_DOWNLINK_PASSTHROUGH】 ====================
 *
 * 1 = 云端下发什么字节, 空口就原样发什么字节:
 *     · 不看帧头 AA55、不校验版本/长度/CRC、不解释 func、不查从机白名单;
 *     · 不补 0x01 那 6 字节校时, 不等从机上报搭车, 收到就发;
 *     · 从机表 / 上行缓存 / ACK 重传那套业务逻辑照旧保留(上行和"收到从机帧后
 *       回 0x80"都不受影响), 只是【下行不再走业务解释】。
 *   目的: 先把"云端 → 网关 → 空口"这条路单独走通, 别被业务解释(校时/参数范围/
 *   等待上报)干扰判断。对端(声光报警器 / 任意 LoRa 接收机)收到什么就是什么。
 *
 * 0 = 回到带帧校验/业务解释的路径(下面的 #else 分支): 下行必须是以 AA55 开头的
 *     合法协议帧, 再由 LORA_DOWNLINK_RAW_FORWARD 决定"校验后直转"还是业务解释。
 *
 * ★2026-09-22 曾从 1 改回 0★ —— 当时是"透传属台架临时手段, 回正式协议"。
 * ★2026-10-08 又改回 1★ —— 甲方平台要下发自定义载荷(非 LoRa 帧), 见上面的说明。
 *   下面这段"正式下行流程"仍然有效, 只是**现在默认不走它**; 把本宏改回 0 即可恢复。
 *
 *   【正式下行流程(宏=0 时的行为, 现场确认)】
 *     1) 云端下发一帧协议帧到 ★rt/gw/{gwId}/cmd★
 *          (2026-10-08 前是 mod/pc_down —— 该主题已随甲方要求整体删除)
 *          AA 55 | 01 | 网关3B | 从机3B | func | dlen | data | CRC16(2B)
 *        func: 0x01 设置液位传感器 / 0x03 设置声光报警器 /
 *              0x02 上报(上行) / 0x80 应答OK / 0x81 应答错误
 *        0x01 的 data 固定 7 字节, 【不带时间】:
 *          [0-1] 检测参数间隔 [2-3] 上报时间间隔 (单位【秒】, 小端)
 *          [4]   相同液位是否上报  [5-6] 液位告警阈值 mm (小端)
 *     2) 网关校验(帧头/版本/长度/CRC)、查从机白名单、把这条指令放进【下发缓存】;
 *     3) 网关【等液位下次上报】—— 液位是休眠设备, 只在醒来那一小段窗口里能收;
 *     4) 收到液位上报(0x02)后: 等 3s 回 0x80 ACK -> 再等 3s 把缓存的指令发下去,
 *        正好落在液位的接收窗口内(见 _reply_logic / GW_REPLY_*_DELAY_MS);
 *     5) 0x01 实际发出去时, 由网关补上 6 字节校时:
 *          [7]年 [8]月 [9]日 [10]周 [11]时 [12]分  —— 共 13 字节
 *        时间【在要发的这一刻】向 4G 模块取(AT+CCLK?), 应答里没有星期, 固定周五。
 *
 * 【透传的代价(宏=1 时)】lora_proto_send_raw() 会阻塞 LORA_TX_SETTLE_MS(500ms) +
 *   等 TX_DONE(≤1s) —— 最长约 1.5 秒卡在主循环里, 期间 AT 轮询/日志都停下。
 *   宏=0 时走组帧路径(同样阻塞), 但不再是"收到就发", 而是搭在液位上报窗口里,
 *   不影响 4G 下行的投递节奏。这是切回 0 的主要理由, 现在为了自定义载荷接受了它。
 *
 * ★2026-10-08 用户决定: 默认改回 1(最简透传)★
 *   用户口径: "我发不是 lora 的 hex, lora 直接转发" ——
 *   即平台往 rt/gw/{gwId}/cmd 发【任意字节】(不是 AA55 协议帧、也不是合法 hex),
 *   网关【不做任何帧校验/业务解释】, 原样推到 LoRa 空口。
 *
 *   这是 2026-09-22 之前长期使用的行为(那次是为了回正式协议才改成 0);
 *   现在因为甲方平台要下发自定义载荷, 又切回透传。
 *
 *   ⚠️ 透传仍然【只影响下行】: 上行(0x02 解析 → last_value → rt/gw/{gwId}/telemetry)、
 *      从机表、绑定窗口、0x80/0x82/0x83 的内部处理【全部照旧】, 一点没变。
 *   ⚠️ 前置换算(order 很重要): JSON 仍然走云协议! 见 lora_mqtt_recv_cb 开头 ——
 *      `main_cloud_downlink_try()` 先嗅探, 像 JSON(`{`...`}` 且后面有 `}`/`"`)
 *      就交给 cloud_proto, 【根本到不了这里】。所以透传只吃"非 JSON"的下行。
 *   ⚠️ 单包上限 LORA_FRAME_DATA_MAX(240 字节), 超了丢并打日志。
 *   ⚠️ 这条路【不回平台任何应答】(本文件没有任何 g4_at_mqtt_publish 调用),
 *      从机的 0x80/0x82/0x83 也只在网关内部处理, 不转发给平台。
 */
#ifndef LORA_DOWNLINK_PASSTHROUGH
#define LORA_DOWNLINK_PASSTHROUGH   1
#endif

/* 下行前置过滤器(见 lora_business.h 的说明)。
 * 不注册时行为与本改动前完全一致。 */
static lora_dl_prefilter_t g_dl_prefilter;

void lora_business_set_dl_prefilter(lora_dl_prefilter_t fn)
{
    g_dl_prefilter = fn;
}

static void lora_mqtt_recv_cb(const char *topic, const uint8_t *data, uint16_t len)
{
    /* 先让订阅者(云协议层)判断这条归不归它管。
     * 归它管就到此为止, 不再走下面的 LoRa 透传 —— 这是过渡期两种协议共存的关键。 */
    if ((g_dl_prefilter != NULL) && g_dl_prefilter(topic, data, len))
    {
        return;
    }

    (void)topic;        /* 下面的分支不一定用到 topic */
#if LORA_DOWNLINK_PASSTHROUGH
    /* ---------------- 最简透传: 云端发什么, 空口发什么 ---------------- */
    uint16_t i;

    if ((data == NULL) || (len == 0U))
    {
        log_info("[CLOUD->LoRa] !! 下行空载荷, 不发\r\n");
        return;
    }

    log_info("[CLOUD->LoRa] 透传 %u 字节到空口:", (unsigned)len);
    for (i = 0U; (i < len) && (i < 64U); i++)
    {
        log_info(" %02X", (unsigned)data[i]);
    }
    if (len > 64U)
    {
        log_info(" ...(只打印前 64 字节)");
    }
    log_info("\r\n");

    /* 空口单包上限: 超过就只能丢(留着也比发半个包强)。
     * LORA_FRAME_DATA_MAX 见 lora_proto.h。 */
    if (len > (uint16_t)LORA_FRAME_DATA_MAX)
    {
        log_info("[CLOUD->LoRa] !! 下行 %u 字节超过空口单包上限 %u, 丢弃\r\n",
                 (unsigned)len, (unsigned)LORA_FRAME_DATA_MAX);
        return;
    }

    if (lora_proto_send_raw(data, (uint8_t)len))
    {
        log_info("[CLOUD->LoRa] 透传完成\r\n");
    }
    else
    {
        log_info("[CLOUD->LoRa] !! 透传失败(射频没发出去) -> 查 LoRa 供电/EN/BUSY/天线\r\n");
    }

    /* 下面这些是"带帧校验的业务路径"才用到的(#else 分支), 透传模式下不引用它们,
     * 显式引一下, 免得编译器报一堆 "declared but never referenced"。 */
    (void)s_down_bin;
    (void)_hex_str2bin;
    (void)_downlink_drop;
    (void)_downlink_set_sensor;
    (void)_downlink_set_alarm;
    return;
#else
    lora_frame_t frame;
    int          n;
    uint8_t      idx;

    /* ---- 1. 取出原始帧字节 ---- */
    if ((data == NULL) || (len < LORA_FRAME_OVERHEAD))
    {
        log_info("[MQTT->LoRa] !! 下行过短(%u 字节, 至少要 %u)\r\n",
                 (unsigned)len, (unsigned)LORA_FRAME_OVERHEAD);
        return;
    }

    if ((data[0] == LORA_FRAME_HEAD0) && (data[1] == LORA_FRAME_HEAD1))
    {
        /* 原始二进制帧(当前云端的下发方式) */
        if (len > (uint16_t)sizeof(s_down_bin))
        {
            log_info("[MQTT->LoRa] !! 下行过长(%u 字节, 缓冲 %u)\r\n",
                     (unsigned)len, (unsigned)sizeof(s_down_bin));
            return;
        }
        memcpy(s_down_bin, data, len);
        n = (int)len;
        log_info("[MQTT DOWN] 原始二进制帧 %u 字节 (帧头 AA55)\r\n", (unsigned)len);
    }
    else
    {
        /* 兼容分支: 旧格式 ASCII hex 文本 */
        log_info("[MQTT DOWN] %u bytes(按 hex 文本解析): %.*s\r\n",
                 (unsigned)len, (int)((len < 200U) ? len : 200U), (const char *)data);
        n = _hex_str2bin((const char *)data, (int)len, s_down_bin, (int)sizeof(s_down_bin));
        if (n < 0)
        {
            log_info("[MQTT->LoRa] !! 下行既不是 AA55 开头的二进制帧, 也不是合法 hex\r\n");
            return;
        }
    }

    /* 帧头 AA55 + 版本 + 长度自洽 + CRC16, 四项全过才认 (n = 上面解出的字节数) */
    if (!lora_proto_parse_raw(s_down_bin, (uint8_t)n, &frame))
    {
        log_info("[MQTT->LoRa] !! 帧校验失败(帧头/版本/长度/CRC), 共 %d 字节\r\n", n);
        return;
    }

    /* 【设备类型按"地址首位"判断, 不再只认固定从机表】(现场规则)
     *   1xxxxx 网关 / 2xxxxx 液位传感器 / 3xxxxx 声光报警器
     * 固定表还是要查 —— 在表里就能享受"在线跟踪 + ACK 重传"那套有状态的处理;
     * 不在表里【也不代表这帧没用】: 只要地址首位是认得的类型, 后面按类型分发,
     * 声光报警器会走"直接转发"分支(见 _downlink_set_alarm)。
     * (老代码在这里直接 return 丢弃 —— 那样一台没登记的报警器就永远收不到指令。) */
    idx = _find_slave(frame.dev_uid);
    if (idx == 0xFFU)
    {
        log_info("[GW] 从机 %02X%02X%02X 不在固定表里, 按【地址首位=%u】判断设备类型\r\n",
                 frame.dev_uid[0], frame.dev_uid[1], frame.dev_uid[2],
                 (unsigned)_dev_type_of_uid(frame.dev_uid));
    }

    /* 帧里的"网关设备地址"应该是本机。不是也照样转发(要不要拦是策略问题),
     * 但必须打出来 —— 否则云端把网关号填错会一直没人发现。 */
    if (memcmp(frame.gw_uid, g_gw_uid, CFG_UID_BYTES) != 0)
    {
        log_info("[MQTT->LoRa] !! 帧里网关地址 %02X%02X%02X 与本机 %02X%02X%02X 不一致\r\n",
                 frame.gw_uid[0], frame.gw_uid[1], frame.gw_uid[2],
                 g_gw_uid[0], g_gw_uid[1], g_gw_uid[2]);
    }

    log_info("[CLOUD->LoRa] 收到 %d 字节帧: dev=%02X%02X%02X func=0x%02X dlen=%u\r\n",
             n, frame.dev_uid[0], frame.dev_uid[1], frame.dev_uid[2],
             (unsigned)frame.func, (unsigned)frame.data_len);

#if LORA_DOWNLINK_RAW_FORWARD
    /* ---------------- 粗略直转(台架联调) ----------------
     * 校验通过(帧头/版本/长度/CRC)之后【原样把这一帧发到空口】, 不做业务解释:
     *   · 不取 4G 时钟 —— 0x01 本来要靠 AT+CCLK? 校时, 先跳过;
     *   · 不等从机上报搭车下发;
     *   · 不建 WAIT_ACK/重传状态机(收到 ACK 也当普通帧处理)。
     * 目的只有一个: 先确认 "云端 -> 网关 -> 空口" 这条路通不通。
     * 发出去的字节 = 云端发下来的字节(parse 后重新组帧, 字段一致 -> CRC 一致)。
     *
     * 【代价】lora_proto_send_frame() 会阻塞 LORA_TX_SETTLE_MS(500ms) + 等 TX_DONE,
     *        这几百毫秒主循环是停住的。它跑在 g4_urc_poll()(收包泵之外), 所以不会
     *        丢 4G 接收, 但下行连发时会把投递排到后面。台架联调可接受。
     * 想回到"带业务解释"的老路径: 把 LORA_DOWNLINK_RAW_FORWARD 改成 0。 */
    (void)idx;
    /* 老的业务解释路径仍在编译单元里(把 LORA_DOWNLINK_RAW_FORWARD 改成 0 即可切回),
     * 这里显式引用一下, 免得编译器报"定义未使用"。 */
    (void)_downlink_set_sensor;
    (void)_downlink_set_alarm;
    log_info("[CLOUD->LoRa] 直转 %d 字节原始帧到空口: dev=%02X%02X%02X func=0x%02X dlen=%u\r\n",
             n, frame.dev_uid[0], frame.dev_uid[1], frame.dev_uid[2],
             (unsigned)frame.func, (unsigned)frame.data_len);

    if (lora_proto_send_frame(&frame))
    {
        log_info("[CLOUD->LoRa] 直转完成\r\n");
    }
    else
    {
        log_info("[CLOUD->LoRa] !! 直转失败(组帧或射频没发出去)\r\n");
    }
    return;
#else
    switch (frame.func)
    {
    case LORA_FUNC_SET_SENSOR_PARAM:            /* 0x01 设置液位传感器参数 */
        /* 类型判断改看【地址首位】: 2xxxxx = 液位传感器 */
        if (_dev_type_of_uid(frame.dev_uid) != LORA_UID_TYPE_LEVEL)
        {
            _downlink_drop(frame.func, "设备地址首位不是 2 -> 不是液位传感器");
            break;
        }
        _downlink_set_sensor(&frame);
        break;

    case LORA_FUNC_SET_ALARM:                   /* 0x03 设置声光报警器 */
        /* 类型判断改看【地址首位】: 3xxxxx = 声光报警器
         * 声光的数据【网关不处理, 只判断】—— 认出类型后原样转发。 */
        if (_dev_type_of_uid(frame.dev_uid) != LORA_UID_TYPE_ALARM)
        {
            _downlink_drop(frame.func, "设备地址首位不是 3 -> 不是声光报警器");
            break;
        }
        _downlink_set_alarm(&frame, idx);
        break;

    case LORA_FUNC_RESP_OK:                     /* 0x80 云端对上行上报的应答 */
    case LORA_FUNC_RESP_ERR:                    /* 0x81 */
        log_info("[CLOUD->LoRa] 云端应答 0x%02X(%s), 无需处理\r\n",
                 (unsigned)frame.func,
                 (frame.func == LORA_FUNC_RESP_OK) ? "OK" : "ERR");
        break;

    default:
        _downlink_drop(frame.func, "功能码不支持(下行只认 0x01/0x03)");
        break;
    }
#endif /* LORA_DOWNLINK_RAW_FORWARD */
#endif /* LORA_DOWNLINK_PASSTHROUGH */
}

/* 说明: 原来这里还有一个 lora_business_mqtt_recv() —— 给"日志口手敲一行 payload
 * 当云端下发"用的台架后门, 已删除: 下行只走 4G/MQTT 这一条路。
 * 台架上要模拟云端下发, 请直接往 MQTT 下行主题 pub 一条 hex 帧。 */


/* ========== 对外 init / poll / API ========== */

void lora_business_init(void)
{
    g_last_poll_tick = get_tick_ms();

    /* 1. 从 param_save 读取网关 UID (Flash 里如果还没烧写过会用 param_load_factory_default 的默认值) */
    param_save_init();
    device_param_t *p = param_get_ptr();
    memcpy(g_gw_uid, p->gw_uid, CFG_UID_BYTES);

    /* 2. 装载从机表: 从 config_store 的 sensors[]/devices[] 读进来(uid + 设备类型),
     *    其余字段清零。运行期还支持 0x04 绑定追加 / 0x05 解绑删除。 */
    _slave_table_load();

    /* 3. 协议层 init + 注册自己为帧回调 (同时自动开持续 RX) */
    lora_proto_init(on_frame);

    /* 4. data_cache init */
    data_cache_init();

    /* 5. 注册 MQTT 下行回调: 云端发的设置指令会转成 LoRa 帧发出去 */
    g4_urc_set_recv_cb(lora_mqtt_recv_cb);
}

void lora_business_poll(void)
{
    /* 1. 协议层 poll (radio 中断 + 帧解析 + 分发 on_frame) */
    lora_proto_poll();

    /* 1.5 收到请求帧后的分两步应答: 等 3s 回 ACK, 再等 3s 下发指令。
     *     每轮都算(1ms 精度), 只比时间戳, 不阻塞。 */
    _reply_logic();

    /* 1.6 绑定窗口状态机: 每轮都判(1ms 精度), 负责重发 0x80 + 窗口到期处理。
     *     只在非 IDLE 时有动作, 平常(没开窗口)立即 return, 不影响主循环节拍。 */
    _bind_logic();

    /* 1.7 解绑两阶段状态机: 每轮都判(1ms 精度), 负责阶段1/等15s/阶段2 的
     *     3s 间隔发 0x05 + 阶段超时/成功/故障处理。只在非 IDLE 时有动作,
     *     平常立即 return, 不影响主循环节拍。 */
    _unbind_logic();

    /* 2. ACK 超时重传 + 声光报警器主动下发 (周期轮询) */
    uint32_t now = get_tick_ms();
    if ((now - g_last_poll_tick) >= LORA_POLL_PERIOD_MS)
    {
        g_last_poll_tick = now;
        _handle_down_logic();       /* 重传超时的 WAIT_ACK 从机 */
        _try_send_alarm_idle();      /* 扫 IDLE 声光报警器, 捡起缓存的待发指令 */
        /* ★2026-10-08: 这里原来还调 data_cache_poll()(发 mod/mod_upload 那条 hex 上行)。
         * 该通道已按甲方要求整体删除, 调用一并去掉 —— 液位走的是 last_value 快照
         * + main.c 的 rt/gw/{gwId}/telemetry, 与本处无关。 */
    }
}

bool lora_business_send_ack(const uint8_t gw_uid[CFG_UID_BYTES], const uint8_t dev_uid[CFG_UID_BYTES],
                            uint8_t func, bool ok, uint8_t err_code)
{
    lora_frame_t frame;
    memset(&frame, 0, sizeof(frame));
    if (gw_uid)  memcpy(frame.gw_uid,  gw_uid, CFG_UID_BYTES);
    if (dev_uid) memcpy(frame.dev_uid, dev_uid, CFG_UID_BYTES);
    frame.func = ok ? LORA_FUNC_RESP_OK : LORA_FUNC_RESP_ERR;

    /* 【0x80/0x81 是纯应答码, 没有数据域, data_len 必须为 0】
     * 正确帧 13 字节: AA 55 01 <gw3> <dev3> 80 00 <CRC>
     *
     * 这里原来塞了 2 字节 (data[0]=err_code, data[1]=原功能码)。
     * 设备端 app_cmd.c 收到 0x80/0x81 是直接把 ack_len 置 0 返回的, 不看载荷,
     * 所以那 2 字节一直是白带的 —— 把它去掉, 和协议/设备端对齐。
     *
     * ⚠️ 如果以后确实要让应答携带信息(比如 err_code 给报警器判读),
     *    必须【两端同时】改协议, 不能只改这里。 */
    frame.data_len = 0U;

    (void)func;      /* 载荷不带原功能码了, 参数保留是为了不改调用方 */
    (void)err_code;

    return lora_proto_send_frame(&frame);
}

int lora_business_send_req(const uint8_t uid[CFG_UID_BYTES], slave_type_t dev_type,
                           uint8_t func, const uint8_t *data, uint8_t data_len)
{
    uint8_t idx;

    if (data_len > (LORA_FRAME_DATA_MAX - 1U)) return -1;  /* 留 1B 给 func */

    /* 【从机表是动态白名单】: 地址不在表里就直接拒掉 —— 设备必须先经 0x04 绑定
     * 进表(写 config_store + g_slave_list)才会被这里找到。 */
    idx = _find_slave(uid);
    if (idx == 0xFFU) return -2;

    /* 表里的 dev_type 在绑定时确定, 运行期不变。调用方传进来的如果和表不一致,
     * 以表为准并告警(避免一个地址被当成两种设备) */
    if (g_slave_list[idx].dev_type != dev_type)
    {
        log_info("[GW] !! %02X%02X%02X 在从机表里是 %s, 调用方却按 %s 下发, 以表为准\r\n",
                 uid[0], uid[1], uid[2],
                 (g_slave_list[idx].dev_type == SLAVE_TYPE_SENSOR) ? "SENSOR" : "ALARM_OUT",
                 (dev_type == SLAVE_TYPE_SENSOR) ? "SENSOR" : "ALARM_OUT");
        dev_type = g_slave_list[idx].dev_type;
    }

    /* 把 func 塞进 payload[0], data 接在后面 —— 这样 data_cache_down_push
     * 只需要存一段连续字节流, fetch 时取出来直接当 frame.data[] */
    uint8_t payload[LORA_FRAME_DATA_MAX];
    payload[0] = func;
    if (data_len > 0U && data != NULL)
    {
        memcpy(&payload[1], data, data_len);
    }

    int ret = data_cache_down_push(uid, dev_type, payload, (uint8_t)(data_len + 1U));
    if (ret != 0) return -3;

    /* 实际发送时机:
     *   声光报警器(常驻 RX) -> _downlink_set_alarm 里立即 _try_send_downlink
     *   液位传感器(休眠)   -> 等它上报时 _reply_logic 第二步搭车下发 */
    return 0;
}

lora_slave_t *lora_business_get_slave_list(void)
{
    return g_slave_list;
}

/* 从机台数(运行期动态), 给 main 里的状态打印用 */
uint8_t lora_business_get_slave_count(void)
{
    return g_slave_count;
}

/* ==================== 设备绑定/解绑(P5) 对外 API ====================
 *
 * 绑定窗口状态机的对外入口: lora_business_open_bind_window 由 cloud_proto 的
 * pair_hook 调用(经 main.c 注册); lora_business_set_bind_event_cb 由 main.c
 * 在 init 时注册, 回调里组 event.device JSON 发到云端。
 *
 * 解绑 lora_business_unbind / 解绑的 0x05 重试 + cloud_cmd_unbind 的异步化
 * 在步骤5实现, 这里先把绑定的两个 API 落地。
 */
void lora_business_set_bind_event_cb(lora_bind_event_fn fn)
{
    g_bind_event_cb = fn;
}

bool lora_business_open_bind_window(const char *serial, const char *name,
                                     uint16_t window_s, bool is_sensor,
                                     uint8_t dev_type)
{
    /* 参数校验: 允许 serial/name 为空(用 "" 落盘), 但 window_s 必须在合理区间 */
    if (window_s == 0U || window_s > 300U) return false;
    if (g_bind.state != BIND_IDLE) return false;   /* 已有窗口在进行 */

    /* 落盘用的元数据(serial/name), 截断到数组上限, 空指针当 "" 处理 */
    if (serial != NULL)
    {
        strncpy(g_bind.expected_serial, serial, CFG_SERIAL_MAX);
        g_bind.expected_serial[CFG_SERIAL_MAX] = '\0';
    }
    else
    {
        g_bind.expected_serial[0] = '\0';
    }
    if (name != NULL)
    {
        strncpy(g_bind.expected_name, name, CFG_NAME_MAX);
        g_bind.expected_name[CFG_NAME_MAX] = '\0';
    }
    else
    {
        g_bind.expected_name[0] = '\0';
    }

    g_bind.is_sensor       = is_sensor;
    g_bind.dev_type        = dev_type;      /* 第 11 条: dev.bind 的类型, 绑定时写进 devices[].type */
    g_bind.deadline_tick   = get_tick_ms() + (uint32_t)window_s * 1000U;
    g_bind.last_ack_tx_tick = 0U;
    memset(g_bind.pending_uid, 0, sizeof(g_bind.pending_uid));
    g_bind.state           = BIND_LISTENING;

    log_info("[BIND] 开放绑定窗口: window=%us, type=%s, serial=%s, name=%s\r\n",
             (unsigned)window_s,
             is_sensor ? "sensor" : "device",
             g_bind.expected_serial[0] ? g_bind.expected_serial : "(none)",
             g_bind.expected_name[0]    ? g_bind.expected_name    : "(none)");
    return true;
}

/* ==================== 解绑(P5, LoRa 0x05, 两阶段状态机) ====================
 *
 * 【异步】接口: 收到调用立即进入阶段1, 不等结果。真正删 config_store + 从机表
 *   的动作在阶段2 成功路径由 _unbind_logic 调 _unbind_finalize 完成; 失败/故障
 *   路径只清从机状态(记录保留) + 上报事件, 由上层据此重试或人工介入。
 *
 * 【两阶段流程】(详见 g_unbind 上方注释 + lora_business.h 常量):
 *   阶段1(30s): 每 3s 发 0x05, 收到 ack 立即停发 → 等 15s → 阶段2
 *     30s 没收到 ack → 上报 UNBIND_FAILED(从机记录保留)
 *   阶段2(10s, 最多循环3次): 每 3s 发 0x05
 *     10s 没收到 ack → 解绑成功 → 删表 + 上报 UNBOUND
 *     收到 ack → 设备没清 → 循环阶段2(不再等 15s)
 *     3 次都收到 ack → 设备故障 → 上报 UNBIND_FAULT(从机记录保留)
 *
 * 0x05 帧【不进 data_cache】: 走 pending_func=0x05 直接发, 与正常下发隔离。
 *
 * 防重: g_unbind 同一时刻只跟踪一台设备, 已在解绑 → -3。
 *
 * 返回: 0=已启动; -1=uid 非法/找不到; -2=从机表无此设备; -3=已有解绑在进行中。 */
int lora_business_unbind(const char *uid_hex)
{
    if (uid_hex == NULL) return -1;

    uint8_t uid[CFG_UID_BYTES];
    if (!cfg_uid_to_bytes(uid_hex, uid)) return -1;

    uint8_t idx = _find_slave(uid);
    if (idx == 0xFFU) return -2;

    /* 防重: g_unbind 已在跟踪(同一台或另一台) → 拒绝, 避免空口噪声 */
    if (g_unbind.state != UNBIND_IDLE)
    {
        log_info("[UNBIND] !! 已有解绑在进行中(state=%u, %02X%02X%02X), 拒绝重复发起\r\n",
                 (unsigned)g_unbind.state,
                 g_unbind.uid[0], g_unbind.uid[1], g_unbind.uid[2]);
        return -3;
    }

    lora_slave_t *s = &g_slave_list[idx];
    bool    is_sensor = (s->dev_type == SLAVE_TYPE_SENSOR);
    uint32_t now      = get_tick_ms();

    /* 1) 把目标 slave 置 WAIT_ACK+pending_func=0x05 当"锁":
     *    _handle_down_logic 看到 pending_func==0x05 直接 continue 跳过(不干扰),
     *    handle_ack_from_device 顶部把 ack 路由到 _unbind_on_ack。 */
    s->pending_func     = LORA_FUNC_UNBIND;
    s->pending_data_len = 0U;          /* 0x05 帧无数据域 */
    s->retry_cnt        = 0U;
    s->state            = SLAVE_STATE_WAIT_ACK;
    s->send_req_tick    = now;

    /* 2) 启动 g_unbind 状态机(阶段1); last_send_tick 下面 _unbind_send_frame
     *    会再刷新一次, 这里先置 now 防止首次 3s 间隔检查立刻又发一帧。 */
    g_unbind.state            = UNBIND_PHASE1;
    memcpy(g_unbind.uid, uid, CFG_UID_BYTES);
    g_unbind.is_sensor        = is_sensor;
    g_unbind.slave_idx        = idx;
    g_unbind.phase_start_tick = now;
    g_unbind.last_send_tick   = now;
    g_unbind.phase2_retry     = 0U;

    log_info("[UNBIND] 向 %s(%02X%02X%02X) 发起 0x05 解绑, 进入阶段1(30s, 每3s发, type=%s)\r\n",
             uid_hex, uid[0], uid[1], uid[2],
             is_sensor ? "sensor" : "device");

    /* 3) 立即发首帧 0x05(TX 失败也无所谓, 状态机 3s 后会再试) */
    _unbind_send_frame();
    return 0;
}

/* ============================================================================
 * 档位联动下发 —— 边缘自治的"最后一跳" (2026-09-30, 用户第 2 条)
 * ----------------------------------------------------------------------------
 * 【为什么要有它】
 *   文档第 4 章的自治核心: 评估 -> 档位 -> 按 gears[] 生成动作 -> 经 LoRa 下发
 *   联动设备。前四步早就做完并单测过了(level_eval.c / level_supervisor.c),
 *   但【下发】这一步一直空着 —— 于是:
 *     · gears[] 的 targets/soundMode/ledText/ttsText 全是死配置;
 *     · 唯一真正到设备的 0x03 是"把云端原始字节原样转发" -> 断网就不联动。
 *   本函数就是补上那一跳。
 *
 * 【本文件只做"取 uid + 投递", 组帧不在这里】
 *   组帧(9 字节头 + 文本)与"选哪些目标设备"是【纯逻辑】, 放在 level_eval.c 的
 *   level_gear_build_payload() / level_gear_collect_targets() —— 那样能在主机上
 *   跑单测, 而且"待声光设备协议确认"的布局只集中在一个函数里, 设备侧定稿后
 *   只改那一处。
 *
 * 【复用了什么(没另造轮子)】
 *   lora_business_send_req() —— 已有的"投递到下发缓存"入口: 负责找从机 ->
 *   写 pending_func/pending_data -> 由 poll 里的 _try_send_alarm_idle()(联动设备
 *   常驻 RX, 立即发) / _handle_down_logic()(5s 超时重发、最多 4 次、耗尽丢弃)
 *   完成发送与 ACK 跟踪。所以 ACK、重试、窗口适配一行都不用重写。
 *
 * 【时序为什么要守】(网关液位lora规则.txt, 液位侧口径)
 *   液位上报后【立即开 5s 接收窗口】等主机 ack; 收到 ack 再开【10s 窗口】收云端
 *   控制指令; 收到指令后回【两遍 ack(间隔 3s)】再休眠。
 *   -> 本条链只对【联动设备】(常驻 RX, 随时可发), 不涉及液位窗口; 液位那侧的下发
 *      仍走 _reply_logic() 的 3s+3s 两步错开(落在 5s/10s 窗口内)。
 *      设备回的第二遍 ack 会落在 state=IDLE, handle_ack_from_device() 直接忽略
 *      (不是等 ACK 状态) —— 这是安全的, 不需要特判。
 * ========================================================================== */

/* 只对【声光/联动设备】下发 */
#define GEAR_TARGET_TYPE   SLAVE_TYPE_ALARM_OUT

/* 诊断计数(给状态行/排障用) */
static uint16_t g_gear_tx_ok;            /* 成功投递到缓存的目标台数 */
static uint16_t g_gear_tx_skip;          /* 跳过/投递失败的台数 */
static uint8_t  g_last_gear_tx = 0xFFU;  /* 上次真正下发过的档位 */

/**
 * @brief 档位联动下发(最后一跳的入口)
 * @param gear     目标档位 0~3
 * @param level_mm 当前有效水位(mm, 只用于日志)
 * @param led_text 渲染后的 LED 文本(可 NULL/空)
 * @param tts_text 渲染后的语音文本(可 NULL/空)
 * @param out_total 可选输出: 目标设备总台数(区分"没设备"和"发失败")
 * @return >=0 成功投递的设备台数; -1 参数非法/档位越界
 * @note  只投递到下发缓存, 不阻塞; 空口发送由 lora_business_poll() 完成。
 */
/* ==================== ★2026-10-10: dev.ctrl 的本次下发样式覆盖 ====================
 * 三个静态: 登记区(g_style_valid/g_style) + 组装用的临时档位副本(g_style_tmp)。
 * 【为什么是静态】cfg_gear_t 约 250 字节, 本工程栈只有 4KB, 放栈上会吃掉一大块;
 *   本工程单线程、apply_gear_ex 不重入, 用静态是安全的(同 g_alarm_gear_inflight 先例)。
 * 【为什么副本】绝不能改 cfg->gears[] —— 那是持久配置, 改了就"临时覆盖"变成"永久改了"。 */
static uint8_t    g_style_valid;
static cfg_gear_t g_style;
static cfg_gear_t g_style_tmp;

void lora_business_set_gear_style(uint8_t valid_mask, uint8_t show_mode,
                                  uint8_t text_mode_h1, uint8_t text_mode_h2,
                                  uint8_t color_h1, uint8_t color_h2)
{
    g_style_valid                 = valid_mask;
    g_style.led_show_mode         = show_mode;
    g_style.led_text_show_mode[0] = text_mode_h1;
    g_style.led_text_show_mode[1] = text_mode_h2;
    g_style.led_color[0]          = (cfg_led_color_t)color_h1;
    g_style.led_color[1]          = (cfg_led_color_t)color_h2;
}

void lora_business_clear_gear_style(void)
{
    g_style_valid = 0u;
}

/** 取"本台实际要用"的档位样式: 有覆盖就返回改过的副本, 否则直接返回配置 */
static const cfg_gear_t *gear_style_effective(const config_store_t *cfg, uint8_t gear)
{
    if (g_style_valid == 0u)
    {
        return &cfg->gears[gear];
    }

    g_style_tmp = cfg->gears[gear];          /* 整份拷贝, 再按 mask 覆盖 */

    if ((g_style_valid & LORA_STYLE_SHOW_MODE) != 0u)
    {
        g_style_tmp.led_show_mode = g_style.led_show_mode;
    }
    if ((g_style_valid & LORA_STYLE_TEXT_MODE) != 0u)
    {
        g_style_tmp.led_text_show_mode[0] = g_style.led_text_show_mode[0];
        g_style_tmp.led_text_show_mode[1] = g_style.led_text_show_mode[1];
    }
    if ((g_style_valid & LORA_STYLE_COLOR) != 0u)
    {
        g_style_tmp.led_color[0] = g_style.led_color[0];
        g_style_tmp.led_color[1] = g_style.led_color[1];
    }
    return &g_style_tmp;
}

int lora_business_apply_gear_ex(uint8_t gear, const char *only_uid,
                                const char *led_text1, const char *led_text2, const char *tts_text,
                                lora_gear_result_t *res, uint8_t res_cap,
                                uint8_t *out_total)
{
    const config_store_t *cfg = config_store_get();
    uint8_t idx[CFG_DEVICE_MAX];
    uint8_t n_targets;
    uint8_t sent = 0U;
    uint8_t i;

    if (cfg == NULL) { return -1; }
    if (gear >= (uint8_t)CFG_GEAR_COUNT) { return -1; }

    /* (1) 选目标: only_uid==NULL -> 按 gears[].targets(自动联动);
     *             only_uid!=NULL -> 只这一台(dev.ctrl 手动控制) */
    n_targets = level_gear_collect_targets(cfg, gear, only_uid,
                                           idx, (uint8_t)CFG_DEVICE_MAX);
    if (out_total != NULL) { *out_total = n_targets; }

    /* (2) 逐台组帧 + 投递 */
    for (i = 0U; (i < n_targets) && (i < (uint8_t)CFG_DEVICE_MAX); i++)
    {
        const cfg_device_t *d = &cfg->devices[idx[i]];
        uint8_t uid3[CFG_UID_BYTES];
        uint8_t payload[LEVEL_GEAR_PAYLOAD_MAX];
        uint8_t plen;
        uint16_t code = 0U;          /* ★ uint16: 错误码是 1003/1007 三位数 */
        int     r;

        if (!cfg_uid_to_bytes(d->uid, uid3))
        {
            g_gear_tx_skip++;
            code = 1007U;
            log_info("[GEAR] !! devices[%u].uid=%s 不是 26 位 hex 串, 跳过\r\n",
                     (unsigned)idx[i], d->uid);
        }
        else
        {
            /* 每台的音量可能不同, 所以载荷要逐台组(文本部分是一样的)
             * ★2026-10-10: 样式(ledShowMode/ledTextShowMode/ledColor)走
             *   gear_style_effective() —— 有 dev.ctrl 覆盖就用改过的副本, 否则用配置。*/
            plen = level_gear_build_payload(gear_style_effective(cfg, gear), d->volume,
                                            led_text1, led_text2, tts_text,
                                            payload, (uint16_t)sizeof(payload));
            if (plen == 0U)
            {
                g_gear_tx_skip++;
                code = 1007U;
                log_info("[GEAR] !! 档位 %u 给 %s 组帧失败(缓冲不够)\r\n",
                         (unsigned)gear, d->uid);
            }
            else
            {
                /* ★2026-10-08: 记下这条动作的档位★
                 * _transmit_downlink 只知道 uid/func/data, 不知道档位(档位不在空口载荷里),
                 * 而上报 event.action 需要它。单线程 + 入队到发出只隔一个 poll 周期, 安全。 */
                g_alarm_gear_inflight = gear;

                r = lora_business_send_req(uid3, GEAR_TARGET_TYPE,
                                           LORA_FUNC_SET_ALARM, payload, plen);
                if (r == 0)
                {
                    sent++;
                    g_gear_tx_ok++;
                }
                else
                {
                    g_gear_tx_skip++;
                    /* -1 载荷过长(不太可能: 组帧已按 63 上限截过) / -2 从机表里没有它 */
                    code = (r == -1) ? 1007U : 1003U;
                    log_info("[GEAR] !! 档位 %u 投递给 %s 失败(r=%d: %s)\r\n",
                             (unsigned)gear, d->uid, r,
                             (r == -1) ? "载荷过长" : "从机表里没有它(未绑定?)");
                }
            }
        }

        if ((res != NULL) && (i < res_cap))
        {
            (void)snprintf(res[i].uid, sizeof(res[i].uid), "%s", d->uid);
            res[i].code = code;
        }
    }

    if (sent > 0U)
    {
        g_last_gear_tx = gear;
        log_info("[GEAR] 档位 %u 已投递 %u/%u 台联动设备%s(LED1=\"%s\" LED2=\"%s\")\r\n",
                 (unsigned)gear, (unsigned)sent, (unsigned)n_targets,
                 (only_uid != NULL) ? "(手动控制)" : "",
                 (led_text1 != NULL) ? led_text1 : "",
                 (led_text2 != NULL) ? led_text2 : "");
    }
    else
    {
        log_info("[GEAR] 档位 %u 没有可投递的联动设备(启用 %u 台)%s\r\n",
                 (unsigned)gear, (unsigned)n_targets,
                 (only_uid != NULL) ? " 或指定的那台不存在/已停用" : "");
    }

    return (int)sent;
}

int lora_business_apply_gear(uint8_t gear, int32_t level_mm,
                             const char *led_text1, const char *led_text2, const char *tts_text,
                             uint8_t *out_total)
{
    int sent = lora_business_apply_gear_ex(gear, NULL, led_text1, led_text2, tts_text,
                                           NULL, 0U, out_total);

    if (sent > 0)
    {
        log_info("[GEAR] (自动联动) 水位 %ldmm -> 档位 %u\r\n",
                 (long)level_mm, (unsigned)gear);
    }
    return sent;
}

/** 上次成功下发过的档位(0~3); 0xFF = 还没有下发过 */
uint8_t lora_business_last_gear(void)
{
    return g_last_gear_tx;
}

/** 联动下发计数: ok = 成功投递台数, skip = 跳过/失败台数 */
void lora_business_gear_stats(uint16_t *ok, uint16_t *skip)
{
    if (ok   != NULL) { *ok   = g_gear_tx_ok; }
    if (skip != NULL) { *skip = g_gear_tx_skip; }
}

/* ==================== LoRa 射频健康状态 ====================
 * 实现在 app/drivers/lora/LLCC68N32H482-board.c(射频中间层)。这里用 extern 直接声明, 免得为了
 * 两个查状态的函数把射频层的头文件引进业务层。 */
extern uint8_t LLCC68IsStuck(void);
extern uint8_t LLCC68BusyLevel(void);

uint8_t lora_business_radio_stuck(void)
{
    return LLCC68IsStuck();
}

uint8_t lora_business_radio_busy(void)
{
    return LLCC68BusyLevel();
}
