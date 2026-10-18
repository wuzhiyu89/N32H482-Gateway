/**
 * \file  lora_business.c
 * \brief LoRa 业务层 —— 从机表 + 帧分发 + ACK 重传 + data_cache + MQTT 下行
 *
 * 本模块吸收了原 local_gateway.c 的全部业务逻辑:
 *   - lora_gateway_find_slave_by_uid → lora_slave_find (★ 从机表为动态表, 从
 *     config_store 的 sensors[]/devices[] 装载, 运行期支持 0x04 绑定/0x05 解绑)
 *   - lora_gateway_handle_up_frame → on_frame (协议层已做 CRC, 只做业务分发)
 *   - lora_gateway_handle_down_logic → lora_downlink_logic (ACK 超时重传)
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

/* 从机表(g_slave_list / g_slave_count)已拆到 lora_slave.c，
 * 接口见 lora_internal.h。本文件只做帧分发 / 绑定解绑 / 下发，不再维护表结构。 */
static uint32_t g_last_poll_tick;
/* 网关 UID 在 lora_business_init 里从 param_save 读入 (支持后续本地/云端修改) */
uint8_t g_gw_uid[CFG_UID_BYTES];

/* ========== 解绑两阶段状态机(2026-09-30 用户最终口径) ==========
 *
 * 【为什么 g_unbind 完全独立、不复用从机 pending_func 重传逻辑】
 *   原来的解绑走的是从机表的 pending_func=0x05 + lora_downlink_logic 5s 重试/3 次
 *   耗尽强制删, 那套口径是"等不到 ACK 就强制删设备" —— 与用户最终口径冲突:
 *     - 阶段1 等不到 ack 是【失败上报】, 不删设备
 *     - 阶段2 收不到 ack 才是【成功】, 收到 ack 才是失败
 *   而且阶段切换(等15s / 阶段2 循环)用从机 pending_func 表达不了。
 *   所以 g_unbind 单独维护状态机, 只把从机表的 state/pending_func 当"锁":
 *   启动时把目标 slave 置 WAIT_ACK+pending_func=0x05, lora_downlink_logic 看到
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

/* ========== 内部: 解绑收尾(P5, LoRa 0x05) ==========
 *
 * 在两处被调用:
 *   1) handle_ack_from_device: 设备回 ACK, 正常解绑成功
 *   2) lora_downlink_logic 重试耗尽: 用户口径"重试耗尽也强制删除"
 *      (设备可能已经离线/掉电, 不能因为它不回 ACK 就永远删不掉)
 *
 * 【调用方必须在调本函数前清掉自己的 s->state/pending_func】,
 * 本函数只负责 config_store + 运行期表 + 事件, 不碰 s->state(因为
 * lora_slave_del 会把末尾从机搬到当前格, s 已经不一定指向原来了)。
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
    (void)lora_slave_del(uid);
    lora_bind_fire_event(uid_hex, is_sensor, LORA_BIND_EVENT_UNBOUND);
    return true;
}

/* ========== 内部: 下行指令处理 ========== */

/**
 * \brief 组装 lora_frame_t 并发送 (用协议层 lora_proto_send_frame)
 * \return true 成功提交
 */
/* ========== 解绑两阶段状态机 helpers(2026-09-30 用户最终口径) ==========
 *
 * 6 个 helper 配合 g_unbind 状态机:
 *   _unbind_send_frame()        —— 用从机表里目标 slave 的 pending_func=0x05 发一帧
 *   _unbind_fire_event()        —— 把 g_unbind.uid 转 hex + 调 lora_bind_fire_event 上报事件
 *   _unbind_clear_slave_state() —— 清从机表里目标格的 state/pending_func(不删条目)
 *   _unbind_reset()             —— 清 g_unbind 回 IDLE(释放锁)
 *   _unbind_on_ack()            —— 收到设备 ack 时按 state 处理
 *   _unbind_logic()             —— 周期驱动(挂在 lora_business_poll)
 *
 * 【锁的语义】解绑期间目标 slave 一直是 WAIT_ACK+pending_func=0x05:
 *   - lora_downlink_logic 看到 pending_func==0x05 直接 continue 跳过(不干扰)
 *   - handle_ack_from_device 顶部拦截 g_unbind.state!=IDLE 且 slave_idx 匹配的 ack,
 *     路由到 _unbind_on_ack, 不走旧逻辑
 *   - 结束(成功/失败/故障)都清回 IDLE+pending_func=0 释放锁
 *
 * ★ 成功路径(_unbind_finalize 调 lora_slave_del 会让 slave_idx 失效)必须
 *   先拷贝 uid/is_sensor 到本地, 再 _unbind_reset, 最后 _unbind_finalize。
 *   失败/故障路径不删从机记录, 只清 slave 状态 + _unbind_reset + 上报事件。 */

/* 用从机表里目标 slave 的 pending_func=0x05 发一帧 0x05。
 * pending_func/pending_data_len 在 lora_business_unbind 启动时已设好,
 * 这里只负责发送 + 刷新时间戳。失败(TX 忙)也更新 last_send_tick, 3s 后再试。 */
static void _unbind_send_frame(void)
{
    if (g_unbind.slave_idx >= g_slave_count) return;
    lora_slave_t *s = &g_slave_list[g_unbind.slave_idx];
    (void)lora_tx_downlink(g_unbind.slave_idx);
    uint32_t now              = get_tick_ms();
    s->send_req_tick          = now;
    g_unbind.last_send_tick   = now;
}

/* 把 g_unbind.uid 转 hex + 调 lora_bind_fire_event 上报事件 */
static void _unbind_fire_event(lora_bind_event_t event)
{
    char uid_hex[CFG_UID_STR_MAX];
    /* cfg_uid_from_bytes 把 13 字节转 hex, 不会失败(非全 0 即有效) */
    (void)cfg_uid_from_bytes(g_unbind.uid, uid_hex);
    lora_bind_fire_event(uid_hex, g_unbind.is_sensor, event);
}

/* 清从机表里目标 slave 的 state/pending_func —— 不删条目(失败/故障路径用,
 * 从机记录保留); 成功路径走 _unbind_finalize 的 lora_slave_del 删条目。
 * 校验 uid 匹配: 防止 lora_slave_del/add 改了布局后索引错位误清别的从机。 */
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
             * 最后 _unbind_finalize —— 它会 lora_slave_del 让 slave_idx 失效,
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

/* 声光的超时/次数/间隔宏定义在文件前半(见 lora_tx_downlink 之前),
 * 因为发送咽喉也要用它们 —— 这里不再重复定义。 */

static void handle_ack_from_device(const lora_frame_t *frame, uint8_t slave_idx)
{
    /* 【g_unbind 拦截】解绑期间目标 slave 的 ack 由 g_unbind 状态机接管,
     *   不走下面的 data_cache / lora_reply_schedule 旧逻辑。
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
                lora_alarm_fire_result(s, true, false, was_offline);
            }
            return;
        }
        if (frame->func == (uint8_t)LORA_FUNC_ALARM_FAIL)       /* 0x83: 失败 */
        {
            log_info("[GEAR] %02X%02X%02X 回执行失败(0x83), 第 %u/3 条\r\n",
                     s->dev_uid[0], s->dev_uid[1], s->dev_uid[2],
                     (unsigned)s->alarm_attempts);
            lora_alarm_on_fail(s);
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
    /* 如果是 ERR —— 保留缓存, 让下次设备上报时 lora_try_send_downlink 再发 */
}

/* ========== 业务帧分发 (注册给协议层的 LoraFrameCallback_t) ========== */


/* ==================================================================
 * ★2026-10-18: UID 尺寸【静态断言】—— 防止"改了长度忘改缓冲"再次发生
 * ==================================================================
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
_Static_assert(sizeof(g_unbind.uid)          == CFG_UID_BYTES, "g_unbind.uid 必须 13 字节");
/* (tc_record_t / ec_record_t 的断言放在各自的 .c 里 —— 本文件不 include 那两个头) */

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
     * 必须在 lora_slave_touch / 白名单之前 —— 0x04 绑定请求的目标地址是广播
     * FFFFFF, dev_uid 是发起绑定的设备自己, 它还不在从机表里, 会被白名单
     * 丢弃。这里先拦下来走 lora_bind_handle_0x04, 由状态机决定是回 0x80 进入
     * 握手还是回 0x81 拒绝。
     *
     * 同样, 在 BIND_WAIT_DEV_ACK 期间, 对端回的 0x80 是"对 0x04 的确认",
     * 不能再走 handle_ack_from_device(那个是处理"对 0x01/0x03 下发的 ACK")。
     */
    if (lora_bind_accepts_0x04() && (frame->func == LORA_FUNC_BIND_REQ))
    {
        lora_bind_handle_0x04(frame);
        return;
    }
    if ((frame->func == LORA_FUNC_RESP_OK) && lora_bind_waiting_dev_ack(frame->dev_uid))
    {
        lora_bind_handle_dev_ack(frame);
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
     *     1629 行: lora_slave_touch() 摸一下 -> 如果这台设备【还在从机表里】, idx 有效
     *     1648 行: 通用 ACK 分支对"除 0x80/0x81 之外的所有 func"都回 ACK
     *   ⇒ 0x04 就搭上了这趟车。而 0x04 的语义是"我想加入", 不是一个业务请求 ——
     *     它【只】该由配对窗口状态机应答; 窗口没开就代表"没人在等它"。
     *
     * 【为什么必须早于 lora_slave_touch 拦】lora_slave_touch 的副作用是"标记在线 + 刷新时间戳",
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
    uint8_t idx = lora_slave_touch(frame->dev_uid);

    /* 【白名单】: 地址不在从机表里 -> 只打印, 一律丢弃。
     * 不回 ACK、不解析载荷、不入缓存 —— 陌生设备在这一层就被挡住。
     * ⚠ 0x04 绑定请求是特例(目标地址=广播 FFFFFF, dev_uid 是发起绑定的设备自己,
     *   不在表里) —— 步骤4 要在 lora_slave_touch 之前先判 func==0x04 且在绑定窗口,
     *   直接走 0x04 分支, 否则会被这里白名单丢弃。 */
    if (idx == 0xFFU)
    {
        log_info("[GW] !! 未知从机 %02X%02X%02X (不在从机表里), 丢弃该帧\r\n",
                 frame->dev_uid[0], frame->dev_uid[1], frame->dev_uid[2]);
        return;
    }

    /* 【不再立刻回】只登记, 由 lora_reply_logic() 等 3s 后回 ACK, 再等 3s 下发指令。
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
        lora_reply_schedule(frame->dev_uid, frame->func, idx);
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
        /* 未知/暂不处理的功能码 —— ACK 已由 lora_reply_logic() 排上。
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

    /* 注意: 这里【不再】调 lora_try_send_downlink()。
     * 下发改由 lora_reply_logic() 的第二步做(回完 ACK 再等 3s),
     * 这样"回 ACK"和"下发指令"两条时间线不会各算各的。 */
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
    lora_slave_load();

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
    lora_reply_logic();

    /* 1.6 绑定窗口状态机: 每轮都判(1ms 精度), 负责重发 0x80 + 窗口到期处理。
     *     只在非 IDLE 时有动作, 平常(没开窗口)立即 return, 不影响主循环节拍。 */
    lora_bind_logic();

    /* 1.7 解绑两阶段状态机: 每轮都判(1ms 精度), 负责阶段1/等15s/阶段2 的
     *     3s 间隔发 0x05 + 阶段超时/成功/故障处理。只在非 IDLE 时有动作,
     *     平常立即 return, 不影响主循环节拍。 */
    _unbind_logic();

    /* 2. ACK 超时重传 + 声光报警器主动下发 (周期轮询) */
    uint32_t now = get_tick_ms();
    if ((now - g_last_poll_tick) >= LORA_POLL_PERIOD_MS)
    {
        g_last_poll_tick = now;
        lora_downlink_logic();       /* 重传超时的 WAIT_ACK 从机 */
        lora_try_send_alarm_idle();      /* 扫 IDLE 声光报警器, 捡起缓存的待发指令 */
        /* ★2026-10-08: 这里原来还调 data_cache_poll()(发 mod/mod_upload 那条 hex 上行)。
         * 该通道已按甲方要求整体删除, 调用一并去掉 —— 液位走的是 last_value 快照
         * + main.c 的 rt/gw/{gwId}/telemetry, 与本处无关。 */
    }
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

    uint8_t idx = lora_slave_find(uid);
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
     *    lora_downlink_logic 看到 pending_func==0x05 直接 continue 跳过(不干扰),
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
