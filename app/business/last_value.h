#ifndef __LAST_VALUE_H__
#define __LAST_VALUE_H__

#include <stdint.h>
#include <stdbool.h>

/* ==========================================================================
 * 设备"最后值"快照表
 * ==========================================================================
 * 【为什么需要它】
 *   `sen.query` / `dev.query` 要回每台设备"最近一次上报的水位/电量/时间"。
 *   而 `app/business/data_cache.c` 里那个上行缓存是**FIFO 上传队列**:
 *   一旦 MQTT 发出去就 drop 了, 之后按 uid 根本查不到历史值。
 *   所以这里另存一张**按 uid 覆盖**的快照表(FIFO 出队不影响它)。
 *
 * 【为什么单独一个模块, 不直接写进 data_cache.c】
 *   `data_cache.c` 依赖 g4_at.h / lora_business.h / bsp_tim.h, 在主机上编不了,
 *   进去就没法做离线单测。本模块**零依赖**(只用 stdint/stdbool/string.h),
 *   主机可测; `data_cache.c` 只在 `data_cache_up_push()` 里加一行 update 调用。
 *   这也是 `level_eval.c` 用的同一套办法。
 *
 * 【语义与边界】
 *   - 按 uid 覆盖: 同一设备再上报就更新那一槽, 不清空其它设备;
 *   - 槽满: 淘汰 `rx_tick_ms` **最旧**的一槽(和上行队列"满则丢最旧"一致),
 *     并累加内部分隔计数, 可用 last_value_evict_count() 观察, 不静默丢;
 *   - **纯内存, 断电即失**。文档要求的"断电不丢"属于事件缓存那件事, 不在这里,
 *     不要误以为本表能满足掉电续传。
 *
 * 【时间口径(重要, 别搞混)】
 *   `time_mdhhm` 是**设备端**给的 MM.DD.HH.MM(无年份), `rx_tick_ms` 是**网关本地**
 *   tick(非 Unix 秒)。本模块**不做**两者到 Unix 秒的换算 —— 换算需要网关 unix 时钟,
 *   放在渲染/协议层做(`cloud_proto` 侧有 `main_now_unix()` 可用)。
 *   本模块只忠实保存"设备说的时间"和"网关什么时候收到的"。
 * ========================================================================== */

/* 容量: 传感器上限 CFG_SENSOR_MAX=5 + 声光联动设备 + 裕量 */
#define LAST_VALUE_MAX   8U

typedef struct
{
    bool     used;          /* 槽位是否有效 */
    uint8_t  uid[13];        /* 设备 13 字节 uid */
    uint16_t level_mm;      /* 最近水位(mm) —— 与 level_eval 同口径 */
    uint16_t bat_pct;       /* 电量百分比。★注意: 上游参数名叫 bat_mv 但传的是百分比
                             *   (lora_business.c 里 `bat_pct`), 文档口径也是百分比 */
    uint32_t time_mdhhm;    /* 设备端 MM.DD.HH.MM (原始保留, 本模块不解释) */
    int8_t   rssi_dbm;      /* ★ 该帧的 LoRa RSSI(dBm) ★
                             * 2026-09-30 第 10 条新增: telemetry.report 的
                             * data.sensors[].rssi 要用它(文档 7.1 有该字段)。
                             * 以前只打日志没存, 遥测里只能填 0。
                             * 0 = 未知/调用方没带(旧路径用 last_value_update() 时即此值)。 */
    uint32_t rx_tick_ms;    /* 网关收到时刻的本地 tick */
    uint32_t rx_unix;       /* 网关收到时刻的 Unix 秒; 未注册时钟钩子时为 0(不猜) */
    uint16_t rx_count;      /* 该设备累计上报次数(可用于"从没上报过"判断) */
} last_value_t;

/* 清空全表(初始化 / 恢复出厂设置时调用) */
void last_value_reset(void);

/**
 * @brief 丢掉【某一个 UID】的快照 —— 重绑设备时用（2026-10-14 新增）
 * @param uid 13 字节设备 uid, 不可为 NULL
 * @return 清掉的槽数: 0 = 该 UID 本来就没有快照(不算错); 1 = 清了一条
 * @note  【为什么要它】用户场景: 设备坏了换新, 重新烧录后回到同一默认地址
 *        ⇒ 允许同 UID 再绑一次(从机表不新增), 但【新设备不该继承旧设备的历史值】:
 *          否则新设备还没上报时, sen.query 就回旧设备的水位; level_supervisor
 *          会用旧水位算档位(新设备一上电就按旧水位点灯); 失联判定也可能因
 *          "缓存里的 rx_tick_ms 很新"而误判在线。
 *        ⚠️【不要】图省事改用 last_value_reset() —— 那是【全清】, 重绑一台会把
 *           其它所有传感器的快照一起清掉(现场表现: 重绑一台, 其它台全掉线)。
 *        ⚠️【不要】用 config_store_unbind_device() 走"先解绑再绑" —— 它会连带
 *           清掉 gears[].target_uid 里指向该 UID 的条目(见 config_store.h),
 *           那台设备以后收不到联动指令(能绑定、能上报, 就是不动灯)。
 */
uint8_t last_value_clear_uid(const uint8_t *uid);

/**
 * @brief 注册"网关当前 Unix 秒"的取值函数(由 main.c 注册 main_now_unix)
 * @param fn 取值函数; 传 NULL 表示撤销注册
 * @note  不注册时 last_value_update() 把 rx_unix 记 0 —— 应答里就【不输出】
 *        该时间字段。宁可没有时间戳, 也不编一个出来(0 会被解读成 1970)。
 *        之所以用钩子而不是直接调 main.c: 本模块要能在主机上脱机单测。
 */
void last_value_set_now_hook(uint32_t (*fn)(void));

/**
 * @brief 记录/更新某设备的最后值(内部按 uid 覆盖)
 * @param uid        13 字节设备 uid, 不可为 NULL
 * @param level_mm   水位(mm)
 * @param bat_pct    电量百分比 0~100
 * @param time_mdhhm 设备端时间(MD.DD.HH.MM 原样保存)
 * @param now_ms     网关本地 tick(调用方给, 便于单测; 不传时钟进来)
 * @return true 已写入; false 参数非法(uid==NULL)
 * @note  槽满时淘汰 rx_tick_ms 最旧的一槽, 仍返回 true。
 */
bool last_value_update(const uint8_t *uid, uint16_t level_mm, uint16_t bat_pct,
                       uint32_t time_mdhhm, uint32_t now_ms);

/**
 * @brief 同上, 但额外记录该帧的 LoRa RSSI(第 10 条: 遥测要用)
 * @param rssi_dbm 该帧的信号强度(dBm); 0 = 未知
 * @note  为什么保留一个不带 rssi 的 last_value_update(): 它被大量既有调用点与
 *        60 条单测用着, 而那些场景根本不关心 rssi。让它们继续调用旧入口(内部等价于
 *        rssi_dbm=0), 比逼所有调用点改签名要安全 —— 这是"新增能力"而不是"改语义"。
 */
bool last_value_update_ex(const uint8_t *uid, uint16_t level_mm, uint16_t bat_pct,
                          uint32_t time_mdhhm, int8_t rssi_dbm, uint32_t now_ms);

/**
 * @brief 按 uid 查最后值
 * @return true 找到(out 被填充); false 该 uid 从没上报过(out 不动)
 */
bool last_value_get(const uint8_t *uid, last_value_t *out);

/**
 * @brief 按"有效槽序号"取第 idx 条(0 起, 升序), 供 dev.query("all") 遍历
 * @return true 取到; false idx 越界
 * @note  序号是**有效槽**的稠密序号, 不是裸槽位下标 —— 遍历用
 *        `for (i = 0; i < last_value_count(); i++) last_value_at(i, &v);`
 */
bool last_value_at(uint8_t idx, last_value_t *out);

/* 当前有效槽数(0..LAST_VALUE_MAX) */
uint8_t last_value_count(void);

/* 因槽满被淘汰的更新次数(累计, 仅重启清零) —— 非 0 说明容量不够, 该调大 LAST_VALUE_MAX */
uint32_t last_value_evict_count(void);

/**
 * @brief 累计成功 update 的次数(单调递增, reset 也不清零)
 * @note  ★ 这是"有没有新上报"的判据, 给 level_supervisor 用 ★
 *        confirmCount 防抖是"连续几次【上报】一致", 必须按上报计数;
 *        若按主循环轮次喂, 几微秒就能攒够次数, 防抖形同虚设。
 *        刻意不被 last_value_reset() 清零: 清零会让"喂到哪了"的记录失去参照,
 *        可能把旧样本误判成新样本(或反过来漏喂一次)。
 */
uint32_t last_value_update_seq(void);

#endif /* __LAST_VALUE_H__ */
