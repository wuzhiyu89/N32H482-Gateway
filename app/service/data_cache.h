#ifndef __DATA_CACHE_H__
#define __DATA_CACHE_H__
#include <stdint.h>
#include <stdbool.h>

/* =========可配置项========= */
#define DATA_CACHE_DOWN_MAX_ITEM    16U     /* 网关下发指令缓存最大条数 */

/* 设备类型 */
typedef enum
{
    SLAVE_TYPE_SENSOR    = 0,   //液位传感器
    SLAVE_TYPE_ALARM_OUT = 1    //联动告警输出设备（声光TTS‑LED）
}slave_type_t;

/**
 * @brief 下发缓存条目：网关待下发给LoRa从机指令
 */
typedef struct
{
    uint8_t         dev_uid[13];         // 目标从机 UID(13 字节: 类型 + 96 位芯片 UID)
    slave_type_t    dev_type;           //设备类型
    uint8_t         payload[211];       /* 原始待下发报文: 1 字节 func + 最多 210 字节载荷
                                         * ★2026-10-08: 64 -> 211★ 联动 0x03 载荷从 63 涨到 210
                                         * (两段 LED + 一段 TTS), 缓冲必须跟着涨, 否则
                                         * data_cache_down_push() 会把整条指令判成"超长"拒收。
                                         * 代价: 16 条 × 147 字节 ≈ 2.4KB RAM。 */
                                         /* (后面会追加CRC16) */
    uint8_t         pay_len;            //报文有效长度，不含CRC2字节
    bool            valid;              //条目是否有效
}cache_down_item_t;

/**
 * @brief 缓存初始化，清空下发缓存 + 最后值快照表
 */
void data_cache_init(void);

/**************** 上行（★2026-10-08 起只剩"最后值快照"这一件事） ****************/
/**
 * @brief 存入一条传感上报记录 —— 只更新按 uid 的"最后值"快照
 *
 * 【历史说明】本函数原来还会往一个上行 FIFO 里塞一份, 由 data_cache_poll() 取出来
 * 交给 lora_business_publish_report() 发成 hex 到 MQTT 主题 mod/mod_upload。
 * 那条通道已按甲方要求整体删除(见 data_cache.c 顶部说明), 所以现在只剩快照更新。
 * 【为什么还叫 data_cache_up_push】它是"所有传感器上报的唯一漏斗", 改名会牵动调用点,
 * 而语义(存入一条上报)并没变。mod/telemetry 的 JSON 就是从这张快照表取的。
 *
 * @param uid 3字节uid(const —— 调用方传的是 const 帧里的字段, 第 10 条订正)
 * @param level_mm 液位(mm)
 * @param bat_mv 电量百分比(形参名是历史遗留)
 * @param time_mdhhm 4字节打包时间 MM.DD.HH.MM
 * @param rssi_dbm 该帧 LoRa RSSI(dBm); 0 = 未知
 */
void data_cache_up_push(const uint8_t *uid, uint16_t level_mm, uint16_t bat_mv,
                        uint32_t time_mdhhm, int8_t rssi_dbm);

/**************** 下发缓存API（网关下发给从机指令） ****************/
/**
 * @brief 存入一条待下发指令
 * @param uid 3字节uid
 * @param buf 待下发载荷，不含CRC
 * @param len 载荷长度N，不含CRC
 * @return 0成功；‑1缓存满
 */
int data_cache_down_push(uint8_t *uid, slave_type_t dev_type, uint8_t *buf, uint8_t len);
bool data_cache_down_fetch_by_uid(uint8_t *uid, cache_down_item_t *out_item);
void data_cache_down_del_by_uid(uint8_t *uid);

/**
 * @brief 只有"缓存里那条正是刚发出去的这一条"才删除(条件删除)
 *
 * 用途: 声光 0x03 是三段式(0x80 收到 -> 约 10s 后 0x82 结果)。等待期间云端可能
 * 又下发了一条新指令覆盖同一 uid 的缓存 —— 这时【不能】按 uid 无条件删,
 * 否则新指令被连带删掉, 设备停在旧档位且平台以为已成功。
 *
 * @param uid  目标从机
 * @param func 刚发出的功能码
 * @param data 刚发出的数据段(可为 NULL, len=0 时)
 * @param len  数据段长度
 * @return true = 匹配并已删除; false = 缓存里已是别的指令(或没有), 未删除
 */
bool data_cache_down_del_if_match(uint8_t *uid, uint8_t func, const uint8_t *data, uint8_t len);
uint8_t data_cache_down_get_count(void);

#endif /* __DATA_CACHE_H__ */
