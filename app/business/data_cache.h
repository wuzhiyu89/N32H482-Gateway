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
uint8_t data_cache_down_get_count(void);

#endif /* __DATA_CACHE_H__ */
