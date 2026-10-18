#ifndef __OTA_H__
#define __OTA_H__

#include <stdint.h>
#include <stdbool.h>

/**
 * \file  ota.h
 * \brief OTA 升级框架(数据源: 4G/MQTT 下行)
 *
 * 架构分层:
 *   ota.h/c       - 升级状态机 + 分包重组 + CRC 校验 + 缺包检测/超时(协议无关)
 *   ota_port.h/c  - 移植层: Flash 擦写 / CRC32 / 时间 / 持久化 / 重启(N32H482 实现)
 *
 * 数据通路(后续对接时):
 *   4G MQTT 下行 URC -> g4_at recv_cb -> ota_on_mqtt_msg(topic, data, len)
 *                                          │
 *                                          v
 *                                  OTA 状态机(本模块)
 *                                          │
 *   4G MQTT 上行  <── ota_missing_cb(缺包) ─┤
 *                                          v
 *                                  ota_port(flash/crc/reboot)
 *
 * 当前阶段: 仅搭建框架,不与现有代码对接(不修改 g4_at.c / main.c)。
 */

/* ==================== 升级通道主题 ==================== */
#define OTA_TOPIC_CMD      "n32h482/ota/cmd"     /* 命令通道(JSON 文本) */
#define OTA_TOPIC_DATA     "n32h482/ota/data"    /* 数据通道(二进制分包) */

/* ==================== 分包协议(OTA_TOPIC_DATA 的 payload 格式) ====================
 * 偏移   长度   字段        说明
 *  0     4     magic       "OTA1"(0x4F 0x54 0x41 0x31)
 *  4     2     seq         包序号(小端, 0-based)
 *  6     2     total       总包数(小端)
 *  8     2     len         本包载荷长度(小端, 0..OTA_PACKET_PAYLOAD_MAX)
 * 10     2     crc16       本包头(前10字节)的 CRC16(填0跳过)
 * 12     ...   payload     载荷数据, 长度 = len
 *
 * 顺序约束: seq=0 必须先到(携带 cmd 中的元数据已确认), 之后乱序也接受。
 * 每包固定 offset = seq * OTA_PACKET_PAYLOAD_MAX(中间包 len 必须等于 512,
 * 末尾包可小于 512),保证 Flash 写入 8 字节对齐。
 */
#define OTA_PACKET_MAGIC0      'O'
#define OTA_PACKET_MAGIC1      'T'
#define OTA_PACKET_MAGIC2      'A'
#define OTA_PACKET_MAGIC3      '1'
#define OTA_PACKET_HEADER_LEN  12u
#define OTA_PACKET_PAYLOAD_MAX 512u
#define OTA_PACKET_TOTAL_MAX   (OTA_PACKET_HEADER_LEN + OTA_PACKET_PAYLOAD_MAX)

/* ==================== 命令通道 JSON 字段 ==================== */
/* 示例:
 *   {"cmd":"start","version":"1.2.3","size":245760,"crc32":305419896,"packets":480}
 *   {"cmd":"abort"}
 *   {"cmd":"reboot"}
 */
#define OTA_CMD_START   "start"
#define OTA_CMD_ABORT   "abort"
#define OTA_CMD_REBOOT  "reboot"

/* ==================== 超时配置 ==================== */
#ifndef OTA_DOWNLOAD_TIMEOUT_MS
#define OTA_DOWNLOAD_TIMEOUT_MS   30000u   /* 下载期间两包间最大间隔, 超时触发缺包重传 */
#endif
#ifndef OTA_RETRANSMIT_MAX
#define OTA_RETRANSMIT_MAX        3u       /* 同一会话最大重传轮次, 超过进 ERROR */
#endif
#ifndef OTA_ERROR_RECOVER_MS
#define OTA_ERROR_RECOVER_MS      30000u   /* ERROR 状态自动回 IDLE 的超时, 0=不自动恢复 */
#endif

/* ==================== OTA 状态机 ==================== */
typedef enum
{
    OTA_IDLE = 0,        /* 空闲,等待升级命令 */
    OTA_GOT_CMD,         /* 收到 start 命令,参数已校验(预留,当前直进 ERASING) */
    OTA_ERASING,         /* 擦除目标 Flash 区(分页异步) */
    OTA_DOWNLOADING,    /* 接收数据包并写入 Flash */
    OTA_VERIFYING,      /* CRC32 校验整个升级区 */
    OTA_READY,           /* 校验通过,待 reboot 切换 */
    OTA_ERROR            /* 失败,等待外部 abort 或超时回 IDLE */
} ota_state_t;

/* ==================== 升级元数据(来自 cmd JSON) ==================== */
typedef struct
{
    char     version[24];   /* 新版本号字符串 */
    uint32_t total_size;    /* 固件总字节数 */
    uint32_t crc32;         /* 整个固件的 CRC32 */
    uint16_t packet_count;  /* 总包数 */
} ota_meta_t;

/* ==================== 升级状态快照(供业务层 MQTT 上报) ==================== */
typedef struct
{
    ota_state_t state;          /* 当前状态 */
    ota_meta_t  meta;            /* 升级元数据 */
    uint16_t    recv_packets;    /* 已收包数 */
    uint16_t    missing_count;   /* 当前缺失包数 */
    uint16_t    retransmit_cnt;  /* 已触发重传轮次 */
    uint16_t    progress_permil; /* 进度 0..1000 */
} ota_status_t;

/* ==================== 回调类型 ==================== */

/**
 * @brief 缺包上报回调(下载超时后触发,业务层应通过 MQTT 上报缺失 seq 列表)
 * @param seqs    缺失包序号数组(指向内部临时缓冲,回调返回后失效)
 * @param count   缺失包数量
 */
typedef void (*ota_missing_cb_t)(const uint16_t *seqs, uint16_t count);

/* ==================== 对外 API ==================== */

/**
 * @brief OTA 初始化(清状态,不触碰 Flash)
 */
void ota_init(void);

/**
 * @brief OTA 主轮询(推进擦除/校验/超时检测,不阻塞)
 *        需在主循环持续调用
 */
void ota_poll(void);

/**
 * @brief MQTT 下行消息入口(后续 g4_at 的 recv_cb 调用即可对接)
 *        本框架阶段: 实现完整,但不被任何代码调用
 * @param topic  主题(以'\0'结尾)
 * @param data   载荷
 * @param len    载荷长度
 */
void ota_on_mqtt_msg(const char *topic, const uint8_t *data, uint16_t len);

/**
 * @brief 注册缺包上报回调(超时后触发,业务层据此请求重传)
 */
void ota_set_missing_cb(ota_missing_cb_t cb);

/**
 * @brief 获取完整状态快照(供业务层 MQTT 上报服务端)
 */
void ota_get_status(ota_status_t *out);

/**
 * @brief 当前状态查询(轻量)
 */
ota_state_t ota_get_state(void);

/**
 * @brief 获取升级进度(0..1000, 千分比)
 */
uint16_t ota_get_progress_permil(void);

/**
 * @brief 强制中止当前升级(回到 IDLE,不擦 Flash)
 *        注:已写入的缓存区数据保留,但状态标记为无效(bootloader 侧需校验 magic)
 */
void ota_abort(void);

#endif /* __OTA_H__ */
