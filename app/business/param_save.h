#ifndef __PARAM_SAVE_H__
#define __PARAM_SAVE_H__
#include <stdint.h>
#include "n32h47x_48x_flash.h"
#include "cloud_profile.h"     /* 云平台连接档位(P1-0), 随本结构一起掉电保存 */

/* ========== 参数出厂默认值宏定义 ========== */
#define PARAM_DEFAULT_MODBUS_SLAVE_ADDR      0x01U
#define PARAM_DEFAULT_UART_BAUDRATE          115200U
#define PARAM_DEFAULT_REPORT_INTERVAL_MS     10000U
#define PARAM_DEFAULT_LORA_CHANNEL           0U
#define PARAM_DEFAULT_ALARM_POWER_THRESHOLD  3300U
#define PARAM_DEFAULT_ALARM_TTS_LOCAL_EN     0U
#define PARAM_DEFAULT_ALARM_LEVEL_THRESHOLD  500U

/* ========== LoRa 设备地址编码规则 ==========
 * 3字节 UID:  { byte0, byte1, byte2 }
 *   byte0 = (dev_type << 4) | env_id    ← 高4位设备类型, 低4位环境ID
 *   byte1 = (dev_num >> 8) & 0xFF       ← 设备编号高字节 (大端)
 *   byte2 = dev_num & 0xFF              ← 设备编号低字节 (大端)
 *
 *   dev_type:  0x01=网关, 0x02=液位传感器, 0x03=声光报警器
 *   env_id:    0x01=默认环境
 *   dev_num:   0x0001 ~ 0xFFFF
 *   注: 大端编号与 hex 地址串书写顺序一致, 如 "110001" → {0x11,0x00,0x01} → 编号 0x0001
 */
#define PARAM_DEFAULT_GW_TYPE        0x01U   /* 网关 */
#define PARAM_DEFAULT_GW_ENV         0x01U   /* 默认环境 */
#define PARAM_DEFAULT_GW_NUM         0x0001U /* 网关编号 */
/* 默认 gw_uid = {(0x01<<4)|0x01=0x11, 0x00, 0x01}  ← 同 LORA_ADDR_MAKE_BYTE(GATEWAY, DEFAULT) */

/* ========== 从机列表 ========== */
#define PARAM_SLAVE_MAX_CNT          10U     /* 最大从机数量 */

/**
 * @brief 从机 LoRa UID 条目 (固定 3 字节)
 */
typedef struct
{
    uint8_t uid[3];   /* 同 gw_uid 编码规则: byte0=(type<<4)|env, byte1=num高, byte2=num低 (大端) */
} param_slave_uid_t;

/**
 * @brief 设备运行参数结构体, 全部掉电保存
 *
 * 内存布局 (当前): 约 216 字节, 远小于 8KB Flash 页
 *   offset  size  字段
 *   +0      1     alarm_tts_local_en
 *   +1      1     modbus_slave_addr
 *   +2~3    -     padding (到 4 对齐)
 *   +4      4     uart_baudrate
 *   +8      2     report_interval_ms
 *   +10     1     lora_channel
 *   +11~11  -     padding (到 2 对齐)
 *   +12     2     alarm_power_threshold
 *   +14     2     alarm_level_threshold
 *   +16     3     gw_uid[3]
 *   +19     1     slave_cnt
 *   +20     30    slave_uid[10][3]
 *   +50     160   cloud (cloud_profile_t, P1-0 新增)
 *   +210~211 -    padding (到 4 对齐)
 *   +212    4     crc
 *   总计: 216 字节
 *
 * ⚠ 布局变更说明: P1-0 在末尾(crc 之前)插入了 cloud 字段, 因此 device_param_t
 *   的 sizeof 与 CRC 覆盖范围都变了。升级后首次上电, 旧的 Flash 内容 CRC 校验
 *   必然失败 -> param_save_init() 自动回落出厂默认值(本地模式), 这是预期行为。
 *   param_save.c 里有静态断言确保结构不会超出 Flash 页。
 */
typedef struct
{
    /* --- 网关自身参数 --- */
    uint8_t             alarm_tts_local_en;     /* 本地TTS告警开关:1开启,0关闭 */
    uint8_t             modbus_slave_addr;      /* Modbus从站地址 */
    uint32_t            uart_baudrate;          /* 串口波特率 */
    uint16_t            report_interval_ms;     /* 数据上报周期(ms) */
    uint8_t             lora_channel;           /* LoRa信道 */
    uint16_t            alarm_power_threshold;  /* 电源告警阈值(mV) */
    uint16_t            alarm_level_threshold;  /* 液位告警阈值(mm) */

    /* --- 网关自身 LoRa UID ---
     * byte0 = (gw_type << 4) | gw_env
     * byte1 = (gw_num >> 8) & 0xFF (高字节, 大端)
     * byte2 = gw_num & 0xFF (低字节, 大端)
     */
    uint8_t             gw_uid[3];

    /* --- 从机列表 (固定容量, 最多 10 台) --- */
    uint8_t             slave_cnt;                            /* 有效从机数量, 0~10 */
    param_slave_uid_t   slave_uid[PARAM_SLAVE_MAX_CNT];       /* 每台从机 3 字节 UID */

    /* --- 云平台连接档位 (P1-0) ---
     * 本地/文档两套 broker+topic 配置都放这里, 运行期切换, 不用重新烧录。
     * 出厂默认 = 本地模式, 与现场当前行为一致。 */
    cloud_profile_t     cloud;

    /* --- CRC 必须放在末尾 --- */
    uint32_t            crc;
} device_param_t;

/**
 * @brief 参数模块初始化
 * 上电从Flash加载参数; CRC校验失败自动加载出厂默认值
 */
void param_save_init(void);
/**
 * @brief 将内存参数保存到片内Flash, 掉电保存
 * @return 0成功; 非0失败
 * @note 禁止放在循环内频繁调用! 仅修改参数后调用一次
 */
int param_save_store(void);
/**
 * @brief 获取参数结构体指针, 业务读写内存参数
 * @return device_param_t指针
 */
device_param_t* param_get_ptr(void);
/**
 * @brief 恢复出厂默认参数(仅修改内存, 需要调用param_save_store才写入Flash)
 */
void param_set_factory_default(void);

/**
 * @brief 从机列表操作: 添加一台从机
 * @param uid 3字节 UID
 * @return 0成功; -1列表满; -2已存在
 */
int param_slave_add(const uint8_t uid[3]);
/**
 * @brief 从机列表操作: 删除一台从机
 * @param uid 3字节 UID
 * @return 0成功; -1未找到
 */
int param_slave_del(const uint8_t uid[3]);
/**
 * @brief 从机列表操作: 查找一台从机
 * @return 索引(0~9); -1未找到
 */
int param_slave_find(const uint8_t uid[3]);
#endif
