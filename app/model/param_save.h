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

/* ★2026-10-18 新增: 出厂字段(产线写入的编号, 例 "YA26420001")★
 * 只存本地 Flash, 【不进 config_store】⇒ 云端 cfg.query / cfg.set / status.report
 * 全都看不到它 —— 用户明确要求"这个只有我们开发者知道"。
 * 网关自己的出厂值是 "WANGGUAN00"。 */
#define PARAM_FIELD_LEN              10U
#define PARAM_DEFAULT_FIELD          "WANGGUAN00"

/**
 * @brief 从机 LoRa UID 条目（2026-10-18 由 3 字节扩到 13 字节）
 *
 * ★★为什么是 13★★  UID = byte0(类型/环境) + 12 字节芯片自带 96 位唯一 ID(GetUID())
 *   ⇒ 字节数从 3 变 13, RAM 每台从机 +10 字节, 10 台共 +100 字节, 可接受。
 *   ⚠️ 与 config_store.h 的 CFG_UID_BYTES【必须一致】——
 *      param_save.c 里有 _Static_assert 盯着(它同时 include 两个头)。
 *   ⚠️ 本头文件不 include config_store.h(避免循环依赖), 所以用局部宏。
 */
#define PARAM_UID_BYTES              13U     /* = CFG_UID_BYTES */

typedef struct
{
    uint8_t uid[PARAM_UID_BYTES];   /* byte0=(type<<4)|env, byte1..12 = 96 位 UID */
} param_slave_uid_t;

/**
 * @brief 设备运行参数结构体, 全部掉电保存
 *
 * 内存布局: 【由编译器按对齐规则决定, 不在注释里维护逐字段偏移表】
 *   以前这里有一张 offset/size 表, 每次加字段都要手算, 已经错过两次:
 *     · 2026-10-18 UID 3→13 字节(改成 328 字节那版);
 *     · 2026-10-18 加 field[11](出厂字段)之后表又过时了。
 *   ⇒ 手写偏移表**必然会漂**, 而且漂了没人发现(它只是注释)。
 *   ⇒ 改成用【静态断言】把真正要紧的两条约束钉死(见 param_save.c):
 *       ① sizeof(device_param_t) 必须 <= 一个 Flash 页(8KB);
 *       ② crc 必须是【最后一个】字段 —— 存盘时对 offsetof(crc) 之前的字节算 CRC。
 *   具体怎么排布、有多少 padding, 以编译器为准, 不要照注释手工推算。
 *
 * 当前字段清单(顺序即声明顺序):
 *   alarm_tts_local_en / modbus_slave_addr / uart_baudrate / report_interval_ms /
 *   lora_channel / alarm_power_threshold / alarm_level_threshold /
 *   gw_uid[13] / field[11](出厂字段, 云端不可见) / slave_cnt /
 *   slave_uid[10][13] / cloud(cloud_profile_t) / crc
 *
 * ⚠ 布局变更说明: 每次增删字段, sizeof 与 CRC 覆盖范围都会变 ⇒ 升级后首次上电
 *   旧 Flash 内容 CRC 必然失败 -> param_save_init() 自动回落出厂默认值。
 *   这是预期行为(P1-0 插 cloud、2026-10-18 UID 3→13、加 field 都是如此)。
 *   用户已明确"旧数据不用管, 不会再有三字节 UID", 所以不做迁移。
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

    /* --- 网关自身 LoRa UID（2026-10-18: 3 字节 -> 13 字节）
     * byte0 = (gw_type << 4) | gw_env    现在实际是 0x11
     * byte1..12 = 芯片自带 96 位唯一 ID（GetUID() 读出来的 12 字节）
     */
    uint8_t             gw_uid[PARAM_UID_BYTES];

    /* 出厂字段(10 字节 ASCII + 1 字节 NUL)。★云端不可见★, 见 PARAM_FIELD_LEN 的说明 */
    char                field[PARAM_FIELD_LEN + 1U];

    /* --- 从机列表 (固定容量, 最多 10 台) --- */
    uint8_t             slave_cnt;                            /* 有效从机数量, 0~10 */
    param_slave_uid_t   slave_uid[PARAM_SLAVE_MAX_CNT];       /* 每台从机 13 字节 UID */

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
 * @param uid 13 字节 UID（2026-10-18: 原 3 字节）
 * @return 0成功; -1列表满; -2已存在
 */
int param_slave_add(const uint8_t uid[PARAM_UID_BYTES]);
/**
 * @brief 从机列表操作: 删除一台从机
 * @param uid 13 字节 UID
 * @return 0成功; -1未找到
 */
int param_slave_del(const uint8_t uid[PARAM_UID_BYTES]);
/**
 * @brief 从机列表操作: 查找一台从机
 * @return 索引(0~9); -1未找到
 */
int param_slave_find(const uint8_t uid[PARAM_UID_BYTES]);

/**
 * @brief 注册"读芯片 96 位唯一 ID"的取值函数（2026-10-18 新增）
 * @param fn 由 main.c 注册（内部调厂商的 GetUID()）；传 NULL 表示撤销
 *
 * 【为什么用 hook 而不是直接调 GetUID()】
 *   param_save.c 在【主机测试】里也会被编译(test_cfg 套件), 而 GetUID() 是厂商
 *   函数(只在目标构建里有) ⇒ 直接调会让主机 build 链接失败。
 *   ⇒ 目标构建: main.c 注册 hook, 拿到真 UID;
 *     主机测试: 不注册, gw_uid[1..12] 全 0, 与旧行为一致, 不崩。
 *   (与本工程 last_value_set_now_hook() 同一套路。)
 *
 * @note out 必须是 12 字节(96 位), 不含类型字节 —— gw_uid[0] 的类型由
 *       param_save 自己填(网关 = 0x11)。
 */
void param_save_set_uid_hook(void (*fn)(uint8_t out[12]));
#endif
