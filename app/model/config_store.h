/**
 * @file  config_store.h
 * @brief 网关全量配置存储(9 个配置域) —— 任务 P2-2
 *
 * 依据: 《RoadTwin 网关配置项设计 V0.1》第 2/3/6 章
 *
 * 【9 个配置域】对应文档第 2 章"配置模型总览":
 *     system     网关标识、访问密码、时区
 *     network    MQTT 连接参数、重连退避、心跳周期
 *     sensors[]  1~5 个水位传感器的绑定与采集参数
 *     thresholds 三级阈值与防抖参数
 *     gears[]    档位 0~3 的动作定义(LED 文本/声光模式/目标设备)
 *     devices[]  0~5 个联动设备的绑定与个性参数
 *     rules      自动/手动模式、失联 fail-safe、手动回归超时
 *     ota        固件版本(只读)、升级策略
 *     meta       配置版本号、CRC32、更新时间(系统维护)
 *
 * 【存储格式 = JSON, 不是二进制结构体】
 *   文档第 6 章明写"全量配置以 JSON 固化在网关 Flash"。这样做还有个实际好处:
 *   写 Flash 用的序列化函数和 cfg.query 应答用的是【同一个函数】, 不多一份代码 ——
 *   二进制结构体方案反而要额外写一套序列化。解析侧同理, cfg.set 与上电加载共用。
 *
 * 【双区 + 掉电安全】文档第 6 章:
 *   "写入先写备份区、校验通过再写主区" → 本实现把 A 当主区、B 当备份区,
 *   固定顺序【先擦写 B, 回读校验通过, 再擦写 A】。于是任意时刻至少有一区完整:
 *     擦写 B 期间 -> A 完好;  擦写 A 期间 -> B 完好。
 *   上电顺序: 读 A 校验 -> 失败读 B -> 再失败回退出厂默认并报事件。
 *
 * 【CRC32 覆盖范围】文档 meta.crc32 = "全量配置(不含本域)的 CRC32"。
 *   因此序列化时: 先拼 system..ota 得到一段 JSON, 对【这一段】算 CRC32,
 *   再把 meta(含该 CRC)接在后面。于是 "json[0..meta_start)" 就是被校验的原文。
 *
 * 【为什么全部用静态缓冲】本工程 Heap_Size 只有 0x200 = 512 字节(见
 *   startup_n32h47x_48x.s), 任何 malloc 都会挂。配置 JSON 约 2KB, 用调用方缓冲。
 */
#ifndef __CONFIG_STORE_H__
#define __CONFIG_STORE_H__

#include <stdint.h>
#include <stdbool.h>

/* cfg.query 要把配置写进应答 data 对象, 所以对外暴露的签名里出现 cJSON ——
 * 这里直接引 cjson_port.h(它带出 cJSON.h), 不让调用方自己保证先包含:
 * 头文件自洽比省一次 include 重要。
 * ⚠ 用 cJSON 之前必须先 cjson_port_init() 装好静态池 hooks(见 app/cjson/cjson_port.h)。
 * cloud_json.h 保留: 反序列化那半边(cfg_parse_* / cfg_get_*)还在用它。 */
#include "cloud_json.h"
#include "cjson_port.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==================== 字段长度上限(文档取值列) ==================== */
#define CFG_GWID_MAX        26      /* system.gwId        ≤26 字节(2026-10-18: 改成 26 位 hex UID) */
#define CFG_SYSPWD_MAX      32      /* system.password    ≤32 字节 */
#define CFG_HOST_MAX        64      /* network.host       ≤64 字节 */
#define CFG_CLIENTID_MAX    32      /* network.clientId   ≤32 字节 */
#define CFG_MQTTUSER_MAX    32      /* network.username   ≤32 字节 */
#define CFG_MQTTPWD_MAX     64      /* network.password   ≤64 字节 */
#define CFG_SERIAL_MAX      16      /* sensors/devices.serial 8~16 字节 */
#define CFG_NAME_MAX        32      /* name               ≤32 字节 */
/* ★2026-10-08: 联动文本上限按《声光报警器对接》改★
 *   LED: 最多 14 个汉字 -> UTF-8 42 字节 (14*3), 留余量取 48
 *   TTS: 最多 28 个汉字 -> UTF-8 84 字节 (28*3), 留余量取 88
 *   ⚠ 单字段上限是【显示宽度】口径(汉字=1, ASCII=0.5); 网关【不做截断】,
 *     由设备侧截断(用户 2026-10-08 确认)。这里只是"存得下"的缓冲大小。
 *   ⚠ 三段文本的总字节数另有 203 字节的硬预算, 见 level_eval.h 的说明。 */
#define CFG_LEDTEXT_MAX     48      /* gears.ledTextTemplate1/2 ≤48 字节 */
#define CFG_TTSTEXT_MAX     88      /* gears.ttsTextTemplate     ≤88 字节 */
#define CFG_FWVER_MAX       15      /* ota.fwVersion */

/* ★2026-10-08: 三段联动文本(LED1+LED2+TTS)的【合计字节预算】★
 *
 * 依据: 《云端对接偏差说明》§2.5 —— `ledTextTemplate1 + ledTextTemplate2 +
 *       ttsTextTemplate ≤ 203 字节`(按 UTF-8 字节算, 一个汉字 3 字节)。
 * 为什么是 203: LoRa 单包上限 240 字节, 空口载荷固定头/长度域占掉 7 字节,
 *      再给协议帧自身留余量, 于是三段文本合计控制在 203。
 *
 * ⚠ 这里是【唯一的定义处】: level_eval.h 的 LEVEL_GEAR_TEXT_BUDGET 只是它的别名,
 *   两边不一致会有编译期断言报错(见 level_eval.h)。改预算只改这一行。 */
#define CFG_GEAR_TEXT_BUDGET   199u

/* 数组长度(文档 3.3 / 3.5 / 3.6) */
#define CFG_SENSOR_MAX      5       /* 液位传感器 1~5。
                                     * 【口径变更】原来是 3(偏差 D-11, 依据是当时用户
                                     * "最多三个"的口头要求); 2026-09-27 用户改为
                                     * "按文档改成 1~5", 所以这里回到文档值。
                                     * ⚠ 连带影响(改完都要重新实测, 别照抄旧数):
                                     *   · config_store_t 变大 -> 静态 RAM 增加
                                     *     (s_config + s_merge_tmp + cloud_proto 的两个副本);
                                     *   · 最坏配置序列化变长 -> 必须重跑 test_cfg 的
                                     *     "最坏情况 < CFG_JSON_MAX" 那条断言;
                                     *   · cJSON 池峰值要重跑 test_cjson 复测。 */
#define CFG_DEVICE_MAX      5       /* devices[] 0~5(文档一致) */
#define CFG_GEAR_COUNT      4       /* gears[]   固定 0~3 */

/* ==================== 子设备寻址: 13 字节 UID（2026-10-18 由 3 字节扩到 13） ====================
 *
 * 文档 3.3/3.6 的地址字段是 `addr`(uint16, 1~255), 但本工程实际用【UID】寻址。
 *
 * ★★2026-10-18 用户决定: UID 从 3 字节扩成 13 字节★★
 *   形式 = byte0(类型/环境) + 12 字节芯片自带的 96 位唯一 ID(GetUID() 读)
 *     网关       : 0x11 + 96 位
 *     液位传感器 : 0x21 + 96 位
 *     声光报警器 : 0x31 + 96 位
 *   hex 串形式就是 26 个字符, 例如 "11A1B2C3D4E5F60718293A4B5C6D7E8F" 这种长度。
 *
 *   【为什么要 96 位】芯片自带唯一 ID, 每个芯片不同 ⇒ 不怕地址重复、不用人工编号。
 *
 *   【不兼容旧的 3 字节 UID】用户 2026-10-18 明确: 研发阶段, 不会再有 3 字节 UID,
 *     旧设备要重新烧录。⇒ cfg_uid_valid() 只认 26 位 hex, 不做"两种长度都接受"。
 *
 *   ⚠️ byte0 的编码约定(与旧 3 字节格式保持同一套语义):
 *      高 4 位 = 设备类型(1 网关 / 2 液位 / 3 声光), 低 4 位 = 环境(现在一律 0)。
 *      现在实际取值: 网关 0x11 / 液位 0x21 / 声光 0x31。
 *
 * 实现选择: 结构体里直接存 26 个 hex 字符 + '\0' 的字符串, 而不是 uint8_t[13]。
 *   · JSON 侧就是直接的字符串拷贝, 不需要编解码胶水;
 *   · 唯一性/相等判断 = strcmp, 不需要 memcmp + 长度;
 *   · 真正要发给 LoRa 时再用 cfg_uid_to_bytes() 解一次(只在下发路径上)。
 * 代价是每个设备多占 20 字节 RAM(13→26 字符), 在这个量级上完全无所谓。
 */
#define CFG_UID_BYTES       13                          /* 二进制形式: 1 类型 + 12 字节 96 位 UID */
#define CFG_UID_HEX_LEN     (CFG_UID_BYTES * 2)         /* 26 个 hex 字符 */
#define CFG_UID_STR_MAX     (CFG_UID_HEX_LEN + 1)       /* 含 '\0' */

/** 校验 hex 串形式: 必须 26 个 hex 字符且不全为 0("全 0" 视为无效地址=擦除态) */
bool cfg_uid_valid(const char *uid_hex);

/**
 * @brief 注册"读芯片 96 位 UID"的钩子 —— 出厂 gwId / clientId 用它生成（2026-10-18）
 * @param fn 由 main.c 注册（内部调厂商 GetUID()）；NULL = 撤销
 * @note  【为什么要 hook】config_store.c 在主机测试里也编译，而 GetUID() 是厂商函数
 *        ⇒ 直接调会让主机 build 链接失败。目标构建拿到真 UID；主机测试拿 0。
 *        与本文件里 param_save_set_uid_hook() 同一套路。
 */
void config_store_set_uid_hook(void (*fn)(uint8_t out[12]));

/** 26 位 hex 串 -> {13 字节}; 格式非法返回 false */
bool cfg_uid_to_bytes(const char *uid_hex, uint8_t out[CFG_UID_BYTES]);

/** {13 字节} -> 26 位 hex 串(大写); out 至少 CFG_UID_STR_MAX 字节 */
void cfg_uid_from_bytes(const uint8_t in[CFG_UID_BYTES], char *out_hex);

/* ==================== 枚举(内部用 uint8, JSON 里用字符串) ==================== */
typedef enum { CFG_MEASURE_LEVEL = 0, CFG_MEASURE_DISTANCE = 1 } cfg_measure_mode_t;

/* ★★2026-10-08: LED 颜色枚举【按空口位定义重排】★★
 *
 * 【为什么必须重排 —— 这是个会静默出错的坑】
 *   原来的顺序是 `OFF=0, GREEN=1, YELLOW=2, RED=3`, 而《声光报警器对接》里
 *   `[3]` 字节的颜色两位定义是:
 *       bit3,2 / bit1,0 = 00 无色 / 01 红 / 10 绿 / 11 黄
 *   也就是 `OFF=0, RED=1, GREEN=2, YELLOW=3` —— 【红和绿的次序正好相反】。
 *   若沿用旧枚举顺序直接往字节里塞, 结果就是"配红色、设备显示绿色",
 *   而且编译、单测、日志【全都看不出来】(两边都是合法的 1 和 2)。
 *   所以这里把枚举顺序改成与空口一致, 打包时直接或运算即可, 不需要映射表。
 *
 * 【JSON 侧的字符串没变】仍然是 "0"/"red"/"green"/"yellow"(见 config_store.c
 *   的 s_color_names), 变的只是内部编号。 */
typedef enum
{
    CFG_COLOR_OFF    = 0,   /* 空口 00 */
    CFG_COLOR_RED    = 1,   /* 空口 01 */
    CFG_COLOR_GREEN  = 2,   /* 空口 10 */
    CFG_COLOR_YELLOW = 3    /* 空口 11 = 红+绿同时 */
} cfg_led_color_t;

/** ★2026-10-08 新增★ 半屏文本的对齐方式(空口 `[3]` 的 bit5 / bit4, 各 1 位)
 *
 * 【为什么只有两个值】《声光报警器对接》原文只给了 `bit5=0居中/对齐`、`bit4=0居中/对齐`
 *   各一位 ⇒ 每个半屏只有"居中/不对齐"两态。
 *   用户 2026-10-08 确认: "只有两个值(对齐和不对齐), 会和云端沟通的"。
 *   ⇒ 早先 §1.9 里写的 `ledTextShowMode` "两个值各 0~3" 是【未落实的设想】,
 *     空口上装不下 4 态; 云端 JSON 侧怎么表达由用户与云端另行商定,
 *     网关侧只认这 1 位。
 * 【位的含义(由对接文档的例子反推)】
 *   例 `1001 0101` 标注"竖屏, 预留0, 居中, 对齐, 左红, 右红":
 *       bit7=1 竖屏 | bit6=0 预留 | bit5=0 居中 | bit4=1 对齐 | 颜色两位都是 01 红
 *   ⇒ bit5 管【第一个半屏】(竖屏=左 / 横屏=上), bit4 管【第二个半屏】(右 / 下)。 */
typedef enum
{
    CFG_LEDALIGN_CENTER = 0,    /* 居中 */
    CFG_LEDALIGN_ALIGN  = 1     /* 对齐 */
} cfg_led_align_t;

typedef enum { CFG_RULE_AUTO = 0, CFG_RULE_MANUAL = 1 } cfg_rule_mode_t;
typedef enum { CFG_OFFLINE_KEEP = 0, CFG_OFFLINE_ALARM, CFG_OFFLINE_OFF } cfg_offline_policy_t;
typedef enum { CFG_OTA_IMMEDIATE = 0, CFG_OTA_IDLE = 1 } cfg_upgrade_policy_t;
typedef enum { CFG_DEVTYPE_SOUND_LIGHT = 1, CFG_DEVTYPE_LED = 2, CFG_DEVTYPE_ALLINONE = 3 } cfg_dev_type_t;

/* 档位号 = 状态映射(文档 3.4, 固定不可配) */
#define CFG_GEAR_RESOLVED   0
#define CFG_GEAR_NORMAL     1
#define CFG_GEAR_WARNING    2
#define CFG_GEAR_ALARM      3

/* ==================== 结构体 ==================== */

/** 3.1 system */
typedef struct
{
    char     gw_id[CFG_GWID_MAX + 1];
    char     password[CFG_SYSPWD_MAX + 1];
    int8_t   timezone;                  /* -12 ~ +14, 默认 +8 */
    uint8_t  reserved;
} cfg_system_t;

/** 3.2 network (整域"重启生效") */
typedef struct
{
    char     host[CFG_HOST_MAX + 1];
    uint16_t port;
    uint8_t  tls;                       /* bool */
    uint8_t  reserved0;
    char     client_id[CFG_CLIENTID_MAX + 1];
    char     username[CFG_MQTTUSER_MAX + 1];
    char     mqtt_password[CFG_MQTTPWD_MAX + 1];
    uint16_t keep_alive_s;              /* 30~300, 默认 60   */
    uint16_t reconnect_min_s;           /* ≥1,     默认 5    */
    uint16_t reconnect_max_s;           /* ≥min,   默认 300  */
    uint16_t status_interval_s;         /* 10~600, 默认 60   */
} cfg_network_t;

/** 3.3 sensors[] 单元素 */
typedef struct
{
    char     uid[CFG_UID_STR_MAX];      /* 13 字节 UID 的 26 位 hex 串 */
    char     serial[CFG_SERIAL_MAX + 1];/* 文档字段; 用户决策 D-4: 本工程不启用 */
    char     name[CFG_NAME_MAX + 1];
    uint8_t  measure_mode;              /* cfg_measure_mode_t; D-1 决策后为死配置 */
    uint8_t  enabled;
    uint16_t install_height_cm;         /* 0~2000; D-1 决策后为死配置 */
    int16_t  calib_offset_mm;           /* -5000~+5000; D-1 决策后为死配置 */
    uint16_t report_period_s;           /* 5~3600, 默认 60  */
    uint8_t  low_battery_pct;           /* 0 = 禁用; 用户决策 D-5: 按【百分比】不按 mV */
    uint8_t  reserved0;
    uint16_t offline_timeout_s;         /* 默认 90; 硬性要求 > report_period_s */
} cfg_sensor_t;

/** 3.4 thresholds (立即生效) */
typedef struct
{
    uint16_t alarm_cm;                  /* > warning_cm, 默认 10 */
    uint16_t warning_cm;                /* > resolve_cm, 默认 7  */
    uint16_t resolve_cm;                /* ≥0,           默认 2  */
    uint16_t min_change_cm;             /* ≥0,           默认 1  */
    uint16_t event_min_interval_s;      /* ≥0,           默认 60 */
    uint8_t  confirm_count;             /* 1~10,         默认 1  */
    uint8_t  reserved;
} cfg_thresholds_t;

/** 3.5 gears[] 单元素
 *
 * ★★2026-10-08 按《云端对接偏差说明》§1.9 + 《声光报警器对接》改造★★
 *   变化:
 *     · sound_mode  语义变: 0关闭 / 1常亮 / 2慢闪 / 3快闪(原来 0/1/2 = 关/仅闪光/闪光+语音)
 *                   —— "要不要播语音"【不再由它决定】, 改看 tts_text 是否为空
 *     · led_show_mode       新增: 0 横屏 / 1 竖屏
 *     · led_text_show_mode  新增: 【两个半屏各 1 位】居中/对齐(见 cfg_led_align_t)
 *     · led_color           单值 -> 【两个半屏各一个】(见 cfg_led_color_t)
 *     · led_text            单段 -> 【拆成 led_text1 / led_text2】
 *                            横屏: 1=上半屏 2=下半屏;  竖屏: 1=左半屏 2=右半屏
 *     · tts_text            保持, 但"空 = 无语音"(与 sound_mode 解耦) */
typedef struct
{
    uint8_t  gear;                      /* 0~3 */
    uint8_t  targets_all;               /* 1 = "all" */
    uint8_t  target_count;              /* targets_all=0 时有效 */
    uint8_t  sound_mode;                /* 0关闭 / 1常亮 / 2慢闪 / 3快闪 */
    uint8_t  led_show_mode;             /* 0 横屏 / 1 竖屏 */
    uint8_t  led_text_show_mode[2];     /* [0]=第一个半屏, [1]=第二个半屏; cfg_led_align_t */
    uint8_t  led_color[2];              /* [0]=第一个半屏, [1]=第二个半屏; cfg_led_color_t */
    char     target_uid[CFG_DEVICE_MAX][CFG_UID_STR_MAX];  /* 目标设备 UID(hex串) */
    char     name[CFG_NAME_MAX + 1];
    char     led_text1[CFG_LEDTEXT_MAX + 1];   /* 竖左(横上); 支持 {level} 占位符 */
    char     led_text2[CFG_LEDTEXT_MAX + 1];   /* 竖右(横下); 任一为空 => 全屏 */
    char     tts_text[CFG_TTSTEXT_MAX + 1];    /* 空 = 无语音 */
} cfg_gear_t;

/** 3.6 devices[] 单元素 */
typedef struct
{
    char     uid[CFG_UID_STR_MAX];      /* 13 字节 UID 的 26 位 hex 串 */
    char     serial[CFG_SERIAL_MAX + 1];/* 文档字段; 用户决策 D-4: 本工程不启用 */
    char     name[CFG_NAME_MAX + 1];
    uint8_t  type;                      /* cfg_dev_type_t, 只读 */
    uint8_t  volume;                    /* 0~100, 默认 50 */
    uint8_t  flash_mode;                /* 0~3,   默认 1  */
    uint8_t  enabled;
} cfg_device_t;

/** 3.7 rules */
typedef struct
{
    uint8_t  mode;                      /* cfg_rule_mode_t */
    uint8_t  sensor_offline_policy;     /* cfg_offline_policy_t */
    uint16_t manual_timeout_s;          /* 0 = 不回归, 默认 300 */
} cfg_rules_t;

/** 3.8 ota (fwVersion 只读) */
typedef struct
{
    char     fw_version[CFG_FWVER_MAX + 1];
    uint8_t  upgrade_policy;            /* cfg_upgrade_policy_t */
    uint8_t  reserved;
} cfg_ota_t;

/** 3.9 meta (系统维护, 平台只读) */
typedef struct
{
    uint32_t config_version;            /* 每次变更 +1 */
    uint32_t crc32;                     /* 全量配置(不含 meta)的 CRC32 */
    uint32_t updated_at;                /* Unix 秒 */
} cfg_meta_t;

/** 全量配置 */
typedef struct
{
    cfg_system_t     system;
    cfg_network_t    network;
    cfg_sensor_t     sensors[CFG_SENSOR_MAX];
    uint8_t          sensor_count;
    uint8_t          reserved0[3];
    cfg_thresholds_t thresholds;
    cfg_gear_t       gears[CFG_GEAR_COUNT];
    cfg_device_t     devices[CFG_DEVICE_MAX];
    uint8_t          device_count;
    uint8_t          reserved1[3];
    cfg_rules_t      rules;
    cfg_ota_t        ota;
    cfg_meta_t       meta;
} config_store_t;

/* ==================== 结果码 ==================== */
typedef enum
{
    CFG_OK = 0,
    CFG_ERR_PARAM,      /* 入参非法(NULL / 缓冲不足) */
    CFG_ERR_JSON,       /* JSON 语法或结构错 */
    CFG_ERR_RANGE,      /* 字段取值越界(含域间约束, 如 alarm<=warning) */
    CFG_ERR_CRC,        /* CRC32 不匹配 */
    CFG_ERR_FLASH,      /* Flash 擦/写/读失败 */
    CFG_ERR_NOBANK      /* 两个区都不可用 */
} cfg_result_t;

/* ==================== 缓冲尺寸建议 ==================== */
/* 文档第 7 章完整示例约 1.8KB。满配(5 传感器 + 5 设备 + 4 档位)约 3.0KB。
 * ★2026-10-08: 4096 -> 6144★
 *   联动档位每个多了一段 LED 文本(ledTextTemplate1+2 取代单个 ledTextTemplate)、
 *   再加上 ledShowMode / ledTextShowMode / 双值 ledColor 三个新字段,
 *   每档约 +130 字节, 四档就是 +520; 原来的 4KB 余量被吃光,
 *   test_cfg 的"最坏情况序列化"直接返回 0(缓冲不够)。
 *   单区是一整页 8KB, 减 24 字节头部后 6144 仍然装得下。
 * 存 Flash 与 cfg.query 应答共用同一个缓冲尺寸。 */
#define CFG_JSON_MAX        6144

/* ==================== 生命周期 ==================== */

/** 载入出厂默认(文档第 6 章 + 3.x 默认值列)。sensors[]/devices[] 清空。 */
void config_store_factory_default(config_store_t *c);

/**
 * @brief 上电加载: 主区 A -> 备份区 B -> 出厂默认
 * @param c 输出(非 NULL)
 * @return CFG_OK=读到有效配置; CFG_ERR_NOBANK=两区都坏, 已填入出厂默认
 */
cfg_result_t config_store_load(config_store_t *c);

/** 惰性加载并返回只读指针(内部静态实例; 首次调用触发 config_store_load) */
const config_store_t *config_store_get(void);

/** 当前内存中配置的 configVersion */
uint32_t config_store_version(void);

/**
 * @brief 把"当前固件版本"写进配置的 ota.fwVersion
 *
 * 为什么需要它: `ota.fwVersion` 是**固件自己维护**的只读字段(平台不可写)，
 * 但以前全工程没有一处写它 —— 出厂默认写死 "1.0.0" 之后再也没变过，
 * 于是 `cfg.query` 永远显示 1.0.0，和 `status.report`(编译期 MAIN_FW_VERSION) 打架。
 * 上电时调用一次；只有值真的变了才写 Flash，且**不动 configVersion**
 * (版本号不是"配置变更"，平台不该因升级看到版本冲突)。
 */
cfg_result_t config_store_set_fw_version(const char *ver);

/** 标记内存配置已改动: configVersion+1, updatedAt=now */
void config_store_touch(config_store_t *c, uint32_t now_unix);

/**
 * @brief 保存到 Flash: 先擦写 B 并回读校验, 通过后擦写 A
 * @return CFG_OK 成功; 其它失败
 * @note  任意时刻至少一区完整, 掉电不会两区同时坏
 */
cfg_result_t config_store_save(const config_store_t *c);

/** 恢复出厂: 内存置默认(configVersion=0) 并落 Flash */
cfg_result_t config_store_reset(config_store_t *c, uint32_t now_unix);

/* ==================== 序列化 / 反序列化 ==================== */

/**
 * @brief 全量序列化为文档第 7 章形状的 JSON(含 meta.crc32)
 * @return 写入长度(不含 '\0'); 0 = 缓冲不足或参数错
 * @note  meta.crc32 由本函数自动计算并回填, 调用方不必自己算
 */
uint16_t config_store_to_json(const config_store_t *c, char *out, uint16_t cap);

/**
 * @brief 计算 meta.crc32 应取的值 = CRC32(system..ota 那一段 JSON)
 * @return CRC32 值(与 zip/zlib 一致的 IEEE 802.3)
 */
uint32_t config_store_calc_crc(const config_store_t *c);

/**
 * @brief 从 JSON 反序列化(全量覆盖)
 * @param c    输出; 函数内先填出厂默认再逐域覆盖, 故缺失的域回落默认值
 * @param json 以 '\0' 结尾
 * @return CFG_OK / CFG_ERR_JSON / CFG_ERR_RANGE / CFG_ERR_CRC(若含 meta.crc32 且不符)
 */
cfg_result_t config_store_from_json(config_store_t *c, const char *json);

/**
 * @brief 域级增量合并 —— cfg.set 用。只覆盖 json 里出现的域, 其余不动。
 * @param changed_mask 可选输出: bit0=system bit1=network bit2=sensors
 *                     bit3=thresholds bit4=gears bit5=devices bit6=rules
 *                     bit7=ota                (不设传 NULL)
 * @return CFG_OK / CFG_ERR_JSON / CFG_ERR_RANGE
 */
cfg_result_t config_store_merge_json(config_store_t *c, const char *json,
                                     uint8_t *changed_mask);

/** 便捷包装: 只关心成败、不需要知道哪些域被改动时用(传 changed_mask=NULL) */
cfg_result_t config_store_apply_json(config_store_t *c, const char *json);

#define CFG_CH_SYSTEM      0x01u
#define CFG_CH_NETWORK     0x02u
#define CFG_CH_SENSORS     0x04u
#define CFG_CH_THRESHOLDS  0x08u
#define CFG_CH_GEARS       0x10u
#define CFG_CH_DEVICES     0x20u
#define CFG_CH_RULES       0x40u
#define CFG_CH_OTA         0x80u

/**
 * @brief 域间/取值校验(文档 3.x 的"取值"列)
 * @return CFG_OK; CFG_ERR_RANGE 越界
 */
cfg_result_t config_store_validate(const config_store_t *c);

/* ==================== 逐域增量合并(cfg.set 用) —— 任务 P2-4 ====================
 *
 * 【与 config_store_merge_json() 的关键区别: 事务范围不同】
 *   merge_json()      : 【全局事务】任一域失败 -> 整体回滚, 一个字段都不改(上电加载用)。
 *   merge_sections()  : 【逐域事务】每个域单独 copy -> 只应用这一个域 -> validate ->
 *                       成功才提交回 *c 并置 applied 位; 失败则【丢弃该域】并置
 *                       rejected 位, 然后继续处理下一个域。
 *
 * 为什么必须两套: 文档 5.2 明确要求"校验失败【整域拒绝】, 其余域照常生效"。
 * 用全局事务会在"平台一次下发 3 个域、其中 1 个写错"时把另外 2 个正确改动一起扔掉,
 * 而应答里又必须如实告诉平台哪些生效了 —— 这不是实现口味问题, 是协议语义。
 *
 * json 的形状是"域对象"(与 config_store_merge_json 完全相同), 例如:
 *     {"thresholds":{"alarmCm":20},"sensors":[{"uid":"210001","name":"东门"}]}
 * 它同时也是 cfg.set 的 data.sections 那一段。
 *
 * 【meta 域被刻意忽略】文档 5.2 的 sections 域列表里没有 meta(configVersion/crc32/
 * updatedAt 是系统维护字段, 平台只读), 所以 meta 既不算 applied 也不算 rejected。
 *
 * 【返回值】只有入参非法才返回 CFG_ERR_PARAM; 域级失败一律通过 res 汇报,
 * 即使全部域都被拒也返回 CFG_OK —— 由调用方看 res 决定怎么应答。
 */
/** 逐域合并结果(位掩码, 用上面的 CFG_CH_* 常量) */
typedef struct
{
    uint8_t applied;    /* 校验通过并已提交回 *c 的域 */
    uint8_t rejected;   /* 处理/校验失败、改动已被丢弃的域 */
} cfg_sections_result_t;

cfg_result_t config_store_merge_sections(config_store_t *c, const char *json,
                                         cfg_sections_result_t *res);

/* ==================== cfg.query 用: 直接写应答 data ==================== */

/**
 * @brief 把配置的 9 个域写进调用方【已经建好的】JSON 对象里
 *
 * cfg.query 的应答 data 就是配置对象本身, 所以本函数只往传入的对象里挂成员,
 * 不再套一层。meta.crc32 由本函数重新计算后回填(与存 Flash 的那一份一致)。
 *
 * @param obj 已经建好的对象(即应答的 data 对象)
 * @param c 配置
 * @return true = 全部写成功; false = 参数错或池满(调用方应把整棵应答树丢掉)
 */
bool config_store_write_members(cJSON *obj, const config_store_t *c);

/**
 * @brief 取【可写】的当前配置实例 —— cfg.set 用
 *
 * config_store_get() 返回 const(只读)指针, 这是对的: 绝大多数调用方只该读。
 * 但 cfg.set 必须改写"当前配置"本身(否则改完立即 cfg.query 会读到旧值, 应答撒谎),
 * 所以补一个显式的可写入口, 而不是在 cloud_proto.c 里把 const 强转掉 ——
 * 强转会把这个"这里确实要写"的意图藏起来。
 *
 * 语义与 config_store_get() 完全一致(惰性加载, 返回内部静态实例), 不拷贝。
 * 改完记得 config_store_touch() + config_store_save()。
 */
config_store_t *config_store_get_mut(void);

/* ==================== 设备绑定/解绑(P5, LoRa 0x04/0x05 用) ====================
 *
 * 【为什么放这里】绑定/解绑的真相源是 config_store.sensors[]/devices[](2026-09-29
 * 用户决策: 统一用 config_store, 废弃 param_save.slave_uid 作 LoRa 表)。这组函数
 * 封装"加一台/删一台 + 落 Flash", 让 lora_business 不必关心数组搬运与校验默认值。
 *
 * 【容量约束】(用户 2026-09-29): 总数≤5, 液位传感器≤4, 声光报警器≤3。
 *   - can_add() 纯查询, 不改状态
 *   - bind_device() 会先查 can_add + 查重, 通过才写
 *
 * 【默认值】新绑的设备字段填文档默认值(report_period_s=60, offline_timeout_s=90,
 *   type=SOUND_LIGHT, volume=50, flash_mode=1, enabled=1), 保证 config_store_validate
 *   不会因为 0 值把整份配置打回。 */
/** 能否再绑一台该类型设备; true=液位(sensors[]), false=联动(devices[]) */
bool config_store_can_add(bool is_sensor);

/**
 * @brief 绑定一台设备(写 config_store + 落 Flash)
 * @param uid_hex   6 字符 hex 串(如 "210001")
 * @param serial    序列号(可空串; 文档字段, 本工程不启用但保留形状)
 * @param name      名称(可空串)
 * @param is_sensor true → sensors[]; false → devices[]
 * @param dev_type  【联动设备类型】cfg_dev_type_t(1 声光 / 2 LED / 3 一体);
 *                  仅 is_sensor=false 时有效(传感器忽略它)。
 *                  ★ 第 11 条(2026-09-30)新增 ★: 以前 devices[].type 是硬编码 1,
 *                  于是平台绑 LED(type=2)也会被记成声光 —— 而该字段文档标"只读",
 *                  平台改不了, 只能解绑重绑。
 *                  传 0 或越界值 = 用出厂默认(声光), 兼容旧调用点。
 * @param now_unix  当前 Unix 秒(写 meta.updatedAt + configVersion+1)
 * @return CFG_OK; CFG_ERR_PARAM(uid 非法); CFG_ERR_RANGE(超限/已存在)
 */
cfg_result_t config_store_bind_device(const char *uid_hex, const char *serial,
                                       const char *name, bool is_sensor,
                                       uint8_t dev_type, uint32_t now_unix);

/**
 * @brief 解绑一台设备(从 config_store 删除 + 落 Flash)
 * @param uid_hex 6 字符 hex 串(在 sensors[] 和 devices[] 两边都找)
 * @param now_unix 当前 Unix 秒
 * @return CFG_OK; CFG_ERR_PARAM(uid 非法); CFG_ERR_RANGE(未找到)
 * @note  连带清理 gears[].target_uid 里指向该 uid 的条目(与 cloud_cmd_unbind 同口径)
 */
cfg_result_t config_store_unbind_device(const char *uid_hex, uint32_t now_unix);

/**
 * @brief 重绑(同 UID 换新设备): 只更新元数据, 【不新增】条目（2026-10-14 新增）
 * @param uid_hex  设备的 13 字节 UID(26 位 hex 串)
 * @param serial   新序列号(可 NULL, 当空串)
 * @param name     新名称(可 NULL, 当空串)
 * @param dev_type 设备类型(仅对 devices[] 生效; 液位传感器忽略)
 * @param now_unix 当前 Unix 秒(用于 updated_at + 版本递增)
 * @return CFG_OK; CFG_ERR_PARAM(uid 非法); CFG_ERR_RANGE(该 UID 不在配置里)
 *
 * 【为什么要它】config_store_bind_device() 查重后对已存在的 UID 直接返回
 *   CFG_ERR_RANGE, 所以没法用来"覆盖"。
 * 【用户场景(2026-10-14)】设备坏了换新, 重新烧录后回到同一默认地址
 *   ⇒ 允许同 UID 再绑一次, 但条目数不变。
 * 【覆盖什么】serial / name(设备侧还有 type)。
 * 【刻意不动什么】report_period_s / offline_timeout_s / enabled / low_battery_pct
 *   / measure_mode / install_height_cm / calib_offset_mm —— 用户口径:
 *   平台一定会再下发上报周期, 网关不需要动(动了反而把平台设的值悄悄改回 60)。
 * ⚠️【不要】用 unbind + bind 代替本函数 —— unbind 会连带清掉 gears[].target_uid
 *   里指向该 UID 的条目(见上面那条的 note), 那台设备以后收不到联动指令
 *   (现象: 能绑定、能上报, 就是不动灯)。
 */
cfg_result_t config_store_rebind_device(const char *uid_hex, const char *serial,
                                        const char *name, uint8_t dev_type,
                                        uint32_t now_unix);

/* ==================== 诊断(日志口 #cfg 用) ==================== */
/** 打印一行摘要: 版本/传感器数/设备数/阈值/规则 */
void config_store_dump(const config_store_t *c);

/* ==================== 存储端口(主机测试可注入) ==================== */
/*
 * 目标构建(N32H482): 直接操作 FM_CFG_A_ADDR / FM_CFG_B_ADDR 两个 Flash 页。
 * 主机构建: 用两块 RAM 映像, 便于在 PC 上跑"写入->读回->坏一区回退"的测试。
 *
 * offset 相对该 bank 起始地址; bank: 0=A(主区), 1=B(备份区)。
 */
#if !defined(N32H482)
/** 主机测试用: 取该 bank 的 RAM 映像指针(容量 FM_CFG_SIZE), 可直接破坏字节 */
uint8_t *config_store_test_bank(uint8_t bank);
/** 主机测试用: 把某 bank 整体置 0xFF(模拟擦除态 / 空板) */
void     config_store_test_wipe(uint8_t bank);
/**
 * 主机测试用: 让接下来 n 次 Flash 写失败(模拟 Flash 写失败)。
 * 用途: 验证"落盘失败时【绝不回 code=0】"这条约定
 *       (cfg.set / sys.factory_reset 都靠 config_store_save 的返回值判成败,
 *        而目标板上没法人为造一次写失败)。
 * 只影响主机端口; 目标构建(N32H482)下这个函数不存在, 也永远不会被编进去。
 */
void     config_store_test_fail_write(int times);
#endif

#ifdef __cplusplus
}
#endif

#endif /* __CONFIG_STORE_H__ */
