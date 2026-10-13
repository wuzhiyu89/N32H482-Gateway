/**
 * @file  cloud_profile.h
 * @brief 云平台连接档位(profile) —— P1-0
 *
 * 背景(P0-1/P0-2/P0-7 决策):
 *   文档(RoadTwin)要求 clientId=gwId、topic 用 rt/gw/{gwId}/...、可选应用层 auth。
 *   ★2026-10-08 起【文档档位成为出厂默认】★ —— 甲方平台就按文档那套约定,
 *   而且明确不再使用 mod/pc_down / mod/mod_upload(两个遗留主题已整体删除)。
 *   本地档位(mod/xxx)仍保留为第二套配置, 但只作历史/回退用途。
 *
 * 为什么用运行期档位而不是 #if / 注释掉:
 *   1. 同一台设备要能在不同 broker/主题约定间切换。编译期开关意味着每切一次都要
 *      重新烧录; 运行期档位改个配置就行。
 *   2. 注释掉的代码不参与编译 -> 不会有编译错误 -> 半年后打开必炸。
 *      档位里的代码一直参与编译, 不会腐烂。
 *   3. 代价只有约 1KB flash。
 *
 * 存放位置: 嵌入 param_save 的 device_param_t, 与其它掉电参数同一页同一 CRC。
 *           (加字段会改变 device_param_t 布局 -> 旧 Flash 内容 CRC 不过 ->
 *            自动回落出厂默认; 出厂默认现在是【文档档位 + gwId=110001】)
 */
#ifndef __CLOUD_PROFILE_H__
#define __CLOUD_PROFILE_H__

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- 字段容量(含 '\0') ---- */
#define CLOUD_HOST_MAX      48
#define CLOUD_CLIENTID_MAX  24
#define CLOUD_GWID_MAX      24
#define CLOUD_PREFIX_MAX    24
#define CLOUD_AUTH_MAX      32

/* ---- 档位模式 ---- */
typedef enum
{
    CLOUD_MODE_LOCAL = 0,   /* 本地/历史档: mod/cmd · mod/telemetry ...(无 auth) —— 非默认 */
    CLOUD_MODE_DOC   = 1    /* ★出厂默认★ 甲方平台: rt/gw/{gwId}/... + 用户名=gwId + 可选 auth */
} cloud_mode_t;

/* ---- 文档定义的 5 个 topic ---- */
typedef enum
{
    CLOUD_TOPIC_CMD = 0,    /* 平台 -> 网关  下行命令     QoS1 */
    CLOUD_TOPIC_CMDRESP,    /* 网关 -> 平台  命令应答     QoS1 */
    CLOUD_TOPIC_TELEMETRY,  /* 网关 -> 平台  水位遥测     QoS0 */
    CLOUD_TOPIC_EVENT,      /* 网关 -> 平台  事件         QoS1 */
    CLOUD_TOPIC_STATUS,     /* 网关 -> 平台  状态/心跳    QoS1, retained */
    /* ★★2026-10-08 新增: 遗嘱(LWT)专用主题 ★★
     *
     * 【为什么必须独立一个主题 —— 现场真的踩了】
     *   原来遗嘱发到 status 主题、内容 {"online":false}、retained。
     *   于是【每次网关上电】平台都会先收到一条 {"online":false},
     *   然后才是 status.report(online:true) —— 看起来像"网关刚掉线又上线"。
     *   根因是 MQTT 协议行为: clientID 固定时, 设备掉电后 broker 侧旧会话要等
     *   keepalive 超时才过期; 网关重连时 broker 踢掉旧会话, 那一刻【旧会话的遗嘱
     *   才真正发出】—— 也就是"上电瞬间收到 offline"。
     *   我方的缓解是"一上线立刻补发一条 retained 状态"把遗嘱盖回去(main.c 里有),
     *   但那只是缓解: 平台仍会在毫秒级窗口里看到一次假的 offline, 而且 status 的
     *   retained 值被短暂污染。
     *
     * 【现在】遗嘱走 rt/gw/{gwId}/lwt, 与 status 完全隔离:
     *   · status 的 retained 只由本机主动上报维护, 不再被 broker 侧的旧遗嘱覆盖;
     *   · 平台要【多订阅一个主题】rt/gw/{gwId}/lwt 才能拿到掉线通知
     *     (已在《云端对接偏差说明》§2.4 提过, 用户 2026-10-08 拍板采用)。
     *   · 本地档位(mod/xxx)没有这个主题的历史, 用 "mod/lwt" 保持一致。 */
    CLOUD_TOPIC_LWT,        /* 网关 -> 平台  遗嘱         QoS1, retained */
    CLOUD_TOPIC_COUNT
} cloud_topic_t;

typedef struct
{
    uint8_t mode;                               /* cloud_mode_t */
    uint8_t id_map_enable;                      /* 1 = 启用 gwId <-> LoRa 3字节地址 映射 */
    uint8_t auth_enable;                        /* 1 = 下行命令必须带正确的 auth */
    uint8_t reserved0;

    char     host[CLOUD_HOST_MAX];              /* MQTT broker 域名/IP */
    uint16_t port;                              /* 1883 等 */
    uint16_t reserved1;

    char     client_id[CLOUD_CLIENTID_MAX];     /* 空串 = 运行时用模组 IMEI */
    char     gw_id[CLOUD_GWID_MAX];             /* 文档用网关 ID, 如 "GW0001" */
    char     topic_prefix[CLOUD_PREFIX_MAX];    /* 文档用 "rt/gw"; 本地模式忽略 */
    char     auth[CLOUD_AUTH_MAX];              /* 应用层二次密码; 本地模式可为空 */
} cloud_profile_t;

/* ==================== 默认值 / 校验 ==================== */

/** 写入出厂默认(= 本地模式, 与当前现场行为一致) */
void cloud_profile_set_factory_default(cloud_profile_t *p);

/**
 * @brief 校验档位各字段
 * @return 0 通过; 1 模式非法; 2 host 空/超长; 3 port 为 0; 4 gw_id 非法;
 *         5 topic_prefix 非法; 6 auth 超长
 */
int cloud_profile_validate(const cloud_profile_t *p);

/* ==================== 读写 ==================== */

/**
 * @brief 取当前档位(懒加载: 首次调用时从 param_save 读, 失败则用出厂默认)
 * @note  不依赖初始化顺序, 任何时候调用都安全
 */
const cloud_profile_t *cloud_profile_get(void);

/**
 * @brief 校验并保存档位(写 Flash)
 * @return 0 成功; 非 0 见 cloud_profile_validate 的返回值
 */
int cloud_profile_set(const cloud_profile_t *p);

/** 恢复出厂默认(仅内存, 需 cloud_profile_set 才落 Flash) */
void cloud_profile_reset(void);

/**
 * @brief ★2026-10-08 新增★ 把 Flash 里的"遗留 LOCAL 档"一次性迁移到 DOC 档
 *
 * 【为什么需要】`cloud_profile` 存在 param_save 的 Flash 页里(不在程序区),
 * 旧固件写进去的 LOCAL 档【重烧 App 也带不走】—— 结果是网关订阅 mod/cmd,
 * 而甲方平台往 rt/gw/{gwId}/cmd 发, 现场表现是"心跳看得到、指令收不到"。
 * 由 main.c 在启动时调用一次: 不是 DOC 档就改成 DOC 并落盘, 返回 true。
 *
 * @return true = 刚刚做了迁移(应打一条显眼日志); false = 本来就是 DOC 档
 */
bool cloud_profile_migrate_to_doc(void);

/**
 * @brief 只改 RAM 副本, 【不写 Flash】
 * @return 0 成功; 非 0 见 cloud_profile_validate
 * @note  用途: 1) 自检时临时构造场景; 2) 以后做"临时切到文档 broker 试一下"
 *        而不想固化到 Flash。掉电即失效。
 */
int cloud_profile_set_ram(const cloud_profile_t *p);

/* ==================== 便捷查询 ==================== */

bool cloud_profile_is_doc(void);
bool cloud_profile_auth_required(void);

/**
 * @brief ★2026-10-08 新增★ 用配置里的"设备 ID"(config_store.system.gwId) 覆盖档位 gwId
 *
 * topic 里的 {gwId} 和 MQTT 用户名必须是同一个值 —— 甲方平台就是这么认设备的。
 * 真相源是 config_store(出厂 "110001", 可用 cfg.set 改); 本模块不依赖它,
 * 由 main.c 启动时注入。传 NULL/空串 = 不改。只在【下次 MQTT 建链】生效。
 */
void cloud_profile_set_gw_id(const char *gw_id);

/**
 * @brief 取某个 topic 的完整名字
 * @param out 输出缓冲(cap 建议 >= 48)
 * @return 写入长度; 失败返回 0
 */
uint16_t cloud_profile_build_topic(cloud_topic_t t, char *out, uint16_t cap);

/** 该 topic 应有的 QoS(文档: cmd/cmdresp/event/status=1, telemetry=0) */
uint8_t cloud_profile_topic_qos(cloud_topic_t t);

/** 该 topic 是否需要 retained(文档: 只有 status) */
bool cloud_profile_topic_retained(cloud_topic_t t);

/** topic 的固定名字部分, 便于日志 */
const char *cloud_profile_topic_name(cloud_topic_t t);

/**
 * @brief 校验下行命令自带的 auth 是否正确
 * @return true 通过(未启用 auth 时恒为 true)
 */
bool cloud_profile_check_auth(const char *auth_from_msg);

#ifdef __cplusplus
}
#endif

#endif /* __CLOUD_PROFILE_H__ */
