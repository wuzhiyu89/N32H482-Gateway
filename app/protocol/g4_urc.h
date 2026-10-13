#ifndef __G4_URC_H__
#define __G4_URC_H__

#include <stdint.h>
#include <stdbool.h>

/**
 * \file  g4_urc.h
 * \brief ML307R MQTT URC 解析 + 在线状态维护 + 下行消息【队列化】投递
 *
 * 依据《MQTT 用户手册 V6.7.0》第 3.9 节 URC 集合:
 *   +MQTTURC: "conn",<connect_id>,<conn_state>          连接状态(统一入口)
 *   +MQTTURC: "publish",<id>,<mid>,<topic>,<total>,<plen>,<payload>  下行消息
 *   +MQTTURC: "suback",<id>,<mid>,<code>[,...]           订阅结果
 *   +MQTTURC: "unsuback",<id>,<mid>                      取消订阅结果
 *   +MQTTURC: "puback"/"pubrec"/"pubcomp",<id>,<mid>,<dup>  发布应答
 *   +MQTTURC: "timeout",<id>,<mid>                       超时
 *   +MQTTURC: "pubnmi",<id>,<mid>,<data_len>             缓存模式新数据提示
 *   +MQTTURC: "drop",<id>,<dropped_length>               缓存满丢包
 *   +MQTTURC: "pingresp",<id>,<ping_ret>                 心跳结果
 *
 * <conn_state> 取值(Page 30):
 *   0 连接成功 / 1 正在重连 / 2 用户断开 / 3 服务器拒绝 / 4 服务器断开 /
 *   5 ping 超时 / 6 网络异常 / 255 未知
 *
 * ============================ 2026-09 结构性改版 ============================
 *
 * 【改版原因】老版本的 g4_urc_scan() 是在 AT 收发【间隙】被调用的:
 *   · 发布事务期间它整个被跳过 → 那段窗口里的云端下行没人正确解析, 被"按行"读掉;
 *   · 它自己又和 AT 等待循环抢同一条字节流 → publish 头被读到、payload 被丢。
 * 两条合起来就是现场那句"云端下发一快就收不到, 过一会儿还断线"。
 *
 * 【现在】:
 *   · 帧解析交给 g4_io.c 的字节级状态机(它按 <payload_len> 收定长二进制, 不受
 *     payload 里的 0x00/0x0A/0x0D 影响), 本模块只负责【协议语义】:
 *         g4_urc_input_line()     ← 一条完整行(返回 true = 这行是 URC, 已被消费)
 *         g4_urc_input_publish()  ← 一条收全的 publish(只入队, 不做业务)
 *   · 业务回调【不在收包路径上执行】: publish 进一个小队列, 由 g4_urc_poll() 在主
 *     循环里取出来投递。这样即使上层回调里做了耗时的活(组帧/LoRa 发送),
 *     也不会卡住收包泵, 更不会造成"回调里再阻塞等 AT"的重入。
 */

/**
 * @brief MQTT 下行消息回调(在 g4_urc_poll() 上下文执行, 不在收包中断/泵内部)
 * @param topic 主题字符串(以'\0'结尾, 指向内部缓冲, 回调返回后失效)
 * @param data  载荷(原始二进制)
 * @param len   载荷长度
 */
typedef void (*g4_mqtt_recv_cb_t)(const char *topic, const uint8_t *data, uint16_t len);

/** @brief URC 模块初始化(内部会把自己注册到 g4_io 的接收泵上) */
void g4_urc_init(void);

/** @brief 注册 MQTT 下行接收回调(可空) */
void g4_urc_set_recv_cb(g4_mqtt_recv_cb_t cb);

/* ==================== OTA over HTTP: +MHTTPURC 钩子 ====================
 *
 * ML307R 原生 HTTP 客户端(AT+MHTTP*)的 URC 是 +MHTTPURC: "content" / "header"
 * / "err" / "recv" / "ind", 与 +MQTTURC 完全不同的家族。
 *
 * 【为什么用回调而不是直接调 ota_http_on_urc()】: 本模块是"URC 分派器", 不该
 * 反向依赖 OTA 模块; 而且主机单测里可以不链接 ota_http.c 也能测 g4_urc.c。
 *
 * 【回调在收包泵内部执行】: 与 g4_urc_input_publish() 同一约束 —— 绝不能在里面
 * 写 Flash / 发 AT / 阻塞。ota_http_on_urc() 只做"解 HEX + 入队", 写盘留给
 * ota_http_poll()。
 *
 * @param line 完整行
 * @return true = 这一行已被 OTA 模块消费(不要再拿去做 AT 应答匹配)
 */
typedef bool (*g4_urc_http_cb_t)(const char *line);

/** @brief 注册 +MHTTPURC 处理回调(可空; 为空则该家族 URC 只打日志) */
void g4_urc_set_http_cb(g4_urc_http_cb_t cb);

/**
 * @brief 取出一条排队中的下行消息并投递给回调(非阻塞, 每次最多投一条)
 *        由主循环持续调用 —— 必须在 g4_io_pump() 【之外】调用。
 * @return true = 本次投递了一条
 */
bool g4_urc_poll(void);

/* ---- 下面两个由 g4_io.c 的接收泵调用, 上层不要直接调 ---- */

/**
 * @brief 收到一条完整行
 * @return true = 这行是 URC 家族(已被本模块消费, 不要再拿去做 AT 应答匹配)
 */
bool g4_urc_input_line(const char *line, uint16_t len);

/** @brief 收到一条收全的 publish(定长二进制 payload 已按 <payload_len> 收满) */
void g4_urc_input_publish(const char *topic, const uint8_t *data, uint16_t len);

/* ---- 状态查询 ---- */

/** @brief MQTT 是否在线(仅由 URC/状态机维护) */
bool g4_urc_is_online(void);

/** @brief 标记在线(状态机订阅成功后调用) */
void g4_urc_set_online(void);

/** @brief 强制标记离线(状态机进入 ERROR/RESET 时调用) */
void g4_urc_set_offline(void);

/**
 * @brief 最近一次 +MQTTURC: "conn",<id>,<state> 的 <state>
 * @return 0 连接成功 / 1 正在重连 / 2 用户断开 / 3 服务器拒绝 /
 *         4 服务器断开 / 5 ping 超时 / 6 网络异常 / 255 未知 /
 *         0xFF = 本次建链还没收到过 conn URC
 */
uint8_t g4_urc_get_conn_state(void);

/** @brief 清掉上次的 conn 结果(发 MQTTCONN 之前调用)
 *         ⚠️ 它同时会把"在线"标志清掉 —— 不然新一轮建链会看见【上一轮的】在线状态 */
void g4_urc_reset_conn_state(void);

/**
 * @brief 收到的 +MQTTURC: "conn" URC 的【累计条数】(单调递增, 不会被清)
 *
 * 【为什么需要它】: g4_urc_is_online() 是个锁存标志, 只能说明"历史上连上过"。
 * 建链时的 WAIT_CONN 必须等【本次 MQTTCONN 之后新到】的那条 conn URC,
 * 靠比较"发命令前后的 seq 有没有变"来保证这一点 —— 否会拿着上一轮的 true
 * 瞬间判成功(现场日志 78.txt: "MQTT conn ok (res=255 unknown)" 后
 * MQTTSUB 连吃 4 个 +CME ERROR: 607 再 rebuild, 每轮白烧 30~40 秒)。
 */
uint32_t g4_urc_conn_seq(void);

/* ---- 诊断计数 ---- */

/** @brief broker 确认过的 publish 次数(+MQTTURC: "puback") —— 上行"真的到了"的证据 */
uint32_t g4_urc_puback_count(void);

/** @brief 收到的 publish 下行条数 */
uint32_t g4_urc_pub_rx_count(void);

/** @brief 因为下行队列满而丢掉的条数(>0 说明云端下发比主循环处理还快) */
uint32_t g4_urc_pub_drop_count(void);

/* ==================== 驻网(注册)与 PS 附着观测 ====================
 *
 * 由 g4_at.c 打开 AT+CEREG=2 / AT+CGEREP=1 之后, 模组会主动上报注册状态变化与
 * PS 去附着。本模块【只统计 + 把原始行打日志】, 不做任何解析或行为 —— 两种形态的
 * 字段位置不同(URC 首数是 stat, 轮询应答首数是 <n>), 判读见 g4_urc.c 的说明。
 *
 * ⚠️ 计数里【含 AT+CEREG? 的轮询应答】(它和 URC 前缀一样, 此处无法区分);
 *    要"纯粹的主动上报条数", 请用日志里的 "[REG] 驻网行(原始)" 行数人工判读。
 */

/** @brief +CEREG/+CGREG/+CREG 行累计条数(含轮询应答) */
uint32_t g4_urc_reg_line_count(void);

/** @brief 最近一条驻网行的第一个数; 0xFF = 还没见过(仅人工判读用, 勿当 stat) */
uint8_t g4_urc_reg_first_num(void);

/** @brief +CGEV 行累计条数 */
uint32_t g4_urc_cgev_count(void);

/** @brief 其中含 "DETACH" 的条数 —— 【最强的断网证据】
 *         手册原文: "网络强制 PS 去附着, 这意味着所有激活的上下文都被去激活" */
uint32_t g4_urc_cgev_detach_count(void);

#endif /* __G4_URC_H__ */
