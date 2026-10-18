/**
 * \file  lora_proto.h
 * \brief LoRa 自定义二进制帧协议（网关 ↔ 液位传感器 / 声光报警器）
 *
 * 协议规格：帧头 AA 55 | 小端字节序 | Modbus CRC16（0xA001, 0xFFFF）
 * CRC 覆盖范围: 从协议版本字节开始算到 data 末尾（不含帧头 AA 55 和末尾 CRC 两字节）
 *
 * 帧格式（完整 13 + N 字节）:
 *   [0]   AA                  帧头 1
 *   [1]   55                  帧头 2
 *   [2]   ver                 协议版本 (当前 0x01)
 *   [3]   gw_uid[0]           网关地址 字节0: 设备类型(高4位)+环境ID(低4位)
 *   [4]   gw_uid[1]           网关地址 字节1: 设备编号大端高字节
 *   [5]   gw_uid[2]           网关地址 字节2: 设备编号大端低字节
 *   [6]   dev_uid[0]          设备地址 字节0: 设备类型(高4位)+环境ID(低4位)
 *   [7]   dev_uid[1]          设备地址 字节1: 设备编号大端高字节
 *   [8]   dev_uid[2]          设备地址 字节2: 设备编号大端低字节
 *   [9]   func                功能码
 *   [10]  data_len            data 载荷长度 N (0~255)
 *   [11]  data[0..N-1]        载荷
 *   [11+N] crc16_low          Modbus CRC16 低字节
 *   [12+N] crc16_high         Modbus CRC16 高字节
 *
 * 恒定量: 11 字节帧头部分 + 2 字节 CRC = 13, 即帧总长 = 13 + N。
 * CRC 计算范围: &ver ~ &data[N-1] —— 共 (10 + N) 字节
 *
 * 20 字节上报帧(设备端 app_sensor.c, N=7)实例:
 *   AA 55 | 01 | 11 00 01 | 21 00 01 | 02 | 07 | 04 D2 64 00 00 00 00 | FF A8
 *   └─头─┘  ver  └─ gw ─┘  └─ dev ─┘  fc  dlen  └── 数据域 7 字节 ──┘  CRC
 *   数据域: 液位(2B 大端) | 电量%(1B) | 月 | 日 | 时 | 分
 *
 * 地址编码规则:
 *   前一字节 = (dev_type << 4) | env_id     ← 设备类型高4位 + 环境ID低4位
 *   后两字节 = dev_num (大端)              ← 设备编号高字节在前, 低字节在后
 *   例: 0x81 0x00 0x34 → 类型=0x08, 环境ID=0x01, 编号=0x0034=52
 *   注: 大端编号与 hex 地址串书写顺序一致, 如 "110001" → {0x11,0x00,0x01} → 编号 0x0001
 */
#ifndef __LORA_PROTO_H
#define __LORA_PROTO_H

#include <stdint.h>
#include <stdbool.h>

/* ==================== Radio 底层接口（复用原有 LoraApp_*） ==================== */
#include "project_config.h"

/* 回调：LoRa 收到一包完整数据时触发（radio RxDone 里调） */
typedef void (*LoraRxCallback_t)(uint8_t *buf, uint8_t len, int8_t rssi, int8_t snr);

uint8_t  LoraApp_Init(void);
void     LoraApp_SetRxCallback(LoraRxCallback_t cb);
uint8_t  LoraApp_SendAsync(uint8_t *buf, uint8_t len, uint32_t msTimeout);
void     LoraApp_StartRxContinuous(void);
void     LoraApp_StopRx(void);
void     LoraApp_ProcessIrq(void);

/**
 * \brief 立刻切回【连续接收】(幂等; 网关的常态就应该是"一直在收")
 * \param why 非 NULL = 这是一次"修复", 会打日志并累加补开计数;
 *            正常补位(发射之后)传 NULL, 不吵日志
 *
 * 【什么时候需要它】芯片在这些时刻会离开 RX, 必须补回来:
 *   · 每次发射之后(radio.c 的 TxDone 分支会把模式置成 STDBY);
 *   · 发射超时(TxTimeout) —— 这条路径原来【不会】请求重启 RX,
 *     一次发射超时就能让网关此后一直收不到;
 *   · 接收窗口超时 / 接收错误;
 *   · 任何外因造成的模式漂移。
 * lora_proto_poll() 里还有一层"接收守护": 每秒读一次芯片真实状态,
 * 不在 RX 就自动补开 —— 所以"网关一直在接收"是可被验证的, 不是假设。
 */
void     LoraApp_EnsureRxContinuous(const char *why);

/** \brief 芯片现在真的在接收吗(读芯片状态字节, 不是读软件缓存) */
bool     LoraApp_IsRxActive(void);

/** \brief 累计"发现不在接收并补开"的次数(诊断用; 只增不减) */
uint32_t LoraApp_RxRearmCount(void);

/**
 * \brief 射频诊断命令 "#lora": 打印【芯片里实际是什么配置】+ IRQ 状态 + 一句话判读
 *
 * 【只读】只发 Get 类命令(GetStatus/GetIrqStatus/GetStats/GetErrors/GetRssiInst)
 * 和读寄存器; 【绝不】发 Set/Calibrate/SetRx/SetStandby/ResetStats —— 目的就是
 * "看清楚现状", 顺手改一下就把唯一的证据毁掉了。因此不会打断正在进行的接收
 * (芯片一直留在 RX, 只是被多读了几次状态)。
 *
 * 【为什么不能只看 IRQ status 寄存器】主循环的 RadioIrqProcess() 每处理一次中断
 * 就把 IRQ status 清一次, 所以事后读它永远是 0 —— "RxDone 到底有没有触发过"
 * 只能看 radio.c 里新增的 IRQ 累计计数(g_irqStat, 只增不减)。
 *
 * 用法: 日志口(USART1, 115200)敲 "#lora" + 回车; 命令挂在 main.c 的
 *       g4_at_set_console_ext() 扩展钩子上(#help 末尾也会列出)。
 */
void LoraApp_DiagPrint(void);
/**
 * \brief 阻塞等这一包真的发完(TX_DONE)
 * \param timeout_ms 超时上限
 * \return true = 收到 TX_DONE(空口上确实发完了); false = 超时(芯片没发出去)
 * \note  任何 LoraApp_SendAsync()/lora_proto_send_frame() 之后, 只要还想动芯片
 *        (Standby / 切 RX / 再发下一包), 都【必须】先调本函数。
 *        否则 SetStandby/SetRx 会把还没发完的包当场掐断。
 */
bool     LoraApp_WaitTxDone(uint32_t timeout_ms);

/* ==================== 帧协议常量 ==================== */

#define LORA_FRAME_HEAD0        0xAAU
#define LORA_FRAME_HEAD1        0x55U
#define LORA_PROTO_VER          0x01U

/* 帧结构(与控制端/设备端 app_cmd.c / app_sensor.c 完全一致, 共 31 字节帧头 + 2 字节 CRC):
 *   [0]AA [1]55 [2]ver [3..15]gw_uid [16..28]dev_uid [29]func [30]data_len [31..]data [..]crc16
 *
 * ★★2026-10-18: UID 由 3 字节扩到 13 字节 ⇒ 帧头 11 -> 31, 开销 13 -> 33 ★★
 *   用户口径: "LORA 的总帧长保持 240" ⇒ 载荷上限随之从 240 降到 207。
 *   ⚠️ 连带必须改的两处(否则档位载荷装不下):
 *        LEVEL_GEAR_PAYLOAD_MAX  210 -> 206   (207 - 1, 留 1B 给 func)
 *        LEVEL_GEAR_TEXT_BUDGET  203 -> ~199
 *   ⚠️ 设备端(app_cmd.c / app_sensor.c)【必须同步】改成 (33 + data_len) == rx_len,
 *      且两边的 gw_uid/dev_uid 都要按 13 字节收发。新旧不互通。
 *
 * ⚠️ 历史上这里曾错写成 14, 结果:
 *     上报帧按 14+7=21 算, 实收 20 -> 长度校验必然失败 -> 帧被静默丢弃
 *     -> 帧回调从不被调用 -> 表现成"网关一直收到帧但永远不回 ACK"。
 *     ⇒ 常量必须与设备端的判据【逐字节】一致, 改这里务必同步设备端。 */
#define LORA_UID_BYTES          13U     /* = CFG_UID_BYTES = PARAM_UID_BYTES */
#define LORA_FRAME_OVERHEAD     33U     /* 2 头 + 1 版本 + 13 gw + 13 dev + 1 func + 1 dlen + 2 CRC */
/* 无载荷时的最小帧长(即 31 + 0 + 2), 用于解析前的最低长度检查 */
#define LORA_FRAME_HDR_SIZE     LORA_FRAME_OVERHEAD
/* 载荷长度上限（声光报警器带 LED+TTS 的 GB2312 文本）
 * 2026-10-18: 240 -> 207（总帧长保持 240: 240 - 33 = 207） */
#define LORA_FRAME_DATA_MAX     207U
/* 完整帧缓冲大小 */
#define LORA_FRAME_RAW_MAX      (LORA_FRAME_OVERHEAD + LORA_FRAME_DATA_MAX)

/* ===== 帧内字段偏移（2026-10-18: 全部由常量算出，禁止再写死数字）=====
 * 布局: [0]AA [1]55 [2]ver [3..15]gw [16..28]dev [29]func [30]dlen [31..]data [..]crc
 * ⚠️ 以前这些偏移是【写死的 3/6/9/10/11】—— 把 UID 改成 13 字节时,
 *    只要漏改一处, 帧就会【静默错位】(解析出来 func 是 UID 的一个字节),
 *    而且编译期完全看不出来。⇒ 现在一律用下面的宏。 */
#define LORA_OFF_GW_UID         3U
#define LORA_OFF_DEV_UID        (LORA_OFF_GW_UID  + LORA_UID_BYTES)     /* 16 */
#define LORA_OFF_FUNC           (LORA_OFF_DEV_UID + LORA_UID_BYTES)     /* 29 */
#define LORA_OFF_DLEN           (LORA_OFF_FUNC + 1U)                    /* 30 */
#define LORA_OFF_DATA           (LORA_OFF_DLEN + 1U)                    /* 31 */

/* CRC 覆盖范围: 从 ver 到 data 末尾 (跳过帧头 2B + 跳过末尾 CRC 2B)
 *  = 完整帧总长 - 4 */
#define LORA_CRC_START_OFFSET   2U

/* ===== 收发模式切换等待(ms) =====
 * LLCC68 从接收切到发射要重新走 PLL/PA ramp 校准, 从发射切回接收也要重新上电
 * 接收链路, 这段时间里发出的前导码对方解不出来、正在收的包也会丢。
 *
 * 两边约定(2026-xx 调整):
 *   - 收到消息 -> 切到发射 -> 等 LORA_TX_SETTLE_MS 再发 ACK;
 *     "收到消息"到"发出 ACK"之间的总等待就是这一个值, 不再另外叠加延时。
 *   - 发送方切回接收不需要额外等待, 所以 LORA_RX_SETTLE_MS 只用于
 *     "给别处留的余量", 默认取 0。
 *
 * 连发场景(一条消息连发 5 次 ACK)注意: 发送路径每次都会等这个值, 如果给 5s,
 * 连发的"3s 间隔"就变成 8s 了。连发期间芯片本来就在发送侧, 切换开销很小,
 * 所以这里收到 500ms 即可。
 * 用日志里的 "[TX] .. submitted ok (切TX等待=..ms)" 验证实际耗时。 */
#define LORA_TX_SETTLE_MS       500U
#define LORA_RX_SETTLE_MS       0U

/* ===== 等 TX_DONE 的超时上限(ms) =====
 * 【必须等】Radio.Send() 只把 SET_TX 命令写进芯片就返回了, 发射是芯片异步完成的。
 * 调用方若在 Send() 之后紧接着调 Standby()/Rx(), 芯片会当场中止这次发射:
 * 日志上"submitted ok"照打, 空口上却一个字节都没出去 —— 对端永远收不到 ACK。
 * 所以发完之后一律用 LoraApp_WaitTxDone() 等 TX_DONE, 再切模式。
 *
 * 13 字节 @SF8/BW500 空中约 20ms, 这里取 1s 留足余量:
 * 即使对端被改成 SF12/125k(空中可达 1s 级)也不会误判成"没发出去"。 */
#define LORA_TX_DONE_TIMEOUT_MS 1000U

/* ==================== 地址编码（设备类型 + 环境ID + 编号） ==================== */

/* 设备类型（高4位） */
#define LORA_DEV_TYPE_GATEWAY       0x01U   /* 网关 (云端/本板) */
#define LORA_DEV_TYPE_LEVEL_SENSOR  0x02U   /* 液位传感器（超声波） */
#define LORA_DEV_TYPE_SOUND_LIGHT   0x03U   /* 声光报警器 */

/* 环境ID（低4位，当前仅 0x01 一种） */
#define LORA_ENV_ID_DEFAULT         0x01U

/* 地址编码宏: (type << 4) | env */
#define LORA_ADDR_MAKE_BYTE(type, env)   (((uint8_t)(type) << 4) | ((uint8_t)(env) & 0x0FU))
#define LORA_ADDR_GET_TYPE(byte)          (((byte) >> 4) & 0x0FU)
#define LORA_ADDR_GET_ENV(byte)           ((byte) & 0x0FU)

/* 辅助: 从 13 字节 UID 里取类型/环境
 * ★2026-10-18: UID 由 3 字节扩到 13 字节(byte0 类型 + 12 字节芯片 96 位唯一 ID)。
 *   ⇒ lora_addr_type()/lora_addr_env() 语义不变(仍只看 byte0)。
 *   ⇒ ⚠️ lora_addr_num() 【已失去意义】并【已删除】——
 *        原来它是 "byte1<<8 | byte2" 的 2 字节编号; 现在 byte1..12 是 96 位芯片 UID,
 *        用 uint16 根本装不下。寻址改成"13 字节全等比较"(memcmp), 不再按编号。
 *      ⇒ 原来所有用 lora_addr_num() 的地方都已改成直接比 UID。 */
static inline uint8_t lora_addr_type(const uint8_t uid[LORA_UID_BYTES]) { return LORA_ADDR_GET_TYPE(uid[0]); }
static inline uint8_t lora_addr_env(const uint8_t uid[LORA_UID_BYTES])  { return LORA_ADDR_GET_ENV(uid[0]); }

/** 两个 13 字节 UID 是否相同 —— 取代原来的 lora_addr_num() 比较 */
static inline bool lora_uid_equal(const uint8_t a[LORA_UID_BYTES], const uint8_t b[LORA_UID_BYTES])
{
    uint8_t i;
    if ((a == NULL) || (b == NULL)) { return false; }
    for (i = 0u; i < LORA_UID_BYTES; i++)
    {
        if (a[i] != b[i]) { return false; }
    }
    return true;
}

/* ==================== 功能码 ==================== */

typedef enum
{
    LORA_FUNC_SET_SENSOR_PARAM   = 0x01U,   /* 云端→网关→液位传感器 设置传感器参数 */
    LORA_FUNC_SENSOR_REPORT      = 0x02U,   /* 液位传感器→网关→云端 上报液位数据 */
    LORA_FUNC_SET_ALARM          = 0x03U,   /* 云端→网关→声光报警器 设置声光报警器 */
    LORA_FUNC_BIND_REQ           = 0x04U,   /* 设备→网关广播 绑定请求(目标地址=FFFFFF) */
    LORA_FUNC_UNBIND             = 0x05U,   /* 网关→设备 解绑通知 */
    /* ★2026-10-18 新增: 出厂字段读写(仅产线/开发者用, 与云端无关)★
     * 字段 = 10 字节 ASCII, 例 "YA26420001"(Y=液位, A, 年26, 周42, 编号0001);
     * 网关自己的出厂值是 "WANGGUAN00"。
     *   · 0x06 网关→设备: data = 10 字节字段   => 设备回 0x80(成功)/0x81(失败)
     *   · 0x07 网关→设备: data 为空            => 设备回 0x08(data = 10 字节字段)
     *   · 0x08 设备→网关: data = 10 字节字段   => 网关【必须回 0x80】做 ack
     * 设备侧 0x07 的处理(用户 2026-10-18 定): 发 0x08 后开 5 秒窗口等网关 ack;
     *   没收到就再发一次 + 再等 5 秒, 之后不管收没收到都回主流程(最多发 2 次)。
     * ⚠️ 0x80/0x81 是纯应答码(不带"应答的是哪个 func"), 所以设备端处理 0x06/0x07
     *    时必须【串行等 ack】, 不能与别的下发交错 —— 否则 ack 会被张冠李戴。 */
    LORA_FUNC_SET_FIELD          = 0x06U,   /* 网关→设备 写入出厂字段 */
    LORA_FUNC_GET_FIELD          = 0x07U,   /* 网关→设备 读取出厂字段(请求) */
    LORA_FUNC_FIELD_RESP         = 0x08U,   /* 设备→网关 字段应答(data = 字段) */
    LORA_FUNC_OTA                = 0x70U,   /* 云端→网关→设备 OTA 升级（格式待补充） */
    LORA_FUNC_RESP_OK            = 0x80U,   /* 双向 正常应答("收到了", 不代表执行成功) */
    LORA_FUNC_RESP_ERR           = 0x81U,   /* 双向 错误/重传应答 */
    /* ★★2026-10-08 新增: 声光设备(0x03)的执行结果码★★
     *
     * 【为什么 0x80 不够用】
     *   0x80 是"我收到了"(链路层应答)。声光设备收到 0x03 后会【先后回两条】:
     *       立即       回 0x80      —— 收到了(还没执行)
     *       约 10 秒后 回 0x82/0x83 —— 执行成功 / 执行失败
     *   用户 2026-10-08 确认: "0x82 是正常应答所以 data 字段为空, 0x83 现在
     *   data 字段也为空, 执行成功要上报云端设备执行成功 event"。
     *   ⇒ 要判断"灯到底亮没亮", 必须等 0x82/0x83, 不能拿 0x80 当成功。
     *
     * ⚠️ 只对 pending_func == LORA_FUNC_SET_ALARM 生效; 0x01(设液位参数)等
     *   其它功能码仍然把 0x80/0x81 当终态, 行为完全不变。 */
    LORA_FUNC_ALARM_DONE         = 0x82U,   /* 设备→网关 声光执行成功(data 为空) */
    LORA_FUNC_ALARM_FAIL         = 0x83U    /* 设备→网关 声光执行失败(data 为空) */
} lora_func_code_t;

/* ==================== 通用帧结构 ==================== */

/**
 * \brief 通用 LoRa 帧结构（所有功能码共用）
 *        协议层解析/组包时统一用这个结构，业务层再按 func 分发
 */
typedef struct
{
    uint8_t gw_uid[LORA_UID_BYTES];     /* 网关地址(13 字节: 类型 + 96 位芯片 UID) */
    uint8_t dev_uid[LORA_UID_BYTES];    /* 设备地址(13 字节, 同上) */
    uint8_t func;               /* 功能码 */
    uint8_t data_len;           /* 载荷长度 */
    uint8_t data[LORA_FRAME_DATA_MAX];  /* 载荷 */
    uint8_t raw[LORA_FRAME_RAW_MAX];    /* 完整原始帧(含帧头+CRC, parse_frame 时填充) */
    uint8_t raw_len;            /* 原始帧总长(含帧头+CRC) */
} lora_frame_t;

/* ==================== 业务层回调 ==================== */

/**
 * \brief 业务层 LoRa 帧处理回调（协议层解析完调用）
 * \param frame  已解析好的通用帧(含 gw_addr/dev_addr/func/data_len/data)
 * \param rssi   信号强度 dBm
 * \param snr    信噪比 dB
 */
typedef void (*LoraFrameCallback_t)(const lora_frame_t *frame, int8_t rssi, int8_t snr);

/* ==================== 协议层对外 API ==================== */

/**
 * \brief 初始化：注册 radio 底层接收回调 + 初始化内部环形缓冲
 * \param cb 业务层帧回调（可选，传 NULL 则仅投递到 ring_buf）
 */
void lora_proto_init(LoraFrameCallback_t cb);

/**
 * \brief 组帧：把通用帧结构编码成完整字节流（含帧头 + 自动追加 CRC16）
 * \param frame  输入: 填好 gw_uid/dev_uid/func/data_len/data
 *               输出: raw[] 和 raw_len 被填充
 * \param raw_out 可选: 若不为 NULL, 完整帧字节流复制到这里（长度 raw_len）
 * \return true 组帧成功；false 参数非法(data_len 超出上限等)
 */
bool lora_proto_build_frame(lora_frame_t *frame, uint8_t *raw_out);

/**
 * \brief 直接解析一帧原始字节流（给定完整帧, 解析 + CRC 校验）
 * \param raw     完整原始帧字节流(含帧头 + CRC16)
 * \param raw_len 帧总长
 * \param out     解析输出(通用帧结构)
 * \return true 解析成功且 CRC 通过; false 帧头错/长度错/CRC 错/参数空
 * \note 协议层内部会维护双缓冲: RxDone 中断投递 → poll 主循环解析并分发回调
 */
bool lora_proto_parse_raw(const uint8_t *raw, uint8_t raw_len, lora_frame_t *out);

/**
 * \brief 发送通用帧（内部先组帧 + 调 LoraApp_SendAsync，然后自动开 RX）
 * \param frame 待发送帧(填好 gw_uid/dev_uid/func/data_len/data)
 * \return true 提交发送成功；false 参数非法或 radio 忙
 */
bool lora_proto_send_frame(lora_frame_t *frame);

/**
 * \brief 【原样透传发送】把给定的字节一个不改地发到空口(不加帧头/版本/CRC)
 * \param data 待发送字节(任意内容, 可以是二进制)
 * \param len  字节数(1 ~ LORA_FRAME_DATA_MAX)
 * \return true = 收到 TX_DONE(空口上确实发出去了); false = 参数非法/芯片没发出去
 *
 * 【和 lora_proto_send_frame 的区别】
 *   send_frame() 一定会重新组帧(加 AA 55 + 版本 + 重算 CRC), 用在协议帧转发;
 *   send_raw()   一个字节都不加不改, 用在"云端发什么, 空口就发什么"的透传场景。
 *
 * 【内部照样会等 TX_DONE】LoraApp_SendAsync() 只是把 SET_TX 写进芯片就返回,
 * 不等 TX_DONE 就切模式会把这一包掐断在 PA ramp 阶段 —— 日志说发了, 空口没有。
 */
bool lora_proto_send_raw(const uint8_t *data, uint8_t len);

/**
 * \brief 主循环轮询：处理 radio 中断 + 从环形缓冲解析帧 + 分发业务回调
 * \note main while(1) 里持续调用
 */
void lora_proto_poll(void);

/**
 * \brief 撤销"TX 完成后重进接收"的挂起请求
 * \note  连发多帧、且要求帧间隔期不切回接收时, 每发完一帧调一次。
 *        否则 TxDone 中断会把芯片切回 RX, 间隔期就变成"在接收"了。
 */
void lora_proto_cancel_rx_restart(void);

#endif /* __LORA_PROTO_H__ */
