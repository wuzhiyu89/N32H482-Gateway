#ifndef __G4_IO_H__
#define __G4_IO_H__

#include <stdint.h>
#include <stdbool.h>

/* 单条普通行的上限。
 *
 * 【为什么放在头文件里】: 它不只是 g4_io.c 的内部实现细节, 还是一个【对外约束】——
 * 任何"把一大段数据塞进一条 AT/URC 行"的模块都必须服从它, 否则超长部分会被
 * g4_io 强制切行, 后半截不再以原来的前缀开头, 数据就丢了。
 * 典型例子: OTA 的 HEX 内容 URC(`+MHTTPURC: "content"...`), 行长约 2*分片 + 58。
 * ota_http.c 里有一条 #if 用它做编译期自检 —— 那个 #if 曾经因为
 * 这个宏只在 g4_io.c 里定义而【永远成立】(未定义标识符在 #if 里算 0),
 * 白白让固件编不过。所以提到头文件, 值保持不变。
 *
 * ★★2026-10-14: 512 -> 4352(+64 头余量) —— 修一个"大下行全坏"的真 bug ★★
 * ---------------------------------------------------------------------------
 * 【症状】平台下发一条 951 字节的 rule.set, 网关回 1001 bad request, 且日志里
 *   同一条命令被记成【两次下行】:
 *       [4G DOWN] topic=rt/gw/110001/cmd len=489 hex: 7B 22 63 6D 64 ...   ("{"cmd")
 *       [4G DOWN] topic=rt/gw/110001/cmd len=462 hex: E9 A2 84 ...          (切在中文串中间!)
 *   第二段不以 '{' 开头 -> 被判"不像 JSON" -> 落到 LoRa 透传路径 -> 又被丢。
 *
 * 【根因】g4_io.c 的字节泵在缓冲满时是这么干的:
 *       if (s_line_len < (G4_IO_LINE_BUF_SZ - 1u)) { s_line[s_line_len++] = ch; }
 *       else { g4_io_process_line(); s_line_len = 0u; ... }     <-- 把【半条】当完整行交上去
 *   ⇒ 512 字节的上限把一条 951 字节的 +MQTTURC 下行在第 511 字节处【强制截断】,
 *     前半段被当成一条完整 URC ⇒ 截断的 JSON 必然解析失败 ⇒ 1001。
 *   ⇒ 而且模块本身【会分包】(旧归档注释就写着"ML307R publish URC 会分包"),
 *     长度超过行缓冲时无论如何都会被切。
 *
 * 【为什么直接加大缓冲就够(而不是改成"按 payload_len 定长收")】
 *   本工程对下行的【法律上限】就是 G4_URC_PUB_MAX = 4352(见 g4_urc.c):
 *   超过它的下行本来就会被那一层拒掉, 根本到不了这里。
 *   ⇒ 只要行缓冲 > 4352 + URC 头长度, 这个类别的 bug 就【全类】消失,
 *     不需要引入"publish 收集态"那套状态机(改核心接收泵的风险更高)。
 *   ⇒ 换句话说: 值必须 >= 下行法律上限, 这是两者之间的不变式。
 *
 * ⚠️ RAM 代价: +3840 字节静态。改前 160.80 KB / 192 KB(83.7%),
 *   改后 ~164.6 KB(85.7%) —— 仍有余量, 但要留意不要再往上加。
 * ⚠️ 如果将来把 G4_URC_PUB_MAX 调大, 这里【必须跟着调大】, 否则同一个 bug 会
 *   以新的阈值重现。下面 g4_io.c 里有一条编译期断言盯着这个不变式。 */
#ifndef G4_IO_LINE_BUF_SZ
#define G4_IO_LINE_BUF_SZ     4416u    /* 4352(下行法律上限) + 64(URC 头余量) */
#endif

/**
 * \file  g4_io.h
 * \brief 4G 模块 AT 传输层: USART2 (PC8-TX / PC9-RX) + 【统一接收泵】+ 非阻塞 AT 命令引擎
 *
 * ============================ 为什么要改成"泵 + 非阻塞" ============================
 *
 * 老版本有三个消费者各自调 g4_io_read_line() 去抢同一条字节流:
 *     AT 等待循环 / 发布子流程 / URC 扫描
 * 于是出现了一整串现场故障:
 *   · 云端下发是【原始二进制】payload, 里面含 0x00 / 0x0A / 0x0D。谁先按"行"读,
 *     就把 payload 劈开, 后半截被当垃圾丢掉 —— "云端下发收不到";
 *   · 等 AT 应答的循环把不认识的完整行读掉就丢, 只有含 +MQTTURC: 的行被"抢救"
 *     到 512 字节暂存区, payload 字节全丢。这条"没有 payload 的 publish 头"再被
 *     塞回缓冲时, 解析器会按 <payload_len> 去后面凑字节 —— 吃到下一条 URC,
 *     从此【永久失步】, 连 +MQTTURC: "conn"/"puback" 也一起被吃:
 *     表现就是"下发收不到 + 过一会儿自己断线重建";
 *   · 每次发 AT 前 g4_io_drain_rx() 把没成行的字节整块扔掉;
 *   · 行拼装进度是 g4_io_read_line() 里的 static, 清缓冲时【不复位】,
 *     残留半行和下一行粘在一起。
 *
 * 本版本的设计:
 *   1) 【只有一个消费者】—— g4_io_pump()。它把 rb_g4 的字节喂进一个字节级状态机:
 *          LINE      : 攒一条普通行(CR/LF 结尾) -> 先给 URC 层, 再给 AT 命令引擎
 *          PUB_HDR   : 认出 "+MQTTURC: \"publish\"," 后按【字节】解析头字段
 *          PAYLOAD   : 按 <payload_len> 收【定长原始字节】, 完全不看 CR/LF
 *          RESYNC    : 出错后丢弃到下一个行边界, 保证一定能重新同步
 *      payload 里含 0x0A / 0x00 再也不会把消息劈开或截断。
 *   2) 【收包有超时】: PUB_HDR/PAYLOAD 超过 G4_IO_RX_GAP_TMO_MS 没有新字节就
 *      放弃这条并重同步。这是"永久失步"的解药 —— 丢一个字节只损失一条消息,
 *      不会把整条 AT 流带偏。
 *   3) 【AT 收发改成非阻塞】: g4_io_cmd_run() 每次只推进一步, 不睡等。
 *      主循环每拍都能回到泵上, 环形缓冲不会被"阻塞窗口"灌爆。
 *   4) 【收包路径日志限流】: 115200 下逐字等 TXC 打日志本来就与来数据同速,
 *      下行一快日志自己就是瓶颈。现在每 100 ms 最多打 12 行, 丢掉的计数可见。
 *
 * 数据通路:
 *   发送  g4_io_send_all -> g4_uart_send_bytes (阻塞写 USART2)
 *   接收  USART2 RXDNE 中断 -> rb_g4 -> g4_io_pump() -> 行/payload -> URC 回调 + AT 引擎
 *
 * 注: 原 USB CDC 方案已停用(模组固件不放行 USB AT 口), 函数名里的 "cdc" 仅为兼容保留。
 */

/* ============================ 统一接收泵 ============================ */

/**
 * @brief 推进接收: 把 rb_g4 里的字节全部消费掉(非阻塞, 耗尽即返回)
 *
 * 【必须频繁调用】: 主循环每拍(g4_at_poll 开头)调一次; 任何"等 AT 应答"的地方
 * 也由命令引擎内部调用。可安全重入 —— 泵内会置忙标志, 递归调用直接返回。
 */
void g4_io_pump(void);

/** @brief 当前是否正在泵内(URC 回调里判断"能不能阻塞等 AT"用) */
bool g4_io_in_pump(void);

/**
 * @brief 注册 URC 消费回调(由 g4_urc 注册)
 * @param on_line 收到一条完整行。返回 true = 这一行被 URC 层消费(不再参与 AT 应答匹配);
 *                false = 交给 AT 命令引擎继续匹配
 * @param on_pub  收到一条完整的 publish payload(定长二进制, 已按 <payload_len> 收全)
 */
typedef bool (*g4_io_urc_line_cb_t)(const char *line, uint16_t len);
typedef void (*g4_io_urc_pub_cb_t)(const char *topic, const uint8_t *data, uint16_t len);

void g4_io_set_urc_cb(g4_io_urc_line_cb_t on_line, g4_io_urc_pub_cb_t on_pub);

/* ============================ 非阻塞 AT 命令引擎 ============================
 *
 * 用法(状态机里):
 *     g4_cmd_res_t r = g4_io_cmd_run("AT+CSQ", "+CSQ:", 1000u);
 *     if (r == G4_CMD_RUNNING) { return; }          // 下一拍继续, 状态不变
 *     // 到这里 r 属于 {OK, ERR, TIMEOUT}; 用 g4_io_cmd_resp() 拿累积应答
 *
 * 语义:
 *   · 第一次调用发出命令并返回 RUNNING; 之后每次调用只推进一步;
 *   · 返回终态后引擎回到空闲, 但【结果快照】一直保留到下一次命令发出为止,
 *     所以调用方可以放心地紧接着发下一条命令, 再回来读上一拍的结果。
 */

typedef enum
{
    G4_CMD_RUNNING = 0,   /* 在途, 下一拍继续 */
    G4_CMD_OK,            /* 命中关键字 */
    G4_CMD_ERR,           /* 模组回 ERROR / +CME ERROR */
    G4_CMD_TIMEOUT,       /* 超时(关键字为 NULL 的"只发不判"也走这里) */

    /* ★等 '>' 提示时先收到 OK —— 提示符丢了★
     *
     * 二段式发布(AT+MQTTPUB 省略 payload)在 '>' 之前【不该】有 OK。
     * 一旦出现, 说明模组的 '>' 没送到我们这里(固件 URC 与应答抢串口资源时会丢),
     * 而它【可能已经进了数据模式在等 payload】—— 那之后我们每发一条 AT 都会被它
     * 当成 payload 吃掉, AT 流就脏了。
     * 所以拿到这个结果【不要】再走 AT 层重试, 直接断电重启模组(断电是唯一不依赖
     * AT 通道的恢复手段)。见 g4_at.c 中 pub_poll 里 PUB_CMD_SENT 的处理。 */
    G4_CMD_NOPROMPT,

    G4_CMD__MAX           /* 只为占位, 不要用 */
} g4_cmd_res_t;

/* ==================== 命令引擎的"属主"(★2026-10-07★) ====================
 *
 * 【为什么必须加这个】引擎只有一条, 而 g4_at 的发布子流程和 ota_http 会同时用它。
 * g4_io_cmd_poll() 里那份【唯一的结果快照 + 超时判定】对两个调用方是共享的:
 *
 *     A 发了一条命令(超时 10s)
 *       -> B 也来 poll
 *          -> 到点后 poll 把 s_cmd_result 置成 TIMEOUT 并 s_eng = IDLE(A 的命令被掐掉)
 *          -> B 拿到这个 TIMEOUT, 当成【自己】命令的失败
 *          -> A 下一拍发现引擎 IDLE, 只能重发
 *
 * 2026-10-07 OTA 现场实测就是这个:(日志 log/新建 文本文档.txt)
 *   15:40:03.809  OTA 发 AT+MHTTPCFG="ssl",0,1,0(超时 10s)
 *   15:40:03.828  心跳/事件发布来抢引擎(它自己那条 AT+MQTTPUB 【根本没发出去】)
 *   15:40:13.814  [G4] PUB fail(等 '>' 超时) —— 正好是 03.809 + 10.005s
 *                 = 发布流程把 OTA 的 10s 超时当成了自己"等 '>'"的失败
 *   15:40:28/43   再来两次 -> 连续 3 次 -> 判"链路已断" -> 4G_RST 断电重启模组
 *   15:40:44~47   OTA 完全不知情, 继续往正在重启的模组发 CFG -> +CME ERROR: 652
 *                 -> ota.start 失败(2002), percent 永远是 0
 *
 * 所以给命令打一个属主: 【只有开这条命令的人能取走它的结果、也只有他能触发它的超时】。
 * 外人来 poll 只会拿到 G4_CMD_RUNNING(="还没轮到你"), 既不偷结果也不掐超时。
 *
 * 【兼容性】老调用方继续用 g4_io_cmd_run()(属主 = G4_OWNER_NONE), 行为一字不变:
 *   两个 NONE 调用方之间照旧(它们本来就靠"ONLINE 稳态一条 AT 都不发"错开)。
 *   只有【被明确标了属主】的命令会被保护起来 —— 现在只有 ota_http.c 标了自己。 */
typedef enum
{
    G4_OWNER_NONE    = 0,   /* 未标属主(老调用方: g4_at 的状态机与发布子流程) */
    G4_OWNER_OTA     = 1,   /* app/ota/ota_http.c */
    G4_OWNER_CONSOLE = 2    /* 日志口手敲的 #at(台架调试, 自己一个属主) */
} g4_cmd_owner_t;

/**
 * @brief 发一条 AT 命令并等关键字(非阻塞)
 * @param cmd        命令(不含 CRLF); NULL = 不发送, 只从当前时刻开始等关键字
 *                   (发布流程发完 payload 之后就用 NULL 起等)
 * @param keyword    期望命中的关键字; NULL = 只发不判, 到点返回 G4_CMD_TIMEOUT
 * @param timeout_ms 超时
 */
g4_cmd_res_t g4_io_cmd_run(const char *cmd, const char *keyword, uint32_t timeout_ms);

/**
 * @brief 同 g4_io_cmd_run(), 但带属主 —— 见上面"命令引擎的属主"整段说明
 * @param owner 调用方身份。引擎上的命令不属于本人时, 本函数【不偷结果、不判超时】,
 *              一律返回 G4_CMD_RUNNING(调用方下一拍再来)。
 *              ⚠ 标了属主之后, 调用方【必须持续 poll 到终态】—— 结果只等他一个人;
 *                进入不再 poll 的状态前请调 g4_io_cmd_release() 交还引擎。
 */
g4_cmd_res_t g4_io_cmd_run_owned(const char *cmd, const char *keyword,
                                 uint32_t timeout_ms, g4_cmd_owner_t owner);

/**
 * @brief 当前【持有】引擎的属主; 引擎空闲(IDLE)时返回 G4_OWNER_NONE
 * @note  "有没有被别人占着"就问这个 —— 发布子流程靠它区分
 *        "我这条发出去了在等应答" 和 "我还没发出去, 在排队等引擎"。
 */
g4_cmd_owner_t g4_io_cmd_owner(void);

/**
 * @brief 交还引擎: 若当前属主是 owner 且命令还没被取走, 把它收尾成 IDLE
 * @note  给"结果只等我一个人"的属主用(例如 ota_http 进入 READY/FAILED 之后
 *        不再 poll 引擎了)。不调也不会立刻出错, 但会让别的调用方一直等到天荒地老。
 */
void g4_io_cmd_release(g4_cmd_owner_t owner);

/**
 * @brief 引擎"被真正占用过"的单调递增计数(只增, 不回绕处理)
 *
 * 调用方在发命令【前后各读一次】就能判断: 我这条命令到底写进串口没有。
 *
 * 【为什么需要它】引擎有 IDLE/RUNNING/DONE 三态, 而"拿到一个终态结果"并不等于
 * "这条命令是我发的、而且发出去了"。2026-10-07 现场(16:06):
 *   OTA 的 AT+MHTTPCFG="ssl" 那一步【整个被跳过】-> 实例上没配 ssl ->
 *   后面拿 https 去请求 -> 模组回 +CME ERROR: 655(SSL 未使用) -> 升级失败 2002。
 *   而 ota_http 的 conn_cmd() 之所以会"没发出去也当成功、把步骤推进"，
 *   就是因为只看返回码、看不出"这一条根本没发"。
 * 有了这个计数, 调用方就能加一道安全网: 【本次调用没让它自增 => 命令没发出去 =>
 * 这个终态结果不是我的, 绝不拿它推进状态机】。
 */
uint32_t g4_io_cmd_tx_seq(void);

/**
 * @brief 【不等结果】先把命令发出去, 由调用方自己决定什么时候开始等
 *
 * 专门给 MQTT 发布这种"命令头和数据要分成两次发送"的流程用:
 * 发完 payload 之后必须【立刻】进入等 OK 的状态, 中间空一拍就可能把模组的
 * OK 漏在命令窗口之外。cmd 传 NULL = 不发送, 只登记"从现在开始等 keyword"。
 *
 * @note keyword 必须是【静态存储期】的字符串(字面量), 引擎只存指针不拷贝。
 */
void g4_io_cmd_begin(const char *cmd, const char *keyword, uint32_t timeout_ms);

/**
 * @brief 等模组的【数据模式提示 '>'】—— 严格的整行判定
 *
 * 老版本用 strchr(line, '>'), 云端下发的二进制 payload 里只要有一个 0x3E('>')
 * 就会在模组还没进数据模式时把 payload 当 AT 数据发出去, 结果被模组当命令解析:
 * 下一条 AT+MQTTPUB 会和这堆垃圾粘在一起 -> ERROR / AT 引擎乱掉 -> 掉线。
 * 这里只认"整行只有 '>' 和空白"的行。
 */
g4_cmd_res_t g4_io_cmd_run_prompt(const char *cmd, uint32_t timeout_ms);

/** @brief 只推进当前在途命令; 空闲时返回 G4_CMD_OK 且不做任何事 */
g4_cmd_res_t g4_io_cmd_poll(void);

/** @brief 发命令但不等关键字, 只等 wait_ms 让模组消化(结果总是 TIMEOUT) */
g4_cmd_res_t g4_io_cmd_run_silent(const char *cmd, uint32_t wait_ms);

/** @brief 当前是否有在途命令 */
bool g4_io_cmd_active(void);

/**
 * @brief 命令引擎是否【真正空闲】—— 既没有在途命令, 也没有"没被取走的结果"
 *
 * 【为什么必须单列这一条, 不能拿 g4_io_cmd_active() 当"能用了吗"】
 * 引擎有三个状态: IDLE / RUNNING / DONE。而 g4_io_cmd_active() 只报 RUNNING,
 * 于是 DONE(命令已完成、结果还没被调用方取走)会被误判成"空闲"。
 * 此时调 g4_io_cmd_run() 【不会发送新命令】—— 它按规定把上一条的结果交给你
 * (见 g4_io_cmd_run 里那段 ★关键★ 注释)。对"我只想读一个数据"的调用方来说,
 * 那等于把【别人的应答】当成自己的结果: 假数据比没数据更糟。
 *
 * 所以任何"要发一条自己的命令并解析它自己的应答"的路径, 必须先确认本函数
 * 返回 true(或用 g4_io_cmd_poll() 把 DONE 排掉), 再发命令。
 * 2026-10-04 的驻网(CEREG)只读轮询就是踩在这个点上才加的它。
 */
bool g4_io_cmd_idle(void);

/** @brief 作废在途命令(状态切换/模组重启时调) */
void g4_io_cmd_abort(void);

/** @brief 上一条命令的累积应答('\n' 分隔, 以 '\0' 结尾; 结果快照, 不会中途失效) */
const char *g4_io_cmd_resp(void);

/** @brief 上一条命令命中关键字的那一行(结果快照) */
const char *g4_io_cmd_line(void);

/* ============================ 传输层基础 ============================ */

/** @brief 初始化 4G AT 串口(USART2 PC8/PC9, 115200)及接收缓冲/状态机 */
void g4_io_init(void);

/** @brief 保留接口(接收由中断+泵完成, 此处为空) */
void g4_io_poll(void);

/** @brief CDC 是否就绪(串口方案恒为 true, 兼容旧调用) */
bool g4_io_cdc_ready(void);

/** @brief USB CDC 接收回调占位(串口方案不使用, 保留符号兼容 usbh_user.c) */
void g4_io_receive_cb(uint8_t *data);

/** @brief 把数据全部写入 AT 口 */
bool g4_io_send_all(const uint8_t *data, uint16_t len);

/** @brief 发送一条 AT 命令(自动追加 CRLF) */
void g4_io_send_cmd(const char *cmd);

/** @brief 清空接收缓冲并复位收包状态机(断电/重同步用) */
void g4_io_drain_rx(void);

/* ---- 阻塞封装: 只给"日志口手工诊断"和必须同步拿结果的地方用 ----
 *
 * 【不要在 URC 回调里调】(那是在泵内部), 会直接返回失败并打一行日志。
 * 状态机内部请用上面的非阻塞 g4_io_cmd_run()。 */

/**
 * @brief 发命令并【阻塞】等关键字(内部就是在循环里跑泵 + 非阻塞引擎)
 * @return true 命中; false 超时/ERROR/嵌套调用
 */
bool g4_io_send_expect(const char *cmd, const char *keyword, uint32_t timeout_ms,
                       char **got_line);

/**
 * @brief 同 g4_io_send_expect, 但把【整段应答】累积给调用方(仿 ml307_Expect)
 *        AT+CIMI / AT*ICCID? 这类"应答本身就是数据"的命令必须用它。
 * @param resp 应答累积缓冲(可空); resp_sz 字节数
 */
bool g4_io_send_expect_resp(const char *cmd, const char *keyword, uint32_t timeout_ms,
                            char **got_line, char *resp, uint16_t resp_sz);

/* ============================ 诊断计数 ============================ */

/** @brief 上电以来 AT 口累计收到的字节数(中断里自增, 只增不减) */
uint32_t g4_io_rx_total(void);
/** @brief 清零累计接收计数 */
void g4_io_rx_reset(void);

/** @brief 环形缓冲写满而【丢掉的字节数】(中断里统计; 0 = 从没溢出) */
uint32_t g4_io_rx_ovf_count(void);

/** @brief URC 收包中途超时/畸形而放弃并重同步的次数(>0 说明下行曾经被截断) */
uint32_t g4_io_dl_abort_count(void);

/** @brief 被限流丢掉的收包路径日志行数(排"日志是不是瓶颈"用) */
uint32_t g4_io_rxlog_drop_count(void);

/** @brief 开机以来【离上次发送 >G4_READY_AFTER_TX_MS】才冒出来的 "+MATREADY"/"RDY"
 *
 * ⚠️ 【别拿这个计数当"模组重启次数"】—— 现场日志 78.txt 就是被它骗的:
 *    注册阶段的轮询间隔(AT_CEREG_POLL_MS = 2000ms)比这个窗口(3000ms)还短,
 *    于是那段时间里每一条 +MATREADY 都会被算进 after_tx 那一档, 真复位也数不到,
 *    [STAT2] 因此一路显示 "模组自复位(无缘无故)=0", 而 +MATREADY 已经出现了 4 次。
 *    现在每一条 +MATREADY 都会打一行带"距上次发送 Xms"的日志 ——
 *    排查请【看那行日志】, 不要只看这个计数。
 */
uint32_t g4_io_ready_urc_count(void);

/** @brief 离最近一次发送 ≤ G4_READY_AFTER_TX_MS 的 "+MATREADY"/"RDY" 次数
 *         (模组确实有"一条 AT 补一条 +MATREADY"的习惯, 但这一档里也混着真复位) */
uint32_t g4_io_ready_after_tx_count(void);

/** @brief 两项之和 = 开机以来 +MATREADY/RDY 的【总次数】
 *         (WAIT_MODULE 判"模组开口了没有"用; 这个总数才是可信的那个) */
uint32_t g4_io_ready_total(void);

#endif /* __G4_IO_H__ */
