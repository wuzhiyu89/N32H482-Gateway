#ifndef __PROJECT_CONFIG_H__
#define __PROJECT_CONFIG_H__

//基于semtech官网驱动移植
//官网驱动网址 https://github.com/Lora-net/LoRaMac-node/tree/master/src/radio  下载日期 2021/2/3
#define SOFT_VERSION	"LLCC68 driver for stm32f103 V0.0.0"

//--------------------------------------------- 测试默认配置 ---------------------------------------------
#define LORA_FRE 433000000               // 收发频率
/* 发射功率：本模块芯片自报 SX1261(版本串 0x0320="SX1261")，SX1261 分支上限就是 +14dBm。
 * 2026-09-28 从 22 改回 14（本机 #lora 已确认走 SX1261 分支，22 会被钳成 14，功能一样；
 * 写 14 是为了配置与注释一致，避免以后换芯片时踩 PA 分支的坑）。 */
#define LORA_TX_OUTPUT_POWER 14
/* ★ 2026-09-28 空口速率调慢一档，补回约 +8.5dB 链路余量 ★
 *   现场实测: 本机收液位时 SNR≈-13dB，而 SF8/BW500k 的解调门限约 -7dB → 差约 6dB，
 *   现象是"20 字节里偶尔错 1 个字节、CRC 不过 → 判坏帧丢掉"(网关侧 [RAW-RX] CRC错包)。
 *   500k → 125k : 带内噪声 -6dB      -> +6.0 dB
 *   SF8  → SF9  : 处理增益 +2.5dB    -> +2.5 dB
 *                         合计约       +8.5 dB
 *   ⚠️ 必须与对端(液位节点)【完全一致】：节点已同步改为 LORA_BANDWIDTH 0 / SF9。
 *   ⚠️ 空口时间变长(20字节 ~40ms → ~280ms)，本工程周期是秒/分钟级，不受影响。
 *   ⚠️ 声光报警器(310001)如果也是这套空口参数，必须同步改。 */
#define LORA_BANDWIDTH 0                // [ 0: 125 kHz,	测试默认使用的带宽，LLCC68：[0: 125 kHz,1: 250 kHz,2: 500 kHz,3: Reserved]
#define LORA_SPREADING_FACTOR 9          // 测试默认使用的扩频因子范围7~12
#define LORA_CODINGRATE 2                // 测试默认使用的纠错编码率[1: 4/5,2: 4/6,3: 4/7,4: 4/8]
#define LORA_PREAMBLE_LENGTH 10           // 前导码长度
#define LORA_LLCC68_SYMBOL_TIMEOUT 0     // Symbols(LLCC68用到的是0,127x用到的是5)
#define LORA_FIX_LENGTH_PAYLOAD_ON false // 是否为固定长度包(暂时只是LLCC68用到了)
#define LORA_IQ_INVERSION_ON false       // 这个应该是设置是否翻转中断电平的(暂时只是LLCC68用到了)
#define LORA_RX_TIMEOUT_VALUE 5000
#define TCXO_CTRL_VOLTAGE 2             // setting TCXO voltage supply source [0: 1.6v, 1: 1.7v, 2: 1.8v, 3: 2.2v, 4: 2.4v, 5: 2.7v, 6: 3.0v, 7: 3.5v]
#define RF_WAKEUP_TIME 100              // setting RF wakeup time,in second
#define LORA_DATA_SEND_OR_RECEIV_MODE 0 // 数据收发模式，1为发送模式，0为接收模式

/*!
 * Board MCU pins definitions
 */
//SPI引脚(SPI3: SCK=PC10 MISO=PC11 MOSI=PC12, 均为AF1; NSS=PA15软件控制)
#define RADIO_NSS_PIN       GPIO_PIN_15
#define RADIO_NSS_PORT      GPIOA
#define RADIO_MOSI_PIN      GPIO_PIN_12
#define RADIO_MOSI_PORT     GPIOC
#define RADIO_MISO_PIN      GPIO_PIN_11
#define RADIO_MISO_PORT     GPIOC
#define RADIO_SCK_PIN       GPIO_PIN_10
#define RADIO_SCK_PORT      GPIOC
//RST复位脚
#define RADIO_nRESET_PIN    GPIO_PIN_2
#define RADIO_nRESET_PORT   GPIOD
//DIO1 引脚
#define RADIO_DIO1_PIN      GPIO_PIN_4
#define RADIO_DIO1_PORT     GPIOB
//BUSY 引脚
#define RADIO_DIO4_BUSY_PIN      GPIO_PIN_3
#define RADIO_DIO4_BUSY_PORT     GPIOB

//  EN 引脚 高电平lora上电，低电平lora下电，设置为推挽输出，先不管，这是后续的功能，用于直接断掉lora的供电
#define RADIO_EN_PIN      GPIO_PIN_5
#define RADIO_EN_PORT     GPIOB

//下面这几个引脚没用用到，设置为浮空输入模式
//TXEN 引脚
#define RADIO_DIO0_TXEN_PIN      GPIO_PIN_NONE
#define RADIO_DIO0_TXEN_PORT     GPIOB
//DIO2
#define RADIO_DIO2_PIN      GPIO_PIN_NONE
#define RADIO_DIO2_PORT     GPIOB
//DIO3
#define RADIO_DIO3_PIN      GPIO_PIN_NONE
#define RADIO_DIO3_PORT     GPIOB
//RXEN
#define RADIO_DIO5_RXEN_PIN      GPIO_PIN_NONE
#define RADIO_DIO5_RXEN_PORT     GPIOB


/* ============================================================================
 *          【最小 LoRa 收发隔离测试开关】MAIN_LORA_RAW_TEST
 * ----------------------------------------------------------------------------
 * 1 = 只跑"5 秒发两条 + 其余时间一直连续接收 + 在【射频 IRQ 寄存器层】把收到的
 *     原始字节原样打出来"的最小射频测试; 4G / 业务层 / 协议层 / 参数 Flash /
 *     心跳 全部让开(见 src/main.c 末尾的 rawtest_run() 与 main() 的 #if/#else)。
 * 0 = 正常固件 —— 与加这个开关之前【完全一致】(测试代码整体不参与编译)。
 *
 * 【为什么这个开关写在本文件, 而不是 src/main.c】
 *   它必须被【三个不同的编译单元】看到:
 *     src/main.c                  —— 测试主循环(发/收/打印累计计数)
 *     app/protocol/lora_proto.c   —— 生产射频初始化 + 生产发送路径(被测试复用)
 *     app/radio/llcc68/radio.c    —— IRQ 寄存器层/原始字节的原样打印
 *   C 语言的宏【不跨 .c 传递】: 只写在 main.c 里, 另外两个 .c 永远看到"未定义
 *   (按 0 处理)" —— 在 main.c 里把 0 改成 1 会【静默无效】(测试模式的打印点
 *   一个都不生效, 现场会误判成"射频一个字节都没收到")。
 *   而本文件是这三个编译单元都已经包含的射频工程配置头
 *   (lora_proto.h -> 本文件; radio.c / LLCC68N32H482-board.c 直接包含),
 *   放在这里就是唯一的真相源, 改一处即可。
 *
 * 【★怎么切回正常固件★】
 *   把下面的 1 改成 0, 重新编译即可(也可以在 Keil/IDE 的 C/C++ -> Define 里
 *   定义 MAIN_LORA_RAW_TEST=0 覆盖本文件 —— 本开关是 #ifndef 保护的)。
 *
 * 【对本文件其它内容零影响】
 *   本开关【不修改任何射频配置值】: 频率/SF/带宽/编码率/前导码/报头/CRC/功率/
 *   同步字 一律不动 —— 本次测试的目的就是"用现状去测"。
 *
 * ★★★ 归档版(v30)这里就是 1 ★★★
 *   本目录归档的 CDC.bin【就是 MAIN_LORA_RAW_TEST=1 的测试固件】。
 *   形态 0 也已实际编译验证过: Program Size 与基线逐项一致
 *   (Code=76412 RO-data=31416 RW-data=176 ZI-data=76496), 详见说明.txt。
 * ============================================================================ */
/* ==================== 主功能模式(三选一) —— ★ 要切模式就改这里 ★ ====================
 *
 * 【为什么放在本文件】模式宏必须在【任何 #include "project_config.h" 之前】就已确定,
 *   否则本文件下面那句"新宏 -> 老宏"的映射就看不到它。
 *   2026-09-28 的现场问题正是如此: 把 MAIN_MODE_LORA_TEST 写在 main.c 里 ——
 *   而 main.c 是【先包含本文件、后定义模式宏】的, 于是映射没生效,
 *   结果"想进 LoRa 测试模式, 却编成了云联调模式(4G 照样初始化)"。
 *   ★ 所以: 模式开关请改【本文件】, 或改 Keil → Options for Target → C/C++ → Define。★
 *
 *   1 = 4G + LoRa + 云端联调(现场默认)
 *   1 = 从机配对模式(M)  1 = LoRa 收发测试(M)
 *   三者只能有一个为 1; CLOUD 是兜底: 另两个都没设时它才默认 1。
 */
/* ---- 老宏先取默认值: 它必须在下面"老宏提升为模式③"之前就可见 ---- */
#ifndef MAIN_LORA_RAW_TEST
#define MAIN_LORA_RAW_TEST   0     /* 1 = 只跑 LoRa 最小收发测试(老开关, 仍兼容) */
#endif

/* ---- 三个模式宏的默认值 ---- */
#ifndef MAIN_MODE_PAIRING
#define MAIN_MODE_PAIRING        0     /* 1 = 从机配对模式 */
#endif
#ifndef MAIN_MODE_LORA_TEST
#define MAIN_MODE_LORA_TEST      0     /* ★ 默认必须是 0 ★
                                        * 这里写成 1 时, "不加任何宏"就会编成 LoRa 测试
                                        * 模式(与"默认云联调"的承诺不符); 而且"只加一个
                                        * MAIN_MODE_CLOUD=1 切回云联调"会撞下面的三选一
                                        * 自检直接编译失败 —— 现场就是这么被卡住的。
                                        * (2026-09-29 从 1 改回 0) */
#endif

/* ---- 向后兼容: 老的 MAIN_LORA_RAW_TEST=1 单独使用也要能选中模式③ ---- */
#if (MAIN_LORA_RAW_TEST != 0) && (MAIN_MODE_LORA_TEST == 0)
#undef  MAIN_MODE_LORA_TEST
#define MAIN_MODE_LORA_TEST      1
#endif

/* ---- CLOUD 是兜底: 另两个都没选时才为 1 ----
 * 这样"只加一个新宏"(MAIN_MODE_PAIRING=1 或 MAIN_MODE_LORA_TEST=1)就能切走,
 * 不必回头再把 MAIN_MODE_CLOUD 改成 0。 */
#ifndef MAIN_MODE_CLOUD
#  if ((MAIN_MODE_PAIRING != 0) || (MAIN_MODE_LORA_TEST != 0))
#    define MAIN_MODE_CLOUD      0
#  else
#    define MAIN_MODE_CLOUD      1     /* 兜底: 不加任何模式宏 = 云联调(现场行为) */
#  endif
#endif

/* ---- 三选一自检: 放在本文件, 于是【每个包含它的编译单元】都会检查 ---- */
#if ((MAIN_MODE_CLOUD + MAIN_MODE_PAIRING + MAIN_MODE_LORA_TEST) != 1)
#error "主功能模式必须【恰好选一个】(MAIN_MODE_CLOUD / PAIRING / LORA_TEST), 见 app/radio/llcc68/project_config.h"
#endif

/* ---- 模式名字符串: 开机横幅打出来, 一眼确认"这次到底编进了哪个模式" ---- */
#if MAIN_MODE_PAIRING
#define MAIN_MODE_NAME   "配对模式(MAIN_MODE_PAIRING)"
#elif MAIN_MODE_LORA_TEST
#define MAIN_MODE_NAME   "LoRa收发测试(MAIN_MODE_LORA_TEST)"
#else
#define MAIN_MODE_NAME   "云端联调(MAIN_MODE_CLOUD)"
#endif

/* ★ 模式③ 需要那些"测试专用代码段" (2026-09-28) ★
 * radio.c / lora_proto.c / main.c 里的 rawtest 段都按老宏 MAIN_LORA_RAW_TEST 分隔,
 * 而这几个文件是【各自单独编译】的 —— 所以必须在这一处(大家都包含的头文件)把新宏
 * 映射回老宏, 否则会出现"主流程进了测试模式, 射频层的测试代码却没被编译"的不一致。
 */
#if (MAIN_MODE_LORA_TEST != 0) || (MAIN_LORA_RAW_TEST != 0)
#undef  MAIN_LORA_RAW_TEST
#define MAIN_LORA_RAW_TEST   1
#endif


#endif // __PROJECT_CONFIG_H__
