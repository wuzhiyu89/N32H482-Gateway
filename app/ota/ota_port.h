#ifndef __OTA_PORT_H__
#define __OTA_PORT_H__

#include <stdint.h>
#include <stdbool.h>
#include "ota_meta.h"
#include "ota_boot.h"   /* ota_meta_t 定义在此(ota_port 持久化时引用) */
#include "flash_map.h"  /* 分区地址唯一真相源 */

/**
 * \file  ota_port.h
 * \brief OTA 移植层接口(与硬件/平台解耦)
 *
 * ota.c 只调用本接口,不直接碰 Flash 寄存器/CRC 外设/NVIC。
 * ota_port.c 提供 N32H482 实现(用固件库 FLASH_EraseOnePage /
 * FLASH_ProgramdoubleWord + 软件 CRC32 + NVIC_SystemReset)。
 *
 * Flash 布局: 见 app/common/flash_map.h (唯一真相源), 本文件只做别名。
 *   运行区       FM_APP_ADDR      (bootloader 决定 app 偏移)
 *   OTA 缓存区   FM_STAGE_ADDR    (双 Slot 模式下即 Slot B)
 *
 * 切换启动逻辑依赖 bootloader 是否存在:
 *   - 无 bootloader: ota_port_reboot() 直接复位, 由 app 在启动时检测缓存区
 *                    有效并自更新(本框架不实现,留接口)
 *   - 有 bootloader: ota_port_reboot() 进入 bootloader, 由 bootloader 完成搬运
 *
 * 当前阶段: ota_port.c 提供可工作的实现,但 reboot 后的"切换"逻辑
 *          属于 bootloader 范畴, 不在本框架内。
 */

/* ==================== Flash 布局配置 ====================
 * ⚠ 分区地址已全部收拢到 app/common/flash_map.h (唯一真相源)。
 *   要改分区请【只改 flash_map.h】, 并同步 App 的链接地址(见该文件头部注释)。
 *   下面 4 个宏保留原名只是为了不破坏既有调用点, 其值全部来自 flash_map.h。
 *
 * 与 param_save.c / storage.c 的隔离:
 *   - 地址完全不重叠(参数页 FM_PARAM_ADDR, 暂存区 FM_STAGE_ADDR 起)
 *   - ota_port.c 直接复用 storage.c 的 Flash 读写接口(带中断保护 + Unlock/Lock),
 *     因此与 param_save 并发时共享同一套 Unlock/Lock, 不会互相踩寄存器
 *
 * Bootloader 搬运安全策略:
 *   1. 先校验暂存区的 CRC32 与 mark 里的 crc32 一致后, 才擦除 app 区
 *      —— 如果先擦后发现 CRC 不对, 设备就砖了
 *   2. bootloader 只擦写 FM_APP_ADDR 以后的页, 绝不碰自己所在的低地址区
 *   3. 搬运完成后擦除 mark 页(mark_invalid), 防止下次重启重复搬运
 *   4. 跳转前设 SCB->VTOR = FM_APP_ADDR, MSP/PC 取自该处向量表
 *   5. bootloader 源码位于独立目录 app/bootloader/, 与 app 工程解耦
 *
 * 标记区布局(FM_META_ADDR 起, 32 字节有效, 剩余保留):
 *   [0..3]   magic    "OTAV" (0x4F 0x54 0x41 0x56)
 *   [4..7]   crc32    整个固件 CRC32
 *   [8..11]  size     固件总字节数(小端)
 *   [12..15] reserved  0
 *   [16..31] reserved  0
 *   无 magic(全 0xFF 或其他值) = 暂存区无效 = bootloader 跳过
 */
#ifndef OTA_FLASH_ADDR
#define OTA_FLASH_ADDR      FM_STAGE_ADDR
#endif
#ifndef OTA_FLASH_SIZE
#define OTA_FLASH_SIZE      FM_STAGE_SIZE
#endif
#ifndef OTA_FLASH_PAGE_SIZE
#define OTA_FLASH_PAGE_SIZE FM_PAGE_SIZE
#endif
/* 标记区地址 = flash_map.h 中的标记页(值与原 OTA_FLASH_ADDR+OTA_FLASH_SIZE 相同) */
#define OTA_META_ADDR       FM_META_ADDR

/* ==================== Flash 操作 ==================== */

/**
 * @brief 擦除 OTA 缓存区指定页数(从 OTA_FLASH_ADDR 起)
 * @param pages 页数(每页 OTA_FLASH_PAGE_SIZE)
 * @return true 全部擦除成功
 * @note  单次调用应非阻塞分步,否则 ota_poll 无法推进。
 *        本实现采用"按页擦除,每次 ota_port_flash_erase_step 推进一页"。
 */
bool ota_port_flash_erase_begin(uint32_t pages);

/**
 * @brief 同 erase_begin, 但从第 first_page 页开始擦(擦 [first_page, first_page+pages) )
 * @param first_page 起始页序号(相对 OTA_FLASH_ADDR), 0 = 与 erase_begin 等价
 * @param pages      要擦的页数
 *
 * 【为什么需要它】: OTA over HTTP 的掉电续传。重启后已经下好的前缀还在 Flash 里,
 * 如果还从第 0 页擦起就把前缀也擦了, "从断点续"就无从谈起。
 * 见 OTA_HTTP_设计_v1.md §6bis。
 */
bool ota_port_flash_erase_begin_at(uint32_t first_page, uint32_t pages);

/**
 * @brief 推进擦除(每调用一次擦一页)
 * @return true 全部擦除完成;false 仍在进行中
 * @note  若某页擦除失败,本函数仍会推进到下一页(快速跑完),
 *        最终返回 true(完成)后,必须调用 ota_port_flash_erase_ok()
 *        检查是否有失败,若 false 应立即中止升级。
 */
bool ota_port_flash_erase_step(void);

/**
 * @brief 擦除是否全部成功(在 erase_step 返回 true 后调用)
 * @return true 所有页擦除成功;false 有至少一页擦除失败
 */
bool ota_port_flash_erase_ok(void);

/**
 * @brief 写 Flash(地址必须已擦除,自动按 8 字节对齐补 0xFF)
 * @param offset 相对 OTA_FLASH_ADDR 的偏移
 * @param data   数据
 * @param len    长度(任意)
 * @return true 成功
 */
bool ota_port_flash_write(uint32_t offset, const uint8_t *data, uint16_t len);

/**
 * @brief 读 OTA 缓存区(用于 CRC 校验)
 */
bool ota_port_flash_read(uint32_t offset, uint8_t *buf, uint16_t len);

/* ==================== CRC32 ==================== */

/**
 * @brief CRC32 累加(IEEE 802.3, 多项式 0xEDB88320, 与 zip/zlib 一致)
 * @param init   上次结果(首包传 0xFFFFFFFF)
 * @param data   数据
 * @param len    长度
 * @return 累加结果(最终需异或 0xFFFFFFFF, 由 ota.c 完成或调用方完成)
 * @note  本接口不自动异或,便于流式处理;最终值 = crc ^ 0xFFFFFFFF
 */
uint32_t ota_port_crc32_update(uint32_t init, const uint8_t *data, uint16_t len);

/* ==================== CRC16 (Modbus, 多项式 0xA001) ==================== */

/**
 * @brief CRC16 计算(Modbus 标准, 多项式 0xA001)
 *        用于 OTA 包头校验(协议中 crc16 字段填 0 表示跳过校验)
 * @param data 数据
 * @param len  长度
 * @return CRC16 值(小端序存储,即低字节在前)
 */
uint16_t ota_port_crc16(const uint8_t *data, uint16_t len);

/* ==================== 时间 ==================== */

/**
 * @brief 获取系统毫秒时间戳(用于下载超时检测)
 *        N32H482 实现委托 bsp_tim.get_tick_ms
 */
uint32_t ota_port_get_tick_ms(void);

/* ==================== 升级上下文持久化(断点续传) ====================
 * 用于掉电后恢复升级会话。持久化介质由 ota_port.c 决定:
 *   - 方案A: 写到 OTA 缓存区开头的元数据扇区(需预留 1 页)
 *   - 方案B: 用 Option Bytes(容量有限)
 *   - 方案C: 委托现有 param_save 模块
 * 框架阶段: 提供接口 + RAM 占位实现,实际持久化待对接。
 */
#define OTA_CTX_MAGIC   0x4F544131U   /* "OTA1" */

/**
 * @brief 保存升级上下文到持久化介质(掉电后可恢复)
 * @param meta           当前升级元数据
 * @param recv_packets   已收包数
 * @param bitmap         包接收位图(可 NULL,此时仅保存元数据)
 * @param bitmap_words   位图有效字数(与 ota.c OTA_BITMAP_WORDS 一致)
 * @return true 写入成功
 * @note   写前会自动检测 ctx 区是否已被写过(magic != 0xFFFFFFFF),
 *         若已写过则先擦除整页再写入 —— Flash 只能 1→0 不能 0→1
 */
bool ota_port_save_ctx(const ota_meta_t *meta, uint16_t recv_packets,
                       const uint32_t *bitmap, uint16_t bitmap_words);

/**
 * @brief 加载升级上下文(启动时调用,判断是否有未完成升级)
 * @param meta           输出元数据
 * @param recv_packets   输出已收包数
 * @param bitmap         输出位图缓冲(调用方传入,大小 >= bitmap_words*4B, 可 NULL)
 * @param bitmap_words   输入: 调用方缓冲字数上限; 输出: 实际读出的字数
 * @return true 存在有效未完成上下文;false 无效或不存在
 */
bool ota_port_load_ctx(ota_meta_t *meta, uint16_t *recv_packets,
                       uint32_t *bitmap, uint16_t *bitmap_words);

/**
 * @brief 清除升级上下文(升级完成或中止后调用)
 */
bool ota_port_clear_ctx(void);

/* ==================== OTA 缓存区有效标记(bootloader 检测用) ====================
 * 标记区位于 OTA_META_ADDR(固件缓存区的下一页, 独立 8KB 页)。
 * bootloader 启动时读取该页开头 32 字节判断是否存在有效升级镜像。
 *
 * 标记区(32 字节有效):
 *   [0..3]   magic    "OTAV" (0x4F 0x54 0x41 0x56)
 *   [4..7]   crc32    整个固件 CRC32(小端)
 *   [8..11]  size     固件总字节数(小端)
 *   [12..31] reserved 填 0
 *
 * 无 magic(全 0xFF) = OTA 缓存区无效 = bootloader 跳过。
 */

/**
 * @brief 擦除标记页 + 写有效标记(VERIFYING 成功后调用)
 *        bootloader 据此判断是否需要搬运升级
 * @param meta 升级元数据(用 crc32 / total_size 字段)
 * @return true 写入成功
 */
bool ota_port_mark_valid(const ota_meta_t *meta);

/**
 * @brief 清除有效标记(任意失败/中止时调用)
 *        实现: 擦除标记页(整页 0xFF, magic 自然消失)
 */
void ota_port_mark_invalid(void);

/* ==================== OTA over HTTP 的掉电续传记录 ====================
 *
 * 记录放在标记页的 [256..335], 与 mark_valid 的 [0..31] 和 MQTT 分包版的
 * ota_ctx_storage_t [40..] 互不重叠(标记页 8 KB, 富余很大)。
 *
 * ⚠️ 【关键约束, 用的人必须知道】: mark_valid() / mark_invalid() 都会【擦整页】,
 *    所以它们会顺手把续传记录一起擦掉。这是刻意的语义:
 *      · mark_valid   = 升级已成功 → 记录本就该消失
 *      · mark_invalid = 放弃/失败   → 从 0 重来才是对的
 *    因此 ota_http_start() 必须【先读记录、再调 mark_invalid()】, 顺序不能反。
 *
 * 记录只在下好一部分、且还没校验通过时有效。用户可见的写接口只有 save 一个,
 * 而 save 内部会先擦页 —— 所以它只能在【下载阶段】调用(那时没有有效标记)。
 */
#define OTA_HTTP_RESUME_OFFSET   256u
#define OTA_HTTP_RESUME_MAGIC    0x50545448u   /* "HTTP" */

typedef struct
{
    uint32_t magic;        /* OTA_HTTP_RESUME_MAGIC; 其它值(含 0xFFFFFFFF) = 无记录 */
    uint32_t written;      /* 已写进暂存区的字节数(页对齐) = 续传偏移 */
    uint32_t size;         /* 目标总长 —— 判"是不是同一次升级" */
    char     md5hex[33];   /* 目标 MD5 —— 同上判据 */
    char     version[24];  /* 版本号(日志/上报用) */
    uint8_t  reserved[11]; /* 凑到 80 字节: 8 的倍数, 满足 Flash 编程对齐要求 */
} ota_http_resume_t;

/**
 * @brief 写下/更新续传记录(内部会先擦标记页 —— 只能在下载阶段调)
 * @param written 已写字节数(会被向下页对齐后存)
 * @return true 成功
 */
bool ota_port_http_resume_save(uint32_t written, uint32_t size,
                               const char *md5hex, const char *version);

/**
 * @brief 读回续传记录
 * @param written/size 输出
 * @param md5hex/vercap 输出缓冲及其容量(不足则截断并置 '\0')
 * @return true = 有有效记录; false = 无记录(magic 不对)
 */
bool ota_port_http_resume_load(uint32_t *written, uint32_t *size,
                               char *md5hex, uint16_t md5cap,
                               char *version, uint16_t vercap);

/**
 * @brief 使续传记录失效(只把 magic 清 0, 【不擦页】—— 不会误伤 bootloader 标记)
 */
void ota_port_http_resume_clear(void);

/* ==================== A/B 回滚状态(2026-10-11 新增) ====================
 *
 * 【用途】双槽(App A/B)模式下, 新固件装上后要有个"自证健康"的机制:
 *   启动时给本槽计数, 连续 N 次都没确认健康 -> 请求回滚到另一个槽。
 *   本记录就是那个计数器 + 当前槽号。
 *
 * 【存哪 / 为什么】放标记页(FM_META_ADDR)的 512 偏移 —— 与 OTA 固件标记、
 *   HTTP 续传记录同一页。选它的理由: 回滚是 bootloader 的职责, 状态和 OTA
 *   标记放一起, BL 读起来是最短路径(不用再认识参数页)。
 *
 * 【标记页占用图(8KB)】
 *     [0, 32)        OTA 固件标记 "OTAV"          (ota_port.h 上面那段)
 *     [32, 256)      OTA 上下文(OTA_CTX_*)        (ota_port.c 的断言)
 *     [256, 328)     HTTP 续传记录(72 字节)       (OTA_HTTP_RESUME_OFFSET)
 *     [512, 544)     ★ 本记录 ★                   ← 与上一段留 184 字节余量
 *     [544, 8192)    空闲
 *   下面 ota_port.c 里有 4 条编译期断言把这个不重叠关系钉死 —— 以后谁改
 *   OTA_HTTP_RESUME_OFFSET 或那两个结构体, 编译器会立刻拦住, 不会静默踩坏。
 *
 * ⚠️⚠️ 【已知且已接受的一个交互】⚠️⚠️
 *   ota_port_http_resume_save() 会【擦整个标记页】(见上面它的说明), 所以
 *   每次 OTA 下载都会把本记录一起擦掉, 也就是 try_count 归零。
 *   评估后【接受】这个行为, 理由:
 *     · 调用 resume_save() 意味着正在下载【另一个】固件 —— 既然要换固件,
 *       回滚计数本来就该清零, 这时归零是【对的】不是 bug;
 *     · 真正亏的只有窄场景: 下载后又取消/失败、设备继续跑同一个不健康固件
 *       —— 代价是"回滚晚几次发生", 不是"永不回滚"。可接受。
 *   如果要改成绝对不丢, 得动 ota_port_http_resume_save() 或改存参数页,
 *   两者成本都高于收益, 故不做。
 *
 * ⚠️ 本记录的 save() 只能【写】, 绝不能擦页 —— 擦页会毁掉同页的
 *    OTA 固件标记和续传记录。clear() 只清 magic(与 resume_clear 同模式)。
 * ======================================================================== */
#define OTA_ROLLBACK_OFFSET   512u
#define OTA_ROLLBACK_MAGIC    0x54524241u   /* "ABRT" */

typedef struct
{
    uint32_t magic;        /* = OTA_ROLLBACK_MAGIC; 其它值(含 0xFFFFFFFF) = 无记录 */
    uint8_t  boot_slot;    /* 0 = Slot A, 1 = Slot B —— 当前在跑哪个槽 */
    /* ★★ 为什么是"倒数的位图"而不是普通计数器 ★★
     * Flash 只能 1->0。普通计数器递增(1->2 = 0b01->0b10)需要把 bit0 从 0 变回 1,
     * 而本记录的 save() 【不能擦页】(同页还有 OTA 标记和续传记录)
     * ⇒ 计数器写法根本写不进去, 且会静默失效。
     * 改成"还剩几次机会"的位图后, 每次启动【只多清一个 bit】(1->0, 合法):
     *     0xFF(8次) -> 0xFE(7次) -> 0xFC(6次) -> 0xF8(5次) -> ... -> 0x00(0次)
     *   ⚠️ 注意清的是【最低的 1 位】: `m & (m-1)`。
     *     0xFF & 0xFE = 0xFE(不是 0x7F —— 那是清最高位, 数值上看着更"整"
     *     但没关系, 只要"每次只少一个 1 位"就满足 1->0 约束)。
     *     用 `& (m-1)` 而不是 `m-1`: 减法会产生进位, 可能把 0 变回 1
     *     (例: 0x10-1 = 0x0F, 低 4 位从 0 全变 1 ⇒ 非法)。 */
    uint8_t  tries_left;   /* 位图: 每启动一次清一个 bit; 0x00 = 机会用完, 该回滚 */
    /* ★★ 注意是"未确认"(not confirmed), 不是 confirmed ★★
     * 同理: 确认健康时要把标志"立起来", 若写成 confirmed(0->1) 就又违反 1->0 约束。
     * 反转语义后: 初值 1 = 还没确认 -> 确认时【清成 0】 = 合法。
     * ⇒ 整条记录的所有位在生命周期内【只做 1->0】, "只写不擦页"才真正成立。 */
    uint8_t  unconfirmed;  /* 1 = 本槽还没确认健康; 0 = 已定案 */
    uint8_t  rsv0;
    /* ⚠️【这里刻意没有"校验和"字段】—— 别再加回来。
     * 校验和是整条内容的函数: tries_left 一变校验和就变, 而它的新值【必然包含
     * 0->1 的位】⇒ 在"只写不擦页"下根本写不进去 ⇒ save() 失败/写坏 ⇒ load()
     * 判无效 ⇒ 整个机制失效。
     * 改用【值域校验】代替, 而且更硬: "写到一半掉电"留下的典型特征是 0xFF,
     * 恰好会被 boot_slot<=1 / unconfirmed<=1 挡掉(见 ota_port.c 的 load)。
     * (2026-10-11: 就是因为先写了校验和才在动手前发现这个矛盾, 记在这里。) */
    uint8_t  rsv[24];      /* 凑到 32 字节(8 的倍数, 满足 Flash 双字编程对齐):
                            * 4 + 1 + 1 + 1 + 1 + 24 = 32 */
} ota_rollback_t;

/** 刚开始用本机制时的初值: 8 次机会 */
#define OTA_ROLLBACK_TRIES_INIT   0xFFu
/** 默认允许的尝试次数(每次启动清一位, 所以初值里"1 的个数"就是这个数) */
#define OTA_ROLLBACK_TRIES_MAX    8u

/**
 * @brief 读回 A/B 回滚状态
 * @param out 输出
 * @return true = 有【有效】记录(magic + crc32 都对); false = 无记录/记录损坏
 * @note  false 时调用方应【当作"从未记录"】处理 —— 重新开始计数,
 *        【绝不能】当成"次数已满"去回滚(那会把好固件回滚掉)。
 */
bool ota_port_rollback_load(ota_rollback_t *out);

/**
 * @brief 写入 A/B 回滚状态(只写本记录的 32 字节, 【不擦页】)
 * @return true 成功
 * @note ⚠️ 本函数【不擦页】是刻意的 —— 同页还有 OTA 固件标记和续传记录。
 *       调用前该区域必须已是擦除态, 或写的是同一份内容(Flash 只能 1->0)。
 */
bool ota_port_rollback_save(const ota_rollback_t *rec);

/**
 * @brief 使回滚记录失效(只把 magic 清 0, 【不擦页】—— 与 resume_clear 同模式)
 */
void ota_port_rollback_clear(void);

/* ==================== A/B 启动记录 ==================== */
/** 读取最新一条有效启动记录；false = 没有(首次/记录损坏) */
bool ota_port_boot_load(ota_boot_rec_t *out);

/** 追加写一条启动记录；false = 槽满/写入失败(调用方保持原状态) */
bool ota_port_boot_save(const ota_boot_rec_t *rec);

/* ==================== A/B 槽 / 运行时暂存区 ==================== */
/** 当前运行槽(0=A,1=B)；由 SCB->VTOR 判定，模式 1 恒为 0 */
uint8_t  ota_port_current_slot(void);
/** 本次 OTA 的目标槽(0=A,1=B) */
uint8_t  ota_port_target_slot(void);
/** 把下载暂存区指向某个槽(addr=槽基址)；同时更新目标槽 */
void     ota_port_set_stage(uint32_t addr, uint32_t size);
uint32_t ota_port_stage_addr(void);
uint32_t ota_port_stage_size(void);
/** 主循环每拍：pending 槽跑够时间 + 联网 + 有上行确认 -> 转正 */
void     ota_port_boot_poll(bool online, bool uplink_ok, uint32_t uptime_s);


/* ==================== 系统 ==================== */

/**
 * @brief 获取当前固件版本号
 * @return 版本字符串(静态存储,以'\0'结尾)
 * @note  值来自 ota_port_set_version()；未注入时返回 "0.0.0"
 *        (以前这里返回写死的 "1.0.0"，导致同版本 OTA 永远不被拒)
 */
const char *ota_port_get_version(void);

/** @brief 注入当前固件版本(app_boot 用 MAIN_FW_VERSION 调一次) */
void ota_port_set_version(const char *ver);

/**
 * @brief 系统重启(进入 bootloader 或复位)
 *        bootloader 侧应检测 OTA 缓存区有效后搬运到运行区
 */
void ota_port_reboot(void);

#endif /* __OTA_PORT_H__ */
