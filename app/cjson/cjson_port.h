/**
 * @file  cjson_port.h
 * @brief cJSON 在本工程的落地层 —— 静态池分配器 + cJSON_InitHooks + 资源统计
 *
 * ---------------------------------------------------------------------------
 * 【为什么必须有这一层，不能直接用 cJSON】
 *   cJSON 的默认分配器就是裸的 malloc/free/realloc（见 cJSON.c 的 `#else` 分支：
 *   `#define internal_malloc malloc`）。而本工程：
 *     · startup_n32h47x_48x.s 里 Heap_Size = 0x200 = 【512 字节】；
 *     · 实测 v29 正常固件的库成员表里【根本没有 malloc/free】(见 CDC.map 的
 *       Library Member Name 段)，也就是说本工程此前一行动态分配都没有。
 *   512 字节堆跑 cJSON 必然一跑就挂，所以唯一可行的做法是：给 cJSON 换一套
 *   【架在静态数组上的池式分配器】，通过 cJSON_InitHooks() 装进去。
 *   cJSON 官方支持这个（cJSON.h 的 cJSON_Hooks 提供 malloc_fn / free_fn），
 *   所以【不需要改 cJSON 一行源码】。
 *
 * 【一个必须知道的版本差异：这版 cJSON_Hooks 没有 realloc_fn】
 *   而 cJSON_Print 的缓冲扩容本来需要一个 realloc 语义。不用怕 ——
 *   cJSON_InitHooks() 在发现我们给了自定义 malloc/free 之后，会把内部的
 *   global_hooks.reallocate 置成 NULL，随后自动改走"手动扩容"(新分配 + memcpy +
 *   释放旧块)。也就是【libc 的 realloc 永远不会碰到池里的指针】，这正是本工程要的。
 *   详见 cjson_port.c 里"这里为什么没有 realloc"那段。已实际编译验证：往 hooks 里
 *   塞 realloc_fn 是编译不过的（cJSON_Hooks 结构体里没有这个成员）。
 *
 * 【池的尺寸怎么定 —— 别猜，用主机测试量】
 *   cJSON 是"建树"式解析：每个 JSON 值一个节点(32 位下 cJSON 结构约 40 字节)，
 *   每个 key、每个字符串还要各分配一次。所以解析同样的报文，cJSON 的 RAM 开销
 *   是报文长度的数倍。本工程的实测最坏输入：
 *     · 下行 JSON 上限 MAIN_CLOUD_JSON_MAX = 4160 字节(见 src/main.c)；
 *     · 满配配置序列化 3288 字节(见 _host_test/test_cfg.c 的"最坏情况")。
 *   到底需要多少，由 host 测试 [_host_test/test_cjson.c] 实测峰值后写回这里，
 *   而不是估算。
 *
 * 【分配失败怎么办】
 *   池耗尽时分配器返回 NULL，cJSON 的解析/构造会返回 NULL —— 【不崩、不越界】。
 *   调用方必须把它当成一条失败的命令处理(回 1007 网关内部错误)。
 *   失败次数是【只增不减的累计计数器】(沿用本工程"瞬时状态会被清掉、累计值才查得动"
 *   的诊断惯例)，由 cjson_port_fail_count() 暴露，并进日志。
 *
 * 【对齐】必须 8 字节
 *   cJSON 的节点里有一个 double valuedouble，32 位 ABI 下要求 8 字节对齐；
 *   本分配器的块头固定 8 字节且池基址 8 字节对齐，因此数据区天然 8 字节对齐。
 *
 * 【单线程假设】本工程是无 RTOS 的裸机轮询，分配器不加锁。
 * ---------------------------------------------------------------------------
 */
#ifndef __CJSON_PORT_H__
#define __CJSON_PORT_H__

#include <stdint.h>
#include <stdbool.h>
#include "cJSON.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 池容量(字节)。
 *
 * ★ 这个数不是估的，是量出来的 ★ (_host_test/test_cjson.c，3196 字节的最坏配置)
 *     (a) 纯解析 .............................. 13968 B
 *     (b) 解析 + cJSON_PrintPreallocated ...... 13968 B  ← 零额外开销
 *     (c) 解析 + cJSON_PrintUnformatted ....... 21528 B  ← 池内扩容多花 7560 B
 *     (d) 下行解析树 + 应答树 同时存活 ......... 14632 B  ← 真正的上限
 *   由此定下两条规矩：
 *     1) 【应答/上报一律用 cJSON_PrintPreallocated() 打进入调用方的静态缓冲】，
 *        绝不用 cJSON_Print/PrintUnformatted —— 那会额外要 7.5 KB 的池内扩容空间；
 *     2) 池给 24 KB = 实测最坏 14632 B 的 1.68 倍，留出"平台多塞了几个未知字段"
 *        和"cfg.query 的应答以后也改成树"的余量。
 *   ⚠ 以后每动一次协议层【都要重跑 test_cjson 看峰值】，涨了就得同步调这个数。
 *
 * 可用编译宏覆盖(例如主机测试想放大来量需求时)。 */
#ifndef CJSON_PORT_POOL_SIZE
#define CJSON_PORT_POOL_SIZE    24576u
#endif

/**
 * @brief 安装分配器 hooks 并清空池
 * @note  上电时调用一次即可。可以重复调用(会先清空)。
 *        任何 cJSON_Parse / cJSON_Create* 之前必须先调用本函数，
 *        否则 cJSON 会走裸 malloc —— 本工程 512 字节堆下那是必挂的。
 */
void cjson_port_init(void);

/**
 * @brief hooks 是否已经装好(即 cjson_port_init() 调用过)
 * @return true = 已装
 *
 * @note  ★ 这是护栏, 不是装饰 ★
 *   没装 hooks 就调 cJSON_Parse / cJSON_Create*, cJSON 会去用它的默认分配器 =
 *   裸 malloc/free; 而本工程堆只有 512 字节 —— 那是【必挂、且没有任何编译或链接
 *   错误】的静默故障。(实测踩过一次: 只把 cloud_proto.c 迁到 cJSON、忘了在 main()
 *   里 init, 链接器就会把 cjson_port.o 整个丢掉, 代码看着完全正常。)
 *   所以解析/生成之前先问一句: 没装就明确报 1007, 而不是悄悄去动那 512 字节堆。
 */
bool cjson_port_is_ready(void);

/**
 * @brief 清空池与统计(不重装 hooks)
 * @note  给主机单测用：每个用例要一个确定的初始状态。
 */
void cjson_port_reset(void);

/** 池总容量(字节) */
uint32_t cjson_port_pool_size(void);
/** 当前占用(字节) —— 全部释放后应回到 0 */
uint32_t cjson_port_used(void);
/** 历史峰值占用(字节) —— 用来定池尺寸的那个数 */
uint32_t cjson_port_peak(void);
/** 分配失败累计次数(只增不减)；不为 0 说明池小了 */
uint32_t cjson_port_fail_count(void);
/** 成功分配累计次数 */
uint32_t cjson_port_alloc_count(void);

/**
 * @brief 空闲块自检：块链必须首尾相接、正好铺满整个池、且没有相邻的两个空闲块
 * @return true = 池结构完好
 * @note  主机单测专用(用来抓"漏释放/重复释放/碎片没合并")。
 *        固件里不要在主循环常调(O(块数))。
 */
bool cjson_port_check(void);

/** 把统计打进日志(一行)，便于现场从串口确认有没有分配失败 */
void cjson_port_log_stats(const char *tag);

#ifdef __cplusplus
}
#endif

#endif /* __CJSON_PORT_H__ */
