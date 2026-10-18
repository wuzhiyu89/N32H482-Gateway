/**
 * @file  cloud_json.h
 * @brief 零堆 JSON 解析 + 极简写出（端侧专用）
 *
 * 本工程有【两层】JSON，职责严格分开 —— 新代码请遵守：
 *
 *   解析（下行命令里的标量 / 子对象）      -> 本文件 cj_get_* 系列
 *       零堆、只读原缓冲、按路径取值；不需要建树。
 *
 *   构造 / 打印（信封、应答、上行 data）    -> cJSON + cjson_port
 *       走 24 KB 静态池（cjson_port_init() 装 hooks）。本工程 Heap_Size 只有
 *       0x200 = 512 B，所以 cJSON 必须走池，不能用裸 malloc。
 *
 *   两边不要交叉：能 cj_get 取到的字段不要用 cJSON 再解析一遍；
 *   需要池管理的大对象（信封/整份配置）不要用 cjw_* 手拼。
 *
 * 解析能力:
 *   - 按"路径"取标量值: "cmd" / "data.threshold" / "data.sensors[0].addr"
 *   - 取子对象指针(零拷贝, 指向原缓冲)
 *   - 统计数组元素个数
 *   - 字符串反转义: \" \\ \/ \b \f \n \r \t \uXXXX(转 UTF-8, 仅 BMP)
 *   - 不支持的: 注释、单引号、NaN/Inf、顶层多文档
 *
 * 生成能力:
 *   cjw_* 系列, 自动处理逗号/引号/转义, 溢出时置 ovf 标志而不是越界。
 *
 * 体积: 约 4KB flash, 0 额外 RAM(全部用调用方缓冲)。
 *
 * 边界与示例见 docs/JSON.md。
 */
#ifndef __CLOUD_JSON_H__
#define __CLOUD_JSON_H__

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==================== 解析 ==================== */

/** 取值结果类型; CJ_NOTFOUND 表示路径不存在, CJ_ERR 表示 JSON 本身非法 */
typedef enum
{
    CJ_NOTFOUND = 0,
    CJ_NULL,
    CJ_BOOL,
    CJ_NUM,
    CJ_STR,
    CJ_OBJ,
    CJ_ARR,
    CJ_ERR
} cj_type_t;

/**
 * @brief 按路径取值
 * @param json 以 '\0' 结尾的 JSON 文本
 * @param path 路径, 例 "cmd" / "data.threshold" / "data.sensors[0].addr"
 * @param out  输出缓冲(可为 NULL, 此时只查询类型)
 * @param cap  out 容量; 字符串会自动反转义并补 '\0'
 * @return 值类型; 数值/布尔以文本形式写入 out
 * @note  CJ_OBJ / CJ_ARR 时 out 得到 "[OBJ]"/"[ARR]", 要拿指针用 cj_get_sub()
 */
cj_type_t cj_get(const char *json, const char *path, char *out, uint16_t cap);

/** 取字符串(自动反转义)。找不到 / 不是字符串 -> 返回 false, out 置空串 */
bool cj_get_str(const char *json, const char *path, char *out, uint16_t cap);

/** 取有符号整数; 非数值或缺失返回 def */
int32_t cj_get_int(const char *json, const char *path, int32_t def);

/** 取无符号整数; 负值会被夹成 0 */
uint32_t cj_get_uint(const char *json, const char *path, uint32_t def);

/** 取布尔; 接受 true/false 以及 0/1 */
bool cj_get_bool(const char *json, const char *path, bool def);

/**
 * @brief 取子对象/子数组的起始指针(零拷贝, 指向 json 缓冲内部)
 * @return 指向 '{' 或 '[' 的指针; 不是对象/数组或不存在则 NULL
 */
const char *cj_get_sub(const char *json, const char *path);

/**
 * @brief 统计路径所指数组的元素个数
 * @return 元素数; 不是数组或不存在返回 0
 */
uint16_t cj_count(const char *json, const char *path);

/** 原地反转义(NUL 结尾)。返回新长度 */
uint16_t cj_unescape(char *s);

/* ==================== 生成 ==================== */

/** JSON 写入器; 深度上限 8 */
#define CJW_MAX_DEPTH 8

typedef struct
{
    char    *buf;
    uint16_t cap;                        /* 可用容量(含结尾 '\0' 位) */
    uint16_t len;                        /* 已写字节数(不含 '\0') */
    uint8_t  depth;
    uint8_t  first[CJW_MAX_DEPTH];       /* 每层是否还没写过成员 */
    bool     after_key;                   /* 刚写过 key, 紧随的值不需要前置逗号 */
    bool     root_done;                   /* 顶层根值是否已写过(顶层只允许一个根值) */
    bool     ovf;                         /* 溢出过 —— 结果不可用 */
    bool     err;                         /* 参数/状态错误 */
} cjw_t;

void cjw_init(cjw_t *w, char *buf, uint16_t cap);
void cjw_obj_begin(cjw_t *w);
void cjw_obj_end(cjw_t *w);
void cjw_arr_begin(cjw_t *w);
void cjw_arr_end(cjw_t *w);
void cjw_key(cjw_t *w, const char *key);
void cjw_str(cjw_t *w, const char *s);
void cjw_raw(cjw_t *w, const char *raw);   /* 原样写入(数字/true/false/null/已构造片段) */
void cjw_int(cjw_t *w, int32_t v);
void cjw_uint(cjw_t *w, uint32_t v);
void cjw_bool(cjw_t *w, bool v);

/* 便捷: key + value */
void cjw_kv_str(cjw_t *w, const char *key, const char *s);
void cjw_kv_int(cjw_t *w, const char *key, int32_t v);
void cjw_kv_uint(cjw_t *w, const char *key, uint32_t v);
void cjw_kv_bool(cjw_t *w, const char *key, bool v);

/** 是否成功(无溢出、无错误、深度已归零) */
bool     cjw_ok(const cjw_t *w);
/** 当前长度 */
uint16_t cjw_len(const cjw_t *w);

/* ==================== 转义辅助 ==================== */

/**
 * @brief 往缓冲追加转义后的 JSON 字符串内容(不含首尾引号)
 * @return 实际写入字节数; 溢出返回 0 且置 *ovf
 */
uint16_t cj_escape_to(char *dst, uint16_t cap, const char *src, bool *ovf);

#ifdef __cplusplus
}
#endif

#endif /* __CLOUD_JSON_H__ */
