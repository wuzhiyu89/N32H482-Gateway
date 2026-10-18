/**
 * @file  cloud_json.c
 * @brief 极简 JSON 解析/生成实现(无动态内存, 见 cloud_json.h 的设计说明)
 *
 * 实现要点:
 *   - 解析: 全部在调用方提供的文本上原地扫描, 不复制整棵文档树。
 *           cj_get 系列走"路径"定位; cj_get_sub 返回原缓冲内的指针(零拷贝)。
 *   - 生成: cjw_t 写入器, 自动处理逗号/引号/转义; 容量不足置 ovf 而不是越界。
 *   - 安全性: 所有输出都做边界判断; 解析遇到非法结构返回 CJ_ERR 而不是死循环。
 */
#include "cloud_json.h"
#include <string.h>

/* ==================================================================
 * 一、基础扫描
 * ================================================================== */

static const char *j_ws(const char *p)
{
    if (p == NULL)
    {
        return NULL;
    }
    while ((*p == ' ') || (*p == '\t') || (*p == '\r') || (*p == '\n'))
    {
        p++;
    }
    return p;
}

/**
 * @brief 跳过字符串, 返回指向闭合引号之后的位置
 * @param p 必须指向开引号
 * @return 闭合引号之后; 结构非法返回 NULL
 */
static const char *j_after_string(const char *p)
{
    if ((p == NULL) || (*p != '"'))
    {
        return NULL;
    }
    p++;
    while (*p != '\0')
    {
        if (*p == '\\')
        {
            if (p[1] == '\0')
            {
                return NULL;
            }
            p += 2;
        }
        else if (*p == '"')
        {
            return (p + 1);
        }
        else
        {
            p++;
        }
    }
    return NULL;
}

/**
 * @brief 读取字符串内容并反转义
 * @param p     指向开引号
 * @param out   输出缓冲(可 NULL, 只求长度)
 * @param cap   out 容量(含 '\0')
 * @param out_n 输出: 反转义后的真实长度(即使 out 太小也会给出完整长度)
 * @param end   输出: 闭合引号之后的位置
 * @return true 成功
 * @note  内容超长时截断写入并补 '\0', 但 *out_n 是真实长度, 调用方可据此判断截断。
 *        \uXXXX 仅处理 BMP(不支持代理对), 转为 UTF-8。
 */
static bool j_read_string(const char *p, char *out, uint16_t cap,
                          uint16_t *out_n, const char **end)
{
    uint16_t n = 0;

    if ((p == NULL) || (*p != '"'))
    {
        return false;
    }
    p++;

    for (;;)
    {
        char c = *p;

        if (c == '\0')
        {
            return false;               /* 引号未闭合 */
        }
        if (c == '"')
        {
            p++;
            break;
        }
        if (c == '\\')
        {
            char e = p[1];

            if (e == '\0')
            {
                return false;
            }
            if (e == 'u')
            {
                uint32_t cp = 0u;
                uint8_t  i;
                uint8_t  seq[3];
                uint8_t  sn = 0u;

                for (i = 0u; i < 4u; i++)
                {
                    char h = p[2 + i];
                    uint8_t v;
                    if ((h >= '0') && (h <= '9'))       { v = (uint8_t)(h - '0'); }
                    else if ((h >= 'a') && (h <= 'f'))  { v = (uint8_t)(h - 'a' + 10); }
                    else if ((h >= 'A') && (h <= 'F'))  { v = (uint8_t)(h - 'A' + 10); }
                    else                                { return false; }
                    cp = (cp << 4) | (uint32_t)v;
                }
                if (cp < 0x80u)
                {
                    seq[sn++] = (uint8_t)cp;
                }
                else if (cp < 0x800u)
                {
                    seq[sn++] = (uint8_t)(0xC0u | (cp >> 6));
                    seq[sn++] = (uint8_t)(0x80u | (cp & 0x3Fu));
                }
                else
                {
                    seq[sn++] = (uint8_t)(0xE0u | (cp >> 12));
                    seq[sn++] = (uint8_t)(0x80u | ((cp >> 6) & 0x3Fu));
                    seq[sn++] = (uint8_t)(0x80u | (cp & 0x3Fu));
                }
                for (i = 0u; i < sn; i++)
                {
                    if ((out != NULL) && ((uint16_t)(n + 1u) < cap))
                    {
                        out[n] = (char)seq[i];
                    }
                    n++;
                }
                p += 6;
                continue;
            }

            {
                char d;
                switch (e)
                {
                    case '"':  d = '"';  break;
                    case '\\': d = '\\'; break;
                    case '/':  d = '/';  break;
                    case 'b':  d = '\b'; break;
                    case 'f':  d = '\f'; break;
                    case 'n':  d = '\n'; break;
                    case 'r':  d = '\r'; break;
                    case 't':  d = '\t'; break;
                    default:   return false;
                }
                if ((out != NULL) && ((uint16_t)(n + 1u) < cap))
                {
                    out[n] = d;
                }
                n++;
            }
            p += 2;
            continue;
        }

        if ((out != NULL) && ((uint16_t)(n + 1u) < cap))
        {
            out[n] = c;
        }
        n++;
        p++;
    }

    if ((out != NULL) && (cap > 0u))
    {
        uint16_t t = (n < cap) ? n : (uint16_t)(cap - 1u);
        out[t] = '\0';
    }
    if (out_n != NULL)
    {
        *out_n = n;
    }
    if (end != NULL)
    {
        *end = p;
    }
    return true;
}

/**
 * @brief 跳过一个完整的值(标量/对象/数组), 返回其后的位置
 */
static const char *j_skip_value(const char *p)
{
    p = j_ws(p);
    if (p == NULL)
    {
        return NULL;
    }
    if (*p == '"')
    {
        return j_after_string(p);
    }
    if ((*p == '{') || (*p == '['))
    {
        char    open  = *p;
        char    close = (open == '{') ? '}' : ']';
        uint8_t depth = 0u;

        while (*p != '\0')
        {
            if (*p == '"')
            {
                const char *q = j_after_string(p);
                if (q == NULL)
                {
                    return NULL;
                }
                p = q;
                continue;
            }
            if (*p == open)
            {
                depth++;
            }
            else if (*p == close)
            {
                depth--;
                if (depth == 0u)
                {
                    return (p + 1);
                }
            }
            p++;
        }
        return NULL;
    }
    /* number / true / false / null */
    while ((*p != '\0') && (*p != ',') && (*p != '}') && (*p != ']') &&
           (*p != ' ') && (*p != '\t') && (*p != '\r') && (*p != '\n'))
    {
        p++;
    }
    return p;
}

/**
 * @brief 在对象里查找成员, 返回成员值的起始位置
 */
static const char *j_obj_member(const char *obj, const char *key, uint16_t key_len)
{
    obj = j_ws(obj);
    if ((obj == NULL) || (*obj != '{'))
    {
        return NULL;
    }
    obj = j_ws(obj + 1);
    if (*obj == '}')                    /* 空对象 */
    {
        return NULL;
    }

    for (;;)
    {
        char        kbuf[64];
        uint16_t    klen = 0u;
        const char *after;
        const char *p;
        const char *next;

        if (*obj != '"')
        {
            return NULL;
        }
        if (!j_read_string(obj, kbuf, (uint16_t)sizeof(kbuf), &klen, &after))
        {
            return NULL;
        }
        p = j_ws(after);
        if ((p == NULL) || (*p != ':'))
        {
            return NULL;
        }
        p = j_ws(p + 1);
        if (p == NULL)
        {
            return NULL;
        }

        /* 用真实长度比较, 避免 kbuf 截断造成的误匹配 */
        if ((klen == key_len) && (memcmp(kbuf, key, key_len) == 0))
        {
            return p;
        }

        next = j_skip_value(p);
        if (next == NULL)
        {
            return NULL;
        }
        next = j_ws(next);
        if ((next != NULL) && (*next == ','))
        {
            obj = j_ws(next + 1);
            continue;
        }
        return NULL;                    /* '}' 或结构错误 */
    }
}

/**
 * @brief 取数组第 idx 个元素的值起始位置
 */
static const char *j_arr_item(const char *arr, uint16_t idx)
{
    uint16_t i = 0u;

    arr = j_ws(arr);
    if ((arr == NULL) || (*arr != '['))
    {
        return NULL;
    }
    arr = j_ws(arr + 1);
    if (*arr == ']')                    /* 空数组 */
    {
        return NULL;
    }

    for (;;)
    {
        const char *next;

        if (i == idx)
        {
            return arr;
        }
        next = j_skip_value(arr);
        if (next == NULL)
        {
            return NULL;
        }
        next = j_ws(next);
        if ((next != NULL) && (*next == ','))
        {
            arr = j_ws(next + 1);
            i++;
            continue;
        }
        return NULL;                    /* 越界 */
    }
}

/**
 * @brief 按路径走到底, 返回值起始位置
 * @param path 形如 "cmd" / "data.threshold" / "data.sensors[0].addr"
 */
static const char *j_walk(const char *json, const char *path)
{
    const char *cur;

    if ((json == NULL) || (path == NULL))
    {
        return NULL;
    }
    cur = j_ws(json);
    if (cur == NULL)
    {
        return NULL;
    }

    while (*path != '\0')
    {
        if (*path == '.')
        {
            path++;
            continue;
        }
        if (*path == '[')
        {
            uint32_t idx = 0u;
            bool     any = false;

            path++;
            while ((*path >= '0') && (*path <= '9'))
            {
                idx = (idx * 10u) + (uint32_t)(*path - '0');
                path++;
                any = true;
                if (idx > 0xFFFFu)
                {
                    return NULL;
                }
            }
            if ((*path != ']') || (!any))
            {
                return NULL;
            }
            path++;
            cur = j_arr_item(cur, (uint16_t)idx);
            if (cur == NULL)
            {
                return NULL;
            }
            continue;
        }

        {
            const char *dot = path;
            uint16_t    klen;

            while ((*dot != '\0') && (*dot != '.') && (*dot != '['))
            {
                dot++;
            }
            klen = (uint16_t)(dot - path);
            if (klen == 0u)
            {
                return NULL;
            }
            cur = j_obj_member(cur, path, klen);
            if (cur == NULL)
            {
                return NULL;
            }
            path = dot;
        }
    }
    return j_ws(cur);
}

static cj_type_t j_type_at(const char *p)
{
    switch (*p)
    {
        case '"':  return CJ_STR;
        case '{':  return CJ_OBJ;
        case '[':  return CJ_ARR;
        case 't':
        case 'f':  return CJ_BOOL;
        case 'n':  return CJ_NULL;
        case '\0': return CJ_ERR;
        default:   return CJ_NUM;
    }
}

/* ==================================================================
 * 二、对外解析接口
 * ================================================================== */

cj_type_t cj_get(const char *json, const char *path, char *out, uint16_t cap)
{
    const char *v;
    cj_type_t   t;

    if ((out != NULL) && (cap > 0u))
    {
        out[0] = '\0';
    }
    v = j_walk(json, path);
    if (v == NULL)
    {
        return (json == NULL) ? CJ_ERR : CJ_NOTFOUND;
    }

    t = j_type_at(v);
    switch (t)
    {
        case CJ_STR:
        {
            uint16_t    n   = 0u;
            const char *end = NULL;
            if (!j_read_string(v, out, cap, &n, &end))
            {
                return CJ_ERR;
            }
            return CJ_STR;
        }
        case CJ_OBJ:
        case CJ_ARR:
        {
            const char *tag = (t == CJ_OBJ) ? "[OBJ]" : "[ARR]";
            if ((out != NULL) && (cap > 0u))
            {
                uint16_t n = (uint16_t)strlen(tag);
                if (n > (uint16_t)(cap - 1u))
                {
                    n = (uint16_t)(cap - 1u);
                }
                memcpy(out, tag, n);
                out[n] = '\0';
            }
            return t;
        }
        case CJ_NUM:
        case CJ_BOOL:
        case CJ_NULL:
        {
            const char *e = j_skip_value(v);
            uint16_t    n;
            if (e == NULL)
            {
                return CJ_ERR;
            }
            n = (uint16_t)(e - v);
            while ((n > 0u) && ((v[n - 1u] == ' ') || (v[n - 1u] == '\t') ||
                                (v[n - 1u] == '\r') || (v[n - 1u] == '\n')))
            {
                n--;
            }
            if ((out != NULL) && (cap > 0u))
            {
                if (n > (uint16_t)(cap - 1u))
                {
                    n = (uint16_t)(cap - 1u);
                }
                memcpy(out, v, n);
                out[n] = '\0';
            }
            return t;
        }
        default:
            return CJ_ERR;
    }
}

bool cj_get_str(const char *json, const char *path, char *out, uint16_t cap)
{
    if ((out != NULL) && (cap > 0u))
    {
        out[0] = '\0';
    }
    return (cj_get(json, path, out, cap) == CJ_STR);
}

int32_t cj_get_int(const char *json, const char *path, int32_t def)
{
    char     buf[24];
    uint32_t mag = 0u;
    bool     neg = false;
    uint8_t  i   = 0u;

    if (cj_get(json, path, buf, (uint16_t)sizeof(buf)) != CJ_NUM)
    {
        return def;
    }
    if (buf[0] == '-')
    {
        neg = true;
        i = 1u;
    }
    else if (buf[0] == '+')
    {
        i = 1u;
    }
    if ((buf[i] < '0') || (buf[i] > '9'))
    {
        return def;
    }
    for (; (buf[i] >= '0') && (buf[i] <= '9'); i++)
    {
        /* mag*10 会溢出 uint32 的界 */
        if (mag > 429496729u)
        {
            return def;
        }
        mag = (mag * 10u) + (uint32_t)(buf[i] - '0');
    }
    if (neg)
    {
        if (mag > 2147483648u)
        {
            return def;
        }
        if (mag == 2147483648u)
        {
            return (-2147483647 - 1);
        }
        return -(int32_t)mag;
    }
    if (mag > 2147483647u)
    {
        return def;
    }
    return (int32_t)mag;
}

uint32_t cj_get_uint(const char *json, const char *path, uint32_t def)
{
    char     buf[24];
    uint32_t mag = 0u;
    uint8_t  i   = 0u;

    if (cj_get(json, path, buf, (uint16_t)sizeof(buf)) != CJ_NUM)
    {
        return def;
    }
    if (buf[0] == '-')
    {
        return def;                     /* 负数按无符号取无意义, 交回默认值 */
    }
    if (buf[0] == '+')
    {
        i = 1u;
    }
    if ((buf[i] < '0') || (buf[i] > '9'))
    {
        return def;
    }
    for (; (buf[i] >= '0') && (buf[i] <= '9'); i++)
    {
        uint32_t d = (uint32_t)(buf[i] - '0');
        if (mag > 429496729u)
        {
            return def;
        }
        mag = (mag * 10u) + d;
    }
    return mag;
}

bool cj_get_bool(const char *json, const char *path, bool def)
{
    char buf[8];

    if (cj_get(json, path, buf, (uint16_t)sizeof(buf)) != CJ_BOOL)
    {
        return def;
    }
    if (strcmp(buf, "true") == 0)
    {
        return true;
    }
    if (strcmp(buf, "false") == 0)
    {
        return false;
    }
    return def;
}

const char *cj_get_sub(const char *json, const char *path)
{
    const char *v = j_walk(json, path);
    if (v == NULL)
    {
        return NULL;
    }
    if ((*v != '{') && (*v != '['))
    {
        return NULL;
    }
    return v;
}

uint16_t cj_count(const char *json, const char *path)
{
    const char *v = j_walk(json, path);
    uint16_t    n = 0u;

    if ((v == NULL) || (*v != '['))
    {
        return 0u;
    }
    v = j_ws(v + 1);
    if (*v == ']')
    {
        return 0u;
    }
    for (;;)
    {
        const char *next = j_skip_value(v);
        if (next == NULL)
        {
            return 0u;
        }
        n++;
        next = j_ws(next);
        if ((next != NULL) && (*next == ','))
        {
            v = j_ws(next + 1);
            continue;
        }
        break;
    }
    return n;
}

uint16_t cj_unescape(char *s)
{
    char *r;
    char *w;

    if (s == NULL)
    {
        return 0u;
    }
    r = s;
    w = s;

    while (*r != '\0')
    {
        if (*r != '\\')
        {
            *w++ = *r++;
            continue;
        }
        r++;
        switch (*r)
        {
            case '"':  *w++ = '"';  r++; break;
            case '\\': *w++ = '\\'; r++; break;
            case '/':  *w++ = '/';  r++; break;
            case 'b':  *w++ = '\b'; r++; break;
            case 'f':  *w++ = '\f'; r++; break;
            case 'n':  *w++ = '\n'; r++; break;
            case 'r':  *w++ = '\r'; r++; break;
            case 't':  *w++ = '\t'; r++; break;
            case 'u':
            {
                uint32_t cp = 0u;
                uint8_t  i;
                bool     ok = true;

                for (i = 0u; i < 4u; i++)
                {
                    char    h = r[1 + i];
                    uint8_t v;
                    if ((h >= '0') && (h <= '9'))      { v = (uint8_t)(h - '0'); }
                    else if ((h >= 'a') && (h <= 'f')) { v = (uint8_t)(h - 'a' + 10); }
                    else if ((h >= 'A') && (h <= 'F')) { v = (uint8_t)(h - 'A' + 10); }
                    else                               { ok = false; break; }
                    cp = (cp << 4) | (uint32_t)v;
                }
                if (!ok)
                {
                    *w++ = '?';
                    r++;
                    break;
                }
                if (cp < 0x80u)
                {
                    *w++ = (char)cp;
                }
                else if (cp < 0x800u)
                {
                    *w++ = (char)(0xC0u | (cp >> 6));
                    *w++ = (char)(0x80u | (cp & 0x3Fu));
                }
                else
                {
                    *w++ = (char)(0xE0u | (cp >> 12));
                    *w++ = (char)(0x80u | ((cp >> 6) & 0x3Fu));
                    *w++ = (char)(0x80u | (cp & 0x3Fu));
                }
                r += 5;
                break;
            }
            case '\0':
                *w = '\0';
                return (uint16_t)(w - s);
            default:
                *w++ = *r++;
                break;
        }
    }
    *w = '\0';
    return (uint16_t)(w - s);
}

/* ==================================================================
 * 三、生成器
 * ================================================================== */

static void jw_putc(cjw_t *w, char c)
{
    if ((w == NULL) || w->ovf || w->err)
    {
        return;
    }
    if ((uint16_t)(w->len + 1u) >= w->cap)
    {
        w->ovf = true;
        return;
    }
    w->buf[w->len] = c;
    w->len++;
    w->buf[w->len] = '\0';
}

static void jw_puts(cjw_t *w, const char *s)
{
    while ((s != NULL) && (*s != '\0'))
    {
        jw_putc(w, *s);
        s++;
    }
}

/** 写成员前的前置逗号 */
static void jw_pre(cjw_t *w)
{
    uint8_t d;

    if ((w == NULL) || w->err)
    {
        return;
    }
    if (w->depth == 0u)
    {
        w->err = true;                  /* 顶层没开容器就写成员 */
        return;
    }
    d = (uint8_t)(w->depth - 1u);
    if (w->first[d])
    {
        w->first[d] = 0u;
    }
    else
    {
        jw_putc(w, ',');
    }
}

/** 写"值"前的前置逗号:
 *   - 紧跟 key 之后 -> 不要逗号
 *   - 顶层根值      -> 不要逗号(但只允许一个根值)
 *   - 其它          -> 按所在层的 first 标志补逗号
 *  ← 这里曾经写成"depth==0 就报错", 结果顶层 cjw_obj_begin 直接把整个写入器
 *    置成 err, 所有生成接口全部返回 0。主机单测抓到了这个 bug。 */
static void jw_pre_val(cjw_t *w)
{
    if ((w == NULL) || w->err)
    {
        return;
    }
    if (w->after_key)
    {
        w->after_key = false;
        return;
    }
    if (w->depth == 0u)
    {
        if (w->root_done)
        {
            w->err = true;              /* 顶层第二个根值: 非法 */
            return;
        }
        w->root_done = true;
        return;
    }
    jw_pre(w);
}

void cjw_init(cjw_t *w, char *buf, uint16_t cap)
{
    uint8_t i;

    if (w == NULL)
    {
        return;
    }
    w->buf       = buf;
    w->cap       = cap;
    w->len       = 0u;
    w->depth     = 0u;
    w->after_key = false;
    w->root_done = false;
    w->ovf       = false;
    w->err       = (buf == NULL) || (cap < 2u);
    for (i = 0u; i < CJW_MAX_DEPTH; i++)
    {
        w->first[i] = 1u;
    }
    if (!w->err)
    {
        buf[0] = '\0';
    }
}

void cjw_obj_begin(cjw_t *w)
{
    if ((w == NULL) || w->err)
    {
        return;
    }
    jw_pre_val(w);
    if (w->depth >= CJW_MAX_DEPTH)
    {
        w->err = true;
        return;
    }
    jw_putc(w, '{');
    w->first[w->depth] = 1u;
    w->depth++;
}

void cjw_obj_end(cjw_t *w)
{
    if ((w == NULL) || w->err)
    {
        return;
    }
    if (w->depth == 0u)
    {
        w->err = true;
        return;
    }
    w->depth--;
    jw_putc(w, '}');
}

void cjw_arr_begin(cjw_t *w)
{
    if ((w == NULL) || w->err)
    {
        return;
    }
    jw_pre_val(w);
    if (w->depth >= CJW_MAX_DEPTH)
    {
        w->err = true;
        return;
    }
    jw_putc(w, '[');
    w->first[w->depth] = 1u;
    w->depth++;
}

void cjw_arr_end(cjw_t *w)
{
    if ((w == NULL) || w->err)
    {
        return;
    }
    if (w->depth == 0u)
    {
        w->err = true;
        return;
    }
    w->depth--;
    jw_putc(w, ']');
}

void cjw_key(cjw_t *w, const char *key)
{
    if ((w == NULL) || w->err)
    {
        return;
    }
    jw_pre(w);
    jw_putc(w, '"');
    jw_puts(w, key);
    jw_putc(w, '"');
    jw_putc(w, ':');
    w->after_key = true;
}

uint16_t cj_escape_to(char *dst, uint16_t cap, const char *src, bool *ovf)
{
    uint16_t n = 0u;

    if (ovf != NULL)
    {
        *ovf = false;
    }
    if ((dst == NULL) || (cap == 0u) || (src == NULL))
    {
        if (ovf != NULL)
        {
            *ovf = true;
        }
        return 0u;
    }

    while (*src != '\0')
    {
        const char *rep = NULL;
        char        tmp[8];
        uint8_t     k = 0u;

        switch (*src)
        {
            case '"':  rep = "\\\""; break;
            case '\\': rep = "\\\\"; break;
            case '\b': rep = "\\b";  break;
            case '\f': rep = "\\f";  break;
            case '\n': rep = "\\n";  break;
            case '\r': rep = "\\r";  break;
            case '\t': rep = "\\t";  break;
            default:   break;
        }
        if (rep == NULL)
        {
            if ((uint8_t)*src < 0x20u)
            {
                /* 其它控制字符 -> \u00XX */
                static const char hex[] = "0123456789abcdef";
                tmp[0] = '\\'; tmp[1] = 'u'; tmp[2] = '0'; tmp[3] = '0';
                tmp[4] = hex[((uint8_t)*src >> 4) & 0x0Fu];
                tmp[5] = hex[(uint8_t)*src & 0x0Fu];
                tmp[6] = '\0';
                rep = tmp;
            }
            else
            {
                tmp[0] = *src;
                tmp[1] = '\0';
                rep = tmp;
            }
        }
        while (rep[k] != '\0')
        {
            if ((uint16_t)(n + 1u) >= cap)
            {
                dst[n] = '\0';
                if (ovf != NULL)
                {
                    *ovf = true;
                }
                return n;
            }
            dst[n] = rep[k];
            n++;
            k++;
        }
        src++;
    }
    dst[n] = '\0';
    return n;
}

void cjw_str(cjw_t *w, const char *s)
{
    if ((w == NULL) || w->err)
    {
        return;
    }
    jw_pre_val(w);
    jw_putc(w, '"');
    while ((s != NULL) && (*s != '\0'))
    {
        switch (*s)
        {
            case '"':  jw_puts(w, "\\\""); break;
            case '\\': jw_puts(w, "\\\\"); break;
            case '\n': jw_puts(w, "\\n");  break;
            case '\r': jw_puts(w, "\\r");  break;
            case '\t': jw_puts(w, "\\t");  break;
            default:
                if ((uint8_t)*s < 0x20u)
                {
                    static const char hex[] = "0123456789abcdef";
                    char t[7];
                    t[0] = '\\'; t[1] = 'u'; t[2] = '0'; t[3] = '0';
                    t[4] = hex[((uint8_t)*s >> 4) & 0x0Fu];
                    t[5] = hex[(uint8_t)*s & 0x0Fu];
                    t[6] = '\0';
                    jw_puts(w, t);
                }
                else
                {
                    jw_putc(w, *s);
                }
                break;
        }
        s++;
    }
    jw_putc(w, '"');
}

void cjw_raw(cjw_t *w, const char *raw)
{
    if ((w == NULL) || w->err)
    {
        return;
    }
    jw_pre_val(w);
    jw_puts(w, raw);
}

void cjw_int(cjw_t *w, int32_t v)
{
    char    t[12];
    uint8_t i = 0u;
    uint32_t u;
    bool     neg = (v < 0);

    u = neg ? (uint32_t)(-(int64_t)v) : (uint32_t)v;
    if (u == 0u)
    {
        t[i++] = '0';
    }
    while (u > 0u)
    {
        t[i++] = (char)('0' + (u % 10u));
        u /= 10u;
    }
    if (neg)
    {
        t[i++] = '-';
    }
    /* 反转 */
    {
        uint8_t a = 0u;
        uint8_t b = (uint8_t)(i - 1u);
        while (a < b)
        {
            char tmp = t[a];
            t[a] = t[b];
            t[b] = tmp;
            a++;
            b--;
        }
    }
    t[i] = '\0';
    cjw_raw(w, t);
}

void cjw_uint(cjw_t *w, uint32_t v)
{
    char    t[12];
    uint8_t i = 0u;

    if (v == 0u)
    {
        t[i++] = '0';
    }
    while (v > 0u)
    {
        t[i++] = (char)('0' + (v % 10u));
        v /= 10u;
    }
    {
        uint8_t a = 0u;
        uint8_t b = (uint8_t)(i - 1u);
        while (a < b)
        {
            char tmp = t[a];
            t[a] = t[b];
            t[b] = tmp;
            a++;
            b--;
        }
    }
    t[i] = '\0';
    cjw_raw(w, t);
}

void cjw_bool(cjw_t *w, bool v)
{
    cjw_raw(w, v ? "true" : "false");
}

void cjw_kv_str(cjw_t *w, const char *key, const char *s)
{
    cjw_key(w, key);
    cjw_str(w, s);
}

void cjw_kv_int(cjw_t *w, const char *key, int32_t v)
{
    cjw_key(w, key);
    cjw_int(w, v);
}

void cjw_kv_uint(cjw_t *w, const char *key, uint32_t v)
{
    cjw_key(w, key);
    cjw_uint(w, v);
}

void cjw_kv_bool(cjw_t *w, const char *key, bool v)
{
    cjw_key(w, key);
    cjw_bool(w, v);
}

bool cjw_ok(const cjw_t *w)
{
    if (w == NULL)
    {
        return false;
    }
    return (!w->ovf) && (!w->err) && (w->depth == 0u);
}

uint16_t cjw_len(const cjw_t *w)
{
    return (w == NULL) ? 0u : w->len;
}
