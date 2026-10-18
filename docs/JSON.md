# JSON 的两层实现与使用边界

> 这份文档只回答一件事：**写云协议相关代码时，该用哪套 JSON、为什么。**
> 两套实现都保留，不做合并 —— 合并会把"零堆解析"或"池化打印"其中一边的
> 内存特性弄丢。

## 1. 两层是什么

| 层 | 文件 | 内存 | 用途 |
|---|---|---|---|
| **解析层** | `app/cloud/cloud_json.{c,h}` | 零堆，只读调用方缓冲 | 从下行命令里按路径取标量/子对象 |
| **构造层** | `app/cjson/cJSON.c` + `app/cjson/cjson_port.c` | 24 KB 静态池（`cjson_port_init()` 装 hooks） | 组信封、组应答、组上行 `data`、打印 |

背景：本工程 `Heap_Size = 0x200`（512 B）。cJSON 默认走 `malloc`，不装 hooks 就会挂；
装了 hooks 之后全程从静态池分配，**不允许**绕过池直接 `malloc`。

## 2. 决策树

```
要读下行 JSON 里的一个字段?
  └─ 用 cj_get_*（路径取值，零拷贝）
     例: cj_get_str(req->data, "version", buf, sizeof(buf))
         cj_get(req->data, "delaySeconds", NULL, 0) -> CJ_NUM

要生成 JSON 文本?
  ├─ 小对象 / 单层、字段少、调用方已有缓冲
  │    └─ 可以用 cjw_*（零堆，溢出置 ovf 标志）
  └─ 信封、命令应答、上行 data、任何需要嵌套数组/转义复杂的
       └─ 用 cJSON_* + cloud_json_print()（池化打印）
```

**不要交叉**：

- 已经能用 `cj_get` 取到的字段，不要为了"统一"再 `cJSON_Parse` 一遍（多一次池分配、多一份代码）；
- 需要 cJSON 池管理的大对象（`cfg.query` 的整份配置、`sensors[]` 数组），不要用 `cjw_*` 手拼
  （拼接顺序、逗号、转义都得自己维护，已经踩过坑）。

## 3. 典型例子

### 解析（下行命令）

```c
char mode[12];
if (!cj_get_str(req->data, "mode", mode, (uint16_t)sizeof(mode)))
{
    return CLOUD_EC_PARAM;
}
```

### 构造（命令应答/上行 data）

```c
cJSON *data = cJSON_CreateObject();
cJSON_AddNumberToObject(data, "configVersion", (double)config_store_version());

char *txt = cJSON_PrintPreallocated(root, out, cap, 0);  /* 池内打印 */
```

实际代码里统一走 `cloud_json_print()`（`cloud_internal.h`），它封装了
`cJSON_PrintPreallocated` 的失败处理。

### 原样嵌入一段已构造好的 JSON

```c
cJSON_AddItemToObject(root, "data", cJSON_CreateRaw(data_json));
```

`cJSON_Raw` 不会加引号/二次转义，与迁移前的 `cjw_raw` 行为一致 —— 补报事件就是
靠它把"每条事件对象"原样嵌进 `events[]`。

## 4. 内存纪律

1. 解析缓冲、打印缓冲**一律 `static`**（栈只有 4 KB）；
2. 打印用 `cJSON_PrintPreallocated`，缓冲不够时**返回 0 而不是越界**；
3. 池大小见 `cjson_port.h`；如果新增的上行报文让池峰值接近上限，先算 RAM 再加；
4. 任何新代码都不允许 `malloc/free`。

## 5. 已知的"历史包袱"（改代码时会遇到）

- `config_store.c` 的解析用 `cj_get_*`，序列化用 cJSON —— 这是刻意的：
  - 解析走零堆、只读原缓冲；
  - 序列化要处理嵌套数组（`sensors[]`/`gears[]`），手拼容易错。
- `cloud_proto.c` 解析信封用 cJSON（`cloud_proto_parse_req`），取 `req->data`
  之后由各命令用 `cj_get_*` 读字段 —— 两条路径都在用，边界就是上面那张表。
- `main.c` 的 `#json` 自测直接把 JSON 文本喂给 `app_downlink_try()`，与线上同一条路径。
