/* ============================================================================
 * fake_lora_business.h —— 主机测试用的 lora_business 替身
 * ----------------------------------------------------------------------------
 * 【为什么需要它】
 *   P5(2026-09-29)起 cloud_proto.c 直接调 lora_business_unbind() 发起解绑(异步),
 *   而 lora_business.c 依赖射频层(radio.c / LLCC68 / SPI), 主机测试链不进来
 *   (也用不着) —— 于是 test_cloud 会 undefined reference。
 *   这里给一个【会记录调用】的替身, 而不是空桩:
 *   这样 test_cloud 依然能验证"cloud_proto 确实把 uid 交给了业务层、并且把
 *   业务层的返回码 -1/-2/-3 正确映射成 1001/1003/1006"。
 *
 * 【它不做什么】
 *   不模拟 LoRa 空口、不模拟 ACK 时序。要验绑定/解绑状态机本身, 应该单独给
 *   lora_business.c 写一套带假射频的主机测试(目前还没有, 见归档说明的遗留项)。
 * ========================================================================== */
#ifndef FAKE_LORA_BUSINESS_H
#define FAKE_LORA_BUSINESS_H

#include <stdbool.h>
#include <stdint.h>
#include "lora_business.h"      /* lora_dev_state_t —— 2026-10-18 新增的状态查询 */

/** lora_business_unbind() 被调用的次数 */
extern int  fake_unbind_calls;

/** 最后一次传入的 uid(空串表示传了 NULL) */
extern char fake_unbind_last_uid[27];

/** 让替身返回这个值: 0=成功发起 / -1=uid 非法 / -2=不在从机表 / -3=已有解绑在跑 */
extern int  fake_unbind_result;

/** 1 = 收到调用时顺手模拟"业务层完成删除"(调 config_store_unbind_device + 落盘),
 *  用来验证真正的收尾动作(_unbind_finalize 那条路径)对配置的影响 */
extern int  fake_unbind_simulate_finalize;

/** 把替身状态复位(每个用例开头调) */
/* ★2026-10-18: lora_business_device_state() 替身可控返回值 */
extern lora_dev_state_t fake_device_state;

void fake_lora_business_reset(void);

/* ---- 档位联动下发(dev.ctrl 与自动联动都走 lora_business_apply_gear_ex) ---- */
extern int      fake_gear_calls;         /* 被调用次数 */
extern int      fake_gear_last_gear;     /* 最后一次的档位 */
extern int      fake_gear_uid_is_null;   /* 最后一次 only_uid 是否 NULL(="all") */
extern char     fake_gear_last_uid[27];  /* 最后一次的 only_uid(空 = NULL) */
extern char     fake_gear_last_led[80];  /* 最后一次的 LED 文本 */
extern char     fake_gear_last_tts[80];  /* 最后一次的语音文本 */
extern int      fake_gear_ret;           /* 让替身返回这个"受理台数" */
extern int      fake_gear_total;         /* out_total 回填值 */
extern uint16_t fake_gear_res_code;      /* res[0].code 回填值 */

/** 假的 pair_hook: 记录 cloud_proto 传下来的 serial/name/窗口/类型 */
extern int      fake_pair_calls;
extern char     fake_pair_serial[24];
extern char     fake_pair_name[40];
extern uint16_t fake_pair_window;
extern bool     fake_pair_is_sensor;
extern uint8_t  fake_pair_dev_type;   /* ★ 第 11 条: dev.bind 的 type(1/2/3) */

/* ---- ★2026-10-05: sen.set -> LoRa 0x01 参数下发(lora_business_set_sensor_params) ----
 * 用来断言"cloud_proto 确实把参数交给了业务层, 且单位/数值没被改写"。 */
extern int      fake_sensor_param_calls;      /* 被调用次数 */
extern char     fake_sensor_param_uid[27];    /* 最后一次的 uid */
extern uint16_t fake_sensor_param_detect_s;   /* 最后一次的检测间隔(秒) */
extern uint16_t fake_sensor_param_report_s;   /* 最后一次的上报间隔(秒) */
extern int      fake_sensor_param_same;       /* 最后一次的"相同液位也上报" */
extern uint16_t fake_sensor_param_thresh_mm;  /* 最后一次的阈值 mm */
extern int      fake_sensor_param_ret;        /* 让替身返回这个值(默认 true) */

#endif /* FAKE_LORA_BUSINESS_H */
