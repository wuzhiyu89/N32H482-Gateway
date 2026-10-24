/* ============================================================================
 * fake_lora_business.c —— 主机测试用的 lora_business 替身(实现)
 *   只实现 cloud_proto.c 真正引用的那两个符号:
 *     · lora_business_unbind()          —— dev.unbind / sen.unbind 的异步入口
 *     · lora_business_open_bind_window() —— sen.bind / dev.bind 的 pair_hook 后端
 *   详见 fake_lora_business.h 的说明。
 * ========================================================================== */
#include "fake_lora_business.h"

#include <string.h>

#include "config_store.h"
#include "lora_business.h"

int      fake_unbind_calls;
char     fake_unbind_last_uid[27];
int      fake_unbind_result;
int      fake_unbind_simulate_finalize;

int      fake_pair_calls;
char     fake_pair_serial[24];
char     fake_pair_name[40];
uint16_t fake_pair_window;
bool     fake_pair_is_sensor;
uint8_t  fake_pair_dev_type;      /* ★ 第 11 条: dev.bind 的 type(1/2/3); sen.bind 为 0 */

/* ---- 档位联动下发(dev.ctrl / 自动联动都走 lora_business_apply_gear_ex) ---- */
int      fake_gear_calls;
int      fake_gear_last_gear;
int      fake_gear_uid_is_null;
char     fake_gear_last_uid[27];
char     fake_gear_last_led[80];
char     fake_gear_last_tts[80];
int      fake_gear_ret   = 1;
int      fake_gear_total = 1;
uint16_t fake_gear_res_code;

/* ---- ★2026-10-05: sen.set -> LoRa 0x01 参数下发 ---- */
int      fake_sensor_param_calls;
char     fake_sensor_param_uid[27];
uint32_t fake_sensor_param_report_s;
uint16_t fake_sensor_param_thresh_mm;
int      fake_sensor_param_ret  = 1;

void fake_lora_business_reset(void)
{
    fake_device_state              = LORA_DEV_STATE_UNKNOWN;
    fake_unbind_calls              = 0;
    fake_unbind_last_uid[0]        = '\0';
    fake_unbind_result             = 0;
    fake_unbind_simulate_finalize  = 0;
    fake_pair_calls                = 0;
    fake_pair_serial[0]            = '\0';
    fake_pair_name[0]              = '\0';
    fake_pair_window               = 0u;
    fake_pair_is_sensor            = false;
    fake_pair_dev_type             = 0u;

    fake_gear_calls        = 0;
    fake_gear_last_gear    = -1;
    fake_gear_uid_is_null  = -1;
    fake_gear_last_uid[0]  = '\0';
    fake_gear_last_led[0]  = '\0';
    fake_gear_last_tts[0]  = '\0';
    fake_gear_ret          = 1;
    fake_gear_total        = 1;
    fake_gear_res_code     = 0u;

    fake_sensor_param_calls        = 0;
    fake_sensor_param_uid[0]       = '\0';
    fake_sensor_param_report_s     = 0u;
    fake_sensor_param_thresh_mm    = 0u;
    fake_sensor_param_ret          = 1;
}

/* ---- 被 cloud_proto.c 调用(sen.set): 把平台参数打包成 0x01 帧下发 ---- */
bool lora_business_set_sensor_params(const char *uid_hex,
                                     uint32_t report_period_s,
                                     uint16_t alarm_thresh_mm)
{
    fake_sensor_param_calls++;
    (void)strncpy(fake_sensor_param_uid, (uid_hex != NULL) ? uid_hex : "",
                  sizeof(fake_sensor_param_uid) - 1u);
    fake_sensor_param_uid[sizeof(fake_sensor_param_uid) - 1u] = '\0';
    fake_sensor_param_report_s  = report_period_s;
    fake_sensor_param_thresh_mm = alarm_thresh_mm;
    return (fake_sensor_param_ret != 0);
}

/* ---- ★2026-10-10 新增: dev.ctrl 的样式覆盖(ledShowMode/ledTextShowMode/ledColor) ----
 * 真机上这两个函数操作 lora_business.c 里的静态覆盖, 在这里对主机单测没有意义 ——
 * 空口载荷(out[3] 那几位)的构造由 level_gear_build_payload() 负责, 它在
 * test_level.c 里【单独测过】, 不经过本假实现。
 * ⇒ 这里给空实现即可, 只为让链接通过。
 * ⚠️ 但【绝不能省】: cloud_proto.c 引用了这两个符号, 少一个 test_cloud 就链接失败。 */
void lora_business_set_gear_style(uint8_t valid_mask, uint8_t show_mode,
                                  uint8_t text_mode_h1, uint8_t text_mode_h2,
                                  uint8_t color_h1, uint8_t color_h2)
{
    (void)valid_mask;
    (void)show_mode;
    (void)text_mode_h1;
    (void)text_mode_h2;
    (void)color_h1;
    (void)color_h2;
}

void lora_business_clear_gear_style(void)
{
    /* 空实现: 见上面的说明 */
}

/* ---- 被 cloud_proto.c 调用(dev.ctrl): 档位联动下发(记录调用, 便于断言) ---- */
int lora_business_apply_gear_ex(uint8_t gear, const char *only_uid,
                                const char *led_text1, const char *led_text2,
                                const char *tts_text,
                                lora_gear_result_t *res, uint8_t res_cap,
                                uint8_t *out_total)
{
    fake_gear_calls++;
    fake_gear_last_gear   = (int)gear;
    fake_gear_uid_is_null = (only_uid == NULL) ? 1 : 0;

    (void)strncpy(fake_gear_last_uid, (only_uid != NULL) ? only_uid : "",
                  sizeof(fake_gear_last_uid) - 1u);
    fake_gear_last_uid[sizeof(fake_gear_last_uid) - 1u] = '\0';
    (void)strncpy(fake_gear_last_led, (led_text1 != NULL) ? led_text1 : "",
                  sizeof(fake_gear_last_led) - 1u);
    fake_gear_last_led[sizeof(fake_gear_last_led) - 1u] = '\0';
    (void)strncpy(fake_gear_last_tts, (tts_text != NULL) ? tts_text : "",
                  sizeof(fake_gear_last_tts) - 1u);
    fake_gear_last_tts[sizeof(fake_gear_last_tts) - 1u] = '\0';

    if (out_total != NULL) { *out_total = (uint8_t)fake_gear_total; }

    if ((res != NULL) && (res_cap > 0u))
    {
        (void)strncpy(res[0].uid, (only_uid != NULL) ? only_uid : "31000000000000000000000001",
                      sizeof(res[0].uid) - 1u);
        res[0].uid[sizeof(res[0].uid) - 1u] = '\0';
        res[0].code = fake_gear_res_code;
    }
    return fake_gear_ret;
}

/* ---- 被 cloud_proto.c 调用: 发起解绑(异步) ---- */
int lora_business_unbind(const char *uid_hex)
{
    fake_unbind_calls++;

    if (uid_hex != NULL)
    {
        (void)strncpy(fake_unbind_last_uid, uid_hex, sizeof(fake_unbind_last_uid) - 1u);
        fake_unbind_last_uid[sizeof(fake_unbind_last_uid) - 1u] = '\0';
    }
    else
    {
        fake_unbind_last_uid[0] = '\0';
    }

    /* 模拟"业务层收到设备 0x80 ACK 后的收尾": 与 lora_business.c 的
     * _unbind_finalize() 做的是同一件事(删 config_store 含 gears 清理 + 落盘),
     * 好让用例能继续验证这条真实代码路径的落盘效果。 */
    if ((fake_unbind_result == 0) && (fake_unbind_simulate_finalize != 0)
        && (fake_unbind_last_uid[0] != '\0'))
    {
        (void)config_store_unbind_device(fake_unbind_last_uid, 0u);
    }

    return fake_unbind_result;
}

/* ---- 被 main.c 注册成 pair_hook; test_cloud 里由用例自己注册, 这里给个同签名实现 ---- */
bool lora_business_open_bind_window(const char *serial, const char *name,
                                    uint16_t window_s, bool is_sensor,
                                    uint8_t dev_type)
{
    fake_pair_calls++;
    (void)strncpy(fake_pair_serial, (serial != NULL) ? serial : "",
                  sizeof(fake_pair_serial) - 1u);
    fake_pair_serial[sizeof(fake_pair_serial) - 1u] = '\0';
    (void)strncpy(fake_pair_name, (name != NULL) ? name : "",
                  sizeof(fake_pair_name) - 1u);
    fake_pair_name[sizeof(fake_pair_name) - 1u] = '\0';
    fake_pair_window    = window_s;
    fake_pair_is_sensor = is_sensor;
    fake_pair_dev_type  = dev_type;      /* ★ 第 11 条: dev.bind 的 type 要传到这里 */
    return true;
}

/* ★2026-10-18: sen.query / dev.query 的 online 现在会问这个函数。
 * 替身默认回 UNKNOWN(=> online=false), 测试可用 fake_device_state 改成 ONLINE/OFFLINE。 */
lora_dev_state_t fake_device_state = LORA_DEV_STATE_UNKNOWN;

lora_dev_state_t lora_business_device_state(const char *uid_hex)
{
    (void)uid_hex;
    return fake_device_state;
}
lora_dev_state_t lora_business_device_state_bytes(const uint8_t uid[LORA_UID_BYTES])
{
    (void)uid;
    return fake_device_state;
}

void lora_business_ensure_slaves(void) { }
