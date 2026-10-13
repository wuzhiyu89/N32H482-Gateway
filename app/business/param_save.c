#include "param_save.h"
#include "n32h47x_48x.h"
#include "flash_map.h"   /* 分区地址唯一真相源 */
#include <string.h>
#include <stddef.h>
#include "utils.h"

/* ---------------- Flash页配置 ----------------
 * 分区地址不再写死在这里, 统一由 app/common/flash_map.h 决定。
 * 参数页 = FM_PARAM_ADDR (单镜像模式 0x0807E000 / 双Slot模式 0x0806E000),
 * 与 OTA 标记页、暂存区、配置双备份区均不重叠(flash_map.h 内有静态断言保证)。
 */
#define PARAM_FLASH_PAGE_ADDR      ((uint32_t)FM_PARAM_ADDR)
#define PARAM_FLASH_PAGE_SIZE      ((uint32_t)FM_PARAM_SIZE)

/* 编译期保证参数结构不会超出参数页(P1-0 加入 cloud_profile_t 后约 216 字节) */
typedef char param_save_size_ok[(sizeof(device_param_t) <= PARAM_FLASH_PAGE_SIZE) ? 1 : -1];

/* 静态内存备份, Flash读取出来的参数复制到此RAM */
static device_param_t s_device_param;

// /**
//  * @brief crc32简易校验, 用于参数完整性校验
//  */
// static uint32_t param_crc_calc(const uint8_t *buf, uint16_t len)
// {
//     uint32_t crc = 0xFFFFFFFFU;
//     for(uint16_t i = 0; i < len; i++)
//     {
//         crc ^= buf[i];
//         for(uint8_t bit = 0; bit < 8U; bit++)
//         {
//             if(crc & 0x01U)
//             {
//                 crc = (crc >> 1U) ^ 0xEDB88320U;
//             }
//             else
//             {
//                 crc >>= 1U;
//             }
//         }
//     }
//     return ~crc;
// }

/**
 * @brief 加载出厂默认参数, 写入RAM s_device_param
 *
 * gw_uid 编码:
 *   byte0 = (PARAM_DEFAULT_GW_TYPE << 4) | PARAM_DEFAULT_GW_ENV = 0x11
 *   byte1 = (PARAM_DEFAULT_GW_NUM >> 8) & 0xFF = 0x00 (编号高字节, 大端)
 *   byte2 = PARAM_DEFAULT_GW_NUM & 0xFF = 0x01 (编号低字节, 大端)
 *   → gw_uid = {0x11, 0x00, 0x01}  ↔ hex 地址串 "110001"
 */
static void param_load_factory_default(void)
{
    memset(&s_device_param, 0, sizeof(device_param_t));

    /* --- 网关自身参数 --- */
    s_device_param.modbus_slave_addr     = PARAM_DEFAULT_MODBUS_SLAVE_ADDR;
    s_device_param.uart_baudrate         = PARAM_DEFAULT_UART_BAUDRATE;
    s_device_param.report_interval_ms    = PARAM_DEFAULT_REPORT_INTERVAL_MS;
    s_device_param.lora_channel          = PARAM_DEFAULT_LORA_CHANNEL;
    s_device_param.alarm_power_threshold = PARAM_DEFAULT_ALARM_POWER_THRESHOLD;
    s_device_param.alarm_tts_local_en    = PARAM_DEFAULT_ALARM_TTS_LOCAL_EN;
    s_device_param.alarm_level_threshold = PARAM_DEFAULT_ALARM_LEVEL_THRESHOLD;

    /* --- 网关 LoRa UID 默认值 (编号大端: 高字节在前) --- */
    s_device_param.gw_uid[0] = (uint8_t)((PARAM_DEFAULT_GW_TYPE << 4) | (PARAM_DEFAULT_GW_ENV & 0x0FU));
    s_device_param.gw_uid[1] = (uint8_t)((PARAM_DEFAULT_GW_NUM >> 8) & 0x00FFU);
    s_device_param.gw_uid[2] = (uint8_t)(PARAM_DEFAULT_GW_NUM & 0x00FFU);

    /* --- 从机列表默认空 --- */
    s_device_param.slave_cnt = 0U;

    /* --- 云平台连接档位默认值 (P1-0): 本地模式, 与现场当前行为一致 --- */
    cloud_profile_set_factory_default(&s_device_param.cloud);

    /* --- CRC --- */
    s_device_param.crc = 0U;
    uint16_t calc_len = offsetof(device_param_t, crc);
    s_device_param.crc = crc32_ieee((uint8_t *)&s_device_param, calc_len);
}

/**
 * @brief 从Flash页读取参数到RAM s_device_param
 * @retval 0:CRC校验成功; -1:Flash页为空(全0xFF未烧写); -2:CRC校验失败, 数据损坏
 */
static int param_load_from_flash(void)
{
    const device_param_t *p_flash_param = (const device_param_t *)PARAM_FLASH_PAGE_ADDR;

    /* 判断是否整段结构体全部是0xFF, 代表从未写过参数 */
    uint8_t *p_tmp = (uint8_t *)p_flash_param;
    uint8_t is_all_ff = 1;
    for(uint16_t i = 0; i < sizeof(device_param_t); i++)
    {
        if(p_tmp[i] != 0xFFU)
        {
            is_all_ff = 0;
            break;
        }
    }
    if(is_all_ff != 0U)
    {
        return -1;
    }

    memcpy(&s_device_param, p_flash_param, sizeof(device_param_t));

    /* 计算校验: 不包含结构体最后的crc成员 */
    uint16_t calc_len = offsetof(device_param_t, crc);
    uint32_t calc_crc = crc32_ieee((uint8_t *)&s_device_param, calc_len);
    if(calc_crc != s_device_param.crc)
    {
        return -2;
    }
    return 0;
}

/**
 * @brief 将RAM内s_device_param写入Flash (先擦页, 再写)
 * @return 0成功, 非0失败, 返回FLASH_STS错误码
 */
static int param_write_to_flash(void)
{
    FLASH_STS sts;
    uint32_t addr = PARAM_FLASH_PAGE_ADDR;
    uint8_t *p_src = (uint8_t *)&s_device_param;

    /* 8字节对齐 */
    uint32_t total_bytes = sizeof(device_param_t);
    uint32_t align_len = ((total_bytes + 7U) / 8U) * 8U;

    FLASH_Unlock();

    sts = FLASH_EraseOnePage(addr);
    if(sts != FLASH_EOP)
    {
        FLASH_Lock();
        return (int)sts;
    }
    sts = FLASH_WaitForLastOpt(0xFFFFFFFFU);
    if(sts != FLASH_EOP)
    {
        FLASH_Lock();
        return (int)sts;
    }

    /* 循环写, 每次 8 字节(双字) */
    for(uint32_t offset = 0U; offset < align_len; offset += 8U)
    {
        uint64_t write_data = 0ULL;
        uint8_t *p_dst = (uint8_t *)&write_data;

        for(uint32_t j = 0U; j < 8U; j++)
        {
            if((offset + j) < total_bytes)
            {
                p_dst[j] = p_src[offset + j];
            }
            else
            {
                p_dst[j] = 0U;
            }
        }

        uint32_t data0 = (uint32_t)(write_data & 0xFFFFFFFFULL);
        uint32_t data1 = (uint32_t)((write_data >> 32U) & 0xFFFFFFFFULL);

        sts = FLASH_ProgramdoubleWord(addr + offset, data0, data1);
        if(sts != FLASH_EOP)
        {
            FLASH_Lock();
            return (int)sts;
        }
        sts = FLASH_WaitForLastOpt(0xFFFFFFFFU);
        if(sts != FLASH_EOP)
        {
            FLASH_Lock();
            return (int)sts;
        }
    }

    FLASH_Lock();
    return 0;
}

/* ========== 对外 init / store / ptr ========== */

void param_save_init(void)
{
    int ret = param_load_from_flash();
    if(ret != 0)
    {
        /* Flash为空 / CRC校验失败, 载入出厂默认 */
        param_load_factory_default();
    }
}

int param_save_store(void)
{
    uint16_t calc_len = offsetof(device_param_t, crc);
    s_device_param.crc = crc32_ieee((uint8_t *)&s_device_param, calc_len);
    return param_write_to_flash();
}

device_param_t* param_get_ptr(void)
{
    return &s_device_param;
}

void param_set_factory_default(void)
{
    param_load_factory_default();
}

/* ========== 从机列表操作 ========== */

int param_slave_find(const uint8_t uid[3])
{
    uint8_t cnt = s_device_param.slave_cnt;
    if(cnt > PARAM_SLAVE_MAX_CNT) cnt = PARAM_SLAVE_MAX_CNT;
    for(uint8_t i = 0; i < cnt; i++)
    {
        if(memcmp(s_device_param.slave_uid[i].uid, uid, 3) == 0)
            return (int)i;
    }
    return -1;
}

int param_slave_add(const uint8_t uid[3])
{
    /* 已存在? */
    if(param_slave_find(uid) >= 0) return -2;
    /* 满了? */
    if(s_device_param.slave_cnt >= PARAM_SLAVE_MAX_CNT) return -1;

    uint8_t idx = s_device_param.slave_cnt;
    memcpy(s_device_param.slave_uid[idx].uid, uid, 3);
    s_device_param.slave_cnt = idx + 1U;
    return 0;
}

int param_slave_del(const uint8_t uid[3])
{
    int idx = param_slave_find(uid);
    if(idx < 0) return -1;

    /* 从 idx 开始, 后面的前移一位 */
    uint8_t cnt = s_device_param.slave_cnt;
    for(uint8_t i = (uint8_t)idx; i < cnt - 1U; i++)
    {
        s_device_param.slave_uid[i] = s_device_param.slave_uid[i + 1U];
    }
    /* 最后一个清零 */
    memset(&s_device_param.slave_uid[cnt - 1U], 0, sizeof(param_slave_uid_t));
    s_device_param.slave_cnt = cnt - 1U;
    return 0;
}
