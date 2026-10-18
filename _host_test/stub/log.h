#ifndef __LOG_H__
#define __LOG_H__
/* 主机单元测试用: 把日志收进一个可控开关, 免得刷屏 */
void host_log_printf(const char *fmt, ...);
void host_log_enable(int on);

#define log_info(...)    host_log_printf(__VA_ARGS__)
#define log_error(...)   host_log_printf(__VA_ARGS__)
#define log_warning(...) host_log_printf(__VA_ARGS__)
#define log_debug(...)   host_log_printf(__VA_ARGS__)

#endif
