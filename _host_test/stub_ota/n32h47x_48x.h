#ifndef __N32H47X_48X_H__
#define __N32H47X_48X_H__
/* 主机单测替身: ota_port.c 只用到 NVIC_SystemReset()。
 * 盖住 _host_test/stub/n32h47x_48x.h(那个是空文件, 给别的套件用的)。 */
void NVIC_SystemReset(void);
#endif
