# N32H482-Gateway

基于国民技术 N32H482RE 的工业网关固件。

## 功能特性

- LoRa (SX1261/LLCC68) 无线通信
- 4G (ML307R/ML307Y) MQTT 上云
- USB CDC 调试/配置接口
- OTA 远程升级
- 本地告警与参数存储
- 传感器离线检测与数据缓存

## 目录结构

```
H482/
├── app/            # 应用层代码
│   ├── bsp/        # 板级支持包
│   ├── business/   # 业务逻辑
│   ├── common/     # 通用工具
│   ├── protocol/   # 通信协议
│   ├── radio/      # LoRa 射频驱动
│   └── ota/        # OTA 升级
├── bsp/            # 底层驱动
├── firmware/       # N32H48x 标准库
├── inc/            # 公共头文件
├── src/            # 主程序
└── MDK-ARM/        # Keil 工程
```

## 开发环境

- Keil MDK V5.43a
- ARMClang V6.24
- 芯片: N32H482RE (Cortex-M4, 512KB Flash, 192KB RAM)

## 构建

1. 打开 `MDK-ARM/CDC.uvprojx`
2. 选择目标 `N32H48X`
3. 编译输出在 `MDK-ARM/Objects/`
