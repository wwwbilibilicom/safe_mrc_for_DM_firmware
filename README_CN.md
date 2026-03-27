# SafeMRC

**[English](README.md)** | **[中文](README_CN.md)**

> 基于 STM32H7 的磁流变液 (MRF) 离合器/轴承锁控制器嵌入式固件。

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
![MCU: STM32H723](https://img.shields.io/badge/MCU-STM32H723-03234B?logo=stmicroelectronics)
![IDE: Keil MDK](https://img.shields.io/badge/IDE-Keil_MDK--ARM-red)
![CubeMX](https://img.shields.io/badge/Config-STM32CubeMX-blue)

<p align="center">
  <img src="images/hardware.png" alt="SafeMRC 硬件" width="600">
</p>

## 功能特性

- **双控制模式** - 电压控制与线圈电流控制 (前馈 + PI)，可平滑切换
- **实时碰撞检测** - 碰撞时自动退磁，保障安全
- **双通信后端** - RS-485 (4 Mbps) 与经典 CAN (1 Mbps)，编译时选择
- **状态机管理** - FREE / FIX_LIMIT / ADAPTATION / DEBUG 模式，安全转换
- **PWM 非线性补偿** - 查找表 + 插值，实现低压段 H 桥精确输出
- **多种滤波器** - 滑动平均、低通、卡尔曼、带通滤波器
- **Flash 持久化** - 设备 ID 和线圈电阻在断电后保持
- **调试 CLI** - UART1 交互式命令行，用于测试和校准
- **Python 上位机** - 跨平台 GUI，支持实时绘图和数据导出

## 快速开始

### 环境要求

- [Keil MDK-ARM](https://www.keil.com/) v5.37+
- [STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html)（外设配置）
- ST-Link 编程器

### 编译与烧录

```bash
# 1. 在 Keil 中打开工程
MDK-ARM/safeMRC_for_DM_firmware.uvprojx

# 2. 编译 (Keil 中按 F7)
# 3. 通过 ST-Link 烧录 (Download 按钮或 STM32CubeProgrammer)
```

### Python 上位机

```bash
conda env create -f scripts/environment.yml
conda activate safemrc_host
python scripts/main.py
```

需要 USB-RS485 转换器，波特率最高 4 Mbps。详见 [scripts/README.md](scripts/README.md)。

### Python SDK 示例

```python
from safeMRC_sdk import SafeMRC, SafeMRCCmd, SafeMRCData

mrc = SafeMRC('/dev/ttyUSB0')
cmd = SafeMRCCmd(mode=1, current=0.5)
fbk = SafeMRCData()

if mrc.sendRecv(cmd, fbk):
    print(f"角度={fbk.encoder:.3f}, 电流={fbk.current:.3f}")
```

## 架构

```
Core/           HAL 生成的外设初始化 (CubeMX 输出)
Common/         通用工具库: PID、滤波器、Flash、系统时钟
Devices/        MRC 应用驱动 (核心逻辑层)
Applications/   回调函数，连接 HAL 中断与设备逻辑
scripts/        PC 端 Python 上位机 + SDK
```

**定时器:** TIM6 @ 1 kHz 驱动状态机和反馈发送。TIM4 @ 10 kHz 驱动线圈电流控制环和通信处理。

**全局单实例:** 所有应用代码操作 `main.c` 中声明的唯一 `Device_MRC_t MRC` 实例。无 RTOS - 通过 ISR 设置标志位、主循环轮询实现并发。

## 通信

通过 `Devices/Inc/mrc_com_backend.h` 编译时选择两种后端：

| 后端 | 总线 | 波特率 | 帧大小 | 校验方式 |
|------|------|--------|--------|----------|
| RS-485 | USART2 | 4 Mbps | CMD: 10 B, FBK: 17 B | CRC-16-CCITT |
| CAN    | FDCAN1 | 1 Mbps | CMD: 8 B, FBK: 8 B   | 硬件 CRC     |

两种后端均使用请求-响应模式：主机发送命令，设备回复包含编码器位置、速度、电流和碰撞状态的反馈帧。

## 文档

| 文档 | 说明 |
|------|------|
| [RS-485 协议](docs/protocol-rs485.md) | 帧格式、字节布局、CRC、模式定义 |
| [CAN 协议](docs/CAN_Protocol_Handoff.md) | CAN ID 方案、帧布局、SocketCAN 示例 |
| [API 参考](docs/api-reference.md) | 设备结构体、控制函数、滤波器、定时器 |
| [调试 CLI 与 Flash](docs/debug-cli.md) | UART1 CLI 命令、Flash 持久化 |
| [故障排除](docs/troubleshooting.md) | 常见问题、安全特性、调试输出 |
| [Python 上位机](scripts/README.md) | GUI 安装、SDK 使用、数据导出 |

## 贡献

1. Fork 本仓库
2. 创建特性分支 (`git checkout -b feature/my-feature`)
3. 提交更改 (`git commit -am 'Add my feature'`)
4. 推送分支 (`git push origin feature/my-feature`)
5. 发起 Pull Request

## 许可证

本项目基于 [MIT License](LICENSE) 开源。

## 致谢

开发于中国科学技术大学 (USTC)。
