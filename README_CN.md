# SafeMRC

**[English](README.md)** | **[中文](README_CN.md)**

> 基于 STM32H7 的磁流变液 (MRF) 离合器/轴承锁控制器嵌入式固件。

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
![MCU: STM32H723](https://img.shields.io/badge/MCU-STM32H723-03234B?logo=stmicroelectronics)
![构建: CMake/Ninja](https://img.shields.io/badge/Build-CMake%2FNinja-064F8C)
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

- CMake 3.22 或更高版本，以及 [Ninja](https://ninja-build.org/)
- GNU Arm Embedded 工具链（`arm-none-eabi-gcc`、`objcopy`、`size`）
- [STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html)（外设配置）
- ST-Link 编程器；命令行烧录需要 [OpenOCD](https://openocd.org/) 或 `st-flash`

`MDK-ARM/` 下的 Keil 工程仍保留作为备用，Linux 构建不再依赖 Keil。

### 编译与烧录

```bash
# 配置 CubeMX 生成的 RS-485 Debug preset
cmake --preset Debug

# 生成 ELF、HEX、BIN 和 MAP
cmake --build --preset Debug
cmake --build --preset Debug --target size

# CAN 使用相同的 CubeMX 工具链配置独立构建目录
cmake -S . -B build/CAN -G Ninja \
  -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake \
  -DCMAKE_BUILD_TYPE=Debug \
  -DMRC_COM_BACKEND=CAN
cmake --build build/CAN

# 使用 OpenOCD 烧录（校验后复位）
cmake --build --preset Debug --target flash

# 或使用 st-flash 备用命令
cmake --build --preset Debug --target flash-stlink
```

日常开发可以直接运行构建脚本，不需要重复输入 CMake 和 Ninja 命令。默认是
增量编译；需要重新编译全部源文件时加上 `--full`：

```bash
scripts/build.sh
scripts/build.sh --full
scripts/build.sh --backend CAN --full
scripts/build.sh --backend RS485 --config Release --jobs 8
```

首次使用某个构建目录时，脚本会配置 CMake；增量编译时由 Ninja 自动检查是否
需要重新生成，`--full` 会显式重新配置并清理后再编译。脚本最后会打印固件大小。
RS485 和 CAN 使用独立目录，例如 `build/RS485-Debug/`、`build/CAN-Debug/`，
不会混用不同通信后端的目标文件。

直接使用 `cmake --preset Debug` 时，生成文件位于 `build/Debug/`；使用构建脚本
时位于脚本选择的目录，例如 `build/RS485-Debug/`。`flash` 目标默认使用
`interface/stlink.cfg` 和 `target/stm32h7x.cfg`。如果探针或目标配置不同，
可以直接调用脚本：

```bash
scripts/flash_openocd.sh build/Debug/safeMRC_for_DM_firmware.elf \
  interface/my_probe.cfg target/stm32h7x.cfg
```

OpenOCD 默认只擦除和写入 ELF 镜像覆盖的扇区，并在写入后校验，不会执行整片
擦除。Linux 下使用 ST-Link 工具可能需要将当前用户加入发行版的 `plugdev`
组，或安装 ST-Link 的 udev 规则。如果当前版本的 `st-flash` 不支持 `reset`，
写入完成后请使用探针复位或重新给目标板上电。

### GitHub Actions CI/CD

Pull Request 和分支 Push 会自动执行
`.github/workflows/firmware-ci.yml`，构建 RS485、CAN 两种后端的 Debug 和
Release 配置，共四种组合；同时检查 ELF、HEX、BIN、MAP 文件并上传为临时
构建产物。推送类似 `v1.0.0` 的版本标签后，Action 会自动创建 GitHub Release，
并附带 RS485 和 CAN 的 Release 固件及自动生成的变更说明。

GitHub 托管 runner 没有连接项目的 ST-Link 和开发板，因此 CI/CD 不会执行
实体硬件烧录。下载并确认产物后，仍需在本地使用 `flash` 或 `flash-stlink`
目标完成烧录。

GDB 调试可以在一个终端启动 OpenOCD：

```bash
openocd -f interface/stlink.cfg -f target/stm32h7x.cfg
```

然后在另一个终端连接：

```text
arm-none-eabi-gdb build/Debug/safeMRC_for_DM_firmware.elf
(gdb) target extended-remote :3333
(gdb) monitor reset halt
(gdb) load
(gdb) monitor reset
(gdb) continue
```

VS Code 用户可以安装 Cortex-Debug，并使用下面的可选配置；构建过程不依赖
VS Code：

```json
{
  "name": "SafeMRC (OpenOCD)",
  "type": "cortex-debug",
  "request": "launch",
  "cwd": "${workspaceFolder}",
  "executable": "${workspaceFolder}/build/Debug/safeMRC_for_DM_firmware.elf",
  "servertype": "openocd",
  "configFiles": ["interface/stlink.cfg", "target/stm32h7x.cfg"],
  "runToEntryPoint": "main"
}
```

### CubeMX 重新生成

`safeMRC_for_DM_firmware.ioc` 仍是 MCU、时钟树、GPIO、DMA 和外设配置的唯一
来源。CubeMX 现在已经在项目中生成 `CMakePresets.json`、
`cmake/gcc-arm-none-eabi.cmake` 和 `cmake/stm32cubemx/CMakeLists.txt`。
顶层 `CMakeLists.txt` 使用这些生成文件管理 `Core/` 和 HAL，并补充
`Common/`、`Devices/` 应用层。

CubeMX 生成的 linker 和 startup 文件会保留作参考，但顶层构建明确使用仓库
维护的 GCC startup 和 linker，因为固件需要把 DMA 缓冲区放到独立的
`RAM_D2` 段。不要删除 `cmake/STM32H723VGHx.ld` 或
`startup/startup_stm32h723xx_gcc.s`。

每次 CubeMX 生成代码后都要：

1. 生成前启用备份和保留用户代码选项。
2. 检查 `git diff`，将生成改动范围限制在 CubeMX 管理的 `Core/`、`Drivers/`、
   `cmake/stm32cubemx/`、preset 和工具链文件。
3. 使用 `cmake --preset Debug` 重新配置，并分别编译 `RS485` 与 `CAN` 后端。
4. 检查 MAP 文件，再烧录并完成硬件冒烟测试后提交。

Keil 备用工程仍可使用：

```text
MDK-ARM/safeMRC_for_DM_firmware.uvprojx

# 在 Keil 中按 F7 编译，并通过 Keil 调试器或 STM32CubeProgrammer 烧录。
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
3. 按照[提交信息规范](dev_docs/standards/commit-message.md)提交更改，包含 `TYPE:`、`WHY:`、`WHAT:`、`TEST:`
4. 推送分支 (`git push origin feature/my-feature`)
5. 发起 Pull Request

开发工作文档、迁移记录和协作规范统一从 [dev_docs](dev_docs/README.md) 查阅。

## 许可证

本项目基于 [MIT License](LICENSE) 开源。

## 致谢

开发于中国科学技术大学 (USTC)。
