# Linux/CMake 迁移工作记录

## 当前方案

CubeMX 使用项目内的 `safeMRC_for_DM_firmware.ioc` 配置外设，并生成 CMake
工具链、预设和 Core/HAL 源文件清单。顶层 `CMakeLists.txt` 接入生成结果，
加入 `Common/`、`Devices/` 业务代码。Keil 工程保留作备用。

项目继续使用 `startup/startup_stm32h723xx_gcc.s`、
`cmake/STM32H723VGHx.ld` 和 `Common/Src/sysmem.c`，以保留启动流程、
D2 SRAM DMA 分区，以及独立堆栈内存区域的分配方式。CubeMX 生成的对应文件
保留作参考，集成逻辑放在顶层 CMake 中。

协议枚举 `DEBUG` 已改为 `MRC_MODE_DEBUG`，兼容 CubeMX Debug 配置的
`-DDEBUG`。协议数值仍为 `3`，CLI 命令仍为 `DEBUG ON/OFF`。

## 日常构建

```bash
./scripts/build.sh
./scripts/build.sh --full
./scripts/build.sh --backend CAN
./scripts/build.sh --backend RS485 --config Release --jobs 8
```

默认增量编译 RS485 Debug；`--full` 重新配置并清理选定构建目录后重编。
产物位于 `build/<backend>-<config>/`，包含 ELF、HEX、BIN 和 MAP。

CubeMX 重新生成后先检查 `git diff`，然后对两个通信后端执行全量构建并检查
MAP 中的 DMA 内存位置，再进行烧录和硬件验证。

## 验证记录与边界

Linux 交叉编译已验证 RS485/CAN Debug 全量和增量构建，以及 RS485 Release
构建。Shell 语法和参数错误返回也已检查。

当前仍有固件源码的编译警告，包括未使用参数、静态函数声明以及 CAN 接收
长度变量可能未初始化。本次迁移未处理这些已有问题。

## 待办

- 在 CubeMX 中启用生成前备份；当前 `.ioc` 的 `BackupPrevious=false`。
- 实机验证 ST-Link/OpenOCD 烧录、复位启动和 UART CLI。
- 实机验证 RS485、CAN、DMA、ADC、编码器、PWM 和碰撞响应。
- 再次执行 CubeMX 生成流程，检查手工维护文件并验证两个后端。
- 补充原迁移计划中尚未完成的主机协议测试。

编译和链接通过不代表硬件功能已经通过验证。
