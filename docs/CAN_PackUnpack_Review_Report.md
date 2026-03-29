# CAN 打包/解包逻辑审查报告

**分支**: `class_can`
**日期**: 2026-03-29
**审查范围**: CAN 2.0 帧的打包 (Pack) 和解包 (Unpack) 全链路

---

## 1. 审查结论

**CAN 打包/解包核心逻辑正确，未发现导致数据损坏或通信失败的严重 Bug。**

CAN 帧结构体 (`CAN_Cmd_t`, `CAN_Fbk_t`) 设计合理：
- 使用 `#pragma pack(1)` 消除编译器填充
- mode 字段使用显式 `uint8_t`（避免 enum 大小不确定问题）
- 总长度均为 8 字节，符合 Classic CAN 数据帧规范
- 字段顺序和类型与协议文档 (`CAN_Protocol_Handoff.md`) 一致
- `memcpy` 解包在 STM32H7（小端 Cortex-M7）上正确
- `INT16_CLAMP` 宏正确处理 present_current 的 int32→int16 截断

---

## 2. 已修复的问题

### 2.1 `mrc_protocol.h` — RS485 结构体注释字段大小错误

**文件**: `Devices/Inc/mrc_protocol.h`
**严重度**: 低（文档错误，不影响运行时）

| 结构体 | 字段 | 注释标注 | 实际大小 | 修复 |
|--------|------|----------|----------|------|
| `MRC_Cmd_Protocol` | `des_coil_current` | 2 bytes | 4 bytes (`int32_t`) | 已修正注释为 4 bytes |
| `MRC_Cmd_Protocol` | 总大小 | 8 bytes | 10 bytes | 已修正注释为 10 bytes |
| `MRC_Fbk_Protocol` | `encoder_velocity` | 2 bytes | 4 bytes (`int32_t`) | 已修正注释为 4 bytes |

> **注意**: `MRC_Mode` 枚举在结构体中直接使用，依赖 `-fshort-enums` 编译选项确保为 1 字节。
> CAN 结构体 (`CAN_Cmd_t`, `CAN_Fbk_t`) 已正确使用 `uint8_t` 避免此问题。

### 2.2 `drv_mrc.c` — `cmd_correct` 标志处理后未清除

**文件**: `Devices/Src/drv_mrc.c` (`MRC_Can_Process` 函数)
**严重度**: 低（当前不会导致逻辑错误，但不符合最佳实践）

**问题**: `CAN_Com_UnpackCmd()` 在 ISR 中设置 `cmd_correct = 1`，但 `MRC_Can_Process()` 处理完命令后只清除了 `RxFlag`，未清除 `cmd_correct`。该标志变成"粘滞"标志，一旦置 1 永不归零。

**修复**: 在 `MRC_Can_Process()` 处理完成后同时清除 `cmd_correct`:
```c
MRC->can_com.RxFlag      = 0;
MRC->can_com.cmd_correct = 0;  // <-- 新增
```

**为何当前无害**: `RxFlag` 只在 `CAN_Com_UnpackCmd` 返回 0（ID 匹配成功）时才置 1，所以 `MRC_Can_Process` 中的 `cmd_correct == 1` 检查实际上是冗余的。但清除标志是正确的防御性编程。

### 2.3 `can_com.c` — `CAN_Com_SendFbk` 注释过时

**文件**: `Devices/Src/can_com.c`
**严重度**: 低（文档错误）

**问题**: 注释声称 `fdcanx_send_data()` 硬编码了 `FDCAN_FD_CAN / FDCAN_BRS_ON`，但 `bsp_fdcan.c` 中的 `fdcanx_send_data()` 已修复为根据 `hfdcan->Init.FrameFormat` 动态选择:
```c
pTxHeader.BitRateSwitch = (hfdcan->Init.FrameFormat == FDCAN_FRAME_FD_BRS) ? FDCAN_BRS_ON : FDCAN_BRS_OFF;
pTxHeader.FDFormat      = (hfdcan->Init.FrameFormat == FDCAN_FRAME_CLASSIC) ? FDCAN_CLASSIC_CAN : FDCAN_FD_CAN;
```

**修复**: 更新注释，说明保留独立发送函数的原因是为了与通用 BSP 路径解耦。

---

## 3. 未修改的观察项（建议）

### 3.1 CAN 硬件过滤器未按设备 ID 配置

**文件**: `Devices/Src/bsp_fdcan.c` (`can_filter_init`)

当前过滤器配置:
```c
fdcan_filter.FilterID1 = 0x00;
fdcan_filter.FilterID2 = 0x00;  // 接受所有标准 ID
```

**影响**: 总线上所有 CAN 帧都会触发 ISR，由 `CAN_Com_UnpackCmd()` 做软件过滤。在多设备总线上会增加不必要的 ISR 开销。

**建议**: 在 `CAN_Com_Init()` 完成后，可用设备 ID 重新配置硬件过滤器：
```c
fdcan_filter.FilterID1 = CAN_CMD_ID(device_id);
fdcan_filter.FilterID2 = 0x7FF;  // 精确匹配
```

### 3.2 `#pragma pack(1)` 应用于非线协议结构体

**文件**: `Devices/Inc/can_com.h` (`MRC_Can_Com_t`), `Devices/Inc/drv_mrc.h` (`Device_MRC_t`)

**影响**: `MRC_Can_Com_t` 中的 `hcan_t *hfdcan`（指针）和 `uint64_t rx_time/tx_time` 可能位于非对齐地址。Cortex-M7 支持非对齐访问，不会 fault，但会有性能损失。

**建议**: 仅对线协议结构体（`CAN_Cmd_t`, `CAN_Fbk_t`）使用 `#pragma pack(1)`，运行时结构体使用自然对齐。

### 3.3 `MRC_Can_send_data` 中的 `main_loop_freq_calculateor`

**文件**: `Devices/Src/drv_mrc.c:321`

```c
getFreq(&MRC->main_loop_freq_calculateor);
```

在 CAN 模式下，`MRC_Can_send_data()` 从 ISR (`fdcan1_rx_callback`) 中调用，因此 `main_loop_freq_calculateor` 实际测量的是 CAN 接收频率（~1 kHz），而非主循环频率。RS485 模式下 `MRC_send_data()` 从主循环调用，语义正确。

**建议**: CAN 路径应使用 `can_com.freq_calculator`（已在 `CAN_Com_UnpackCmd` 中更新），移除 `MRC_Can_send_data` 中对 `main_loop_freq_calculateor` 的调用。

---

## 4. CAN 数据链路完整性验证

### 4.1 命令帧 (Host -> Device)

| 字节 | 字段 | 类型 | 缩放 | 验证 |
|------|------|------|------|------|
| 0 | mode | uint8_t | MRC_Mode enum | OK |
| 1 | reserved | uint8_t | 0x00 | OK |
| 2-5 | des_coil_current | int32_t LE | mA x 1000 | OK |
| 6-7 | reserved | uint8_t[2] | 0x00 | OK |

### 4.2 反馈帧 (Device -> Host)

| 字节 | 字段 | 类型 | 缩放 | 验证 |
|------|------|------|------|------|
| 0 | mode | uint8_t | MRC_Mode enum | OK |
| 1 | collision_flag | uint8_t | 0/1 | OK |
| 2-5 | encoder_value | int32_t LE | rad x 65535 | OK |
| 6-7 | present_current | int16_t LE | mA x 1000, clamped | OK |

### 4.3 CAN ID 方案

| 方向 | ID 公式 | 示例 (device_id=1) | 验证 |
|------|---------|---------------------|------|
| 命令 | 0x100 \| device_id | 0x101 | OK |
| 反馈 | 0x200 \| device_id | 0x201 | OK |

---

## 5. 修改文件清单

| 文件 | 修改类型 |
|------|----------|
| `Devices/Inc/mrc_protocol.h` | 修正字段大小注释 |
| `Devices/Src/drv_mrc.c` | 新增 `cmd_correct = 0` 清除 |
| `Devices/Src/can_com.c` | 更新 `CAN_Com_SendFbk` 注释 |
