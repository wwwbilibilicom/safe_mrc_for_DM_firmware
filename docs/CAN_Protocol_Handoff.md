# SafeMRC CAN 通讯协议交接文档

> **目标读者**: Linux 端 CAN Socket 通讯开发者
> **固件分支**: `class_can`
> **MCU**: STM32H723VGT6 (FDCAN1)
> **生成日期**: 2026-03-26

---

## 1. 总览

SafeMRC 轴承锁控制器通过 **Classic CAN (CAN 2.0B)** 与上位机进行通讯。每个设备节点占用两个 CAN ID：一个用于接收命令，一个用于发送反馈。协议为 **8 字节定长帧**，所有多字节字段使用 **小端序 (Little-Endian)**。

通讯模式为 **请求-应答**: 上位机发送命令帧 → 设备在 ISR 中立即回复反馈帧。设计通讯频率为 **1 kHz**。

---

## 2. 总线参数

| 参数 | 值 |
|------|------|
| CAN 标准 | Classic CAN (CAN 2.0B) |
| 波特率 | **1 Mbps** (默认) |
| 帧格式 | 标准帧 (11-bit ID)，数据帧 |
| DLC | 固定 8 字节 |
| 自动重传 | 启用 (Auto Retransmission = ENABLE) |
| 终端电阻 | 总线两端各 120Ω (硬件需保证) |

### 波特率计算依据

固件端 FDCAN 内核时钟 = **120 MHz** (HSE 8MHz → PLL1Q)

```
baud = 120MHz / (BRP × (1 + SEG1 + SEG2))
1Mbps = 120MHz / (3 × (1 + 32 + 7)) = 120MHz / 120
采样点 = (1 + 32) / (1 + 32 + 7) = 82.5%
```

### 固件支持的波特率一览

| 标识 | 波特率 | BRP | SEG1 | SEG2 | SJW |
|------|--------|-----|------|------|-----|
| CAN_BR_125K | 125 kbps | 6 | 139 | 20 | 20 |
| CAN_BR_250K | 250 kbps | 3 | 139 | 20 | 20 |
| CAN_BR_500K | 500 kbps | 2 | 99 | 20 | 20 |
| **CAN_BR_1M** | **1 Mbps** | **3** | **32** | **7** | **7** |

> 如需更改波特率，固件端调用 `bsp_fdcan_set_baud(&hfdcan1, CAN_CLASS, CAN_BR_xxx)` 即可。Linux 端需同步配置 `ip link set can0 type can bitrate <value>`。

---

## 3. CAN ID 编址方案

使用 **11-bit 标准 ID**，高 3 位为方向前缀，低 8 位为设备地址：

```
命令帧 (Host → Device):  CAN_ID = 0x100 | device_id
反馈帧 (Device → Host):  CAN_ID = 0x200 | device_id
```

| 方向 | ID 范围 | 示例 (device_id=1) |
|------|---------|-------------------|
| 命令 (Host→Dev) | 0x101 ~ 0x1FE | 0x101 |
| 反馈 (Dev→Host) | 0x201 ~ 0x2FE | 0x201 |

- `device_id` 有效范围: **1 ~ 254** (0x01 ~ 0xFE)
- 0x00 和 0xFF 保留未使用

### Linux 端过滤建议

如需只接收反馈帧，可设置 CAN 过滤器：

```c
// 只接收 0x200~0x2FE 的帧
struct can_filter rfilter;
rfilter.can_id   = 0x200;
rfilter.can_mask = 0x700;  // 匹配高 3 位 = 0x200
setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));
```

---

## 4. 命令帧格式 (Host → Device)

**CAN ID**: `0x100 | device_id`
**DLC**: 8 bytes

| 字节偏移 | 字段名 | 类型 | 说明 |
|----------|--------|------|------|
| 0 | `mode` | uint8_t | 工作模式枚举值 |
| 1 | `reserved` | uint8_t | 保留，填 0x00 |
| 2~5 | `des_coil_current` | int32_t (LE) | 期望线圈电流，单位 mA，×1000 缩放 |
| 6~7 | `reserved2` | uint8_t[2] | 保留，填 0x00 |

### C 结构体定义 (可直接在 Linux 端使用)

```c
#pragma pack(push, 1)
typedef struct {
    uint8_t  mode;              // 工作模式
    uint8_t  reserved;          // 保留
    int32_t  des_coil_current;  // 期望电流 (mA), little-endian
    uint8_t  reserved2[2];      // 保留
} CAN_Cmd_t;                    // 共 8 字节
#pragma pack(pop)

_Static_assert(sizeof(CAN_Cmd_t) == 8, "CAN_Cmd_t must be 8 bytes");
```

### 电流值换算

```
实际电流 (A) = des_coil_current / 1000.0

示例:
  5.0 A → des_coil_current = 5000
  0.5 A → des_coil_current = 500
 -3.0 A → des_coil_current = -3000 (反向电流)
```

---

## 5. 反馈帧格式 (Device → Host)

**CAN ID**: `0x200 | device_id`
**DLC**: 8 bytes

| 字节偏移 | 字段名 | 类型 | 说明 |
|----------|--------|------|------|
| 0 | `mode` | uint8_t | 当前工作模式枚举值 |
| 1 | `collision_flag` | uint8_t | 碰撞标志: 0x00=安全, 0x01=碰撞 |
| 2~5 | `encoder_value` | int32_t (LE) | 编码器位置，= 弧度 × 65535 |
| 6~7 | `present_current` | int16_t (LE) | 当前线圈电流，单位 mA，×1000 缩放 |

### C 结构体定义 (可直接在 Linux 端使用)

```c
#pragma pack(push, 1)
typedef struct {
    uint8_t  mode;              // 当前模式
    uint8_t  collision_flag;    // 碰撞标志
    int32_t  encoder_value;     // 编码器位置 (rad × 65535), little-endian
    int16_t  present_current;   // 当前电流 (mA), little-endian
} CAN_Fbk_t;                    // 共 8 字节
#pragma pack(pop)

_Static_assert(sizeof(CAN_Fbk_t) == 8, "CAN_Fbk_t must be 8 bytes");
```

### 反馈值换算

```
编码器位置 (rad) = encoder_value / 65535.0
当前电流 (A)     = present_current / 1000.0

示例:
  encoder_value = 65535  → 1.0 rad
  encoder_value = -32768 → -0.5 rad
  present_current = 3500 → 3.5 A
```

> **注意**: CAN 反馈帧中 **没有编码器速度** 字段 (与 RS485 协议不同)。如需速度信息，Linux 端应在两个连续位置采样之间求差分：`velocity = (pos[n] - pos[n-1]) / dt`，其中 `dt = 1ms` (1 kHz 通讯频率)。

---

## 6. 工作模式枚举

```c
typedef enum {
    FREE       = 0,  // 自由/解锁状态，线圈无电流
    FIX_LIMIT  = 1,  // 固定磁力模式（电流控制），使用 des_coil_current
    ADAPTATION = 2,  // 自适应模式，使用 des_coil_current
    DEBUG      = 3,  // 调试模式（CLI 控制，忽略 CAN 命令）
    MRC_RESET  = 4,  // 碰撞恢复指令
    ZERO       = 5,  // 编码器归零
    REFRESH    = 6,  // 刷新/更新
} MRC_Mode;
```

### 各模式行为说明

| 模式 | 命令帧 des_coil_current | 设备行为 |
|------|------------------------|----------|
| FREE (0) | 忽略 | 线圈断电，设备自由转动 |
| FIX_LIMIT (1) | **有效** | 按指定电流驱动线圈，产生固定制动力矩 |
| ADAPTATION (2) | **有效** | 自适应控制，同时启用碰撞检测 |
| DEBUG (3) | 忽略 | 进入 CLI 调试模式，CAN 命令被忽略 |
| MRC_RESET (4) | 忽略 | 碰撞恢复：清除碰撞标志，执行退磁序列 |
| ZERO (5) | 忽略 | 将编码器当前位置设为零点 |
| REFRESH (6) | 忽略 | 保留 |

### 碰撞保护逻辑

当设备处于 ADAPTATION 模式检测到碰撞时：
1. `collision_flag` 置为 `0x01`
2. 设备自动执行退磁保护
3. 此后只响应 `MRC_RESET` 命令
4. 收到 `MRC_RESET` 后恢复正常，`collision_flag` 清零

---

## 7. 通讯时序

```
    Host (Linux)                    Device (STM32)
        |                               |
        |--- CMD (0x101, 8B) ---------->|  ISR 中断接收
        |                               |  立即打包反馈
        |<-- FBK (0x201, 8B) ----------|  ISR 中发送
        |                               |
        |        ~1 ms interval         |
        |                               |
        |--- CMD (0x101, 8B) ---------->|
        |<-- FBK (0x201, 8B) ----------|
        |                               |
```

- **通讯频率**: 设计为 1 kHz (每 1ms 一次命令-反馈交互)
- **响应方式**: 设备在 FDCAN RX 中断中 **立即** 回复反馈 (不等待主循环)
- **典型延迟**: < 100 µs (ISR 中直接处理)
- **无需软件 CRC**: CAN 总线硬件提供 CRC-15 校验和 ACK 机制

---

## 8. Linux SocketCAN 示例代码

### 8.1 初始化 CAN 接口

```bash
# 设置 CAN 接口波特率并启动
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up

# (可选) 查看接口状态
ip -details link show can0
```

### 8.2 发送命令帧

```c
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#define CAN_CMD_ID(id) (0x100U | ((uint8_t)(id)))
#define CAN_FBK_ID(id) (0x200U | ((uint8_t)(id)))

int can_socket_init(const char *ifname)
{
    int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) {
        perror("socket");
        return -1;
    }

    struct ifreq ifr;
    strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1);
    if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
        perror("ioctl");
        close(s);
        return -1;
    }

    struct sockaddr_can addr = {0};
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("bind");
        close(s);
        return -1;
    }

    return s;
}

int send_mrc_cmd(int sock, uint8_t device_id, uint8_t mode, int32_t current_mA)
{
    struct can_frame frame = {0};
    frame.can_id  = CAN_CMD_ID(device_id);
    frame.can_dlc = 8;

    // 直接按偏移填充（避免对齐问题）
    frame.data[0] = mode;
    frame.data[1] = 0x00;  // reserved
    // des_coil_current: int32_t, little-endian
    frame.data[2] = (uint8_t)(current_mA & 0xFF);
    frame.data[3] = (uint8_t)((current_mA >> 8) & 0xFF);
    frame.data[4] = (uint8_t)((current_mA >> 16) & 0xFF);
    frame.data[5] = (uint8_t)((current_mA >> 24) & 0xFF);
    frame.data[6] = 0x00;  // reserved
    frame.data[7] = 0x00;  // reserved

    ssize_t nbytes = write(sock, &frame, sizeof(frame));
    return (nbytes == sizeof(frame)) ? 0 : -1;
}
```

### 8.3 接收反馈帧

```c
int recv_mrc_fbk(int sock, uint8_t device_id,
                 uint8_t *mode, uint8_t *collision,
                 float *position_rad, float *current_A)
{
    struct can_frame frame;
    ssize_t nbytes = read(sock, &frame, sizeof(frame));
    if (nbytes < (ssize_t)sizeof(frame))
        return -1;

    if (frame.can_id != CAN_FBK_ID(device_id))
        return -2;  // 不是目标设备的反馈

    *mode      = frame.data[0];
    *collision = frame.data[1];

    // encoder_value: int32_t, little-endian, bytes 2-5
    int32_t enc_raw = (int32_t)(
        (uint32_t)frame.data[2]       |
        ((uint32_t)frame.data[3] << 8)  |
        ((uint32_t)frame.data[4] << 16) |
        ((uint32_t)frame.data[5] << 24)
    );
    *position_rad = (float)enc_raw / 65535.0f;

    // present_current: int16_t, little-endian, bytes 6-7
    int16_t cur_raw = (int16_t)(
        (uint16_t)frame.data[6] |
        ((uint16_t)frame.data[7] << 8)
    );
    *current_A = (float)cur_raw / 1000.0f;

    return 0;
}
```

### 8.4 完整通讯循环示例

```c
#include <time.h>

// 1 kHz 控制循环
void control_loop(int sock, uint8_t device_id)
{
    struct timespec ts;
    ts.tv_sec  = 0;
    ts.tv_nsec = 1000000;  // 1 ms

    float des_current = 2.0f;  // 2A

    while (1) {
        // 发送命令
        int32_t current_mA = (int32_t)(des_current * 1000.0f);
        send_mrc_cmd(sock, device_id, 1 /* FIX_LIMIT */, current_mA);

        // 接收反馈 (建议使用 select/poll 加超时)
        uint8_t mode, collision;
        float position, current;
        if (recv_mrc_fbk(sock, device_id, &mode, &collision,
                         &position, &current) == 0) {
            printf("Mode=%d Collision=%d Pos=%.4f rad Cur=%.3f A\n",
                   mode, collision, position, current);
        }

        nanosleep(&ts, NULL);
    }
}
```

---

## 9. 多设备通讯

固件支持通过 `device_id` 区分多个设备。每个设备有独立的 CMD/FBK ID 对：

| 设备 | 命令 ID | 反馈 ID |
|------|---------|---------|
| #1 | 0x101 | 0x201 |
| #2 | 0x102 | 0x202 |
| #3 | 0x103 | 0x203 |
| ... | ... | ... |
| #N | 0x100+N | 0x200+N |

Linux 端如需同时控制多个设备，建议：
1. 在同一个 socket 上发送不同 CMD ID 的帧
2. 使用 `CAN_RAW_FILTER` 过滤器只接收 0x2xx 范围的反馈帧
3. 接收时根据 `can_id & 0xFF` 区分设备来源

---

## 10. 与 RS485 协议的差异对比

| 特性 | RS485 (USART2) | CAN (FDCAN1) |
|------|----------------|--------------|
| 帧长度 | 命令 10B / 反馈 17B | 均为 8B |
| 帧头 | 0xFE 0xEE | 无 (CAN ID 区分) |
| 设备地址 | 帧内 id 字段 | CAN ID 编码 |
| 校验 | CRC-16-CCITT (软件) | CRC-15 (硬件) |
| 编码器速度 | 有 (int32, rad/s×1000) | **无** (需上位机差分计算) |
| 波特率 | 4 Mbps | 1 Mbps |
| 总线拓扑 | 点对点 / 半双工多点 | 多点总线 |
| 后端切换 | `mrc_com_backend.h` 中 `#define MRC_COM_BACKEND` |

---

## 11. 设备 ID 和电阻参数

设备 ID 和线圈电阻存储在 STM32H723 的 Flash 中：

| 参数 | Flash 地址 | 默认值 |
|------|-----------|--------|
| Device ID | 0x080E0000 (最后扇区) | 1 |
| Coil Resistance | 0x080C0000 (倒数第二扇区) | 可通过 CLI 设置 |

通过 Debug CLI (USART1, 115200 baud) 可修改：
```
ID CHECK        # 查看当前 ID
ID CHANGE 3     # 修改 ID 为 3
RES CHECK       # 查看线圈电阻
RES CHANGE 5.2  # 修改线圈电阻为 5.2Ω
```

---

## 12. 错误处理建议

### Linux 端应处理的异常情况

1. **超时无反馈**: 发送命令后未在合理时间 (如 5ms) 内收到反馈
   - 可能原因: 设备断电、CAN 线断开、波特率不匹配、设备处于 DEBUG 模式
   - 建议: 使用 `select()` / `poll()` 设置超时

2. **碰撞标志**: `collision_flag == 0x01`
   - 设备已进入碰撞保护，只接受 `MRC_RESET` 命令
   - 应在上位机逻辑中处理碰撞恢复流程

3. **CAN 总线错误**: 通过 `/sys/class/net/can0/statistics/` 监控
   ```bash
   cat /sys/class/net/can0/statistics/bus_error
   cat /sys/class/net/can0/statistics/error_warning
   cat /sys/class/net/can0/statistics/error_passive
   cat /sys/class/net/can0/statistics/bus_off
   ```

4. **TX FIFO 满**: 设备端 TX FIFO 为 8 帧深度，正常 1 kHz 通讯不会溢出

---

## 13. 典型使用流程

```
1. 上电 / 初始化
   Host: 配置 CAN 接口 1 Mbps
   Device: 自动初始化，进入 FREE 模式

2. 编码器归零
   Host → CMD: mode=ZERO(5), current=0
   Device → FBK: mode=ZERO, encoder=0

3. 固定力矩控制
   Host → CMD: mode=FIX_LIMIT(1), current=3000 (3A)
   Device → FBK: mode=FIX_LIMIT, encoder=xxx, current≈3000

4. 自适应模式
   Host → CMD: mode=ADAPTATION(2), current=2000 (2A)
   Device → FBK: 持续反馈，碰撞检测激活

5. 碰撞发生
   Device → FBK: collision_flag=0x01, 自动退磁
   Host: 检测到碰撞，停止发送力矩命令

6. 碰撞恢复
   Host → CMD: mode=MRC_RESET(4), current=0
   Device → FBK: collision_flag=0x00, 恢复正常

7. 释放
   Host → CMD: mode=FREE(0), current=0
   Device → FBK: mode=FREE, 线圈断电
```

---

## 14. 文件索引 (固件源码参考)

| 文件 | 说明 |
|------|------|
| `Devices/Inc/can_com.h` | CAN 帧结构体定义 (CAN_Cmd_t, CAN_Fbk_t, MRC_Can_Com_t) |
| `Devices/Src/can_com.c` | CAN 打包/解包/发送函数实现 |
| `Devices/Inc/bsp_fdcan.h` | FDCAN BSP 层: 波特率常量、API 声明 |
| `Devices/Src/bsp_fdcan.c` | FDCAN 初始化、滤波器、收发底层实现 |
| `Core/Src/fdcan.c` | CubeMX 生成的 FDCAN1 外设初始化 |
| `Core/Inc/fdcan.h` | FDCAN 外设句柄声明 |
| `Devices/Inc/mrc_com_backend.h` | 通讯后端选择宏 (RS485/CAN 切换) |
| `Devices/Inc/mrc_protocol.h` | MRC_Mode 枚举、RS485 帧结构体 |
| `Devices/Src/drv_mrc.c` | MRC_Can_Process()、MRC_Can_send_data() |
| `Devices/Inc/drv_mrc.h` | Device_MRC_t 完整结构体定义 |
| `Core/Src/main.c` | fdcan1_rx_callback() ISR 回调实现 |

---

## 15. 快速验证

使用 `can-utils` 工具快速验证通讯:

```bash
# 安装 can-utils
sudo apt install can-utils

# 监听所有 CAN 帧
candump can0

# 发送一个 FIX_LIMIT 命令 (ID=1, 电流=2000mA=2A)
# 0x101 = CMD_ID(1)
# data: 01 00 D0070000 0000
#       mode=1(FIX_LIMIT), reserved, 2000(LE), reserved
cansend can0 101#0100D007000000000

# 发送 FREE 命令 (释放)
cansend can0 101#0000000000000000

# 发送编码器归零命令
cansend can0 101#0500000000000000

# 发送碰撞恢复命令
cansend can0 101#0400000000000000
```
