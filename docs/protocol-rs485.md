# RS-485 Communication Protocol

## Hardware Interface

| Parameter | Value |
|-----------|-------|
| Physical Layer | RS-485 differential bus |
| Baudrate | 4 Mbps (4,000,000 bps) |
| Topology | Multi-drop (one master, multiple slaves) |
| Connector | Standard 485-A/B differential pair |
| Termination | 120 Ohm resistors recommended at both ends |
| Cable | Shielded twisted pair, < 50 m for 4 Mbps |

## Protocol Overview

- **Pattern**: Request-Response (master sends command, slave responds)
- **Frame Format**: Binary, fixed length
- **Error Checking**: CRC-16-CCITT (poly=0x1021, init=0xFFFF, no xorout, no reflection)
- **Bus Arbitration**: Only one device transmits at a time (slave responds only after receiving a command)

## Command Message (Host -> Device, 10 bytes)

| Byte Index | Field Name     | Size (bytes) | Description                                     |
|------------|----------------|--------------|--------------------------------------------------|
| 0-1        | Header         | 2            | Frame header, fixed `0xFE`, `0xEE`              |
| 2          | Device ID      | 1            | Target device address (0-255)                    |
| 3          | Mode           | 1            | Work mode (see Mode Field below)                 |
| 4-7        | Target Current | 4            | Desired coil current, int32, little-endian (mA)  |
| 8-9        | CRC-16-CCITT   | 2            | CRC of bytes 0-7, little-endian                  |

## Feedback Message (Device -> Host, 17 bytes)

| Byte Index | Field Name       | Size (bytes) | Description                                                  |
|------------|------------------|--------------|--------------------------------------------------------------|
| 0-1        | Header           | 2            | Frame header, fixed `0xFE`, `0xEE`                           |
| 2          | Device ID        | 1            | Responding device address                                    |
| 3          | Mode             | 1            | Current work mode                                            |
| 4          | Collision Flag   | 1            | 0: safe, 1: collision detected                               |
| 5-8        | Encoder Value    | 4            | Encoder reading, int32, little-endian (deg * 1000)           |
| 9-12       | Encoder Velocity | 4            | Encoder angular velocity, int32, little-endian (deg/s * 1000)|
| 13-14      | Present Current  | 2            | Coil current, int16, little-endian (mA)                      |
| 15-16      | CRC-16-CCITT     | 2            | CRC of bytes 0-14, little-endian                             |

> **Note:** All multi-byte fields use little-endian byte order. CRC is calculated over all bytes except the CRC field itself.

## CRC-16-CCITT

| Parameter | Value |
|-----------|-------|
| Polynomial | 0x1021 |
| Initial Value | 0xFFFF |
| Input/Output Reflection | None |
| Final XOR | None |
| Command Range | Bytes 0-7 |
| Feedback Range | Bytes 0-14 |

```c
uint16_t crc_ccitt(uint16_t crc, const uint8_t *data, size_t len);
```

## Mode Field Definition (`MRC_Mode`)

| Value | Name       | Description                                  |
|-------|------------|----------------------------------------------|
| 0     | FREE       | Free mode - coil de-energized                |
| 1     | FIX_LIMIT  | Fixed current mode - uses `des_coil_current` |
| 2     | ADAPTATION | Adaptive mode - collision detection active   |
| 3     | DEBUG      | Debug mode - CAN/RS-485 commands ignored     |
| 4     | MRC_RESET  | Collision recovery - clears collision flag   |
| 5     | ZERO       | Zero encoder - sets current position as zero |
| 6     | REFRESH    | Reserved / refresh                           |

## Communication Sequence

```
Host                          Device
  |                              |
  |--- 10-byte Command Frame -->|
  |                              | (verify ID + CRC, parse command)
  |<-- 17-byte Feedback Frame --|
  |                              |
  (verify CRC, read status)
```

1. The host sends a 10-byte command frame to the RS-485 bus.
2. The target device verifies the ID and CRC, then parses the command.
3. The device replies with a 17-byte feedback frame containing its current status.
4. The host verifies the feedback CRC and reads the status.

## Physical Layer Notes

- All nodes should use RS-485 transceivers; termination resistors (120 Ohm) are recommended on A/B lines.
- At 4 Mbps, cable length should be limited to several tens of meters with shielded twisted pair.
- In case of communication errors, the host should retransmit the command.

## Backend Selection

The firmware supports RS-485 and CAN backends, selected at compile-time in `Devices/Inc/mrc_com_backend.h`:

```c
#define MRC_COM_RS485  0
#define MRC_COM_CAN    1

#define MRC_COM_BACKEND  MRC_COM_CAN   // <- change this line to switch
```

See also: [CAN Protocol](CAN_Protocol_Handoff.md)
