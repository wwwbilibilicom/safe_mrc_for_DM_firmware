# Debug CLI & Flash Persistence

## USART1 Debug CLI

The firmware exposes a lightweight CLI on USART1 (DMA + IDLE) for development and testing.

### Enable Debug Mode

```
DEBUG ON       # Enter debug mode
DEBUG OFF      # Exit debug mode
```

Once in DEBUG mode, the following commands are available:

### Control Mode

```
MODE VOLTAGE   # Switch to voltage control
MODE CURRENT   # Switch to current control
```

### Set Targets

```
SET VOLTAGE <value>   # Range: [-12.0, 12.0] V
SET CURRENT <value>   # Range: [-5.0, 5.0] A
```

### Device ID Management

```
ID CHECK              # Print current device ID
ID CHANGE <n>         # Set device ID (range: 0-15)
```

### Coil Resistance Management

```
RES CHECK             # Print current coil resistance (Ohm)
RES CHANGE <ohm>      # Set coil resistance (range: 0.01-10.0 Ohm)
```

### Help

```
-h / --help           # Print concise help menu
```

> **Note:** Invalid commands or out-of-range parameters produce a clear hint. Changing ID updates both the communication address and feedback packet ID immediately.

## Persistent Flash Parameters

**MCU:** STM32H723xG (1 MB Flash, 128 KB sectors, single bank)

| Parameter       | Address        | Sector           |
|-----------------|----------------|------------------|
| Device ID       | `0x080E0000`   | Last sector      |
| Coil resistance | `0x080C0000`   | Second-to-last   |

Both parameters are saved with sector erase + 32-byte flashword programming, without overlapping sectors to avoid unintended erasures.

## First Boot vs Later Resets

### First Boot (after programming)

The resistance address is in erased state (`0xFF...`). Firmware detects this and:
1. Loads a default resistance (e.g., 4.22 Ohm)
2. Writes it to `FLASH_RES_ADDRESS` for future boots

### Later Resets

1. Firmware reads resistance from flash
2. Validates range; if invalid, falls back to default and rewrites flash

### Runtime Changes

When using `ID CHANGE` or `RES CHANGE`, the new value is immediately persisted to the corresponding flash address.
