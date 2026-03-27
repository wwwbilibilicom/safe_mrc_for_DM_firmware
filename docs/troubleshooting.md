# Troubleshooting

## Safety Features

1. **Automatic Demagnetization** - On collision detection, coils are automatically demagnetized
2. **Voltage Limiting** - Coil voltage is limited to safe operating range
3. **CRC Validation** - All communication messages are validated using CRC-16
4. **Timeout Protection** - DMA operations include timeout protection
5. **Error Recovery** - Automatic recovery from communication errors
6. **Bus Conflict Prevention** - Request-response pattern prevents simultaneous transmissions

## Common Issues

### Communication Not Working

- Check UART configuration in STM32CubeMX
- Verify DMA settings for USART2
- Ensure IDLE interrupt is enabled
- Confirm request-response timing
- For CAN: verify FDCAN1 pin configuration (PB8 RX / PB9 TX)

### CRC Errors

- Verify protocol implementation on host side
- Check byte order (little-endian) and data types
- Ensure consistent CRC calculation (poly=0x1021, init=0xFFFF)
- See [RS-485 Protocol](protocol-rs485.md) for CRC details

### DMA Issues

- Verify DMA stream configuration
- Check interrupt priorities
- Ensure proper buffer alignment

### Bus Conflicts

- Ensure only one device responds at a time
- Check device ID configuration
- Verify request-response timing

## Debug Output

The system provides debug output through USART1:

- Device initialization status
- Communication errors
- Collision detection events
- Voltage/current changes
- Interactive CLI commands (see [Debug CLI](debug-cli.md))

## Host Tool Connection Issues

- Check serial port permissions and cable quality
- Verify device power supply
- Use a reliable USB-to-serial adapter for high-speed operation (4 Mbps)
- Ensure both sides use the same baud rate, frame format, and CRC settings
- See also: [Python SDK documentation](../scripts/README.md)
