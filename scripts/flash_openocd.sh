#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 || $# -gt 3 ]]; then
    echo "usage: $0 <firmware.elf> [interface.cfg] [target.cfg]" >&2
    exit 2
fi

image=$1
interface_cfg=${2:-interface/stlink.cfg}
target_cfg=${3:-target/stm32h7x.cfg}

if ! command -v openocd >/dev/null 2>&1; then
    echo "error: openocd is not installed or not on PATH" >&2
    exit 127
fi
if [[ ! -f "$image" ]]; then
    echo "error: firmware image does not exist: $image" >&2
    exit 1
fi

echo "Flashing $image with OpenOCD ($interface_cfg, $target_cfg)"
# 'program' erases and writes only sectors covered by the ELF sections.
exec openocd \
    -f "$interface_cfg" \
    -f "$target_cfg" \
    -c "program {$image} verify reset exit"
