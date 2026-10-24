#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 1 ]]; then
    echo "usage: $0 <firmware.bin>" >&2
    exit 2
fi

image=$1
if ! command -v st-flash >/dev/null 2>&1; then
    echo "error: st-flash is not installed or not on PATH" >&2
    exit 127
fi
if [[ ! -f "$image" ]]; then
    echo "error: firmware image does not exist: $image" >&2
    exit 1
fi

echo "Flashing $image with st-flash at 0x08000000"
st-flash write "$image" 0x08000000

# Older stlink releases do not implement the reset subcommand. A successful
# write is still useful in that case; the probe or power cycle can reset the MCU.
if ! st-flash reset; then
    echo "warning: st-flash reset is unavailable; reset the target manually" >&2
fi
