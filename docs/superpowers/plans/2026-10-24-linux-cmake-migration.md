# Linux CMake Migration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build and validate the STM32H723 firmware on Linux with CubeMX-generated CMake input, GCC/Ninja, and reproducible flash/debug commands while retaining the Keil project as a fallback.

**Architecture:** CubeMX remains the hardware configuration source and generates `Core/` and `Drivers/`. A hand-maintained top-level CMake layer adds `Common/`, `Devices/`, the HAL/CMSIS sources, GCC startup code, and a custom GNU linker script that preserves the existing D2 SRAM DMA section. Keil files remain unchanged until the GCC build is validated on hardware.

**Tech Stack:** STM32CubeMX, CMake, Ninja, `arm-none-eabi-gcc`, GNU binutils, OpenOCD or `st-flash`, STM32H7 HAL/CMSIS, host Python tooling.

---

### Task 1: Establish the migration baseline

**Files:**
- Inspect: `safeMRC_for_DM_firmware.ioc`
- Inspect: `MDK-ARM/safeMRC_for_DM_firmware.uvprojx`
- Inspect: `MDK-ARM/safeMRC_for_DM_firmware.sct`
- Inspect: `Core/Src/main.c`
- Inspect: `Devices/Src/drv_mrc.c`
- Inspect: `Devices/Src/mrc_com.c`

- [ ] **Step 1: Record the current generated project settings.**

Confirm the device (`STM32H723VGHx`), compiler macros (`USE_PWR_LDO_SUPPLY`, `USE_HAL_DRIVER`, `STM32H723xx`), include directories, source groups, selected communication backend, and the Keil memory regions.

- [ ] **Step 2: Verify the clean working tree.**

Run:

```bash
git status --short --branch
```

Expected: the migration starts from a clean branch, or any existing user changes are explicitly recorded before CubeMX regeneration.

- [ ] **Step 3: Create a baseline commit before CubeMX changes.**

Run:

```bash
git add safeMRC_for_DM_firmware.ioc MDK-ARM Core Common Devices Drivers README.md README_CN.md docs
git commit -m "chore: checkpoint Keil firmware before Linux migration"
```

Do not delete or rewrite `MDK-ARM/` files.

### Task 2: Switch CubeMX generation from Keil to CMake

**Files:**
- Modify through CubeMX UI: `safeMRC_for_DM_firmware.ioc`
- Generated and reviewed: `Core/`, `Drivers/`, CubeMX CMake files

- [ ] **Step 1: Enable backup and keep user code in CubeMX.**

Open the project in CubeMX, enable the option that backs up the previous generated result, and confirm that `Keep User Code` remains enabled. The current `.ioc` has `DeletePrevious=true` and `BackupPrevious=false`; do not generate until backup is enabled or the baseline commit is confirmed.

- [ ] **Step 2: Select the CMake toolchain.**

In `Project Manager -> Toolchain / IDE`, select `CMake` and generate code. Do not hand-edit the unknown CubeMX toolchain enum; let CubeMX write it.

- [ ] **Step 3: Review generated changes.**

Run:

```bash
git status --short
git diff --stat
git diff -- safeMRC_for_DM_firmware.ioc
```

Expected: `TargetToolchain` changes from the Keil target to CubeMX's CMake target, while MCU/peripheral settings remain unchanged. If CubeMX rewrites application logic outside `USER CODE` blocks, restore it from the baseline and keep the regeneration diff focused.

- [ ] **Step 4: Commit only the intentional CubeMX transition.**

Run:

```bash
git add safeMRC_for_DM_firmware.ioc Core Drivers
git commit -m "build: switch CubeMX generation target to CMake"
```

### Task 3: Add the GCC toolchain and project source manifest

**Files:**
- Create: `cmake/arm-none-eabi.cmake`
- Create: `CMakeLists.txt`
- Create: `cmake/stm32h723_sources.cmake`
- Modify: `Devices/Inc/mrc_com_backend.h`

- [ ] **Step 1: Define the cross-compilation toolchain.**

Configure `arm-none-eabi-gcc`, `arm-none-eabi-g++`, `arm-none-eabi-objcopy`, `arm-none-eabi-size`, `CMAKE_SYSTEM_NAME=Generic`, Cortex-M7 CPU flags, hard-float FPU flags, and compiler dependency scanning. Fail configuration with a clear message if the compiler is unavailable.

- [ ] **Step 2: Enumerate the exact project sources.**

List the CubeMX-generated `Core/Src/*.c`, CMSIS system source, required HAL source files, every `Common/Src/*.c`, every `Devices/Src/*.c`, and the GCC startup assembly. Exclude unrelated HAL drivers only after link verification proves they are unused.

- [ ] **Step 3: Add include paths and definitions.**

Use the same include directories and definitions as Keil:

```text
Core/Inc
Drivers/STM32H7xx_HAL_Driver/Inc
Drivers/STM32H7xx_HAL_Driver/Inc/Legacy
Drivers/CMSIS/Device/ST/STM32H7xx/Include
Drivers/CMSIS/Include
Devices/Inc
Common/Inc
USE_PWR_LDO_SUPPLY
USE_HAL_DRIVER
STM32H723xx
```

Expose `MRC_COM_BACKEND` as a CMake cache option and translate `RS485` and `CAN` to the existing preprocessor values. Change the header's unconditional `#define MRC_COM_BACKEND MRC_COM_CAN` to an `#ifndef MRC_COM_BACKEND` default so a CMake definition can select either backend.

- [ ] **Step 4: Add firmware output conversions.**

Generate `safeMRC_for_DM_firmware.elf`, `.hex`, `.bin`, and `.map` in the selected build directory. Add a `size` target that runs `arm-none-eabi-size` and prints the map path.

- [ ] **Step 5: Configure and inspect the first source-only build.**

Run:

```bash
cmake -S . -B build/debug -G Ninja \
  -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake \
  -DCMAKE_BUILD_TYPE=Debug \
  -DMRC_COM_BACKEND=RS485
ninja -C build/debug
```

Expected at this stage: configuration succeeds; compilation may stop on the known legacy `Devices/Inc/stdint.h` issue or missing startup/linker files. Capture the first real compiler error before making portability changes.

### Task 4: Replace Keil-only startup and memory description

**Files:**
- Create: `startup/startup_stm32h723xx_gcc.s`
- Create: `cmake/STM32H723VGHx.ld`
- Modify: `CMakeLists.txt`
- Reference: `MDK-ARM/startup_stm32h723xx.s`
- Reference: `MDK-ARM/safeMRC_for_DM_firmware.sct`

- [ ] **Step 1: Add the GCC vector table and reset handler.**

Port the STM32H723 vector table and handler symbol names from the existing startup file to GNU assembler syntax. Define the initial stack symbol, `Reset_Handler`, weak default handlers, data copy from flash to RAM, BSS zeroing, `SystemInit`, and `__libc_init_array` before `main`.

- [ ] **Step 2: Define the linker memory regions.**

Verify the addresses against `Drivers/CMSIS/Device/ST/STM32H7xx/Include/stm32h723xx.h` and `MDK-ARM/safeMRC_for_DM_firmware.sct`, then declare the corresponding regions in `cmake/STM32H723VGHx.ld`:

```text
FLASH       0x08000000 1M
DTCMRAM     0x20000000 128K
AXI_SRAM    0x24000000 320K
RAM_D2      0x30000000 32K
```

Place `.isr_vector` first in `FLASH`, normal `.text`/`.rodata` in `FLASH`, ordinary RW/ZI in the available CPU SRAM, the initial stack at the top of DTCM, and `*(RAM_D2)` in a `NOLOAD` `RAM_D2` section. Define symbols required by startup and C library code.

- [ ] **Step 3: Link and inspect memory placement.**

Run:

```bash
ninja -C build/debug
arm-none-eabi-size build/debug/safeMRC_for_DM_firmware.elf
arm-none-eabi-objdump -h build/debug/safeMRC_for_DM_firmware.elf
arm-none-eabi-nm -n build/debug/safeMRC_for_DM_firmware.elf | rg 'ADC_BUFFER|rs485_rx_dma_buf|RAM_D2|Reset_Handler'
```

Expected: the vector table is in flash, `Reset_Handler` is present, and all `RAM_D2` objects resolve between the verified D2 SRAM start and end addresses (currently `0x30000000` through `0x30008000` in the CMSIS header). A linker overflow is fixed by reviewing actual section sizes, not by silently removing safety-critical buffers.

- [ ] **Step 4: Commit the independent GCC memory layer.**

Run:

```bash
git add CMakeLists.txt cmake startup
git commit -m "build: add GCC startup and STM32H723 linker layout"
```

### Task 5: Remove the ARMCC `stdint.h` blocker without breaking Keil

**Files:**
- Rename: `Devices/Inc/stdint.h` to `Devices/Inc/armcc_compat_stdint.h`
- Modify: `Devices/Inc/drv_encoder.h`
- Modify: `Devices/Src/drv_encoder.c`
- Modify: any source found by `rg '"stdint.h"'`
- Test: GCC compilation of all firmware sources

- [ ] **Step 1: Rename the ARMCC-only compatibility header.**

Run:

```bash
git mv Devices/Inc/stdint.h Devices/Inc/armcc_compat_stdint.h
```

This removes the name that shadows GCC's standard header while retaining the old implementation for the Keil fallback/reference path.

- [ ] **Step 2: Find and replace active legacy-header includes.**

Run:

```bash
rg -n '"stdint\\.h"|<stdint\\.h>' Core Common Devices Drivers
```

Change the two project includes currently found in `Devices/Src/drv_encoder.c` and `Devices/Inc/drv_encoder.h` from `"stdint.h"` to `<stdint.h>`. Leave CMSIS and HAL includes unchanged; they already request the standard header.

- [ ] **Step 3: Keep the Keil fallback isolated.**

Do not add a new `stdint.h` under `Devices/Inc`, and confirm the Keil project still resolves its compiler-provided standard header after the rename. The renamed `armcc_compat_stdint.h` is retained but is not included by any GCC source.

- [ ] **Step 4: Rebuild both communication backends.**

Run:

```bash
rm -rf build/debug
cmake -S . -B build/debug -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake -DCMAKE_BUILD_TYPE=Debug -DMRC_COM_BACKEND=RS485
ninja -C build/debug
rm -rf build/can-debug
cmake -S . -B build/can-debug -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake -DCMAKE_BUILD_TYPE=Debug -DMRC_COM_BACKEND=CAN
ninja -C build/can-debug
```

Expected: both configurations compile and link, producing ELF/HEX/BIN/MAP artifacts.

- [ ] **Step 5: Commit the compiler portability fix.**

Run:

```bash
git add Devices/Inc Devices/Src Common/Inc Common/Src Core/Inc Core/Src CMakeLists.txt
git commit -m "build: make firmware headers GCC-compatible"
```

### Task 6: Add flash and debug commands

**Files:**
- Create: `scripts/flash_openocd.sh`
- Create: `scripts/flash_stlink.sh`
- Modify: `CMakeLists.txt`
- Update: `README.md`
- Update: `README_CN.md`

- [ ] **Step 1: Add an OpenOCD flash target.**

Use the generated ELF or BIN and fail if `openocd` is unavailable. The command must reset and verify the image; it must not erase unrelated flash sectors by default.

- [ ] **Step 2: Add an ST-LINK fallback command.**

Use `st-flash write <bin> 0x08000000` followed by reset where supported. Document that ST-LINK tools may need udev permissions on Linux.

- [ ] **Step 3: Add GDB debug configuration.**

Document the `arm-none-eabi-gdb` + OpenOCD server sequence and provide an optional VS Code Cortex-Debug configuration without making VS Code a build dependency.

- [ ] **Step 4: Document CubeMX regeneration and build commands.**

Update both READMEs with:

```bash
cmake -S . -B build/debug -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake -DCMAKE_BUILD_TYPE=Debug
ninja -C build/debug
ninja -C build/debug size
ninja -C build/debug flash
```

Document that CubeMX regeneration must be followed by `git diff`, a clean configure, both backend builds, and hardware smoke tests.

### Task 7: Add host-side tests for portable protocol logic

**Files:**
- Create: `tests/host/test_protocol.py`
- Create: `tests/host/__init__.py`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Select hardware-independent functions.**

Use the standard-library `unittest` runner and the existing `scripts/safeMRC_sdk.py` implementation. Cover CRC-16-CCITT and protocol pack/unpack behavior because these have documented byte layouts and must remain identical between firmware and the Python SDK.

- [ ] **Step 2: Add deterministic vectors.**

Use known command and feedback byte sequences from `docs/protocol-rs485.md` and verify CRC, little-endian fields, frame length, header rejection, device-ID filtering, and invalid-CRC rejection.

- [ ] **Step 3: Run tests in the normal Linux workflow.**

Run:

```bash
python3 -m unittest discover -s tests/host -p 'test_*.py' -v
```

Expected: tests pass without requiring an STM32 board, HAL runtime, or serial adapter.

- [ ] **Step 4: Commit the portable test coverage.**

Run:

```bash
git add tests CMakeLists.txt docs
git commit -m "test: cover portable SafeMRC protocol behavior"
```

### Task 8: Perform hardware smoke and regeneration validation

**Files:**
- Update: `docs/troubleshooting.md`
- Update: `README.md`
- Update: `README_CN.md`
- Validate: `Core/`, `Drivers/`, `Common/`, `Devices/`, `cmake/`, `startup/`

- [ ] **Step 1: Flash the RS-485 Debug image.**

Use the project flash target, connect the Python host to the correct `/dev/ttyUSB*`, and confirm reset plus UART1 CLI output.

- [ ] **Step 2: Verify timing and safety behavior.**

Check TIM6 1 kHz state-machine execution, TIM4 10 kHz current updates, encoder capture, ADC DMA current sampling, PWM output, collision-triggered demagnetization, and Flash ID/resistance persistence.

- [ ] **Step 3: Verify both communication backends.**

Build and flash the CAN image, configure Linux SocketCAN at the documented bitrate, and verify command/feedback frames. Repeat RS-485 request/response and CRC checks.

- [ ] **Step 4: Regenerate once more from CubeMX.**

Make a harmless `.ioc` change, regenerate, restore the setting if needed, and verify that hand-maintained CMake, startup, linker, scripts, `Common/`, and `Devices/` files remain intact. Re-run both backend builds and map checks.

- [ ] **Step 5: Record the final workflow.**

Document the exact CubeMX -> `git diff` -> CMake/Ninja -> flash -> hardware test loop, including Linux udev permissions and the Keil fallback command.

- [ ] **Step 6: Commit the completed migration documentation.**

Run:

```bash
git add README.md README_CN.md docs/troubleshooting.md
git commit -m "docs: document Linux CubeMX CMake workflow"
```
