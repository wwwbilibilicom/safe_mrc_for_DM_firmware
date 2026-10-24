# Linux CMake Migration Design

## Goal

Enable reproducible Linux development for the STM32H723 firmware with CMake and Ninja while keeping STM32CubeMX as the source of truth for MCU, clock, peripheral, DMA, GPIO, and interrupt configuration. The existing Keil MDK project remains available as a fallback until the GCC build and hardware behavior are validated.

## Scope

The migration covers the firmware build, link, flash, debug entry points, CubeMX regeneration workflow, and host-side build checks. It does not change the firmware protocol, control behavior, peripheral configuration, PCB, or Python GUI behavior unless a build portability fix is required.

## Architecture

`safeMRC_for_DM_firmware.ioc` remains the hardware configuration source. CubeMX is switched from `MDK-ARM V5` to its CMake generator and continues generating `Core/` and `Drivers/`. A hand-maintained top-level `CMakeLists.txt` includes the generated sources plus `Common/`, `Devices/`, the STM32H7 HAL/CMSIS sources, a GCC startup file, and a custom STM32H723 GNU linker script.

The linker script preserves the device memory map used by the Keil scatter file: 1 MB internal flash, 128 KB DTCM for normal RW/ZI and stack, AXI SRAM for overflow RW/ZI, and a dedicated D2 SRAM region for objects marked `section("RAM_D2")`. The D2 base and length are checked against the CMSIS device definitions (`D2_AHBSRAM1_BASE`, `D2_AHBSRAM2_BASE`) and the existing scatter file before the GNU linker script is finalized. This is required for DMA buffers used by ADC, SPI, USART, and the debug CLI.

The old `MDK-ARM/` project, ARMASM startup file, and scatter file are not deleted. Linux-specific files are independent so CubeMX regeneration cannot overwrite them.

## Build Interface

The project exposes these CMake cache options:

- `MRC_COM_BACKEND=RS485` or `MRC_COM_BACKEND=CAN`, mapped to the existing compile-time backend definitions.
- `CMAKE_BUILD_TYPE=Debug` or `Release`.
- `CMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake`.

The default build produces an ELF, HEX, BIN, and MAP file under `build/<configuration>/`. Ninja targets provide `firmware`, `hex`, `bin`, `flash`, and `size` actions where the required host tools are available.

## CubeMX Regeneration Contract

Before the first regeneration, the current Keil project is committed or copied. CubeMX is configured with `TargetToolchain=CMake`, `KeepUserCode=true`, and backup enabled. Generated files remain under `Core/` and `Drivers/` during the initial migration. After regeneration, `git diff` is reviewed before building.

CubeMX-owned files are allowed to change during regeneration. Hand-maintained CMake, startup, linker, and flash/debug files are outside CubeMX's generated set. Application logic stays in `Common/` and `Devices/`; only small integration hooks remain in CubeMX `USER CODE` blocks.

## Portability Fixes

The legacy `Devices/Inc/stdint.h` is an ARMCC compatibility header and shadows the standard GCC `<stdint.h>`. It will be renamed to `Devices/Inc/armcc_compat_stdint.h`, and the two project files that explicitly include it will use the compiler standard `<stdint.h>` instead. The renamed header remains in the tree for Keil fallback/reference, but it is no longer a candidate for GCC's standard-header lookup.

The GCC startup file and linker script must define the interrupt vector symbols expected by the generated interrupt handlers, provide `Reset_Handler`, and place the `RAM_D2` input section at an address accessible to DMA1/DMA2.

## Validation

Validation is staged:

1. Compile and link both RS-485 and CAN configurations with GCC.
2. Check ELF section addresses and sizes, especially `.isr_vector`, flash, DTCM/AXI SRAM, and `RAM_D2`.
3. Run protocol/CRC and hardware-independent module checks on the host where practical.
4. Flash a development board with OpenOCD or ST-LINK and verify reset, clock startup, PWM, ADC/DMA, encoder capture, 1 kHz and 10 kHz loops, RS-485, FDCAN, Flash persistence, and collision demagnetization.
5. Regenerate from CubeMX once more, repeat the build, and confirm that generated changes do not remove hand-maintained files or application sources.

## Risks and Decisions

- CubeMX CMake output varies between CubeMX releases. The generated files are treated as input, while the top-level CMake remains the stable project interface.
- The exact GCC startup/linker output cannot be validated without compiling and, ultimately, running on hardware. Link-time map checks are mandatory before flashing.
- The custom D2 SRAM placement is safety-critical for DMA correctness and must not be replaced by a default linker script.
- Keil and GCC may produce different floating-point code and timing. Control-loop timing and collision behavior must be checked on hardware, not inferred from a successful build.
