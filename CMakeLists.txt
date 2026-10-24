cmake_minimum_required(VERSION 3.20)

project(safeMRC_for_DM_firmware LANGUAGES C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

set(MRC_COM_BACKEND "RS485" CACHE STRING "Communication backend (RS485 or CAN)")
set_property(CACHE MRC_COM_BACKEND PROPERTY STRINGS RS485 CAN)
string(TOUPPER "${MRC_COM_BACKEND}" MRC_COM_BACKEND)
if(MRC_COM_BACKEND STREQUAL "RS485")
    set(MRC_COM_BACKEND_VALUE 0)
elseif(MRC_COM_BACKEND STREQUAL "CAN")
    set(MRC_COM_BACKEND_VALUE 1)
else()
    message(FATAL_ERROR "MRC_COM_BACKEND must be RS485 or CAN, got '${MRC_COM_BACKEND}'")
endif()

include(cmake/stm32h723_sources.cmake)

set(FIRMWARE_INCLUDES
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32H7xx_HAL_Driver/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32H7xx_HAL_Driver/Inc/Legacy
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32H7xx/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/Devices/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Common/Inc
)

add_executable(safeMRC_for_DM_firmware ${FIRMWARE_SOURCES})
set_target_properties(safeMRC_for_DM_firmware PROPERTIES
    OUTPUT_NAME safeMRC_for_DM_firmware
    SUFFIX .elf
)
target_include_directories(safeMRC_for_DM_firmware PRIVATE ${FIRMWARE_INCLUDES})
target_compile_definitions(safeMRC_for_DM_firmware PRIVATE
    USE_PWR_LDO_SUPPLY
    USE_HAL_DRIVER
    STM32H723xx
    MRC_COM_BACKEND=${MRC_COM_BACKEND_VALUE}
)
target_compile_options(safeMRC_for_DM_firmware PRIVATE
    -mcpu=cortex-m7
    -mthumb
    -mfpu=fpv5-d16
    -mfloat-abi=hard
    -ffunction-sections
    -fdata-sections
    -fno-common
    $<$<CONFIG:Debug>:-Og;-g3>
    $<$<CONFIG:Release>:-O2>
)
target_link_options(safeMRC_for_DM_firmware PRIVATE
    -mcpu=cortex-m7
    -mthumb
    -mfpu=fpv5-d16
    -mfloat-abi=hard
    --specs=nano.specs
    --specs=nosys.specs
    -u _printf_float
    -Wl,--gc-sections
    -Wl,--print-memory-usage
    -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/safeMRC_for_DM_firmware.map
    -T${CMAKE_CURRENT_SOURCE_DIR}/cmake/STM32H723VGHx.ld
)

add_custom_command(TARGET safeMRC_for_DM_firmware POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex
            $<TARGET_FILE:safeMRC_for_DM_firmware>
            ${CMAKE_CURRENT_BINARY_DIR}/safeMRC_for_DM_firmware.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary
            $<TARGET_FILE:safeMRC_for_DM_firmware>
            ${CMAKE_CURRENT_BINARY_DIR}/safeMRC_for_DM_firmware.bin
    VERBATIM
)

add_custom_target(size
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:safeMRC_for_DM_firmware>
    DEPENDS safeMRC_for_DM_firmware
    COMMENT "Firmware size (map: ${CMAKE_CURRENT_BINARY_DIR}/safeMRC_for_DM_firmware.map)"
    VERBATIM
)
