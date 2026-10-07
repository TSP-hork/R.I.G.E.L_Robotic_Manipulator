cmake_minimum_required(VERSION 3.16)
# --- 1. FORCE ARM TOOLCHAIN ---
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(TOOLCHAIN_PREFIX arm-none-eabi-)

set(CMAKE_C_COMPILER ${TOOLCHAIN_PREFIX}gcc)
set(CMAKE_ASM_COMPILER ${TOOLCHAIN_PREFIX}gcc)
set(CMAKE_OBJCOPY ${TOOLCHAIN_PREFIX}objcopy)
set(CMAKE_SIZE ${TOOLCHAIN_PREFIX}size)

find_program(ARM_GCC_PATH ${TOOLCHAIN_PREFIX}gcc)
if(NOT ARM_GCC_PATH)
    message(FATAL_ERROR "ARM GCC not found! Did you install arm-none-eabi-gcc?")
endif()


set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# --- 2. PROJECT SETTINGS ---
project(blade_firmware C ASM)

set(CMAKE_C_STANDARD 11)

# Settings for STM32G431
set(MCU_FLAGS "-mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16")
set(LINKER_SCRIPT "${CMAKE_SOURCE_DIR}/ld/STM32G431CBUx_flash.ld")

add_definitions(-DSTM32G431xx)

# --- COMPILER FLAGS ---

set(COMMON_FLAGS "${MCU_FLAGS} -Wall -Wextra -ffunction-sections -fdata-sections")

set(CMAKE_C_FLAGS_DEBUG "${COMMON_FLAGS} -Og -g3")

set(CMAKE_C_FLAGS_RELEASE "${COMMON_FLAGS} -O3 -flto")

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Linker Flags:

set(CMAKE_EXE_LINKER_FLAGS "${MCU_FLAGS} ${CMAKE_C_FLAGS} -T${LINKER_SCRIPT} -Wl,--gc-sections --specs=nosys.specs -lc -lm")
# Include directories
include_directories(
    Core
    Core/adc
    Core/clockup_to_170mhz
    Core/spi_slave
    Drivers/include
)

# Source files
file(GLOB_RECURSE SOURCES
    "Core/*.c"
    "Drivers/src/*.c"
    "Drivers/src/*.s"
)

add_executable(${PROJECT_NAME}.elf ${SOURCES})

# Generate BIN/HEX
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary ${PROJECT_NAME}.elf ${PROJECT_NAME}.bin
    COMMAND ${CMAKE_OBJCOPY} -O ihex ${PROJECT_NAME}.elf ${PROJECT_NAME}.hex
    COMMAND ${CMAKE_SIZE} ${PROJECT_NAME}.elf
)

# Flash command
add_custom_target(flash
    COMMAND openocd -f interface/stlink.cfg -f target/stm32g4x.cfg -c "program ${PROJECT_NAME}.elf verify reset exit"
    DEPENDS ${PROJECT_NAME}.elf
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMENT "Flashing firmware via OpenOCD..."
)
