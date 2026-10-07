cmake_minimum_required(VERSION 3.20)
project(hsps_robot C CXX ASM)

# ==============================================================================
# 1. Processor and compiler settings (ARM Cortex-M7 for STM32H7)
# ==============================================================================
set(CMAKE_SYSTEM_NAME Generic)          # Bare-metal (no OS)
set(CMAKE_SYSTEM_PROCESSOR arm)         # ARM architecture

# Cross-compilers (must be in system PATH)
set(CMAKE_C_COMPILER "arm-none-eabi-gcc")
set(CMAKE_CXX_COMPILER "arm-none-eabi-g++")
set(CMAKE_ASM_COMPILER "arm-none-eabi-gcc")

# CPU-specific compiler flags for Cortex-M7 with hardware FPU
add_compile_options(
    -mcpu=cortex-m7          # Target: ARM Cortex-M7
    -mthumb                  # Use Thumb-2 instruction set
    -mfpu=fpv5-d16           # Hardware FPU: single+double precision, 16 D-registers
    -mfloat-abi=hard         # Hard float ABI (FPU registers used for function args)
    -O0 -g3                  # No optimization + maximum debug info (for development)
    -Wall -Wextra            # Enable all common compiler warnings
    -ffunction-sections      # Place each function in its own section (for --gc-sections)
    -fdata-sections          # Place each variable in its own section (for --gc-sections)
)

# Language standard settings
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c11")                 # C11 standard
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -fno-rtti -fno-exceptions -fno-threadsafe-statics")
#                                        ^^^^^^^^^ ^^^^^^^^^ ^^^^^^^^^^^^^^^ ^^^^^^^^^^^^^^^^^^^^^^^^^
#                                        C++17     No RTTI   No exceptions   No static-init guards
#                                        (saves flash/RAM on embedded target)

# Preprocessor defines — select the correct STM32 device header from CMSIS
add_definitions(-DSTM32H7A3xx -DSTM32H7A3xxQ)

# ==============================================================================
# 2. Project directory structure
# ==============================================================================
# The project uses a layered architecture:
#   1_logic    — application logic (FOC, PID, servo loops)
#   3_abstract — hardware abstraction interfaces
#   4_driver   — MCU-specific peripheral drivers (RCC, GPIO, SPI, TIM, startup)
set(LOGIC_DIR    "${CMAKE_SOURCE_DIR}/1_logic")
set(ABSTRACT_DIR "${CMAKE_SOURCE_DIR}/3_abstract")
set(DRIVER_DIR   "${CMAKE_SOURCE_DIR}/4_driver/stm32h7a3zit6_Q")

# ==============================================================================
# 3. Automatic source file discovery (.c, .cpp, .s)
# ==============================================================================
# GLOB_RECURSE scans all subdirectories under the specified paths.
# Adding a new .c/.cpp/.s file to any of these folders automatically
# includes it in the build — no manual CMakeLists.txt edits needed.
file(GLOB_RECURSE SOURCES
    "main.cpp"                                              # Application entry point
    "${LOGIC_DIR}/*.c"   "${LOGIC_DIR}/*.cpp"               # FOC, PID, servo logic
    "${ABSTRACT_DIR}/*.c" "${ABSTRACT_DIR}/*.cpp"           # HAL abstractions (if any)
    "${DRIVER_DIR}/*.c"  "${DRIVER_DIR}/*.cpp" "${DRIVER_DIR}/*.s"  # Peripheral drivers + startup assembly
)

# ==============================================================================
# 4. Automatic header include-path discovery (.h, .hpp)
# ==============================================================================
# This block finds EVERY directory containing a header file anywhere in the
# project tree and adds it to the compiler's include search path.
# This allows simple includes like #include "gpio.h" regardless of directory depth.

# Step 1: find all header files in the project
file(GLOB_RECURSE ALL_HEADERS 
    "${CMAKE_SOURCE_DIR}/*.h" 
    "${CMAKE_SOURCE_DIR}/*.hpp"
)

# Step 2: extract the parent directory of each header file
set(INCLUDE_DIRS "")
foreach(_headerFile ${ALL_HEADERS})
    get_filename_component(_dir ${_headerFile} DIRECTORY)
    list(APPEND INCLUDE_DIRS ${_dir})
endforeach()

# Step 3: remove duplicate paths (many headers may share the same directory)
list(REMOVE_DUPLICATES INCLUDE_DIRS)

# Note: if building inside the source tree, you may want to filter out
# build/cmake directories from INCLUDE_DIRS. CLion typically uses an
# out-of-source build directory, so this is usually not needed.

# ==============================================================================
# 5. Build the firmware ELF executable
# ==============================================================================
add_executable(firmware.elf ${SOURCES})

# Add all discovered header directories to the include search path
target_include_directories(firmware.elf PRIVATE ${INCLUDE_DIRS})

# ==============================================================================
# 6. Linker configuration
# ==============================================================================
# Use the MCU-specific linker script that defines Flash/RAM memory regions
set(LINKER_SCRIPT "${DRIVER_DIR}/startup/STM32H7A3ZITXQ_FLASH.ld")

target_link_options(firmware.elf PUBLIC
    -T${LINKER_SCRIPT}          # Linker script with memory map
    -mcpu=cortex-m7             # Must match compile options
    -mthumb
    -mfpu=fpv5-d16
    -mfloat-abi=hard
    -Wl,--gc-sections           # Remove unused code/data sections (saves flash)
    --specs=nano.specs          # Use newlib-nano (smaller libc for embedded)
    --specs=nosys.specs         # No OS syscalls (bare-metal stubs)
    -Wl,-Map=${CMAKE_PROJECT_NAME}.map   # Generate a .map file for memory analysis
    -Wl,--print-memory-usage    # Print Flash/RAM usage summary after linking
)

# ==============================================================================
# 7. Post-build utilities (size report and flash programming)
# ==============================================================================

# Print firmware section sizes after every successful build
add_custom_command(TARGET firmware.elf POST_BUILD
    COMMAND arm-none-eabi-size firmware.elf
    COMMENT "Firmware Size:"
)

# 'make flash' or 'cmake --build . --target flash' — programs the MCU via ST-Link
add_custom_target(flash
    COMMAND openocd -f interface/stlink.cfg -f target/stm32h7x.cfg -c "program firmware.elf verify reset exit"
    DEPENDS firmware.elf
    COMMENT "Flashing..."
)