# CNC Coordinator Module ESP32-S3

Standalone firmware module for coordinating a 3-axis robotic peripheral, a spindle variable-frequency drive, and a coolant/lubrication pump system.


## Architecture Principles

1. **Hardware Decoupling:**
   * The `Drivers/` layer handles chip-specific registers and startup for the ESP32-S3.
   * The `Core/` layer is portable and hardware-agnostic. All machine protocols, packet framing, and parsing logic are written in pure C without register dependencies.
2. **Centralized Application State (`main.c`):**
   * The main super-loop and master state machine live directly inside `main.c`, serving as the single source of truth for machine coordination, safety interlocks, and sequencing.
3. **WebSerial:**
   * The operator interface is a standalone local Web application (`src/ui/index.html`) running in any browser communicating directly over native USB Type-C via the WebSerial API.


## Project Structure

modules/cnc/
│
├── Makefile                      # Root build and flash script
├── CMakeLists.txt                # Tooling config for LSP, IDEs, and linters
├── README.md                     # Project documentation and quick-start guide
│
├── docs/                         # Specifications and hardware documentation
│   ├── pinout.md                 # ESP32-S3 pin assignments (UART, RS485, relays, USB-JTAG)
│   └── protocol.md               # 3-axis robot serial wire-protocol specification
│
└── src/
    ├── main.c                    # System entry point, master state machine & super-loop
    │
    ├── Core/                     # Hardware-independent machine logic
    │   ├── command_parser/       # Host command stream decoder
    │   │   ├── command_parser.c  # Ring-buffer parser and G-code/CLI tokenizer
    │   │   └── command_parser.h  # Command definitions and parser interface
    │   │
    │   ├── pump/                 # Coolant and lubrication subsystem
    │   │   ├── pump.c            # Relay timing logic and pressure/flow monitoring
    │   │   └── pump.h            # Pump control interface and status queries
    │   │
    │   ├── robot_interface/      # 3-axis robot communication driver
    │   │   ├── robot_interface.c # Frame serialization, timeout tracking & ACK handling
    │   │   └── robot_interface.h # Robot dispatch API and motion state flags
    │   │
    │   └── vfd/                  # Spindle inverter driver and Modbus RTU
    │       ├── vfd.c             # Modbus state machine, frequency scaling & current polling
    │       └── vfd.h             # VFD run/stop interface and register maps
    │
    ├── Drivers/                  # Hardware layer for ESP32-S3 
    │   ├── include/
    │   │   ├── bsp.h             # Hardware contract: exposed HAL interface for Core/
    │   │   └── esp32s3_regs.h    # Physical register addresses and bitmasks (GPIO, UART, WDT)
    │   └── src/
    │       ├── startup.S         # Reset vector, stack pointer setup & .bss zeroing
    │       ├── bsp_system.c      # Disabling MWDT/SWD watchdogs and PLL clock configuration
    │       ├── bsp_gpio.c        # Direct register pin-level read/write operations
    │       ├── bsp_uart.c        # Non-blocking UART FIFO handling and ring buffers
    │       └── bsp_timer.c       # Hardware timer tick generator
    │
    ├── ld/                       # Linker scripts
    │   └── esp32s3.ld            # SRAM and Flash memory region mappings
    │
    └── ui/                       # Operator control interface
        └── index.html            # Standalone WebSerial dashboard (inlined HTML, CSS & JS)
---

## Directory & Subsystem Overview

### `src/main.c`
The central commander of the system. Initializes the BSP layer and runs the primary non-blocking super-loop:
* Polls incoming USB packets and forwards them to `command_parser`.
* Evaluates safety interlocks (e.g., forbidding axis movement if spindle RPM or pump flow is unconfirmed).
* Dispatches motion commands to `robot_interface`.
* Polls `vfd` status and drives the master state machine (`IDLE`, `RUNNING`, `ERROR`, `PAUSED`).

### `src/Core/`
* **`command_parser/`**: Consumes raw byte streams from the host interface and converts them into structured command payloads.
* **`pump/`**: Controls the coolant/lubrication relay, manages start-up priming delays, and monitors pressure/flow switches.
* **`robot_interface/`**: Formats motion commands into the wire protocol expected by the peripheral 3-axis robot, handling timeouts and `ACK`/`BUSY` responses.
* **`vfd/`**: Calculates Modbus RTU CRC16 checksums, issues spindle run/stop commands, sets target frequency, and polls operating current/load.

### `src/Drivers/`
Contains all low-level hardware routines for the ESP32-S3:
* **`Drivers/include/bsp.h`**: The boundary contract exposing safe, non-blocking hardware functions (`bsp_uart_send`, `bsp_gpio_set`, `bsp_millis`) to `Core/` and `main.c`.
* **Hardware Drivers**: Register-level implementations of clock trees, watchdog timer disable sequences, UART FIFO handling, and interrupt vectors.

### `src/ld/`
Contains the linker script defining internal SRAM boundaries, vector tables, and Flash mappings.

### `src/ui/`
Self-contained HTML/JavaScript application. Runs locally on the host machine without web servers or Node.js. Connects directly to the board's native USB-Serial port to provide manual jogging, spindle overrides, and telemetry graphs.

---

## Toolchain & Prerequisites

ESP32-S3 utilizes the dual-core Xtensa LX7 architecture. Compilation requires the standalone Xtensa toolchain and OpenOCD:

1. **Toolchain:** Download the standalone `xtensa-esp32s3-elf` GCC toolchain release from Espressif's crosstool-NG repository and add its `/bin` directory to your `PATH` or set `PREFIX` in the `Makefile`.
2. **OpenOCD & Make :**
   ```bash
   # Ubuntu / Debian
   sudo apt update
   sudo apt install openocd make

   # Arch Linux
   sudo pacman -S openocd make
   ```


## Quick Start

### 1. Build the Binary
```bash
make
```
Executes a unified build with Link-Time Optimization `-flto`. Artifacts are written to `build/`.

### 2. Flash via Native USB-JTAG
ESP32-S3 features an on-chip USB-Serial/JTAG controller directly on the Type-C port (GPIO19/GPIO20):
```bash
make flash
```

### 3. Clean Build Artifacts
```bash
make clean
```
