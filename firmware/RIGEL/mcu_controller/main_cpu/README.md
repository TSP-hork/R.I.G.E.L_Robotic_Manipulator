# Main CPU Firmware Architecture

## Overview

This firmware runs on the STM32H7 (280 MHz Cortex-M7) and implements the
control algorithms for a robotic manipulator. The architecture is designed
around three principles:

1. **Portability** — algorithms are hardware-independent and can run on any MCU
2. **Extensibility** — new features are added without modifying existing code
3. **Determinism** — zero runtime overhead from the architecture itself

The system is split into layers. Each layer has exactly one responsibility
and exactly one reason to change.

```
┌─────────────────────────────────────────────┐
│  logic/         Pure algorithms (C/C++)     │ ← portable
│  utility/       Reusable building blocks    │ ← portable (planned)
│  abstract/      Hardware contract (C)       │ ← stable, rarely changes
│  driver/        Chip-specific code (C/ASM)  │ ← replaced per platform
└─────────────────────────────────────────────┘
```

The final system will implement a real-time cascade:

```
Trajectory (100 Hz)                             ← logic done (not integrated)
    → Kinematics (100 Hz)                       ← positioner done, wrist skeleton
        → Servo loop (1 kHz)                    ← logic done (not integrated)
            → FOC current loop (20 kHz)         ← IMPLEMENTED
                → PWM output (via FPGA)         ← IMPLEMENTED
```

Each level only talks to its neighbors. FOC does not know about kinematics.
Kinematics does not know about PWM registers.


## Current State

The FOC current control layer is fully operational on hardware. The higher
cascade levels are being built bottom-up as portable, desktop-testable logic:
the servo loop, the trajectory planner, and the 3-axis kinematics positioner
are written and validated but not yet wired into the running cascade. The wrist
(orientation) kinematics, runtime state machine, and safety guardian remain to
be done.

**What works today:**
- Field-Oriented Control with Clarke/Park transforms
- PI current regulators (d-axis and q-axis)
- Space-Vector PWM modulation with min-max injection
- Full-duplex SPI data exchange with the FPGA at 20 kHz
- Multiple compile-time run modes for calibration and testing
- 280 MHz clock configuration (HSE bypass + PLL)
- DMA-driven SPI slave with EXTI interrupt

**Logic written & validated (portable, desktop-tested, not yet integrated):**
- Position/velocity servo loop with feed-forward
- Jerk-limited S-curve trajectory planning + multi-axis interpolator
- Forward kinematics (general, any axis count) + 3-axis closed-form inverse
- Geometric Jacobian and manipulability (singularity) measure
- Software FPU sin/cos provider (swappable backend)

**What is planned:**
- Servo loop compile-time extensions (damping, friction, …)
- Wrist / orientation inverse kinematics (4/5/6-axis, skeleton in place)
- Cascade integration (trajectory → kinematics → servo → FOC)
- State machine and operating modes
- Technologist module communication protocol
- Safety guardian (last barrier before hardware)
- Hardware CORDIC acceleration for sin/cos (needs a CORDIC-equipped MCU)


## Directory Structure

```
main_cpu/
│
├── 1_logic/                            # Pure algorithms — PORTABLE
│   ├── foc.c                           # FOC: Clarke, Park, PI, inv. Park, SVPWM
│   ├── foc.h                           # FOC interface and output structures
│   └── pid.h                           # PI controller (static inline)
│   │
│   ├── 0_foc/                          # (planned) FOC refactored into C++ class
│   │   ├── foc.hpp
│   │   ├── clarke_park.hpp
│   │   └── svpwm.hpp
│   │
│   ├── 1_servo/                        # Position/velocity servo — IMPLEMENTED
│   │   ├── servo.c / .h                # Cascade P-pos + PI-vel, with feed-forward
│   │   └── extensions/                 # (planned) Servo loop extensions (∞)
│   │       ├── damping/
│   │       │   └── damping.hpp
│   │       ├── friction/
│   │       │   └── friction.hpp
│   │       └── .../
│   │
│   ├── 2_kinematics/                   # Forward/inverse kinematics — IMPLEMENTED (logic only)
│   │   ├── kinematics.c / .h           # Façade + core: general FK, IK dispatch, Jacobian
│   │   ├── seg_position3.c / .h        # Closed-form 3-axis positioner (base + shoulder + elbow)
│   │   └── seg_wrist3.c / .h           # (skeleton) 1/2/3-DOF wrist for 4/5/6-axis arms
│   │
│   ├── 3_trajectory/                   # Path planning — IMPLEMENTED (logic only)
│   │   ├── s_curve.c / .h              # Jerk-limited 7-segment S-curve profile
│   │   └── interpolator.c / .h         # Multi-axis sync + waypoint sequencing
│   │
│   ├── 4_runtime/                      # (planned) System orchestration
│   │   ├── state_machine.cpp
│   │   ├── homing.cpp
│   │   ├── protocol/                   # Technologist module communication
│   │   │   ├── packet.cpp
│   │   │   ├── commands.cpp
│   │   │   └── feedback.cpp
│   │   └── modes/                      # Special operating modes (∞)
│   │       └── resonance_scan/
│   │
│   └── safety/                         # (planned) Safety — ALWAYS ON
│       ├── guardian.cpp
│       └── watchdog.cpp
│
├── 2_utility/                                # (planned) Reusable building blocks
│   ├── math/
│   │   ├── matrix/
│   │   ├── quaternion/
│   │   └── spline/
│   ├── control/
│   │   ├── pid/
│   │   └── filters/
│   ├── signal/
│   │   └── dsp/
│   └── utils/
│       ├── ring_buffer/
│       └── crc/
│
├── 3_abstract/                         # Hardware contract — STABLE
│   └── types.h                         # Shared structures (SPI packets, phase_t)
│   └── hw_contract.h                   # (planned) Documents required HW functions
│
├── 4_driver/stm32h7a3zit6_Q/          # Chip-specific — REPLACEABLE
│   ├── config.h                        # Run mode, motor params, ADC calibration
│   ├── hw_impl.h                       # Inline HW functions (the bridge)
│   ├── hw_binding.c                    # Peripheral init orchestration
│   ├── isr.c                           # EXTI4 handler (FOC pipeline runs here)
│   ├── spi.c                           # SPI3 slave + DMA configuration
│   ├── spi.h
│   ├── rcc.c                           # Clock tree (280 MHz from HSE + PLL)
│   ├── rcc.h
│   ├── gpio.c                          # Diagnostic LED pins
│   ├── gpio.h
│   ├── tim.c                           # TIM6 microsecond timer
│   ├── tim.h
│   └── startup/
│       ├── STM32H7A3ZITXQ_FLASH.ld    # Linker script
│       └── startup_stm32h7a3xx.s       # Vector table + reset handler
│
├── tests/                              # Desktop unit tests (NOT built into firmware)
│   └── test_trajectory.c               # Host-compiled tests for 3_trajectory/
├── main.cpp                            # Entry point + configuration
├── CMakeLists.txt                      # Build system (ARM cross-compilation)
└── Makefile                            # Convenience wrapper (config/build/flash)
```

Items marked **(planned)** do not exist yet. They represent the target
architecture that will be built incrementally.

---

## Layer Rules

### logic/ — Pure Algorithms

**Language:** C today, C++ when servo layer is added

**Rules:**
- NEVER includes chip-specific headers directly
- NEVER accesses hardware registers
- ONLY calls functions from `hw_impl.h` and `utility/`
- CAN be compiled and tested on a desktop PC

**Numbering convention:** prefixes `0_`, `1_`, `2_`, `3_`, `4_` indicate the
cascade level. Lower number = faster loop = closer to hardware.

```
0_foc        20 kHz    current control          ← IMPLEMENTED (as foc.c)
1_servo       1 kHz    position/velocity        ← logic done, not integrated
2_kinematics  100 Hz   joint-to-cartesian       ← positioner done, wrist skeleton
3_trajectory  100 Hz   path planning            ← logic done, not integrated
4_runtime     main()   state machine, commands  ← planned
```

### utility/ — Reusable Building Blocks (planned)

**Language:** C or C++

**Rules:**
- Pure math and utilities — no hardware, no state machines
- Each block is self-contained with its own `.h` and `.c`/`.cpp`
- ONLY placed in `utility/` if used by **more than one** module in `logic/`
- If used by only one module — keep it inside that module

**Grouping:**
- `math/` — linear algebra, quaternions, splines
- `control/` — PID, filters, observers
- `signal/` — DSP, FFT, spectral analysis
- `utils/` — ring buffers, CRC, data structures

### abstract/ — Hardware Contract

**Language:** C (with `extern "C"` guards)

**Rules:**
- Contains ONLY header files
- `types.h` defines shared data structures used across all layers
- `hw_contract.h` (planned) documents the functions that `driver/` must implement
- This layer **almost never changes** — it is the stable interface

**Currently implemented:** `types.h` with SPI packet structures (`spi_rx_packet_t`,
`spi_tx_packet_t`, `axis_rx_t`, `axis_tx_t`), FOC types (`phase_t`, `rotor_t`),
and compile-time size assertions.

**`hw_contract.h` will document** the functions that every `driver/xxx/hw_impl.h`
must provide:

```
// hw_contract.h — Every driver/ MUST provide hw_impl.h containing:
//
//   static inline phase_t  hw_read_currents(void);
//   static inline rotor_t  hw_read_rotor(void);
//   static inline void     hw_write_pwm(phase_t duty);
//   static inline void     hw_sincos(float angle, float *s, float *c);
//   static inline void     hw_enable_gate(void);
//   static inline void     hw_disable_gate(void);
//   static inline uint32_t hw_micros(void);
```

### driver/ — Chip-Specific Implementation

**Language:** C + ASM

**Rules:**
- ONLY this layer knows about registers, DMA channels, pin numbers
- Implements all functions listed in `hw_contract.h`
- `hw_impl.h` uses `static inline` — zero overhead, compiler inlines everything
- `vendor/CMSIS/` contains third-party headers — **do not edit**
- When switching MCU, **only this layer is rewritten**

**Currently implemented:**
- `rcc.c` — full clock tree (280 MHz: HSE bypass → PLL → SYSCLK)
- `gpio.c` — diagnostic LED pins (PB0, PE1, PB14)
- `tim.c` — TIM6 as 1 µs free-running timer
- `spi.c` — SPI3 slave with DMA + EXTI4 interrupt on CS line
- `isr.c` — EXTI4 handler running the entire FOC pipeline
- `hw_binding.c` — calls all peripheral inits in correct order
- `hw_impl.h` — SPI buffer accessors (full hw functions are stubs for now)
- `config.h` — motor parameters, run mode selection, calibration constants

---

## How It Works Today

### The Interrupt-Driven FOC Loop

The entire control loop is driven by a single interrupt — EXTI4 on PA4,
triggered by the FPGA's SPI chip-select signal at 20 kHz:

```
FPGA asserts CS (falling edge)
    │
    ▼
EXTI4 ISR: configure DMA, enable SPI3
    │
    ... 256-bit SPI transfer (hardware, no CPU) ...
    │
FPGA releases CS (rising edge)
    │
    ▼
EXTI4 ISR: disable SPI3, then:
    1. Read RX buffer (encoder positions + ADC currents from FPGA)
    2. Run FOC: Clarke → Park → PI → inverse Park → SVPWM
    3. Fill TX buffer (PWM duty commands for FPGA)
    4. Reset SPI3 peripheral for next frame
    5. Return from ISR
    │
    ▼
CPU sleeps (__WFI) until next CS edge
```

No timer interrupts. No polling loops. The FPGA's 20 kHz tick is the sole
timing reference for the MCU.

### SPI Data Exchange

The MCU communicates with the FPGA via a 256-bit (32-byte) full-duplex
SPI frame every 50 µs. Packet structures are defined in `types.h`:

**FPGA → MCU (sensor data):**
```
typedef struct __attribute__((packed)) {
    uint32_t  sync;                     // 0xAA55AA55 — frame validation
    axis_rx_t axis[AXIS_COUNT];         // Per-axis: 2×16-bit ADC + 32-bit encoder
    uint32_t  sequence;                 // FPGA frame counter
} spi_rx_packet_t;
```

**MCU → FPGA (commands):**
```
typedef struct __attribute__((packed)) {
    uint32_t  magic;                    // 0xDEADBEEF — link validation
    axis_tx_t axis[AXIS_COUNT];         // Per-axis: 3×16-bit PWM + 16-bit flags
    uint32_t  status;                   // MCU processed-frame counter
} spi_tx_packet_t;
```

Both packets are exactly 32 bytes. Compile-time `_Static_assert` checks
guarantee size consistency.

### Run Modes

Operating behavior is selected at compile time via `RUN_MODE` in `config.h`.
This avoids runtime branching overhead and allows the compiler to eliminate
dead code entirely.

| Mode | Name              | Description                                    |
|------|-------------------|------------------------------------------------|
| 0    | Open-loop rotation | Rotating voltage vector, no feedback. ADC test |
| 1    | Encoder test       | Motor disabled, encoder readback only          |
| 2    | Alignment          | Static d-axis vector, captures encoder offset  |
| 3    | Align → FOC        | 2s alignment, then closed-loop current control |
| 4    | Full FOC           | All axes, production mode (needs outer loop)   |

### FOC Pipeline (foc.c)

Executes every 50 µs inside the EXTI4 ISR:

```
Phase currents (ia, ib from FPGA's ADC data)
        │
        ▼
  Clarke Transform
  ia, ib → iα, iβ
        │
        ▼
  Park Transform (using electrical angle from encoder)
  iα, iβ → id, iq
        │
        ▼
  PI Current Regulators (pid.h, static inline)
  id_error → vd    (flux, always targets 0)
  iq_error → vq    (torque, set by outer loop or test constant)
        │
        ▼
  Inverse Park Transform
  vd, vq → vα, vβ
        │
        ▼
  SVPWM Modulator (min-max injection, +15% bus utilization)
  vα, vβ → pwm_a, pwm_b, pwm_c  (0..PWM_PERIOD ticks)
        │
        ▼
  Packed into SPI TX buffer → sent to FPGA on next frame
```

### The Bridge: hw_impl.h

This file connects portable logic to real hardware. Currently it provides
SPI buffer accessors. As the architecture matures, it will contain direct
register access for ADC, PWM, and CORDIC:

```
// Current state: SPI-mediated access to FPGA buffers
static inline volatile spi_rx_packet_t* hw_spi_rx_buf(void) {
    return spi_get_rx();
}

// Future state: direct hardware access (zero overhead)
static inline phase_t hw_read_currents(void) {
    return (phase_t){
        .a = (float)adc_buf[0] * CURRENT_SCALE,
        .b = (float)adc_buf[1] * CURRENT_SCALE,
        .c = (float)adc_buf[2] * CURRENT_SCALE,
    };
}
```

Logic calls `hw_read_currents()`. At compile time, the compiler replaces this
with direct register access. **Zero function call overhead.**

### main.cpp — Entry Point

```
int main(void) {
    SCB->VTOR = 0x08000000;    // Vector table in Flash
    hw_binding_init();          // RCC → GPIO → TIM6 → SPI3+DMA+EXTI
    foc_init();                 // Zero PI integrators, set gains
    __enable_irq();             // EXTI4 starts firing at 20 kHz

    while (1) {
        __WFI();                // Sleep until next interrupt
    }
}
```

**main.cpp is the configuration file.** When the servo layer is added, it
will also define the extension pipeline:

```
// Future main.cpp
auto servo_chain = ServoChain{
    Damping{.freq = 45.0f, .gain = 0.3f},
    FrictionComp{.coulomb = 0.1f},
};
```

---

## Configuration

All tunable parameters are in `4_driver/stm32h7a3zit6_Q/config.h`:

```
#define RUN_MODE         3          // Operating mode (0–4)
#define AXIS_COUNT       3          // Number of motor axes
#define PWM_PERIOD       1350       // PWM counter ceiling (matches FPGA)
#define POLE_PAIRS       2          // Motor pole pairs
#define ENCODER_CPR      2048       // Encoder counts per revolution (X4)
#define CURRENT_SCALE    0.5f       // ADC counts → amperes
#define CURRENT_OFFSET   2048       // ADC zero-current midpoint (12-bit mid-scale)
#define TEST_IQ_REF      0.12f      // Test torque reference (amps)
#define ALIGN_TIME       3000       // Alignment duration (packets ≈ 150 ms)
#define ALIGN_AMPLITUDE  0.16f      // Alignment voltage (normalized 0..1)
```

---

## Clock Tree

```
HSE (8 MHz external bypass oscillator)
    │
    ▼
PLL1: 8 / 1 × 70 = 560 MHz VCO
    │
    ├── / 2 → SYSCLK = 280 MHz  (CPU core)
    ├── / 1 → AHB    = 280 MHz  (DMA, GPIO)
    ├── / 2 → APB1   = 140 MHz  (SPI3, TIM6)
    └── / 2 → APB2   = 140 MHz

Power: SMPS mode, VOS0 voltage scaling (required for > 225 MHz)
Flash: 6 wait states (required for 280 MHz)
```

---

## Safety Architecture (planned)

Guardian will sit between extensions and hardware output. It catches any
invalid value before it reaches the motor:

```
Extensions (may have bugs)
        │
        ▼
╔═══════════════════╗
║  GUARDIAN          ║  NaN? → STOP
║                   ║  Over limit? → STOP
║                   ║  Watchdog? → STOP
╚════════╤══════════╝
         ▼
    FOC output
         ▼
    HW comparator (last resort, hardware-level)
```

Guardian will be **never disabled.** There will be no `#if FEATURE_SAFETY`.

Three independent safety layers (planned):
1. `safety/guardian` — software checks (NaN, limits, watchdog)
2. FOC overcurrent — software current limit inside PI regulators
3. Hardware comparator — analog, works even if CPU hangs

---

## How to Extend

### Adding a Servo Loop Extension (future)

**You need to know:**
1. Create a struct with a `step()` method
2. Add one line in `main.cpp`

**You do NOT need to know:**
- How FOC works
- What MCU is used
- How interrupts are configured
- How the pipeline works internally

**Step 1:** Create your extension

```
logic/1_servo/extensions/my_feature/
└── my_feature.hpp
```

```
struct MyFeature {
    float my_param;

    float step(float torque, Axis& axis) {
        return torque + my_param * axis.velocity;
    }
};
```

**Step 2:** Add to main.cpp

```
auto servo_chain = ServoChain{
    Damping{.freq = 45.0f, .gain = 0.3f},
    MyFeature{.my_param = 0.5f},             // ← this line
};
```

**Step 3:** Build. Done.

The compiler inlines your `step()` into the servo loop. Zero overhead.
No function pointers. No vtables. As fast as if you wrote it by hand.

### Adding a New Run Mode (today)

1. Add a new `#elif RUN_MODE == N` block in `isr.c`
2. Define any new constants in `config.h`
3. Set `RUN_MODE` to your new value in `config.h`
4. Build and flash

### Adding a New Hardware Sensor

**Step 1:** Add inline reader to `driver/hw_impl.h`

```
static inline float hw_read_new_sensor(void) {
    return (float)SOME_REGISTER * SCALE;
}
```

**Step 2:** Document in `abstract/hw_contract.h`

```
//   static inline float hw_read_new_sensor(void);
```

**Step 3:** Use from `logic/` — done

```
float val = hw_read_new_sensor();  // zero overhead, inlined
```

### Adding a New Library Block (future)

```
2_utility/control/my_filter/
├── my_filter.h
└── my_filter.c
```

**Rule:** only move to `2_utility/` if used by more than one module in `logic/`.
Otherwise keep it inside the module that uses it.

---

## How to Port to a New MCU

1. Create new `4_driver/new_mcu/` directory
2. Implement every function listed in `abstract/hw_contract.h`
3. Write startup, ISR, and peripheral init for the new chip
4. Update `CMakeLists.txt` to point to the new driver directory
5. **Do not touch `1_logic/` or `3_abstract/`**

```
What changes:           4_driver/   (rewrite)
                        main.cpp    (adjust init if needed)
                        CMakeLists  (point to new driver)

What stays identical:   1_logic/    (all algorithms)
                        2_utility/        (all building blocks)
                        3_abstract/ (the contract)
```

---

## Build & Flash

```
# Full cycle: configure + build + flash
make all

# Step by step:
make config              # CMake configure (Release by default)
make config TYPE=Debug   # CMake configure (Debug, -O0 -g3)
make build               # Compile
make flash               # Program via ST-Link + OpenOCD

# Clean
make clean
```

**Requirements:**
- `arm-none-eabi-gcc` (13.x or later)
- `cmake` (3.20+)
- `openocd` (0.12+)
- ST-Link V2/V3 debug probe

---

## Implementation Status

| Feature                  | Status       | Location              |
|--------------------------|--------------|-----------------------|
| FOC current control      | ✅ Working   | `1_logic/foc.c`       |
| SVPWM modulation         | ✅ Working   | `1_logic/foc.c`       |
| PI current regulators    | ✅ Working   | `1_logic/pid.h`       |
| SPI slave (DMA + EXTI)   | ✅ Working   | `4_driver/spi.c`      |
| Clock tree (280 MHz)     | ✅ Working   | `4_driver/rcc.c`      |
| Run modes 0–4            | ✅ Working   | `4_driver/isr.c`      |
| Alignment calibration    | ✅ Working   | `4_driver/isr.c`      |
| SPI packet structures    | ✅ Working   | `3_abstract/types.h`  |
| hw_impl.h (SPI access)   | ✅ Working   | `4_driver/hw_impl.h`  |
| hw_impl.h (direct HW)    | 🔲 Stub     | `4_driver/hw_impl.h`   |
| hw_contract.h            | ✅ Working   | `3_abstract/`          |
| Position servo loop      | ✅ Working   | `1_servo/servo.c`      |
| Velocity control         | ✅ Working   | `1_servo/servo.c`      |
| Servo→FOC integration    | 🔲 Planned  | (caller wires it)      |
| Servo extensions         | 🔲 Planned  | `1_servo/extensions/`  |
| hw_sincos (software FPU) | ✅ Working   | `4_driver/.../sincos/` |
| Safety guardian          | 🔲 Planned  | `safety/`              |
| Forward kinematics (DH)  | ✅ Working   | `2_kinematics/kinematics.c`   |
| Inverse kin. (3-axis)    | ✅ Working   | `2_kinematics/seg_position3.c`|
| Jacobian + manipulability| ✅ Working   | `2_kinematics/kinematics.c`   |
| Wrist / orientation IK   | 🔲 Skeleton | `2_kinematics/seg_wrist3.c`   |
| Kinematics (servo hookup)| 🔲 Planned  | (caller wires it)      |
| Jerk-limited S-curve     | ✅ Working   | `3_trajectory/s_curve.c`     |
| Multi-axis interpolator  | ✅ Working   | `3_trajectory/interpolator.c`|
| Trajectory (servo hookup)| 🔲 Planned  | `1_servo/` (consumer)  |
| State machine            | 🔲 Planned  | `4_runtime/`           |
| Technologist protocol    | 🔲 Planned  | `4_runtime/protocol/`  |
| Hardware CORDIC          | 🔲 Planned  | `4_driver/`            |
| Library blocks           | 🔲 Planned  | `2_utility/`           |
| Desktop unit testing     | ✅ Working   | `tests/`               |

---

## Language Policy

| Layer        | Language    | Why                                          |
|--------------|-------------|----------------------------------------------|
| `1_logic/`   | C (→ C++)   | Portable today; C++ when servo chain is added |
| `2_utility/` | C or C++    | Pure math, maximum compatibility              |
| `3_abstract/`| C           | Must be visible from both C and C++           |
| `4_driver/`  | C + ASM     | Direct register access, CMSIS compatibility   |
| `main.cpp`   | C++         | Future: C++ servo chain configuration         |

C++ features planned: `templates`, `constexpr`, `inline namespaces`,
`designated initializers`. No dynamic allocation. No exceptions. No RTTI.

---

## Architecture Diagram

```
main.cpp          ──────────────────────────────────┐
  │ init + config                                   │
  ▼                                                 │
logic/4_runtime/  ◄── protocol/ (technologist)      │  ← planned
  │ commands                                        │
  ▼                                                 │
logic/3_trajectory/                                 │  ← logic done
  │ waypoints                                       │
  ▼                                                 │
logic/2_kinematics/                                 │  ← positioner done
  │ joint angles                                    │
  ▼                                                 │
logic/1_servo/    ◄── extensions/ (compile-time)    │  ← logic done
  │ torque         │                                │
  │                ▼                                │
  │            safety/guardian  ← ALWAYS ON          │  ← planned
  │                │                                │
  ▼                ▼                                │
logic/0_foc/  (foc.c + pid.h)                       │  ← IMPLEMENTED
  │ PWM duty                                        │
  ▼                                                 │
abstract/types.h  ◄──────── driver/hw_impl.h ───────┘  ← IMPLEMENTED
  │                             │
  ▼                             ▼
EXTI4 ISR (isr.c)          SPI3 + DMA (spi.c)         ← IMPLEMENTED
  │                             │
  ▼                             ▼
SPI packets ◄──────────────► FPGA (external)
```

---

## FAQ

**Q: Where do I put a new algorithm?**
A: If it modifies the servo torque → `logic/1_servo/extensions/`.
If it is a new operating mode → `logic/4_runtime/modes/`.
If it is reusable math → `2_utility/`.
If it is a new run mode today → `#elif` block in `isr.c`.

**Q: Can I write my extension in pure C?**
A: Yes. Write a C function and call it from your C++ `step()` method.
The struct wrapper is minimal — one method, one line.

**Q: What if I need data from a sensor that hw_impl.h doesn't expose?**
A: Add a `static inline` function to `driver/hw_impl.h`, document it in
`abstract/hw_contract.h`. Then call it from `logic/`.

**Q: How do I disable a feature?**
A: Remove its line from `servo_chain` in `main.cpp`. The compiler will
eliminate all dead code. Zero bytes in the binary.

**Q: Why is FOC in a .c file instead of a C++ class?**
A: The current implementation prioritizes getting the motor spinning.
Once the servo layer is added, FOC will be refactored into `0_foc/foc.hpp`
as a C++ class to match the extension architecture.

**Q: Why does the MCU sleep in main() instead of running a loop?**
A: All real-time work happens inside EXTI4_IRQHandler, triggered by the
FPGA at exactly 20 kHz. Between interrupts there is nothing to do — `__WFI`
saves power and guarantees deterministic interrupt entry latency.

**Q: Why not use virtual functions / function pointers for extensions?**
A: The servo loop will run at 1 kHz. Each indirect call costs 3-5 clock
cycles. With many extensions that adds up. C++ templates resolve everything
at compile time — zero overhead, as fast as hand-written code.

**Q: What guarantees safety today?**
A: Currently only the PI output limiter (±1.0 normalized) and PWM clamping
(0..PWM_PERIOD) in SVPWM. The full three-layer safety system (guardian +
software overcurrent + hardware comparator) is planned.
```