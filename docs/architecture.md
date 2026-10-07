## Architecture.md

# Architecture of the RIGEL Robotic Manipulator

## Table of Contents

- [1. Overview](#1-overview)
- [2. Core Concept](#2-core-concept)
- [3. System Architecture](#3-system-architecture)
  - [3.1 System Block Diagram](#31-system-block-diagram)
  - [3.2 Data Flow: Control Loop](#32-data-flow-control-loop)
  - [3.3 Power Distribution](#33-power-distribution)
- [4. Electronics Architecture](#4-electronics-architecture)
  - [4.1 Motherboard](#41-motherboard)
  - [4.2 MCU: STM32H7A3](#42-mcu-stm32h7a3)
  - [4.3 FPGA: Lattice ECP5 (LFE5U-45F)](#43-fpga-lattice-ecp5-lfe5u-45f)
  - [4.4 MCU ↔ FPGA Communication](#44-mcu--fpga-communication)
  - [4.5 Power Blade (Axis Channel)](#45-power-blade-axis-channel)
  - [4.6 Technologist Module](#46-technologist-module)
  - [4.7 Input Power Board](#47-input-power-board)
- [5. Firmware Architecture](#5-firmware-architecture)
  - [5.1 FPGA Modules](#51-fpga-modules)
  - [5.2 MCU Firmware](#52-mcu-firmware)
  - [5.3 Control Loop Timing](#53-control-loop-timing)
- [6. Mechanical Architecture](#6-mechanical-architecture)
  - [6.1 Kinematic Configuration](#61-kinematic-configuration)
  - [6.2 Slew Drive (Rotary Support)](#62-slew-drive-rotary-support)
  - [6.3 Cabin](#63-cabin)
  - [6.4 Shoulder Module](#64-shoulder-module)
  - [6.5 Backpack (Motor + Reducer Mount)](#65-backpack-motor--reducer-mount)
  - [6.6 Balanced Cycloidal Reducer](#66-balanced-cycloidal-reducer)
  - [6.7 Base](#67-base)
- [7. The Technologist Module: Design Philosophy](#7-the-technologist-module-design-philosophy)
- [8. Design Principles](#8-design-principles)
- [9. Target Specifications](#9-target-specifications)
- [10. Current Status and Roadmap](#10-current-status-and-roadmap)

---

## 1. Overview

The RIGEL Robotic Manipulator is a fast, precise,
and affordable robotic arm designed for simple assembly, easy operation,
and tool-agnostic deployment. It is built from four electronic subsystems:
an Input Power Board, a Motherboard, hot-swappable Power Blades, and a
hot-swappable Technologist Module.

The robot is designed so that any person with a 3D printer and basic tools
can build an instrument capable of precise spatial positioning — precise
enough to mill aluminum and manufacture parts for a stronger, metal version
of itself.



## 2. Core Concept

The central idea behind RIGEL is the separation of concerns:

- **The Robot** is a precision positioning tool. It moves a point in space
  from A to B. It is a black box with a simple API.
- **The Technologist Module** defines *what to do*. It controls the tool,
  sends high-level commands, and can be swapped in seconds.
- **The Power Blades** drive the motors. They are hot-swappable cards that
  plug into the motherboard via a standard PCIe physical connector.

This separation means that switching from milling to welding requires only
swapping the Technologist Module and the tool — not reprogramming the robot.



## 3. System Architecture

### 3.1 System Block Diagram

```
[PSU 8-60V] ──→ [Input Power Board*] ──→ [MOTHERBOARD]
                                              │
                 ┌────────────┬───────────────┼───────────────┬────────────┐
                 │            │               │               │            │
            [Blade 1]   [Blade 2]   ...  [Blade 6]   [Technologist Module]
                 │            │               │               │
           [Motor+Enc]  [Motor+Enc]     [Motor+Enc]      [Tool/End Effector]


Inside the Motherboard:
┌─────────────────────────────────────────────────────────────────────┐
│                                                                     │
│   [FPGA: Lattice ECP5] ←── data bus ──→ [MCU: STM32H7A3]          │
│        │                                      │                     │
│        ├─ PWM signals      → Blade PCIe slots  ├─ FOC algorithm     │
│        ├─ Encoder data     ← Blade PCIe slots  ├─ Inverse kinematics│
│        ├─ SPI (ADC data)   ← Blade PCIe slots  ├─ API ↔ Tech Module│
│        ├─ Hardware dead-time protection         ├─ State machine     │
│        └─ Hardware clock sync (for clustering)  └─ Trajectory plan  │
│                                                                     │
│   [PCIe x8 Slots × 6]              [PCIe Slot × 1: Tech Module]   │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘

* Input Power Board is not implemented in the prototype.
  The prototype is powered directly from a 24V PSU.
```

### 3.2 Data Flow: Control Loop

Every control cycle (target: 20 µs / 50 kHz):

```
Step 1: FPGA triggers ADC conversion on the Blade
            │
Step 2: STM32G431 (on Blade) samples 3 phase currents
            │
Step 3: ADC data is sent via SPI to the FPGA          ← ~400 ns
            │
Step 4: FPGA decodes quadrature encoders (parallel)   ← 1-2 clock cycles
            │
Step 5: FPGA writes currents + positions directly
        into MCU registers (via data bus)              ← no CPU overhead
            │
Step 6: MCU (STM32H7) executes:
        - Clarke/Park transforms
        - PID current regulators (FOC)
        - Inverse kinematics
        - Trajectory interpolation
            │
Step 7: MCU writes PWM duty cycles directly
        into FPGA registers (via data bus)             ← no CPU overhead
            │
Step 8: FPGA generates 6 PWM signals with
        hardware dead-time insertion
            │
Step 9: PWM signals reach gate drivers on the Blade
            │
Step 10: Motor responds. Cycle repeats.
```

Key architectural advantage: Steps 3, 4, 5, and 7 happen with **zero CPU
involvement**. The FPGA handles all signal acquisition, decoding, and
routing in parallel, while the MCU focuses purely on math. Adding more
sensors or encoders does not increase MCU load or introduce jitter.

> **Current prototype status:** The full control chain has been validated
> at 20 kHz using development boards (Tang Nano 9K as FPGA, STM32H7 Nucleo
> as MCU) connected via SPI. The FPGA acts as SPI master and triggers the
> MCU via EXTI interrupt on each frame. The target architecture (direct
> register bus, 50 kHz) will be implemented on the motherboard.

### 3.3 Power Distribution

```
[External PSU: 8-60V DC]
        │
[Input Power Board*] ── filtering, protection, regulation
        │
[Motherboard Power Rail]
        ├──→ Blade slots (Vbus passthrough to motor drivers)
        ├──→ 12V DC/DC → FPGA, gate driver logic supply
        ├──→ 5V  DC/DC → MCU, digital logic
        ├──→ 3.3V LDO  → MCU core, FPGA I/O banks
        └──→ Technologist Module slot (5V or 3.3V selectable)

Each Blade has its own on-board power regulation:
[Vbus from Motherboard]
        ├──→ 12V DC/DC (TPS54360B) → gate driver bootstrap
        ├──→ 5V  LDO  (LM7805)    → STM32G431, RS-485
        └──→ 3.3V LDO (TLV1117)   → logic, analog reference

* Not implemented in prototype. Robot is powered directly at 24V.
```

---

## 4. Electronics Architecture

### 4.1 Motherboard

The Motherboard is the central hub. It hosts:

- **MCU:** STM32H7A3 — the brain of the robot.
- **FPGA:** Lattice ECP5 (LFE5U-45F) — the signal engine.
- **PCIe x8 slots** (×6) for Power Blades.
- **PCIe x8 slot** (×1) for the Technologist Module.
- **Power input terminals** from the Input Power Board (or directly from PSU).
- **Clock synchronization output** on the Technologist Module connector.

> **Important:** The PCIe x8 connector (98-pin, part: 10018784-10202TLF) is
> used **only as a physical/mechanical connector**. The PCIe protocol is
> **not** used. The pinout is custom-defined for this project.

### 4.2 MCU: STM32H7A3

The MCU is the high-level computational center. Its responsibilities:

- **FOC (Field-Oriented Control)** — Clarke/Park transforms, PID regulators.
- **Inverse kinematics** — converting Cartesian coordinates to joint angles.
- **Technologist Module API** — receiving high-level commands (e.g., "move to
  point X,Y,Z") and decomposing them into per-axis motor control tasks.
- **Trajectory planning** — interpolation, acceleration profiles.
- **System state machine** — initialization, calibration, operation, error
  handling.

The MCU does **not** directly read encoders or generate PWM. All signal I/O
is handled by the FPGA and mapped into MCU memory via the data bus.

### 4.3 FPGA: Lattice ECP5 (LFE5U-45F)

The FPGA is the real-time signal engine. Its responsibilities:

- **PWM generation** — center-aligned PWM at the duty cycle specified by
  the MCU. All channels are generated in parallel.
- **Hardware dead-time insertion** — prevents shoot-through on half-bridges.
  This is a critical safety function implemented in hardware, not software.
- **Quadrature encoder decoding** — reads and decodes encoder signals for
  all axes simultaneously, in parallel, with glitch filtering.
- **ADC data reception** — receives SPI data from the STM32G431 ADC on each
  Blade and places it directly into MCU-accessible registers.
- **Hardware clock synchronization** — generates a synchronization clock
  signal for multi-robot clustering. This clock is routed to the
  Technologist Module connector.

The FPGA communicates with the MCU via a direct data bus that allows
register-level read/write access in both directions.

### 4.4 MCU ↔ FPGA Communication

The MCU and FPGA are connected via a parallel data bus that enables direct
register access:

- The **MCU writes** PWM duty cycles and frequencies directly into FPGA
  registers. No serialization, no protocol overhead.
- The **FPGA writes** encoder positions and ADC values directly into
  MCU-accessible memory. No interrupt handling, no polling.

This architecture eliminates the communication bottleneck that typically
limits multi-axis servo systems. Adding a new axis (a new Blade with
motor and encoders) does not increase the MCU's I/O workload — it only
adds more math to compute.

> The specific bus interface (FMC, FSMC, or custom parallel bus) will be
> finalized during motherboard development.

### 4.5 Power Blade (Axis Channel)

The Power Blade is a modular motor driver on a PCB with a PCIe x8 edge
connector. Each Blade drives one motor (one axis).

**Specifications:**

| Parameter                | Value                              |
|--------------------------|------------------------------------|
| Connector                | PCIe x8 (98-pin, physical only)    |
| Rated power              | 150–300 W (continuous)             |
| Peak power (by design)   | 1200 W                             |
| Switching frequency       | 50 kHz                             |
| MOSFETs                  | NTMFS3D5N08XT1G (80V, 3mΩ, 23nC)  |
| Gate drivers             | IRS21867S (4A peak, 600V)          |
| Current sensing          | INA240A1DR (×3, 20 V/V gain)       |
| Shunt resistors          | 5mΩ, 3W, 1%                       |
| On-board ADC             | STM32G431RBT6                      |
| Communication to FPGA    | RS-485 (SN65HVD75DR), SPI          |

**Why STM32G431 as ADC?**

A dedicated ADC IC would be cheaper per channel but would require more
external circuitry. The STM32G431 provides three 12-bit ADCs with hardware
synchronization, all in a single $1.5 package. It also runs at 170 MHz,
which allows it to handle the ADC-to-SPI pipeline with a latency of
approximately 400 ns and zero CPU intervention on the main MCU. In the
future, the G431 will also monitor Blade health (temperature, voltage,
fault conditions).

**Thermal note:** At the nominal operating point (24V, 4A), total
conduction losses across all 6 MOSFETs are under 0.3W. Even at the
maximum design current (10A), losses remain under 3W. The SuperSO8
package dissipates this without a heatsink. Active cooling is not required.

> Full component list and selection rationale: see `hardware/docs/components.md`

### 4.6 Technologist Module

The Technologist Module is the user-facing control interface. It plugs into
a dedicated PCIe slot on the Motherboard and communicates with the MCU via
a minimal interface:

```
Pin allocation (conceptual):
- Power:  VCC (5V or 3.3V)
- Ground: GND
- Data:   TX (commands from module → MCU)
- Data:   RX (feedback from MCU → module)
- Clock:  SYNC (hardware synchronization clock from FPGA)

```

From the Technologist Module's perspective, the entire robot is a **black box** with a simple API:

- Send: `MOVE_TO(x, y, z, speed)`
- Send: `SET_JOINT(axis, angle)`
- Receive: `POSITION(x, y, z)`
- Receive: `STATUS(state, error_code)`

The Technologist Module has **no access** to the internal control loop,
FOC parameters, or motor-level commands. It cannot modify the closed-loop
control algorithms.

The module is also directly connected to the end-effector (tool). It
controls the tool independently of the robot's positioning system.

**Key design intent:** Swapping the Technologist Module changes the robot's
task. A milling module commands precise paths and controls a spindle.
A welding module commands different paths and controls a welder. The robot
itself does not change. This is the "hot-swap" in HSPS.

> The physical communication protocol (UART, SPI, or custom) and the
> command format will be finalized during Technologist Module development.

### 4.7 Input Power Board

The Input Power Board sits between the external power supply and the
Motherboard. Its purpose is to protect and condition the incoming power:

- Input voltage filtering (EMI suppression)
- Overvoltage and overcurrent protection (disconnect on fault)
- Voltage regulation/stabilization for noisy or unstable sources
- Accepted input range: 8V to 60V DC

> **Not implemented in the prototype.** The prototype connects a regulated
> 24V PSU directly to the Motherboard. The Input Power Board will be
> developed for the production version after the core concept is validated.

---

## 5. Firmware Architecture

### 5.1 FPGA Modules

The FPGA design is written in SystemVerilog and consists of independent,
parallel modules:

| Module                | Function                                           |
|-----------------------|----------------------------------------------------|
| `system_storage`      | Central register file (CSR). Shared memory between  |
|                       | FPGA and MCU for all axis data.                    |
| `pwm_generator`       | Center-aligned PWM generation for FOC.             |
| `deadtime`            | Hardware dead-time insertion. Prevents shoot-through|
|                       | on half-bridge MOSFETs. Cannot be bypassed by SW.  |
| `quadrature_decoder`  | Decodes ABZ encoder signals with glitch filtering. |
| `spi_master`          | Configurable-width SPI master for MCU and ADC      |
|                       | communication.                                     |
| `top`                 | Top-level integration: 3-axis instantiation,       |
|                       | exchange FSM, tick timer.                          |

All modules run in parallel. Adding a new axis means instantiating another
set of modules — no impact on existing axes' timing.

### 5.2 MCU Firmware

> **Status:** The FOC algorithm is implemented and validated at 20 kHz on
> development boards (Tang Nano 9K + STM32H7 Nucleo). The target frequency
> of 50 kHz will be achieved on the final motherboard with FMC bus.

The MCU firmware runs on the STM32H7A3 and implements:

- **FOC (Field-Oriented Control):** Clarke/Park transforms, PI current
  regulators in the d-q reference frame. **Implemented and tested.**
- **SVPWM modulation:** Space-vector PWM with min-max injection for
  optimal DC bus utilization. **Implemented and tested.**
- **Position/velocity control loop:** Outer loop using encoder feedback.
  *Planned.*
- **Inverse kinematics solver:** Converts Cartesian commands from the
  Technologist Module into joint-space targets. *Planned.*
- **Technologist Module communication handler:** Parses incoming commands,
  queues them (FIFO), executes in order, sends back position feedback.
  *Planned.*
- **State machine:** Manages robot states (boot → calibration → idle →
  moving → error → emergency stop). *Planned.*

### 5.3 Control Loop Timing

Target control loop period: **20 µs (50 kHz)**.

```
Budget breakdown (target):

ADC sampling + SPI transfer:     ~0.5 µs
FPGA encoder decoding:           ~0.02 µs (1-2 FPGA clock cycles)
FPGA → MCU register write:       ~0.1 µs
MCU: FOC computation:            ~5-10 µs (estimated)
MCU → FPGA register write:       ~0.1 µs
FPGA PWM update:                 ~0.02 µs
────────────────────────────────────────────
Total estimated:                 ~6-11 µs
Margin:                          ~9-14 µs
```

This timing budget is achievable because signal acquisition (steps 1-3)
and signal generation (step 6) are handled entirely by the FPGA in
parallel with MCU computation. The MCU only performs math.

**Why this matters for precision:**

Two encoders per axis (one on the motor shaft, one on the output after
the reducer) are read simultaneously by the FPGA. At 50 kHz, the robot
corrects its position every 20 µs. This high update rate, combined with
dual-encoder feedback, compensates for reducer backlash and mechanical
imperfections in software — reducing the need for expensive, high-precision
mechanical components.

Scaling to 100 kHz (10 µs) is architecturally possible by using a faster
MCU or offloading parts of the FOC computation to the FPGA, with no
changes to the system architecture.

---

## 6. Mechanical Architecture

### 6.1 Kinematic Configuration

The robot is a serial-link articulated manipulator. The prototype
implements 3 axes (for concept validation), with the architecture designed
to scale to 6 axes.

The mechanical structure consists of modular, standardized elements
connected via a 20mm shaft interface. Each joint is driven by a BLDC motor
through a balanced cycloidal reducer and a belt final stage.

```
[Base] → [Slew Drive: Axis 1 (rotation)]
              → [Cabin]
                   → [Shoulder Module: Axis 2 (pitch)]
                        → [Shoulder Module: Axis 3 (pitch)]
                             → [End Effector / Tool]
```

### 6.2 Slew Drive (Rotary Support)

The slew drive provides Axis 1 rotation. It consists of three 3D-printed
parts:

- **Stator:** A circular base with bolt holes for mounting to the base.
  It has an outer wall and an inner wall forming a rectangular channel
  (raceway) in which bearings ride.
- **Rotor:** A ring that fits inside the stator bore with finger
  protrusions around its circumference. Bearings are pressed onto the
  fingers and ride in the stator channel.
- **Cover:** A ring that bolts onto the stator, closing the channel and
  constraining the bearings axially.

The assembly creates a rigid constraint that allows free rotation while
preventing tilting (nodding) of the robot. The stator raceway is expected
to wear over time with 3D-printed materials and will be reinforced in
future iterations.

### 6.3 Cabin

The cabin connects the slew drive (Axis 1) to the first shoulder joint
(Axis 2). Its design is inspired by industrial heavy equipment
(excavators), where the rotation axis is offset from the arm attachment
point:

- The arm attaches to one side via two bearing ears forming a U-shaped
  yoke.
- The opposite side houses the motor and reducer for Axis 2, acting as a
  counterweight.
- This layout eliminates the tuning-fork resonance effect, increases
  structural rigidity, and partially balances the arm when fully extended.

### 6.4 Shoulder Module

The shoulder module is a standardized structural link. It consists of:

- **Base end:** Two ears that clamp onto a 20mm shaft using hex nuts and
  bolts (6 bolt points per ear), providing a rigid connection.
- **Truss section:** A lightweight lattice/truss structure for
  stiffness-to-weight optimization. Walls are 5mm+ PETG with 100% infill.
  Four 8mm steel rods run inside the truss as internal reinforcement,
  creating a composite structure with rigidity comparable to aluminum
  tube of similar cross-section.
- **Head end:** A U-shaped yoke with bearing bores for the next joint's
  shaft, plus mounting points for the Backpack module.

Multiple shoulder modules can be chained using the same 20mm shaft
interface to extend the arm. The prototype uses a long module and a short
module.

### 6.5 Backpack (Motor + Reducer Mount)

The Backpack is a box-shaped module that mounts underneath the shoulder
module's head, attaching to the same interface used for the yoke. It
serves multiple purposes:

- Houses the motor and reducer for the next joint.
- Stiffens the yoke ears, eliminating tuning-fork resonance.
- Shifts the center of mass closer to the previous joint axis, reducing
  dynamic loads.
- Aligns the reducer output shaft and belt drive with the joint axis.

### 6.6 Balanced Cycloidal Reducer

The reducer is a custom-designed, 3D-printable balanced cycloidal drive.

**Specifications:**

| Parameter                    | Value                               |
|------------------------------|-------------------------------------|
| Reduction per stage          | 10:1                                |
| Stages in prototype          | 2 (= 100:1)                        |
| Maximum stages (by volume)   | 3 (= 1000:1)                       |
| Belt final stage             | 1:3                                 |
| Total reduction (prototype)  | 300:1                               |
| Form factor                  | Fits in the palm of a hand          |

The high reduction ratio is necessary because the design uses high-RPM,
low-torque BLDC motors. The architecture allows any motor to be used — if
high-torque motors are chosen, the reducer can be simplified or removed.

**Key discovery during development:**

For a rigidly connected, balanced (counter-rotating) dual-disc cycloidal
drive, the standard clearance formula does not apply. The correct formula
for the hole diameter in the cycloidal discs is:

```
Hole_Diameter = Pin_Diameter + (4 × Eccentricity)
```

This is **double** the clearance required for single-disc or independently
suspended designs. This finding was validated through physical testing.

> Detailed development history and the reasoning behind this formula can
> be found in the [Development Log](DEVLOG.md).

### 6.7 Base

The prototype base is a heavy concrete block. An earlier plan to 3D-print
a plastic base was abandoned because it would have required over 2 kg of
plastic while still being too light to provide adequate stability. The
concrete block is cheap, heavy, and meets all requirements.

---

## 7. The Technologist Module: Design Philosophy

The Technologist Module is the most architecturally significant part of
this project. It is what differentiates this robot from others at a
fundamental level.

**The problem it solves:**

Most robotic arms tightly couple the control software with the robot
hardware. Changing the robot's task means rewriting code, reconfiguring
the controller, and often physically modifying the system. This makes
robots expensive to deploy and inflexible in practice.

**The solution:**

The Technologist Module separates *what the robot does* from *how the
robot moves*. The robot's internal control system (FOC, kinematics,
trajectory planning) is a sealed black box. The Technologist Module
communicates with this box through a simple, high-level API.

**What this enables:**

- **Tool independence:** The module controls the tool directly. The robot
  only positions it. Swapping tools means swapping modules.
- **Protocol independence:** The module can communicate with the outside
  world via any interface — Ethernet, EtherCAT, USB, CAN, Wi-Fi, or
  simply execute a pre-programmed sequence from flash memory. The robot
  does not care.
- **Rapid deployment:** A new application requires designing only a small
  PCB with a microcontroller and a PCIe connector. The robot, its
  firmware, and its calibration remain untouched.
- **Clustering:** Multiple robots can be synchronized via the hardware
  clock signal exposed on the Technologist Module connector. This enables
  multi-robot cooperative tasks (e.g., holding a workpiece with one robot
  while another machines it).

**Physical form:**

A small PCB with a microcontroller, a PCIe edge connector, and
(optionally) an external communication interface. It plugs into the
dedicated slot on the Motherboard and is powered by it (5V or 3.3V).

---

## 8. Design Principles

1. **The robot is a black box.** It moves a point in space from A to B.
   Everything inside stays inside. Everything outside stays outside.

2. **Modularity over monolith.** Every subsystem — mechanical and
   electronic — must be independently testable, replaceable, and
   repairable. A Blade swap should take seconds. A shoulder module swap
   should take minutes.

3. **Precision through speed, not mechanics.** Industrial robots achieve
   precision through expensive, ultra-tight-tolerance mechanical
   components. This robot achieves precision through a fast control loop
   (50 kHz) with dual-encoder feedback, correcting mechanical
   imperfections 50,000 times per second.

4. **Infinite scalability (as a goal).** The architecture should allow
   adding more axes, more sensors, and more robots without fundamental
   redesign. The FPGA's parallel nature makes this feasible for I/O.
   The standardized mechanical interfaces make this feasible for
   structure.

5. **Power agnosticism.** The robot should accept any DC power source
   from 8V to 60V. If it can be deployed anywhere, it should be powered
   from anywhere.

6. **Accessibility.** The robot must be buildable by anyone with a 3D
   printer and common tools. The BOM cost should be comparable to a
   power tool, not an automobile.

7. **Reusability.** Like good code, no part of the robot should be
   single-purpose. The shoulder module is the same for every joint. The
   Blade is the same for every axis. The reducer is the same at every
   stage.

8. **Motor agnosticism.** The control system must work with any BLDC
   motor. The system should be able to characterize an unknown motor
   and configure itself accordingly.

9. **The tool defines the task, not the robot.** Swapping the
   Technologist Module and the end-effector changes the robot's purpose.
   Today it mills. Tomorrow it welds. The robot itself does not change.

10. **Mechanics is necessary but not primary.** We cannot build a robot
    from sticks, but we also must not rely solely on mechanical precision.
    A $50,000 harmonic drive on a poorly controlled system is still a
    poorly controlled system.

---

## 9. Target Specifications

| Parameter                     | Prototype (3-axis)    | Target (6-axis)      |
|-------------------------------|-----------------------|----------------------|
| Degrees of freedom            | 3                     | 6                    |
| Control loop frequency        | 50 kHz (20 µs)        | 50–100 kHz           |
| Input voltage                 | 24V DC                | 8–60V DC             |
| Current per axis (nominal)    | 4 A                   | configurable         |
| Current per axis (peak)       | 10 A                  | configurable         |
| Power per axis (nominal)      | ~100 W                | ~150–300 W           |
| Encoder resolution            | TBD                   | TBD                  |
| Encoders per axis             | 2 (motor + output)    | 2 (motor + output)   |
| Reducer ratio                 | 300:1 (100:1 + 1:3)   | up to 1000:1 + belt  |
| Motor type                    | BLDC (any)            | BLDC (any)           |
| Positioning accuracy (target) | < 0.1 mm (with cal.)  | < 0.05 mm (with cal.)|
| Payload                       | TBD                   | TBD                  |
| Reach                         | TBD                   | TBD                  |
| Robot mass (estimated)        | TBD                   | TBD                  |
| Electronics BOM (3-axis)      | ~$80–120 (estimated)  | TBD                  |
| Frame material (prototype)    | PETG (3D-printed)     | Aluminum (milled)    |
| Blade swap time               | Seconds               | Seconds              |
| Technologist Module swap time | Seconds               | Seconds              |

---

## 10. Current Status and Roadmap

> For detailed week-by-week progress, see the [Development Log](DEVLOG.md).

### Completed

- [x] Core mechanical design (3-axis CAD)
- [x] Balanced cycloidal reducer (designed, printed, assembled, tested)
- [x] Shoulder modules (printed and assembled)
- [x] Slew drive (printed and assembled)
- [x] Power Blade schematic design
- [x] Power Blade PCB layout and routing
- [x] Component simulations (DC/DC, current amplifier, half-bridge)
- [x] FPGA core (PWM generator, dead-time, encoder decoder, CSR, SPI master)
- [x] ADC-to-SPI pipeline on STM32G431 (~400 ns latency)
- [x] Gerber/BOM/CPL files generated
- [x] PCBs ordered and received
- [x] Power Blade power-on test (all voltages verified)
- [x] FOC algorithm implementation on STM32H7 (Clarke, Park, PI, SVPWM)
- [x] Full control chain validated: ADC → FPGA → MCU → FPGA → motor (20 kHz)

### In Progress

- [ ] Remaining mechanical assembly (shoulder fitting, belt tensioning)
- [ ] Motor spinning on Power Blade (transitioning from dev boards)
- [ ] Motherboard schematic design

### Future

- [ ] Motherboard PCB design
- [ ] Technologist Module prototype
- [ ] Input Power Board
- [ ] Position/velocity servo loop
- [ ] Inverse kinematics
- [ ] Multi-robot clustering
- [ ] Calibration system (reference flat surface)
- [ ] Aluminum version (self-manufactured)

---

*Project started: May 27, 2025*
*This document reflects the architecture as of the prototype development phase.*
*For component details, see [components.md](hardware/docs/components.md).*
*For development history, see [DEVLOG.md](DEVLOG.md).*
