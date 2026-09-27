# RIGEL — Robotic Interchangeable General-purpose Extensible Linkage

This repository is dedicated to the development of **RIGEL**, a high-performance, open-source modular robotic manipulator designed around an interchangeable and extensible architecture.

The goal of RIGEL is to create a versatile robotic platform where mechanical, electronic, and functional modules can be reconfigured, replaced, and extended without redesigning the entire system.

## Modular... what?

"Modular robotic platform" might sound like a fancy way of saying "a robot made from a bunch of parts." And, well... that's basically the idea.

Building and controlling a robotic arm usually requires deep programming skills and a complex understanding of the entire system, making most designs rigid and difficult to modify.

**RIGEL is my attempt to make that easier.**

The idea is to build the robot from interchangeable and extensible modules, so parts can be replaced, added, or reconfigured without redesigning the entire system.

And no, this is not a toy for grabbing cubes.

## What does "not a toy" mean?

A toy can grab something. A toy can rotate. This robot is designed to be a true industrial tool. It's designed to be precise and powerful enough to mill its own parts from aluminum, to weld, or to perform any task you can imagine.

The core idea is simple: This project provides a blueprint for anyone with common tools (like a 3D printer) and dedication to build an instrument capable of creating almost anything.

## How "good" is this cheap thing?

You might be thinking: "Okay, so you're making a cheap little arm and believe it can compete with a true, big, metal one? That's nonsense." And you are half right. This robot can't mill hardened steel or titanium. But does it need to?

What we all really need are strong, precise parts. We can't 3D-print metal, but this robot is designed to be precise enough to **mill aluminum**. This means you can use the first plastic robot to create the parts for its much stronger, metal successor. Sounds more realistic now, doesn't it?

But what about the "brains"? I've got you covered there too. This robot is being designed to run its control loop at **50 kHz**. This means it can react to a command in **20 microseconds** - the time from your signal to a change in the motor's current. And yes, it will work with almost any motor you want.

## Nice to meet you

I bet this sounds interesting. I just want to say hello, and I'm glad to see you here. This project started on May 27, 2025, and my goal is to build the first working prototype by next summer.

Maybe I'll make it, maybe I won't, but we'll find that out together. For now, you can find me here every Saturday. Let's get started!

## 05.07.26 Update "After a year"

Yeah, looking back, I wasn't sure what I was getting into at the start. I honestly thought I'd just tinker for a month and drop it, but here we are. Let's see what we've achieved!

I have finished the mechanics (I'd say they are 99% done). I also developed a 1kW motor driver—you can grab the Gerbers and order the boards for yourself(please note the non-commercial license below! sry for that). I've tested it and it works great, though I will personally be running it at 200W for this robot. I also wrote the FOC algorithm to control the motors.

I'll be continuing this journey, and I might change the name of the project soon. I really was here almost every Saturday (well, maybe not every single one, but I tried!).

A huge thank you to the two guys who starred my repo—it means a lot! I will keep going, we can do this! <Have a good day!> :>


## ⚖️ License

This project uses a **Dual-Licensing** model to protect both the software and the hardware designs.

*   **Software / Firmware:** All source code in this repository is licensed under the **GNU General Public License v3.0 (GPLv3)**. See the `LICENSE` file in the root directory for details.
*   **Hardware Designs:** All hardware files (Gerbers, Schematics, BOM, Pick and Place files, and 3D CAD models) located in the hardware/cad directories are licensed under the **Creative Commons Attribution-NonCommercial 4.0 International (CC BY-NC 4.0)** license. See the `LICENSE_HARDWARE.md` file for details.

**What this means for you:**
You are completely free to use, modify, and build the software and hardware for your own personal, educational, or hobby projects. However, **you may not use the hardware files for commercial purposes** (e.g., you cannot mass-produce the PCBs to sell them) without explicit permission.
