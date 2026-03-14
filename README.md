About The Project
This repository contains the firmware and hardware configurations for a custom-built 2-Axis Drawing Robot. The core system is powered by an STM32 microcontroller, programmed entirely in bare-metal C/C++.

Key Technical Highlights:

Firmware Architecture: Optimized stepper motor control using STM32 Hardware Timers and Interrupts to ensure precise, jitter-free movements.

Path Planning: Implemented Bresenham's line algorithm combined with a custom G-code parser to translate standard vector commands into accurate X-Y coordinate steps.

Hardware Integration: Calibrated and integrated A4988 stepper drivers, rigorously validating torque control logic to maintain system stability under mechanical load.

Tech Stack:
STM32 | C/C++ | G-code Processing | UART Debugging | Hardware Timers/Interrupts
