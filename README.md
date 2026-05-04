# FPGA Hardware Accelerator IP Core for Differential Drive Kinematics

[![Portfolio](https://img.shields.io/badge/Portfolio-omerhub.com-blue?style=flat-square)](https://omerhub.com) [![arXiv](https://img.shields.io/badge/arXiv-Paper_Link-red?style=flat-square)](https://arxiv.org) [![License: MIT](https://img.shields.io/badge/License-MIT-green.svg?style=flat-square)](https://opensource.org/licenses/MIT)

This repository provides an open-source, hardware-agnostic FPGA IP Core that offloads complex differential drive kinematics, trigonometric calculations (CORDIC), and continuous spatial odometry from the CPU to the hardware level.

By bypassing traditional software-based execution and communication delays, this module achieves deterministic calculation speeds of **740.74 nanoseconds (20 clock cycles at 27 MHz)** using highly precise Q16.16 fixed-point arithmetic with 64-bit overflow protections.

> **Research & Documentation:**
> 
> 
> This IP Core was developed as part of an ongoing research on deterministic robot locomotion. For a detailed academic breakdown of the hardware architecture, FSM design, and HIL testing methodology, please refer to our full paper on [arXiv](https://arxiv.org/abs/XXXX.XXXXX).
> 
> For future updates (Ackermann, Omniwheel implementations) and more projects, visit the developer's portfolio at [omerhub.com](https://omerhub.com/).
> 

## Features

- **No Hardware Dividers:** Uses pre-calculated inverted parameters.
- **Multiplier-less Trigonometry:** Implements a 16-step CORDIC algorithm for Sine/Cosine.
- **Overflow Protection:** 64-bit internal multipliers to prevent 32-bit fixed-point clipping.
- **Python API:** Ready-to-use Python class for Hardware-in-the-Loop (HIL) integration.
- **Parameterizable (IP Core):** Wheel radius and base length can be easily overridden during module instantiation for different robot chassis.

## UART Communication Protocol (115200 Baud, 8-N-1)

The system acts as a coprocessor. You send target velocities, and it returns the motor commands and new odometry estimations.

**RX Data Frame (PC to FPGA) - 8 Bytes:**

- `Byte 0-3` : Target Linear Velocity (V) - *Q16.16*
- `Byte 4-7` : Target Angular Velocity (Omega) - *Q16.16*

**TX Data Frame (FPGA to PC) - 24 Bytes:**

- `Byte 0-3` : Right Wheel Velocity Command - *Q16.16*
- `Byte 4-7` : Left Wheel Velocity Command - *Q16.16*
- `Byte 8-11` : Estimated X Position - *Q16.16*
- `Byte 12-15`: Estimated Y Position - *Q16.16*
- `Byte 16-19`: Estimated Orientation (Theta) - *Q16.16*
- `Byte 20-23`: Hardware Execution Cycles - *Unsigned 32-bit Integer*

## Quick Start (Python)

Ensure your FPGA is flashed with the provided Verilog codes and connected via USB.

```python
from fpga_differential_drive import FPGAKinematicsCore

# Initialize the FPGA Hardware
robot = FPGAKinematicsCore(port='COM4')

# Send target velocities (1.5 m/s linear, 0.5 rad/s angular)
result = robot.compute(target_v=1.5, target_omega=0.5)

print(f"Right Motor Command: {result['phi_r']} rad/s")
print(f"Execution Time: {result['fpga_time_ns']} nanoseconds")

robot.close()
```

## About the Developer

Designed and developed by **Ömer Lütfi**.
If you have any questions about adapting this IP Core to your custom robot chassis or integrating it into your autonomous systems, feel free to reach out.

- **Website:** [omerhub.com](https://omerhub.com/)
- **Email: contact@omerhub.com**
