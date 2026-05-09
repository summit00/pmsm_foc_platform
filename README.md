# PMSM FOC Platform

This project implements a Field-Oriented Control (FOC) platform for Permanent Magnet Synchronous Motors (PMSM), targeting the STM32G431 microcontroller. It provides a complete motor control solution including sensorless control, parameter identification, and simulation capabilities for development and testing.

## Features

- **Field-Oriented Control (FOC)**: Implements advanced motor control algorithms for precise torque and speed control.
- **Motor Parameter Identification**: Automatic setup and tuning of motor parameters (resistance, inductance).
- **Multi-Platform Support**: Builds for both embedded STM32G431 and host simulation environments.
- **Simulation and Testing**: Software-in-the-Loop (SIL) simulation and unit tests for validation.
- **Modular Architecture**: Separated BSP, HAL, and application layers for portability.

## Project Structure

- `app/`: Core application logic including FOC, controllers, and motor models.
- `bsp/`: Board Support Package for STM32G431.
- `hal/`: Hardware Abstraction Layer for peripherals (ADC, PWM, encoders).
- `platform/`: Platform-specific implementations (STM32G431 and simulation).
- `tests/`: Unit tests and system-level simulation tests.
- `simulation/`: Motor models and solvers for SIL testing.
- `third_party/`: External dependencies like Catch2 for testing.

## Tools Needed

- **CMake** (tested with 4.2.3)
- **Arm GNU Toolchain** (tested with Arm GNU Toolchain 13.3.Rel1)
- **Ninja** (tested with 1.13.2)
- **STM32CubeProgrammer** (tested with v2.21.0)

## Building and Running

### Embedded Build (STM32G431)

1. Configure the project:
   ```
   cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Debug -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi-toolchain.cmake
   ```

2. Build the project:
   ```
   cmake --build build
   ```

3. Flash to the board:
   ```
   STM32_Programmer_CLI.exe -c port=SWD -w build/platform/pmsm_foc_platform.elf -v -rst
   ```

### Host Build (Simulation and Tests)

1. Configure for host:
   ```
   cmake -S . -B build-host -G Ninja -DPLATFORM=host -DBUILD_TESTS=ON -DCMAKE_BUILD_TYPE=Debug
   ```

2. Build unit tests:
   ```
   cmake --build build-host --target unit_tests
   ```

3. Run unit tests:
   ```
   ./build-host/tests/unit_tests
   ```

4. Build SIL simulation:
   ```
   cmake --build build-host --target system_test
   ```

5. Run SIL simulation:
   ```
   ./build-host/tests/sim/system_test.exe -s
   ```

## Encoder Wiring (for Reference)

- Red --> Vcc
- Black --> GND
- Green --> A
- White --> B
- Orange --> Index

## License

MIT License

Copyright (c) 2026

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.