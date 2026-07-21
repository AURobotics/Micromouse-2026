# Micromouse

STM32F446-based micromouse robot.

## Structure
- `firmware/` — STM32 CMake project (drivers, RTOS, board code)
- `raw_algorithm/` — MCU-agnostic maze-solving & control algorithms, unit-tested
  on the desktop, linked into firmware as a library 

## Setup
```
git clone --recurse-submodules <repo-url>
```
