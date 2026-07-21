# Micromouse

STM32F446-based micromouse robot.

## Structure
- `firmware/` — STM32 CMake project (drivers, RTOS, board code)
- `raw_algorithm/` — MCU-agnostic maze-solving & control algorithms, unit-tested
  on the desktop, linked into firmware as a library (no copy-paste, no drift
  between sim and real robot)

## Setup
```
git clone --recurse-submodules <repo-url>
```
