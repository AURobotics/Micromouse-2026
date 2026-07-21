# raw_algorithm/

MCU-agnostic algorithm code, written in portable C/C++ with zero STM32/HAL
dependency. Build and test it on your desktop with a normal compiler — no
hardware or MCU toolchain needed.

- `maze/` — maze representation + solving algorithm (flood-fill, etc.)
- `control/` — PID controllers, motion profile generation
- `tests/` — unit tests (Unity, GoogleTest, etc.)

firmware/CMakeLists.txt add_subdirectory()'s this folder and links against
it, so the exact code tested here runs unmodified on the real robot.
