# firmware/

STM32F446 CMake project.

- `Core/` — main.c, startup, interrupt handlers, Lib/ (drivers)
- `Config/` — FreeRTOSConfig.h, stm32f4xx_hal_conf.h (project-specific, not vendor)
- `cmake/` — toolchain file
- `STM32CubeF4/` — submodule: kol haga 
- `CMakeLists.txt` — also add_subdirectory's ../raw_algorithm so the same
  maze/control code tested on desktop compiles straight into the firmware
  (still not sure law dah hayenfa3)
