# Repository Guidelines

## Project Structure & Module Organization

TRobot is STM32H723 firmware built with CMake, STM32 HAL, FreeRTOS, and C++ components. Application behavior and task orchestration live in `app/`; board drivers belong in `bsp/` (`include/bsp/` for public APIs, `src/` for implementations, `internal/` for private headers). Reusable modules are organized under `components/`. CubeMX-managed code and configuration live in `Core/`, `Drivers/`, `Middlewares/`, `USB_DEVICE/`, `cmake/stm32cubemx/`, and `trobot.ioc`. Hardware/debug configurations are in the root `.cfg` files.

## Build, Test, and Development

Clone with submodules using `git clone --recursive <repo>`. Configure and build with `cmake --preset Debug` and `cmake --build --preset Debug`; use the corresponding `Release` preset for optimized builds. There is no standalone automated test suite, so compile changes and document relevant board-level checks. Use CLion’s CMake preset profiles when developing in the IDE.

Flashing is hardware-affecting: run `flash_and_verify` or OpenOCD programming only when explicitly requested. Confirm the connected probe before selecting `daplink.cfg` or `stlink.cfg`.

## Coding Style & Naming

Use C17 for BSP/generated C and C++23 for application and components. Indent with four spaces; use descriptive `snake_case` for functions and variables. Keep BSP APIs narrow and prefixed `bsp_`; preserve established component namespaces and third-party naming. Prefer typed constants and short functions. Comments should clarify non-obvious hardware timing, units, protocols, concurrency, or DMA/cache constraints.

## Configuration & Safety

Treat `trobot.ioc` as the source of truth for clocks, pins, DMA, NVIC, and peripheral settings. Do not directly edit the `.ioc`, generated `Core/` code outside user sections, `Drivers/`, `Middlewares/`, `USB_DEVICE/`, or startup assembly; describe required CubeMX changes for the maintainer to apply. After CubeMX regeneration, check whether `STM32H723xG_flash.ld` was overwritten. Keep custom behavior in `app/`, `bsp/`, or `components/`.

## Commits & Pull Requests

Recent history uses concise Chinese subjects with conventional prefixes, such as `feat:`, `fix:`, `chore:`, and `doc:`; scopes are welcome, e.g. `fix(bsp): 修复 CAN 接收`. Pull requests should summarize behavior and affected modules, report build results and hardware verification (or state that it was not performed), and link related issues. Include logs or screenshots when they clarify board behavior.
