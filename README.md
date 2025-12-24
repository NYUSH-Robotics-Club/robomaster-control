# RoboMaster Control

Firmware and tooling for the NYUSH Robotics Club RoboMaster C Board (STM32F4).

- Quick start: see the [Setup Guide](docs/tutorials/setup-guide.md)
- Toolchain: CMake + Ninja, ARM GNU Toolchain, STM32CubeProgrammer
- Target: STM32F407

## Documentation

- **[Setup Guide](docs/tutorials/setup-guide.md)** - Getting started with development environment setup
- **[Architecture Overview](docs/architecture.md)** - Three-layer architecture (application, module, hardware)
- **[Publish-Subscribe System](docs/pub-sub.md)** - Message center and event-driven communication
- **[Vision Protocol](docs/vision-protocol.md)** - Seasky protocol for vision system communication
- **[CAN Communication](docs/tutorials/can.md)** - CAN bus protocol and motor communication

## Repository layout

- `Inc/`, `Src/` — application sources and headers
- `Drivers/`, `Middlewares/` — vendor libraries (STM32 HAL, USB device)
- `cmake/`, `CMakeLists.txt` — CMake configuration
- `Debug/`, `build/` — build outputs (generated)
- `docs/` — guides and reference

## Build & flash (summary)

### Build
- Use CMake Tools in VS Code (see Setup Guide Step 5)
- Or use VSCode tasks: `Ctrl+Shift+B` / `Cmd+Shift+B`

### Flash
Two methods available:

1. **STM32CubeProgrammer** (GUI method)
   - Use STM32CubeProgrammer via USB (see Setup Guide Step 6)

2. **VSCode Tasks** (command-line method)
   - `flash: dfu-util` - Flash via USB using DFU mode (default build task)
   - `flash: openocd (stlink)` - Flash using ST-Link debugger
   - See Setup Guide Step 7 for detailed instructions
