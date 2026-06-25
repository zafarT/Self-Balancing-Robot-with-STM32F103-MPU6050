# Self-Balancing-Robot-with-STM32F103-MPU6050
This repository contains the firmware and documentation for a self-balancing two-wheeled robot built using the STM32F103 microcontroller and the MPU6050 IMU sensor. The project demonstrates real-time control, sensor fusion, and PID-based stabilization on a resource-constrained embedded platform.

## Architecture

The application-owned code is organized in an AUTOSAR-inspired layout:

- `ASW/` contains balancing, control, IMU processing, and sensor-fusion software components.
- `BSW/ECUAbstraction/` contains STM32-facing sensor and motor adapters.
- `Core/` and `Drivers/` keep the CubeMX/HAL generated structure.
- `tests/` contains host-side unit and integration tests for non-generated logic.

See `docs/autosar_layout.md` for the layer map and test commands.

## Tests

Run all host-side tests:

```sh
make -C tests run
```

Run the project-owned coverage gate:

```sh
make -C tests coverage
```

The coverage gate enforces 100% statement and branch coverage for `ASW/` and `BSW/` project-owned code, including the application glue and STM32-facing ECU abstraction through host-side HAL fakes. Vendor HAL/CMSIS, CubeMX-generated startup/interrupt code, and the third-party SSD1306 graphics driver are outside the coverage target.
