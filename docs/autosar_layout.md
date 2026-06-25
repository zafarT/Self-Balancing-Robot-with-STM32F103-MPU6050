# AUTOSAR-Inspired Layout

This project is still a CubeMX/HAL bare-metal firmware project, so it is not a full AUTOSAR Classic stack. The source tree now follows AUTOSAR layering as far as practical without breaking CubeMX regeneration.

## Ownership

- `Core/` remains CubeMX-owned integration code. User code blocks call into application modules, but peripheral init and interrupt vectors stay in the generated shape.
- `Drivers/` remains vendor HAL/CMSIS.
- `ASW/` contains application software components and pure logic.
- `BSW/ECUAbstraction/` contains hardware-facing drivers and actuator/sensor abstractions above STM32 HAL.
- `tests/` contains host-side unit and integration tests for non-generated code.

## Layer Map

| Layer | Project path | Main responsibility |
| --- | --- | --- |
| Application SWC | `ASW/Balance` | Main balancing runnable and callback dispatch |
| Control SWC | `ASW/Control` | PID controller and fall/target-angle policy |
| Sensor SWC | `ASW/Sensors/Imu` | MPU6050 byte decoding, calibration, fusion input/output state |
| Sensor fusion SWC | `ASW/SensorFusion` | Kalman filter and accelerometer angle helpers |
| ECU abstraction | `BSW/ECUAbstraction/Motor` | Motor command mapping plus STM32 GPIO/PWM adapter |
| ECU abstraction | `BSW/ECUAbstraction/Imu` | MPU6050 STM32 I2C/DMA adapter |
| MCAL/vendor | `Drivers/STM32F1xx_HAL_Driver`, `Drivers/CMSIS` | HAL, CMSIS, device headers |

Compatibility headers are kept in `Core/Inc` for existing includes. They forward to the new module headers.

## Tests

Run all host tests:

```sh
make -C tests run
```

Run the project-owned 100% statement/branch coverage gate:

```sh
make -C tests coverage
```

The tests cover:

- PID controller reset, saturation, rate limiting, and proportional/integral behavior
- Balance fall detection and motor-command decisions
- Motor actuator direction, deadband, saturation, and PWM compare calculation
- IMU byte conversion, accelerometer calibration, pitch/roll calculation, sample timing
- Kalman filter initialization and angle helpers
- Integration flow from IMU frame to balance command to motor actuator output
- STM32-facing MPU6050, motor-driver, and application glue using host HAL fakes

Coverage scope:

- Included: project-owned `ASW/` and `BSW/` code plus `ASW/Balance` application glue.
- Excluded: vendor HAL/CMSIS, CubeMX-generated startup/interrupt scaffolding, and the third-party SSD1306 graphics library.
- The coverage gate checks both statement and branch coverage, matching the ISTQB CTFL white-box coverage concepts while the test cases use CTFL-style equivalence partitioning, boundary values, decision tables, state transitions, and error guessing.

## Firmware Build

Build with the STM32 makefile:

```sh
make -f STM32Make.make -j4
```

The makefile and VS Code/CubeIDE include paths have been updated for `ASW` and `BSW`.
