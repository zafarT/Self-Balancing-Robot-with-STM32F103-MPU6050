# Test Strategy

This project uses host-side tests for project-owned firmware code. The test basis is the AUTOSAR-inspired architecture in `docs/autosar_layout.md`, the public module headers, the observable behavior of the STM32 adapters through HAL fakes, and the ISTQB CTFL v4.0.1 syllabus: https://istqb.org/wp-content/uploads/2024/11/ISTQB_CTFL_Syllabus_v4.0.1.pdf.

## ISTQB CTFL Alignment

The tests are organized around the CTFL v4.0.1 techniques:

- Equivalence partitioning and boundary value analysis for PID, motor command ranges, IMU timing, deadband, saturation, and sensor scaling.
- Decision table testing for initialization success/failure combinations in the application, MPU6050 adapter, timer start, display init, and HAL return codes.
- State transition testing for balance active/fall states, MPU6050 data-ready/I2C-ready states, and callback routing.
- White-box statement and branch testing for project-owned modules.
- Error guessing for null pointers, busy buses, failed DMA/I2C writes, invalid timing, and defensive fallback paths.

## Commands

Run all tests:

```sh
make -C tests run
```

Run the 100% coverage gate:

```sh
make -C tests coverage
```

The coverage gate currently checks:

- `ASW/Balance/Src/BalanceApp.c`
- `ASW/Control/Src/BalanceControl.c`
- `ASW/Control/Src/PidController.c`
- `ASW/SensorFusion/Src/KalmanFilter.c`
- `ASW/Sensors/Imu/Src/ImuProcessing.c`
- `BSW/ECUAbstraction/Imu/Src/Mpu6050_Stm32.c`
- `BSW/ECUAbstraction/Motor/Src/MotorActuator.c`
- `BSW/ECUAbstraction/Motor/Src/MotorDriver_Stm32.c`

Vendor HAL/CMSIS, CubeMX-generated startup/interrupt scaffolding, and the third-party SSD1306 graphics library are intentionally excluded from this host coverage gate.
