CC ?= cc
BUILD_DIR := build

CFLAGS ?= -std=c11 -Wall -Wextra -Wpedantic -O0 -g
INCLUDES := \
	-Isupport \
	-I../ASW/Balance/Inc \
	-I../ASW/Control/Inc \
	-I../ASW/SensorFusion/Inc \
	-I../ASW/Sensors/Imu/Inc \
	-I../BSW/ECUAbstraction/Imu/Inc \
	-I../BSW/ECUAbstraction/Motor/Inc \
	-I../Core/Inc

PID_SRCS := ../ASW/Control/Src/PidController.c
BALANCE_SRCS := ../ASW/Control/Src/BalanceControl.c $(PID_SRCS)
MOTOR_SRCS := ../BSW/ECUAbstraction/Motor/Src/MotorActuator.c
KALMAN_SRCS := ../ASW/SensorFusion/Src/KalmanFilter.c
IMU_SRCS := ../ASW/Sensors/Imu/Src/ImuProcessing.c $(KALMAN_SRCS)
PIPELINE_SRCS := $(IMU_SRCS) $(BALANCE_SRCS) $(MOTOR_SRCS)
PROJECT_OWNED_SRCS := \
	../ASW/Balance/Src/BalanceApp.c \
	../ASW/Control/Src/BalanceControl.c \
	../ASW/Control/Src/PidController.c \
	../ASW/SensorFusion/Src/KalmanFilter.c \
	../ASW/Sensors/Imu/Src/ImuProcessing.c \
	../BSW/ECUAbstraction/Imu/Src/Mpu6050_Stm32.c \
	../BSW/ECUAbstraction/Motor/Src/MotorActuator.c \
	../BSW/ECUAbstraction/Motor/Src/MotorDriver_Stm32.c
PROJECT_COVERAGE_SUPPORT_SRCS := support/fake_stm32.c
PROJECT_COVERAGE_GCNO := \
	$(BUILD_DIR)/test_project_owned_coverage-BalanceApp.gcno \
	$(BUILD_DIR)/test_project_owned_coverage-BalanceControl.gcno \
	$(BUILD_DIR)/test_project_owned_coverage-PidController.gcno \
	$(BUILD_DIR)/test_project_owned_coverage-KalmanFilter.gcno \
	$(BUILD_DIR)/test_project_owned_coverage-ImuProcessing.gcno \
	$(BUILD_DIR)/test_project_owned_coverage-Mpu6050_Stm32.gcno \
	$(BUILD_DIR)/test_project_owned_coverage-MotorActuator.gcno \
	$(BUILD_DIR)/test_project_owned_coverage-MotorDriver_Stm32.gcno
PROJECT_COVERAGE_REPORTS := \
	BalanceApp.c.gcov \
	BalanceControl.c.gcov \
	PidController.c.gcov \
	KalmanFilter.c.gcov \
	ImuProcessing.c.gcov \
	Mpu6050_Stm32.c.gcov \
	MotorActuator.c.gcov \
	MotorDriver_Stm32.c.gcov

TESTS := \
	$(BUILD_DIR)/test_pid_controller \
	$(BUILD_DIR)/test_balance_control \
	$(BUILD_DIR)/test_motor_actuator \
	$(BUILD_DIR)/test_imu_processing \
	$(BUILD_DIR)/test_kalman_filter \
	$(BUILD_DIR)/test_balance_pipeline \
	$(BUILD_DIR)/test_project_owned_coverage

.PHONY: all run coverage clean

all: $(TESTS)

run: all
	@for test in $(TESTS); do \
		echo "== $$test =="; \
		./$$test || exit 1; \
	done

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(BUILD_DIR)/test_pid_controller: unit/test_pid_controller.c $(PID_SRCS) | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(INCLUDES) $^ -lm -o $@

$(BUILD_DIR)/test_balance_control: unit/test_balance_control.c $(BALANCE_SRCS) | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(INCLUDES) $^ -lm -o $@

$(BUILD_DIR)/test_motor_actuator: unit/test_motor_actuator.c $(MOTOR_SRCS) | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(INCLUDES) $^ -lm -o $@

$(BUILD_DIR)/test_imu_processing: unit/test_imu_processing.c $(IMU_SRCS) | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(INCLUDES) $^ -lm -o $@

$(BUILD_DIR)/test_kalman_filter: unit/test_kalman_filter.c $(KALMAN_SRCS) | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(INCLUDES) $^ -lm -o $@

$(BUILD_DIR)/test_balance_pipeline: integration/test_balance_pipeline.c $(PIPELINE_SRCS) | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(INCLUDES) $^ -lm -o $@

$(BUILD_DIR)/test_project_owned_coverage: coverage/test_project_owned_coverage.c $(PROJECT_OWNED_SRCS) $(PROJECT_COVERAGE_SUPPORT_SRCS) | $(BUILD_DIR)
	$(CC) $(CFLAGS) -DUNIT_TEST $(INCLUDES) $^ -lm -o $@

coverage: clean
	$(MAKE) CFLAGS="$(CFLAGS) --coverage" run
	rm -f *.gcov
	@for gcno in $(PROJECT_COVERAGE_GCNO); do \
		gcov -b -c -o $(BUILD_DIR) $$gcno >/dev/null || exit 1; \
	done
	sh support/check_gcov_100.sh $(PROJECT_COVERAGE_REPORTS)

clean:
	rm -rf $(BUILD_DIR)
	rm -f *.gcov
