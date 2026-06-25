#include "BalanceControl.h"
#include "ImuProcessing.h"
#include "MotorActuator.h"
#include "test_framework.h"
#include <string.h>

static void put_i16(uint8_t *raw, uint32_t index, int16_t value)
{
    raw[index * 2U] = (uint8_t)(((uint16_t)value >> 8U) & 0xFFU);
    raw[(index * 2U) + 1U] = (uint8_t)((uint16_t)value & 0xFFU);
}

static ImuProcessing_Config imu_test_config(void)
{
    ImuProcessing_Config config = ImuProcessing_DefaultConfig;

    config.accel_calibration[0] = 1.0f;
    config.accel_calibration[1] = 0.0f;
    config.accel_calibration[2] = 1.0f;
    config.accel_calibration[3] = 0.0f;
    config.accel_calibration[4] = 1.0f;
    config.accel_calibration[5] = 0.0f;
    config.gyro_bias[0] = 0.0f;
    config.gyro_bias[1] = 0.0f;
    config.gyro_bias[2] = 0.0f;
    config.accel_lsb_sensitivity = 2048.0f;
    config.gyro_lsb_sensitivity = 16.4f;
    config.fusion_mode = IMU_PROCESSING_FUSION_ACCEL_ONLY;

    return config;
}

static BalanceControl_Config balance_test_config(void)
{
    BalanceControl_Config config = BalanceControl_DefaultConfig;

    config.pid.Kp = 10.0f;
    config.pid.Ki = 0.0f;
    config.pid.Kd = 0.0f;
    config.pid.max_rate = 0.0f;

    return config;
}

static void balance_pipeline_converts_sensor_frame_to_motor_output(void)
{
    ImuProcessing_State imu;
    ImuProcessing_Config imu_config = imu_test_config();
    BalanceControl balance;
    BalanceControl_Config balance_config = balance_test_config();
    MotorActuator_Config motor_config = MotorActuator_DefaultConfig;
    BalanceControl_Output balance_output;
    MotorActuator_Output motor_output;
    uint8_t raw[IMU_PROCESSING_RAW_SIZE];

    memset(raw, 0, sizeof(raw));
    put_i16(raw, 2U, 2048);
    motor_config.max_duty = 99.0f;

    ImuProcessing_Init(&imu, &imu_config);
    BalanceControl_Init(&balance, &balance_config);

    ImuProcessing_Update(&imu, &imu_config, raw, 0.001f);
    balance_output = BalanceControl_Update(&balance, &balance_config, imu.pitch_deg, imu.dt_s);
    motor_output = MotorActuator_Calculate(balance_output.motor_command, &motor_config);

    TEST_ASSERT_FLOAT_NEAR(0.0f, imu.pitch_deg, 0.0001f);
    TEST_ASSERT_UINT_EQ(1U, balance_output.motor_enable);
    TEST_ASSERT_FLOAT_NEAR(-25.0f, balance_output.motor_command, 0.0001f);
    TEST_ASSERT_INT_EQ(MOTOR_ACTUATOR_DIRECTION_REVERSE, motor_output.left_direction);
    TEST_ASSERT_UINT_EQ(35U, motor_output.left_compare);
}

int main(void)
{
    RUN_TEST(balance_pipeline_converts_sensor_frame_to_motor_output);
    return TEST_REPORT();
}
