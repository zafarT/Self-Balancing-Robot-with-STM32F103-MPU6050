#include "ImuProcessing.h"
#include "test_framework.h"
#include <string.h>

static void put_i16(uint8_t *raw, uint32_t index, int16_t value)
{
    raw[index * 2U] = (uint8_t)(((uint16_t)value >> 8U) & 0xFFU);
    raw[(index * 2U) + 1U] = (uint8_t)((uint16_t)value & 0xFFU);
}

static ImuProcessing_Config identity_config(void)
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

static void imu_processing_converts_signed_big_endian_bytes(void)
{
    TEST_ASSERT_INT_EQ(-2, ImuProcessing_BytesToInt(0xFFU, 0xFEU));
    TEST_ASSERT_INT_EQ(2048, ImuProcessing_BytesToInt(0x08U, 0x00U));
}

static void imu_processing_decodes_level_sensor_frame(void)
{
    ImuProcessing_State state;
    ImuProcessing_Config config = identity_config();
    uint8_t raw[IMU_PROCESSING_RAW_SIZE];

    memset(raw, 0, sizeof(raw));
    put_i16(raw, 2U, 2048);

    ImuProcessing_Init(&state, &config);
    ImuProcessing_Update(&state, &config, raw, 0.001f);

    TEST_ASSERT_FLOAT_NEAR(0.0f, state.data[0], 0.0001f);
    TEST_ASSERT_FLOAT_NEAR(1.0f, state.data[2], 0.0001f);
    TEST_ASSERT_FLOAT_NEAR(36.53f, state.data[3], 0.0001f);
    TEST_ASSERT_FLOAT_NEAR(0.0f, state.roll_deg, 0.0001f);
    TEST_ASSERT_FLOAT_NEAR(0.0f, state.pitch_deg, 0.0001f);
    TEST_ASSERT_FLOAT_NEAR(1000.0f, state.sample_hz, 0.1f);
    TEST_ASSERT_UINT_EQ(1U, state.sample_count);
}

static void imu_processing_reports_pitch_from_accelerometer(void)
{
    ImuProcessing_State state;
    ImuProcessing_Config config = identity_config();
    uint8_t raw[IMU_PROCESSING_RAW_SIZE];

    memset(raw, 0, sizeof(raw));
    put_i16(raw, 0U, 2048);

    ImuProcessing_Init(&state, &config);
    ImuProcessing_Update(&state, &config, raw, 0.001f);

    TEST_ASSERT_FLOAT_NEAR(-90.0f, state.pitch_deg, 0.001f);
}

int main(void)
{
    RUN_TEST(imu_processing_converts_signed_big_endian_bytes);
    RUN_TEST(imu_processing_decodes_level_sensor_frame);
    RUN_TEST(imu_processing_reports_pitch_from_accelerometer);
    return TEST_REPORT();
}
