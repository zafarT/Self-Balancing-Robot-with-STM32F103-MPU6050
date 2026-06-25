#include "ImuProcessing.h"
#include <string.h>

#define IMU_PROCESSING_TEMP_INDEX      3U
#define IMU_PROCESSING_GYRO_START      4U
#define IMU_PROCESSING_TEMP_SCALE      340.0f
#define IMU_PROCESSING_TEMP_OFFSET     36.53f
#define IMU_PROCESSING_DEFAULT_DT_S    0.001f

const ImuProcessing_Config ImuProcessing_DefaultConfig = {
    {
        0.9973217f, -0.0090093f,
        0.9980508f, 0.0058476f,
        1.0061410f, 0.1957754f
    },
    {
        0.1220f,
        0.060976f,
        -0.060976f
    },
    16384.0f,
    131.0f,
    0.995f,
    IMU_PROCESSING_FUSION_COMPLEMENTARY
};

void ImuProcessing_Init(ImuProcessing_State *state, const ImuProcessing_Config *config)
{
    (void)config;

    if (state == 0)
    {
        return;
    }

    memset(state, 0, sizeof(*state));
    state->dt_s = IMU_PROCESSING_DEFAULT_DT_S;
    Kalman_Init(&state->kalman_roll);
    Kalman_Init(&state->kalman_pitch);
}

int16_t ImuProcessing_BytesToInt(uint8_t first_byte, uint8_t second_byte)
{
    return (int16_t)(((uint16_t)first_byte << 8U) | (uint16_t)second_byte);
}

void ImuProcessing_Update(ImuProcessing_State *state,
                          const ImuProcessing_Config *config,
                          const uint8_t raw_bytes[IMU_PROCESSING_RAW_SIZE],
                          float dt_s)
{
    const ImuProcessing_Config *active_config = config;
    float roll_acc;
    float pitch_acc;
    uint32_t index;

    if ((state == 0) || (raw_bytes == 0))
    {
        return;
    }

    if (active_config == 0)
    {
        active_config = &ImuProcessing_DefaultConfig;
    }

    if (dt_s <= 0.0f)
    {
        dt_s = IMU_PROCESSING_DEFAULT_DT_S;
    }

    for (index = 0U; index < IMU_PROCESSING_DATA_COUNT; index++)
    {
        const int16_t raw = ImuProcessing_BytesToInt(raw_bytes[index * 2U], raw_bytes[(index * 2U) + 1U]);

        if (index == IMU_PROCESSING_TEMP_INDEX)
        {
            state->data[index] = ((float)raw / IMU_PROCESSING_TEMP_SCALE) + IMU_PROCESSING_TEMP_OFFSET;
        }
        else if (index >= IMU_PROCESSING_GYRO_START)
        {
            const uint32_t gyro_index = index - IMU_PROCESSING_GYRO_START;
            state->data[index] = ((float)raw / active_config->gyro_lsb_sensitivity) -
                                 active_config->gyro_bias[gyro_index];
        }
        else
        {
            const float scaled = (float)raw / active_config->accel_lsb_sensitivity;
            state->data[index] = (active_config->accel_calibration[index * 2U] * scaled) +
                                 active_config->accel_calibration[(index * 2U) + 1U];
        }
    }

    state->dt_s = dt_s;
    state->sample_count++;
    state->sample_hz = 1.0f / dt_s;

    roll_acc = getRoll(state->data[1], state->data[2]);
    pitch_acc = getPitch(state->data[0], state->data[1], state->data[2]);

    if (active_config->fusion_mode == IMU_PROCESSING_FUSION_KALMAN)
    {
        state->roll_deg = Kalman_Update(&state->kalman_roll, roll_acc, state->data[4], dt_s);
        state->pitch_deg = Kalman_Update(&state->kalman_pitch, pitch_acc, state->data[5], dt_s);
    }
    else if (active_config->fusion_mode == IMU_PROCESSING_FUSION_COMPLEMENTARY)
    {
        state->roll_deg = (active_config->complementary_alpha * (state->roll_deg + (state->data[4] * dt_s))) +
                          ((1.0f - active_config->complementary_alpha) * roll_acc);
        state->pitch_deg = (active_config->complementary_alpha * (state->pitch_deg + (state->data[5] * dt_s))) +
                           ((1.0f - active_config->complementary_alpha) * pitch_acc);
    }
    else
    {
        state->roll_deg = roll_acc;
        state->pitch_deg = pitch_acc;
    }
}
