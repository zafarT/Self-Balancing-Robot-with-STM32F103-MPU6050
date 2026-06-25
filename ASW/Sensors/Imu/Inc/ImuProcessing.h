#ifndef IMU_PROCESSING_H_
#define IMU_PROCESSING_H_

#include <stdint.h>
#include "KalmanFilter.h"

#ifdef __cplusplus
extern "C" {
#endif

#define IMU_PROCESSING_AXIS_COUNT 3U
#define IMU_PROCESSING_DATA_COUNT 7U
#define IMU_PROCESSING_RAW_SIZE   14U

typedef enum
{
    IMU_PROCESSING_FUSION_ACCEL_ONLY = 0,
    IMU_PROCESSING_FUSION_COMPLEMENTARY,
    IMU_PROCESSING_FUSION_KALMAN
} ImuProcessing_FusionMode;

typedef struct
{
    float accel_calibration[IMU_PROCESSING_AXIS_COUNT * 2U];
    float gyro_bias[IMU_PROCESSING_AXIS_COUNT];
    float accel_lsb_sensitivity;
    float gyro_lsb_sensitivity;
    float complementary_alpha;
    ImuProcessing_FusionMode fusion_mode;
} ImuProcessing_Config;

typedef struct
{
    float data[IMU_PROCESSING_DATA_COUNT];
    float roll_deg;
    float pitch_deg;
    float dt_s;
    float sample_hz;
    uint32_t sample_count;
    KalmanFilter kalman_roll;
    KalmanFilter kalman_pitch;
} ImuProcessing_State;

extern const ImuProcessing_Config ImuProcessing_DefaultConfig;

void ImuProcessing_Init(ImuProcessing_State *state, const ImuProcessing_Config *config);
int16_t ImuProcessing_BytesToInt(uint8_t first_byte, uint8_t second_byte);
void ImuProcessing_Update(ImuProcessing_State *state,
                          const ImuProcessing_Config *config,
                          const uint8_t raw_bytes[IMU_PROCESSING_RAW_SIZE],
                          float dt_s);

#ifdef __cplusplus
}
#endif

#endif /* IMU_PROCESSING_H_ */
