#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float angle;
    float bias;
    float rate;
    float P[2][2];
    float Q_angle;
    float Q_bias;
    float R_measure;
} KalmanFilter;

void Kalman_Init(KalmanFilter *kf);
float Kalman_Update(KalmanFilter *kf, float newAngle, float newRate, float dt);
float getRoll(float accY, float accZ);
float getPitch(float accX, float accY, float accZ);

#ifdef __cplusplus
}
#endif

#endif /* KALMAN_FILTER_H_ */
