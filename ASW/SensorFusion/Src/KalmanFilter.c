#include "KalmanFilter.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void Kalman_Init(KalmanFilter *kf)
{
    if (kf == 0)
    {
        return;
    }

    kf->angle = 0.0f;
    kf->bias = 0.0f;
    kf->rate = 0.0f;
    kf->P[0][0] = 0.0f;
    kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f;
    kf->P[1][1] = 0.0f;
    kf->Q_angle = 0.001f;
    kf->Q_bias = 0.003f;
    kf->R_measure = 0.03f;
}

float Kalman_Update(KalmanFilter *kf, float newAngle, float newRate, float dt)
{
    float S;
    float K[2];
    float y;
    float P00_temp;
    float P01_temp;

    if (kf == 0)
    {
        return 0.0f;
    }

    kf->rate = newRate - kf->bias;
    kf->angle += dt * kf->rate;

    kf->P[0][0] += dt * ((dt * kf->P[1][1]) - kf->P[0][1] - kf->P[1][0] + kf->Q_angle);
    kf->P[0][1] -= dt * kf->P[1][1];
    kf->P[1][0] -= dt * kf->P[1][1];
    kf->P[1][1] += kf->Q_bias * dt;

    S = kf->P[0][0] + kf->R_measure;
    K[0] = kf->P[0][0] / S;
    K[1] = kf->P[1][0] / S;

    y = newAngle - kf->angle;
    kf->angle += K[0] * y;
    kf->bias += K[1] * y;

    P00_temp = kf->P[0][0];
    P01_temp = kf->P[0][1];

    kf->P[0][0] -= K[0] * P00_temp;
    kf->P[0][1] -= K[0] * P01_temp;
    kf->P[1][0] -= K[1] * P00_temp;
    kf->P[1][1] -= K[1] * P01_temp;

    return kf->angle;
}

float getRoll(float accY, float accZ)
{
    return atan2f(accY, accZ) * 180.0f / (float)M_PI;
}

float getPitch(float accX, float accY, float accZ)
{
    return atan2f(-accX, sqrtf((accY * accY) + (accZ * accZ))) * 180.0f / (float)M_PI;
}
