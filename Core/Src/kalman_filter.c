


#include "kalman_filter.h"
#include "math.h"


void Kalman_Init(KalmanFilter *kf) {
    kf->angle = 0.0f;
    kf->bias = 0.0f;
    kf->P[0][0] = 0.0f;
    kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f;
    kf->P[1][1] = 0.0f;
    kf->Q_angle = 0.001f;
    kf->Q_bias = 0.003f;
    kf->R_measure = 0.03f;
}





float Kalman_Update(KalmanFilter *kf, float newAngle, float newRate, float dt) {
    // Predict
    kf->rate = newRate - kf->bias;
    kf->angle += dt * kf->rate;

    // Update error covariance matrix
    kf->P[0][0] += dt * (dt*kf->P[1][1] - kf->P[0][1] - kf->P[1][0] + kf->Q_angle);
    kf->P[0][1] -= dt * kf->P[1][1];
    kf->P[1][0] -= dt * kf->P[1][1];
    kf->P[1][1] += kf->Q_bias * dt;

    // Measurement update
    float S = kf->P[0][0] + kf->R_measure;
    float K[2];
    K[0] = kf->P[0][0] / S;
    K[1] = kf->P[1][0] / S;

    float y = newAngle - kf->angle;
    kf->angle += K[0] * y;
    kf->bias += K[1] * y;

    // Update covariance matrix
    float P00_temp = kf->P[0][0];
    float P01_temp = kf->P[0][1];

    kf->P[0][0] -= K[0] * P00_temp;
    kf->P[0][1] -= K[0] * P01_temp;
    kf->P[1][0] -= K[1] * P00_temp;
    kf->P[1][1] -= K[1] * P01_temp;

    return kf->angle;
}


float getRoll(float accY, float accZ) {
    return atan2(accY, accZ) * 180.0f / M_PI;
}

float getPitch(float accX, float accY, float accZ) {
    return atan2(-accX, sqrt(accY*accY + accZ*accZ)) * 180.0f / M_PI;
}

