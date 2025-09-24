/*
 * kalman_filter.h
 *
 *  Created on: Sep 17, 2025
 *      Author: zafar
 */


typedef struct {
    float angle;      // Estimated angle
    float bias;       // Gyro bias
    float rate;       // Unbiased rate
    float P[2][2];    // Error covariance matrix
    float Q_angle;    // Process noise variance for angle
    float Q_bias;     // Process noise variance for bias
    float R_measure;  // Measurement noise variance
} KalmanFilter;



void Kalman_Init(KalmanFilter *kf);
float Kalman_Update(KalmanFilter *kf, float newAngle, float newRate, float dt);
float getRoll(float accY, float accZ);
float getPitch(float accX, float accY, float accZ);



/*
 * kalman_filter.c
 *
 *  Created on: Sep 17, 2025
 *      Author: zafar
 */

#ifndef INC_KALMAN_FILTER_C_
#define INC_KALMAN_FILTER_C_



#endif /* INC_KALMAN_FILTER_C_ */
