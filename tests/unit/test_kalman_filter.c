#include "KalmanFilter.h"
#include "test_framework.h"
#include <math.h>

static void kalman_init_sets_expected_defaults(void)
{
    KalmanFilter filter;

    Kalman_Init(&filter);

    TEST_ASSERT_FLOAT_NEAR(0.0f, filter.angle, 0.0001f);
    TEST_ASSERT_FLOAT_NEAR(0.001f, filter.Q_angle, 0.0001f);
    TEST_ASSERT_FLOAT_NEAR(0.003f, filter.Q_bias, 0.0001f);
    TEST_ASSERT_FLOAT_NEAR(0.03f, filter.R_measure, 0.0001f);
}

static void angle_helpers_convert_accelerometer_vectors(void)
{
    TEST_ASSERT_FLOAT_NEAR(90.0f, getRoll(1.0f, 0.0f), 0.001f);
    TEST_ASSERT_FLOAT_NEAR(-90.0f, getPitch(1.0f, 0.0f, 0.0f), 0.001f);
}

static void kalman_update_returns_finite_angle(void)
{
    KalmanFilter filter;
    float angle;

    Kalman_Init(&filter);
    angle = Kalman_Update(&filter, 10.0f, 0.0f, 0.01f);

    TEST_ASSERT_TRUE(isfinite(angle));
}

int main(void)
{
    RUN_TEST(kalman_init_sets_expected_defaults);
    RUN_TEST(angle_helpers_convert_accelerometer_vectors);
    RUN_TEST(kalman_update_returns_finite_angle);
    return TEST_REPORT();
}
