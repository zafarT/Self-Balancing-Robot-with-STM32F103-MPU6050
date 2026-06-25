#include "PidController.h"
#include "test_framework.h"

static void pid_step_applies_proportional_and_integral_terms(void)
{
    PidController controller;
    PidController_Config config = {
        2.0f,
        1.0f,
        0.0f,
        0.0f,
        0.0f,
        0.1f,
        10.0f,
        -10.0f,
        0.0f
    };

    PidController_Init(&controller, &config);

    TEST_ASSERT_FLOAT_NEAR(-6.3f, PidController_Step(&controller, 3.0f, 0.1f), 0.0001f);
    TEST_ASSERT_FLOAT_NEAR(-0.3f, controller.integral, 0.0001f);
}

static void pid_step_saturates_output(void)
{
    PidController controller;
    PidController_Config config = {
        10.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0.1f,
        5.0f,
        -5.0f,
        0.0f
    };

    PidController_Init(&controller, &config);

    TEST_ASSERT_FLOAT_NEAR(-5.0f, PidController_Step(&controller, 1.0f, 0.1f), 0.0001f);
}

static void pid_step_limits_output_rate(void)
{
    PidController controller;
    PidController_Config config = {
        10.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0.1f,
        100.0f,
        -100.0f,
        10.0f
    };

    PidController_Init(&controller, &config);

    TEST_ASSERT_FLOAT_NEAR(1.0f, PidController_Step(&controller, -10.0f, 0.1f), 0.0001f);
}

static void pid_reset_clears_runtime_state(void)
{
    PidController controller;

    PidController_Init(&controller, 0);
    (void)PidController_Step(&controller, 2.0f, 0.01f);
    PidController_Reset(&controller);

    TEST_ASSERT_FLOAT_NEAR(0.0f, controller.integral, 0.0001f);
    TEST_ASSERT_FLOAT_NEAR(0.0f, controller.err_prev, 0.0001f);
    TEST_ASSERT_FLOAT_NEAR(0.0f, controller.command_sat_prev, 0.0001f);
}

int main(void)
{
    RUN_TEST(pid_step_applies_proportional_and_integral_terms);
    RUN_TEST(pid_step_saturates_output);
    RUN_TEST(pid_step_limits_output_rate);
    RUN_TEST(pid_reset_clears_runtime_state);
    return TEST_REPORT();
}
