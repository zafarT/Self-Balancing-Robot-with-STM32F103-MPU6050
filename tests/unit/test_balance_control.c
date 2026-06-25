#include "BalanceControl.h"
#include "test_framework.h"

static BalanceControl_Config test_config(void)
{
    BalanceControl_Config config = BalanceControl_DefaultConfig;

    config.pid.Kp = 10.0f;
    config.pid.Ki = 0.0f;
    config.pid.Kd = 0.0f;
    config.pid.max_rate = 0.0f;

    return config;
}

static void balance_control_generates_motor_command_when_upright(void)
{
    BalanceControl control;
    BalanceControl_Config config = test_config();
    BalanceControl_Output output;

    BalanceControl_Init(&control, &config);
    output = BalanceControl_Update(&control, &config, 0.0f, 0.01f);

    TEST_ASSERT_UINT_EQ(1U, output.motor_enable);
    TEST_ASSERT_FLOAT_NEAR(2.5f, output.balance_error_deg, 0.0001f);
    TEST_ASSERT_FLOAT_NEAR(-25.0f, output.motor_command, 0.0001f);
    TEST_ASSERT_INT_EQ(BALANCE_CONTROL_STATE_ACTIVE, output.state);
}

static void balance_control_stops_motor_after_fall(void)
{
    BalanceControl control;
    BalanceControl_Config config = test_config();
    BalanceControl_Output output;

    BalanceControl_Init(&control, &config);
    output = BalanceControl_Update(&control, &config, 100.0f, 0.01f);

    TEST_ASSERT_UINT_EQ(0U, output.motor_enable);
    TEST_ASSERT_FLOAT_NEAR(0.0f, output.motor_command, 0.0001f);
    TEST_ASSERT_INT_EQ(BALANCE_CONTROL_STATE_FALL_DETECTED, output.state);
}

static void balance_control_reset_returns_to_active_state(void)
{
    BalanceControl control;
    BalanceControl_Config config = test_config();

    BalanceControl_Init(&control, &config);
    (void)BalanceControl_Update(&control, &config, 100.0f, 0.01f);
    BalanceControl_Reset(&control);

    TEST_ASSERT_INT_EQ(BALANCE_CONTROL_STATE_ACTIVE, control.state);
}

int main(void)
{
    RUN_TEST(balance_control_generates_motor_command_when_upright);
    RUN_TEST(balance_control_stops_motor_after_fall);
    RUN_TEST(balance_control_reset_returns_to_active_state);
    return TEST_REPORT();
}
