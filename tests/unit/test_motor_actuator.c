#include "MotorActuator.h"
#include "test_framework.h"

static MotorActuator_Config test_config(void)
{
    MotorActuator_Config config = MotorActuator_DefaultConfig;
    config.max_duty = 99.0f;
    return config;
}

static void motor_actuator_maps_positive_command_to_forward_pwm(void)
{
    const MotorActuator_Config config = test_config();
    const MotorActuator_Output output = MotorActuator_Calculate(50.0f, &config);

    TEST_ASSERT_UINT_EQ(1U, output.standby_enable);
    TEST_ASSERT_INT_EQ(MOTOR_ACTUATOR_DIRECTION_FORWARD, output.left_direction);
    TEST_ASSERT_INT_EQ(MOTOR_ACTUATOR_DIRECTION_FORWARD, output.right_direction);
    TEST_ASSERT_UINT_EQ(56U, output.left_compare);
    TEST_ASSERT_UINT_EQ(56U, output.right_compare);
}

static void motor_actuator_maps_negative_command_to_reverse_pwm(void)
{
    const MotorActuator_Config config = test_config();
    const MotorActuator_Output output = MotorActuator_Calculate(-25.0f, &config);

    TEST_ASSERT_INT_EQ(MOTOR_ACTUATOR_DIRECTION_REVERSE, output.left_direction);
    TEST_ASSERT_INT_EQ(MOTOR_ACTUATOR_DIRECTION_REVERSE, output.right_direction);
    TEST_ASSERT_UINT_EQ(35U, output.left_compare);
    TEST_ASSERT_UINT_EQ(35U, output.right_compare);
}

static void motor_actuator_stops_inside_zero_deadband(void)
{
    const MotorActuator_Config config = test_config();
    const MotorActuator_Output output = MotorActuator_Calculate(0.4f, &config);

    TEST_ASSERT_INT_EQ(MOTOR_ACTUATOR_DIRECTION_STOP, output.left_direction);
    TEST_ASSERT_INT_EQ(MOTOR_ACTUATOR_DIRECTION_STOP, output.right_direction);
    TEST_ASSERT_UINT_EQ(0U, output.left_compare);
    TEST_ASSERT_UINT_EQ(0U, output.right_compare);
}

static void motor_actuator_saturates_to_timer_period(void)
{
    const MotorActuator_Config config = test_config();
    const MotorActuator_Output output = MotorActuator_Calculate(250.0f, &config);

    TEST_ASSERT_UINT_EQ(99U, output.left_compare);
    TEST_ASSERT_UINT_EQ(99U, output.right_compare);
}

int main(void)
{
    RUN_TEST(motor_actuator_maps_positive_command_to_forward_pwm);
    RUN_TEST(motor_actuator_maps_negative_command_to_reverse_pwm);
    RUN_TEST(motor_actuator_stops_inside_zero_deadband);
    RUN_TEST(motor_actuator_saturates_to_timer_period);
    return TEST_REPORT();
}
