#include "MotorActuator.h"

const MotorActuator_Config MotorActuator_DefaultConfig = {
    100.0f,
    -100.0f,
    0.5f,
    13.0f,
    13.0f,
    1.0f,
    1.0f,
    99.0f
};

static float MotorActuator_Clamp(float value, float min_value, float max_value)
{
    if (value > max_value)
    {
        return max_value;
    }

    if (value < min_value)
    {
        return min_value;
    }

    return value;
}

static uint32_t MotorActuator_CommandToCompare(float command,
                                               float max_command,
                                               float max_duty,
                                               float deadband,
                                               float scale)
{
    float compare;
    float pwm_span;

    if ((command <= 0.0f) || (max_command <= 0.0f) || (max_duty <= 0.0f))
    {
        return 0U;
    }

    if (command > max_command)
    {
        command = max_command;
    }

    pwm_span = max_duty - deadband;
    if (pwm_span < 0.0f)
    {
        pwm_span = 0.0f;
    }

    compare = deadband + ((command / max_command) * pwm_span);
    compare *= scale;

    if (compare > max_duty)
    {
        compare = max_duty;
    }

    if (compare < 0.0f)
    {
        compare = 0.0f;
    }

    return (uint32_t)(compare + 0.5f);
}

#ifdef UNIT_TEST
uint32_t MotorActuator_Test_CommandToCompare(float command,
                                             float max_command,
                                             float max_duty,
                                             float deadband,
                                             float scale)
{
    return MotorActuator_CommandToCompare(command, max_command, max_duty, deadband, scale);
}
#endif

MotorActuator_Output MotorActuator_Calculate(float command, const MotorActuator_Config *config)
{
    const MotorActuator_Config *active_config = config;
    MotorActuator_Output output = {
        1U,
        MOTOR_ACTUATOR_DIRECTION_STOP,
        MOTOR_ACTUATOR_DIRECTION_STOP,
        0U,
        0U
    };
    float duty_command;

    if (active_config == 0)
    {
        active_config = &MotorActuator_DefaultConfig;
    }

    command = MotorActuator_Clamp(command, active_config->min_command, active_config->max_command);

    if (command > active_config->zero_epsilon)
    {
        output.left_direction = MOTOR_ACTUATOR_DIRECTION_FORWARD;
        output.right_direction = MOTOR_ACTUATOR_DIRECTION_FORWARD;
        duty_command = command;
    }
    else if (command < -active_config->zero_epsilon)
    {
        output.left_direction = MOTOR_ACTUATOR_DIRECTION_REVERSE;
        output.right_direction = MOTOR_ACTUATOR_DIRECTION_REVERSE;
        duty_command = -command;
    }
    else
    {
        duty_command = 0.0f;
    }

    output.left_compare = MotorActuator_CommandToCompare(duty_command,
                                                         active_config->max_command,
                                                         active_config->max_duty,
                                                         active_config->left_deadband_duty,
                                                         active_config->left_scale);
    output.right_compare = MotorActuator_CommandToCompare(duty_command,
                                                          active_config->max_command,
                                                          active_config->max_duty,
                                                          active_config->right_deadband_duty,
                                                          active_config->right_scale);

    return output;
}
