#include "MotorDriver.h"
#include "MotorActuator.h"

#define MOTOR_DRIVER_PORT            GPIOA
#define MOTOR_A_IN1_PIN              GPIO_PIN_1
#define MOTOR_A_IN2_PIN              GPIO_PIN_2
#define MOTOR_STBY_PIN               GPIO_PIN_3
#define MOTOR_B_IN1_PIN              GPIO_PIN_5
#define MOTOR_B_IN2_PIN              GPIO_PIN_4
#define MOTOR_LEFT_PWM_CHANNEL       TIM_CHANNEL_1
#define MOTOR_RIGHT_PWM_CHANNEL      TIM_CHANNEL_2
#define MOTOR_B_REVERSED             1

static TIM_HandleTypeDef *motor_pwm_timer = 0;

static void MotorDriver_Enable(void)
{
    HAL_GPIO_WritePin(MOTOR_DRIVER_PORT, MOTOR_STBY_PIN, GPIO_PIN_SET);
}

static void MotorDriver_SetPins(uint16_t in_1_pin, uint16_t in_2_pin, MotorActuator_Direction direction)
{
    GPIO_PinState in_1 = GPIO_PIN_RESET;
    GPIO_PinState in_2 = GPIO_PIN_RESET;

    if (direction == MOTOR_ACTUATOR_DIRECTION_FORWARD)
    {
        in_1 = GPIO_PIN_SET;
        in_2 = GPIO_PIN_RESET;
    }
    else if (direction == MOTOR_ACTUATOR_DIRECTION_REVERSE)
    {
        in_1 = GPIO_PIN_RESET;
        in_2 = GPIO_PIN_SET;
    }

    HAL_GPIO_WritePin(MOTOR_DRIVER_PORT, in_1_pin, in_1);
    HAL_GPIO_WritePin(MOTOR_DRIVER_PORT, in_2_pin, in_2);
}

static void MotorDriver_SetDirection(MotorActuator_Direction left_direction,
                                     MotorActuator_Direction right_direction)
{
    MotorDriver_SetPins(MOTOR_A_IN1_PIN, MOTOR_A_IN2_PIN, left_direction);

#if MOTOR_B_REVERSED
    if (right_direction == MOTOR_ACTUATOR_DIRECTION_FORWARD)
    {
        right_direction = MOTOR_ACTUATOR_DIRECTION_REVERSE;
    }
    else if (right_direction == MOTOR_ACTUATOR_DIRECTION_REVERSE)
    {
        right_direction = MOTOR_ACTUATOR_DIRECTION_FORWARD;
    }
#endif
    MotorDriver_SetPins(MOTOR_B_IN1_PIN, MOTOR_B_IN2_PIN, right_direction);
}

static void MotorDriver_SetPwm(uint32_t left_compare, uint32_t right_compare)
{
    if (motor_pwm_timer == 0)
    {
        return;
    }

    __HAL_TIM_SET_COMPARE(motor_pwm_timer, MOTOR_LEFT_PWM_CHANNEL, left_compare);
    __HAL_TIM_SET_COMPARE(motor_pwm_timer, MOTOR_RIGHT_PWM_CHANNEL, right_compare);
}

void MotorDriver_Init(TIM_HandleTypeDef *pwm_timer)
{
    motor_pwm_timer = pwm_timer;
}

#ifdef UNIT_TEST
void MotorDriver_Test_Reset(void)
{
    motor_pwm_timer = 0;
}

void MotorDriver_Test_SetPwm(uint32_t left_compare, uint32_t right_compare)
{
    MotorDriver_SetPwm(left_compare, right_compare);
}
#endif

void MotorDriver_Apply(float command)
{
    MotorActuator_Config config = MotorActuator_DefaultConfig;
    MotorActuator_Output output;

    if (motor_pwm_timer == 0)
    {
        return;
    }

    config.max_duty = (float)__HAL_TIM_GET_AUTORELOAD(motor_pwm_timer);
    if (config.max_duty <= 0.0f)
    {
        config.max_duty = MotorActuator_DefaultConfig.max_command;
    }

    output = MotorActuator_Calculate(command, &config);
    (void)output.standby_enable;
    MotorDriver_Enable();
    MotorDriver_SetDirection(output.left_direction, output.right_direction);
    MotorDriver_SetPwm(output.left_compare, output.right_compare);
}

void MotorDriver_Stop(void)
{
    MotorDriver_Apply(0.0f);
}
