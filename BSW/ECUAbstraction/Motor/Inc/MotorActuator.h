#ifndef MOTOR_ACTUATOR_H_
#define MOTOR_ACTUATOR_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    MOTOR_ACTUATOR_DIRECTION_STOP = 0,
    MOTOR_ACTUATOR_DIRECTION_FORWARD,
    MOTOR_ACTUATOR_DIRECTION_REVERSE
} MotorActuator_Direction;

typedef struct
{
    float max_command;
    float min_command;
    float zero_epsilon;
    float left_deadband_duty;
    float right_deadband_duty;
    float left_scale;
    float right_scale;
    float max_duty;
} MotorActuator_Config;

typedef struct
{
    uint8_t standby_enable;
    MotorActuator_Direction left_direction;
    MotorActuator_Direction right_direction;
    uint32_t left_compare;
    uint32_t right_compare;
} MotorActuator_Output;

extern const MotorActuator_Config MotorActuator_DefaultConfig;

MotorActuator_Output MotorActuator_Calculate(float command, const MotorActuator_Config *config);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_ACTUATOR_H_ */
