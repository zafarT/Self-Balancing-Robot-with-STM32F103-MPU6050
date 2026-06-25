#ifndef MOTOR_DRIVER_H_
#define MOTOR_DRIVER_H_

#include "stm32f1xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

void MotorDriver_Init(TIM_HandleTypeDef *pwm_timer);
void MotorDriver_Apply(float command);
void MotorDriver_Stop(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_DRIVER_H_ */
