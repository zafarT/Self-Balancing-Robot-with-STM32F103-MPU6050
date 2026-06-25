#ifndef TEST_MAIN_H_
#define TEST_MAIN_H_

#include "stm32f1xx_hal.h"

void Error_Handler(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

#endif /* TEST_MAIN_H_ */
