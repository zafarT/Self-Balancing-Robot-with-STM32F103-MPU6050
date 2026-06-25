#ifndef BALANCE_APP_H_
#define BALANCE_APP_H_

#include <stdint.h>
#include "stm32f1xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

uint8_t BalanceApp_Init(I2C_HandleTypeDef *imu_i2c, TIM_HandleTypeDef *control_timer);
void BalanceApp_MainFunction(void);
void BalanceApp_TimerElapsedCallback(TIM_HandleTypeDef *htim);
void BalanceApp_I2cMemRxCompleteCallback(I2C_HandleTypeDef *hi2c);
void BalanceApp_I2cMasterTxCompleteCallback(I2C_HandleTypeDef *hi2c);
void BalanceApp_I2cErrorCallback(I2C_HandleTypeDef *hi2c);

#ifdef __cplusplus
}
#endif

#endif /* BALANCE_APP_H_ */
