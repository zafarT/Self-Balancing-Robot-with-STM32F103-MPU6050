/*
 * mpu6050.h
 *
 *  Created on: Sep 10, 2025
 *      Author: zafar
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include "stm32f1xx_hal.h"


extern float MPU6050_DATA[7];

extern  uint8_t MPU_I2C_Address	;
extern  uint8_t MPU_PowerUpAddress;
extern  uint8_t AccelConfigAdd 	;
extern  uint8_t GyroConfigAdd 	;
extern  uint8_t DataRegStart	;
extern  uint16_t AccelLSBSens	;
extern  double GyrolLSBSens		;
extern uint8_t mpu6050_data[14];
extern volatile uint8_t MPU_I2C_Ready;
extern volatile uint8_t MPU_Data_Ready;
extern float roll, pitch;
extern float MPU6050_Dt;
extern float MPU6050_Sample_Hz;
extern uint32_t MPU6050_Sample_Count;



uint8_t MPU6050_Init(I2C_HandleTypeDef *i2c);
uint8_t MPU6050_AccelerometerConfig(uint8_t AccRegValue);
uint8_t MPU6050_GyroConfig(uint8_t GRegValue);
int16_t bytes_to_int(uint8_t firstbyte, uint8_t secondbyte);
void MPU6050_Read(void);
void MPU6050_Mark_Data_Ready(void);
void MPU6050_I2C_Error(void);
uint8_t MPU6050_Process_Data_Ready(void);
void MPU6050_Data_Update(void);
float MPU6050_Get_Dt(void);
void MPU6050_Set_Display_Enabled(uint8_t enabled);
void MPU6050_Display_Update(void);
void DWT_Init(void);
#endif /* INC_MPU6050_H_ */
