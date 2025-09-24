/*
 * mpu6050.h
 *
 *  Created on: Sep 10, 2025
 *      Author: zafar
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_



extern float MPU6050_DATA[7];

extern  uint8_t MPU_I2C_Address	;
extern  uint8_t MPU_PowerUp	 	;
extern  uint8_t AccelConfigAdd 	;
extern  uint8_t GyroConfigAdd 	;
extern  uint8_t DataRegStart	;
extern  uint16_t AccelLSBSens	;
extern  double GyrolLSBSens		;
extern uint8_t mpu6050_data[14];
extern uint8_t MPU_I2C_Ready;
extern float roll, pitch;



uint8_t MPU6050_Init(I2C_HandleTypeDef *i2c);
uint8_t MPU6050_AccelerometerConfig(uint8_t AccRegValue);
uint8_t MPU6050_GyroConfig(uint8_t GRegValue);
int16_t bytes_to_int(uint8_t firstbyte, uint8_t secondbyte);
void MPU6050_Read(void);
void MPU6050_Data_Update(void);
void DWT_Init(void);
#endif /* INC_MPU6050_H_ */
