/*
 * mpu6050.c
 *
 *  Created on: Sep 10, 2025
 *      Author: zafar
 */

#include "main.h"
#include "mpu6050.h"
#include "ssd1306.h"
#include "kalman_filter.h"

uint8_t MPU_I2C_Address		= 0xD0;
uint8_t MPU_PowerUpAddress	= 0x6B;
uint8_t AccelConfigAdd 		= 0x1B;
uint8_t GyroConfigAdd 		= 0x1C;
uint8_t DataRegStart		= 0x3B;
uint16_t AccelLSBSens		= 16384;
double GyrolLSBSens			= 131;
uint8_t mpu6050_data[14];
float MPU6050_DATA[7];
float average_values[7];
float CalibrationParameterAccelerometer[6] = {0.99732169259, -0.00900930551, 0.99805080677, 0.0058475796768, 1.00614098148, 0.1957754152469};
uint32_t counter_average = 1;
float CalibratedGyro[3] = {0.1220, 0.060976, -0.060976};
static I2C_HandleTypeDef *i2c_config = NULL;
uint8_t MPU_I2C_Ready = 1;
static KalmanFilter kalmanRoll, kalmanPitch;

float roll = 0, pitch = 0;

uint8_t MPU6050_Init(I2C_HandleTypeDef *i2c)
{
	Kalman_Init(&kalmanRoll);
	Kalman_Init(&kalmanPitch);
	DWT_Init();
	i2c_config = i2c;
	uint8_t PowerRegValue = 0x00;
	HAL_StatusTypeDef retInit = HAL_I2C_IsDeviceReady(i2c_config, MPU_I2C_Address, 1, 100);
	HAL_StatusTypeDef retPower = HAL_I2C_Mem_Write(i2c_config, MPU_I2C_Address, MPU_PowerUpAddress, I2C_MEMADD_SIZE_8BIT, &PowerRegValue, I2C_MEMADD_SIZE_8BIT, 100);
	if(retInit == HAL_OK && retPower == HAL_OK)
	{
		return 0;
	}
	else
	{
		return 1;
	}




}

uint8_t MPU6050_AccelerometerConfig(uint8_t AccRegValue)
{
	HAL_StatusTypeDef retA = HAL_I2C_Mem_Write(i2c_config, MPU_I2C_Address, AccelConfigAdd, I2C_MEMADD_SIZE_8BIT, &AccRegValue, I2C_MEMADD_SIZE_8BIT, 100);
	if(retA == HAL_OK)
	{
		if((AccRegValue & 0x18) == 0x18)
		{
			AccelLSBSens = 2048;
		}
		else if((AccRegValue & 0x18) == 0x10)
		{
			AccelLSBSens = 4096;
		}
		else if((AccRegValue & 0x18) == 0x08)
		{
			AccelLSBSens = 8192;
		}
		return 0;
	}
	else
	{
		return 1;
	}
}

uint8_t MPU6050_GyroConfig(uint8_t GRegValue)
{
	HAL_StatusTypeDef retG = HAL_I2C_Mem_Write(i2c_config, MPU_I2C_Address, GyroConfigAdd, I2C_MEMADD_SIZE_8BIT, &GRegValue, I2C_MEMADD_SIZE_8BIT, 100);
	if(retG == HAL_OK)
	{
		if((GRegValue & 0x18) == 0x18)
		{
			GyrolLSBSens = 16.4;
		}
		else if((GRegValue & 0x18) == 0x10)
		{
			GyrolLSBSens = 32.8;
		}
		else if((GRegValue & 0x18) == 0x08)
		{
			GyrolLSBSens = 65.5;
		}
		return 0;
	}
	else
	{
		return 1;
	}
}

int16_t bytes_to_int(uint8_t firstbyte, uint8_t secondbyte)
{
	int16_t val = (int16_t)((firstbyte << 8) | secondbyte);
}


void MPU6050_Read(void)
{
	if (MPU_I2C_Ready && oled_ready)
	{
		MPU_I2C_Ready = 0;
		HAL_I2C_Mem_Read_DMA(i2c_config, MPU_I2C_Address, DataRegStart, I2C_MEMADD_SIZE_8BIT,  mpu6050_data, sizeof(mpu6050_data));
	}

}


void MPU6050_Data_Update(void)
{
	double LSB = AccelLSBSens;
	for(int i=0; i < 7; i++)
	{
		if(i == 3)
		{
			MPU6050_DATA[i] = (bytes_to_int(mpu6050_data[i*2], mpu6050_data[i*2+1]))/340 + 36.53;
			LSB = GyrolLSBSens;
		}
		else
		{
			if(i > 3)
				MPU6050_DATA[i] = (float)bytes_to_int(mpu6050_data[i*2], mpu6050_data[i*2+1])/LSB - CalibratedGyro[i-4];
			else
			{
				float temp = (float)bytes_to_int(mpu6050_data[i*2], mpu6050_data[i*2+1])/LSB;
				MPU6050_DATA[i] = CalibrationParameterAccelerometer[i*2]*temp + CalibrationParameterAccelerometer[i*2+1];
			}

			//average_values[i] = (MPU6050_DATA[i] + average_values[i]*(counter_average - 1))/counter_average;
			//printf("DataAvr[%d]: %.6f\t", i, average_values[i]);
		}
	}
	static uint32_t lastTicks = 0;
	uint32_t now = DWT->CYCCNT;
	uint32_t cycles = now - lastTicks;
	lastTicks = now;

	float dt = (float)cycles / (float)SystemCoreClock; // seconds

	float rollAcc = getRoll(MPU6050_DATA[1], MPU6050_DATA[2]);
	float pitchAcc = getPitch(MPU6050_DATA[0], MPU6050_DATA[1], MPU6050_DATA[2]);

	roll = Kalman_Update(&kalmanRoll, rollAcc, MPU6050_DATA[4], dt);
	pitch = Kalman_Update(&kalmanPitch, pitchAcc, MPU6050_DATA[5],  dt);
//	if (oled_ready && MPU_I2C_Ready)
//	{
//		SSD1306_Clear();
//	}

	char stringvalues[20];
	SSD1306_GotoXY(0, 0);
	SSD1306_Puts("Roll: ", &Font_7x10, 1);
	SSD1306_GotoXY(40, 0);
	sprintf(stringvalues, "%.3f", roll);
	SSD1306_Puts(stringvalues, &Font_7x10, 1);
	SSD1306_GotoXY(0, 30);
	SSD1306_Puts("Pitch : ", &Font_7x10, 1);
	SSD1306_GotoXY(40, 30);
	sprintf(stringvalues, "%.3f", pitch);
	SSD1306_Puts(stringvalues, &Font_7x10, 1);
	if (oled_ready && MPU_I2C_Ready)
	{
		SSD1306_UpdateScreen_DMA();
	}


//	printf("Roll_Acc: %.6f\t\tPitch_Acc: %.6f\t", roll, pitch);
//	printf("\n\n");
}



void DWT_Init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // Enable DWT
    DWT->CYCCNT = 0;                                // Reset counter
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;            // Enable cycle counter
}

