/*
 * mpu6050.c
 *
 *  Created on: Sep 10, 2025
 *      Author: zafar
 */

#include "main.h"
#include "mpu6050.h"
#include "ssd1306.h"
#include "ImuProcessing.h"
#include <stdio.h>
#include <string.h>

uint8_t MPU_I2C_Address		= 0xD0;
uint8_t MPU_PowerUpAddress	= 0x6B;
uint8_t AccelConfigAdd 		= 0x1C;
uint8_t GyroConfigAdd 		= 0x1B;
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
volatile uint8_t MPU_I2C_Ready = 1;
volatile uint8_t MPU_Data_Ready = 0;
static uint8_t display_enabled = 0;
static ImuProcessing_State imu_state;
static ImuProcessing_Config imu_config = {
	{
		0.9973217f, -0.0090093f,
		0.9980508f, 0.0058476f,
		1.0061410f, 0.1957754f
	},
	{
		0.1220f,
		0.060976f,
		-0.060976f
	},
	16384.0f,
	131.0f,
	0.995f,
	IMU_PROCESSING_FUSION_COMPLEMENTARY
};

float roll = 0, pitch = 0;
float MPU6050_Dt = 0.001f;
float MPU6050_Sample_Hz = 0.0f;
uint32_t MPU6050_Sample_Count = 0;
static uint32_t mpu6050_last_ticks = 0;
static uint8_t mpu6050_has_last_ticks = 0;
static uint32_t mpu6050_last_display_ticks = 0;

#define MPU6050_REG_SMPLRT_DIV       0x19
#define MPU6050_REG_CONFIG           0x1A
#define MPU6050_DISPLAY_PERIOD_MS    1000U
#define MPU6050_DT_DEFAULT           0.001f
#define MPU6050_DT_MAX               0.050f
#define MPU6050_SMPLRT_DIV_VALUE     0x00
#define MPU6050_DLPF_CFG_VALUE       0x03

static void MPU6050_SyncProcessingConfig(void)
{
	memcpy(imu_config.accel_calibration, CalibrationParameterAccelerometer, sizeof(imu_config.accel_calibration));
	memcpy(imu_config.gyro_bias, CalibratedGyro, sizeof(imu_config.gyro_bias));
	imu_config.accel_lsb_sensitivity = (float)AccelLSBSens;
	imu_config.gyro_lsb_sensitivity = (float)GyrolLSBSens;
}

static uint8_t MPU6050_WriteRegister(uint8_t reg, uint8_t value)
{
	if (i2c_config == NULL)
	{
		return 1;
	}

	return (HAL_I2C_Mem_Write(i2c_config, MPU_I2C_Address, reg, I2C_MEMADD_SIZE_8BIT, &value, sizeof(value), 100) == HAL_OK) ? 0 : 1;
}

#ifdef UNIT_TEST
void MPU6050_Test_Reset(void)
{
	i2c_config = NULL;
	MPU_I2C_Ready = 1U;
	MPU_Data_Ready = 0U;
	display_enabled = 0U;
	memset(mpu6050_data, 0, sizeof(mpu6050_data));
	memset(MPU6050_DATA, 0, sizeof(MPU6050_DATA));
	roll = 0.0f;
	pitch = 0.0f;
	MPU6050_Dt = MPU6050_DT_DEFAULT;
	MPU6050_Sample_Hz = 0.0f;
	MPU6050_Sample_Count = 0U;
	mpu6050_last_ticks = 0U;
	mpu6050_has_last_ticks = 0U;
	mpu6050_last_display_ticks = 0U;
	MPU6050_SyncProcessingConfig();
	ImuProcessing_Init(&imu_state, &imu_config);
}

void MPU6050_Test_SetI2c(I2C_HandleTypeDef *i2c)
{
	i2c_config = i2c;
}

uint8_t MPU6050_Test_WriteRegister(uint8_t reg, uint8_t value)
{
	return MPU6050_WriteRegister(reg, value);
}
#endif

uint8_t MPU6050_Init(I2C_HandleTypeDef *i2c)
{
	if (i2c == NULL)
	{
		return 1;
	}

	MPU6050_SyncProcessingConfig();
	ImuProcessing_Init(&imu_state, &imu_config);
	DWT_Init();
	i2c_config = i2c;
	uint8_t PowerRegValue = 0x01;
	HAL_StatusTypeDef retInit = HAL_I2C_IsDeviceReady(i2c_config, MPU_I2C_Address, 1, 100);
	HAL_StatusTypeDef retPower = HAL_I2C_Mem_Write(i2c_config, MPU_I2C_Address, MPU_PowerUpAddress, I2C_MEMADD_SIZE_8BIT, &PowerRegValue, sizeof(PowerRegValue), 100);
	if (retPower == HAL_OK)
	{
		HAL_Delay(10);
	}
	uint8_t retSampleRate = MPU6050_WriteRegister(MPU6050_REG_SMPLRT_DIV, MPU6050_SMPLRT_DIV_VALUE);
	uint8_t retFilter = MPU6050_WriteRegister(MPU6050_REG_CONFIG, MPU6050_DLPF_CFG_VALUE);
	if(retInit == HAL_OK && retPower == HAL_OK && retSampleRate == 0 && retFilter == 0)
	{
		return 0;
	}
	else
	{
		i2c_config = NULL;
		return 1;
	}
}

uint8_t MPU6050_AccelerometerConfig(uint8_t AccRegValue)
{
	if (i2c_config == NULL)
	{
		return 1;
	}

	HAL_StatusTypeDef retA = HAL_I2C_Mem_Write(i2c_config, MPU_I2C_Address, AccelConfigAdd, I2C_MEMADD_SIZE_8BIT, &AccRegValue, sizeof(AccRegValue), 100);
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
			MPU6050_SyncProcessingConfig();
			return 0;
	}
	else
	{
		return 1;
	}
}

uint8_t MPU6050_GyroConfig(uint8_t GRegValue)
{
	if (i2c_config == NULL)
	{
		return 1;
	}

	HAL_StatusTypeDef retG = HAL_I2C_Mem_Write(i2c_config, MPU_I2C_Address, GyroConfigAdd, I2C_MEMADD_SIZE_8BIT, &GRegValue, sizeof(GRegValue), 100);
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
			MPU6050_SyncProcessingConfig();
			return 0;
	}
	else
	{
		return 1;
	}
}

int16_t bytes_to_int(uint8_t firstbyte, uint8_t secondbyte)
{
	return ImuProcessing_BytesToInt(firstbyte, secondbyte);
}


void MPU6050_Read(void)
{
	if (i2c_config == NULL || !MPU_I2C_Ready || HAL_I2C_GetState(i2c_config) != HAL_I2C_STATE_READY)
	{
		return;
	}

	if (HAL_I2C_Mem_Read_DMA(i2c_config, MPU_I2C_Address, DataRegStart, I2C_MEMADD_SIZE_8BIT, mpu6050_data, sizeof(mpu6050_data)) == HAL_OK)
	{
		MPU_I2C_Ready = 0;
	}
}

void MPU6050_Mark_Data_Ready(void)
{
	MPU_Data_Ready = 1;
}

void MPU6050_I2C_Error(void)
{
	MPU_Data_Ready = 0;
	MPU_I2C_Ready = 1;
}

uint8_t MPU6050_Process_Data_Ready(void)
{
	if (!MPU_Data_Ready)
	{
		return 0;
	}

	MPU_Data_Ready = 0;
	MPU6050_Data_Update();
	MPU_I2C_Ready = 1;
	return 1;
}


void MPU6050_Data_Update(void)
{
	uint32_t now = DWT->CYCCNT;
	if (!mpu6050_has_last_ticks)
	{
		mpu6050_last_ticks = now;
		mpu6050_has_last_ticks = 1;
		MPU6050_Dt = MPU6050_DT_DEFAULT;
	}
	else
	{
		uint32_t cycles = now - mpu6050_last_ticks;
		mpu6050_last_ticks = now;
		MPU6050_Dt = (float)cycles / (float)SystemCoreClock;

		if (MPU6050_Dt <= 0.0f || MPU6050_Dt > MPU6050_DT_MAX)
		{
			MPU6050_Dt = MPU6050_DT_DEFAULT;
		}
	}

	MPU6050_SyncProcessingConfig();
	ImuProcessing_Update(&imu_state, &imu_config, mpu6050_data, MPU6050_Dt);
	memcpy(MPU6050_DATA, imu_state.data, sizeof(MPU6050_DATA));
	roll = imu_state.roll_deg;
	pitch = imu_state.pitch_deg;
	MPU6050_Dt = imu_state.dt_s;
	MPU6050_Sample_Hz = imu_state.sample_hz;
	MPU6050_Sample_Count = imu_state.sample_count;
}

float MPU6050_Get_Dt(void)
{
	return MPU6050_Dt;
}

void MPU6050_Set_Display_Enabled(uint8_t enabled)
{
	display_enabled = enabled;
}

void MPU6050_Display_Update(void)
{
	uint32_t now = HAL_GetTick();

	if (!display_enabled || !oled_ready || !MPU_I2C_Ready || i2c_config == NULL)
	{
		return;
	}

	if ((uint32_t)(now - mpu6050_last_display_ticks) < MPU6050_DISPLAY_PERIOD_MS)
	{
		return;
	}

	if (HAL_I2C_GetState(i2c_config) != HAL_I2C_STATE_READY)
	{
		return;
	}

	mpu6050_last_display_ticks = now;
	char stringvalues[20];
	SSD1306_Fill(SSD1306_COLOR_BLACK);
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
	SSD1306_GotoXY(0, 54);
	sprintf(stringvalues, "Hz:%4.0f", MPU6050_Sample_Hz);
	SSD1306_Puts(stringvalues, &Font_7x10, 1);
	SSD1306_UpdateScreen_DMA();
}



void DWT_Init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // Enable DWT
    DWT->CYCCNT = 0;                                // Reset counter
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;            // Enable cycle counter
}
