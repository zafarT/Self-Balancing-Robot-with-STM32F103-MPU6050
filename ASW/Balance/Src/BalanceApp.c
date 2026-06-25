#include "BalanceApp.h"
#include "BalanceControl.h"
#include "MotorDriver.h"
#include "mpu6050.h"
#include "ssd1306.h"

static I2C_HandleTypeDef *app_imu_i2c = 0;
static TIM_HandleTypeDef *app_control_timer = 0;
static BalanceControl balance_control;
static const BalanceControl_Config balance_control_config = {
    -2.5f,
    40.0f,
    {
        10.0f,
        0.0f,
        1.0f,
        0.0f,
        0.02f,
        0.001f,
        100.0f,
        -100.0f,
        5000.0f
    }
};

uint8_t BalanceApp_Init(I2C_HandleTypeDef *imu_i2c, TIM_HandleTypeDef *control_timer)
{
    if ((imu_i2c == 0) || (control_timer == 0))
    {
        return 1U;
    }

    app_imu_i2c = imu_i2c;
    app_control_timer = control_timer;

    MotorDriver_Init(control_timer);
    MotorDriver_Stop();

    if (MPU6050_Init(imu_i2c) != 0U)
    {
        return 1U;
    }

    if (MPU6050_AccelerometerConfig(0x18U) != 0U)
    {
        return 1U;
    }

    if (MPU6050_GyroConfig(0x18U) != 0U)
    {
        return 1U;
    }

    if (SSD1306_Init() != 0U)
    {
        MPU6050_Set_Display_Enabled(1U);
    }

    BalanceControl_Init(&balance_control, &balance_control_config);
    MotorDriver_Stop();

    if (HAL_TIM_PWM_Start(control_timer, TIM_CHANNEL_1) != HAL_OK)
    {
        return 1U;
    }

    if (HAL_TIM_PWM_Start(control_timer, TIM_CHANNEL_2) != HAL_OK)
    {
        return 1U;
    }

    if (HAL_TIM_Base_Start_IT(control_timer) != HAL_OK)
    {
        return 1U;
    }

    return 0U;
}

#ifdef UNIT_TEST
void BalanceApp_Test_Reset(void)
{
    app_imu_i2c = 0;
    app_control_timer = 0;
    BalanceControl_Init(&balance_control, &balance_control_config);
}
#endif

void BalanceApp_MainFunction(void)
{
    if (MPU6050_Process_Data_Ready() != 0U)
    {
        const BalanceControl_Output output = BalanceControl_Update(&balance_control,
                                                                   &balance_control_config,
                                                                   pitch,
                                                                   MPU6050_Get_Dt());

        if (output.motor_enable != 0U)
        {
            MotorDriver_Apply(output.motor_command);
        }
        else
        {
            MotorDriver_Stop();
        }
    }

    MPU6050_Display_Update();
}

void BalanceApp_TimerElapsedCallback(TIM_HandleTypeDef *htim)
{
    if ((app_control_timer != 0) && (htim != 0) && (htim->Instance == app_control_timer->Instance))
    {
        MPU6050_Read();
    }
}

void BalanceApp_I2cMemRxCompleteCallback(I2C_HandleTypeDef *hi2c)
{
    if ((app_imu_i2c != 0) && (hi2c != 0) && (hi2c->Instance == app_imu_i2c->Instance))
    {
        MPU6050_Mark_Data_Ready();
    }
}

void BalanceApp_I2cMasterTxCompleteCallback(I2C_HandleTypeDef *hi2c)
{
    if ((app_imu_i2c != 0) && (hi2c != 0) && (hi2c->Instance == app_imu_i2c->Instance))
    {
        oled_ready = 1U;
    }
}

void BalanceApp_I2cErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if ((app_imu_i2c != 0) && (hi2c != 0) && (hi2c->Instance == app_imu_i2c->Instance))
    {
        oled_ready = 1U;
        MPU6050_I2C_Error();
    }
}
