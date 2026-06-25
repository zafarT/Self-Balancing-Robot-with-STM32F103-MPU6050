#include "BalanceApp.h"
#include "BalanceControl.h"
#include "ImuProcessing.h"
#include "MotorActuator.h"
#include "MotorDriver.h"
#include "PidController.h"
#include "fake_stm32.h"
#include "mpu6050.h"
#include "test_framework.h"

#include <string.h>

uint32_t MotorActuator_Test_CommandToCompare(float command,
                                             float max_command,
                                             float max_duty,
                                             float deadband,
                                             float scale);
void MotorDriver_Test_Reset(void);
void MotorDriver_Test_SetPwm(uint32_t left_compare, uint32_t right_compare);
void BalanceApp_Test_Reset(void);
void MPU6050_Test_Reset(void);
void MPU6050_Test_SetI2c(I2C_HandleTypeDef *i2c);
uint8_t MPU6050_Test_WriteRegister(uint8_t reg, uint8_t value);

static int i2c_instance_a;
static int i2c_instance_b;
static int timer_instance_a;
static int timer_instance_b;

static void put_i16(uint8_t *raw, uint32_t index, int16_t value)
{
    raw[index * 2U] = (uint8_t)(((uint16_t)value >> 8U) & 0xFFU);
    raw[(index * 2U) + 1U] = (uint8_t)((uint16_t)value & 0xFFU);
}

static I2C_HandleTypeDef make_i2c(void *instance)
{
    I2C_HandleTypeDef i2c;
    i2c.Instance = instance;
    return i2c;
}

static TIM_HandleTypeDef make_timer(void *instance, uint32_t period)
{
    TIM_HandleTypeDef timer;
    memset(&timer, 0, sizeof(timer));
    timer.Instance = instance;
    timer.Init.Period = period;
    return timer;
}

static PidController_Config pid_config(float kp,
                                       float ki,
                                       float kd,
                                       float kaw,
                                       float filter_time,
                                       float sample_time,
                                       float max,
                                       float min,
                                       float max_rate)
{
    PidController_Config config;
    config.Kp = kp;
    config.Ki = ki;
    config.Kd = kd;
    config.Kaw = kaw;
    config.T_C = filter_time;
    config.T = sample_time;
    config.max = max;
    config.min = min;
    config.max_rate = max_rate;
    return config;
}

static ImuProcessing_Config identity_imu_config(ImuProcessing_FusionMode fusion_mode)
{
    ImuProcessing_Config config = ImuProcessing_DefaultConfig;

    config.accel_calibration[0] = 1.0f;
    config.accel_calibration[1] = 0.0f;
    config.accel_calibration[2] = 1.0f;
    config.accel_calibration[3] = 0.0f;
    config.accel_calibration[4] = 1.0f;
    config.accel_calibration[5] = 0.0f;
    config.gyro_bias[0] = 0.0f;
    config.gyro_bias[1] = 0.0f;
    config.gyro_bias[2] = 0.0f;
    config.accel_lsb_sensitivity = 2048.0f;
    config.gyro_lsb_sensitivity = 16.4f;
    config.complementary_alpha = 0.5f;
    config.fusion_mode = fusion_mode;

    return config;
}

static void set_i2c_write_sequence(const HAL_StatusTypeDef *statuses, uint32_t count)
{
    uint32_t index;

    fake_stm32.i2c_mem_write_sequence_count = count;
    for (index = 0U; index < count; index++)
    {
        fake_stm32.i2c_mem_write_sequence[index] = statuses[index];
    }
}

static void reset_project_state(void)
{
    FakeStm32_Reset();
    MPU6050_Test_Reset();
    MotorDriver_Test_Reset();
    BalanceApp_Test_Reset();
}

static void load_level_frame(void)
{
    memset(mpu6050_data, 0, sizeof(mpu6050_data));
    put_i16(mpu6050_data, 2U, 2048);
}

static void load_pitch_fall_frame(void)
{
    memset(mpu6050_data, 0, sizeof(mpu6050_data));
    put_i16(mpu6050_data, 0U, -2048);
    put_i16(mpu6050_data, 5U, 32767);
}

static void ctfl_white_box_null_pointer_guards(void)
{
    uint8_t raw[IMU_PROCESSING_RAW_SIZE];
    ImuProcessing_State imu_state;
    BalanceControl_Output balance_output;

    memset(raw, 0, sizeof(raw));
    PidController_Init(0, 0);
    PidController_Reset(0);
    TEST_ASSERT_FLOAT_NEAR(0.0f, PidController_Step(0, 1.0f, 0.1f), 0.0001f);

    BalanceControl_Init(0, 0);
    BalanceControl_Reset(0);
    balance_output = BalanceControl_Update(0, 0, 0.0f, 0.1f);
    TEST_ASSERT_UINT_EQ(0U, balance_output.motor_enable);
    TEST_ASSERT_UINT_EQ(BALANCE_CONTROL_STATE_FALL_DETECTED, balance_output.state);

    Kalman_Init(0);
    TEST_ASSERT_FLOAT_NEAR(0.0f, Kalman_Update(0, 1.0f, 1.0f, 0.1f), 0.0001f);

    ImuProcessing_Init(0, 0);
    ImuProcessing_Update(0, 0, raw, 0.001f);
    ImuProcessing_Init(&imu_state, 0);
    ImuProcessing_Update(&imu_state, 0, 0, 0.001f);
}

static void ctfl_pid_equivalence_boundaries_and_branches(void)
{
    PidController controller;
    PidController_Config config;

    PidController_Init(&controller, 0);
    TEST_ASSERT_FLOAT_NEAR(PidController_DefaultConfig.Kp, controller.config.Kp, 0.0001f);

    config = pid_config(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, -1.0f, 0.0f);
    PidController_Init(&controller, &config);
    TEST_ASSERT_FLOAT_NEAR(0.0f, PidController_Step(&controller, 0.0f, 0.0f), 0.0001f);
    TEST_ASSERT_FLOAT_NEAR(PidController_DefaultConfig.T, controller.config.T, 0.0001f);

    config = pid_config(10.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.1f, 5.0f, -5.0f, 0.0f);
    PidController_Init(&controller, &config);
    TEST_ASSERT_FLOAT_NEAR(5.0f, PidController_Step(&controller, -1.0f, 0.1f), 0.0001f);
    PidController_Reset(&controller);
    TEST_ASSERT_FLOAT_NEAR(-5.0f, PidController_Step(&controller, 1.0f, 0.1f), 0.0001f);

    config = pid_config(10.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.1f, 100.0f, -100.0f, 10.0f);
    PidController_Init(&controller, &config);
    TEST_ASSERT_FLOAT_NEAR(1.0f, PidController_Step(&controller, -10.0f, 0.1f), 0.0001f);
    PidController_Reset(&controller);
    TEST_ASSERT_FLOAT_NEAR(-1.0f, PidController_Step(&controller, 10.0f, 0.1f), 0.0001f);
    TEST_ASSERT_FLOAT_NEAR(-1.0f, controller.command_sat_prev, 0.0001f);
}

static void ctfl_balance_control_state_transition_decisions(void)
{
    BalanceControl control;
    BalanceControl_Config config = BalanceControl_DefaultConfig;
    BalanceControl_Output output;

    config.target_angle_deg = 0.0f;
    config.fall_limit_deg = 10.0f;
    config.pid = pid_config(1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.01f, 100.0f, -100.0f, 0.0f);

    BalanceControl_Init(&control, 0);
    output = BalanceControl_Update(&control, 0, BalanceControl_DefaultConfig.target_angle_deg, 0.01f);
    TEST_ASSERT_UINT_EQ(1U, output.motor_enable);

    BalanceControl_Init(&control, &config);
    output = BalanceControl_Update(&control, &config, 2.0f, 0.01f);
    TEST_ASSERT_UINT_EQ(1U, output.motor_enable);
    TEST_ASSERT_UINT_EQ(BALANCE_CONTROL_STATE_ACTIVE, output.state);

    output = BalanceControl_Update(&control, &config, 11.0f, 0.01f);
    TEST_ASSERT_UINT_EQ(0U, output.motor_enable);
    TEST_ASSERT_UINT_EQ(BALANCE_CONTROL_STATE_FALL_DETECTED, output.state);

    BalanceControl_Reset(&control);
    output = BalanceControl_Update(&control, &config, -11.0f, 0.01f);
    TEST_ASSERT_UINT_EQ(0U, output.motor_enable);
    TEST_ASSERT_UINT_EQ(BALANCE_CONTROL_STATE_FALL_DETECTED, output.state);
}

static void ctfl_motor_actuator_partitions_and_boundaries(void)
{
    MotorActuator_Config config = MotorActuator_DefaultConfig;
    MotorActuator_Output output;

    config.max_command = 10.0f;
    config.min_command = -10.0f;
    config.zero_epsilon = 0.5f;
    config.left_deadband_duty = 2.0f;
    config.right_deadband_duty = 3.0f;
    config.left_scale = 1.0f;
    config.right_scale = 1.0f;
    config.max_duty = 20.0f;

    output = MotorActuator_Calculate(1.0f, 0);
    TEST_ASSERT_UINT_EQ(MOTOR_ACTUATOR_DIRECTION_FORWARD, output.left_direction);

    output = MotorActuator_Calculate(4.0f, &config);
    TEST_ASSERT_UINT_EQ(MOTOR_ACTUATOR_DIRECTION_FORWARD, output.left_direction);
    TEST_ASSERT_TRUE(output.left_compare > 0U);

    output = MotorActuator_Calculate(-4.0f, &config);
    TEST_ASSERT_UINT_EQ(MOTOR_ACTUATOR_DIRECTION_REVERSE, output.right_direction);

    output = MotorActuator_Calculate(0.1f, &config);
    TEST_ASSERT_UINT_EQ(MOTOR_ACTUATOR_DIRECTION_STOP, output.left_direction);
    TEST_ASSERT_UINT_EQ(0U, output.left_compare);

    output = MotorActuator_Calculate(99.0f, &config);
    TEST_ASSERT_UINT_EQ(20U, output.left_compare);

    output = MotorActuator_Calculate(-99.0f, &config);
    TEST_ASSERT_UINT_EQ(MOTOR_ACTUATOR_DIRECTION_REVERSE, output.left_direction);

    TEST_ASSERT_UINT_EQ(0U, MotorActuator_Test_CommandToCompare(0.0f, 10.0f, 20.0f, 1.0f, 1.0f));
    TEST_ASSERT_UINT_EQ(0U, MotorActuator_Test_CommandToCompare(1.0f, 0.0f, 20.0f, 1.0f, 1.0f));
    TEST_ASSERT_UINT_EQ(0U, MotorActuator_Test_CommandToCompare(1.0f, 10.0f, 0.0f, 1.0f, 1.0f));
    TEST_ASSERT_UINT_EQ(20U, MotorActuator_Test_CommandToCompare(30.0f, 10.0f, 20.0f, 1.0f, 1.0f));
    TEST_ASSERT_UINT_EQ(20U, MotorActuator_Test_CommandToCompare(1.0f, 10.0f, 20.0f, 25.0f, 1.0f));
    TEST_ASSERT_UINT_EQ(20U, MotorActuator_Test_CommandToCompare(5.0f, 10.0f, 20.0f, 1.0f, 3.0f));
    TEST_ASSERT_UINT_EQ(0U, MotorActuator_Test_CommandToCompare(5.0f, 10.0f, 20.0f, 1.0f, -1.0f));
}

static void ctfl_imu_processing_fusion_modes_and_boundaries(void)
{
    ImuProcessing_State state;
    ImuProcessing_Config config;
    uint8_t raw[IMU_PROCESSING_RAW_SIZE];

    memset(raw, 0, sizeof(raw));
    put_i16(raw, 0U, 2048);
    put_i16(raw, 2U, 2048);
    put_i16(raw, 3U, 340);
    put_i16(raw, 4U, 164);
    put_i16(raw, 5U, -164);

    ImuProcessing_Init(&state, 0);
    ImuProcessing_Update(&state, 0, raw, -0.001f);
    TEST_ASSERT_FLOAT_NEAR(0.001f, state.dt_s, 0.000001f);
    TEST_ASSERT_UINT_EQ(1U, state.sample_count);

    config = identity_imu_config(IMU_PROCESSING_FUSION_ACCEL_ONLY);
    ImuProcessing_Init(&state, &config);
    ImuProcessing_Update(&state, &config, raw, 0.002f);
    TEST_ASSERT_FLOAT_NEAR(500.0f, state.sample_hz, 0.1f);
    TEST_ASSERT_FLOAT_NEAR(37.53f, state.data[3], 0.0001f);

    config = identity_imu_config(IMU_PROCESSING_FUSION_COMPLEMENTARY);
    ImuProcessing_Init(&state, &config);
    ImuProcessing_Update(&state, &config, raw, 0.01f);
    TEST_ASSERT_TRUE(state.pitch_deg < 0.0f);

    config = identity_imu_config(IMU_PROCESSING_FUSION_KALMAN);
    ImuProcessing_Init(&state, &config);
    ImuProcessing_Update(&state, &config, raw, 0.01f);
    TEST_ASSERT_TRUE(isfinite(state.roll_deg));
    TEST_ASSERT_TRUE(isfinite(state.pitch_deg));
}

static void ctfl_kalman_filter_statement_and_branch_coverage(void)
{
    KalmanFilter filter;
    float angle;

    Kalman_Init(&filter);
    angle = Kalman_Update(&filter, 10.0f, 1.0f, 0.02f);
    TEST_ASSERT_TRUE(isfinite(angle));
    TEST_ASSERT_FLOAT_NEAR(90.0f, getRoll(1.0f, 0.0f), 0.001f);
    TEST_ASSERT_FLOAT_NEAR(-90.0f, getPitch(1.0f, 0.0f, 0.0f), 0.001f);
}

static void ctfl_motor_driver_hal_adapter_decision_table(void)
{
    TIM_HandleTypeDef timer = make_timer(&timer_instance_a, 0U);

    reset_project_state();
    MotorDriver_Apply(10.0f);
    TEST_ASSERT_UINT_EQ(0U, fake_stm32.gpio_write_count);

    MotorDriver_Test_SetPwm(1U, 2U);
    TEST_ASSERT_UINT_EQ(0U, fake_stm32.compare_write_calls[TIM_CHANNEL_1]);

    MotorDriver_Init(&timer);
    MotorDriver_Apply(10.0f);
    TEST_ASSERT_TRUE(FakeStm32_CountPinWrites(GPIO_PIN_3, GPIO_PIN_SET) > 0U);
    TEST_ASSERT_TRUE(FakeStm32_CountPinWrites(GPIO_PIN_1, GPIO_PIN_SET) > 0U);
    TEST_ASSERT_TRUE(FakeStm32_CountPinWrites(GPIO_PIN_5, GPIO_PIN_RESET) > 0U);
    TEST_ASSERT_TRUE(fake_stm32.compare_write_calls[TIM_CHANNEL_1] > 0U);

    MotorDriver_Apply(-10.0f);
    TEST_ASSERT_TRUE(FakeStm32_CountPinWrites(GPIO_PIN_2, GPIO_PIN_SET) > 0U);
    TEST_ASSERT_TRUE(FakeStm32_CountPinWrites(GPIO_PIN_5, GPIO_PIN_SET) > 0U);

    MotorDriver_Apply(0.0f);
    TEST_ASSERT_TRUE(FakeStm32_CountPinWrites(GPIO_PIN_1, GPIO_PIN_RESET) > 0U);
    TEST_ASSERT_TRUE(FakeStm32_CountPinWrites(GPIO_PIN_2, GPIO_PIN_RESET) > 0U);

    MotorDriver_Stop();
    TEST_ASSERT_TRUE(fake_stm32.compare_write_calls[TIM_CHANNEL_2] > 0U);
}

static void ctfl_mpu6050_init_config_decision_table(void)
{
    I2C_HandleTypeDef i2c = make_i2c(&i2c_instance_a);
    const HAL_StatusTypeDef ret_power_fails[] = {HAL_ERROR};
    const HAL_StatusTypeDef ret_sample_fails[] = {HAL_OK, HAL_ERROR};
    const HAL_StatusTypeDef ret_filter_fails[] = {HAL_OK, HAL_OK, HAL_ERROR};

    reset_project_state();
    TEST_ASSERT_UINT_EQ(1U, MPU6050_Init(0));
    TEST_ASSERT_UINT_EQ(1U, MPU6050_Test_WriteRegister(0x19U, 0U));

    reset_project_state();
    TEST_ASSERT_UINT_EQ(0U, MPU6050_Init(&i2c));
    TEST_ASSERT_TRUE((fake_CoreDebug.DEMCR & CoreDebug_DEMCR_TRCENA_Msk) != 0U);
    TEST_ASSERT_TRUE((fake_DWT.CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0U);
    TEST_ASSERT_UINT_EQ(1U, fake_stm32.delay_calls);

    reset_project_state();
    fake_stm32.i2c_ready_status = HAL_ERROR;
    TEST_ASSERT_UINT_EQ(1U, MPU6050_Init(&i2c));

    reset_project_state();
    set_i2c_write_sequence(ret_power_fails, 1U);
    TEST_ASSERT_UINT_EQ(1U, MPU6050_Init(&i2c));

    reset_project_state();
    set_i2c_write_sequence(ret_sample_fails, 2U);
    TEST_ASSERT_UINT_EQ(1U, MPU6050_Init(&i2c));

    reset_project_state();
    set_i2c_write_sequence(ret_filter_fails, 3U);
    TEST_ASSERT_UINT_EQ(1U, MPU6050_Init(&i2c));

    reset_project_state();
    MPU6050_Test_SetI2c(&i2c);
    fake_stm32.i2c_mem_write_status = HAL_ERROR;
    TEST_ASSERT_UINT_EQ(1U, MPU6050_Test_WriteRegister(0x1AU, 0x03U));

    reset_project_state();
    TEST_ASSERT_UINT_EQ(1U, MPU6050_AccelerometerConfig(0x18U));
    TEST_ASSERT_UINT_EQ(1U, MPU6050_GyroConfig(0x18U));

    MPU6050_Test_SetI2c(&i2c);
    TEST_ASSERT_UINT_EQ(0U, MPU6050_AccelerometerConfig(0x18U));
    TEST_ASSERT_UINT_EQ(2048U, AccelLSBSens);
    TEST_ASSERT_UINT_EQ(0U, MPU6050_AccelerometerConfig(0x10U));
    TEST_ASSERT_UINT_EQ(4096U, AccelLSBSens);
    TEST_ASSERT_UINT_EQ(0U, MPU6050_AccelerometerConfig(0x08U));
    TEST_ASSERT_UINT_EQ(8192U, AccelLSBSens);
    TEST_ASSERT_UINT_EQ(0U, MPU6050_AccelerometerConfig(0x00U));

    TEST_ASSERT_UINT_EQ(0U, MPU6050_GyroConfig(0x18U));
    TEST_ASSERT_FLOAT_NEAR(16.4f, (float)GyrolLSBSens, 0.001f);
    TEST_ASSERT_UINT_EQ(0U, MPU6050_GyroConfig(0x10U));
    TEST_ASSERT_FLOAT_NEAR(32.8f, (float)GyrolLSBSens, 0.001f);
    TEST_ASSERT_UINT_EQ(0U, MPU6050_GyroConfig(0x08U));
    TEST_ASSERT_FLOAT_NEAR(65.5f, (float)GyrolLSBSens, 0.001f);
    TEST_ASSERT_UINT_EQ(0U, MPU6050_GyroConfig(0x00U));

    fake_stm32.i2c_mem_write_status = HAL_ERROR;
    TEST_ASSERT_UINT_EQ(1U, MPU6050_AccelerometerConfig(0x18U));
    TEST_ASSERT_UINT_EQ(1U, MPU6050_GyroConfig(0x18U));
    TEST_ASSERT_INT_EQ(-2, bytes_to_int(0xFFU, 0xFEU));
}

static void ctfl_mpu6050_sampling_display_state_transitions(void)
{
    I2C_HandleTypeDef i2c = make_i2c(&i2c_instance_a);

    reset_project_state();
    MPU6050_Read();
    TEST_ASSERT_UINT_EQ(0U, fake_stm32.i2c_mem_read_dma_calls);

    MPU6050_Test_SetI2c(&i2c);
    MPU_I2C_Ready = 0U;
    MPU6050_Read();
    TEST_ASSERT_UINT_EQ(0U, fake_stm32.i2c_mem_read_dma_calls);

    MPU_I2C_Ready = 1U;
    fake_stm32.i2c_state = HAL_I2C_STATE_BUSY;
    MPU6050_Read();
    TEST_ASSERT_UINT_EQ(0U, fake_stm32.i2c_mem_read_dma_calls);

    fake_stm32.i2c_state = HAL_I2C_STATE_READY;
    fake_stm32.i2c_mem_read_dma_status = HAL_ERROR;
    MPU6050_Read();
    TEST_ASSERT_UINT_EQ(1U, MPU_I2C_Ready);

    fake_stm32.i2c_mem_read_dma_status = HAL_OK;
    MPU6050_Read();
    TEST_ASSERT_UINT_EQ(0U, MPU_I2C_Ready);

    MPU6050_I2C_Error();
    TEST_ASSERT_UINT_EQ(1U, MPU_I2C_Ready);
    TEST_ASSERT_UINT_EQ(0U, MPU_Data_Ready);

    TEST_ASSERT_UINT_EQ(0U, MPU6050_Process_Data_Ready());
    MPU6050_Mark_Data_Ready();
    load_level_frame();
    fake_DWT.CYCCNT = 10U;
    TEST_ASSERT_UINT_EQ(1U, MPU6050_Process_Data_Ready());
    TEST_ASSERT_UINT_EQ(1U, MPU_I2C_Ready);

    reset_project_state();
    MPU6050_Test_SetI2c(&i2c);
    SystemCoreClock = 1000U;
    load_level_frame();
    fake_DWT.CYCCNT = 10U;
    MPU6050_Data_Update();
    TEST_ASSERT_FLOAT_NEAR(0.001f, MPU6050_Dt, 0.000001f);
    fake_DWT.CYCCNT = 20U;
    MPU6050_Data_Update();
    TEST_ASSERT_FLOAT_NEAR(0.010f, MPU6050_Dt, 0.000001f);
    fake_DWT.CYCCNT = 1000U;
    MPU6050_Data_Update();
    TEST_ASSERT_FLOAT_NEAR(0.001f, MPU6050_Dt, 0.000001f);
    MPU6050_Data_Update();
    TEST_ASSERT_FLOAT_NEAR(0.001f, MPU6050_Dt, 0.000001f);

    reset_project_state();
    MPU6050_Display_Update();
    MPU6050_Set_Display_Enabled(1U);
    oled_ready = 0U;
    MPU6050_Display_Update();
    oled_ready = 1U;
    MPU_I2C_Ready = 0U;
    MPU6050_Display_Update();
    MPU_I2C_Ready = 1U;
    MPU6050_Display_Update();
    MPU6050_Test_SetI2c(&i2c);
    fake_stm32.tick_ms = 10U;
    MPU6050_Display_Update();
    fake_stm32.tick_ms = 1000U;
    fake_stm32.i2c_state = HAL_I2C_STATE_BUSY;
    MPU6050_Display_Update();
    fake_stm32.i2c_state = HAL_I2C_STATE_READY;
    MPU6050_Display_Update();
    TEST_ASSERT_UINT_EQ(1U, fake_stm32.ssd1306_update_dma_calls);
    TEST_ASSERT_UINT_EQ(5U, fake_stm32.ssd1306_puts_calls);
}

static void ctfl_balance_app_initialization_decision_table(void)
{
    I2C_HandleTypeDef i2c = make_i2c(&i2c_instance_a);
    TIM_HandleTypeDef timer = make_timer(&timer_instance_a, 99U);
    const HAL_StatusTypeDef accel_fails[] = {HAL_OK, HAL_OK, HAL_OK, HAL_ERROR};
    const HAL_StatusTypeDef gyro_fails[] = {HAL_OK, HAL_OK, HAL_OK, HAL_OK, HAL_ERROR};

    reset_project_state();
    TEST_ASSERT_UINT_EQ(1U, BalanceApp_Init(0, &timer));
    TEST_ASSERT_UINT_EQ(1U, BalanceApp_Init(&i2c, 0));

    reset_project_state();
    fake_stm32.i2c_ready_status = HAL_ERROR;
    TEST_ASSERT_UINT_EQ(1U, BalanceApp_Init(&i2c, &timer));

    reset_project_state();
    set_i2c_write_sequence(accel_fails, 4U);
    TEST_ASSERT_UINT_EQ(1U, BalanceApp_Init(&i2c, &timer));

    reset_project_state();
    set_i2c_write_sequence(gyro_fails, 5U);
    TEST_ASSERT_UINT_EQ(1U, BalanceApp_Init(&i2c, &timer));

    reset_project_state();
    fake_stm32.ssd1306_init_return = 0U;
    TEST_ASSERT_UINT_EQ(0U, BalanceApp_Init(&i2c, &timer));

    reset_project_state();
    fake_stm32.tim_pwm_start_status[TIM_CHANNEL_1] = HAL_ERROR;
    TEST_ASSERT_UINT_EQ(1U, BalanceApp_Init(&i2c, &timer));

    reset_project_state();
    fake_stm32.tim_pwm_start_status[TIM_CHANNEL_2] = HAL_ERROR;
    TEST_ASSERT_UINT_EQ(1U, BalanceApp_Init(&i2c, &timer));

    reset_project_state();
    fake_stm32.tim_base_start_status = HAL_ERROR;
    TEST_ASSERT_UINT_EQ(1U, BalanceApp_Init(&i2c, &timer));

    reset_project_state();
    TEST_ASSERT_UINT_EQ(0U, BalanceApp_Init(&i2c, &timer));
    TEST_ASSERT_UINT_EQ(1U, fake_stm32.tim_base_start_calls);
}

static void ctfl_balance_app_callbacks_and_main_state_transitions(void)
{
    I2C_HandleTypeDef i2c_a = make_i2c(&i2c_instance_a);
    I2C_HandleTypeDef i2c_b = make_i2c(&i2c_instance_b);
    TIM_HandleTypeDef timer_a = make_timer(&timer_instance_a, 99U);
    TIM_HandleTypeDef timer_b = make_timer(&timer_instance_b, 99U);

    reset_project_state();
    BalanceApp_TimerElapsedCallback(&timer_a);
    BalanceApp_I2cMemRxCompleteCallback(&i2c_a);
    oled_ready = 0U;
    BalanceApp_I2cMasterTxCompleteCallback(&i2c_a);
    BalanceApp_I2cErrorCallback(&i2c_a);
    TEST_ASSERT_UINT_EQ(0U, fake_stm32.i2c_mem_read_dma_calls);
    TEST_ASSERT_UINT_EQ(0U, MPU_Data_Ready);
    TEST_ASSERT_UINT_EQ(0U, oled_ready);

    TEST_ASSERT_UINT_EQ(0U, BalanceApp_Init(&i2c_a, &timer_a));
    FakeStm32_Reset();
    MPU_I2C_Ready = 1U;

    BalanceApp_TimerElapsedCallback(0);
    BalanceApp_TimerElapsedCallback(&timer_b);
    BalanceApp_TimerElapsedCallback(&timer_a);
    TEST_ASSERT_UINT_EQ(1U, fake_stm32.i2c_mem_read_dma_calls);

    MPU_Data_Ready = 0U;
    BalanceApp_I2cMemRxCompleteCallback(0);
    BalanceApp_I2cMemRxCompleteCallback(&i2c_b);
    BalanceApp_I2cMemRxCompleteCallback(&i2c_a);
    TEST_ASSERT_UINT_EQ(1U, MPU_Data_Ready);

    oled_ready = 0U;
    BalanceApp_I2cMasterTxCompleteCallback(0);
    BalanceApp_I2cMasterTxCompleteCallback(&i2c_b);
    TEST_ASSERT_UINT_EQ(0U, oled_ready);
    BalanceApp_I2cMasterTxCompleteCallback(&i2c_a);
    TEST_ASSERT_UINT_EQ(1U, oled_ready);

    MPU_Data_Ready = 1U;
    MPU_I2C_Ready = 0U;
    oled_ready = 0U;
    BalanceApp_I2cErrorCallback(0);
    BalanceApp_I2cErrorCallback(&i2c_b);
    TEST_ASSERT_UINT_EQ(0U, oled_ready);
    BalanceApp_I2cErrorCallback(&i2c_a);
    TEST_ASSERT_UINT_EQ(1U, oled_ready);
    TEST_ASSERT_UINT_EQ(0U, MPU_Data_Ready);
    TEST_ASSERT_UINT_EQ(1U, MPU_I2C_Ready);

    MPU_Data_Ready = 0U;
    BalanceApp_MainFunction();
    TEST_ASSERT_UINT_EQ(0U, fake_stm32.compare_write_calls[TIM_CHANNEL_1]);

    load_level_frame();
    MPU_Data_Ready = 1U;
    SystemCoreClock = 1000U;
    fake_DWT.CYCCNT = 100U;
    BalanceApp_MainFunction();
    TEST_ASSERT_TRUE(fake_stm32.compare_write_calls[TIM_CHANNEL_1] > 0U);

    load_pitch_fall_frame();
    MPU_Data_Ready = 1U;
    fake_DWT.CYCCNT = 150U;
    BalanceApp_MainFunction();
    TEST_ASSERT_TRUE(FakeStm32_CountPinWrites(GPIO_PIN_1, GPIO_PIN_RESET) > 0U);
}

int main(void)
{
    RUN_TEST(ctfl_white_box_null_pointer_guards);
    RUN_TEST(ctfl_pid_equivalence_boundaries_and_branches);
    RUN_TEST(ctfl_balance_control_state_transition_decisions);
    RUN_TEST(ctfl_motor_actuator_partitions_and_boundaries);
    RUN_TEST(ctfl_imu_processing_fusion_modes_and_boundaries);
    RUN_TEST(ctfl_kalman_filter_statement_and_branch_coverage);
    RUN_TEST(ctfl_motor_driver_hal_adapter_decision_table);
    RUN_TEST(ctfl_mpu6050_init_config_decision_table);
    RUN_TEST(ctfl_mpu6050_sampling_display_state_transitions);
    RUN_TEST(ctfl_balance_app_initialization_decision_table);
    RUN_TEST(ctfl_balance_app_callbacks_and_main_state_transitions);
    return TEST_REPORT();
}
