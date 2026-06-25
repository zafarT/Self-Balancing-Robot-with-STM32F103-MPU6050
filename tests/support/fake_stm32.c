#include "fake_stm32.h"

#include <string.h>

GPIO_TypeDef fake_GPIOA = {0x40010800U};
DWT_Type fake_DWT;
CoreDebug_Type fake_CoreDebug;
uint32_t SystemCoreClock = 72000000U;
volatile uint8_t oled_ready = 1U;
FontDef_t Font_7x10 = {7U, 10U, 0};
FakeStm32_State fake_stm32;

void FakeStm32_Reset(void)
{
    memset(&fake_stm32, 0, sizeof(fake_stm32));
    fake_stm32.i2c_ready_status = HAL_OK;
    fake_stm32.i2c_mem_write_status = HAL_OK;
    fake_stm32.i2c_mem_read_dma_status = HAL_OK;
    fake_stm32.i2c_state = HAL_I2C_STATE_READY;
    fake_stm32.tim_pwm_start_status[TIM_CHANNEL_1] = HAL_OK;
    fake_stm32.tim_pwm_start_status[TIM_CHANNEL_2] = HAL_OK;
    fake_stm32.tim_base_start_status = HAL_OK;
    fake_stm32.ssd1306_init_return = 1U;
    fake_DWT.CTRL = 0U;
    fake_DWT.CYCCNT = 0U;
    fake_CoreDebug.DEMCR = 0U;
    SystemCoreClock = 72000000U;
    oled_ready = 1U;
}

uint32_t FakeStm32_CountPinWrites(uint16_t pin, GPIO_PinState state)
{
    uint32_t index;
    uint32_t count = 0U;

    for (index = 0U; index < fake_stm32.gpio_write_count; index++)
    {
        if ((fake_stm32.gpio_writes[index].pin == pin) &&
            (fake_stm32.gpio_writes[index].state == state))
        {
            count++;
        }
    }

    return count;
}

void HAL_GPIO_WritePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState)
{
    if (fake_stm32.gpio_write_count < FAKE_STM32_MAX_GPIO_WRITES)
    {
        FakeStm32_GpioWrite *write = &fake_stm32.gpio_writes[fake_stm32.gpio_write_count];
        write->port = GPIOx;
        write->pin = GPIO_Pin;
        write->state = PinState;
    }

    fake_stm32.gpio_write_count++;
}

HAL_StatusTypeDef HAL_I2C_IsDeviceReady(I2C_HandleTypeDef *hi2c,
                                        uint16_t DevAddress,
                                        uint32_t Trials,
                                        uint32_t Timeout)
{
    (void)hi2c;
    (void)DevAddress;
    (void)Trials;
    (void)Timeout;
    fake_stm32.i2c_ready_calls++;
    return fake_stm32.i2c_ready_status;
}

HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c,
                                    uint16_t DevAddress,
                                    uint16_t MemAddress,
                                    uint16_t MemAddSize,
                                    uint8_t *pData,
                                    uint16_t Size,
                                    uint32_t Timeout)
{
    HAL_StatusTypeDef status = fake_stm32.i2c_mem_write_status;
    const uint32_t call_index = fake_stm32.i2c_mem_write_calls;

    (void)hi2c;
    (void)MemAddSize;
    (void)Size;
    (void)Timeout;

    if (call_index < fake_stm32.i2c_mem_write_sequence_count)
    {
        status = fake_stm32.i2c_mem_write_sequence[call_index];
    }

    if (fake_stm32.i2c_write_count < FAKE_STM32_MAX_I2C_WRITES)
    {
        FakeStm32_I2cWrite *write = &fake_stm32.i2c_writes[fake_stm32.i2c_write_count];
        write->dev_address = DevAddress;
        write->mem_address = MemAddress;
        write->value = (pData != 0) ? *pData : 0U;
    }

    fake_stm32.i2c_write_count++;
    fake_stm32.i2c_mem_write_calls++;
    return status;
}

HAL_StatusTypeDef HAL_I2C_Mem_Read_DMA(I2C_HandleTypeDef *hi2c,
                                       uint16_t DevAddress,
                                       uint16_t MemAddress,
                                       uint16_t MemAddSize,
                                       uint8_t *pData,
                                       uint16_t Size)
{
    (void)hi2c;
    (void)DevAddress;
    (void)MemAddress;
    (void)MemAddSize;

    fake_stm32.i2c_mem_read_dma_calls++;

    if ((pData != 0) && (fake_stm32.dma_payload_size > 0U))
    {
        const uint16_t copy_size = (Size < fake_stm32.dma_payload_size) ? Size : fake_stm32.dma_payload_size;
        memcpy(pData, fake_stm32.dma_payload, copy_size);
    }

    return fake_stm32.i2c_mem_read_dma_status;
}

HAL_I2C_StateTypeDef HAL_I2C_GetState(I2C_HandleTypeDef *hi2c)
{
    (void)hi2c;
    return fake_stm32.i2c_state;
}

void HAL_Delay(uint32_t Delay)
{
    fake_stm32.delay_calls++;
    fake_stm32.delay_last_ms = Delay;
}

HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *htim, uint32_t Channel)
{
    (void)htim;

    if (Channel <= TIM_CHANNEL_2)
    {
        fake_stm32.tim_pwm_start_calls[Channel]++;
        return fake_stm32.tim_pwm_start_status[Channel];
    }

    return HAL_ERROR;
}

HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *htim)
{
    (void)htim;
    fake_stm32.tim_base_start_calls++;
    return fake_stm32.tim_base_start_status;
}

uint32_t HAL_GetTick(void)
{
    return fake_stm32.tick_ms;
}

void Fake_HAL_TIM_SetCompare(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t Compare)
{
    if ((htim != 0) && (Channel <= TIM_CHANNEL_2))
    {
        htim->Compare[Channel] = Compare;
        fake_stm32.compare_last[Channel] = Compare;
        fake_stm32.compare_write_calls[Channel]++;
    }
}

uint32_t Fake_HAL_TIM_GetAutoreload(TIM_HandleTypeDef *htim)
{
    return (htim != 0) ? htim->Init.Period : 0U;
}

uint8_t SSD1306_Init(void)
{
    fake_stm32.ssd1306_init_calls++;
    return fake_stm32.ssd1306_init_return;
}

void SSD1306_Fill(SSD1306_COLOR_t color)
{
    (void)color;
    fake_stm32.ssd1306_fill_calls++;
}

void SSD1306_GotoXY(uint16_t x, uint16_t y)
{
    (void)x;
    (void)y;
    fake_stm32.ssd1306_goto_calls++;
}

char SSD1306_Puts(const char *str, FontDef_t *font, SSD1306_COLOR_t color)
{
    (void)str;
    (void)font;
    (void)color;
    fake_stm32.ssd1306_puts_calls++;
    return 0;
}

void SSD1306_UpdateScreen_DMA(void)
{
    fake_stm32.ssd1306_update_dma_calls++;
}
