#ifndef FAKE_STM32_H_
#define FAKE_STM32_H_

#include <stdint.h>

#include "ssd1306.h"
#include "stm32f1xx_hal.h"

#define FAKE_STM32_MAX_GPIO_WRITES 128U
#define FAKE_STM32_MAX_I2C_WRITES  32U

typedef struct
{
    GPIO_TypeDef *port;
    uint16_t pin;
    GPIO_PinState state;
} FakeStm32_GpioWrite;

typedef struct
{
    uint16_t dev_address;
    uint16_t mem_address;
    uint8_t value;
} FakeStm32_I2cWrite;

typedef struct
{
    HAL_StatusTypeDef i2c_ready_status;
    HAL_StatusTypeDef i2c_mem_write_status;
    HAL_StatusTypeDef i2c_mem_write_sequence[FAKE_STM32_MAX_I2C_WRITES];
    uint32_t i2c_mem_write_sequence_count;
    HAL_StatusTypeDef i2c_mem_read_dma_status;
    HAL_I2C_StateTypeDef i2c_state;
    HAL_StatusTypeDef tim_pwm_start_status[3];
    HAL_StatusTypeDef tim_base_start_status;
    uint32_t tick_ms;
    uint32_t delay_calls;
    uint32_t delay_last_ms;
    uint32_t i2c_ready_calls;
    uint32_t i2c_mem_write_calls;
    uint32_t i2c_mem_read_dma_calls;
    uint32_t tim_pwm_start_calls[3];
    uint32_t tim_base_start_calls;
    uint32_t compare_write_calls[3];
    uint32_t compare_last[3];
    uint32_t gpio_write_count;
    FakeStm32_GpioWrite gpio_writes[FAKE_STM32_MAX_GPIO_WRITES];
    uint32_t i2c_write_count;
    FakeStm32_I2cWrite i2c_writes[FAKE_STM32_MAX_I2C_WRITES];
    uint8_t dma_payload[14];
    uint16_t dma_payload_size;
    uint8_t ssd1306_init_return;
    uint32_t ssd1306_init_calls;
    uint32_t ssd1306_fill_calls;
    uint32_t ssd1306_goto_calls;
    uint32_t ssd1306_puts_calls;
    uint32_t ssd1306_update_dma_calls;
} FakeStm32_State;

extern FakeStm32_State fake_stm32;

void FakeStm32_Reset(void);
uint32_t FakeStm32_CountPinWrites(uint16_t pin, GPIO_PinState state);

#endif /* FAKE_STM32_H_ */
