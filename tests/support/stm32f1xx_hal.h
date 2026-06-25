#ifndef TEST_STM32F1XX_HAL_H_
#define TEST_STM32F1XX_HAL_H_

#include <stddef.h>
#include <stdint.h>

typedef enum
{
    HAL_OK = 0x00U,
    HAL_ERROR = 0x01U,
    HAL_BUSY = 0x02U,
    HAL_TIMEOUT = 0x03U
} HAL_StatusTypeDef;

typedef enum
{
    HAL_I2C_STATE_RESET = 0x00U,
    HAL_I2C_STATE_READY = 0x20U,
    HAL_I2C_STATE_BUSY = 0x24U
} HAL_I2C_StateTypeDef;

typedef enum
{
    GPIO_PIN_RESET = 0U,
    GPIO_PIN_SET
} GPIO_PinState;

typedef struct
{
    uintptr_t id;
} GPIO_TypeDef;

typedef struct
{
    void *Instance;
} I2C_HandleTypeDef;

typedef struct
{
    uint32_t Period;
} TIM_Base_InitTypeDef;

typedef struct
{
    void *Instance;
    TIM_Base_InitTypeDef Init;
    uint32_t Compare[5];
} TIM_HandleTypeDef;

typedef struct
{
    volatile uint32_t CTRL;
    volatile uint32_t CYCCNT;
} DWT_Type;

typedef struct
{
    volatile uint32_t DEMCR;
} CoreDebug_Type;

extern GPIO_TypeDef fake_GPIOA;
extern DWT_Type fake_DWT;
extern CoreDebug_Type fake_CoreDebug;
extern uint32_t SystemCoreClock;

#define GPIOA (&fake_GPIOA)

#define GPIO_PIN_1 ((uint16_t)0x0002U)
#define GPIO_PIN_2 ((uint16_t)0x0004U)
#define GPIO_PIN_3 ((uint16_t)0x0008U)
#define GPIO_PIN_4 ((uint16_t)0x0010U)
#define GPIO_PIN_5 ((uint16_t)0x0020U)

#define I2C_MEMADD_SIZE_8BIT 1U

#define TIM_CHANNEL_1 1U
#define TIM_CHANNEL_2 2U

#define CoreDebug (&fake_CoreDebug)
#define DWT (&fake_DWT)
#define CoreDebug_DEMCR_TRCENA_Msk (1UL << 24)
#define DWT_CTRL_CYCCNTENA_Msk (1UL << 0)

void HAL_GPIO_WritePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);
HAL_StatusTypeDef HAL_I2C_IsDeviceReady(I2C_HandleTypeDef *hi2c,
                                        uint16_t DevAddress,
                                        uint32_t Trials,
                                        uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c,
                                    uint16_t DevAddress,
                                    uint16_t MemAddress,
                                    uint16_t MemAddSize,
                                    uint8_t *pData,
                                    uint16_t Size,
                                    uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Mem_Read_DMA(I2C_HandleTypeDef *hi2c,
                                       uint16_t DevAddress,
                                       uint16_t MemAddress,
                                       uint16_t MemAddSize,
                                       uint8_t *pData,
                                       uint16_t Size);
HAL_I2C_StateTypeDef HAL_I2C_GetState(I2C_HandleTypeDef *hi2c);
void HAL_Delay(uint32_t Delay);
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *htim);
uint32_t HAL_GetTick(void);

void Fake_HAL_TIM_SetCompare(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t Compare);
uint32_t Fake_HAL_TIM_GetAutoreload(TIM_HandleTypeDef *htim);

#define __HAL_TIM_SET_COMPARE(htim, channel, compare) \
    Fake_HAL_TIM_SetCompare((htim), (channel), (compare))

#define __HAL_TIM_GET_AUTORELOAD(htim) \
    Fake_HAL_TIM_GetAutoreload((htim))

#endif /* TEST_STM32F1XX_HAL_H_ */
