#ifndef TEST_SSD1306_H_
#define TEST_SSD1306_H_

#include <stdint.h>

#include "stm32f1xx_hal.h"

typedef struct
{
    uint8_t FontWidth;
    uint8_t FontHeight;
    const uint16_t *data;
} FontDef_t;

typedef enum
{
    SSD1306_COLOR_BLACK = 0x00,
    SSD1306_COLOR_WHITE = 0x01
} SSD1306_COLOR_t;

extern volatile uint8_t oled_ready;
extern FontDef_t Font_7x10;

uint8_t SSD1306_Init(void);
void SSD1306_Fill(SSD1306_COLOR_t color);
void SSD1306_GotoXY(uint16_t x, uint16_t y);
char SSD1306_Puts(const char *str, FontDef_t *font, SSD1306_COLOR_t color);
void SSD1306_UpdateScreen_DMA(void);

#endif /* TEST_SSD1306_H_ */
