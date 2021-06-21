#ifndef _OLED_H_
#define _OLED_H_

#define _OLED_USE_DMA_

#define DC_HIGH HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_SET)
#define DC_LOW HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_RESET)

#include "main.h"
#include "spi.h"
#include "oledfonts.h"

typedef enum
{
    Black = 0,
    White = 1
} OLED_Color;

typedef struct
{
    uint8_t FPS;
    uint8_t FPS_001;
    uint8_t FPS_010;
    uint8_t FPS_100;
} FPS_Struct;

typedef struct
{
    uint8_t x;
    uint8_t y;
} OLED_Poisition_struct;

extern uint8_t GRAM[1024];

void OLED_Init();
void OLED_Reset();
void OLED_WriteCmd(uint8_t cmd);
void OLED_WriteData(uint8_t *buffer, uint16_t buffersize);
void OLED_Clean();
void OLED_Update();
void OLED_DrawPixel(uint8_t x, uint8_t y, uint8_t color);
void OLED_WriteChar(const uint8_t *ch, uint8_t x, uint8_t y);
void OLED_WriteChinese(const uint8_t *chinese, uint8_t x, uint8_t y);

#endif
