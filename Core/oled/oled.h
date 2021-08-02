#ifndef __OLED_H
#define __OLED_H

#define OLED_SPI_HANDLE hspi1
#define OLED_DC_GPIO GPIOE
#define OLED_DC_PIN GPIO_PIN_6
#define OLED_RST_GPIO GPIOE
#define OLED_RST_PIN GPIO_PIN_5
#define OLED_CS_GPIO GPIOA
#define OLED_CS_PIN GPIO_PIN_6

#define DC_LOW HAL_GPIO_WritePin(OLED_DC_GPIO, OLED_DC_PIN, GPIO_PIN_RESET)
#define DC_HIGH HAL_GPIO_WritePin(OLED_DC_GPIO, OLED_DC_PIN, GPIO_PIN_SET)

#define RST_LOW HAL_GPIO_WritePin(OLED_RST_GPIO, OLED_RST_PIN, GPIO_PIN_RESET)
#define RST_HIGH HAL_GPIO_WritePin(OLED_RST_GPIO, OLED_RST_PIN, GPIO_PIN_SET)

#define CS_LOW HAL_GPIO_WritePin(OLED_CS_GPIO, OLED_CS_PIN, GPIO_PIN_RESET)
#define CS_HIGH HAL_GPIO_WritePin(OLED_CS_GPIO, OLED_CS_PIN, GPIO_PIN_SET)

#include "stm32f1xx_hal.h"

typedef enum
{
    Black = 0,
    White = 1
} OLED_Color;

extern uint8_t GRAM[1024];
extern uint16_t OLED_FPS;
extern uint8_t OLED_Display;

void OLED_Init();
void OLED_Reset();
void OLED_WriteCmd(uint8_t cmd);
void OLED_WriteData(uint8_t *buffer, uint16_t buffersize);
void OLED_Clean();
void OLED_Update();
void OLED_DrawPixel(uint8_t x, uint8_t y, uint8_t color);
void OLED_WriteChar(uint8_t ch, uint8_t x, uint8_t y);
void OLED_WriteString(char *str, uint8_t x, uint8_t y);
void OLED_WriteNumber(float number, uint8_t x, uint8_t y);
void OLED_WriteChinese(const uint8_t *chinese, uint8_t x, uint8_t y);

#endif
