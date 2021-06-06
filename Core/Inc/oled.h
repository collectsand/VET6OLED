#ifndef __OLED_H_
#define __OLED_H_

#define RES_HIGH HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET)
#define RES_LOW HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET)
#define DC_HIGH HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET)
#define DC_LOW HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET)

#include "main.h"
#include "spi.h"

extern uint8_t Display_Data[];

void OLED_Init();
void OLED_Write_One_CMD(uint8_t cmd);
void OLED_Write_Data(uint8_t column_start, uint8_t column_end, uint8_t page_start, uint8_t page_end, uint8_t *data);
void OLED_Clean();

#endif
