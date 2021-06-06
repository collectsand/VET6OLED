#include "oled.h"

uint8_t OLED_Init_CMD[] = {0xAE, 0xD5, 80, 0xA8, 0X3F, 0xD3, 0X00, 0x40, 0x8D, 0x14, 0x20, 0x02, 0xA1, 0xC0, 0xDA, 0x12, 0x81, 0xEF, 0xD9, 0xf1, 0xDB, 0x30, 0xA4, 0xA6, 0xAF};
uint8_t Display_Data[] = {0x00, 0x00, 0x00, 0xE7, 0x42, 0x42, 0x42, 0x42, 0x7E, 0x42, 0x42, 0x42, 0x42, 0xE7, 0x00, 0x00 /*"H",0*/};
uint8_t GDDRAM[8][128] = {0};

void OLED_Init()
{
    RES_LOW;
    HAL_Delay(100);
    RES_HIGH;
    DC_LOW;
    HAL_SPI_Transmit(&hspi1, OLED_Init_CMD, 25, 0xFFFF);
    OLED_Clean();
}

void OLED_Write_One_CMD(uint8_t cmd)
{
    DC_LOW;
    HAL_SPI_Transmit(&hspi1, &cmd, 1, 0xFF);
}

void OLED_Write_Data(uint8_t column_start, uint8_t column_end, uint8_t page_start, uint8_t page_end, uint8_t *data)
{
    uint8_t position_cmd[6] = {0x21, 0, 0, 0x22, 0, 0};
    uint8_t size;
    position_cmd[1] = column_start;
    position_cmd[2] = column_end;
    position_cmd[4] = page_start;
    position_cmd[5] = page_end;
    size = sizeof(data);
    DC_LOW;
    HAL_SPI_Transmit(&hspi1, position_cmd, 6, 0xFF);
    DC_HIGH;
    HAL_SPI_Transmit(&hspi1, data, size, 0xFF);
}

void OLED_Clean()
{
    uint8_t i = 0;
    uint8_t position_cmd[6] = {0x21, 0, 0, 0x22, 0, 0};
    DC_HIGH;
    HAL_SPI_Transmit(&hspi1, position_cmd, 6, 0xFF);
    for (i = 0; i < 8; i++)
    {
        DC_HIGH;
        HAL_SPI_Transmit(&hspi1, *(GDDRAM + i), 128, 0xFF);
    }
}