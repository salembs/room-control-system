#include "stm32f4xx_hal.h"
#include "lcd.h"
#include <string.h>
#include <stdio.h>

// Change to your connections
#define RS_Pin GPIO_PIN_2
#define EN_Pin GPIO_PIN_3
#define D4_Pin GPIO_PIN_4
#define D5_Pin GPIO_PIN_5
#define D6_Pin GPIO_PIN_6
#define D7_Pin GPIO_PIN_7
#define LCD_Port GPIOE

void delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t delayTicks = us * (HAL_RCC_GetHCLKFreq() / 1000000);
    while ((DWT->CYCCNT - start) < delayTicks);
}

void LCD_Enable()
{
    HAL_GPIO_WritePin(LCD_Port, EN_Pin, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(LCD_Port, EN_Pin, GPIO_PIN_RESET);
    HAL_Delay(1);
}

void LCD_Send_4Bits(uint8_t data)
{
    HAL_GPIO_WritePin(LCD_Port, D4_Pin, (data >> 0) & 0x01);
    HAL_GPIO_WritePin(LCD_Port, D5_Pin, (data >> 1) & 0x01);
    HAL_GPIO_WritePin(LCD_Port, D6_Pin, (data >> 2) & 0x01);
    HAL_GPIO_WritePin(LCD_Port, D7_Pin, (data >> 3) & 0x01);
    LCD_Enable();
}

void LCD_Send_Cmd(char cmd)
{
    HAL_GPIO_WritePin(LCD_Port, RS_Pin, GPIO_PIN_RESET);
    LCD_Send_4Bits(cmd >> 4);
    LCD_Send_4Bits(cmd & 0x0F);
    HAL_Delay(2);
}

void LCD_Send_Data(char data)
{
    HAL_GPIO_WritePin(LCD_Port, RS_Pin, GPIO_PIN_SET);
    LCD_Send_4Bits(data >> 4);
    LCD_Send_4Bits(data & 0x0F);
    HAL_Delay(2);
}

void LCD_Send_String(char *str)
{
    while (*str) {
        LCD_Send_Data(*str++);
    }
}

void LCD_Set_Cursor(uint8_t row, uint8_t col)
{
    uint8_t address = (row == 0) ? 0x80 + col : 0xC0 + col;
    LCD_Send_Cmd(address);
}

void LCD_Init(void)
{
    HAL_Delay(20);
    LCD_Send_Cmd(0x02); // Init LCD in 4-bit mode
    LCD_Send_Cmd(0x28); // 2 line, 5x8 font
    LCD_Send_Cmd(0x0C); // Display ON, cursor OFF
    LCD_Send_Cmd(0x06); // Entry mode
    LCD_Send_Cmd(0x01); // Clear display
    HAL_Delay(2);
}
