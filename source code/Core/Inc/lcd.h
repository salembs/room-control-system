#ifndef LCD_H
#define LCD_H

void LCD_Init(void);
void LCD_Send_Cmd(char cmd);
void LCD_Send_Data(char data);
void LCD_Send_String(char *str);
void LCD_Set_Cursor(uint8_t row, uint8_t col);

#endif
