#pragma once

#include "mbed.h"

#define LCD_DisplayOn 0x41
#define LCD_DisplayOff 0x42
#define LCD_SetCursor 0x45
#define LCD_CursorHome 0x46
#define LCD_UnderlineOn 0x47
#define LCD_UnderlineOff 0x48
#define LCD_MoveCursorLeft 0x49
#define LCD_MoveCursorRight 0x4A
#define LCD_BlinkCursorOn 0x4B
#define LCD_BlinkCursorOff 0x4C
#define LCD_BackSpace 0x4E
#define LCD_ClearScreen 0x51
#define LCD_SetContrast 0x52
#define LCD_SetBacklight 0x53
#define LCD_MoveDisplayLeft 0x55
#define LCD_MoveDisplayRight 0x56

#define LCD_ChangeRS232 0x61
#define LCD_ChangeI2C 0x62
#define LCD_DisplayVersion 0x70
#define LCD_DisplayRS232 0x71
#define LCD_DisplayI2C 0x72

class NewHavenDisplay
{
  public:
    NewHavenDisplay(Serial &_ser); //constructor

    void home();
    void clear();

    void display();
    void noDisplay();
    void setCursor(uint8_t line, uint8_t column);
    void underline();
    void noUnderline();
    void moveCursor(uint8_t direction, uint8_t column);
    void moveCursorRight();
    void moveCursorLeft();
    void blink();
    void noBlink();
    void backspace();
    void setContrast(uint8_t contrast);
    void setBacklight(uint8_t brightness);
    void shift(uint8_t direction, uint8_t column);
    void rightShift();
    void leftShift();

    void changeI2C(uint8_t address);
    void changeRS232(uint8_t baud);

    void showFirmware();
    void showI2C();
    void showRS232();

  private:
    int pin, baud;
    Serial *serial;
    void send(uint8_t command);
    void send(uint8_t command, uint8_t parameter);
};