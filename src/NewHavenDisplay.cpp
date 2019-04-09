#include "NewHavenDisplay.h"

NewHavenDisplay::NewHavenDisplay(Serial &_ser)
{
    serial = &_ser;
    send(LCD_DisplayOn);
    send(LCD_ClearScreen);
    send(LCD_SetContrast, 50);
    send(LCD_SetBacklight, 0x08);
}

/*********** INTERNAL FUNCTIONS ***********/
void NewHavenDisplay::send(uint8_t command)
{

    serial->putc(0xFE);
    serial->putc(command);
}
void NewHavenDisplay::send(uint8_t command, uint8_t parameter)
{
    serial->putc(0xFE);
    serial->putc(command);
    serial->putc(parameter);
}
/***********************************************/

//Clear Display and Set cursor to line 1 column 1
void NewHavenDisplay::clear()
{
    send(LCD_ClearScreen);
    send(LCD_SetCursor, 0x00);
}
//Set cursor lime 1 column 1
void NewHavenDisplay::home()
{
    send(LCD_CursorHome);
}

//Turn on / off Display
void NewHavenDisplay::display()
{
    send(LCD_DisplayOn);
}
void NewHavenDisplay::noDisplay()
{
    send(LCD_DisplayOff);
}

//Set cursor to user inputed line and column
void NewHavenDisplay::setCursor(uint8_t line, uint8_t column)
{
    uint8_t position = 0x00;
    switch (line)
    {
    case 1:
        position += 0x00;
        break;
    case 2:
        position += 0x40;
        break;
    case 3:
        position += 0x14;
        break;
    case 4:
        position += 0x54;
        break;
    }
    position += column;
    send(LCD_SetCursor, position);
}

//Turn on / off Underline
void NewHavenDisplay::underline()
{
    send(LCD_UnderlineOn);
}
void NewHavenDisplay::noUnderline()
{
    send(LCD_UnderlineOff);
}

//Move Cursor USER INPUTED TIMES COLUMN LEFT / RIGHT
void NewHavenDisplay::moveCursor(uint8_t direction, uint8_t column)
{
    for (int i = 0; i < column; i++)
    {
        if (direction == 'r')
        {
            send(LCD_MoveCursorRight);
        }
        else
        {
            send(LCD_MoveCursorLeft);
        }
    }
}
void NewHavenDisplay::moveCursorRight()
{
    send(LCD_MoveCursorRight);
}
void NewHavenDisplay::moveCursorLeft()
{
    send(LCD_MoveCursorLeft);
}

//Turn on / off blinking
void NewHavenDisplay::blink()
{
    send(LCD_BlinkCursorOn);
}
void NewHavenDisplay::noBlink()
{
    send(LCD_BlinkCursorOff);
}

//Erase 1 letter
void NewHavenDisplay::backspace()
{
    send(LCD_BackSpace);
}

//modify Display contrast (min 1 to max 50)
void NewHavenDisplay::setContrast(uint8_t contrast)
{
    send(LCD_SetContrast, contrast);
}

//modify Display backlight (min 1 to max 8)
void NewHavenDisplay::setBacklight(uint8_t brightness)
{
    send(LCD_SetBacklight, brightness);
}

//Move Display left / right with text
void NewHavenDisplay::shift(uint8_t direction, uint8_t column)
{
    for (int i = 0; i < column; i++)
    {
        if (direction == 'r')
        {
            send(LCD_MoveDisplayRight);
        }
        else
        {
            send(LCD_MoveDisplayLeft);
        }
    }
}
void NewHavenDisplay::rightShift()
{
    send(LCD_MoveDisplayRight);
}
void NewHavenDisplay::leftShift()
{
    send(LCD_MoveDisplayLeft);
}

//Change communicating address
void NewHavenDisplay::changeI2C(uint8_t address)
{
    send(LCD_ChangeI2C, address);
}
void NewHavenDisplay::changeRS232(uint8_t baudrate)
{
    send(LCD_ChangeRS232, baudrate);
}

//Display communicating address
void NewHavenDisplay::showI2C()
{
    send(LCD_DisplayI2C);
}
void NewHavenDisplay::showRS232()
{
    send(LCD_DisplayRS232);
}

//Display Firmware version
void NewHavenDisplay::showFirmware()
{
    send(LCD_DisplayVersion);
}