#include "DebounceSwitch.h"

DebounceSwitch::DebounceSwitch(PinName inputPin, char mode) : _switch(inputPin)
{
    if (mode == 'u' || mode == 'U')
    {
        _switch.mode(PullUp);
        PINMODE = 1;
    }
    else if (mode == 'd' || mode == 'D')
    {
        _switch.mode(PullDown);
        PINMODE = 2;
    }
    else
    {
        _switch.mode(PullNone);
        PINMODE = 3;
    }
    detectFlagNumber = 500;
    ButtonStats = 0;
}

void DebounceSwitch::update()
{
    int buttonPressCount = 0;

    switch (PINMODE)
    {
    case 1:
        for (int i = 0; i < 500; i++)
        {
            buttonPressCount += !_switch.read();
        }
        break;
    case 2:
        for (int i = 0; i < 500; i++)
        {
            buttonPressCount += _switch.read();
        }
        break;
    case 3:
        for (int i = 0; i < 500; i++)
        {
            buttonPressCount += _switch.read();
        }
        break;
    default:
        break;
    }

    if (buttonPressCount == 500)
    {
        ButtonStats = 1;
    }
    else
    {
        ButtonStats = 0;
    }
}