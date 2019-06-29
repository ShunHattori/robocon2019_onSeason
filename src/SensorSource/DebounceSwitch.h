#pragma once

#include "mbed.h"

class DebounceSwitch
{
public:
    DebounceSwitch(PinName, char);

    void update();

    bool stats()
    {
        return ButtonStats;
    }

private:
    DigitalIn _switch;
    int detectFlagNumber;
    bool ButtonStats;
    int PINMODE;
};