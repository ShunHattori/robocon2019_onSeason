/**
#include "mbed.h"
#include "Encoder.h"
#include "MWodometry.h"
#include "LocationManager.h"
#include "OmniKinematics.h"
#include "MotorDriverAdapter.h"

Encoder encoder_XAxis_1(PA_0, PA_0);
Encoder encoder_XAxis_2(PA_0, PA_0);
Encoder encoder_YAxis(PA_0, PA_0);

MWodometry odometry_XAxis_1(encoder_XAxis_1, 48, 5);
MWodometry odometry_XAxis_2(encoder_XAxis_2, 48, 5);
MWodometry odometry_YAxis(encoder_YAxis, 48, 5);

LocationManager<int> LCM(25, 25, 0); //中心位置がちょっとずれてる想定
OmniKinematics Drive(4);
MotorDriverAdapter driveWheel(PA_0, PA_0, PA_0, PA_0, PA_0, PA_0, PA_0, PA_0);

Serial Monitor(USBTX, USBRX);
InterruptIn startButton(USER_BUTTON);
Ticker listener;

void button_pressed()
{
    LCM.addPoint(LCM.getXData(), LCM.getYData() + 100, 0);
}

int main()
{
    Monitor.baud(9600);
    startButton.mode(PullDown);
    startButton.rise(&button_pressed);
    listener.attach(callback(&LCM, &LocationManager::update()), 0.1);
    Drive.setMaxPWM(0.5);

    for (;;)
    {
        LCM.addPoint(100,500);
        while (!LCM.isHere(100, 500))
        {
        }
        fallout.launch(2);
        while (!fallout.done())
        {
        }
    }
}
*/