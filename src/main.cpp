#include "mbed.h"
#include "Encoder.h"
#include "MWodometry.h"

Encoder encoder_XAxis_1(PD_15, PF_12);
MWodometry odometry_XAxis_1(encoder_XAxis_1, 48, 5);

Serial PC(USBTX, USBRX);
DigitalOut LED(LED1);
DigitalOut LEDtest(LED2);
InterruptIn button(USER_BUTTON);
InterruptIn enctest(PF_12);

void button_pressed()
{
    //PC.printf("Pulse:%ld\r\n",encoder_XAxis_1.getPulse());
}

void test()
{
    LEDtest = !LEDtest;
}

int main()
{
    PC.baud(9600);
    button.mode(PullDown);
    button.rise(&button_pressed);
    enctest.rise(&test);
    // encoder_XAxis_1.setPulse(0);
    //PC.printf("got encoder pulse:%ld\r\n", encoder_XAxis_1.getPulse());
    LED = 1;

    encoder_XAxis_1.EnableDebugOutput(PB_7);
    for (;;)
    {
        PC.printf("encoder pulse:%ld\todometry value:%ld\n\r", encoder_XAxis_1.getPulse(), odometry_XAxis_1.getDistance());
        wait(0.2);
    }
}
