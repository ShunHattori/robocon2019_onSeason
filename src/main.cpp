#include "mbed.h"
#include "Encoder.h"
#include "MWodometry.h"
#include "LocationManager.h"
#include "OmniKinematics.h"
#include "MotorDriverAdapter.h"
/*
Encoder encoder_XAxis_1(PD_15, PF_12);
MWodometry odometry_XAxis_1(encoder_XAxis_1, 48, 5);
//LocationManager<int> Robot(45, 83, 125);
OmniKinematics wheel(4);
MotorDriverAdapter driveWheel(PB_10, PB_11, PE_12, PE_14, PD_12, PD_13, PE_8, PE_10);

Serial PC(USBTX, USBRX);
DigitalOut LED(LED1);
DigitalOut LEDtest(LED2);
InterruptIn button(USER_BUTTON);
InterruptIn enctest(PF_12);

void button_pressed()
{
    PC.printf("%d\n\r", Robot.getXData());
    PC.printf("%d\n\r", Robot.getYData());
    PC.printf("%d\n\r", Robot.getYawData());
}

void test()
{
    LEDtest = !LEDtest;
}
*/
int main()
{
    /*
    PC.baud(9600);
    button.mode(PullDown);
    button.rise(&button_pressed);
    enctest.rise(&test);
    // encoder_XAxis_1.setPulse(0);
    //PC.printf("got encoder pulse:%ld\r\n", encoder_XAxis_1.getPulse());
    LED = 1;

    //encoder_XAxis_1.EnableDebugOutput(PB_7);
    //Robot.setCurrentPoint(100, 642, 45);
    PC.printf("%d\n\r", Robot.getXLocationData());
    PC.printf("%d\n\r", Robot.getYLocationData());
    PC.printf("%d\n\r", Robot.getYawStatsData());
    Robot.setCurrentPoint(90, 42, 531);

    float output[4] = {}; 
    float vector[3] = {};
    wheel.setMaxPWM(0.9);
    wheel.getOutput(0.0, 1.0, 0.0, output);
    PC.printf("%f,%f,%f,%f", output[0], output[1], output[2], output[3]);
    driveWheel.apply(output);
    for (;;)
    {
        //PC.printf("encoder pulse:%ld\todometry value:%ld\n\r", encoder_XAxis_1.getPulse(), odometry_XAxis_1.getDistance());
        //wait(0.2);

    }
    */
}
