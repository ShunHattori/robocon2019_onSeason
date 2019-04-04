#include "mbed.h"
#include "Encoder.h"
#include "MWodometry.h"
#include "LocationManager.h"
#include "DriveTrain.h"
#include "OmniKinematics.h"
#include "MotorDriverAdapter.h"

Encoder encoder_XAxis_1(PD_15, PF_12);
Encoder encoder_XAxis_2(PA_4, PB_4);
Encoder encoder_YAxis_1(PA_5, PA_6);
MWodometry odometry_XAxis_1(encoder_XAxis_1, 48, 5);
MWodometry odometry_XAxis_2(encoder_XAxis_2, 48, 5);
MWodometry odometry_YAxis_1(encoder_YAxis_1, 48, 5);

LocationManager<int> robotLocation(0, 0, 0);
DriveTrain algorithm(robotLocation,odometry_XAxis_1,odometry_XAxis_2,odometry_YAxis_1,50,5,30);
OmniKinematics wheel(4);
MotorDriverAdapter driveWheel(PB_10, PB_11, PE_12, PE_14, PD_12, PD_13, PE_8, PE_10);

Serial PC(USBTX, USBRX);
DigitalOut LED(LED1);
DigitalOut LEDtest(LED2);
InterruptIn button(USER_BUTTON);
InterruptIn enctest(PF_12);

void button_pressed()
{
    PC.printf("%d\n\r", robotLocation.getXLocationData());
    PC.printf("%d\n\r", robotLocation.getYLocationData());
    PC.printf("%d\n\r", robotLocation.getYawStatsData());
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

    //encoder_XAxis_1.EnableDebugOutput(PB_7);
    //robotLocation.setCurrentPoint(100, 642, 45);
    PC.printf("%d\n\r", robotLocation.getXLocationData());
    PC.printf("%d\n\r", robotLocation.getYLocationData());
    PC.printf("%d\n\r", robotLocation.getYawStatsData());
    robotLocation.setCurrentPoint(90, 42, 531);

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
}
