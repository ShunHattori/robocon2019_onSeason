#include "mbed.h"
#include "Encoder.h"
#include "MWodometry.h"
#include "LocationManager.h"
#include "DriveTrain.h"
#include "OmniKinematics.h"
#include "MotorDriverAdapter.h"
double vector[3] = {};

Encoder encoder_XAxis_2(PA_4, PB_4);
Encoder encoder_XAxis_1(PA_5, PA_6);
Encoder encoder_YAxis_1(PD_15, PF_12);

MWodometry odometry_XAxis_1(encoder_XAxis_1, 48, 5);
MWodometry odometry_XAxis_2(encoder_XAxis_2, 48, 5);
MWodometry odometry_YAxis_1(encoder_YAxis_1, 48, 5);

LocationManager<int> robotLocation(0, 0, 0);
DriveTrain algorithm(robotLocation, odometry_XAxis_1, odometry_XAxis_2, odometry_YAxis_1, 50, 5, 30);
OmniKinematics wheel(4);
MotorDriverAdapter driveWheel(PB_10, PB_11, PE_12, PE_14, PD_12, PD_13, PE_8, PE_10);

Serial PC(USBTX, USBRX);
DigitalOut LED(LED1);
DigitalOut LEDtest(LED2);
InterruptIn button(USER_BUTTON);
InterruptIn enctest(PF_12);
Ticker updateOutput;

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
    algorithm.setMaxOutput(9000);
    algorithm.setMinOutput(200);
    updateOutput.attach(callback(&algorithm, &DriveTrain::update), 0.2);
    // encoder_XAxis_1.setPulse(0);
    //PC.printf("got encoder pulse:%ld\r\n", encoder_XAxis_1.getPulse());
    LED = 1;

    //encoder_XAxis_1.EnableDebugOutput(PB_7);
    //robotLocation.setCurrentPoint(100, 642, 45);
    PC.printf("%lf\n\r", robotLocation.getXLocationData());
    PC.printf("%lf\n\r", robotLocation.getYLocationData());
    PC.printf("%lf\n\r", robotLocation.getYawStatsData());
    //robotLocation.setCurrentPoint(90, 42, 531);

    int output[4] = {};
    wheel.setMaxPWM(9000);
    for (;;)
    {
        //PC.printf("encoder pulse:%ld\todometry value:%ld\n\r", encoder_XAxis_1.getPulse(), odometry_XAxis_1.getDistance());
        //wait(0.2);
        PC.printf("%d,%d,%d\r\n", algorithm.getXVector(), algorithm.getYVector(), algorithm.getYawVector());
        wheel.getOutput(algorithm.getXVector(), algorithm.getYVector(), algorithm.getYawVector(), output);
        PC.printf("%d,%d,%d,%d\r\n", output[0], output[1], output[2], output[3]);
        driveWheel.apply(output);
    }
}
