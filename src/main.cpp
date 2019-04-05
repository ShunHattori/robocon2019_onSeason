#include "mbed.h"
#include "Encoder.h"
#include "MWodometry.h"
#include "LocationManager.h"
#include "DriveTrain.h"
#include "OmniKinematics.h"
#include "MotorDriverAdapter.h"

/***********************************************************/
const uint8_t ENCODER_PULSE_PER_ROUND = 48;
const uint8_t ENCODER_ATTACHED_WHEEL_RADIUS = 5;
const uint8_t DISTANCE_BETWEEN_ENCODER_WHEELS = 60;
const uint8_t PERMIT_ERROR_CIRCLE_RADIUS = 5;
const uint8_t DECREASE_PWM_CIRCLE_RADIUS = 50;
const uint16_t ESTIMATE_MAX_PWM = 9000;
const uint16_t ESTIMATE_MIN_PWM = 500;
const float DRIVETRAIN_UPDATE_CYCLE = 0.2;

Encoder encoder_XAxis_2(PA_4, PB_4);
Encoder encoder_XAxis_1(PA_5, PA_6);
Encoder encoder_YAxis_1(PD_15, PF_12);
MWodometry odometry_XAxis_1(encoder_XAxis_1, ENCODER_PULSE_PER_ROUND, ENCODER_ATTACHED_WHEEL_RADIUS);
MWodometry odometry_XAxis_2(encoder_XAxis_2, ENCODER_PULSE_PER_ROUND, ENCODER_ATTACHED_WHEEL_RADIUS);
MWodometry odometry_YAxis_1(encoder_YAxis_1, ENCODER_PULSE_PER_ROUND, ENCODER_ATTACHED_WHEEL_RADIUS);
LocationManager<int> robotLocation(0, 0, 0);
DriveTrain accelAlgorithm(robotLocation, odometry_XAxis_1, odometry_XAxis_2, odometry_YAxis_1, DISTANCE_BETWEEN_ENCODER_WHEELS, PERMIT_ERROR_CIRCLE_RADIUS, DECREASE_PWM_CIRCLE_RADIUS);
Ticker updateOutput;
OmniKinematics wheel(4);
MotorDriverAdapter driveWheel(PB_10, PB_11, PE_12, PE_14, PD_12, PD_13, PE_8, PE_10);
int output[4] = {};
/***********************************************************/

Serial PC(USBTX, USBRX);
DigitalOut BUILD_IN_LED_YELLOW(LED1);
DigitalOut BUILD_IN_LED_BLUE(LED2);
InterruptIn button(USER_BUTTON);
InterruptIn DEBUG_ENCODER_PULSE(PF_12);

void button_pressed()
{
    PC.printf("%d\n\r", robotLocation.getXLocationData());
    PC.printf("%d\n\r", robotLocation.getYLocationData());
    PC.printf("%d\n\r", robotLocation.getYawStatsData());
}

void LED_BLUE_FRIPPER()
{
    BUILD_IN_LED_BLUE = !BUILD_IN_LED_BLUE;
}

int main()
{
    PC.baud(9600);
    button.mode(PullDown);
    button.rise(&button_pressed);
    DEBUG_ENCODER_PULSE.rise(&LED_BLUE_FRIPPER);
    BUILD_IN_LED_YELLOW = 1;

    accelAlgorithm.setMaxOutput(ESTIMATE_MAX_PWM);
    accelAlgorithm.setMinOutput(ESTIMATE_MIN_PWM);
    updateOutput.attach(callback(&accelAlgorithm, &DriveTrain::update), DRIVETRAIN_UPDATE_CYCLE);
    wheel.setMaxPWM(ESTIMATE_MAX_PWM);

    //encoder_XAxis_1.EnableDebugOutput(PB_7);
    //robotLocation.setCurrentPoint(100, 642, 45);
    //robotLocation.setCurrentPoint(90, 42, 531);

    for (;;)
    {
        //PC.printf("encoder pulse:%ld\todometry value:%ld\n\r", encoder_XAxis_1.getPulse(), odometry_XAxis_1.getDistance());
        //wait(0.2);
        PC.printf("%d,%d,%d\r\n", accelAlgorithm.getXVector(), accelAlgorithm.getYVector(), accelAlgorithm.getYawVector());
        wheel.getOutput(accelAlgorithm.getXVector(), accelAlgorithm.getYVector(), accelAlgorithm.getYawVector(), output);
        PC.printf("%d,%d,%d,%d\r\n", output[0], output[1], output[2], output[3]);
        driveWheel.apply(output);
    }
}
