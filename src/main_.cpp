#include "mbed.h"
#include "MWodometry.h"
#include "LocationManager.h"
#include "DriveTrain.h"
#include "NewHavenDisplay.h"
#include "QEI.h"
#include "MPU9250.h"

//#define USING_4WD
#define USING_3WD
//#define TEST_COURSE_1
#define TEST_COURSE_2
//#define IMUSENSOR_TEST
//#define LOCATIONMANAGER_TEST
//#define NEWHAVENDISPLAY_TEST
//#define ZEAL_BTMODULE_TEST
//#define LIDARLITE_TEST
//#define ENABLE_LCD_DEBUG
//#define ENABLE_BOTTLE_FLIP

/***********************************************************/
/*
*   ENCODER_PULSE_PER_ROUND         :   取り付けエンコーダの分解能
*   ENCODER_ATTACHED_WHEEL_RADIUS   :   計測輪の半径
*   DISTANCE_BETWEEN_ENCODER_WHEELS :   同軸の計測輪取り付け距離
*   PERMIT_ERROR_CIRCLE_RADIUS      :   停止地点の許容誤差判定円の半径
*   DECREASE_PWM_CIRCLE_RADIUS      :   減速開始判定円の半径
*   ESTIMATE_MAX_PWM                :   MDに出力される想定最大PWM
*   ESTIMATE_MIN_PWM                :   MDに出力される想定最小PWM
*   DRIVETRAIN_UPDATE_CYCLE         :   速度計算アルゴリズムの更新周期(s)
*/
#define ENCODER_PULSE_PER_ROUND 48
#define ENCODER_ATTACHED_WHEEL_RADIUS_BY_NEXUS_ROBOT 5.0
#define ENCODER_ATTACHED_WHEEL_RADIUS_BY_HANGFA 5.08
#define DISTANCE_BETWEEN_ENCODER_WHEELS 72
#define PERMIT_ERROR_CIRCLE_RADIUS 2 //3.5
#define DECREASE_PWM_CIRCLE_RADIUS 120
#define ESTIMATE_MAX_PWM 0.6
#define ESTIMATE_MIN_PWM 0.04
#define DRIVETRAIN_UPDATE_CYCLE 0.15


#ifdef USING_4WD
#include "OmniKinematics4WD.h"
#include "MotorDriverAdapter4WD.h"
OmniKinematics4WD wheel;
MotorDriverAdapter4WD driveWheel(PB_10, PB_11, PE_12, PE_14, PD_12, PD_13, PE_8, PE_10);
float output[4] = {};
#endif // USING_4WD

#ifdef USING_3WD
#include "OmniKinematics3WD.h"
#include "MotorDriverAdapter3WD.h"
OmniKinematics3WD wheel;
MotorDriverAdapter3WD driveWheel(PB_10, PB_11, PE_12, PE_14, PD_12, PD_13);
float output[3] = {};
#endif // USING_3WD

Timer QEITimer;
MPU9250 IMU;
QEI encoder_XAxis_1(PB_5, PC_7, NC, ENCODER_PULSE_PER_ROUND, &QEITimer,QEI::X2_ENCODING);
QEI encoder_YAxis_1(PF_13, PE_9, NC, ENCODER_PULSE_PER_ROUND, &QEITimer,QEI::X2_ENCODING);
MWodometry odometry_XAxis_1(encoder_XAxis_1, ENCODER_PULSE_PER_ROUND, ENCODER_ATTACHED_WHEEL_RADIUS_BY_HANGFA);
MWodometry odometry_YAxis_1(encoder_YAxis_1, ENCODER_PULSE_PER_ROUND, ENCODER_ATTACHED_WHEEL_RADIUS_BY_HANGFA);
LocationManager<int> robotLocation(0, 0, 0);
DriveTrain accelAlgorithm(robotLocation, odometry_XAxis_1, odometry_YAxis_1, IMU, PERMIT_ERROR_CIRCLE_RADIUS, DECREASE_PWM_CIRCLE_RADIUS);
Ticker updateOutput;

/***********************************************************/

Serial PC(USBTX, USBRX);
DigitalOut shoot1(PG_0); //フリップ用信号ピン
DigitalOut shoot2(PD_1); //フリップ用信号ピン
DigitalOut IMUReady(LED2);

#ifdef ENABLE_LCD_DEBUG
NewHavenDisplay LCDmanager(LCD);
Ticker debugLCD;
void debug_LCD()
{
    LCDmanager.clear();
    //LCD.printf("CP:%.2lf,%.2lf,%.2lf", accelAlgorithm.getCurrentXPosition(), accelAlgorithm.getCurrentYPosition(), accelAlgorithm.getCurrentYawPosition());
    LCD.printf("%d,%d,%d", encoder_XAxis_1.getPulses() / 4, SUBencoder.getPulses(), encoder_YAxis_1.getPulses());
    LCDmanager.setCursor(2, 0);
    LCD.printf("TP:%d,%d,%d", robotLocation.getXLocationData(), robotLocation.getYLocationData(), robotLocation.getYawStatsData());
    LCDmanager.setCursor(3, 0);
    LCD.printf("CV:%.2lf,%.2lf,%.2lf", accelAlgorithm.getXVector(), accelAlgorithm.getYVector(), accelAlgorithm.getYawVector());
}
#endif
#ifdef LIDARLITE_TEST
#include "LidarLite.h"
#define LIDARLite1_SDA PB_9                        //SDA pin on 767ZI
#define LIDARLite1_SCL PB_8                        //SCL pin on 767ZI
LidarLite sensor1(LIDARLite1_SDA, LIDARLite1_SCL); //Define LIDAR Lite sensor 1
Timer dt;
#endif

int main()
{
    PC.baud(9600);
    IMU.setup(PB_9, PB_8);
    accelAlgorithm.setMaxOutput(ESTIMATE_MAX_PWM);
    accelAlgorithm.setMinOutput(ESTIMATE_MIN_PWM);
    updateOutput.attach(callback(&accelAlgorithm, &DriveTrain::update), DRIVETRAIN_UPDATE_CYCLE);
    wheel.setMaxPWM(ESTIMATE_MAX_PWM);
    IMUReady = 1;
#ifdef IMUSENSOR_TEST
    for(;;){ /ジャイロテスト
        PC.printf("%.2lf\r\n",IMU.gyro_Yaw());
    }
#endif
#ifdef ENABLE_LCD_DEBUG
    debugLCD.attach(debug_LCD, 0.2);
#endif
#ifdef NEWHAVENDISPLAY_TEST
    LCDmanager.clear();
    LCDmanager.home();
    LCDmanager.display();
    LCDmanager.setContrast(50);
    LCD.printf("device ready");
    LCDmanager.setCursor(2, 5);
    LCD.printf("cursor moved");
#endif
#ifdef LIDARLITE_TEST
    dt.start();
    while (1)
    {
        //sensor1.refreshRange();
        //sensor1.refreshVelocity();
        sensor1.refreshRange();
        PC.printf("range: %d cm, rate: %.2f Hz\n\r", sensor1.getRange_cm(), 1 / dt.read());
        dt.reset();

        robotLocation.addPoint(0, 100, 0);
        robotLocation.sendNext(); //0,100が参照可能になる
        accelAlgorithm.switchMode();
        while (!robotLocation.checkMovingStats(accelAlgorithm.getStats()))
        {
            accelAlgorithm.setSensorDistance(sensor1.getRange_cm());
            wheel.getOutput(accelAlgorithm.getXVector(), accelAlgorithm.getYVector(), accelAlgorithm.getYawVector(), output);
            driveWheel.apply(output);
            PC.printf("%d\r\n", accelAlgorithm.getCurrentYPosition());
            sensor1.refreshRange();
            PC.printf("range: %d cm, rate: %.2f Hz\n\r", sensor1.getRange_cm(), 1 / dt.read());
        }
    }
#endif

    for (;;)
    {
#ifdef ZEAL_BTMODULE_TEST

        static char buffer[32];
        static uint8_t bfCount = 0;
        if (Zeal.readable())
        {
            LCD.putc(Zeal.getc());
        }
        if (Zeal.readable())
        {
            PC.printf("%d\r\n", bfCount);
            buffer[bfCount] = Zeal.getc();
            if (buffer[bfCount] == '\n')
            {
                LCD.putc(0xFE);
                LCD.putc(0x51);
                LCD.putc(0xFE);
                LCD.putc(0x46);
                for (int i = 0; i <= bfCount - 2; i++)
                {
                    LCD.putc(buffer[i]);
                }
                bfCount = 0;
            }
            else
            {
                bfCount++;
            }
        }
#endif
#ifdef LOCATIONMANAGER_TEST
        robotLocation.addPoint(130, 0, 0);
        robotLocation.addPoint(130, -90, 0);
        robotLocation.addPoint(0, 0, 0);
        robotLocation.sendNext();
        while (!robotLocation.checkMovingStats(accelAlgorithm.getStats()))
        {
            accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
            wheel.getOutput(accelAlgorithm.getXVector(), accelAlgorithm.getYVector(), accelAlgorithm.getYawVector(), output);
            driveWheel.apply(output);
        }
        for (int i = 0; i < 100; i++) //完全停止用
        {
            accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
            wheel.getOutput(0, 0, 0, output);
            driveWheel.apply(output);
        }
#ifdef ENABLE_BOTTLE_FLIP
        shoot1 = 1;
        shoot2 = 0;
        wait(1.5);
        shoot1 = 0;
        shoot2 = 1;
        wait(1);
        shoot1 = 0;
        shoot2 = 0;
        wait(1.5);
#endif
        robotLocation.sendNext();
        while (!robotLocation.checkMovingStats(accelAlgorithm.getStats()))
        {
            accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
            wheel.getOutput(accelAlgorithm.getXVector(), accelAlgorithm.getYVector(), accelAlgorithm.getYawVector(), output);
            driveWheel.apply(output);
        }
        robotLocation.sendNext();
        while (!robotLocation.checkMovingStats(accelAlgorithm.getStats()))
        {
            accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
            wheel.getOutput(accelAlgorithm.getXVector(), accelAlgorithm.getYVector(), accelAlgorithm.getYawVector(), output);
            driveWheel.apply(output);
        }
        while (1)
        {
            accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
            wheel.getOutput(accelAlgorithm.getXVector(), accelAlgorithm.getYVector(), accelAlgorithm.getYawVector(), output);
            driveWheel.apply(output);
        }
#endif

#ifdef TEST_COURSE_1
        robotLocation.addPoint(100, 0, 45);
        robotLocation.addPoint(0, 0, 0);
        robotLocation.sendNext();
        while (!robotLocation.checkMovingStats(accelAlgorithm.getStats()))
        {
            accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
            wheel.getOutput(accelAlgorithm.getXVector(), accelAlgorithm.getYVector(), accelAlgorithm.getYawVector(), output);
            driveWheel.apply(output);
        }
        for (int i = 0; i < 100; i++) //完全停止用
        {
            accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
            wheel.getOutput(0, 0, 0, output);
            driveWheel.apply(output);
        }
        robotLocation.sendNext();
        while (!robotLocation.checkMovingStats(accelAlgorithm.getStats()))
        {
            accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
            wheel.getOutput(accelAlgorithm.getXVector(), accelAlgorithm.getYVector(), accelAlgorithm.getYawVector(), output);
            driveWheel.apply(output);
        }
        while (1)
        {
            accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
            wheel.getOutput(accelAlgorithm.getXVector(), accelAlgorithm.getYVector(), accelAlgorithm.getYawVector(), output);
            driveWheel.apply(output);
        }
#endif
#ifdef TEST_COURSE_2
        robotLocation.addPoint(170, 0, 0);
        robotLocation.addPoint(170, -140, 0);
        robotLocation.addPoint(0, -140, 0);
        robotLocation.addPoint(0, 0, 0);

        robotLocation.sendNext();
        while (!robotLocation.checkMovingStats(accelAlgorithm.getStats()))
        {
            accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
            wheel.getOutput(accelAlgorithm.getXVector(), accelAlgorithm.getYVector(), accelAlgorithm.getYawVector(), output);
            driveWheel.apply(output);
        }
        robotLocation.sendNext();
        while (!robotLocation.checkMovingStats(accelAlgorithm.getStats()))
        {
            accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
            wheel.getOutput(accelAlgorithm.getXVector(), accelAlgorithm.getYVector(), accelAlgorithm.getYawVector(), output);
            driveWheel.apply(output);
        }
        robotLocation.sendNext();
        while (!robotLocation.checkMovingStats(accelAlgorithm.getStats()))
        {
            accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
            wheel.getOutput(accelAlgorithm.getXVector(), accelAlgorithm.getYVector(), accelAlgorithm.getYawVector(), output);
            driveWheel.apply(output);
        }
        robotLocation.sendNext();
        while (!robotLocation.checkMovingStats(accelAlgorithm.getStats()))
        {
            accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
            wheel.getOutput(accelAlgorithm.getXVector(), accelAlgorithm.getYVector(), accelAlgorithm.getYawVector(), output);
            driveWheel.apply(output);
        }
        while (1)
        {
            accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
            wheel.getOutput(accelAlgorithm.getXVector(), accelAlgorithm.getYVector(), accelAlgorithm.getYawVector(), output);
            driveWheel.apply(output);
        }
#endif
    }
    while (1)
    {
        //192.168.163.
        //192.168.163.140~169
    }
}
