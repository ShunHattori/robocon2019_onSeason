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
//#define TEST_COURSE_2
//#define TEST_COURSE_GAME
//#define TEST_COURSE_STRAIGHT
//#define TEST_SL
//#define TEST_MECA_SHEET
//#define TEST_ROJAR_ENCODER
//#define TEST_SHEET_LAUNCH_ENCODER
//#define TEST_ROJAR_MOTOR
//#define TEST_SHEET_LAUNCH_MOTOR
//#define TEST_SHHET_LAUNCH
//#define TEST_SOLENOID_SEQ
//#define TEST_SOL_SHEET
#define TEST_MOTOR_SHEET
//#define TEST_FEET_LOOP
//#define TEST_CATCH_MOTOR_STOLE
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
#define PERMIT_ERROR_CIRCLE_RADIUS 3.5 // 3.5
#define DECREASE_PWM_CIRCLE_RADIUS 150
#define ESTIMATE_MAX_PWM 0.64 // max:0.7, recommend:0.64
#define ESTIMATE_MIN_PWM 0.09
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
MotorDriverAdapter3WD driveWheel(PB_11, PB_10, PE_12, PE_14, PD_12, PD_13);
float output[3] = {};
#endif // USING_3WD

Timer QEITimer;
MPU9250 IMU;
QEI encoder_XAxis_1(PE_9, PF_13, NC, ENCODER_PULSE_PER_ROUND, &QEITimer, QEI::X2_ENCODING);
QEI encoder_YAxis_1(PB_5, PC_7, NC, ENCODER_PULSE_PER_ROUND, &QEITimer, QEI::X2_ENCODING);
MWodometry odometry_XAxis_1(encoder_XAxis_1, ENCODER_PULSE_PER_ROUND, ENCODER_ATTACHED_WHEEL_RADIUS_BY_HANGFA);
MWodometry odometry_YAxis_1(encoder_YAxis_1, ENCODER_PULSE_PER_ROUND, ENCODER_ATTACHED_WHEEL_RADIUS_BY_HANGFA);
LocationManager<int> robotLocation(0, 0, 0);
DriveTrain accelAlgorithm(robotLocation, odometry_XAxis_1, odometry_YAxis_1, IMU, PERMIT_ERROR_CIRCLE_RADIUS, DECREASE_PWM_CIRCLE_RADIUS);
Ticker updateOutput;

/***********************************************************/

Serial PC(USBTX, USBRX);
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
#define LIDARLite1_SDA PF_15                       //SDA pin on 767ZI
#define LIDARLite1_SCL PF_14                       //SCL pin on 767ZI
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
    for (;;)
    {
        PC.printf("%.2lf\r\n", IMU.gyro_Yaw());
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
    //sensor1.refreshRange();
    //sensor1.refreshVelocity();
    sensor1.refreshRange();
    PC.printf("range: %d cm, rate: %.2f Hz\n\r", sensor1.getRange_cm(), 1 / dt.read());
    dt.reset();

    robotLocation.addPoint(100, 0, 0);
    robotLocation.sendNext(); //100,0が参照可能になる
    accelAlgorithm.switchMode();
    while (1)
    {
        static int prevScanRange = 0;
        int filleredRange = (0.9 * sensor1.getRange_cm()) + ((1 - 0.9) * prevScanRange);
        prevScanRange = filleredRange;
        accelAlgorithm.setSensorDistance(filleredRange);
        accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
        wheel.getOutput(accelAlgorithm.getXVector(), accelAlgorithm.getYVector(), accelAlgorithm.getYawVector(), output);
        driveWheel.apply(output);
        PC.printf("%lf\r\n", accelAlgorithm.getCurrentYPosition());
        sensor1.refreshRange();
        PC.printf("range: %d cm, rate: %.2f Hz\n\r", sensor1.getRange_cm(), 1 / dt.read());
    }
#endif

    for (;;)
    {
#ifdef TEST_SL
        robotLocation.addPoint(0, 0, 0);
        robotLocation.sendNext();
        while (!robotLocation.checkMovingStats(accelAlgorithm.getStats()))
        {
            accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
            wheel.getOutput(accelAlgorithm.getXVector(), accelAlgorithm.getYVector(), accelAlgorithm.getYawVector(), output);
            driveWheel.apply(output);
        }
        while (1)
        {
            //PC.printf("%d,%d\r\n", encoder_XAxis_1.getPulses(), encoder_YAxis_1.getPulses());
            PC.printf("%lf,%lf,%lf\r\n", accelAlgorithm.getXVector(), accelAlgorithm.getYVector(), accelAlgorithm.getYawVector());
            accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
            wheel.getOutput(accelAlgorithm.getXVector(), accelAlgorithm.getYVector(), accelAlgorithm.getYawVector(), output);
            driveWheel.apply(output);
        }
#endif
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
        robotLocation.addPoint(0, 135, 0);
        robotLocation.addPoint(0, 0, 0);

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
#ifdef TEST_COURSE_STRAIGHT
        DigitalIn startButton(PG_2);
        startButton.mode(PullUp);
        while (1)
        {
            static bool buttonPressed = 0;
            int buttonPressCount = 0;
            for (int i = 0; i < 10000; i++)
            {
                buttonPressCount += !startButton.read();
            }
            if (buttonPressCount == 10000)
            {
                buttonPressed = 1;
            }
            if (buttonPressed)
            {
                buttonPressed = 0;
                break;
            }
        }
        robotLocation.addPoint(0, 600, 0); // 一度目のアプローチ
        robotLocation.addPoint(0, 300, 0);
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
        wait(3);
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
        for (int i = 0; i < 100; i++) //完全停止用
        {
            accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
            wheel.getOutput(0, 0, 0, output);
            driveWheel.apply(output);
        }
#endif
#ifdef TEST_COURSE_GAME
        DigitalIn startButton(PG_2);
        startButton.mode(PullUp);
        while (1)
        {
            static bool buttonPressed = 0;
            int buttonPressCount = 0;
            for (int i = 0; i < 10000; i++)
            {
                buttonPressCount += !startButton.read();
            }
            if (buttonPressCount == 10000)
            {
                buttonPressed = 1;
            }
            if (buttonPressed)
            {
                buttonPressed = 0;
                break;
            }
        }
        robotLocation.addPoint(0, 570, 0); // 一度目のアプローチ
        robotLocation.addPoint(180, 570, 0);
        robotLocation.addPoint(180, 630, 0);
        robotLocation.addPoint(330, 630, 0);
        robotLocation.addPoint(330, 570, 0);
        robotLocation.addPoint(0, 570, 0); //フェンスに当たる可能性があるからあえて横にずらす
        robotLocation.addPoint(0, 10, 0);
        for (int i = 0; i < 7; i++)
        {
            robotLocation.sendNext();
            while (!robotLocation.checkMovingStats(accelAlgorithm.getStats()))
            {
                accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
                wheel.getOutput(accelAlgorithm.getXVector(), accelAlgorithm.getYVector(), accelAlgorithm.getYawVector(), output);
                driveWheel.apply(output);
            }
        }
        for (int i = 0; i < 100; i++) //完全停止用
        {
            accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
            wheel.getOutput(0, 0, 0, output);
            driveWheel.apply(output);
        }

        while (1)
        {
            static bool buttonPressed = 0;
            int buttonPressCount = 0;
            for (int i = 0; i < 10000; i++)
            {
                buttonPressCount += !startButton.read();
            }
            if (buttonPressCount == 10000)
            {
                buttonPressed = 1;
            }
            if (buttonPressed)
            {
                buttonPressed = 0;
                break;
            }
        }
        robotLocation.addPoint(0, 370, 0); //二度目のアプローチ
        robotLocation.addPoint(180, 370, 0);
        robotLocation.addPoint(180, 430, 0);
        robotLocation.addPoint(255, 430, 0);
        robotLocation.addPoint(330, 430, 0);
        robotLocation.addPoint(330, 370, 0);
        robotLocation.addPoint(0, 370, 0); //フェンスに当たる可能性があるからあえて横にずらす
        robotLocation.addPoint(0, 0, 0);
        for (int i = 0; i < 8; i++)
        {
            robotLocation.sendNext();
            while (!robotLocation.checkMovingStats(accelAlgorithm.getStats()))
            {
                accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
                wheel.getOutput(accelAlgorithm.getXVector(), accelAlgorithm.getYVector(), accelAlgorithm.getYawVector(), output);
                driveWheel.apply(output);
            }
        }
        for (int i = 0; i < 100; i++) //完全停止用
        {
            accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
            wheel.getOutput(0, 0, 0, output);
            driveWheel.apply(output);
        }

#endif //TEST_COURSE_GAME
#ifdef TEST_MECA_SHEET
        DigitalIn startButton(PG_2);
        startButton.mode(PullUp);
        while (1)
        {
            static bool buttonPressed = 0;
            int buttonPressCount = 0;
            for (int i = 0; i < 10000; i++)
            {
                buttonPressCount += !startButton.read();
            }
            if (buttonPressCount == 10000)
            {
                buttonPressed = 1;
            }
            if (buttonPressed)
            {
                buttonPressed = 0;
                break;
            }
        } /*
        robotLocation.addPoint(0, -100, 0);
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
        }*/
        QEI rojarArm(PG_0, PD_1, NC, ENCODER_PULSE_PER_ROUND, &QEITimer, QEI::X4_ENCODING);
        PwmOut rojarArmCW(PF_9);
        PwmOut rojarArmCCW(PF_7);
        rojarArmCW.period_us(100);
        rojarArmCCW.period_us(100);
        while (1)
        { //展開
            if (rojarArm.getPulses() < 2790)
            {
                rojarArmCW.write(0.35);
                rojarArmCCW.write(0);
            }
            else
            {
                rojarArmCW.write(0);
                rojarArmCCW.write(0);
                break;
            }
        }
        wait(0.5); //一時的な停止 //5 sec

        PwmOut sheetLaunchCW(PE_5);
        PwmOut sheetLaunchCCW(PE_6);
        sheetLaunchCW.period_us(100);
        sheetLaunchCCW.period_us(100);
        sheetLaunchCW.write(0.9);
        sheetLaunchCCW.write(0);
        wait(4.2);
        sheetLaunchCW.write(0);
        sheetLaunchCCW.write(0.9);
        wait(4.1);
        sheetLaunchCW.write(0);
        sheetLaunchCCW.write(0.0);
        /*while (1)
        { //押し出す機構

        }*/
        DigitalOut solenoidCatchPull(PC_0);
        DigitalOut solenoidCatchPush(PA_3);
        DigitalOut solenoidUp(PF_5);
        DigitalOut solenoidDown(PF_10);
        solenoidCatchPull = 1;
        wait(0.35);
        solenoidCatchPull = 0;
        solenoidUp = 1;
        wait(0.35);
        solenoidUp = 0;
        while (1) //移動する代わりにボタン
        {
            static bool buttonPressed = 0;
            int buttonPressCount = 0;
            for (int i = 0; i < 10000; i++)
            {
                buttonPressCount += !startButton.read();
            }
            if (buttonPressCount == 10000)
            {
                buttonPressed = 1;
            }
            if (buttonPressed)
            {
                buttonPressed = 0;
                break;
            }
        }
        /*
        robotLocation.addPoint(-150, -100, 0);
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
        }*/
        solenoidCatchPush = 1;
        wait(0.35);
        solenoidDown = 1;
        solenoidCatchPush = 0;
        wait(0.35);
        solenoidDown = 0;
        while (1)
        { //展開縮小
            if (rojarArm.getPulses() > 0)
            {
                rojarArmCW.write(0);
                rojarArmCCW.write(0.175);
            }
            else
            {
                rojarArmCW.write(0);
                rojarArmCCW.write(0);
                break;
            }
        } /*
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
        }*/
#endif    //TEST_MECA_SHEET
#ifdef TEST_ROJAR_ENCODER
        QEI rojarArm(PG_0, PD_1, NC, ENCODER_PULSE_PER_ROUND, &QEITimer, QEI::X4_ENCODING);
        while (1)
        {
            PC.printf("ENCODER:%d\r\n", rojarArm.getPulses());
        }
#endif //TEST_ROJAR_ENCODER
#ifdef TEST_SHEET_LAUNCH_ENCODER
        QEI sheetLaunch(PE_2, PD_11, NC, ENCODER_PULSE_PER_ROUND, &QEITimer, QEI::X4_ENCODING);
        while (1)
        {//0,1,5,7,9,13
            PC.printf("ENCODER:%d\r\n", sheetLaunch.getPulses()); //-447
        }
#endif //TEST_SHEET_LAUNCH_ENCODER
#ifdef TEST_ROJAR_MOTOR //回転方向確認用
        PwmOut rojarArmCW(PF_9);
        PwmOut rojarArmCCW(PF_7);
        rojarArmCW.period_us(100);
        rojarArmCCW.period_us(100);
        while (1)
        { //回転方向確認用
            rojarArmCW.write(0.3);
            rojarArmCCW.write(0);
        }
#endif //TEST_ROJAR_MOTOR
#ifdef TEST_SHEET_LAUNCH_MOTOR //回転方向確認用
        PwmOut sheetLaunchCW(PA_0);
        PwmOut sheetLaunchCCW(PF_8);
        sheetLaunchCW.period_us(100);
        sheetLaunchCCW.period_us(100);
        while (1)
        { //回転方向確認用
            sheetLaunchCW.write(0);
            sheetLaunchCCW.write(0.3);
        }
#endif //TEST_SHEET_LAUNCH_MOTOR
#ifdef TEST_SHHET_LAUNCH
        PwmOut sheetLaunchCW(PE_5);
        PwmOut sheetLaunchCCW(PE_6);
        sheetLaunchCW.period_us(100);
        sheetLaunchCCW.period_us(100);
        sheetLaunchCW.write(0.2);
        sheetLaunchCCW.write(0);
        wait(1.5);
        sheetLaunchCW.write(0);
        sheetLaunchCCW.write(0.2);
        wait(1.5);
        sheetLaunchCW.write(0);
        sheetLaunchCCW.write(0.0);
#endif //TEST_SHHET_LAUNCH
#ifdef TEST_SOLENOID_SEQ
        DigitalOut solenoidCatchPull(PC_0);
        DigitalOut solenoidCatchPush(PA_3);
        DigitalOut solenoidUp(PF_5);
        DigitalOut solenoidDown(PF_10);
        solenoidCatchPull.write(1);
        wait(0.35);
        solenoidCatchPull.write(0);
        solenoidUp.write(1);
        wait(0.35);
        solenoidUp.write(0);
        wait(1); //本来は移動
        solenoidCatchPush.write(1);
        wait(0.35);
        solenoidDown.write(1);
        solenoidCatchPush.write(0);
        wait(0.35);
        solenoidDown.write(0);
        while (1)
        {
        }
#endif
#ifdef TEST_SOL_SHEET
        DigitalIn startButton(PG_2);
        startButton.mode(PullUp);
        while (1)
        {
            static bool buttonPressed = 0;
            int buttonPressCount = 0;
            for (int i = 0; i < 10000; i++)
            {
                buttonPressCount += !startButton.read();
            }
            if (buttonPressCount == 10000)
            {
                buttonPressed = 1;
            }
            if (buttonPressed)
            {
                buttonPressed = 0;
                break;
            }
        }
        DigitalOut solenoidCatchPull(PF_5);
        DigitalOut solenoidCatchPush(PC_0);
        DigitalOut solenoidUp(PA_3);
        DigitalOut solenoidDown(PF_10);
        solenoidCatchPush.write(1); //ボタン押したら開く
        wait(0.4);
        solenoidCatchPush.write(0);
        while (1)
        {
            static bool buttonPressed = 0;
            int buttonPressCount = 0;
            for (int i = 0; i < 10000; i++)
            {
                buttonPressCount += !startButton.read();
            }
            if (buttonPressCount == 10000)
            {
                buttonPressed = 1;
            }
            if (buttonPressed)
            {
                buttonPressed = 0;
                break;
            }
        }
        solenoidCatchPull = 1; //ボタン押したら閉じる
        wait(0.4);
        solenoidCatchPull = 0;
        while (1) //スタート
        {
            static bool buttonPressed = 0;
            int buttonPressCount = 0;
            for (int i = 0; i < 10000; i++)
            {
                buttonPressCount += !startButton.read();
            }
            if (buttonPressCount == 10000)
            {
                buttonPressed = 1;
            }
            if (buttonPressed)
            {
                buttonPressed = 0;
                break;
            }
        }
        PwmOut sheetLaunchCW(PE_5);
        PwmOut sheetLaunchCCW(PE_6);
        sheetLaunchCW.period_us(100);
        sheetLaunchCCW.period_us(100);
        sheetLaunchCW.write(0.9);
        sheetLaunchCCW.write(0);
        wait(4.2);
        sheetLaunchCW.write(0);
        sheetLaunchCCW.write(0.9);
        wait(4.1);
        sheetLaunchCW.write(0);
        sheetLaunchCCW.write(0);
        solenoidUp = 1;
        wait(0.2);
        solenoidUp = 0;
        wait(0.5); //上にあげる

        robotLocation.addPoint(-75, 0, 0);
        robotLocation.sendNext();
        Timer teisi;
        while (1)
        {
            PC.printf("%lf,%lf,%lf\r\n", accelAlgorithm.getCurrentXPosition(), accelAlgorithm.getCurrentYPosition(), accelAlgorithm.getCurrentYawPosition());
            accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
            wheel.getOutput(accelAlgorithm.getXVector(), accelAlgorithm.getYVector(), accelAlgorithm.getYawVector(), output);
            driveWheel.apply(output);
            if (robotLocation.checkMovingStats(accelAlgorithm.getStats()))
            {
                static bool initialFlag = true;
                if (initialFlag)
                {
                    teisi.start();
                    initialFlag = false;
                }
                if (teisi.read_ms() > 2000)
                {
                    break;
                }
            }
        }
        for (int i = 0; i < 100; i++) //完全停止用
        {
            accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
            wheel.getOutput(0, 0, 0, output);
            driveWheel.apply(output);
        }

        solenoidCatchPush.write(1);
        wait(0.35);
        solenoidCatchPush = 0;
        solenoidDown.write(1);
        wait(0.35);
        solenoidDown.write(0);

        robotLocation.addPoint(-75, 30, 0);
        robotLocation.sendNext();
        while (!robotLocation.checkMovingStats(accelAlgorithm.getStats()))
        {
            accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
            wheel.getOutput(accelAlgorithm.getXVector(), accelAlgorithm.getYVector(), accelAlgorithm.getYawVector(), output);
            driveWheel.apply(output);
        }
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
#endif //TEST_SOL_SHEET
#ifdef TEST_MOTOR_SHEET
        DigitalIn startButton(PG_2);
        startButton.mode(PullUp);
        while (1)
        {
            static bool buttonPressed = 0;
            int buttonPressCount = 0;
            for (int i = 0; i < 10000; i++)
            {
                buttonPressCount += !startButton.read();
            }
            if (buttonPressCount == 10000)
            {
                buttonPressed = 1;
            }
            if (buttonPressed)
            {
                buttonPressed = 0;
                break;
            }
        }
        PwmOut catchRightCW(PC_8);
        PwmOut catchRightCCW(PC_9);
        PwmOut catchLeftCW(PE_5);
        PwmOut catchLeftCCW(PE_6);
        catchRightCW.period_us(100);
        catchRightCCW.period_us(100);
        catchLeftCW.period_us(100);
        catchLeftCCW.period_us(100);

        catchRightCW.write(0.7); //閉じる
        catchRightCCW.write(0);
        catchLeftCW.write(0.7); //閉じる
        catchLeftCCW.write(0);
        wait(0.2);
        catchRightCW.write(0.15);
        catchRightCCW.write(0);
        catchLeftCW.write(0.15);
        catchLeftCCW.write(0);
        wait(1.2);

        QEI rojarArm(PG_0, PD_1, NC, ENCODER_PULSE_PER_ROUND, &QEITimer, QEI::X4_ENCODING);
        PwmOut rojarArmCW(PF_9);
        PwmOut rojarArmCCW(PF_7);
        rojarArmCW.period_us(100);
        rojarArmCCW.period_us(100);
        while (1)
        { //展開
            if (rojarArm.getPulses() < 1400) //1370 def
            {
                rojarArmCW.write(0.35);
                rojarArmCCW.write(0);
            }
            else
            {
                rojarArmCW.write(0);
                rojarArmCCW.write(0);
                break;
            }
        }
        QEI sheetLaunch(PE_2, PD_11, NC, ENCODER_PULSE_PER_ROUND, &QEITimer, QEI::X4_ENCODING);
        PwmOut sheetLaunchCW(PA_0);
        PwmOut sheetLaunchCCW(PF_8);
        sheetLaunchCW.period_us(100);
        sheetLaunchCCW.period_us(100);
        while (1)
        { //シーツかける
            PC.printf("ENCODER:%d\r\n", sheetLaunch.getPulses()); //-447
            if (sheetLaunch.getPulses() < 1375)
            {
                sheetLaunchCW.write(0.85);
                sheetLaunchCCW.write(0);
            }
            else
            {
                sheetLaunchCW.write(0);
                sheetLaunchCCW.write(0);
                break;
            }
        }
        while (1)
        { //縮小
            if (sheetLaunch.getPulses() > 0)
            {
                sheetLaunchCW.write(0);
                sheetLaunchCCW.write(0.85);
            }
            else
            {
                sheetLaunchCW.write(0);
                sheetLaunchCCW.write(0);
                break;
            }
        }

        /*catchLeftCW.write(0); //進行方向のモータを開放
        catchLeftCCW.write(0.3);
        wait(0.5);
        catchLeftCW.write(0.3); //閉じる
        catchLeftCCW.write(0);
        wait(0.5);*/
        catchRightCW.write(0); //反対側を開放
        catchRightCCW.write(0.2);
        catchLeftCW.write(0.175);
        catchLeftCCW.write(0);
        wait(0.5);
        catchRightCW.write(0);
        catchRightCCW.write(0);
        while (1)
        {
            static bool buttonPressed = 0;
            int buttonPressCount = 0;
            for (int i = 0; i < 10000; i++)
            {
                buttonPressCount += !startButton.read();
            }
            if (buttonPressCount == 10000)
            {
                buttonPressed = 1;
            }
            if (buttonPressed)
            {
                buttonPressed = 0;
                break;
            }
        }
        robotLocation.addPoint(-220, 0, 0); // short -37 ,long -94, sheets -205
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
        //タオル離す
        wait(0.3);
        catchLeftCW.write(0); //開放
        catchLeftCCW.write(0.7);
        wait(0.2);
        catchLeftCW.write(0);
        catchLeftCCW.write(0);
        wait(1);
        while (1)
        { //展開縮小
            if (rojarArm.getPulses() > 0)
            {
                rojarArmCW.write(0);
                rojarArmCCW.write(0.175);
            }
            else
            {
                rojarArmCW.write(0);
                rojarArmCCW.write(0);
                break;
            }
        }
        while (1)
        {
            static bool buttonPressed = 0;
            int buttonPressCount = 0;
            for (int i = 0; i < 10000; i++)
            {
                buttonPressCount += !startButton.read();
            }
            if (buttonPressCount == 10000)
            {
                buttonPressed = 1;
            }
            if (buttonPressed)
            {
                buttonPressed = 0;
                break;
            }
        }
        robotLocation.addPoint(0, 0, 0);
        //robotLocation.addPoint(-20, 10, 0);
        robotLocation.sendNext();
        while (!robotLocation.checkMovingStats(accelAlgorithm.getStats()))
        {
            accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
            wheel.getOutput(accelAlgorithm.getXVector(), accelAlgorithm.getYVector(), accelAlgorithm.getYawVector(), output);
            driveWheel.apply(output);
        }                             /*
        robotLocation.sendNext();
        while (!robotLocation.checkMovingStats(accelAlgorithm.getStats()))
        {
            accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
            wheel.getOutput(accelAlgorithm.getXVector(), accelAlgorithm.getYVector(), accelAlgorithm.getYawVector(), output);
            driveWheel.apply(output);
        }*/
        for (int i = 0; i < 100; i++) //完全停止用
        {
            accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
            wheel.getOutput(0, 0, 0, output);
            driveWheel.apply(output);
        }
        while (1)
        {
        }
#endif //TEST_MOTOR_SHEET
#ifdef TEST_FEET_LOOP
        for (;;)
        {
            robotLocation.addPoint(0, -80, 0);
            robotLocation.sendNext();
            while (!robotLocation.checkMovingStats(accelAlgorithm.getStats()))
            {
                accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
                wheel.getOutput(accelAlgorithm.getXVector(), accelAlgorithm.getYVector(), accelAlgorithm.getYawVector(), output);
                driveWheel.apply(output);
            }
            robotLocation.addPoint(80, -80, 0);
            robotLocation.sendNext();
            while (!robotLocation.checkMovingStats(accelAlgorithm.getStats()))
            {
                accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
                wheel.getOutput(accelAlgorithm.getXVector(), accelAlgorithm.getYVector(), accelAlgorithm.getYawVector(), output);
                driveWheel.apply(output);
            }
            robotLocation.addPoint(0, 0, 0);
            robotLocation.sendNext();
            while (!robotLocation.checkMovingStats(accelAlgorithm.getStats()))
            {
                accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
                wheel.getOutput(accelAlgorithm.getXVector(), accelAlgorithm.getYVector(), accelAlgorithm.getYawVector(), output);
                driveWheel.apply(output);
            }
        }
#endif //TEST_FEET_LOOP
#ifdef TEST_CATCH_MOTOR_STOLE
        PwmOut catchRightCW(PC_8);
        PwmOut catchRightCCW(PC_9);
        PwmOut catchLeftCW(PE_5);
        PwmOut catchLeftCCW(PE_6);
        catchRightCW.period_us(10);
        catchRightCCW.period_us(10);
        catchLeftCW.period_us(10);
        catchLeftCCW.period_us(10);

        catchRightCW.write(0.4); //閉じる
        catchRightCCW.write(0);
        catchLeftCW.write(0.4); //閉じる
        catchLeftCCW.write(0);
        while (1)
        {
        }
#endif
    }
    while (1)
    {
        //192.168.163.
        //192.168.163.140~169
    }
}
