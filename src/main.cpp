#include "mbed.h"
#include "DriveSource\MWodometry.h"
#include "DriveSource\LocationManager.h"
#include "DriveSource\DriveTrain.h"
#include "SensorSource\QEI.h"
#include "SensorSource\MPU9250.h"
#include "SensorSource\DebounceSwitch.h"
#include "MechanismSource\ClothHang.h"
#include "MechanismSource\ClothHold.h"
#include "MechanismSource\Peg.h"
#include "MechanismSource\RogerArm.h"
#include "MechanismSource\Servo.h"
#include "NewHavenDisplay.h"

/*
    使用中の足回りドライブの選択
        USING_4WD➡4輪メカナム・オムニ用逆運動学計算プログラム
        USING_3WD➡3輪オムニ用逆運動学計算プログラム
    使用するドライブのコメントアウトを消去、未使用側をコメントアウトする。
 */
//#define USING_4WD
#define USING_3WD

/*
    足回りパラメータ調整用テストコース
        1.ロボットX軸方向に100cm移動（完全停止）
        2.ロボットを原点位置に移動
        3.ロボットを自己位置に固定
 */
//#define TEST_COURSE_1

/*
    足回りパラメータ調整用テストコース
        1.ロボットY軸方向に100cm移動（完全停止）
        2.ロボットを原点位置に移動
        3.ロボットを自己位置に固定
 */
//#define TEST_COURSE_2

/*
    足回りパラメータ調整用テストコース（本番仕様）
        1.ユーザーボタン入力待機(シーツ)
        2.押下後目的地に移動
        3.ユーザーボタン入力待機(バスタオル)
        4.押下後目的地に移動
 */
//#define TEST_COURSE_GAME

/*
    足回りパラメータ調整用テストコース（長直線）
        1.ユーザーボタン入力待機
        2.y軸方向に600cm移動
        3.一定時間後(3s)300cmに移動
        4.原点に移動
 */
//#define TEST_COURSE_STRAIGHT

/*
    ロボットを位置を点(0,0,0)に固定する。
    各軸計測輪・IMUセンサのテストに使用する。
 */
//#define TEST_SELF_LOCALIZATION

//#define MECA_CLASS_DEBUG

//#define TEST_MECA_SHEET
//#define TEST_ROJAR_ENCODER
//#define TEST_SHEET_LAUNCH_ENCODER
//#define TEST_ROJAR_MOTOR
//#define TEST_SHEET_LAUNCH_MOTOR
//#define TEST_SHHET_LAUNCH
//#define TEST_MOTOR_SHEET
//#define GAME_SHEET_1
//#define TEST_FEET_LOOP

/*
    その場ですべての機構を動かす
    ビルドインスイッチで動作シーケンス切り替え
        NO pushing → middle extend
        1 time → max extend 
    外部スイッチで動作開始
 */
//#define MECA_TESTING_WITH_NO_MOVE

/*
    IMUセンサの値をシリアルモニタに出力する
 */
//#define IMUSENSOR_TEST

/*
    位置情報をLCDに表示する
 */
//#define DEBUG_LCD

/*
    新型足回り制御テスト用コード
 */
//#define TEST_DRIVE_NEWTYPE

/*
    ゲーム用新型足回り制御
 */
//#define GAME_DRIVE_NEWTYPE

#define rojarArm_test

/*
　  マイコン(F767ZI)に取り付けられている青いスイッチによって動作シーケンスを切り替える。
        0度押し➡バスタオル縦掛け
        1度押し➡バスタオル横掛け
        2度押し➡シーツ掛け
    別に取りつけられているスイッチで動作開始。
 */
//#define TEST_SWITCHING_ALL_MECA_BY_BUTTON

/*
    ENCODER_PULSE_PER_ROUND         :   取り付けエンコーダの分解能
    ENCODER_ATTACHED_WHEEL_RADIUS   :   計測輪の半径
    DISTANCE_BETWEEN_ENCODER_WHEELS :   同軸の計測輪取り付け距離
    PERMIT_ERROR_CIRCLE_RADIUS      :   停止地点の許容誤差判定円の半径
    DECREASE_PWM_CIRCLE_RADIUS      :   減速開始判定円の半径
    ESTIMATE_MAX_PWM                :   MDに出力される想定最大PWM
    ESTIMATE_MIN_PWM                :   MDに出力される想定最小PWM
    DRIVETRAIN_UPDATE_CYCLE         :   速度計算アルゴリズムの更新周期(s)
*/
#define ENCODER_PULSE_PER_ROUND 48
#define ENCODER_ATTACHED_WHEEL_RADIUS_BY_NEXUS_ROBOT 5.0
#define ENCODER_ATTACHED_WHEEL_RADIUS_BY_HANGFA 5.08
#define DISTANCE_BETWEEN_ENCODER_WHEELS 72
#define PERMIT_ERROR_CIRCLE_RADIUS 3   // 3.5
#define DECREASE_PWM_CIRCLE_RADIUS 100 //150
#define ESTIMATE_MAX_PWM 0.3           // max:0.7, recommend:0.64
#define ESTIMATE_MIN_PWM 0.1
#define DRIVETRAIN_UPDATE_CYCLE 0.13

#ifdef USING_4WD
#include "DriveSource\OmniKinematics4WD.h"
#include "DriveSource\MotorDriverAdapter4WD.h"
OmniKinematics4WD OmniKinematics;
MotorDriverAdapter4WD driveWheel(PB_10, PB_11, PE_12, PE_14, PD_12, PD_13, PE_8, PE_10);
float output[4] = {};
#endif // USING_4WD

#ifdef USING_3WD
#include "DriveSource\OmniKinematics3WD.h"
#include "DriveSource\MotorDriverAdapter3WD.h"
OmniKinematics3WD OmniKinematics;
MotorDriverAdapter3WD driveWheel(PB_11, PB_10, PE_12, PE_14, PD_12, PD_13);
float output[3] = {};
#endif // USING_3WD

Timer QEITimer;
MPU9250 IMU;
QEI encoder_XAxis_1(PE_9, PF_13, NC, ENCODER_PULSE_PER_ROUND, &QEITimer, QEI::X4_ENCODING);
QEI encoder_YAxis_1(PB_5, PC_7, NC, ENCODER_PULSE_PER_ROUND, &QEITimer, QEI::X4_ENCODING);
MWodometry odometry_XAxis_1(encoder_XAxis_1, ENCODER_PULSE_PER_ROUND, ENCODER_ATTACHED_WHEEL_RADIUS_BY_HANGFA / 2);
MWodometry odometry_YAxis_1(encoder_YAxis_1, ENCODER_PULSE_PER_ROUND, ENCODER_ATTACHED_WHEEL_RADIUS_BY_HANGFA / 2);
LocationManager<int> robotLocation(0, 0, 0);
DriveTrain accelAlgorithm(robotLocation, odometry_XAxis_1, odometry_YAxis_1, IMU, PERMIT_ERROR_CIRCLE_RADIUS, DECREASE_PWM_CIRCLE_RADIUS);
Ticker updateOutput;

Serial STLinkTerminal(USBTX, USBRX); //Surfaceのターミナルとの通信用ポート
DigitalOut IMUisReadyLED(LED3);      //IMUセンサキャリブレーション完了表示用LED用デジタル出力

int main(void)
{
#ifdef rojarArm_test
    RogerArm myArm(PF_7, PF_9); //PF_7, PF_9->pe10
    myArm.setMaxPWM(0.96);      //0.92
    QEI rogerArmEncoder(PG_0, PD_1, NC, 48, &QEITimer, QEI::X4_ENCODING);
    while (1)
    {
        myArm.setEncoderPulse(rogerArmEncoder.getPulses());
        myArm.update();
        static int armPhase = 0;
        if (myArm.stats() && armPhase == 0)
        {
            wait(2);
            armPhase++;
            myArm.setHeight(3200);
            myArm.update();
        }
        else if (myArm.stats() && armPhase == 1)
        {
            wait(2);
            armPhase = 0;
            myArm.setHeight(0);
            myArm.update();
        }
    }
#endif //rojarArm_test

#ifdef GAME_DRIVE_NEWTYPE
    Serial serialLCD(PC_6, NC, 9600);
    NewHavenDisplay LCDDriver(serialLCD);
    Timer LCDtimer;
    Timer clothHangertimer;
    LCDtimer.start();
    serialLCD.printf("WAITING...");
    DebounceSwitch startButton(PG_2, 'U'); //create object using pin "PG_2" with PullUpped
    int initialButtonPressToken = 1, startButtonPressedFlag = 0;
    ClothHold holder(PE_5, PE_6); //right,leftServo //PE_5, PE_6
    holder.free('r');
    holder.free('l');
    Peg pegAttacher(PC_9, PC_8, 0.5, 0.75); //pin ,pin pwm, time
    ClothHang hanger(PF_8, PA_0);           //PF_8, PA_0
    hanger.setMaxPWM(0.6);                  //0.85
    QEI clothHangEncoder(PE_2, PD_11, NC, 48, &QEITimer, QEI::X4_ENCODING);
    RogerArm myArm(PF_7, PF_9); //PF_7, PF_9->pe10
    myArm.setMaxPWM(0.96);      //0.92
    QEI rogerArmEncoder(PG_0, PD_1, NC, 48, &QEITimer, QEI::X4_ENCODING);
    int numberOfWayPoint = 1;
    STLinkTerminal.baud(9600);
    IMU.setup(PB_9, PB_8);
    IMUisReadyLED.write(1);
    accelAlgorithm.setMaxOutput(ESTIMATE_MAX_PWM);
    accelAlgorithm.setMinOutput(ESTIMATE_MIN_PWM);
    OmniKinematics.setMaxPWM(ESTIMATE_MAX_PWM);
    driveWheel.setMaxPWM(ESTIMATE_MAX_PWM);
    robotLocation.addPoint(0, -500, 0);
    robotLocation.addPoint(112, -550, 0);
    robotLocation.addPoint(112, -583, 0);
    robotLocation.addPoint(350, -580, 0);
    robotLocation.addPoint(0, -520, 0);
    robotLocation.addPoint(0, 0, 0);
    while (1)
    {
        if (initialButtonPressToken)
        {
            startButton.update();
            if (startButton.stats() && initialButtonPressToken)
            {
                initialButtonPressToken = 0;
                robotLocation.sendNext(); //機構テスト時はコメントアウト
                holder.grasp('r');
                holder.grasp('l');
            }
        }
        myArm.setEncoderPulse(rogerArmEncoder.getPulses());
        myArm.update();
        pegAttacher.update();
        hanger.setEncoderPulse(clothHangEncoder.getPulses());
        hanger.update();
        accelAlgorithm.update();
        accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
        OmniKinematics.getOutput(accelAlgorithm.getXVector(), accelAlgorithm.getYVector(), accelAlgorithm.getYawVector(), output);
        //display current robot vectors (3-Axis) and calculated PWMs
        //STLinkTerminal.printf("CurrentVector:%.1lf %.1lf %.1lf  \t", accelAlgorithm.getXVector(), accelAlgorithm.getYVector(), accelAlgorithm.getYawVector());
        //STLinkTerminal.printf("CalculatedPWM:%.2f %.2f %.2f \r\n", output[0], output[1], output[2]);
        driveWheel.apply(output);
        static unsigned long int prevDisplayed = 0;
        if (((LCDtimer.read_ms() - prevDisplayed) > 40) && !initialButtonPressToken) //about 24Hz flash rate
        {
            LCDDriver.clear();
            LCDDriver.home();
            serialLCD.printf("%d %d %d", robotLocation.getXLocationData(), robotLocation.getYLocationData(), robotLocation.getYawStatsData());
            LCDDriver.setCursor(2, 0);
            serialLCD.printf("%.1lf %.1lf %.1lf   ", accelAlgorithm.getCurrentXPosition(), accelAlgorithm.getCurrentYPosition(), IMU.gyro_Yaw());
            prevDisplayed = LCDtimer.read_ms();
        }
        /*if (!initialButtonPressToken) //機構テスト
        {
            static int hangerHasDoneFlag = 0, initialHangerFlag = 1;
            if (initialHangerFlag) //ロジャー展開後初めての処理
            {
                hanger.setLength(1000); //洗濯物掛ける
                hanger.update();        //あとのstats判定のために一度状態を更新する
                initialHangerFlag = 0;
            }
            if (hanger.stats() && !initialHangerFlag && !hangerHasDoneFlag)
            {
                hangerHasDoneFlag = 1;
                hanger.setLength(0);
                hanger.update();
            }
            if (hanger.stats() && hangerHasDoneFlag) //洗濯物が竿にかかっているだけの状態
            {
                static unsigned int seq = 1, timerStartFlag = 1;
                if (timerStartFlag)
                {
                    clothHangertimer.start();
                    timerStartFlag = 0;
                }
                if (0 < clothHangertimer.read_ms() && clothHangertimer.read_ms() < 1500 && seq == 1)
                {
                    holder.release('r');
                    seq = 2;
                }
                if (1500 < clothHangertimer.read_ms() && clothHangertimer.read_ms() < 3000 && seq == 2)
                {
                    holder.grasp('r');
                    seq = 3;
                }
                if (3000 < clothHangertimer.read_ms() && clothHangertimer.read_ms() < 4000 && seq == 3)
                {
                    pegAttacher.launch();
                    seq = 4;
                }
                if (4000 < clothHangertimer.read_ms() && seq == 4)
                {
                    //robotLocation.sendNext(); //次の座標を送信
                    //numberOfWayPoint++;
                }
            }
        }*/

        if (robotLocation.checkMovingStats(accelAlgorithm.getStats()) && !initialButtonPressToken)
        {
            switch (numberOfWayPoint)
            {
            case 1:
                robotLocation.sendNext();    //三本目のポール少し手前の位置
                myArm.setHeight(2100 * 0.8); //2100-洗濯物干し最適高さ , 2850-洗濯バサミ最適高さ
                numberOfWayPoint++;
                break;

            case 2:
                robotLocation.sendNext(); //三本目のポール少し手前の位置
                numberOfWayPoint++;
                break;

            case 3:
                static int armPhase = 1, hangerHasDoneFlag = 0; //phase1=洗濯物掛ける, phase2=洗濯ばさみつける
                if (myArm.stats() && armPhase == 1)             //ロジャーアーム展開完了
                {
                    static int initialHangerFlag = 1;
                    if (initialHangerFlag) //ロジャー展開後初めての処理
                    {
                        hanger.setLength(1200); //洗濯物掛ける //1000 for test , 1600 for max lenght, recommend:1530
                        hanger.update();        //すぐ下のstats判定のために一度状態を更新し判定フラグを未完了に設定する
                        initialHangerFlag = 0;
                    }
                    if (hanger.stats() && !initialHangerFlag && !hangerHasDoneFlag)
                    {
                        hangerHasDoneFlag = 1;
                        hanger.setLength(0);
                        hanger.update();
                    }
                }
                static int initialChangeHeightFlag = 1;
                if (hanger.stats() && hangerHasDoneFlag && initialChangeHeightFlag)
                {
                    static unsigned int seq = 1, timerStartFlag = 1;
                    if (timerStartFlag)
                    {
                        clothHangertimer.start();
                        timerStartFlag = 0;
                    }
                    if (0 < clothHangertimer.read_ms() && clothHangertimer.read_ms() < 1500 && seq == 1)
                    {
                        holder.release('r');
                        seq = 2;
                    }
                    if (1500 < clothHangertimer.read_ms() && clothHangertimer.read_ms() < 3000 && seq == 2)
                    {
                        holder.center('r');
                        seq = 3;
                    }
                    if (seq == 3)
                    {
                        armPhase = 2;
                        myArm.setHeight(2850 * 0.8);
                        myArm.update();
                        clothHangertimer.stop();
                        clothHangertimer.reset();
                        initialChangeHeightFlag = 0;
                    }
                }
                if (myArm.stats() && armPhase == 2)
                {
                    if (hanger.stats() && hangerHasDoneFlag) //洗濯物が竿にかかっているだけの状態
                    {
                        static unsigned int seq = 1, timerStartFlag = 1;
                        if (timerStartFlag)
                        {
                            clothHangertimer.start();
                            timerStartFlag = 0;
                        }
                        if (0 < clothHangertimer.read_ms() && clothHangertimer.read_ms() < 1700 && seq == 1)
                        {
                            pegAttacher.launch();
                            seq = 2;
                        }
                        if (1700 < clothHangertimer.read_ms() && clothHangertimer.read_ms() < 3000 && seq == 2)
                        {
                            robotLocation.sendNext(); //次の座標を送信
                            numberOfWayPoint++;
                        }
                    }
                }
                break;

            case 4:
                holder.release('l');
                myArm.setHeight(0); //ロジャーアーム縮小
                robotLocation.sendNext();
                numberOfWayPoint++;
                break;

            case 5:
                holder.grasp('r');
                holder.grasp('l');
                robotLocation.sendNext(); //三本目のポールの手前まで移動
                numberOfWayPoint++;
                break;

            case 6:
                if (myArm.stats())
                {
                    robotLocation.sendNext(); //最初の位置に戻る
                    numberOfWayPoint++;
                }
                break;
            case 7:
                LCDDriver.clear();
                serialLCD.printf("seq has done");
                while (1)
                {
                    LCDDriver.setCursor(2, 0);
                    accelAlgorithm.update();
                    accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
                    serialLCD.printf("%.1lf %.1lf %.1lf", accelAlgorithm.getCurrentXPosition(), accelAlgorithm.getCurrentYPosition(), IMU.gyro_Yaw());
                }
                break;
            default:
                LCDDriver.clear();
                serialLCD.printf("WayPoint ERROR");
                while (1)
                {
                }
                break;
            }
        }
    }
    while (1)
    {
        /* code */
    }

#endif //GAME_DRIVE_NEWTYPE

#ifdef TEST_DRIVE_NEWTYPE
    Serial serialLCD(PC_6, NC, 9600);
    NewHavenDisplay LCDDriver(serialLCD);
    Timer LCDtimer;
    LCDtimer.start();
    serialLCD.printf("WAITING...");
    DigitalIn startButton(PG_2);
    startButton.mode(PullUp);
    int initialButtonPressToken = 1, startButtonPressedFlag = 0;
    ClothHold holder(PE_5, PE_6); //right,leftServo
    holder.release('r');
    holder.release('l');
    int numberOfWayPoint = 0;
    STLinkTerminal.baud(9600);
    IMU.setup(PB_9, PB_8);
    IMUisReadyLED.write(1);
    accelAlgorithm.setMaxOutput(ESTIMATE_MAX_PWM);
    accelAlgorithm.setMinOutput(ESTIMATE_MIN_PWM);
    OmniKinematics.setMaxPWM(ESTIMATE_MAX_PWM);
    robotLocation.addPoint(0, -100, 0);
    robotLocation.addPoint(100, -100, 0);
    robotLocation.addPoint(0, 0, 0);
    while (1)
    {
        if (initialButtonPressToken)
        {
            static bool buttonPressed = 0;
            int buttonPressCount = 0;
            for (int i = 0; i < 50; i++)
            {
                buttonPressCount += !startButton.read();
            }
            if (buttonPressCount == 50)
            {
                buttonPressed = 1;
            }
            if (buttonPressed)
            {
                buttonPressed = 0;
                startButtonPressedFlag = 1;
            }
            if (startButtonPressedFlag && initialButtonPressToken)
            {
                initialButtonPressToken = 0;
                robotLocation.sendNext();
            }
        }
        accelAlgorithm.update();
        accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
        OmniKinematics.getOutput(accelAlgorithm.getXVector(), accelAlgorithm.getYVector(), accelAlgorithm.getYawVector(), output);
        driveWheel.apply(output);
        static unsigned long int prevDisplayed = 0;
        if (((LCDtimer.read_ms() - prevDisplayed) > 40) && !initialButtonPressToken) //about 24Hz flash rate
        {
            LCDDriver.clear();
            LCDDriver.home();
            serialLCD.printf("%d %d %d", robotLocation.getXLocationData(), robotLocation.getYLocationData(), robotLocation.getYawStatsData());
            LCDDriver.setCursor(2, 0);
            serialLCD.printf("%.1lf %.1lf %.1lf", accelAlgorithm.getCurrentXPosition(), accelAlgorithm.getCurrentYPosition(), IMU.gyro_Yaw());
            prevDisplayed = LCDtimer.read_ms();
        }

        if (robotLocation.checkMovingStats(accelAlgorithm.getStats()) && !initialButtonPressToken)
        {
            numberOfWayPoint++;
            switch (numberOfWayPoint)
            {
            case 1:
                holder.release('r');
                holder.release('l');
                break;

            case 2:
                holder.grasp('r');
                holder.grasp('l');
                break;

            case 3:
                holder.release('r');
                holder.release('l');
                LCDDriver.clear();
                serialLCD.printf("sequence done");
                while (1)
                {
                    LCDDriver.setCursor(2, 0);
                    accelAlgorithm.update();
                    accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
                    serialLCD.printf("%.1lf %.1lf %.1lf", accelAlgorithm.getCurrentXPosition(), accelAlgorithm.getCurrentYPosition(), IMU.gyro_Yaw());
                }
                break;
            default:
                LCDDriver.clear();
                serialLCD.printf("WayPoint ERROR");
                while (1)
                {
                }
                break;
            }
            robotLocation.sendNext();
        }
    }
#endif //TEST_DRIVE_NEWTYPE

#ifndef TEST_DRIVE_NEWTYPE
#ifndef GAME_DRIVE_NEWTYPE
    STLinkTerminal.baud(9600);
    IMU.setup(PB_9, PB_8);
    IMUisReadyLED.write(1);
    accelAlgorithm.setMaxOutput(ESTIMATE_MAX_PWM);
    accelAlgorithm.setMinOutput(ESTIMATE_MIN_PWM);
    updateOutput.attach(callback(&accelAlgorithm, &DriveTrain::update), DRIVETRAIN_UPDATE_CYCLE);
    OmniKinematics.setMaxPWM(ESTIMATE_MAX_PWM);
#endif
#endif //TEST_DRIVE_NEWTYPE

#ifdef MECA_CLASS_DEBUG

    DigitalIn userButton(USER_BUTTON);
    userButton.mode(PullDown);
    while (1)
    {
        if (userButton.read() == 1)
            break;
    }
//#define TEST_PEG //tested on 6/20(Thur)
#ifdef TEST_PEG
    Peg pegAttacher(PE_12, PE_14, 0.5, 0.5);
    pegAttacher.launch();
    while (1)
    {
        pegAttacher.update();
    }
#endif // TEST_PEG

//#define TEST_HANG //tested on 6/20(Thur)
#ifdef TEST_HANG
    ClothHang hanger(PB_10, PB_11);
    hanger.setMaxPWM(0.5);
    hanger.setLength(2000);
    QEI clothHangEncoder(PB_8, PB_9, NC, 48, &QEITimer, QEI::X4_ENCODING);
    while (1)
    {
        STLinkTerminal.printf("PULSE:%d \tHANG STATS(extend):%d\r\n", clothHangEncoder.getPulses(), hanger.stats());
        hanger.setEncoderPulse(clothHangEncoder.getPulses());
        hanger.update();
        if (hanger.stats())
            break;
    }
    hanger.setLength(0);
    while (1)
    {
        STLinkTerminal.printf("PULSE:%d \tHANG STATS(reduce):%d\r\n", clothHangEncoder.getPulses(), hanger.stats());
        hanger.setEncoderPulse(clothHangEncoder.getPulses());
        hanger.update();
    }
#endif //TEST_HANG

#define TEST_HOLD
#ifdef TEST_HOLD
    ClothHold holder(PE_9, PE_11); //right,leftServo
    while (1)
    {
        holder.grasp('r');
        wait(1);
        while (1)
        {
            if (userButton.read() == 1)
                break;
        }
        holder.release('r');
        wait(1);
        while (1)
        {
            if (userButton.read() == 1)
                break;
        }
    }
#endif //TEST_HOLD

//#define TEST_ROGER
#ifdef TEST_ROGER
    RogerArm myArm(PD_12, PD_13);
    myArm.setMaxPWM(0.5);
    myArm.setHeight(1500);
    QEI rogerArmEncoder(PB_8, PB_9, NC, 48, &QEITimer, QEI::X4_ENCODING);
    while (1)
    {
        STLinkTerminal.printf("PULSE:%d \tARM STATS(extend):%d\r\n", rogerArmEncoder.getPulses(), myArm.stats());
        myArm.setEncoderPulse(rogerArmEncoder.getPulses());
        myArm.update();
        if (myArm.stats())
            break;
    }
    myArm.setHeight(0);
    while (1)
    {
        STLinkTerminal.printf("PULSE:%d \tARM STATS(reduce):%d\r\n", rogerArmEncoder.getPulses(), myArm.stats());
        myArm.setEncoderPulse(rogerArmEncoder.getPulses());
        myArm.update();
    }
#endif //TEST_ROGER

#endif //MECA_CLASS_DEBUG

#ifdef IMUSENSOR_TEST
    /*
        IMUセンサの値をシリアルモニタに出力する
     */
    while (1)
    {
        STLinkTerminal.printf("%.2lf\r\n", IMU.gyro_Yaw());
    }
#endif
#ifdef DEBUG_LCD
    Serial serialLCD(PC_6, NC, 9600);
    NewHavenDisplay LCDDriver(serialLCD);
    while (1)
    {
        LCDDriver.clear();
        LCDDriver.home();
        serialLCD.printf("%d,%d,%d", robotLocation.getXLocationData(), robotLocation.getYLocationData(), robotLocation.getYawStatsData());
        LCDDriver.setCursor(2, 0);
        serialLCD.printf("%.1lf,%.1lf,%.1lf", accelAlgorithm.getCurrentXPosition(), accelAlgorithm.getCurrentYPosition(), IMU.gyro_Yaw());
        wait_ms(20);
    }
#endif

    for (;;) //ここからメイン関数のループ開始
    {

#ifdef TEST_SELF_LOCALIZATION
        /*
            ロボットを位置を点(0,0,0)に固定する。
            各軸計測輪・IMUセンサのテストに使用する。
        */
        robotLocation.addPoint(0, 0, 0);
        robotLocation.sendNext();
        while (!robotLocation.checkMovingStats(accelAlgorithm.getStats()))
        {
            accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
            OmniKinematics.getOutput(accelAlgorithm.getXVector(), accelAlgorithm.getYVector(), accelAlgorithm.getYawVector(), output);
            driveWheel.apply(output);
        }
        while (1)
        {
            //STLinkTerminal.printf("%d,%d\r\n", encoder_XAxis_1.getPulses(), encoder_YAxis_1.getPulses());
            STLinkTerminal.printf("%lf,%lf,%lf\r\n", accelAlgorithm.getXVector(), accelAlgorithm.getYVector(), accelAlgorithm.getYawVector());
            accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
            OmniKinematics.getOutput(accelAlgorithm.getXVector(), accelAlgorithm.getYVector(), accelAlgorithm.getYawVector(), output);
            driveWheel.apply(output);
        }
#endif

#ifdef TEST_COURSE_1
        /*
        足回りパラメータ調整用テストコース
            1.ロボットX軸方向に100cm移動（完全停止）
            2.ロボットを原点位置に移動
            3.ロボットを自己位置に固定
        */
        robotLocation.addPoint(100, 0, 0);
        robotLocation.addPoint(0, 0, 0);
        robotLocation.sendNext();
        while (!robotLocation.checkMovingStats(accelAlgorithm.getStats()))
        {
            accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
            OmniKinematics.getOutput(accelAlgorithm.getXVector(), accelAlgorithm.getYVector(), accelAlgorithm.getYawVector(), output);
            driveWheel.apply(output);
        }
        for (int i = 0; i < 100; i++) //完全停止用
        {
            accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
            OmniKinematics.getOutput(0, 0, 0, output);
            driveWheel.apply(output);
        }
        robotLocation.sendNext();
        while (!robotLocation.checkMovingStats(accelAlgorithm.getStats()))
        {
            accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
            OmniKinematics.getOutput(accelAlgorithm.getXVector(), accelAlgorithm.getYVector(), accelAlgorithm.getYawVector(), output);
            driveWheel.apply(output);
        }
        while (1)
        {
            accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
            OmniKinematics.getOutput(accelAlgorithm.getXVector(), accelAlgorithm.getYVector(), accelAlgorithm.getYawVector(), output);
            driveWheel.apply(output);
        }
#endif

#ifdef TEST_COURSE_2
        /*
        足回りパラメータ調整用テストコース
            1.ロボットY軸方向に100cm移動（完全停止）
            2.ロボットを原点位置に移動
            3.ロボットを自己位置に固定
         */
        robotLocation.addPoint(0, 100, 0);
        robotLocation.addPoint(0, 0, 0);
        robotLocation.sendNext();
        while (!robotLocation.checkMovingStats(accelAlgorithm.getStats()))
        {
            accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
            OmniKinematics.getOutput(accelAlgorithm.getXVector(), accelAlgorithm.getYVector(), accelAlgorithm.getYawVector(), output);
            driveWheel.apply(output);
        }
        for (int i = 0; i < 100; i++) //完全停止用
        {
            accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
            OmniKinematics.getOutput(0, 0, 0, output);
            driveWheel.apply(output);
        }
        robotLocation.sendNext();
        while (!robotLocation.checkMovingStats(accelAlgorithm.getStats()))
        {
            accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
            OmniKinematics.getOutput(accelAlgorithm.getXVector(), accelAlgorithm.getYVector(), accelAlgorithm.getYawVector(), output);
            driveWheel.apply(output);
        }
        while (1)
        {
            accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
            OmniKinematics.getOutput(accelAlgorithm.getXVector(), accelAlgorithm.getYVector(), accelAlgorithm.getYawVector(), output);
            driveWheel.apply(output);
        }
#endif

#ifdef TEST_COURSE_STRAIGHT
        /*
        足回りパラメータ調整用テストコース（長直線）
            1.ユーザーボタン入力待機
            2.y軸方向に600cm移動
            3.一定時間後(3s)300cmに移動
            4.原点に移動
        */
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
            OmniKinematics.getOutput(accelAlgorithm.getXVector(), accelAlgorithm.getYVector(), accelAlgorithm.getYawVector(), output);
            driveWheel.apply(output);
        }
        for (int i = 0; i < 100; i++) //完全停止用
        {
            accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
            OmniKinematics.getOutput(0, 0, 0, output);
            driveWheel.apply(output);
        }
        wait(3);
        robotLocation.sendNext();
        while (!robotLocation.checkMovingStats(accelAlgorithm.getStats()))
        {
            accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
            OmniKinematics.getOutput(accelAlgorithm.getXVector(), accelAlgorithm.getYVector(), accelAlgorithm.getYawVector(), output);
            driveWheel.apply(output);
        }
        robotLocation.sendNext();
        while (!robotLocation.checkMovingStats(accelAlgorithm.getStats()))
        {
            accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
            OmniKinematics.getOutput(accelAlgorithm.getXVector(), accelAlgorithm.getYVector(), accelAlgorithm.getYawVector(), output);
            driveWheel.apply(output);
        }
        for (int i = 0; i < 100; i++) //完全停止用
        {
            accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
            OmniKinematics.getOutput(0, 0, 0, output);
            driveWheel.apply(output);
        }
#endif

#ifdef TEST_COURSE_GAME
        /*
        足回りパラメータ調整用テストコース（本番仕様）
            1.ユーザーボタン入力待機(シーツ)
            2.押下後目的地に移動
            3.ユーザーボタン入力待機(バスタオル)
            4.押下後目的地に移動
        */
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
        robotLocation.addPoint(0, -550, 0); // 一度目のアプローチ
        robotLocation.addPoint(100, -580, 0);
        robotLocation.addPoint(300, -580, 0);
        robotLocation.addPoint(0, -550, 0);
        robotLocation.addPoint(0, 0, 0);
        for (int i = 0; i < 5; i++)
        {
            robotLocation.sendNext();
            while (!robotLocation.checkMovingStats(accelAlgorithm.getStats()))
            {
                STLinkTerminal.printf("%lf,%lf,%lf\r\n", accelAlgorithm.getCurrentXPosition(), accelAlgorithm.getCurrentYPosition(), accelAlgorithm.getCurrentYawPosition());
                accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
                OmniKinematics.getOutput(accelAlgorithm.getXVector(), accelAlgorithm.getYVector(), accelAlgorithm.getYawVector(), output);
                driveWheel.apply(output);
            }
        }
        for (int i = 0; i < 100; i++) //完全停止用
        {
            accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
            OmniKinematics.getOutput(0, 0, 0, output);
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
        robotLocation.addPoint(0, 370, 0);
        robotLocation.addPoint(0, 0, 0);
        for (int i = 0; i < 8; i++)
        {
            robotLocation.sendNext();
            while (!robotLocation.checkMovingStats(accelAlgorithm.getStats()))
            {
                accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
                OmniKinematics.getOutput(accelAlgorithm.getXVector(), accelAlgorithm.getYVector(), accelAlgorithm.getYawVector(), output);
                driveWheel.apply(output);
            }
        }
        for (int i = 0; i < 100; i++) //完全停止用
        {
            accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
            OmniKinematics.getOutput(0, 0, 0, output);
            driveWheel.apply(output);
        }

#endif //TEST_COURSE_GAME

#ifdef MECA_TESTING_WITH_NO_MOVE
        DigitalIn startButton(PG_2);
        DigitalIn userButton(USER_BUTTON);
        startButton.mode(PullUp);
        userButton.mode(PullDown);
        DigitalOut figLED1(LED1);
        DigitalOut figLED2(LED2);
        uint8_t currentMode = 0;
        while (1)
        {

            static bool prevStats = 0;
            int userButtonPressCount = 0;
            for (int i = 0; i < 10000; i++)
            {
                userButtonPressCount += userButton.read();
            }
            bool currentStats = 0;
            if (userButtonPressCount == 10000)
            {
                currentStats = 1;
            }
            if (userButtonPressCount == 0)
            {
                currentStats = 0;
            }

            if (prevStats != currentStats && currentStats)
            {
                currentMode++;
            }
            prevStats = currentStats;

            if (currentMode == 0)
            {
                figLED1.write(1);
                figLED2.write(0);
            }
            else if (currentMode == 1)
            {
                figLED1.write(1);
                figLED2.write(1);
            }
            if (currentMode > 1)
            {
                currentMode = 0;
            }

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
        Servo catchLeftServo(PB_4);
        Servo catchRightServo(PE_5);
        catchLeftServo.calibrate();
        catchRightServo.calibrate(); //202 deg travel(out of box) , 556~2410 usec
        catchRightServo.position(90);
        catchLeftServo.position(-90);
        wait(1.25);
        //ロジャー展開開始
        QEI rojarArm(PG_0, PD_1, NC, ENCODER_PULSE_PER_ROUND, &QEITimer, QEI::X4_ENCODING);
        PwmOut rojarArmCW(PF_9);
        PwmOut rojarArmCCW(PF_7);
        rojarArmCW.period_us(10);
        rojarArmCCW.period_us(10);
        while (1)
        {
            if (currentMode == 0)
            {
                if (rojarArm.getPulses() < 1630) //1630 def //2850 max
                {
                    rojarArmCW.write(0.95);
                    rojarArmCCW.write(0);
                }
                else
                {
                    rojarArmCW.write(0);
                    rojarArmCCW.write(0);
                    break;
                }
            }
            else if (currentMode == 1)
            {
                if (rojarArm.getPulses() < 2850) //1630 def //2850 max
                {
                    rojarArmCW.write(0.95);
                    rojarArmCCW.write(0);
                }
                else
                {
                    rojarArmCW.write(0);
                    rojarArmCCW.write(0);
                    break;
                }
            }
        }
        QEI sheetLaunch(PE_2, PD_11, NC, ENCODER_PULSE_PER_ROUND, &QEITimer, QEI::X4_ENCODING);
        PwmOut sheetLaunchCW(PF_8);
        PwmOut sheetLaunchCCW(PA_0);
        sheetLaunchCW.period_us(100);
        sheetLaunchCCW.period_us(100);
        while (1)
        { //シーツかける
            if (sheetLaunch.getPulses() < 1500)
            {
                sheetLaunchCW.write(0.9);
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
                sheetLaunchCCW.write(0.9);
            }
            else
            {
                sheetLaunchCW.write(0);
                sheetLaunchCCW.write(0);
                break;
            }
        }
        while (1)
        { //ロジャーアーム縮小
            if (rojarArm.getPulses() > 5)
            {
                rojarArmCW.write(0);
                rojarArmCCW.write(0.8);
            }
            else
            {
                rojarArmCW.write(0);
                rojarArmCCW.write(0);
                break;
            }
        }

#endif //MECA_TESTING_WITH_NO_MOVE

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
            OmniKinematics.getOutput(accelAlgorithm.getXVector(), accelAlgorithm.getYVector(), accelAlgorithm.getYawVector(), output);
            driveWheel.apply(output);
        }
        for (int i = 0; i < 100; i++) //完全停止用
        {
            accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
            OmniKinematics.getOutput(0, 0, 0, output);
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
            OmniKinematics.getOutput(accelAlgorithm.getXVector(), accelAlgorithm.getYVector(), accelAlgorithm.getYawVector(), output);
            driveWheel.apply(output);
        }
        for (int i = 0; i < 100; i++) //完全停止用
        {
            accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
            OmniKinematics.getOutput(0, 0, 0, output);
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
            OmniKinematics.getOutput(accelAlgorithm.getXVector(), accelAlgorithm.getYVector(), accelAlgorithm.getYawVector(), output);
            driveWheel.apply(output);
        }
        for (int i = 0; i < 100; i++) //完全停止用
        {
            accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
            OmniKinematics.getOutput(0, 0, 0, output);
            driveWheel.apply(output);
        }*/
#endif    //TEST_MECA_SHEET
#ifdef TEST_ROJAR_ENCODER
        QEI rojarArm(PG_0, PD_1, NC, ENCODER_PULSE_PER_ROUND, &QEITimer, QEI::X4_ENCODING);
        while (1)
        {
            STLinkTerminal.printf("ENCODER:%d\r\n", rojarArm.getPulses());
        }
#endif //TEST_ROJAR_ENCODER
#ifdef TEST_SHEET_LAUNCH_ENCODER
        QEI sheetLaunch(PE_2, PD_11, NC, ENCODER_PULSE_PER_ROUND, &QEITimer, QEI::X4_ENCODING);
        while (1)
        {                                                                     //0,1,5,7,9,13
            STLinkTerminal.printf("ENCODER:%d\r\n", sheetLaunch.getPulses()); //-447
        }
#endif                  //TEST_SHEET_LAUNCH_ENCODER
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
#endif                         //TEST_ROJAR_MOTOR
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
        {                                    //展開
            if (rojarArm.getPulses() < 1280) //1310 def //2850 max
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
        {                                                                     //シーツかける
            STLinkTerminal.printf("ENCODER:%d\r\n", sheetLaunch.getPulses()); //-447
            if (sheetLaunch.getPulses() < 1510)
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
        wait(0.3);
        DigitalOut solenoidScissorsPush(PD_15);
        DigitalOut solenoidscissorsPull(PF_12);
        solenoidScissorsPush = 1;
        wait(0.5);
        solenoidScissorsPush = 0;
        solenoidscissorsPull = 1;
        wait(0.4);
        solenoidscissorsPull = 0;
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
        robotLocation.addPoint(-230, 0, 0); // short -37 ,long -97, sheets -230
        robotLocation.sendNext();
        while (!robotLocation.checkMovingStats(accelAlgorithm.getStats()))
        {
            accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
            OmniKinematics.getOutput(accelAlgorithm.getXVector(), accelAlgorithm.getYVector(), accelAlgorithm.getYawVector(), output);
            driveWheel.apply(output);
        }
        for (int i = 0; i < 100; i++) //完全停止用
        {
            accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
            OmniKinematics.getOutput(0, 0, 0, output);
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
            OmniKinematics.getOutput(accelAlgorithm.getXVector(), accelAlgorithm.getYVector(), accelAlgorithm.getYawVector(), output);
            driveWheel.apply(output);
        }                             /*
        robotLocation.sendNext();
        while (!robotLocation.checkMovingStats(accelAlgorithm.getStats()))
        {
            accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
            OmniKinematics.getOutput(accelAlgorithm.getXVector(), accelAlgorithm.getYVector(), accelAlgorithm.getYawVector(), output);
            driveWheel.apply(output);
        }*/
        for (int i = 0; i < 100; i++) //完全停止用
        {
            accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
            OmniKinematics.getOutput(0, 0, 0, output);
            driveWheel.apply(output);
        }
        while (1)
        {
        }
#endif //TEST_MOTOR_SHEET

#ifdef GAME_SHEET_1
//#define ENABLE_DRIVE
#define ENABLE_SHEET_ONLY
        DigitalIn startButton(PG_2);
        startButton.mode(PullUp);
        const float rightGraspServo = 0.0F;
        const float rightReleaseServo = 1.0F;
        const float leftGraspServo = 1.0F;
        const float leftReleaseServo = 0.0F;
        while (1) //シーツつかむの待機
        {
            IMU.gyro_Yaw();
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
        Servo catchLeftServo(PE_5);  //PE_5
        Servo catchRightServo(PE_6); //PB_13 //暴走
        catchLeftServo.calibrate(0.0006F, 90.0F);
        catchRightServo.calibrate(0.0006F, 90.0F);
        /*while (1)
        {
            for (int i = 0; i < 100; i++)
            {
                catchLeftServo.write(leftGraspServo);
                catchRightServo.write(rightGraspServo);
                wait(0.01);
            }
            for (int i = 100; i > 0; i--)
            {
                catchLeftServo.write(leftReleaseServo);
                catchRightServo.write(rightReleaseServo);
                wait(0.01);
            }
        }*/
        catchRightServo.write(rightGraspServo);
        catchLeftServo.write(leftGraspServo);
        wait(1.0);
        while (1) //開始待機
        {
            IMU.gyro_Yaw();
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
#ifdef ENABLE_DRIVE
        robotLocation.addPoint(0, -500, 0); // 一度目のアプローチ
        //accelAlgorithm.setAllocateErrorCircleRadius(20);
        robotLocation.sendNext();
        while (!robotLocation.checkMovingStats(accelAlgorithm.getStats()))
        {
            STLinkTerminal.printf("%lf,%lf,%lf\r\n", accelAlgorithm.getCurrentXPosition(), accelAlgorithm.getCurrentYPosition(), accelAlgorithm.getCurrentYawPosition());
            accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
            OmniKinematics.getOutput(accelAlgorithm.getXVector(), accelAlgorithm.getYVector(), accelAlgorithm.getYawVector(), output);
            driveWheel.apply(output);
        }
        //accelAlgorithm.setAllocateErrorCircleRadius(3.5);
        //accelAlgorithm.setDecreaseCircleRadius(100);
#endif
        //ロジャー展開開始
        QEI rojarArm(PG_0, PD_1, NC, ENCODER_PULSE_PER_ROUND, &QEITimer, QEI::X4_ENCODING);
        PwmOut rojarArmCW(PF_9);
        PwmOut rojarArmCCW(PF_7);
        rojarArmCW.period_us(40);
        rojarArmCCW.period_us(40);
        const int tenkaisaizu = 2850;           //def 2850
        if (rojarArm.getPulses() < tenkaisaizu) //1630 def //2850 max
        {
            rojarArmCW.write(0.9);
            rojarArmCCW.write(0);
        }
        else
        {
            rojarArmCW.write(0);
            rojarArmCCW.write(0);
        }
#ifdef ENABLE_DRIVE
        robotLocation.addPoint(125, -555, 0); //575
        robotLocation.sendNext();
        while (!robotLocation.checkMovingStats(accelAlgorithm.getStats()))
        {
            accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
            OmniKinematics.getOutput(accelAlgorithm.getXVector(), accelAlgorithm.getYVector(), accelAlgorithm.getYawVector(), output);
            driveWheel.apply(output);
            if (rojarArm.getPulses() < 1630) //1630 def //2850 max
            {
                rojarArmCW.write(0.35);
                rojarArmCCW.write(0);
            }
            else
            {
                rojarArmCW.write(0);
                rojarArmCCW.write(0);
            }
        }
        for (int i = 0; i < 100; i++) //完全停止用
        {
            accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
            OmniKinematics.getOutput(0, 0, 0, output);
            driveWheel.apply(output);
        }
#endif
#ifdef ENABLE_SHEET_ONLY
        robotLocation.addPoint(95, -55, 0);
        robotLocation.sendNext();
        while (!robotLocation.checkMovingStats(accelAlgorithm.getStats()))
        {
            STLinkTerminal.printf("%lf,%lf,%lf\r\n", accelAlgorithm.getCurrentXPosition(), accelAlgorithm.getCurrentYPosition(), accelAlgorithm.getCurrentYawPosition());
            accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
            OmniKinematics.getOutput(accelAlgorithm.getXVector(), accelAlgorithm.getYVector(), accelAlgorithm.getYawVector(), output);
            driveWheel.apply(output);
            if (rojarArm.getPulses() < 1630) //1630 def //2850 max
            {
                rojarArmCW.write(0.9);
                rojarArmCCW.write(0);
            }
            else
            {
                rojarArmCW.write(0);
                rojarArmCCW.write(0);
            }
        }
        for (int i = 0; i < 100; i++) //完全停止用
        {
            accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
            OmniKinematics.getOutput(0, 0, 0, output);
            driveWheel.apply(output);
        }
#endif            //ENABLE_SHEET_ONLY
        while (1) //ロジャー展開待ち
        {
            IMU.gyro_Yaw();
            STLinkTerminal.printf("ENCODER:%d\r\n", rojarArm.getPulses());
            if (rojarArm.getPulses() < tenkaisaizu) //1630 def //2850 max
            {
                rojarArmCW.write(0.9);
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
        PwmOut sheetLaunchCW(PF_8);
        PwmOut sheetLaunchCCW(PA_0);
        sheetLaunchCW.period_us(40);
        sheetLaunchCCW.period_us(40);
        const int kakutyousaizu = 1500; //def 1500
        while (1)                       //シーツ掛ける
        {
            IMU.gyro_Yaw();
            if (sheetLaunch.getPulses() < kakutyousaizu)
            {
                sheetLaunchCW.write(0.95);
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
            IMU.gyro_Yaw();
            if (sheetLaunch.getPulses() > 0)
            {
                sheetLaunchCW.write(0);
                sheetLaunchCCW.write(0.95);
            }
            else
            {
                sheetLaunchCW.write(0);
                sheetLaunchCCW.write(0);
                break;
            }
        }
        catchLeftServo.write(leftReleaseServo);
        wait(1.5);
        catchLeftServo.write(leftGraspServo);
        wait(1.5);

        PwmOut motorScissorsCW(PC_9);
        PwmOut motorScissorsCCW(PC_8);
        motorScissorsCW.period_us(40);
        motorScissorsCCW.period_us(40);
        motorScissorsCW.write(0.95); //0.7
        motorScissorsCCW.write(0);
        wait(0.47); //0.47
        motorScissorsCW.write(0);
        motorScissorsCCW.write(0.95);
        wait(0.52); //0.52
        motorScissorsCW.write(0);
        motorScissorsCCW.write(0);
#ifdef ENABLE_DRIVE
        robotLocation.addPoint(330, -555, 0); //シーツ広げる
        robotLocation.sendNext();
        while (!robotLocation.checkMovingStats(accelAlgorithm.getStats()))
        {
            accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
            OmniKinematics.getOutput(accelAlgorithm.getXVector(), accelAlgorithm.getYVector(), accelAlgorithm.getYawVector(), output);
            driveWheel.apply(output);
        }
#endif
#ifdef ENABLE_SHEET_ONLY
        robotLocation.addPoint(300, -55, 0); //シーツ広げる
        robotLocation.sendNext();
        while (!robotLocation.checkMovingStats(accelAlgorithm.getStats()))
        {
            IMU.gyro_Yaw();
            STLinkTerminal.printf("%lf,%lf,%lf\r\n", accelAlgorithm.getCurrentXPosition(), accelAlgorithm.getCurrentYPosition(), accelAlgorithm.getCurrentYawPosition());
            accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
            OmniKinematics.getOutput(accelAlgorithm.getXVector(), accelAlgorithm.getYVector(), accelAlgorithm.getYawVector(), output);
            driveWheel.apply(output);
        }
        for (int i = 0; i < 100; i++) //完全停止用
        {
            IMU.gyro_Yaw();
            accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
            OmniKinematics.getOutput(0, 0, 0, output);
            driveWheel.apply(output);
        }
#endif //ENABLE_SHEET_ONLY
        catchRightServo.write(rightReleaseServo);
        wait(2); //足回り有効時は0.5
#ifdef ENABLE_DRIVE
        //accelAlgorithm.setAllocateErrorCircleRadius(20);
        robotLocation.addPoint(10, -500, 0);
        robotLocation.sendNext();
        if (rojarArm.getPulses() > 0)
        {
            rojarArmCW.write(0);
            rojarArmCCW.write(0.9);
        }
        else
        {
            rojarArmCW.write(0);
            rojarArmCCW.write(0);
        }
        while (!robotLocation.checkMovingStats(accelAlgorithm.getStats()))
        {
            accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
            OmniKinematics.getOutput(accelAlgorithm.getXVector(), accelAlgorithm.getYVector(), accelAlgorithm.getYawVector(), output);
            driveWheel.apply(output);
            if (rojarArm.getPulses() > 0)
            {
                rojarArmCW.write(0);
                rojarArmCCW.write(0.9);
            }
            else
            {
                rojarArmCW.write(0);
                rojarArmCCW.write(0);
            }
        }
#endif
        robotLocation.addPoint(0, 0, 0); //シーツ広げる
        robotLocation.sendNext();
        while (!robotLocation.checkMovingStats(accelAlgorithm.getStats()))
        {
            IMU.gyro_Yaw();
            if (rojarArm.getPulses() > 0)
            {
                rojarArmCW.write(0);
                rojarArmCCW.write(0.9);
            }
            else
            {
                rojarArmCW.write(0);
                rojarArmCCW.write(0);
            }
            STLinkTerminal.printf("%lf,%lf,%lf\r\n", accelAlgorithm.getCurrentXPosition(), accelAlgorithm.getCurrentYPosition(), accelAlgorithm.getCurrentYawPosition());
            accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
            OmniKinematics.getOutput(accelAlgorithm.getXVector(), accelAlgorithm.getYVector(), accelAlgorithm.getYawVector(), output);
            driveWheel.apply(output);
        }
        for (int i = 0; i < 100; i++) //完全停止用
        {
            IMU.gyro_Yaw();
            if (rojarArm.getPulses() > 0)
            {
                rojarArmCW.write(0);
                rojarArmCCW.write(0.9);
            }
            else
            {
                rojarArmCW.write(0);
                rojarArmCCW.write(0);
            }
            accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
            OmniKinematics.getOutput(0, 0, 0, output);
            driveWheel.apply(output);
        }
        catchRightServo.write(rightGraspServo);
        catchLeftServo.write(leftGraspServo);
#ifdef ENABLE_DRIVE
        //accelAlgorithm.setAllocateErrorCircleRadius(3.5);
        robotLocation.addPoint(10, 0, 0);
        robotLocation.sendNext();
        while (!robotLocation.checkMovingStats(accelAlgorithm.getStats()))
        {
            accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
            OmniKinematics.getOutput(accelAlgorithm.getXVector(), accelAlgorithm.getYVector(), accelAlgorithm.getYawVector(), output);
            driveWheel.apply(output);
            if (rojarArm.getPulses() > 0)
            {
                rojarArmCW.write(0);
                rojarArmCCW.write(0.9);
            }
            else
            {
                rojarArmCW.write(0);
                rojarArmCCW.write(0);
            }
        }
        for (int i = 0; i < 100; i++) //完全停止用
        {
            accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
            OmniKinematics.getOutput(0, 0, 0, output);
            driveWheel.apply(output);
        }
#endif
#ifndef ENABLE_DRIVE
        while (1)
        {
            IMU.gyro_Yaw();
            if (rojarArm.getPulses() > 0)
            {
                rojarArmCW.write(0);
                rojarArmCCW.write(0.9);
            }
            else
            {
                rojarArmCW.write(0);
                rojarArmCCW.write(0);
            }
        }
#endif
#endif //GAME_SHEET_1
#ifdef TEST_FEET_LOOP
        for (;;)
        {
            robotLocation.addPoint(0, -80, 0);
            robotLocation.sendNext();
            while (!robotLocation.checkMovingStats(accelAlgorithm.getStats()))
            {
                accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
                OmniKinematics.getOutput(accelAlgorithm.getXVector(), accelAlgorithm.getYVector(), accelAlgorithm.getYawVector(), output);
                driveWheel.apply(output);
            }
            robotLocation.addPoint(80, -80, 0);
            robotLocation.sendNext();
            while (!robotLocation.checkMovingStats(accelAlgorithm.getStats()))
            {
                accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
                OmniKinematics.getOutput(accelAlgorithm.getXVector(), accelAlgorithm.getYVector(), accelAlgorithm.getYawVector(), output);
                driveWheel.apply(output);
            }
            robotLocation.addPoint(0, 0, 0);
            robotLocation.sendNext();
            while (!robotLocation.checkMovingStats(accelAlgorithm.getStats()))
            {
                accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
                OmniKinematics.getOutput(accelAlgorithm.getXVector(), accelAlgorithm.getYVector(), accelAlgorithm.getYawVector(), output);
                driveWheel.apply(output);
            }
        }
#endif //TEST_FEET_LOOP

#ifdef TEST_SWITCHING_ALL_MECA_BY_BUTTON
        /*
　      マイコン(F767ZI)に取り付けられている青いスイッチによって動作シーケンスを切り替える。
            0度押し➡バスタオル縦掛け
            1度押し➡バスタオル横掛け
            2度押し➡シーツ掛け
        別に取りつけられているスイッチで動作開始。
         */
        DigitalIn startButton(PG_2);
        DigitalIn userButton(USER_BUTTON);
        startButton.mode(PullUp);
        userButton.mode(PullDown);
        DigitalOut figLED1(LED1);
        DigitalOut figLED2(LED2);
        uint8_t currentMode = 0;
        while (1)
        {

            static bool prevStats = 0, userButtonPressed = 0;
            int userButtonPressCount = 0;
            for (int i = 0; i < 10000; i++)
            {
                userButtonPressCount += userButton.read();
            }
            bool currentStats;
            if (userButtonPressCount == 10000)
            {
                currentStats = 1;
            }
            if (userButtonPressCount == 0)
            {
                currentStats = 0;
            }

            if (prevStats != currentStats && currentStats)
            {
                currentMode++;
            }
            prevStats = currentStats;

            if (currentMode == 0)
            {
                figLED1.write(1);
                figLED2.write(0);
            }
            else if (currentMode == 1)
            {
                figLED1.write(0);
                figLED2.write(1);
            }
            else if (currentMode == 2)
            {
                figLED1.write(1);
                figLED2.write(1);
            }

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

        catchRightCW.write(0.35); //閉じる
        catchRightCCW.write(0);
        catchLeftCW.write(0.35); //閉じる
        catchLeftCCW.write(0);
        wait(0.2);
        catchRightCW.write(0.15);
        catchRightCCW.write(0);
        catchLeftCW.write(0.15);
        catchLeftCCW.write(0);

        QEI rojarArm(PG_0, PD_1, NC, ENCODER_PULSE_PER_ROUND, &QEITimer, QEI::X4_ENCODING);
        PwmOut rojarArmCW(PF_9);
        PwmOut rojarArmCCW(PF_7);
        rojarArmCW.period_us(100);
        rojarArmCCW.period_us(100);
        while (1)
        {                                    //展開
            if (rojarArm.getPulses() < 1280) //1310 def //2850 max
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
        {                                                                     //シーツかける
            STLinkTerminal.printf("ENCODER:%d\r\n", sheetLaunch.getPulses()); //-447
            if (sheetLaunch.getPulses() < 1510)
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
        catchRightCCW.write(0.4);
        catchLeftCW.write(0.175);
        catchLeftCCW.write(0);
        wait(0.5);
        catchRightCW.write(0);
        catchRightCCW.write(0);
        wait(0.3);
        DigitalOut solenoidScissorsPush(PD_15);
        DigitalOut solenoidscissorsPull(PF_12);
        solenoidScissorsPush = 1;
        wait(0.5);
        solenoidScissorsPush = 0;
        solenoidscissorsPull = 1;
        wait(0.4);
        solenoidscissorsPull = 0;
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
        if (currentMode == 0)
        {
            robotLocation.addPoint(-37, 0, 0);
        }
        else if (currentMode == 1)
        {
            robotLocation.addPoint(-97, 0, 0);
        }
        else if (currentMode == 2)
        {
            robotLocation.addPoint(-230, 0, 0);
        }

        //robotLocation.addPoint(-230, 0, 0); // short -37 ,long -97, sheets -230
        robotLocation.sendNext();
        while (!robotLocation.checkMovingStats(accelAlgorithm.getStats()))
        {
            accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
            OmniKinematics.getOutput(accelAlgorithm.getXVector(), accelAlgorithm.getYVector(), accelAlgorithm.getYawVector(), output);
            driveWheel.apply(output);
        }
        for (int i = 0; i < 100; i++) //完全停止用
        {
            accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
            OmniKinematics.getOutput(0, 0, 0, output);
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
            if (rojarArm.getPulses() > 80)
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
            OmniKinematics.getOutput(accelAlgorithm.getXVector(), accelAlgorithm.getYVector(), accelAlgorithm.getYawVector(), output);
            driveWheel.apply(output);
        }
        for (int i = 0; i < 100; i++) //完全停止用
        {
            accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
            OmniKinematics.getOutput(0, 0, 0, output);
            driveWheel.apply(output);
        }
        while (1)
        {
        }
#endif //TEST_SWITCHING_ALL_MECA_BY_BUTTON
    }
    while (1)
    {
    }
}
