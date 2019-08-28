#include "DriveSource\DriveTrain.h"
#include "DriveSource\LocationManager.h"
#include "DriveSource\MWodometry.h"
#include "DriveSource\OmniKinematics3WD.h"
#include "DriveSource\TimeIncreaser.h"
#include "MechanismSource\ClothHang.h"
#include "MechanismSource\ClothHold.h"
#include "MechanismSource\Peg.h"
#include "MechanismSource\RojarArm.h"
#include "MechanismSource\Servo.h"
#include "NewHavenDisplay.h"
#include "SensorSource\DebounceSwitch.h"
#include "SensorSource\MPU9250.h"
#include "SensorSource\QEI.h"
#include "UnitProtocol.hpp"
#include "mbed.h"

#define GAME_MODECHANGE_ONBOARDSWITCH //基板上の青スイッチで動作切り替え（シーツ・バスタオル）
//#define TEST_DRIVE
//#define TEST_RojarArm_UpDownLoop
//#define TEST_PEGLaunch      //tested on 6/20(Thur)
//#define TEST_HangerMovement //tested on 6/20(Thur)
//#define TEST_HoldServo
//#define TEST_RojarArmUpDownOnce

enum gameMode
{ //剰余計算のために作成中は順番を崩している
  RED_FRONT_bathTowelLeft,
  RED_MIDDLE_bathTowelLeft,
  RED_BACK_Sheets,

  //↓ not yet
  RED_MIDDLE_bathTowelRight,
  RED_MIDDLE_bathTowelboth,
  BLUE_FRONT_bathTowelRight,
  BLUE_MIDDLE_bathTowelRight,
  BLUE_MIDDLE_bathTowelLeft,
  BLUE_MIDDLE_bathTowelboth,
  BLUE_BACK_Sheets,
};

struct baudRate
{
  const long HardwareSerial = 256000;
  const int LCD = 9600;
  const long I2C = 400000;
} SerialBaud;

struct parameter
{
  const int encoderPPRLow = 48;
  const int encoderPPRHigh = 100;
  const double encoderAttachedWheelRadius = 2.54; //mini(50) omni
  const double permitErrorCircleRadius = 2.0;
  const int decreaseSpeedCircleRadius = 85;
  const double estimateDriveMaxPWM = 0.6; // max:0.7, recommend:0.64 //DEFAULT 0.5
  const double estimateDriveMinPWM = 0.15;
  const double estimatePegMaxPWM = 0.5;
  const double estimateHangerMaxPWM = 0.6;
  const double estimateRojarArmMaxPWM = 0.96;
  const double PegVoltageImpressionTime = 0.3;
} Robot;

struct
{
  PinName XAxisAPulse = PD_14;
  PinName XAxisBPulse = PD_15;
  PinName XAxisIndexPulse = NC;
  PinName YAxisAPulse = PF_12;
  PinName YAxisBPulse = PF_13;
  PinName YAxisIndexPulse = NC;
} OdometryPin;

struct
{
  PinName LCD1TX = PB_6;
  PinName LCD1RX = PB_15;
  PinName UIFTX = PB_6; //user interface
  PinName UIFRX = PB_15;
  PinName MDD1TX = PD_5;
  PinName MDD1RX = PD_6;
  PinName MDD2TX = PC_10;
  PinName MDD2RX = PC_11;
  PinName MDD3TX = PC_6;
  PinName MDD3RX = PC_7;
} serialDevice;

struct
{
  PinName toBegin = PG_2;
  PinName frontR = PE_12;
  PinName frontL = PE_10;
  PinName sideR = PE_14;
  PinName sideL = PE_15;
  PinName rojarBottomR = PE_7;
  PinName rojarBottomL = PE_8;
} Switch;

struct
{
  PinName IMUSDA = PB_11;
  PinName IMUSCL = PB_10;
} I2CPin;

struct
{
  PinName clothHangRightEncoderAPulse = PB_1;
  PinName clothHangRightEncoderBPulse = PC_2;
  PinName clothHangLeftEncoderAPulse = PD_3;
  PinName clothHangLeftEncoderBPulse = PD_4;
  PinName rojarArmRightEncoderAPulse = PB_8;
  PinName rojarArmRightEncoderBPulse = PB_9;
  PinName rojarArmLeftEncoderAPulse = PA_5;
  PinName rojarArmLeftEncoderBPulse = PA_6;
  PinName holderRightServoR = PF_8; //pf8
  PinName holderRightServoL = PF_7; //pf7
  PinName holderLeftServoR = PE_5;
  PinName holderLeftServoL = PE_6;
} MecaPin;

double driverPWMOutput[3];
static double pegAttacherPWM[2][2]; //right(CW,CCW),left(CW,CCW)
static double hangerPWM[2][2];      //right(CW,CCW),left(CW,CCW)
static double rojarArmPWM[2][2];    //right(CW,CCW),left(CW,CCW)

Serial STLinkTerminal(USBTX, USBRX, SerialBaud.HardwareSerial); //Surfaceのターミナルとの通信用ポート
Serial serialLCD(serialDevice.LCD1TX, serialDevice.LCD1RX, SerialBaud.LCD);
NewHavenDisplay LCDDriver(serialLCD);
Timer TimerForQEI;                  //エンコーダクラス用共有タイマー
Timer TimerForLCD;                  //LCD更新用タイマー
Timer clothHangerTimer;             //機構シーケンス用タイマー
DigitalOut IMUisReadyLED(LED1);     //IMUセンサキャリブレーション完了表示用LED用デジタル出力
DigitalOut modeIndicatorLED1(LED2); //一時的モード切替表示用LED
DigitalOut modeIndicatorLED2(LED3); //一時的モード切替表示用LED
MPU9250 IMU(I2CPin.IMUSDA, I2CPin.IMUSCL, SerialBaud.I2C);
//UnitProtocol UIF(serialDevice.UIFTX, serialDevice.UIFRX, SerialBaud.HardwareSerial);
UnitProtocol MDD1(serialDevice.MDD1TX, serialDevice.MDD1RX, SerialBaud.HardwareSerial);
UnitProtocol MDD2(serialDevice.MDD2TX, serialDevice.MDD2RX, SerialBaud.HardwareSerial);
UnitProtocol MDD3(serialDevice.MDD3TX, serialDevice.MDD3RX, SerialBaud.HardwareSerial);
DebounceSwitch onBoardSwitch(USER_BUTTON, DebounceSwitch::PULLDOWN);
DebounceSwitch startButton(Switch.toBegin, DebounceSwitch::PULLUP); //create object using pin "PG_2" with PullUpped
DebounceSwitch limitSwitchBarFrontRight(Switch.frontR, DebounceSwitch::PULLUP);
DebounceSwitch limitSwitchBarFrontLeft(Switch.frontL, DebounceSwitch::PULLUP);
DebounceSwitch limitSwitchRightSide(Switch.sideR, DebounceSwitch::PULLUP);
DebounceSwitch limitSwitchLeftSide(Switch.sideL, DebounceSwitch::PULLUP);
DebounceSwitch limitSwitchRightRojarArm(Switch.rojarBottomR, DebounceSwitch::PULLUP);
DebounceSwitch limitSwitchLeftRojarArm(Switch.rojarBottomL, DebounceSwitch::PULLUP);
ClothHold holderRight(MecaPin.holderRightServoR, MecaPin.holderRightServoL);                      //right,leftServo
ClothHold holderLeft(MecaPin.holderLeftServoR, MecaPin.holderLeftServoL);                         //right,leftServo
Peg pegAttacherRight(Robot.estimatePegMaxPWM, Robot.PegVoltageImpressionTime, pegAttacherPWM[0]); //pwm, time
Peg pegAttacherLeft(Robot.estimatePegMaxPWM, Robot.PegVoltageImpressionTime, pegAttacherPWM[1]);  //pwm, time
ClothHang hangerRight(hangerPWM[0]);
ClothHang hangerLeft(hangerPWM[1]);
RojarArm rojarArmRight(rojarArmPWM[0], limitSwitchRightRojarArm);
RojarArm rojarArmLeft(rojarArmPWM[1], limitSwitchLeftRojarArm);
QEI encoderXAxis(OdometryPin.XAxisAPulse, OdometryPin.XAxisBPulse, OdometryPin.XAxisIndexPulse, Robot.encoderPPRHigh, &TimerForQEI, QEI::X4_ENCODING);
QEI encoderYAxis(OdometryPin.YAxisAPulse, OdometryPin.YAxisBPulse, OdometryPin.YAxisIndexPulse, Robot.encoderPPRHigh, &TimerForQEI, QEI::X4_ENCODING);
QEI clothHangRightEncoder(MecaPin.clothHangRightEncoderAPulse, MecaPin.clothHangRightEncoderBPulse, NC, Robot.encoderPPRLow, &TimerForQEI, QEI::X4_ENCODING);
QEI clothHangLeftEncoder(MecaPin.clothHangLeftEncoderAPulse, MecaPin.clothHangLeftEncoderBPulse, NC, Robot.encoderPPRLow, &TimerForQEI, QEI::X4_ENCODING);
QEI rojarArmRightEncoder(MecaPin.rojarArmRightEncoderAPulse, MecaPin.rojarArmRightEncoderBPulse, NC, Robot.encoderPPRLow, &TimerForQEI, QEI::X4_ENCODING);
QEI RojarArmLeftEncoder(MecaPin.rojarArmLeftEncoderAPulse, MecaPin.rojarArmLeftEncoderBPulse, NC, Robot.encoderPPRLow, &TimerForQEI, QEI::X4_ENCODING);
MWodometry odometryXAxis(encoderXAxis, Robot.encoderPPRHigh, Robot.encoderAttachedWheelRadius);
MWodometry odometryYAxis(encoderYAxis, Robot.encoderPPRHigh, Robot.encoderAttachedWheelRadius);
LocationManager<double> robotLocation(0, 0, 0);
DriveTrain accelAlgorithm(robotLocation, odometryXAxis, odometryYAxis, IMU, Robot.permitErrorCircleRadius, Robot.decreaseSpeedCircleRadius);
OmniKinematics3WD OmniKinematics;

int getRunningMode();

int main(void)
{
  hangerRight.setMaxPWM(Robot.estimateHangerMaxPWM);
  rojarArmRight.setMaxPWM(Robot.estimateRojarArmMaxPWM);
  accelAlgorithm.setMaxOutput(Robot.estimateDriveMaxPWM);
  accelAlgorithm.setMinOutput(Robot.estimateDriveMinPWM);
  OmniKinematics.setMaxPWM(Robot.estimateDriveMaxPWM);
#ifdef GAME_MODECHANGE_ONBOARDSWITCH
  holderRight.free('r');
  holderRight.free('l');
  IMU.setup();
  IMUisReadyLED.write(1);
  TimerForLCD.start();
  serialLCD.printf("WAITING...");

  static int currentRunningMode;
  while (1) //ボタン待機
  {
    static bool stateHasChanged, previousState;
    static unsigned int pushedCounter;
    onBoardSwitch.update();
    startButton.update();
    if (onBoardSwitch.stats() != previousState)
    {
      stateHasChanged = 1;
    }
    else
      stateHasChanged = 0;
    if (onBoardSwitch.stats() && stateHasChanged)
    {
      pushedCounter++;
      currentRunningMode = pushedCounter % 3;
    }
    previousState = onBoardSwitch.stats();
    if (currentRunningMode == 0)
    {
      modeIndicatorLED1.write(1);
      modeIndicatorLED2.write(0);
    }
    else if (currentRunningMode == 1)
    {
      modeIndicatorLED1.write(0);
      modeIndicatorLED2.write(1);
    }
    else
    {
      modeIndicatorLED1.write(1);
      modeIndicatorLED2.write(1);
    }
    if (startButton.stats())
      break;

    //currentRunningMode = getRunningMode();
  }

  switch (currentRunningMode)
  {
    case RED_FRONT_bathTowelLeft:
      robotLocation.addPoint((157), -(170)); //さらに近づいてリミット監視開始
      robotLocation.addPoint((252), -(190));
      robotLocation.addPoint((252), -(202)); //最初に直進
      robotLocation.addPoint((360), -(202)); //洗濯物横に引っ張る
      robotLocation.addPoint((360), -(190)); //ロジャー降ろしながら後ろに引く
      robotLocation.addPoint(10, 10);        //初期位置
      break;
    case RED_MIDDLE_bathTowelLeft:           //縦:112-170,横112-205
      robotLocation.addPoint(0, -(365));     //二本目のポール前
      robotLocation.addPoint((157), -(370)); //リミット接触前ちょこちょこ進む
      robotLocation.addPoint((252), -(390)); //竿目印のところまで移動
      robotLocation.addPoint((252), -(405)); //最初に直進
      robotLocation.addPoint((360), -(405)); //洗濯物横に引っ張る
      robotLocation.addPoint((360), -(390)); //ロジャー降ろしながら後ろに引く
      robotLocation.addPoint(10, -(335));    //直線移動できる位置まで戻ってくる
      robotLocation.addPoint(10, 10);        //初期位置
      break;
    case RED_MIDDLE_bathTowelRight:
      break;
    case RED_MIDDLE_bathTowelboth:
      break;
    case RED_BACK_Sheets:
      robotLocation.addPoint(0, -(500));
      robotLocation.addPoint((147), -(565));
      robotLocation.addPoint((340), -(596)); //右リミットスイッチが接触しないから585から593に変更
      robotLocation.addPoint((395), -(575));
      robotLocation.addPoint(10, -(520));
      robotLocation.addPoint(10, 10);
      break;
    case BLUE_FRONT_bathTowelRight:
      break;
    case BLUE_MIDDLE_bathTowelLeft:                     //縦:112-170,横112-205
      robotLocation.addPoint(0, -(340 + 47));           //二本目のポール前
      robotLocation.addPoint(-(112 + 35), -(365 + 47)); //さらに近づいてリミット監視開始
      robotLocation.addPoint(-(135 + 35), -(380 + 47));
      robotLocation.addPoint(-(230 + 35), -(385 + 47)); //横移動
      robotLocation.addPoint(-10, -(335 + 47));         //直線移動できる位置まで戻ってくる
      robotLocation.addPoint(0, 0);                     //初期位置
      break;
    case BLUE_MIDDLE_bathTowelRight:
      break;
    case BLUE_MIDDLE_bathTowelboth:
      break;
    case BLUE_BACK_Sheets:
      robotLocation.addPoint(0, -(500 + 47));
      robotLocation.addPoint(-(112 + 35), -(565 + 47));
      robotLocation.addPoint(-(340 + 35), -(585 + 47));
      robotLocation.addPoint(-(340 + 35), -(575 + 47));
      robotLocation.addPoint(0, -(520 + 47));
      robotLocation.addPoint(0, 0);
      break;
  }
  holderRight.grasp('r');
  holderRight.grasp('l');
  robotLocation.sendNext();
  accelAlgorithm.setAllocateErrorCircleRadius(20);
  while (1)
  {
    //STLinkTerminal.printf("%.1lf %.1lf %.1lf   ", accelAlgorithm.getCurrentXPosition(), accelAlgorithm.getCurrentYPosition(), IMU.gyro_Yaw());
    //STLinkTerminal.printf("%.3lf\t%.3lf\t%.3lf\r\n", accelAlgorithm.getXVector(), accelAlgorithm.getYVector(), accelAlgorithm.getYawVector());
    //STLinkTerminal.printf("%d\t%d\t%d\t%d\t%d\t%d\t\r\n", (int)(pegAttacherPWM[0][0] * 100), (int)(pegAttacherPWM[0][1] * 100), (int)(hangerPWM[0][0] * 100), (int)(hangerPWM[0][1] * 100), (int)(rojarArmPWM[0][0] * 100), (int)(rojarArmPWM[0][1] * 100));
    static unsigned long int prevDisplayed = 0;
    if (((TimerForLCD.read_ms() - prevDisplayed) > 100)) //about 10Hz flash rate
    {
      LCDDriver.clear();
      LCDDriver.home();
      serialLCD.printf("%d %d %d", robotLocation.getXLocationData(), robotLocation.getYLocationData(), robotLocation.getYawStatsData());
      LCDDriver.setCursor(2, 0);
      serialLCD.printf("%.1lf %.1lf %.1lf   ", accelAlgorithm.getCurrentXPosition(), accelAlgorithm.getCurrentYPosition(), IMU.gyro_Yaw());
      prevDisplayed = TimerForLCD.read_ms();
    }
    static unsigned int wayPointSignature = 1;
    rojarArmRight.setEncoderPulse(rojarArmRightEncoder.getPulses());
    rojarArmRight.update();
    pegAttacherRight.update();
    hangerRight.setEncoderPulse(clothHangRightEncoder.getPulses());
    hangerRight.update();
    accelAlgorithm.update();
    accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
    OmniKinematics.getOutput(accelAlgorithm.getXVector(), accelAlgorithm.getYVector(), accelAlgorithm.getYawVector(), IMU.gyro_Yaw(), driverPWMOutput);
    int MDD1Packet[6] = {
        driverPWMOutput[0] < 0 ? 0 : (int)(driverPWMOutput[0] * 100),
        driverPWMOutput[0] > 0 ? 0 : -(int)(driverPWMOutput[0] * 100),
        driverPWMOutput[1] < 0 ? 0 : (int)(driverPWMOutput[1] * 100),
        driverPWMOutput[1] > 0 ? 0 : -(int)(driverPWMOutput[1] * 100),
        driverPWMOutput[2] < 0 ? 0 : (int)(driverPWMOutput[2] * 100),
        driverPWMOutput[2] > 0 ? 0 : -(int)(driverPWMOutput[2] * 100),
    };
    int MDD2Packet[6] = {
        (int)(rojarArmPWM[0][1] * 100),
        (int)(rojarArmPWM[0][0] * 100),
        (int)(hangerPWM[0][0] * 100),
        (int)(hangerPWM[0][1] * 100),
        (int)(pegAttacherPWM[0][1] * 100),
        (int)(pegAttacherPWM[0][0] * 100),
    };
    MDD1.transmit(6, MDD1Packet);
    MDD2.transmit(6, MDD2Packet);
    switch (currentRunningMode)
    {
      case RED_FRONT_bathTowelLeft:
        limitSwitchBarFrontRight.update();
        if (limitSwitchBarFrontRight.stats() && wayPointSignature == 1)
        {
          accelAlgorithm.setCurrentYPosition(-(190));
          robotLocation.setCurrentPoint(robotLocation.getXLocationData(), -(190), robotLocation.getYawStatsData());
          static int flag = 0;
          if (flag == 100)
          {
            robotLocation.sendNext();
            accelAlgorithm.update();
            wayPointSignature++;
          }
          flag++;
        }
        if (accelAlgorithm.getStats())
        {
          switch (wayPointSignature)
          {
            case 1:
              accelAlgorithm.setAllocateErrorCircleRadius(Robot.permitErrorCircleRadius);
              limitSwitchBarFrontRight.update();
              if (!limitSwitchBarFrontRight.stats())
              {
                robotLocation.setCurrentPoint(robotLocation.getXLocationData(), robotLocation.getYLocationData() - 5, robotLocation.getYawStatsData());
                break;
              }
              break;
            case 2:
              static int armPhase = 1, hangerHasDoneFlag = 0; //phase1=洗濯物掛ける, phase2=洗濯ばさみつける
              if (rojarArmRight.stats() && armPhase == 1)     //ロジャーアーム展開完了
              {
                static int initialHangerFlag = 1;
                if (initialHangerFlag) //ロジャー展開後初めての処理
                {
                  hangerRight.setLength(1200); //洗濯物掛ける //1000 for test , 1600 for max lenght, recommend:1530
                  hangerRight.update();        //すぐ下のstats判定のために一度状態を更新し判定フラグを未完了に設定する
                  initialHangerFlag = 0;
                }
                if (hangerRight.stats() && !initialHangerFlag && !hangerHasDoneFlag)
                {
                  hangerHasDoneFlag = 1;
                  hangerRight.setLength(0);
                  hangerRight.update();
                }
              }
              if (hangerRight.stats() && hangerHasDoneFlag)
              {
                static unsigned int seq = 1, timerStartFlag = 1;
                if (timerStartFlag)
                {
                  clothHangerTimer.start();
                  timerStartFlag = 0;
                }
                if (0 < clothHangerTimer.read_ms() && seq == 1)
                {
                  rojarArmRight.setHeight(50);
                  holderRight.release('r');
                  seq = 2;
                }
                if (1300 < clothHangerTimer.read_ms() && seq == 2)
                {
                  holderRight.center('r');
                  seq = 3;
                }
                if (1900 < clothHangerTimer.read_ms() && seq == 3)
                {
                  armPhase = 2;
                  clothHangerTimer.stop();
                  clothHangerTimer.reset();
                }
              }
              if (armPhase == 2)
              {
                if (hangerRight.stats()) //洗濯物が竿にかかっているだけの状態
                {
                  static unsigned int seq = 1, timerStartFlag = 1;
                  if (timerStartFlag)
                  {
                    clothHangerTimer.start();
                    timerStartFlag = 0;
                  }
                  if (0 < clothHangerTimer.read_ms() && clothHangerTimer.read_ms() < 100 && seq == 1)
                  {
                    pegAttacherRight.launch();
                    seq = 2;
                  }
                  if (100 < clothHangerTimer.read_ms() && seq == 2)
                  {
                    clothHangerTimer.stop();
                    clothHangerTimer.reset();
                    robotLocation.sendNext();
                    OmniKinematics.setMaxPWM(0.2); //引っ張るときはゆっくり
                    rojarArmRight.setMaxPWM(0.4);
                    rojarArmRight.setHeight(600);
                    wayPointSignature++;
                  }
                }
              }
              break;
            case 3:
              if (rojarArmRight.stats())
              {
                wayPointSignature++;
              }
              break;
            case 4:
              robotLocation.sendNext(); //横移動
              wayPointSignature++;
              break;
            case 5:
              static bool initialFlag = 1;
              if (initialFlag)
              {
                robotLocation.sendNext();
                rojarArmRight.setHeight(130);
                rojarArmRight.update();
                initialFlag = 0;
              }
              if (rojarArmRight.stats())
              {
                wayPointSignature++;
              }
              break;
            case 6:
              rojarArmRight.setHeight(0);
              OmniKinematics.setMaxPWM(Robot.estimateDriveMaxPWM);
              holderRight.release('l');
              robotLocation.sendNext();
              wayPointSignature++;
              break;
            case 7:
              LCDDriver.clear();
              serialLCD.printf("TASKS CLOSED");
              holderRight.free('r');
              holderRight.free('l');
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
                ;
          }
        }
        break;
      case RED_MIDDLE_bathTowelLeft:
        limitSwitchBarFrontRight.update();
        if (limitSwitchBarFrontRight.stats() && wayPointSignature == 2)
        {
          accelAlgorithm.setCurrentYPosition(-(390));
          robotLocation.setCurrentPoint(robotLocation.getXLocationData(), -(390), robotLocation.getYawStatsData());
          static int flag = 0;
          if (flag == 100)
          {
            robotLocation.sendNext();
            accelAlgorithm.update();
            wayPointSignature++;
          }
          flag++;
        }
        if (accelAlgorithm.getStats())
        {
          switch (wayPointSignature)
          {
            case 1:
              accelAlgorithm.setAllocateErrorCircleRadius(3.5);
              robotLocation.sendNext();
              rojarArmRight.setHeight(1650);
              wayPointSignature++;
              break;
            case 2:
              limitSwitchBarFrontRight.update();
              if (!limitSwitchBarFrontRight.stats())
              {
                robotLocation.setCurrentPoint(robotLocation.getXLocationData(), robotLocation.getYLocationData() - 5, robotLocation.getYawStatsData());
                break;
              }
              break;
            case 3:
              static int armPhase = 1, hangerHasDoneFlag = 0; //phase1=洗濯物掛ける, phase2=洗濯ばさみつける
              if (rojarArmRight.stats() && armPhase == 1)     //ロジャーアーム展開完了
              {
                static int initialHangerFlag = 1;
                if (initialHangerFlag) //ロジャー展開後初めての処理
                {
                  hangerRight.setLength(1200); //洗濯物掛ける //1000 for test , 1600 for max lenght, recommend:1530
                  hangerRight.update();        //すぐ下のstats判定のために一度状態を更新し判定フラグを未完了に設定する
                  initialHangerFlag = 0;
                }
                if (hangerRight.stats() && !initialHangerFlag && !hangerHasDoneFlag)
                {
                  hangerHasDoneFlag = 1;
                  hangerRight.setLength(0);
                  hangerRight.update();
                }
              }
              if (hangerRight.stats() && hangerHasDoneFlag)
              {
                static unsigned int seq = 1, timerStartFlag = 1;
                if (timerStartFlag)
                {
                  clothHangerTimer.start();
                  timerStartFlag = 0;
                }
                if (0 < clothHangerTimer.read_ms() && seq == 1)
                {
                  holderRight.release('r');
                  seq = 2;
                }
                if (1300 < clothHangerTimer.read_ms() && seq == 2)
                {
                  holderRight.center('r');
                  seq = 3;
                }
                if (1900 < clothHangerTimer.read_ms() && seq == 3)
                {
                  armPhase = 2;
                  clothHangerTimer.stop();
                  clothHangerTimer.reset();
                }
              }
              if (armPhase == 2)
              {
                if (hangerRight.stats()) //洗濯物が竿にかかっているだけの状態
                {
                  static unsigned int seq = 1, timerStartFlag = 1;
                  if (timerStartFlag)
                  {
                    clothHangerTimer.start();
                    timerStartFlag = 0;
                  }
                  if (0 < clothHangerTimer.read_ms() && clothHangerTimer.read_ms() < 100 && seq == 1)
                  {
                    pegAttacherRight.launch();
                    seq = 2;
                  }
                  if (100 < clothHangerTimer.read_ms() && seq == 2)
                  {
                    clothHangerTimer.stop();
                    clothHangerTimer.reset();
                    robotLocation.sendNext();
                    OmniKinematics.setMaxPWM(0.2); //引っ張るときはゆっくり 前進
                    rojarArmRight.setMaxPWM(0.4);
                    rojarArmRight.setHeight(2100);
                    wayPointSignature++;
                  }
                }
              }
              break;
            case 4:
              if (rojarArmRight.stats())
              {
                wayPointSignature++;
              }
              break;
            case 5:
              robotLocation.sendNext(); //横移動
              wayPointSignature++;
              break;
            case 6:
              static bool initialFlag = 1;
              if (initialFlag)
              {
                robotLocation.sendNext();
                rojarArmRight.setHeight(1730);
                rojarArmRight.update();
                initialFlag = 0;
              }
              if (rojarArmRight.stats())
              {
                wayPointSignature++;
              }
              break;
            case 7:
              OmniKinematics.setMaxPWM(Robot.estimateDriveMaxPWM);
              holderRight.release('l');
              rojarArmRight.setHeight(0);
              robotLocation.sendNext();
              wayPointSignature++;
              break;
            case 8:
              holderRight.grasp('r');
              holderRight.grasp('l');
              robotLocation.sendNext();
              wayPointSignature++;
              break;
            case 9:
              LCDDriver.clear();
              serialLCD.printf("TASKS CLOSED");
              holderRight.free('r');
              holderRight.free('l');
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
                ;
          }
        }
        break;

      case RED_BACK_Sheets:
        limitSwitchBarFrontRight.update();
        if (limitSwitchBarFrontRight.stats() && wayPointSignature == 2)
        {
          accelAlgorithm.setAllocateErrorCircleRadius(Robot.permitErrorCircleRadius);
          accelAlgorithm.setCurrentYPosition(-(590));
          robotLocation.setCurrentPoint(robotLocation.getXLocationData(), -(590), robotLocation.getYawStatsData());
          static int flag = 0;
          if (flag == 100)
          {
            wayPointSignature++;
          }
          flag++;
        }

        limitSwitchRightSide.update();
        if (limitSwitchRightSide.stats() && wayPointSignature == 4)
        {
          accelAlgorithm.setAllocateErrorCircleRadius(Robot.permitErrorCircleRadius);
          accelAlgorithm.setCurrentXPosition(395);
          robotLocation.setCurrentPoint(395, robotLocation.getYLocationData(), robotLocation.getYawStatsData());
          static int flag = 0;
          if (flag == 100)
          {
            wayPointSignature++;
          }
          flag++;
        }

        if (robotLocation.checkMovingStats(accelAlgorithm.getStats()))
        {
          switch (wayPointSignature)
          {
            case 1:
              accelAlgorithm.setAllocateErrorCircleRadius(3.65);
              robotLocation.sendNext();      //三本目のポール少し手前の位置
              rojarArmRight.setHeight(2600); //2100-洗濯物干し最適高さ , 2850-洗濯バサミ最適高さ
              wayPointSignature++;
              break;

            case 2:
              accelAlgorithm.setAllocateErrorCircleRadius(10);
              limitSwitchBarFrontRight.update();
              if (!limitSwitchBarFrontRight.stats())
              {
                robotLocation.setCurrentPoint(robotLocation.getXLocationData(), robotLocation.getYLocationData() - 5, robotLocation.getYawStatsData());
              }
              break;

            case 3:
              static int armPhase = 1, hangerHasDoneFlag = 0; //phase1=洗濯物掛ける, phase2=洗濯ばさみつける
              if (rojarArmRight.stats() && armPhase == 1)     //ロジャーアーム展開完了
              {
                static int initialHangerFlag = 1;
                if (initialHangerFlag) //ロジャー展開後初めての処理
                {
                  hangerRight.setLength(1200); //洗濯物掛ける //1000 for test , 1600 for max lenght, recommend:1530
                  hangerRight.update();        //すぐ下のstats判定のために一度状態を更新し判定フラグを未完了に設定する
                  initialHangerFlag = 0;
                }
                if (hangerRight.stats() && !initialHangerFlag && !hangerHasDoneFlag)
                {
                  hangerHasDoneFlag = 1;
                  hangerRight.setLength(0);
                  hangerRight.update();
                }
              }
              static int initialChangeHeightFlag = 1;
              if (hangerRight.stats() && hangerHasDoneFlag && initialChangeHeightFlag)
              {
                static unsigned int seq = 1, timerStartFlag = 1;
                if (timerStartFlag)
                {
                  clothHangerTimer.start();
                  timerStartFlag = 0;
                }
                if (0 < clothHangerTimer.read_ms() && clothHangerTimer.read_ms() < 1300 && seq == 1)
                {
                  holderRight.release('r');
                  seq = 2;
                }
                if (1300 < clothHangerTimer.read_ms() && clothHangerTimer.read_ms() < 1900 && seq == 2)
                {
                  holderRight.center('r');
                  rojarArmRight.setHeight(3200);
                  rojarArmRight.update();
                  seq = 3;
                }
                if (seq == 3)
                {
                  armPhase = 2;
                  clothHangerTimer.stop();
                  clothHangerTimer.reset();
                  initialChangeHeightFlag = 0;
                }
              }
              if (rojarArmRight.stats() && armPhase == 2)
              {
                if (hangerRight.stats() && hangerHasDoneFlag) //洗濯物が竿にかかっているだけの状態
                {
                  static unsigned int seq = 1, timerStartFlag = 1;
                  if (timerStartFlag)
                  {
                    clothHangerTimer.start();
                    timerStartFlag = 0;
                  }
                  if (0 < clothHangerTimer.read_ms() && clothHangerTimer.read_ms() < 100 && seq == 1) //`hutatume 1700ms
                  {
                    pegAttacherRight.launch();
                    seq = 2;
                  }
                  if (100 < clothHangerTimer.read_ms() && seq == 2)
                  {
                    accelAlgorithm.setMaxOutput(0.27);
                    accelAlgorithm.setMinOutput(0.13);
                    robotLocation.sendNext(); //次の座標を送信
                    wayPointSignature++;
                  }
                }
              }
              break;

            case 4:
              accelAlgorithm.setAllocateErrorCircleRadius(10);
              limitSwitchRightSide.update();
              if (!limitSwitchRightSide.stats())
              {
                robotLocation.setCurrentPoint(robotLocation.getXLocationData() + 5, robotLocation.getYLocationData(), robotLocation.getYawStatsData());
                break;
              }
              break;

            case 5:
              static bool initialFlag = 1;
              if (initialFlag)
              {
                accelAlgorithm.setMaxOutput(Robot.estimateDriveMaxPWM);
                accelAlgorithm.setMinOutput(Robot.estimateDriveMinPWM);
                robotLocation.sendNext();
                rojarArmRight.setMaxPWM(0.4);
                rojarArmRight.setHeight(2420);
                rojarArmRight.update();
                initialFlag = 0;
              }
              if (rojarArmRight.stats())
              {
                wayPointSignature++;
              }
              break;

            case 6:
              holderRight.release('l');
              rojarArmRight.setMaxPWM(Robot.estimateRojarArmMaxPWM);
              rojarArmRight.setHeight(0);
              robotLocation.sendNext();
              wayPointSignature++;
              break;

            case 7:
              holderRight.grasp('r');
              holderRight.grasp('l');
              robotLocation.sendNext();
              wayPointSignature++;
              break;

            case 8:
              holderRight.free('r');
              holderRight.free('l');
              LCDDriver.clear();
              serialLCD.printf("TASKS CLOSED");
              while (1)
              {
                accelAlgorithm.update();
                accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
                LCDDriver.setCursor(2, 0);
                serialLCD.printf("%.1lf %.1lf %.1lf", accelAlgorithm.getCurrentXPosition(), accelAlgorithm.getCurrentYPosition(), IMU.gyro_Yaw());
              }
              break;

            default:
              LCDDriver.clear();
              serialLCD.printf("WayPoint ERROR");
              while (1)
              {
              }
          }
        }
    }
  }

#endif //GAME_MODECHANGE_ONBOARDSWITCH

#ifdef TEST_DRIVE
  robotLocation.addPoint(0, -520, 0);
  robotLocation.addPoint(300, -520, 0);
  robotLocation.addPoint(250, -300, 0);
  robotLocation.addPoint(0, 0, 0);
  IMU.setup();
  IMUisReadyLED.write(1);
  while (1) //モードセレクト処理
  {
    startButton.update();
    if (startButton.stats())
      break;
  }
  while (1)
  {
    static unsigned int wayPointSignature = 0;
    accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
    accelAlgorithm.update();
    OmniKinematics.getOutput(accelAlgorithm.getXVector(), accelAlgorithm.getYVector(), accelAlgorithm.getYawVector(), IMU.gyro_Yaw(), driverPWMOutput);
    int MDD1Packet[6] = {
        driverPWMOutput[0] < 0 ? 0 : (int)(driverPWMOutput[0] * 100),
        driverPWMOutput[0] > 0 ? 0 : -(int)(driverPWMOutput[0] * 100),
        driverPWMOutput[1] < 0 ? 0 : (int)(driverPWMOutput[1] * 100),
        driverPWMOutput[1] > 0 ? 0 : -(int)(driverPWMOutput[1] * 100),
        driverPWMOutput[2] < 0 ? 0 : (int)(driverPWMOutput[2] * 100),
        driverPWMOutput[2] > 0 ? 0 : -(int)(driverPWMOutput[2] * 100),
    };
    MDD1.transmit(6, MDD1Packet);
    if (robotLocation.checkMovingStats(accelAlgorithm.getStats()))
    {
      if (wayPointSignature == 4)
        break;
      robotLocation.sendNext();
      wayPointSignature++;
    }
    STLinkTerminal.printf("%4.4lf,%4.4lf,%4.4lf\r\n", accelAlgorithm.getCurrentXPosition(), accelAlgorithm.getCurrentYPosition(), accelAlgorithm.getCurrentYawPosition());
  }
  while (1)
  {
    accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
    accelAlgorithm.update();
    OmniKinematics.getOutput(accelAlgorithm.getXVector(), accelAlgorithm.getYVector(), accelAlgorithm.getYawVector(), IMU.gyro_Yaw(), driverPWMOutput);
    int MDD1Packet[6] = {
        driverPWMOutput[0] < 0 ? 0 : (int)(driverPWMOutput[0] * 100),
        driverPWMOutput[0] > 0 ? 0 : -(int)(driverPWMOutput[0] * 100),
        driverPWMOutput[1] < 0 ? 0 : (int)(driverPWMOutput[1] * 100),
        driverPWMOutput[1] > 0 ? 0 : -(int)(driverPWMOutput[1] * 100),
        driverPWMOutput[2] < 0 ? 0 : (int)(driverPWMOutput[2] * 100),
        driverPWMOutput[2] > 0 ? 0 : -(int)(driverPWMOutput[2] * 100),
    };
    MDD1.transmit(6, MDD1Packet);
    STLinkTerminal.printf("%4.4lf,%4.4lf,%4.4lf\r\n", accelAlgorithm.getCurrentXPosition(), accelAlgorithm.getCurrentYPosition(), accelAlgorithm.getCurrentYawPosition());
  }
#endif //TEST_DRIVE

#ifdef TEST_RojarArm_UpDownLoop
  while (1)
  {
    startButton.update();
    if (startButton.stats())
      break;
  }
  while (1)
  {
    rojarArmRight.setEncoderPulse(rojarArmRightEncoder.getPulses());
    rojarArmRight.update();
    static int armPhase = 0;
    if (rojarArmRight.stats() && armPhase == 0)
    {
      wait(2);
      armPhase++;
      rojarArmRight.setHeight(3200);
      rojarArmRight.update();
    }
    else if (rojarArmRight.stats() && armPhase == 1)
    {
      wait(2);
      armPhase = 0;
      rojarArmRight.setHeight(0);
      rojarArmRight.update();
    }
  }
#endif //TEST_RojarArm_UpDownLoop

#ifdef TEST_PEGLaunch
  DebounceSwitch startButton(USER_BUTTON, 'd');
  while (1)
  {
    startButton.update();
    if (startButton.stats() == 1)
      break;
  }
  Peg pegAttacher(PE_12, PE_14, 0.5, 0.5);
  pegAttacher.launch();
  while (1)
  {
    pegAttacher.update();
  }
#endif // TEST_PEGLaunch

#ifdef TEST_HangerMovement
  DebounceSwitch startButton(USER_BUTTON, 'd');
  while (1)
  {
    startButton.update();
    if (startButton.stats() == 1)
      break;
  }
  ClothHang hanger(PB_10, PB_11);
  hanger.setMaxPWM(0.5);
  hanger.setLength(2000);
  QEI clothHangEncoder(PB_8, PB_9, NC, 48, &TimerForQEI, QEI::X4_ENCODING);
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
#endif //TEST_HangerMovement

#ifdef TEST_HoldServo
  DebounceSwitch startButton(USER_BUTTON, 'd');
  while (1)
  {
    startButton.update();
    if (startButton.stats() == 1)
      break;
  }
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
#endif //TEST_HoldServo

#ifdef TEST_RojarArmUpDownOnce
  DebounceSwitch startButton(USER_BUTTON, 'd');
  while (1)
  {
    startButton.update();
    if (startButton.stats() == 1)
      break;
  }
  RojarArm rojarArmRight(PD_12, PD_13);
  rojarArmRight.setMaxPWM(0.5);
  rojarArmRight.setHeight(1500);
  QEI rojarArmRightEncoder(PB_8, PB_9, NC, 48, &TimerForQEI, QEI::X4_ENCODING);
  while (1)
  {
    STLinkTerminal.printf("PULSE:%d \tARM STATS(extend):%d\r\n", rojarArmRightEncoder.getPulses(), RojarArmRight.stats());
    rojarArmRight.setEncoderPulse(rojarArmRightEncoder.getPulses());
    rojarArmRight.update();
    if (rojarArmRight.stats())
      break;
  }
  rojarArmRight.setHeight(0);
  while (1)
  {
    STLinkTerminal.printf("PULSE:%d \tARM STATS(reduce):%d\r\n", rojarArmRightEncoder.getPulses(), RojarArmRight.stats());
    rojarArmRight.setEncoderPulse(rojarArmRightEncoder.getPulses());
    rojarArmRight.update();
  }
#endif //TEST_RojarArmUpDownOnce
}

/*int getRunningMode()
{
  int UIFData[1]; //sequence number
  if (!UIF.receive(&UIFData[0]))
  {
    return 0;
  }
  int ACK = 0x06;
  UIF.transmit(1, &ACK); //モード受けとり確認用シグナル
  return UIFData[0];
}*/
