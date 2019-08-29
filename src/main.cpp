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
  RED_FRONT_PRE_bathTowelLeft,
  RED_MIDDLE_PRE_bathTowelRight,
  RED_MIDDLE_PRE_bathTowelLeft,
  RED_MIDDLE_PRE_bathTowelboth,
  RED_BACK_Sheets,

  //↓ not yet
  RED_MIDDLE_FINAL_bathTowelLeft,
  RED_MIDDLE_FINAL_bathTowelRight,
  RED_MIDDLE_FINAL_bathTowelboth,
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
  const double permitErrorCircleRadius = 10.0;
  const double driveDisableRadius = 2.0;
  const int decreaseSpeedCircleRadius = 90;
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
DigitalOut modeIndicatorLED1(LED1); //一時的モード切替表示用LED
DigitalOut modeIndicatorLED2(LED2); //一時的モード切替表示用LED
DigitalOut modeIndicatorLED3(LED3); //一時的モード切替表示用LED
MPU9250 IMU(I2CPin.IMUSDA, I2CPin.IMUSCL, SerialBaud.I2C);
//UnitProtocol UIF(serialDevice.UIFTX, serialDevice.UIFRX, SerialBaud.HardwareSerial);
enum MDDName
{
  Drive,
  Meca1,
  Meca2,
} MDD;
UnitProtocol MDDSlave[3] = {
    UnitProtocol(serialDevice.MDD1TX, serialDevice.MDD1RX, SerialBaud.HardwareSerial),
    UnitProtocol(serialDevice.MDD2TX, serialDevice.MDD2RX, SerialBaud.HardwareSerial),
    UnitProtocol(serialDevice.MDD3TX, serialDevice.MDD3RX, SerialBaud.HardwareSerial),
};
DebounceSwitch onBoardSwitch(USER_BUTTON, DebounceSwitch::PULLDOWN);
DebounceSwitch startButton(Switch.toBegin, DebounceSwitch::PULLUP); //create object using pin "PG_2" with PullUpped
enum set
{
  right,
  left,
} mechaSet;
DebounceSwitch limitSwitchBar[2]{
    DebounceSwitch(Switch.frontR, DebounceSwitch::PULLUP),
    DebounceSwitch(Switch.frontL, DebounceSwitch::PULLUP),
};
DebounceSwitch limitSwitchSide[2]{
    DebounceSwitch(Switch.sideR, DebounceSwitch::PULLUP),
    DebounceSwitch(Switch.sideL, DebounceSwitch::PULLUP),
};
DebounceSwitch limitSwitchRojarArm[2]{
    DebounceSwitch(Switch.rojarBottomR, DebounceSwitch::PULLUP),
    DebounceSwitch(Switch.rojarBottomL, DebounceSwitch::PULLUP),
};
ClothHold holder[2] = {
    ClothHold(MecaPin.holderRightServoR, MecaPin.holderRightServoL), //right,leftServo
    ClothHold(MecaPin.holderLeftServoR, MecaPin.holderLeftServoL),   //right,leftServo
};
Peg pegAttacher[2]{
    Peg(Robot.estimatePegMaxPWM, Robot.PegVoltageImpressionTime, pegAttacherPWM[0]), //pwm, time
    Peg(Robot.estimatePegMaxPWM, Robot.PegVoltageImpressionTime, pegAttacherPWM[1]), //pwm, time
};
ClothHang hanger[2]{
    ClothHang(hangerPWM[right]),
    ClothHang(hangerPWM[left]),
};
RojarArm rojarArm[2]{
    RojarArm(rojarArmPWM[right], limitSwitchRojarArm[right]),
    RojarArm(rojarArmPWM[left], limitSwitchRojarArm[left]),
};
QEI clothHangEncoder[2]{
    QEI(MecaPin.clothHangRightEncoderAPulse, MecaPin.clothHangRightEncoderBPulse, NC, Robot.encoderPPRLow, &TimerForQEI, QEI::X4_ENCODING),
    QEI(MecaPin.clothHangLeftEncoderAPulse, MecaPin.clothHangLeftEncoderBPulse, NC, Robot.encoderPPRLow, &TimerForQEI, QEI::X4_ENCODING),
};
QEI rojarArmEncoder[2]{
    QEI(MecaPin.rojarArmRightEncoderAPulse, MecaPin.rojarArmRightEncoderBPulse, NC, Robot.encoderPPRLow, &TimerForQEI, QEI::X4_ENCODING),
    QEI(MecaPin.rojarArmLeftEncoderAPulse, MecaPin.rojarArmLeftEncoderBPulse, NC, Robot.encoderPPRLow, &TimerForQEI, QEI::X4_ENCODING),
};

QEI encoderXAxis(OdometryPin.XAxisAPulse, OdometryPin.XAxisBPulse, OdometryPin.XAxisIndexPulse, Robot.encoderPPRHigh, &TimerForQEI, QEI::X4_ENCODING);
QEI encoderYAxis(OdometryPin.YAxisAPulse, OdometryPin.YAxisBPulse, OdometryPin.YAxisIndexPulse, Robot.encoderPPRHigh, &TimerForQEI, QEI::X4_ENCODING);
MWodometry odometryXAxis(encoderXAxis, Robot.encoderPPRHigh, Robot.encoderAttachedWheelRadius);
MWodometry odometryYAxis(encoderYAxis, Robot.encoderPPRHigh, Robot.encoderAttachedWheelRadius);
LocationManager<double> robotLocation(0, 0, 0);
DriveTrain accelAlgorithm(robotLocation, odometryXAxis, odometryYAxis, IMU, Robot.permitErrorCircleRadius, Robot.driveDisableRadius, Robot.decreaseSpeedCircleRadius);
OmniKinematics3WD OmniKinematics;

int getRunningMode();

int main(void)
{
  for (int i = 0; i < 2; i++)
  {
    hanger[i].setMaxPWM(Robot.estimateHangerMaxPWM);
    rojarArm[i].setMaxPWM(Robot.estimateRojarArmMaxPWM);
    holder[i].free('r');
    holder[i].free('l');
  }
  accelAlgorithm.setMaxOutput(Robot.estimateDriveMaxPWM);
  accelAlgorithm.setMinOutput(Robot.estimateDriveMinPWM);
  OmniKinematics.setMaxPWM(Robot.estimateDriveMaxPWM);
#ifdef GAME_MODECHANGE_ONBOARDSWITCH

  IMU.setup();
  TimerForLCD.start();
  serialLCD.printf("WAITING...");

  static int currentRunningMode, whichMecha;
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
    if (onBoardSwitch.stats() != previousState)
    {
      stateHasChanged = 1;
    }
    else
      stateHasChanged = 0;
    if (onBoardSwitch.stats() && stateHasChanged)
    {
      pushedCounter++;
      currentRunningMode = pushedCounter % 5;
    }
    previousState = onBoardSwitch.stats();
    switch (currentRunningMode)
    {
      case 0:
        modeIndicatorLED1 = 1;
        modeIndicatorLED2 = 0;
        modeIndicatorLED3 = 0;
        break;
      case 1:
        modeIndicatorLED1 = 0;
        modeIndicatorLED2 = 1;
        modeIndicatorLED3 = 0;
        break;
      case 2:
        modeIndicatorLED1 = 1;
        modeIndicatorLED2 = 1;
        modeIndicatorLED3 = 0;
        break;
      case 3:
        modeIndicatorLED1 = 0;
        modeIndicatorLED2 = 0;
        modeIndicatorLED3 = 1;
        break;
      case 4:
        modeIndicatorLED1 = 1;
        modeIndicatorLED2 = 0;
        modeIndicatorLED3 = 1;
        break;
    }
    if (startButton.stats())
      break;

    //currentRunningMode = getRunningMode();
  }

  switch (currentRunningMode)
  {
    case RED_FRONT_PRE_bathTowelLeft: //左ロジャー
      whichMecha = left;
      robotLocation.addPoint((157), -(170)); //さらに近づいてリミット監視開始
      robotLocation.addPoint((252), -(190));
      robotLocation.addPoint((252), -(202)); //最初に直進
      robotLocation.addPoint((360), -(202)); //洗濯物横に引っ張る
      robotLocation.addPoint((360), -(190)); //ロジャー降ろしながら後ろに引く
      robotLocation.addPoint(10, 10);        //初期位置
      break;
    case RED_MIDDLE_PRE_bathTowelLeft: //(縦掛け), 左ロジャー
      whichMecha = left;
      robotLocation.addPoint(0, -(365));     //二本目のポール前
      robotLocation.addPoint((157), -(370)); //リミット接触前ちょこちょこ進む
      robotLocation.addPoint((172), -(390)); //竿目印のところまで移動
      robotLocation.addPoint((172), -(405)); //最初に直進
      robotLocation.addPoint((230), -(405)); //洗濯物横に引っ張る
      robotLocation.addPoint((230), -(390)); //ロジャー降ろしながら後ろに引く
      robotLocation.addPoint(10, -(335));    //直線移動できる位置まで戻ってくる
      robotLocation.addPoint(10, 10);        //初期位置
      break;
    case RED_MIDDLE_PRE_bathTowelRight: //縦:112-170,横112-205 (横掛け), 右ロジャー
      whichMecha = right;
      robotLocation.addPoint(0, -(365));     //二本目のポール前
      robotLocation.addPoint((157), -(370)); //リミット接触前ちょこちょこ進む
      robotLocation.addPoint((252), -(390)); //竿目印のところまで移動
      robotLocation.addPoint((252), -(405)); //最初に直進
      robotLocation.addPoint((360), -(405)); //洗濯物横に引っ張る
      robotLocation.addPoint((360), -(390)); //ロジャー降ろしながら後ろに引く
      robotLocation.addPoint(10, -(335));    //直線移動できる位置まで戻ってくる
      robotLocation.addPoint(10, 10);        //初期位置
      break;
    case RED_MIDDLE_PRE_bathTowelboth:       //予選(左ロジャー縦掛け),(右ロジャー横掛け)
      whichMecha = left;                     //途中で変わる!!
      robotLocation.addPoint(0, -(365));     //二本目のポール前
      robotLocation.addPoint((157), -(370)); //リミット接触前ちょこちょこ進む
      robotLocation.addPoint((172), -(390)); //竿目印のところまで移動
      robotLocation.addPoint((172), -(405)); //最初に直進
      robotLocation.addPoint((230), -(405)); //洗濯物横に引っ張る
      robotLocation.addPoint((230), -(390)); //ロジャー降ろしながら後ろに引く
      robotLocation.addPoint((252), -(390)); //竿目印のところまで移動     2つ目！
      robotLocation.addPoint((252), -(405)); //最初に直進
      robotLocation.addPoint((360), -(405)); //洗濯物横に引っ張る
      robotLocation.addPoint((360), -(390)); //ロジャー降ろしながら後ろに引く
      robotLocation.addPoint(10, -(335));    //直線移動できる位置まで戻ってくる
      robotLocation.addPoint(10, 10);        //初期位置
      break;
    case RED_MIDDLE_FINAL_bathTowelLeft:
      whichMecha = left;
      break;
    case RED_MIDDLE_FINAL_bathTowelRight:
      whichMecha = right;
      break;
    case RED_MIDDLE_FINAL_bathTowelboth: //決勝(左ロジャー横掛け),(右ロジャー横掛け)
      whichMecha = left;                 //途中で変わる!!
      break;
    case RED_BACK_Sheets: //右ロジャー
      whichMecha = right;
      robotLocation.addPoint(0, -(500));
      robotLocation.addPoint((147), -(565));
      robotLocation.addPoint((340), -(596)); //右リミットスイッチが接触しないから596に変更
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
  for (int i = 0; i < 2; i++)
  {
    holder[i].grasp('r');
    holder[i].grasp('l');
  }
  robotLocation.sendNext();
  accelAlgorithm.setPositionChangedFlag();
  accelAlgorithm.setAllocateErrorCircleRadius(50);
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
    for (int i = 0; i < 2; i++)
    {
      rojarArm[i].setEncoderPulse(rojarArmEncoder[i].getPulses());
      rojarArm[i].update();
      pegAttacher[i].update();
      hanger[i].setEncoderPulse(clothHangEncoder[i].getPulses());
      hanger[i].update();
    }
    accelAlgorithm.update();
    accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
    OmniKinematics.getOutput(accelAlgorithm.getXVector(), accelAlgorithm.getYVector(), accelAlgorithm.getYawVector(), IMU.gyro_Yaw(), driverPWMOutput);
    int MDDDrivePacket[6] = {
        driverPWMOutput[0] < 0 ? 0 : (int)(driverPWMOutput[0] * 100),
        driverPWMOutput[0] > 0 ? 0 : -(int)(driverPWMOutput[0] * 100),
        driverPWMOutput[1] < 0 ? 0 : (int)(driverPWMOutput[1] * 100),
        driverPWMOutput[1] > 0 ? 0 : -(int)(driverPWMOutput[1] * 100),
        driverPWMOutput[2] < 0 ? 0 : (int)(driverPWMOutput[2] * 100),
        driverPWMOutput[2] > 0 ? 0 : -(int)(driverPWMOutput[2] * 100),
    };
    int MDDMeca1Packet[6] = {
        (int)(rojarArmPWM[0][1] * 100),
        (int)(rojarArmPWM[0][0] * 100),
        (int)(hangerPWM[0][0] * 100),
        (int)(hangerPWM[0][1] * 100),
        (int)(pegAttacherPWM[0][1] * 100),
        (int)(pegAttacherPWM[0][0] * 100),
    };
    int MDDMeca2Packet[6] = {
        (int)(rojarArmPWM[1][1] * 100),
        (int)(rojarArmPWM[1][0] * 100),
        (int)(hangerPWM[1][0] * 100),
        (int)(hangerPWM[1][1] * 100),
        (int)(pegAttacherPWM[1][1] * 100),
        (int)(pegAttacherPWM[1][0] * 100),
    };
    MDDSlave[Drive].transmit(6, MDDDrivePacket);
    MDDSlave[Meca1].transmit(6, MDDMeca1Packet);
    MDDSlave[Meca2].transmit(6, MDDMeca2Packet);
    switch (currentRunningMode)
    {
      case RED_FRONT_PRE_bathTowelLeft:
        limitSwitchBar[right].update();
        if (limitSwitchBar[right].stats() && wayPointSignature == 1)
        {
          accelAlgorithm.setCurrentYPosition(-(190));
          robotLocation.setCurrentPoint(robotLocation.getXLocationData(), -(190), robotLocation.getYawStatsData());
          static int flag = 0;
          if (flag == 100)
          {
            robotLocation.sendNext();
            accelAlgorithm.setPositionChangedFlag();
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
              limitSwitchBar[right].update();
              if (!limitSwitchBar[right].stats())
              {
                robotLocation.setCurrentPoint(robotLocation.getXLocationData(), robotLocation.getYLocationData() - 5, robotLocation.getYawStatsData());
                break;
              }
              break;
            case 2:
              static int armPhase = 1, hangerHasDoneFlag = 0;    //phase1=洗濯物掛ける, phase2=洗濯ばさみつける
              if (rojarArm[whichMecha].stats() && armPhase == 1) //ロジャーアーム展開完了
              {
                static int initialHangerFlag = 1;
                if (initialHangerFlag) //ロジャー展開後初めての処理
                {
                  hanger[whichMecha].setLength(1200); //洗濯物掛ける //1000 for test , 1600 for max lenght, recommend:1530
                  hanger[whichMecha].update();        //すぐ下のstats判定のために一度状態を更新し判定フラグを未完了に設定する
                  initialHangerFlag = 0;
                }
                if (hanger[whichMecha].stats() && !initialHangerFlag && !hangerHasDoneFlag)
                {
                  hangerHasDoneFlag = 1;
                  hanger[whichMecha].setLength(0);
                  hanger[whichMecha].update();
                }
              }
              if (hanger[whichMecha].stats() && hangerHasDoneFlag)
              {
                static unsigned int seq = 1, timerStartFlag = 1;
                if (timerStartFlag)
                {
                  clothHangerTimer.start();
                  timerStartFlag = 0;
                }
                if (0 < clothHangerTimer.read_ms() && seq == 1)
                {
                  rojarArm[whichMecha].setHeight(50);
                  holder[whichMecha].release('r');
                  seq = 2;
                }
                if (1300 < clothHangerTimer.read_ms() && seq == 2)
                {
                  holder[whichMecha].center('r');
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
                if (hanger[whichMecha].stats()) //洗濯物が竿にかかっているだけの状態
                {
                  static unsigned int seq = 1, timerStartFlag = 1;
                  if (timerStartFlag)
                  {
                    clothHangerTimer.start();
                    timerStartFlag = 0;
                  }
                  if (0 < clothHangerTimer.read_ms() && clothHangerTimer.read_ms() < 100 && seq == 1)
                  {
                    pegAttacher[whichMecha].launch();
                    seq = 2;
                  }
                  if (100 < clothHangerTimer.read_ms() && seq == 2)
                  {
                    clothHangerTimer.stop();
                    clothHangerTimer.reset();
                    robotLocation.sendNext();
                    accelAlgorithm.setPositionChangedFlag();
                    OmniKinematics.setMaxPWM(0.2); //引っ張るときはゆっくり
                    rojarArm[whichMecha].setMaxPWM(0.4);
                    rojarArm[whichMecha].setHeight(600);
                    wayPointSignature++;
                  }
                }
              }
              break;
            case 3:
              if (rojarArm[whichMecha].stats())
              {
                wayPointSignature++;
              }
              break;
            case 4:
              robotLocation.sendNext(); //横移動
              accelAlgorithm.setPositionChangedFlag();
              wayPointSignature++;
              break;
            case 5:
              static bool initialFlag = 1;
              if (initialFlag)
              {
                robotLocation.sendNext();
                accelAlgorithm.setPositionChangedFlag();
                rojarArm[whichMecha].setHeight(130);
                rojarArm[whichMecha].update();
                initialFlag = 0;
              }
              if (rojarArm[whichMecha].stats())
              {
                wayPointSignature++;
              }
              break;
            case 6:
              rojarArm[whichMecha].setHeight(0);
              OmniKinematics.setMaxPWM(Robot.estimateDriveMaxPWM);
              holder[whichMecha].release('l');
              robotLocation.sendNext();
              accelAlgorithm.setPositionChangedFlag();
              wayPointSignature++;
              break;
            case 7:
              LCDDriver.clear();
              serialLCD.printf("TASKS CLOSED");
              holder[whichMecha].free('r');
              holder[whichMecha].free('l');
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
      case RED_MIDDLE_PRE_bathTowelRight:
        limitSwitchBar[right].update();
        if (limitSwitchBar[right].stats() && wayPointSignature == 2)
        {
          accelAlgorithm.setCurrentYPosition(-(390));
          robotLocation.setCurrentPoint(robotLocation.getXLocationData(), -(390), robotLocation.getYawStatsData());
          static int flag = 0;
          if (flag == 100)
          {
            robotLocation.sendNext();
            accelAlgorithm.setPositionChangedFlag();
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
              accelAlgorithm.setPositionChangedFlag();
              rojarArm[whichMecha].setHeight(1650);
              wayPointSignature++;
              break;
            case 2:
              limitSwitchBar[right].update();
              if (!limitSwitchBar[right].stats())
              {
                robotLocation.setCurrentPoint(robotLocation.getXLocationData(), robotLocation.getYLocationData() - 5, robotLocation.getYawStatsData());
                break;
              }
              break;
            case 3:
              static int armPhase = 1, hangerHasDoneFlag = 0;    //phase1=洗濯物掛ける, phase2=洗濯ばさみつける
              if (rojarArm[whichMecha].stats() && armPhase == 1) //ロジャーアーム展開完了
              {
                static int initialHangerFlag = 1;
                if (initialHangerFlag) //ロジャー展開後初めての処理
                {
                  hanger[whichMecha].setLength(1200); //洗濯物掛ける //1000 for test , 1600 for max lenght, recommend:1530
                  hanger[whichMecha].update();        //すぐ下のstats判定のために一度状態を更新し判定フラグを未完了に設定する
                  initialHangerFlag = 0;
                }
                if (hanger[whichMecha].stats() && !initialHangerFlag && !hangerHasDoneFlag)
                {
                  hangerHasDoneFlag = 1;
                  hanger[whichMecha].setLength(0);
                  hanger[whichMecha].update();
                }
              }
              if (hanger[whichMecha].stats() && hangerHasDoneFlag)
              {
                static unsigned int seq = 1, timerStartFlag = 1;
                if (timerStartFlag)
                {
                  clothHangerTimer.start();
                  timerStartFlag = 0;
                }
                if (0 < clothHangerTimer.read_ms() && seq == 1)
                {
                  holder[whichMecha].release('r');
                  seq = 2;
                }
                if (1300 < clothHangerTimer.read_ms() && seq == 2)
                {
                  holder[whichMecha].center('r');
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
                if (hanger[whichMecha].stats()) //洗濯物が竿にかかっているだけの状態
                {
                  static unsigned int seq = 1, timerStartFlag = 1;
                  if (timerStartFlag)
                  {
                    clothHangerTimer.start();
                    timerStartFlag = 0;
                  }
                  if (0 < clothHangerTimer.read_ms() && clothHangerTimer.read_ms() < 100 && seq == 1)
                  {
                    pegAttacher[whichMecha].launch();
                    seq = 2;
                  }
                  if (100 < clothHangerTimer.read_ms() && seq == 2)
                  {
                    clothHangerTimer.stop();
                    clothHangerTimer.reset();
                    robotLocation.sendNext();
                    accelAlgorithm.setPositionChangedFlag();
                    OmniKinematics.setMaxPWM(0.2); //引っ張るときはゆっくり 前進
                    rojarArm[whichMecha].setMaxPWM(0.4);
                    rojarArm[whichMecha].setHeight(2100);
                    wayPointSignature++;
                  }
                }
              }
              break;
            case 4:
              if (rojarArm[whichMecha].stats())
              {
                wayPointSignature++;
              }
              break;
            case 5:
              robotLocation.sendNext(); //横移動
              accelAlgorithm.setPositionChangedFlag();
              wayPointSignature++;
              break;
            case 6:
              static bool initialFlag = 1;
              if (initialFlag)
              {
                robotLocation.sendNext();
                accelAlgorithm.setPositionChangedFlag();
                rojarArm[whichMecha].setHeight(1730);
                rojarArm[whichMecha].update();
                initialFlag = 0;
              }
              if (rojarArm[whichMecha].stats())
              {
                wayPointSignature++;
              }
              break;
            case 7:
              OmniKinematics.setMaxPWM(Robot.estimateDriveMaxPWM);
              holder[whichMecha].release('l');
              rojarArm[whichMecha].setHeight(0);
              robotLocation.sendNext();
              accelAlgorithm.setPositionChangedFlag();
              wayPointSignature++;
              break;
            case 8:
              holder[whichMecha].grasp('r');
              holder[whichMecha].grasp('l');
              robotLocation.sendNext();
              accelAlgorithm.setPositionChangedFlag();
              wayPointSignature++;
              break;
            case 9:
              LCDDriver.clear();
              serialLCD.printf("TASKS CLOSED");
              holder[whichMecha].free('r');
              holder[whichMecha].free('l');
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
      case RED_MIDDLE_PRE_bathTowelLeft:
        limitSwitchBar[right].update();
        if (limitSwitchBar[right].stats() && wayPointSignature == 2)
        {
          accelAlgorithm.setCurrentYPosition(-(390));
          robotLocation.setCurrentPoint(robotLocation.getXLocationData(), -(390), robotLocation.getYawStatsData());
          static int flag = 0;
          if (flag == 100)
          {
            robotLocation.sendNext();
            accelAlgorithm.setPositionChangedFlag();
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
              accelAlgorithm.setPositionChangedFlag();
              rojarArm[whichMecha].setHeight(1650);
              wayPointSignature++;
              break;
            case 2:
              limitSwitchBar[right].update();
              if (!limitSwitchBar[right].stats())
              {
                robotLocation.setCurrentPoint(robotLocation.getXLocationData(), robotLocation.getYLocationData() - 5, robotLocation.getYawStatsData());
                break;
              }
              break;
            case 3:
              static int armPhase = 1, hangerHasDoneFlag = 0;    //phase1=洗濯物掛ける, phase2=洗濯ばさみつける
              if (rojarArm[whichMecha].stats() && armPhase == 1) //ロジャーアーム展開完了
              {
                static int initialHangerFlag = 1;
                if (initialHangerFlag) //ロジャー展開後初めての処理
                {
                  hanger[whichMecha].setLength(1200); //洗濯物掛ける //1000 for test , 1600 for max lenght, recommend:1530
                  hanger[whichMecha].update();        //すぐ下のstats判定のために一度状態を更新し判定フラグを未完了に設定する
                  initialHangerFlag = 0;
                }
                if (hanger[whichMecha].stats() && !initialHangerFlag && !hangerHasDoneFlag)
                {
                  hangerHasDoneFlag = 1;
                  hanger[whichMecha].setLength(0);
                  hanger[whichMecha].update();
                }
              }
              if (hanger[whichMecha].stats() && hangerHasDoneFlag)
              {
                static unsigned int seq = 1, timerStartFlag = 1;
                if (timerStartFlag)
                {
                  clothHangerTimer.start();
                  timerStartFlag = 0;
                }
                if (0 < clothHangerTimer.read_ms() && seq == 1)
                {
                  holder[whichMecha].release('r');
                  seq = 2;
                }
                if (1300 < clothHangerTimer.read_ms() && seq == 2)
                {
                  holder[whichMecha].center('r');
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
                if (hanger[whichMecha].stats()) //洗濯物が竿にかかっているだけの状態
                {
                  static unsigned int seq = 1, timerStartFlag = 1;
                  if (timerStartFlag)
                  {
                    clothHangerTimer.start();
                    timerStartFlag = 0;
                  }
                  if (0 < clothHangerTimer.read_ms() && clothHangerTimer.read_ms() < 100 && seq == 1)
                  {
                    pegAttacher[whichMecha].launch();
                    seq = 2;
                  }
                  if (100 < clothHangerTimer.read_ms() && seq == 2)
                  {
                    clothHangerTimer.stop();
                    clothHangerTimer.reset();
                    robotLocation.sendNext();
                    accelAlgorithm.setPositionChangedFlag();
                    OmniKinematics.setMaxPWM(0.2); //引っ張るときはゆっくり 前進
                    rojarArm[whichMecha].setMaxPWM(0.4);
                    rojarArm[whichMecha].setHeight(2100);
                    wayPointSignature++;
                  }
                }
              }
              break;
            case 4:
              if (rojarArm[whichMecha].stats())
              {
                wayPointSignature++;
              }
              break;
            case 5:
              robotLocation.sendNext(); //横移動
              accelAlgorithm.setPositionChangedFlag();
              wayPointSignature++;
              break;
            case 6:
              static bool initialFlag = 1;
              if (initialFlag)
              {
                robotLocation.sendNext();
                accelAlgorithm.setPositionChangedFlag();
                rojarArm[whichMecha].setHeight(1730);
                rojarArm[whichMecha].update();
                initialFlag = 0;
              }
              if (rojarArm[whichMecha].stats())
              {
                wayPointSignature++;
              }
              break;
            case 7:
              OmniKinematics.setMaxPWM(Robot.estimateDriveMaxPWM);
              holder[whichMecha].release('l');
              rojarArm[whichMecha].setHeight(0);
              robotLocation.sendNext();
              accelAlgorithm.setPositionChangedFlag();
              wayPointSignature++;
              break;
            case 8:
              holder[whichMecha].grasp('r');
              holder[whichMecha].grasp('l');
              robotLocation.sendNext();
              accelAlgorithm.setPositionChangedFlag();
              wayPointSignature++;
              break;
            case 9:
              LCDDriver.clear();
              serialLCD.printf("TASKS CLOSED");
              holder[whichMecha].free('r');
              holder[whichMecha].free('l');
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
      case RED_MIDDLE_PRE_bathTowelboth:
        limitSwitchBar[right].update();
        if (limitSwitchBar[right].stats() && wayPointSignature == 2)
        {
          accelAlgorithm.setCurrentYPosition(-(390));
          robotLocation.setCurrentPoint(robotLocation.getXLocationData(), -(390), robotLocation.getYawStatsData());
          static int flag = 0;
          if (flag == 100)
          {
            robotLocation.sendNext();
            accelAlgorithm.setPositionChangedFlag();
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
              accelAlgorithm.setPositionChangedFlag();
              rojarArm[whichMecha].setHeight(1650);
              wayPointSignature++;
              break;
            case 2:
              limitSwitchBar[right].update();
              if (!limitSwitchBar[right].stats())
              {
                robotLocation.setCurrentPoint(robotLocation.getXLocationData(), robotLocation.getYLocationData() - 5, robotLocation.getYawStatsData());
                break;
              }
              break;
            case 3:
              static int armPhaseLeft = 1, hangerHasDoneFlagLeft = 0; //phase1=洗濯物掛ける, phase2=洗濯ばさみつける
              if (rojarArm[whichMecha].stats() && armPhaseLeft == 1)  //ロジャーアーム展開完了
              {
                static int initialHangerFlag = 1;
                if (initialHangerFlag) //ロジャー展開後初めての処理
                {
                  hanger[whichMecha].setLength(1200); //洗濯物掛ける //1000 for test , 1600 for max lenght, recommend:1530
                  hanger[whichMecha].update();        //すぐ下のstats判定のために一度状態を更新し判定フラグを未完了に設定する
                  initialHangerFlag = 0;
                }
                if (hanger[whichMecha].stats() && !initialHangerFlag && !hangerHasDoneFlagLeft)
                {
                  hangerHasDoneFlagLeft = 1;
                  hanger[whichMecha].setLength(0);
                  hanger[whichMecha].update();
                }
              }
              if (hanger[whichMecha].stats() && hangerHasDoneFlagLeft)
              {
                static unsigned int seq = 1, timerStartFlag = 1;
                if (timerStartFlag)
                {
                  clothHangerTimer.start();
                  timerStartFlag = 0;
                }
                if (0 < clothHangerTimer.read_ms() && seq == 1)
                {
                  holder[whichMecha].release('r');
                  seq = 2;
                }
                if (1300 < clothHangerTimer.read_ms() && seq == 2)
                {
                  holder[whichMecha].center('r');
                  seq = 3;
                }
                if (1900 < clothHangerTimer.read_ms() && seq == 3)
                {
                  armPhaseLeft = 2;
                  clothHangerTimer.stop();
                  clothHangerTimer.reset();
                }
              }
              if (armPhaseLeft == 2)
              {
                if (hanger[whichMecha].stats()) //洗濯物が竿にかかっているだけの状態
                {
                  static unsigned int seq = 1, timerStartFlag = 1;
                  if (timerStartFlag)
                  {
                    clothHangerTimer.start();
                    timerStartFlag = 0;
                  }
                  if (0 < clothHangerTimer.read_ms() && clothHangerTimer.read_ms() < 100 && seq == 1)
                  {
                    pegAttacher[whichMecha].launch();
                    seq = 2;
                  }
                  if (100 < clothHangerTimer.read_ms() && seq == 2)
                  {
                    clothHangerTimer.stop();
                    clothHangerTimer.reset();
                    robotLocation.sendNext();
                    accelAlgorithm.setPositionChangedFlag();
                    OmniKinematics.setMaxPWM(0.2); //引っ張るときはゆっくり 前進
                    rojarArm[whichMecha].setMaxPWM(0.4);
                    rojarArm[whichMecha].setHeight(2100);
                    wayPointSignature++;
                  }
                }
              }
              break;
            case 4:
              if (rojarArm[whichMecha].stats())
              {
                wayPointSignature++;
              }
              break;
            case 5:
              robotLocation.sendNext(); //横移動
              accelAlgorithm.setPositionChangedFlag();
              wayPointSignature++;
              break;
            case 6:
              static bool initialFlagLeft = 1;
              if (initialFlagLeft)
              {
                robotLocation.sendNext();
                accelAlgorithm.setPositionChangedFlag();
                rojarArm[whichMecha].setHeight(1730);
                rojarArm[whichMecha].update();
                initialFlagLeft = 0;
              }
              if (rojarArm[whichMecha].stats())
              {
                wayPointSignature++;
              }
              break;
            case 7:
              OmniKinematics.setMaxPWM(Robot.estimateDriveMaxPWM);
              holder[whichMecha].free('l');
              rojarArm[whichMecha].setHeight(0);

              whichMecha = !whichMecha; //途中で機構入れ替え！！

              rojarArm[whichMecha].setHeight(1650);
              robotLocation.sendNext();
              accelAlgorithm.setPositionChangedFlag();
              wayPointSignature++;

              break;
            case 8:
              static int armPhaseRight = 1, hangerHasDoneFlagRight = 0; //phase1=洗濯物掛ける, phase2=洗濯ばさみつける
              if (rojarArm[whichMecha].stats() && armPhaseRight == 1)   //ロジャーアーム展開完了
              {
                static int initialHangerFlag = 1;
                if (initialHangerFlag) //ロジャー展開後初めての処理
                {
                  hanger[whichMecha].setLength(1200); //洗濯物掛ける //1000 for test , 1600 for max lenght, recommend:1530
                  hanger[whichMecha].update();        //すぐ下のstats判定のために一度状態を更新し判定フラグを未完了に設定する
                  initialHangerFlag = 0;
                }
                if (hanger[whichMecha].stats() && !initialHangerFlag && !hangerHasDoneFlagRight)
                {
                  hangerHasDoneFlagRight = 1;
                  hanger[whichMecha].setLength(0);
                  hanger[whichMecha].update();
                }
              }
              if (hanger[whichMecha].stats() && hangerHasDoneFlagRight)
              {
                static unsigned int seq = 1, timerStartFlag = 1;
                if (timerStartFlag)
                {
                  clothHangerTimer.start();
                  timerStartFlag = 0;
                }
                if (0 < clothHangerTimer.read_ms() && seq == 1)
                {
                  holder[whichMecha].release('r');
                  seq = 2;
                }
                if (1300 < clothHangerTimer.read_ms() && seq == 2)
                {
                  holder[whichMecha].center('r');
                  seq = 3;
                }
                if (1900 < clothHangerTimer.read_ms() && seq == 3)
                {
                  armPhaseRight = 2;
                  clothHangerTimer.stop();
                  clothHangerTimer.reset();
                }
              }
              if (armPhaseRight == 2)
              {
                if (hanger[whichMecha].stats()) //洗濯物が竿にかかっているだけの状態
                {
                  static unsigned int seq = 1, timerStartFlag = 1;
                  if (timerStartFlag)
                  {
                    clothHangerTimer.start();
                    timerStartFlag = 0;
                  }
                  if (0 < clothHangerTimer.read_ms() && clothHangerTimer.read_ms() < 100 && seq == 1)
                  {
                    pegAttacher[whichMecha].launch();
                    seq = 2;
                  }
                  if (100 < clothHangerTimer.read_ms() && seq == 2)
                  {
                    clothHangerTimer.stop();
                    clothHangerTimer.reset();
                    robotLocation.sendNext();
                    accelAlgorithm.setPositionChangedFlag();
                    OmniKinematics.setMaxPWM(0.2); //引っ張るときはゆっくり 前進
                    rojarArm[whichMecha].setMaxPWM(0.4);
                    rojarArm[whichMecha].setHeight(2100);
                    wayPointSignature++;
                  }
                }
              }
              break;
            case 9:
              if (rojarArm[whichMecha].stats())
              {
                wayPointSignature++;
              }
              break;
            case 10:
              robotLocation.sendNext(); //横移動
              accelAlgorithm.setPositionChangedFlag();
              wayPointSignature++;
              break;
            case 11:
              static bool initialFlagRight = 1;
              if (initialFlagRight)
              {
                robotLocation.sendNext();
                accelAlgorithm.setPositionChangedFlag();
                rojarArm[whichMecha].setHeight(1730);
                rojarArm[whichMecha].update();
                initialFlagRight = 0;
              }
              if (rojarArm[whichMecha].stats())
              {
                wayPointSignature++;
              }
              break;
            case 12:
              OmniKinematics.setMaxPWM(Robot.estimateDriveMaxPWM);
              holder[whichMecha].release('l');
              rojarArm[whichMecha].setHeight(0);
              robotLocation.sendNext();
              accelAlgorithm.setPositionChangedFlag();
              wayPointSignature++;
              break;
            case 13:
              holder[whichMecha].grasp('r');
              holder[whichMecha].grasp('l');
              robotLocation.sendNext();
              accelAlgorithm.setPositionChangedFlag();
              wayPointSignature++;
              break;
            case 14:
              LCDDriver.clear();
              serialLCD.printf("TASKS CLOSED");
              holder[whichMecha].free('r');
              holder[whichMecha].free('l');
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
        limitSwitchBar[right].update();
        if (limitSwitchBar[right].stats() && wayPointSignature == 2)
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

        limitSwitchBar[right].update();
        if (limitSwitchBar[right].stats() && wayPointSignature == 4)
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
              robotLocation.sendNext(); //三本目のポール少し手前の位置
              accelAlgorithm.setPositionChangedFlag();
              rojarArm[whichMecha].setHeight(2600); //2100-洗濯物干し最適高さ , 2850-洗濯バサミ最適高さ
              wayPointSignature++;
              break;

            case 2:
              accelAlgorithm.setAllocateErrorCircleRadius(10);
              limitSwitchBar[right].update();
              if (!limitSwitchBar[right].stats())
              {
                robotLocation.setCurrentPoint(robotLocation.getXLocationData(), robotLocation.getYLocationData() - 5, robotLocation.getYawStatsData());
              }
              break;

            case 3:
              static int armPhase = 1, hangerHasDoneFlag = 0;    //phase1=洗濯物掛ける, phase2=洗濯ばさみつける
              if (rojarArm[whichMecha].stats() && armPhase == 1) //ロジャーアーム展開完了
              {
                static int initialHangerFlag = 1;
                if (initialHangerFlag) //ロジャー展開後初めての処理
                {
                  hanger[whichMecha].setLength(1200); //洗濯物掛ける //1000 for test , 1600 for max lenght, recommend:1530
                  hanger[whichMecha].update();        //すぐ下のstats判定のために一度状態を更新し判定フラグを未完了に設定する
                  initialHangerFlag = 0;
                }
                if (hanger[whichMecha].stats() && !initialHangerFlag && !hangerHasDoneFlag)
                {
                  hangerHasDoneFlag = 1;
                  hanger[whichMecha].setLength(0);
                  hanger[whichMecha].update();
                }
              }
              static int initialChangeHeightFlag = 1;
              if (hanger[whichMecha].stats() && hangerHasDoneFlag && initialChangeHeightFlag)
              {
                static unsigned int seq = 1, timerStartFlag = 1;
                if (timerStartFlag)
                {
                  clothHangerTimer.start();
                  timerStartFlag = 0;
                }
                if (0 < clothHangerTimer.read_ms() && clothHangerTimer.read_ms() < 1300 && seq == 1)
                {
                  holder[whichMecha].release('r');
                  seq = 2;
                }
                if (1300 < clothHangerTimer.read_ms() && clothHangerTimer.read_ms() < 1900 && seq == 2)
                {
                  holder[whichMecha].center('r');
                  rojarArm[whichMecha].setHeight(3200);
                  rojarArm[whichMecha].update();
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
              if (rojarArm[whichMecha].stats() && armPhase == 2)
              {
                if (hanger[whichMecha].stats() && hangerHasDoneFlag) //洗濯物が竿にかかっているだけの状態
                {
                  static unsigned int seq = 1, timerStartFlag = 1;
                  if (timerStartFlag)
                  {
                    clothHangerTimer.start();
                    timerStartFlag = 0;
                  }
                  if (0 < clothHangerTimer.read_ms() && clothHangerTimer.read_ms() < 100 && seq == 1) //`hutatume 1700ms
                  {
                    pegAttacher[whichMecha].launch();
                    seq = 2;
                  }
                  if (100 < clothHangerTimer.read_ms() && seq == 2)
                  {
                    accelAlgorithm.setMaxOutput(0.27);
                    accelAlgorithm.setMinOutput(0.13);
                    robotLocation.sendNext(); //次の座標を送信
                    accelAlgorithm.setPositionChangedFlag();
                    wayPointSignature++;
                  }
                }
              }
              break;

            case 4:
              accelAlgorithm.setAllocateErrorCircleRadius(10);
              limitSwitchBar[whichMecha].update();
              if (!limitSwitchBar[whichMecha].stats())
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
                accelAlgorithm.setPositionChangedFlag();
                rojarArm[whichMecha].setMaxPWM(0.4);
                rojarArm[whichMecha].setHeight(2420);
                rojarArm[whichMecha].update();
                initialFlag = 0;
              }
              if (rojarArm[whichMecha].stats())
              {
                wayPointSignature++;
              }
              break;

            case 6:
              holder[whichMecha].release('l');
              rojarArm[whichMecha].setMaxPWM(Robot.estimateRojarArmMaxPWM);
              rojarArm[whichMecha].setHeight(0);
              robotLocation.sendNext();
              accelAlgorithm.setPositionChangedFlag();
              wayPointSignature++;
              break;

            case 7:
              holder[whichMecha].grasp('r');
              holder[whichMecha].grasp('l');
              robotLocation.sendNext();
              accelAlgorithm.setPositionChangedFlag();
              wayPointSignature++;
              break;

            case 8:
              holder[whichMecha].free('r');
              holder[whichMecha].free('l');
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
